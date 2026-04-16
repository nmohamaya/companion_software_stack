// common/hal/include/hal/cosys_depth.h
// Cosys-AirSim depth estimator backend — retrieves depth images via AirSim RPC.
// Implements IDepthEstimator — connects to Cosys-AirSim simGetImages() with
// DepthPerspective image type to get per-pixel metric depth.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Unlike ML-based depth estimators (e.g. Depth Anything V2), this backend
// retrieves ground-truth depth directly from the simulator — ideal for
// validating perception pipelines without model inference.
//
// The estimate() call ignores the frame_data input parameter; depth comes
// from the simulator via simGetImages(DepthPerspective), not from the frame.
// This means estimate() makes a synchronous RPC call on each invocation.
//
// NOT thread-safe for estimate() — call from a single thread (depth thread in P2).
// Config access is thread-safe (read-only after construction).
//
// Issue: #434, #462
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/idepth_estimator.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim depth estimator — retrieves ground-truth depth from simulation.
///
/// Calls simGetImages() with ImageType::DepthPerspective (pixels_as_float=true)
/// to get metric depth per pixel. Returns DepthMap with confidence=0.95
/// (ground truth from simulator, not perfect due to rendering artifacts).
///
/// Config keys:
///   cosys_airsim.host         (default "127.0.0.1")
///   cosys_airsim.port         (default 41451)
///   cosys_airsim.camera_name  (default "front_center")
///   cosys_airsim.vehicle_name (default "Drone0")
class CosysDepthBackend : public IDepthEstimator {
public:
    /// Construct from shared RPC client and config.
    /// @param client   Shared RPC client (manages connection lifecycle)
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "perception.depth_estimator")
    explicit CosysDepthBackend(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                               const std::string& section)
        : client_(std::move(client))
        , camera_name_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::CAMERA_NAME),
                                            "front_center"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0")) {
        (void)section;  // section reserved for future per-estimator config
        DRONE_LOG_INFO("[CosysDepth] Created for {} camera='{}' vehicle='{}'", client_->endpoint(),
                       camera_name_, vehicle_name_);
    }

    // Non-copyable, non-movable (owns RPC connection state reference)
    CosysDepthBackend(const CosysDepthBackend&)            = delete;
    CosysDepthBackend& operator=(const CosysDepthBackend&) = delete;
    CosysDepthBackend(CosysDepthBackend&&)                 = delete;
    CosysDepthBackend& operator=(CosysDepthBackend&&)      = delete;

    /// Estimate depth by retrieving ground-truth depth from AirSim.
    /// The frame_data parameter is ignored — depth comes from the simulator.
    /// Makes a synchronous RPC call to simGetImages(DepthPerspective).
    [[nodiscard]] drone::util::Result<DepthMap, std::string> estimate(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) override {
        (void)stride;
        if (frame_data == nullptr || width == 0 || height == 0) {
            return drone::util::Result<DepthMap, std::string>::err(
                "CosysDepthBackend: invalid frame (null data or zero dimensions)");
        }
        if (channels == 0) {
            return drone::util::Result<DepthMap, std::string>::err(
                "CosysDepthBackend: invalid channel count (0)");
        }

        try {
            // Use with_client() to prevent TOCTOU race on disconnect.
            // Must be inside try/catch — simGetImages() can throw on RPC failure.
            using namespace msr::airlib;
            std::vector<ImageCaptureBase::ImageResponse> responses;
            bool got_data = client_->with_client([&](auto& rpc) {
                std::vector<ImageCaptureBase::ImageRequest> requests = {
                    ImageCaptureBase::ImageRequest(
                        camera_name_, ImageCaptureBase::ImageType::DepthPerspective,
                        /* pixels_as_float */ true, /* compress */ false)};
                responses = rpc.simGetImages(requests, vehicle_name_);
            });

            if (!got_data) {
                return drone::util::Result<DepthMap, std::string>::err(
                    "CosysDepthBackend: RPC client not connected");
            }
            if (responses.empty()) {
                return drone::util::Result<DepthMap, std::string>::err(
                    "CosysDepthBackend: simGetImages returned empty response");
            }

            const auto& resp = responses[0];

            // Guard signed→unsigned: check > 0 BEFORE casting
            if (resp.image_data_float.empty() || resp.width <= 0 || resp.height <= 0) {
                return drone::util::Result<DepthMap, std::string>::err(
                    "CosysDepthBackend: empty or invalid depth image from AirSim");
            }

            const auto depth_w      = static_cast<uint32_t>(resp.width);
            const auto depth_h      = static_cast<uint32_t>(resp.height);
            const auto total_pixels = static_cast<size_t>(depth_w) * depth_h;

            if (resp.image_data_float.size() < total_pixels) {
                return drone::util::Result<DepthMap, std::string>::err(
                    "CosysDepthBackend: depth data size mismatch (got " +
                    std::to_string(resp.image_data_float.size()) + ", expected " +
                    std::to_string(total_pixels) + ")");
            }

            // Build DepthMap from the float array
            DepthMap depth_map;
            depth_map.width  = depth_w;
            depth_map.height = depth_h;
            depth_map.data.resize(total_pixels);
            std::copy(resp.image_data_float.begin(),
                      resp.image_data_float.begin() + static_cast<ptrdiff_t>(total_pixels),
                      depth_map.data.begin());

            // Sanitise depth values: NaN/Inf from rendering artifacts → 0 (invalid)
            for (auto& d : depth_map.data) {
                if (!std::isfinite(d) || d < 0.0f) {
                    d = 0.0f;
                }
            }

            depth_map.scale = 1.0f;  // AirSim returns metric depth in metres

            // Ground truth from simulator — high confidence but not perfect
            // due to rendering artifacts and floating-point precision
            depth_map.confidence = 0.95f;

            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
            depth_map.timestamp_ns = static_cast<uint64_t>(std::max(decltype(now_ns){0}, now_ns));

            // Record source frame dimensions for bbox-to-depth coordinate mapping
            depth_map.source_width  = width;
            depth_map.source_height = height;

            return drone::util::Result<DepthMap, std::string>::ok(std::move(depth_map));

        } catch (const std::exception& e) {
            return drone::util::Result<DepthMap, std::string>::err(
                std::string("CosysDepthBackend: RPC error — ") + e.what());
        }
    }

    [[nodiscard]] std::string name() const override {
        return "CosysDepth(" + camera_name_ + "@" + client_->endpoint() + ")";
    }

private:
    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    std::string camera_name_;   ///< AirSim camera name for depth retrieval
    std::string vehicle_name_;  ///< AirSim vehicle name
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
