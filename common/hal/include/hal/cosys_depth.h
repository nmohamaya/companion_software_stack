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
// NOT thread-safe for estimate() — call from a single thread (depth thread in P2).
// Config access is thread-safe (read-only after construction).
//
// Issue: #434
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/idepth_estimator.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <memory>
#include <string>

namespace drone::hal {

/// Cosys-AirSim depth estimator — retrieves ground-truth depth from simulation.
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

    // Non-copyable, non-movable (may own RPC connection state in future)
    CosysDepthBackend(const CosysDepthBackend&)            = delete;
    CosysDepthBackend& operator=(const CosysDepthBackend&) = delete;
    CosysDepthBackend(CosysDepthBackend&&)                 = delete;
    CosysDepthBackend& operator=(CosysDepthBackend&&)      = delete;

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

        // TODO(#462): Call Cosys-AirSim simGetImages() via client_ with
        // DepthPerspective image type for camera_name_ on vehicle_name_.
        // Parse the returned float array into the DepthMap.
        //
        // Until the AirSim SDK integration is implemented, return an error
        // so callers don't silently consume a zero depth map as ground truth.

        DRONE_LOG_WARN("[CosysDepth] SDK not connected — cannot estimate depth");

        return drone::util::Result<DepthMap, std::string>::err(
            "CosysDepthBackend: AirSim SDK integration not yet implemented");
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
