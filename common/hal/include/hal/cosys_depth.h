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

#include "hal/idepth_estimator.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
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
    /// Construct from config.
    explicit CosysDepthBackend(const drone::Config& cfg, const std::string& section)
        : host_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::HOST), "127.0.0.1"))
        , port_(cfg.get<int>(std::string(drone::cfg_key::cosys_airsim::PORT), 41451))
        , camera_name_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::CAMERA_NAME),
                                            "front_center"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0")) {
        (void)section;  // section reserved for future per-estimator config
        DRONE_LOG_INFO("[CosysDepth] Created for {}:{} camera='{}' vehicle='{}'", host_, port_,
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

        // TODO(#434): Call Cosys-AirSim simGetImages() with DepthPerspective
        // image type for camera_name_ on vehicle_name_.
        // Parse the returned float array into the DepthMap.
        //
        // For now, return a placeholder depth map with zero depth to indicate
        // that the SDK connection is not yet implemented.

        DepthMap map;
        map.width         = width;
        map.height        = height;
        map.source_width  = width;  // AirSim returns depth at request resolution
        map.source_height = height;
        map.scale         = 1.0f;  // AirSim returns metric depth directly
        map.confidence    = 1.0f;  // Ground truth from simulator
        map.timestamp_ns  = 0;     // Caller should set from frame timestamp

        const auto num_pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
        map.data.resize(num_pixels, 0.0f);  // Zero depth until SDK is connected

        DRONE_LOG_WARN("[CosysDepth] SDK not connected — returning zero depth map");

        return drone::util::Result<DepthMap, std::string>::ok(std::move(map));
    }

    [[nodiscard]] std::string name() const override {
        return "CosysDepth(" + camera_name_ + "@" + host_ + ":" + std::to_string(port_) + ")";
    }

private:
    std::string host_;          ///< Cosys-AirSim RPC host
    int         port_;          ///< Cosys-AirSim RPC port
    std::string camera_name_;   ///< AirSim camera name for depth retrieval
    std::string vehicle_name_;  ///< AirSim vehicle name
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
