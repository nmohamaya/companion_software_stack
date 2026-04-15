// common/hal/include/hal/hal_factory.h
// HAL Factory: creates hardware backends based on JSON configuration.
//
// Usage:
//   drone::Config cfg;
//   cfg.load("config/default.json");
//   auto camera = drone::hal::create_camera(cfg, "video_capture.mission_cam");
//   auto fc     = drone::hal::create_fc_link(cfg, "comms.mavlink");
//   auto gcs    = drone::hal::create_gcs_link(cfg, "comms.gcs");
//   auto gimbal = drone::hal::create_gimbal(cfg, "payload_manager.gimbal");
//   auto imu    = drone::hal::create_imu_source(cfg, "slam.imu");
//
// The factory reads the "backend" key from the given config section.
// Currently supported: "simulated" (default).
// Compile-guarded backends: "mavlink" (HAVE_MAVSDK), "gazebo" (HAVE_GAZEBO).
// Future backends: "v4l2", "udp", "siyi", "bmi088", etc.
#pragma once

#include "hal/icamera.h"
#include "hal/ifc_link.h"
#include "hal/igcs_link.h"
#include "hal/igimbal.h"
#include "hal/iimu_source.h"
#include "hal/simulated_camera.h"
#include "hal/simulated_depth_estimator.h"
#include "hal/simulated_fc_link.h"
#include "hal/simulated_gcs_link.h"
#include "hal/simulated_gimbal.h"
#include "hal/simulated_imu.h"
#include "hal/simulated_radar.h"


// Optional backends — only included when the dependency is found by CMake
#ifdef HAVE_MAVSDK
#include "hal/mavlink_fc_link.h"
#endif

#ifdef HAVE_GAZEBO
#include "hal/gazebo_camera.h"
#include "hal/gazebo_imu.h"
#include "hal/gazebo_radar.h"
#endif

#ifdef HAVE_COSYS_AIRSIM
#include "hal/cosys_camera.h"
#include "hal/cosys_depth.h"
#include "hal/cosys_imu.h"
#include "hal/cosys_radar.h"
#include "hal/cosys_rpc_client.h"
#endif

#ifdef HAS_OPENCV
#include "hal/depth_anything_v2.h"
#endif

#ifdef HAVE_PLUGINS
#include "util/plugin_loader.h"
#endif

#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <memory>
#include <stdexcept>

namespace drone::hal {

// ── Shared Cosys-AirSim RPC client ──────────────────────────
// All 4 Cosys-AirSim backends (camera, radar, IMU, depth) share a single
// RPC connection to the simulator. shared_ptr is acceptable here — it's a
// shared resource across 4 independently-created HAL backends, same
// justification as spdlog/MAVSDK/Zenoh callback contracts.
#ifdef HAVE_COSYS_AIRSIM
namespace detail {
inline std::shared_ptr<CosysRpcClient>& get_shared_cosys_client(const drone::Config& cfg) {
    static std::shared_ptr<CosysRpcClient> client;
    if (!client) {
        auto host = cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::HOST),
                                         "127.0.0.1");
        auto port = cfg.get<int>(std::string(drone::cfg_key::cosys_airsim::PORT), 41451);
        client    = std::make_shared<CosysRpcClient>(host, port);
        client->connect();
    }
    return client;
}
}  // namespace detail
#endif

#ifdef HAVE_PLUGINS
/// Load a plugin backend from config.  Centralises the read-config → load →
/// extract pattern so all create_* functions share the same validation logic.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "video_capture.mission_cam")
/// @param label    Human-readable label for error messages (e.g. "camera")
template<typename Interface>
[[nodiscard]] inline std::unique_ptr<Interface> load_plugin(const drone::Config& cfg,
                                                            const std::string&   section,
                                                            const std::string&   label) {
    auto so_path = cfg.get<std::string>(section + drone::cfg_key::hal::PLUGIN_PATH, "");
    auto factory = cfg.get<std::string>(section + drone::cfg_key::hal::PLUGIN_FACTORY,
                                        "create_instance");
    if (so_path.empty()) {
        throw std::runtime_error("[HAL] " + label +
                                 " plugin requires a non-empty config value "
                                 "at '" +
                                 section + drone::cfg_key::hal::PLUGIN_PATH + "'");
    }
    auto result = drone::util::PluginLoader::load<Interface>(so_path, factory);
    if (result.is_err()) {
        throw std::runtime_error("[HAL] " + label + " plugin load failed: " + result.error());
    }
    return drone::util::PluginRegistry::instance().extract(std::move(result.value()));
}
#endif

/// Create a camera backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "video_capture.mission_cam")
[[nodiscard]] inline std::unique_ptr<ICamera> create_camera(const drone::Config& cfg,
                                                            const std::string&   section) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating camera '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedCamera>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        auto gz_topic = cfg.get<std::string>(section + drone::cfg_key::hal::GZ_TOPIC, "/camera");
        return std::make_unique<GazeboCameraBackend>(gz_topic);
    }
#endif
#ifdef HAVE_COSYS_AIRSIM
    if (backend == "cosys_airsim") {
        return std::make_unique<CosysCameraBackend>(detail::get_shared_cosys_client(cfg), cfg,
                                                    section);
    }
#endif
    // Future: if (backend == "v4l2") return std::make_unique<V4L2Camera>();
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<ICamera>(cfg, section, "camera");
    }
#endif

    throw std::runtime_error("[HAL] Unknown camera backend: " + backend);
}

/// Create a flight controller link from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "comms.mavlink")
[[nodiscard]] inline std::unique_ptr<IFCLink> create_fc_link(const drone::Config& cfg,
                                                             const std::string&   section) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating FC link '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedFCLink>();
    }
#ifdef HAVE_MAVSDK
    if (backend == "mavlink") return std::make_unique<MavlinkFCLink>();
#endif
        // Future: if (backend == "mavlink_v2") return std::make_unique<MavlinkV2Link>();
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IFCLink>(cfg, section, "FC link");
    }
#endif

    throw std::runtime_error("[HAL] Unknown FC link backend: " + backend);
}

/// Create a GCS link from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "comms.gcs")
[[nodiscard]] inline std::unique_ptr<IGCSLink> create_gcs_link(const drone::Config& cfg,
                                                               const std::string&   section) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating GCS link '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedGCSLink>();
    }
    // Future: if (backend == "udp") return std::make_unique<UDPGCSLink>();
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IGCSLink>(cfg, section, "GCS link");
    }
#endif

    throw std::runtime_error("[HAL] Unknown GCS link backend: " + backend);
}

/// Create a gimbal backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "payload_manager.gimbal")
[[nodiscard]] inline std::unique_ptr<IGimbal> create_gimbal(const drone::Config& cfg,
                                                            const std::string&   section) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating gimbal '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedGimbal>();
    }
    // Future: if (backend == "siyi") return std::make_unique<SIYIGimbal>();
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IGimbal>(cfg, section, "gimbal");
    }
#endif

    throw std::runtime_error("[HAL] Unknown gimbal backend: " + backend);
}

/// Create an IMU source from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "slam.imu")
[[nodiscard]] inline std::unique_ptr<IIMUSource> create_imu_source(const drone::Config& cfg,
                                                                   const std::string&   section) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating IMU source '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedIMU>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        auto gz_topic = cfg.get<std::string>(section + drone::cfg_key::hal::GZ_TOPIC, "/imu");
        auto imu      = std::make_unique<GazeboIMUBackend>(gz_topic);
        return imu;
    }
#endif
#ifdef HAVE_COSYS_AIRSIM
    if (backend == "cosys_airsim") {
        return std::make_unique<CosysIMUBackend>(detail::get_shared_cosys_client(cfg), cfg,
                                                 section);
    }
#endif
    // Future: if (backend == "bmi088") return std::make_unique<BMI088IMU>();
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IIMUSource>(cfg, section, "IMU source");
    }
#endif

    throw std::runtime_error("[HAL] Unknown IMU backend: " + backend);
}

/// Create a radar backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "perception.radar")
[[nodiscard]] inline std::unique_ptr<IRadar> create_radar(
    const drone::Config& cfg,
    const std::string&   section = drone::cfg_key::perception::radar::SECTION) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating radar '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedRadar>(cfg, section);
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        return std::make_unique<GazeboRadarBackend>(cfg, section);
    }
#endif

#ifdef HAVE_COSYS_AIRSIM
    if (backend == "cosys_airsim") {
        return std::make_unique<CosysRadarBackend>(detail::get_shared_cosys_client(cfg), cfg,
                                                   section);
    }
#endif

#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IRadar>(cfg, section, "radar");
    }
#endif

    throw std::runtime_error("[HAL] Unknown radar backend: " + backend);
}

/// Create a depth estimator backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "perception.depth_estimator")
[[nodiscard]] inline std::unique_ptr<IDepthEstimator> create_depth_estimator(
    const drone::Config& cfg,
    const std::string&   section = drone::cfg_key::perception::depth_estimator::SECTION) {
    auto backend = cfg.get<std::string>(section + drone::cfg_key::hal::BACKEND, "simulated");
    DRONE_LOG_INFO("[HAL] Creating depth estimator '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedDepthEstimator>(cfg, section);
    }
#ifdef HAVE_COSYS_AIRSIM
    if (backend == "cosys_airsim") {
        return std::make_unique<CosysDepthBackend>(detail::get_shared_cosys_client(cfg), cfg,
                                                   section);
    }
#endif
#ifdef HAS_OPENCV
    if (backend == "depth_anything_v2") {
        return std::make_unique<DepthAnythingV2Estimator>(cfg, section);
    }
#else
    if (backend == "depth_anything_v2") {
        throw std::runtime_error(
            "[HAL] Depth estimator backend 'depth_anything_v2' requires OpenCV "
            "(HAS_OPENCV), but this build was compiled without it");
    }
#endif
    // Future: if (backend == "gazebo") return std::make_unique<GazeboDepthEstimator>(cfg, section);
#ifdef HAVE_PLUGINS
    if (backend == "plugin") {
        return load_plugin<IDepthEstimator>(cfg, section, "depth estimator");
    }
#endif

    throw std::runtime_error("[HAL] Unknown depth estimator backend: " + backend);
}

}  // namespace drone::hal
