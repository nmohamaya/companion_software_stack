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

#include "util/config.h"

#include <memory>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace drone::hal {

/// Create a camera backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "video_capture.mission_cam")
inline std::unique_ptr<ICamera> create_camera(const drone::Config& cfg,
                                              const std::string&   section) {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating camera '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedCamera>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        auto gz_topic = cfg.get<std::string>(section + ".gz_topic", "/camera");
        return std::make_unique<GazeboCameraBackend>(gz_topic);
    }
#endif
    // Future: if (backend == "v4l2") return std::make_unique<V4L2Camera>();

    throw std::runtime_error("[HAL] Unknown camera backend: " + backend);
}

/// Create a flight controller link from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "comms.mavlink")
inline std::unique_ptr<IFCLink> create_fc_link(const drone::Config& cfg,
                                               const std::string&   section) {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating FC link '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedFCLink>();
    }
#ifdef HAVE_MAVSDK
    if (backend == "mavlink") return std::make_unique<MavlinkFCLink>();
#endif
    // Future: if (backend == "mavlink_v2") return std::make_unique<MavlinkV2Link>();

    throw std::runtime_error("[HAL] Unknown FC link backend: " + backend);
}

/// Create a GCS link from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "comms.gcs")
inline std::unique_ptr<IGCSLink> create_gcs_link(const drone::Config& cfg,
                                                 const std::string&   section) {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating GCS link '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedGCSLink>();
    }
    // Future: if (backend == "udp") return std::make_unique<UDPGCSLink>();

    throw std::runtime_error("[HAL] Unknown GCS link backend: " + backend);
}

/// Create a gimbal backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "payload_manager.gimbal")
inline std::unique_ptr<IGimbal> create_gimbal(const drone::Config& cfg,
                                              const std::string&   section) {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating gimbal '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedGimbal>();
    }
    // Future: if (backend == "siyi") return std::make_unique<SIYIGimbal>();

    throw std::runtime_error("[HAL] Unknown gimbal backend: " + backend);
}

/// Create an IMU source from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "slam.imu")
inline std::unique_ptr<IIMUSource> create_imu_source(const drone::Config& cfg,
                                                     const std::string&   section) {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating IMU source '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedIMU>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        auto gz_topic = cfg.get<std::string>(section + ".gz_topic", "/imu");
        auto imu      = std::make_unique<GazeboIMUBackend>(gz_topic);
        return imu;
    }
#endif
    // Future: if (backend == "bmi088") return std::make_unique<BMI088IMU>();

    throw std::runtime_error("[HAL] Unknown IMU backend: " + backend);
}

/// Create a radar backend from config.
/// @param cfg      Loaded configuration
/// @param section  Config path prefix (e.g. "perception.radar")
inline std::unique_ptr<IRadar> create_radar(const drone::Config& cfg,
                                            const std::string&   section = "perception.radar") {
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");
    spdlog::info("[HAL] Creating radar '{}' backend='{}'", section, backend);

    if (backend == "simulated") {
        return std::make_unique<SimulatedRadar>(cfg, section);
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        return std::make_unique<GazeboRadarBackend>(cfg, section);
    }
#endif

    throw std::runtime_error("[HAL] Unknown radar backend: " + backend);
}

}  // namespace drone::hal
