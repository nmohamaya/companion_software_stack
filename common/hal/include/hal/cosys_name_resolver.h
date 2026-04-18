// common/hal/include/hal/cosys_name_resolver.h
// Shared name-resolution helpers for Cosys-AirSim HAL backends.
//
// Both CosysCameraBackend (RGB) and CosysDepthBackend (depth) need to resolve
// AirSim camera and vehicle names with the same precedence ladder:
//   1. Per-section override (e.g. `video_capture.mission_cam.camera_name`)
//   2. Top-level fallback (e.g. `cosys_airsim.camera_name`)
//   3. Hard-coded default
//
// An explicit empty string at the per-section key is treated as "absent" so a
// stray `"camera_name": ""` in a scenario override cannot silently shadow the
// top-level default. See Issue #499 Bug #1 for the original motivation.
//
// Free functions (not class statics) so new Cosys backends can reuse them
// without taking a dependency on any specific backend class.
//
// Issue: #499
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "util/config.h"
#include "util/config_keys.h"

#include <string>

namespace drone::hal {

/// Core name resolver: per-section override → top-level key → hard-coded default.
///
/// @param cfg                 Loaded configuration
/// @param section             Config path prefix (e.g. "video_capture.mission_cam"),
///                            empty to skip per-section lookup
/// @param per_section_subkey  Sub-key appended to section (e.g. ".camera_name")
/// @param top_level_key       Fully-qualified fallback key (e.g. "cosys_airsim.camera_name")
/// @param default_value       Hard-coded default when neither key is present
[[nodiscard]] inline std::string resolve_airsim_name(const drone::Config& cfg,
                                                     const std::string&   section,
                                                     const char*          per_section_subkey,
                                                     const char*          top_level_key,
                                                     const char*          default_value) {
    if (!section.empty()) {
        const auto per_section = cfg.get<std::string>(section + per_section_subkey, "");
        // Empty value must not shadow top-level: treat "" as "absent".
        if (!per_section.empty()) return per_section;
    }
    return cfg.get<std::string>(top_level_key, default_value);
}

/// Resolve AirSim camera name:
///   `<section>.camera_name` → `cosys_airsim.camera_name` → "front_center".
[[nodiscard]] inline std::string resolve_camera_name(const drone::Config& cfg,
                                                     const std::string&   section) {
    return resolve_airsim_name(cfg, section, ".camera_name",
                               drone::cfg_key::cosys_airsim::CAMERA_NAME, "front_center");
}

/// Resolve AirSim vehicle name:
///   `<section>.vehicle_name` → `cosys_airsim.vehicle_name` → "Drone0".
[[nodiscard]] inline std::string resolve_vehicle_name(const drone::Config& cfg,
                                                      const std::string&   section) {
    return resolve_airsim_name(cfg, section, ".vehicle_name",
                               drone::cfg_key::cosys_airsim::VEHICLE_NAME, "Drone0");
}

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
