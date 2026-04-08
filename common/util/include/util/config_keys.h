// common/util/include/util/config_keys.h
// Centralised config key registry — constexpr constants for every runtime config key.
//
// All cfg.get<>() call sites must use these constants instead of string literals.
// Misspelled keys become compile errors instead of silent fallback-to-default bugs.
//
// Organisation mirrors config/default.json structure.
// Pattern follows ipc_types.h topic name constants.
#pragma once

namespace drone::cfg_key {

// ═══════════════════════════════════════════════════════════
// Top-level keys
// ═══════════════════════════════════════════════════════════
constexpr const char* LOG_LEVEL   = "log_level";
constexpr const char* IPC_BACKEND = "ipc_backend";

// ═══════════════════════════════════════════════════════════
// Zenoh IPC settings
// ═══════════════════════════════════════════════════════════
namespace zenoh {
constexpr const char* SHM_POOL_SIZE_MB = "zenoh.shm_pool_size_mb";
constexpr const char* NETWORK          = "zenoh.network";
constexpr const char* NETWORK_ENABLED  = "zenoh.network.enabled";
}  // namespace zenoh

// ═══════════════════════════════════════════════════════════
// P1 — Video Capture
// ═══════════════════════════════════════════════════════════
namespace video_capture {

namespace mission_cam {
constexpr const char* SECTION = "video_capture.mission_cam";
constexpr const char* BACKEND = "video_capture.mission_cam.backend";
constexpr const char* WIDTH   = "video_capture.mission_cam.width";
constexpr const char* HEIGHT  = "video_capture.mission_cam.height";
constexpr const char* FPS     = "video_capture.mission_cam.fps";
}  // namespace mission_cam

namespace stereo_cam {
constexpr const char* SECTION = "video_capture.stereo_cam";
constexpr const char* BACKEND = "video_capture.stereo_cam.backend";
constexpr const char* WIDTH   = "video_capture.stereo_cam.width";
constexpr const char* HEIGHT  = "video_capture.stereo_cam.height";
constexpr const char* FPS     = "video_capture.stereo_cam.fps";
}  // namespace stereo_cam

}  // namespace video_capture

// ═══════════════════════════════════════════════════════════
// P2 — Perception
// ═══════════════════════════════════════════════════════════
namespace perception {

namespace detector {
constexpr const char* SECTION               = "perception.detector";
constexpr const char* BACKEND               = "perception.detector.backend";
constexpr const char* CONFIDENCE_THRESHOLD  = "perception.detector.confidence_threshold";
constexpr const char* NMS_THRESHOLD         = "perception.detector.nms_threshold";
constexpr const char* MAX_DETECTIONS        = "perception.detector.max_detections";
constexpr const char* MIN_CONTOUR_AREA      = "perception.detector.min_contour_area";
constexpr const char* SUBSAMPLE             = "perception.detector.subsample";
constexpr const char* MAX_FPS               = "perception.detector.max_fps";
constexpr const char* CONFIDENCE_MAX        = "perception.detector.confidence_max";
constexpr const char* CONFIDENCE_BASE       = "perception.detector.confidence_base";
constexpr const char* CONFIDENCE_AREA_SCALE = "perception.detector.confidence_area_scale";
constexpr const char* MIN_BBOX_HEIGHT_PX    = "perception.detector.min_bbox_height_px";
constexpr const char* MAX_ASPECT_RATIO      = "perception.detector.max_aspect_ratio";
constexpr const char* COLORS                = "perception.detector.colors";
constexpr const char* MODEL_PATH            = "perception.detector.model_path";
constexpr const char* INPUT_SIZE            = "perception.detector.input_size";
}  // namespace detector

namespace tracker {
constexpr const char* SECTION = "perception.tracker";
constexpr const char* BACKEND = "perception.tracker.backend";
}  // namespace tracker

namespace radar {
constexpr const char* SECTION        = "perception.radar";
constexpr const char* BACKEND        = "perception.radar.backend";
constexpr const char* ENABLED        = "perception.radar.enabled";
constexpr const char* UPDATE_RATE_HZ = "perception.radar.update_rate_hz";
}  // namespace radar

namespace fusion {
constexpr const char* SECTION                   = "perception.fusion";
constexpr const char* BACKEND                   = "perception.fusion.backend";
constexpr const char* RATE_HZ                   = "perception.fusion.rate_hz";
constexpr const char* FX                        = "perception.fusion.fx";
constexpr const char* FY                        = "perception.fusion.fy";
constexpr const char* CX                        = "perception.fusion.cx";
constexpr const char* CY                        = "perception.fusion.cy";
constexpr const char* CAMERA_HEIGHT_M           = "perception.fusion.camera_height_m";
constexpr const char* ASSUMED_OBSTACLE_HEIGHT_M = "perception.fusion.assumed_obstacle_height_m";
constexpr const char* DEPTH_SCALE               = "perception.fusion.depth_scale";
}  // namespace fusion

}  // namespace perception

// ═══════════════════════════════════════════════════════════
// P3 — SLAM / VIO / Navigation
// ═══════════════════════════════════════════════════════════
namespace slam {
constexpr const char* IMU_RATE_HZ = "slam.imu_rate_hz";
constexpr const char* VIO_RATE_HZ = "slam.vio_rate_hz";

namespace vio {
constexpr const char* SECTION            = "slam.vio";
constexpr const char* BACKEND            = "slam.vio.backend";
constexpr const char* GZ_TOPIC           = "slam.vio.gz_topic";
constexpr const char* SIM_SPEED_MPS      = "slam.vio.sim_speed_mps";
constexpr const char* GOOD_TRACE_MAX     = "slam.vio.quality.good_trace_max";
constexpr const char* DEGRADED_TRACE_MAX = "slam.vio.quality.degraded_trace_max";
}  // namespace vio

namespace stereo {
constexpr const char* FX       = "slam.stereo.fx";
constexpr const char* FY       = "slam.stereo.fy";
constexpr const char* CX       = "slam.stereo.cx";
constexpr const char* CY       = "slam.stereo.cy";
constexpr const char* BASELINE = "slam.stereo.baseline";
}  // namespace stereo

namespace imu {
constexpr const char* SECTION             = "slam.imu";
constexpr const char* BACKEND             = "slam.imu.backend";
constexpr const char* GYRO_NOISE_DENSITY  = "slam.imu.gyro_noise_density";
constexpr const char* GYRO_RANDOM_WALK    = "slam.imu.gyro_random_walk";
constexpr const char* ACCEL_NOISE_DENSITY = "slam.imu.accel_noise_density";
constexpr const char* ACCEL_RANDOM_WALK   = "slam.imu.accel_random_walk";
}  // namespace imu

}  // namespace slam

// ═══════════════════════════════════════════════════════════
// P4 — Mission Planner
// ═══════════════════════════════════════════════════════════
namespace mission_planner {
constexpr const char* SECTION                    = "mission_planner";
constexpr const char* UPDATE_RATE_HZ             = "mission_planner.update_rate_hz";
constexpr const char* TAKEOFF_ALTITUDE_M         = "mission_planner.takeoff_altitude_m";
constexpr const char* ACCEPTANCE_RADIUS_M        = "mission_planner.acceptance_radius_m";
constexpr const char* CRUISE_SPEED_MPS           = "mission_planner.cruise_speed_mps";
constexpr const char* RTL_ACCEPTANCE_RADIUS_M    = "mission_planner.rtl_acceptance_radius_m";
constexpr const char* LANDED_ALTITUDE_M          = "mission_planner.landed_altitude_m";
constexpr const char* RTL_MIN_DWELL_SECONDS      = "mission_planner.rtl_min_dwell_seconds";
constexpr const char* SURVEY_DURATION_S          = "mission_planner.survey_duration_s";
constexpr const char* SURVEY_YAW_RATE            = "mission_planner.survey_yaw_rate";
constexpr const char* OVERSHOOT_PROXIMITY_FACTOR = "mission_planner.overshoot_proximity_factor";
constexpr const char* WAYPOINTS                  = "mission_planner.waypoints";
constexpr const char* STATIC_OBSTACLES           = "mission_planner.static_obstacles";

namespace path_planner {
constexpr const char* BACKEND            = "mission_planner.path_planner.backend";
constexpr const char* RESOLUTION_M       = "mission_planner.path_planner.resolution_m";
constexpr const char* GRID_EXTENT_M      = "mission_planner.path_planner.grid_extent_m";
constexpr const char* INFLATION_RADIUS_M = "mission_planner.path_planner.inflation_radius_m";
constexpr const char* REPLAN_INTERVAL_S  = "mission_planner.path_planner.replan_interval_s";
constexpr const char* PATH_SPEED_MPS     = "mission_planner.path_planner.path_speed_mps";
constexpr const char* SMOOTHING_ALPHA    = "mission_planner.path_planner.smoothing_alpha";
constexpr const char* MAX_ITERATIONS     = "mission_planner.path_planner.max_iterations";
constexpr const char* MAX_SEARCH_TIME_MS = "mission_planner.path_planner.max_search_time_ms";
constexpr const char* RAMP_DIST_M        = "mission_planner.path_planner.ramp_dist_m";
constexpr const char* MIN_SPEED_MPS      = "mission_planner.path_planner.min_speed_mps";
constexpr const char* SNAP_SEARCH_RADIUS = "mission_planner.path_planner.snap_search_radius";
constexpr const char* Z_BAND_CELLS       = "mission_planner.path_planner.z_band_cells";
constexpr const char* LOOK_AHEAD_M       = "mission_planner.path_planner.look_ahead_m";
constexpr const char* YAW_TOWARDS_TRAVEL = "mission_planner.path_planner.yaw_towards_travel";
constexpr const char* YAW_SMOOTHING_RATE = "mission_planner.path_planner.yaw_smoothing_rate";
constexpr const char* SNAP_APPROACH_BIAS = "mission_planner.path_planner.snap_approach_bias";
}  // namespace path_planner

namespace occupancy_grid {
constexpr const char* RESOLUTION_M       = "mission_planner.occupancy_grid.resolution_m";
constexpr const char* INFLATION_RADIUS_M = "mission_planner.occupancy_grid.inflation_radius_m";
constexpr const char* DYNAMIC_OBSTACLE_TTL_S =
    "mission_planner.occupancy_grid.dynamic_obstacle_ttl_s";
constexpr const char* MIN_CONFIDENCE       = "mission_planner.occupancy_grid.min_confidence";
constexpr const char* PROMOTION_HITS       = "mission_planner.occupancy_grid.promotion_hits";
constexpr const char* RADAR_PROMOTION_HITS = "mission_planner.occupancy_grid.radar_promotion_hits";
constexpr const char* MIN_PROMOTION_DEPTH_CONFIDENCE =
    "mission_planner.occupancy_grid.min_promotion_depth_confidence";
constexpr const char* MAX_STATIC_CELLS   = "mission_planner.occupancy_grid.max_static_cells";
constexpr const char* PREDICTION_ENABLED = "mission_planner.occupancy_grid.prediction_enabled";
constexpr const char* PREDICTION_DT_S    = "mission_planner.occupancy_grid.prediction_dt_s";
}  // namespace occupancy_grid

namespace obstacle_avoidance {
constexpr const char* INFLUENCE_RADIUS_M = "mission_planner.obstacle_avoidance.influence_radius_m";
constexpr const char* REPULSIVE_GAIN     = "mission_planner.obstacle_avoidance.repulsive_gain";
constexpr const char* MAX_CORRECTION_MPS = "mission_planner.obstacle_avoidance.max_correction_mps";
constexpr const char* MIN_CONFIDENCE     = "mission_planner.obstacle_avoidance.min_confidence";
constexpr const char* PREDICTION_DT_S    = "mission_planner.obstacle_avoidance.prediction_dt_s";
constexpr const char* VERTICAL_GAIN      = "mission_planner.obstacle_avoidance.vertical_gain";
constexpr const char* MAX_AGE_MS         = "mission_planner.obstacle_avoidance.max_age_ms";
constexpr const char* PATH_AWARE         = "mission_planner.obstacle_avoidance.path_aware";
}  // namespace obstacle_avoidance

namespace obstacle_avoider {
constexpr const char* BACKEND = "mission_planner.obstacle_avoider.backend";
}  // namespace obstacle_avoider

namespace geofence {
constexpr const char* ENABLED              = "mission_planner.geofence.enabled";
constexpr const char* POLYGON              = "mission_planner.geofence.polygon";
constexpr const char* ALTITUDE_FLOOR_M     = "mission_planner.geofence.altitude_floor_m";
constexpr const char* ALTITUDE_CEILING_M   = "mission_planner.geofence.altitude_ceiling_m";
constexpr const char* WARNING_MARGIN_M     = "mission_planner.geofence.warning_margin_m";
constexpr const char* ALTITUDE_TOLERANCE_M = "mission_planner.geofence.altitude_tolerance_m";
}  // namespace geofence

namespace collision_recovery {
constexpr const char* ENABLED          = "mission_planner.collision_recovery.enabled";
constexpr const char* CLIMB_DELTA_M    = "mission_planner.collision_recovery.climb_delta_m";
constexpr const char* HOVER_DURATION_S = "mission_planner.collision_recovery.hover_duration_s";
}  // namespace collision_recovery

}  // namespace mission_planner

// ═══════════════════════════════════════════════════════════
// P5 — Comms
// ═══════════════════════════════════════════════════════════
namespace comms {

namespace mavlink {
constexpr const char* SECTION     = "comms.mavlink";
constexpr const char* BACKEND     = "comms.mavlink.backend";
constexpr const char* URI         = "comms.mavlink.uri";
constexpr const char* TIMEOUT_MS  = "comms.mavlink.timeout_ms";
constexpr const char* SERIAL_PORT = "comms.mavlink.serial_port";
constexpr const char* BAUD_RATE   = "comms.mavlink.baud_rate";
}  // namespace mavlink

namespace gcs {
constexpr const char* SECTION  = "comms.gcs";
constexpr const char* BACKEND  = "comms.gcs.backend";
constexpr const char* UDP_PORT = "comms.gcs.udp_port";
}  // namespace gcs

}  // namespace comms

// ═══════════════════════════════════════════════════════════
// P6 — Payload Manager
// ═══════════════════════════════════════════════════════════
namespace payload_manager {
constexpr const char* UPDATE_RATE_HZ = "payload_manager.update_rate_hz";

namespace gimbal {
constexpr const char* SECTION = "payload_manager.gimbal";
constexpr const char* BACKEND = "payload_manager.gimbal.backend";

namespace auto_track {
constexpr const char* ENABLED          = "payload_manager.gimbal.auto_track.enabled";
constexpr const char* MIN_CONFIDENCE   = "payload_manager.gimbal.auto_track.min_confidence";
constexpr const char* MANUAL_HOLDOFF_S = "payload_manager.gimbal.auto_track.manual_holdoff_s";
}  // namespace auto_track

}  // namespace gimbal
}  // namespace payload_manager

// ═══════════════════════════════════════════════════════════
// P7 — System Monitor
// ═══════════════════════════════════════════════════════════
namespace system_monitor {
constexpr const char* SECTION               = "system_monitor";
constexpr const char* BACKEND               = "system_monitor.backend";
constexpr const char* UPDATE_RATE_HZ        = "system_monitor.update_rate_hz";
constexpr const char* DISK_CHECK_INTERVAL_S = "system_monitor.disk_check_interval_s";

namespace thresholds {
constexpr const char* CPU_WARN_PERCENT     = "system_monitor.thresholds.cpu_warn_percent";
constexpr const char* MEM_WARN_PERCENT     = "system_monitor.thresholds.mem_warn_percent";
constexpr const char* TEMP_WARN_C          = "system_monitor.thresholds.temp_warn_c";
constexpr const char* TEMP_CRIT_C          = "system_monitor.thresholds.temp_crit_c";
constexpr const char* BATTERY_WARN_PERCENT = "system_monitor.thresholds.battery_warn_percent";
constexpr const char* BATTERY_CRIT_PERCENT = "system_monitor.thresholds.battery_crit_percent";
constexpr const char* DISK_CRIT_PERCENT    = "system_monitor.thresholds.disk_crit_percent";
}  // namespace thresholds
}  // namespace system_monitor

// ═══════════════════════════════════════════════════════════
// Watchdog
// ═══════════════════════════════════════════════════════════
namespace watchdog {
constexpr const char* PROCESSES = "watchdog.processes";
}  // namespace watchdog

// ═══════════════════════════════════════════════════════════
// Flight Recorder
// ═══════════════════════════════════════════════════════════
namespace recorder {
constexpr const char* MAX_SIZE_MB = "recorder.max_size_mb";
constexpr const char* OUTPUT_DIR  = "recorder.output_dir";
}  // namespace recorder

// ═══════════════════════════════════════════════════════════
// HAL common sub-keys (appended to section prefix)
// Usage: cfg.get<std::string>(section + cfg_key::hal::BACKEND, "simulated")
// ═══════════════════════════════════════════════════════════
namespace hal {
constexpr const char* BACKEND  = ".backend";
constexpr const char* GZ_TOPIC = ".gz_topic";

// Radar-specific sub-keys (appended to section prefix)
constexpr const char* MAX_RANGE_M         = ".max_range_m";
constexpr const char* FOV_AZIMUTH_RAD     = ".fov_azimuth_rad";
constexpr const char* FOV_ELEVATION_RAD   = ".fov_elevation_rad";
constexpr const char* GROUND_FILTER_ALT_M = ".ground_filter_alt_m";
constexpr const char* FALSE_ALARM_RATE    = ".false_alarm_rate";
constexpr const char* NUM_TARGETS         = ".num_targets";
constexpr const char* GZ_SCAN_TOPIC       = ".gz_scan_topic";
constexpr const char* GZ_ODOM_TOPIC       = ".gz_odom_topic";

// Noise sub-keys (appended to section prefix)
constexpr const char* NOISE_RANGE_STD_M       = ".noise.range_std_m";
constexpr const char* NOISE_AZIMUTH_STD_RAD   = ".noise.azimuth_std_rad";
constexpr const char* NOISE_ELEVATION_STD_RAD = ".noise.elevation_std_rad";
constexpr const char* NOISE_VELOCITY_STD_MPS  = ".noise.velocity_std_mps";
}  // namespace hal

}  // namespace drone::cfg_key
