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
inline constexpr const char* LOG_LEVEL   = "log_level";
inline constexpr const char* IPC_BACKEND = "ipc_backend";
/// IPC serialization format. Currently only "raw" is supported.
/// Future options: "protobuf" (for cross-language GCS interop).
inline constexpr const char* IPC_SERIALIZER = "ipc_serializer";
/// Multi-vehicle namespace prefix. When present, must be [a-zA-Z0-9_-] or empty.
/// When set, all IPC topics are namespaced under "/<vehicle_id>/...".
inline constexpr const char* VEHICLE_ID = "vehicle_id";

// ═══════════════════════════════════════════════════════════
// Zenoh IPC settings
// ═══════════════════════════════════════════════════════════
namespace zenoh {
inline constexpr const char* SHM_POOL_SIZE_MB = "zenoh.shm_pool_size_mb";
inline constexpr const char* NETWORK          = "zenoh.network";
inline constexpr const char* NETWORK_ENABLED  = "zenoh.network.enabled";
}  // namespace zenoh

// ═══════════════════════════════════════════════════════════
// P1 — Video Capture
// ═══════════════════════════════════════════════════════════
namespace video_capture {
inline constexpr const char* SECTION = "video_capture";

namespace mission_cam {
inline constexpr const char* SECTION = "video_capture.mission_cam";
inline constexpr const char* BACKEND = "video_capture.mission_cam.backend";
inline constexpr const char* WIDTH   = "video_capture.mission_cam.width";
inline constexpr const char* HEIGHT  = "video_capture.mission_cam.height";
inline constexpr const char* FPS     = "video_capture.mission_cam.fps";
}  // namespace mission_cam

namespace stereo_cam {
inline constexpr const char* SECTION = "video_capture.stereo_cam";
inline constexpr const char* BACKEND = "video_capture.stereo_cam.backend";
inline constexpr const char* WIDTH   = "video_capture.stereo_cam.width";
inline constexpr const char* HEIGHT  = "video_capture.stereo_cam.height";
inline constexpr const char* FPS     = "video_capture.stereo_cam.fps";
}  // namespace stereo_cam

}  // namespace video_capture

// ═══════════════════════════════════════════════════════════
// P2 — Perception
// ═══════════════════════════════════════════════════════════
namespace perception {
inline constexpr const char* SECTION = "perception";

namespace detector {
inline constexpr const char* SECTION               = "perception.detector";
inline constexpr const char* BACKEND               = "perception.detector.backend";
inline constexpr const char* CONFIDENCE_THRESHOLD  = "perception.detector.confidence_threshold";
inline constexpr const char* NMS_THRESHOLD         = "perception.detector.nms_threshold";
inline constexpr const char* MAX_DETECTIONS        = "perception.detector.max_detections";
inline constexpr const char* MIN_CONTOUR_AREA      = "perception.detector.min_contour_area";
inline constexpr const char* SUBSAMPLE             = "perception.detector.subsample";
inline constexpr const char* MAX_FPS               = "perception.detector.max_fps";
inline constexpr const char* CONFIDENCE_MAX        = "perception.detector.confidence_max";
inline constexpr const char* CONFIDENCE_BASE       = "perception.detector.confidence_base";
inline constexpr const char* CONFIDENCE_AREA_SCALE = "perception.detector.confidence_area_scale";
inline constexpr const char* MIN_BBOX_HEIGHT_PX    = "perception.detector.min_bbox_height_px";
inline constexpr const char* MAX_ASPECT_RATIO      = "perception.detector.max_aspect_ratio";
inline constexpr const char* COLORS                = "perception.detector.colors";
inline constexpr const char* MODEL_PATH            = "perception.detector.model_path";
inline constexpr const char* INPUT_SIZE            = "perception.detector.input_size";
}  // namespace detector

namespace tracker {
inline constexpr const char* SECTION              = "perception.tracker";
inline constexpr const char* BACKEND              = "perception.tracker.backend";
inline constexpr const char* MAX_AGE              = "perception.tracker.max_age";
inline constexpr const char* MIN_HITS             = "perception.tracker.min_hits";
inline constexpr const char* MAX_ASSOCIATION_COST = "perception.tracker.max_association_cost";
inline constexpr const char* HIGH_CONF_THRESHOLD  = "perception.tracker.high_conf_threshold";
inline constexpr const char* LOW_CONF_THRESHOLD   = "perception.tracker.low_conf_threshold";
inline constexpr const char* MAX_IOU_COST         = "perception.tracker.max_iou_cost";
}  // namespace tracker

namespace radar {
inline constexpr const char* SECTION        = "perception.radar";
inline constexpr const char* BACKEND        = "perception.radar.backend";
inline constexpr const char* ENABLED        = "perception.radar.enabled";
inline constexpr const char* UPDATE_RATE_HZ = "perception.radar.update_rate_hz";
}  // namespace radar

namespace fusion {
inline constexpr const char* SECTION         = "perception.fusion";
inline constexpr const char* BACKEND         = "perception.fusion.backend";
inline constexpr const char* RATE_HZ         = "perception.fusion.rate_hz";
inline constexpr const char* FX              = "perception.fusion.fx";
inline constexpr const char* FY              = "perception.fusion.fy";
inline constexpr const char* CX              = "perception.fusion.cx";
inline constexpr const char* CY              = "perception.fusion.cy";
inline constexpr const char* CAMERA_HEIGHT_M = "perception.fusion.camera_height_m";
inline constexpr const char* ASSUMED_OBSTACLE_HEIGHT_M =
    "perception.fusion.assumed_obstacle_height_m";
inline constexpr const char* DEPTH_SCALE = "perception.fusion.depth_scale";
}  // namespace fusion

}  // namespace perception

// ═══════════════════════════════════════════════════════════
// P3 — SLAM / VIO / Navigation
// ═══════════════════════════════════════════════════════════
namespace slam {
inline constexpr const char* SECTION                 = "slam";
inline constexpr const char* IMU_RATE_HZ             = "slam.imu_rate_hz";
inline constexpr const char* VIO_RATE_HZ             = "slam.vio_rate_hz";
inline constexpr const char* VISUAL_FRONTEND_RATE_HZ = "slam.visual_frontend_rate_hz";

namespace keyframe {
inline constexpr const char* MIN_PARALLAX_PX   = "slam.keyframe.min_parallax_px";
inline constexpr const char* MIN_TRACKED_RATIO = "slam.keyframe.min_tracked_ratio";
inline constexpr const char* MAX_TIME_SEC      = "slam.keyframe.max_time_sec";
}  // namespace keyframe

namespace vio {
inline constexpr const char* SECTION            = "slam.vio";
inline constexpr const char* BACKEND            = "slam.vio.backend";
inline constexpr const char* GZ_TOPIC           = "slam.vio.gz_topic";
inline constexpr const char* SIM_SPEED_MPS      = "slam.vio.sim_speed_mps";
inline constexpr const char* GOOD_TRACE_MAX     = "slam.vio.quality.good_trace_max";
inline constexpr const char* DEGRADED_TRACE_MAX = "slam.vio.quality.degraded_trace_max";
}  // namespace vio

namespace stereo {
inline constexpr const char* FX       = "slam.stereo.fx";
inline constexpr const char* FY       = "slam.stereo.fy";
inline constexpr const char* CX       = "slam.stereo.cx";
inline constexpr const char* CY       = "slam.stereo.cy";
inline constexpr const char* BASELINE = "slam.stereo.baseline";
}  // namespace stereo

namespace imu {
inline constexpr const char* SECTION             = "slam.imu";
inline constexpr const char* BACKEND             = "slam.imu.backend";
inline constexpr const char* GYRO_NOISE_DENSITY  = "slam.imu.gyro_noise_density";
inline constexpr const char* GYRO_RANDOM_WALK    = "slam.imu.gyro_random_walk";
inline constexpr const char* ACCEL_NOISE_DENSITY = "slam.imu.accel_noise_density";
inline constexpr const char* ACCEL_RANDOM_WALK   = "slam.imu.accel_random_walk";
}  // namespace imu

}  // namespace slam

// ═══════════════════════════════════════════════════════════
// P4 — Mission Planner
// ═══════════════════════════════════════════════════════════
namespace mission_planner {
inline constexpr const char* SECTION                 = "mission_planner";
inline constexpr const char* UPDATE_RATE_HZ          = "mission_planner.update_rate_hz";
inline constexpr const char* TAKEOFF_ALTITUDE_M      = "mission_planner.takeoff_altitude_m";
inline constexpr const char* ACCEPTANCE_RADIUS_M     = "mission_planner.acceptance_radius_m";
inline constexpr const char* CRUISE_SPEED_MPS        = "mission_planner.cruise_speed_mps";
inline constexpr const char* RTL_ACCEPTANCE_RADIUS_M = "mission_planner.rtl_acceptance_radius_m";
inline constexpr const char* LANDED_ALTITUDE_M       = "mission_planner.landed_altitude_m";
inline constexpr const char* RTL_MIN_DWELL_SECONDS   = "mission_planner.rtl_min_dwell_seconds";
inline constexpr const char* SURVEY_DURATION_S       = "mission_planner.survey_duration_s";
inline constexpr const char* SURVEY_YAW_RATE         = "mission_planner.survey_yaw_rate";
inline constexpr const char* OVERSHOOT_PROXIMITY_FACTOR =
    "mission_planner.overshoot_proximity_factor";
inline constexpr const char* WAYPOINTS        = "mission_planner.waypoints";
inline constexpr const char* STATIC_OBSTACLES = "mission_planner.static_obstacles";

namespace path_planner {
inline constexpr const char* BACKEND            = "mission_planner.path_planner.backend";
inline constexpr const char* RESOLUTION_M       = "mission_planner.path_planner.resolution_m";
inline constexpr const char* GRID_EXTENT_M      = "mission_planner.path_planner.grid_extent_m";
inline constexpr const char* INFLATION_RADIUS_M = "mission_planner.path_planner.inflation_radius_m";
inline constexpr const char* REPLAN_INTERVAL_S  = "mission_planner.path_planner.replan_interval_s";
inline constexpr const char* PATH_SPEED_MPS     = "mission_planner.path_planner.path_speed_mps";
inline constexpr const char* SMOOTHING_ALPHA    = "mission_planner.path_planner.smoothing_alpha";
inline constexpr const char* MAX_ITERATIONS     = "mission_planner.path_planner.max_iterations";
inline constexpr const char* MAX_SEARCH_TIME_MS = "mission_planner.path_planner.max_search_time_ms";
inline constexpr const char* RAMP_DIST_M        = "mission_planner.path_planner.ramp_dist_m";
inline constexpr const char* MIN_SPEED_MPS      = "mission_planner.path_planner.min_speed_mps";
inline constexpr const char* SNAP_SEARCH_RADIUS = "mission_planner.path_planner.snap_search_radius";
inline constexpr const char* Z_BAND_CELLS       = "mission_planner.path_planner.z_band_cells";
inline constexpr const char* LOOK_AHEAD_M       = "mission_planner.path_planner.look_ahead_m";
inline constexpr const char* YAW_TOWARDS_TRAVEL = "mission_planner.path_planner.yaw_towards_travel";
inline constexpr const char* YAW_SMOOTHING_RATE = "mission_planner.path_planner.yaw_smoothing_rate";
inline constexpr const char* SNAP_APPROACH_BIAS = "mission_planner.path_planner.snap_approach_bias";
}  // namespace path_planner

namespace occupancy_grid {
inline constexpr const char* RESOLUTION_M = "mission_planner.occupancy_grid.resolution_m";
inline constexpr const char* INFLATION_RADIUS_M =
    "mission_planner.occupancy_grid.inflation_radius_m";
inline constexpr const char* DYNAMIC_OBSTACLE_TTL_S =
    "mission_planner.occupancy_grid.dynamic_obstacle_ttl_s";
inline constexpr const char* MIN_CONFIDENCE = "mission_planner.occupancy_grid.min_confidence";
inline constexpr const char* PROMOTION_HITS = "mission_planner.occupancy_grid.promotion_hits";
inline constexpr const char* RADAR_PROMOTION_HITS =
    "mission_planner.occupancy_grid.radar_promotion_hits";
inline constexpr const char* MIN_PROMOTION_DEPTH_CONFIDENCE =
    "mission_planner.occupancy_grid.min_promotion_depth_confidence";
inline constexpr const char* MAX_STATIC_CELLS = "mission_planner.occupancy_grid.max_static_cells";
inline constexpr const char* PREDICTION_ENABLED =
    "mission_planner.occupancy_grid.prediction_enabled";
inline constexpr const char* PREDICTION_DT_S = "mission_planner.occupancy_grid.prediction_dt_s";
}  // namespace occupancy_grid

namespace obstacle_avoidance {
inline constexpr const char* MIN_DISTANCE_M = "mission_planner.obstacle_avoidance.min_distance_m";
inline constexpr const char* INFLUENCE_RADIUS_M =
    "mission_planner.obstacle_avoidance.influence_radius_m";
inline constexpr const char* REPULSIVE_GAIN = "mission_planner.obstacle_avoidance.repulsive_gain";
inline constexpr const char* MAX_CORRECTION_MPS =
    "mission_planner.obstacle_avoidance.max_correction_mps";
inline constexpr const char* MIN_CONFIDENCE  = "mission_planner.obstacle_avoidance.min_confidence";
inline constexpr const char* PREDICTION_DT_S = "mission_planner.obstacle_avoidance.prediction_dt_s";
inline constexpr const char* VERTICAL_GAIN   = "mission_planner.obstacle_avoidance.vertical_gain";
inline constexpr const char* MAX_AGE_MS      = "mission_planner.obstacle_avoidance.max_age_ms";
inline constexpr const char* PATH_AWARE      = "mission_planner.obstacle_avoidance.path_aware";
}  // namespace obstacle_avoidance

namespace obstacle_avoider {
inline constexpr const char* BACKEND = "mission_planner.obstacle_avoider.backend";
}  // namespace obstacle_avoider

namespace geofence {
inline constexpr const char* ENABLED              = "mission_planner.geofence.enabled";
inline constexpr const char* POLYGON              = "mission_planner.geofence.polygon";
inline constexpr const char* ALTITUDE_FLOOR_M     = "mission_planner.geofence.altitude_floor_m";
inline constexpr const char* ALTITUDE_CEILING_M   = "mission_planner.geofence.altitude_ceiling_m";
inline constexpr const char* WARNING_MARGIN_M     = "mission_planner.geofence.warning_margin_m";
inline constexpr const char* ALTITUDE_TOLERANCE_M = "mission_planner.geofence.altitude_tolerance_m";
}  // namespace geofence

namespace collision_recovery {
inline constexpr const char* ENABLED       = "mission_planner.collision_recovery.enabled";
inline constexpr const char* CLIMB_DELTA_M = "mission_planner.collision_recovery.climb_delta_m";
inline constexpr const char* HOVER_DURATION_S =
    "mission_planner.collision_recovery.hover_duration_s";
}  // namespace collision_recovery

}  // namespace mission_planner

// ═══════════════════════════════════════════════════════════
// Fault Manager (used by P4)
// ═══════════════════════════════════════════════════════════
namespace fault_manager {
inline constexpr const char* POSE_STALE_TIMEOUT_MS = "fault_manager.pose_stale_timeout_ms";
inline constexpr const char* BATTERY_WARN_PERCENT  = "fault_manager.battery_warn_percent";
inline constexpr const char* BATTERY_CRIT_PERCENT  = "fault_manager.battery_crit_percent";
}  // namespace fault_manager

// ═══════════════════════════════════════════════════════════
// P5 — Comms
// ═══════════════════════════════════════════════════════════
namespace comms {
inline constexpr const char* SECTION = "comms";

namespace mavlink {
inline constexpr const char* SECTION           = "comms.mavlink";
inline constexpr const char* BACKEND           = "comms.mavlink.backend";
inline constexpr const char* URI               = "comms.mavlink.uri";
inline constexpr const char* TIMEOUT_MS        = "comms.mavlink.timeout_ms";
inline constexpr const char* SERIAL_PORT       = "comms.mavlink.serial_port";
inline constexpr const char* BAUD_RATE         = "comms.mavlink.baud_rate";
inline constexpr const char* HEARTBEAT_RATE_HZ = "comms.mavlink.heartbeat_rate_hz";
inline constexpr const char* TX_RATE_HZ        = "comms.mavlink.tx_rate_hz";
inline constexpr const char* RX_RATE_HZ        = "comms.mavlink.rx_rate_hz";
}  // namespace mavlink

namespace gcs {
inline constexpr const char* SECTION           = "comms.gcs";
inline constexpr const char* BACKEND           = "comms.gcs.backend";
inline constexpr const char* UDP_PORT          = "comms.gcs.udp_port";
inline constexpr const char* TELEMETRY_RATE_HZ = "comms.gcs.telemetry_rate_hz";
}  // namespace gcs

}  // namespace comms

// ═══════════════════════════════════════════════════════════
// P6 — Payload Manager
// ═══════════════════════════════════════════════════════════
namespace payload_manager {
inline constexpr const char* SECTION        = "payload_manager";
inline constexpr const char* UPDATE_RATE_HZ = "payload_manager.update_rate_hz";

namespace gimbal {
inline constexpr const char* SECTION           = "payload_manager.gimbal";
inline constexpr const char* BACKEND           = "payload_manager.gimbal.backend";
inline constexpr const char* MAX_SLEW_RATE_DPS = "payload_manager.gimbal.max_slew_rate_dps";
inline constexpr const char* PITCH_MIN_DEG     = "payload_manager.gimbal.pitch_min_deg";
inline constexpr const char* PITCH_MAX_DEG     = "payload_manager.gimbal.pitch_max_deg";
inline constexpr const char* YAW_MIN_DEG       = "payload_manager.gimbal.yaw_min_deg";
inline constexpr const char* YAW_MAX_DEG       = "payload_manager.gimbal.yaw_max_deg";

namespace auto_track {
inline constexpr const char* ENABLED        = "payload_manager.gimbal.auto_track.enabled";
inline constexpr const char* MIN_CONFIDENCE = "payload_manager.gimbal.auto_track.min_confidence";
inline constexpr const char* MANUAL_HOLDOFF_S =
    "payload_manager.gimbal.auto_track.manual_holdoff_s";
}  // namespace auto_track

}  // namespace gimbal
}  // namespace payload_manager

// ═══════════════════════════════════════════════════════════
// P7 — System Monitor
// ═══════════════════════════════════════════════════════════
namespace system_monitor {
inline constexpr const char* SECTION = "system_monitor";
/// Platform ISysInfo backend. One of "linux", "jetson", "mock".
inline constexpr const char* PLATFORM              = "system_monitor.platform";
inline constexpr const char* UPDATE_RATE_HZ        = "system_monitor.update_rate_hz";
inline constexpr const char* DISK_CHECK_INTERVAL_S = "system_monitor.disk_check_interval_s";
/// Battery-to-power linear coefficient (watts per %).
inline constexpr const char* POWER_COEFF = "system_monitor.power_coeff";

namespace thresholds {
inline constexpr const char* CPU_WARN_PERCENT = "system_monitor.thresholds.cpu_warn_percent";
inline constexpr const char* MEM_WARN_PERCENT = "system_monitor.thresholds.mem_warn_percent";
inline constexpr const char* TEMP_WARN_C      = "system_monitor.thresholds.temp_warn_c";
inline constexpr const char* TEMP_CRIT_C      = "system_monitor.thresholds.temp_crit_c";
inline constexpr const char* BATTERY_WARN_PERCENT =
    "system_monitor.thresholds.battery_warn_percent";
inline constexpr const char* BATTERY_CRIT_PERCENT =
    "system_monitor.thresholds.battery_crit_percent";
inline constexpr const char* DISK_CRIT_PERCENT = "system_monitor.thresholds.disk_crit_percent";
}  // namespace thresholds
}  // namespace system_monitor

// ═══════════════════════════════════════════════════════════
// Watchdog
// ═══════════════════════════════════════════════════════════
namespace watchdog {
inline constexpr const char* PROCESSES = "watchdog.processes";
}  // namespace watchdog

// ═══════════════════════════════════════════════════════════
// Flight Recorder
// ═══════════════════════════════════════════════════════════
namespace recorder {
inline constexpr const char* MAX_SIZE_MB = "recorder.max_size_mb";
inline constexpr const char* OUTPUT_DIR  = "recorder.output_dir";
}  // namespace recorder

// ═══════════════════════════════════════════════════════════
// HAL common sub-keys (appended to section prefix)
// Usage: cfg.get<std::string>(section + cfg_key::hal::BACKEND, "simulated")
// ═══════════════════════════════════════════════════════════
namespace hal {
inline constexpr const char* BACKEND  = ".backend";
inline constexpr const char* GZ_TOPIC = ".gz_topic";

// Plugin sub-keys (appended to section prefix, used when backend == "plugin")
inline constexpr const char* PLUGIN_PATH    = ".plugin_path";
inline constexpr const char* PLUGIN_FACTORY = ".plugin_factory";

// Radar-specific sub-keys (appended to section prefix)
inline constexpr const char* MAX_RANGE_M         = ".max_range_m";
inline constexpr const char* FOV_AZIMUTH_RAD     = ".fov_azimuth_rad";
inline constexpr const char* FOV_ELEVATION_RAD   = ".fov_elevation_rad";
inline constexpr const char* GROUND_FILTER_ALT_M = ".ground_filter_alt_m";
inline constexpr const char* FALSE_ALARM_RATE    = ".false_alarm_rate";
inline constexpr const char* NUM_TARGETS         = ".num_targets";
inline constexpr const char* GZ_SCAN_TOPIC       = ".gz_scan_topic";
inline constexpr const char* GZ_ODOM_TOPIC       = ".gz_odom_topic";

// Noise sub-keys (appended to section prefix)
inline constexpr const char* NOISE_RANGE_STD_M       = ".noise.range_std_m";
inline constexpr const char* NOISE_AZIMUTH_STD_RAD   = ".noise.azimuth_std_rad";
inline constexpr const char* NOISE_ELEVATION_STD_RAD = ".noise.elevation_std_rad";
inline constexpr const char* NOISE_VELOCITY_STD_MPS  = ".noise.velocity_std_mps";
}  // namespace hal

}  // namespace drone::cfg_key
