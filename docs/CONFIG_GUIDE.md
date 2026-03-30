# Configuration Guide

> How the JSON configuration system works, what every setting means, and how
> scenario configs override defaults.

---

## Overview

The drone companion stack uses a layered JSON configuration system:

1. **`config/default.json`** — base config for all settings (simulated backends, safe defaults)
2. **`config/gazebo_sitl.json`** — overrides for Gazebo SITL simulation (color_contour detector, MAVLink FC, Gazebo cameras/IMU)
3. **`config/hardware.json`** — overrides for real hardware (V4L2 cameras, real IMU, production thermal thresholds)
4. **`config/scenarios/*.json`** — per-scenario overrides for integration testing (waypoints, faults, detector backends, planner tuning)

Settings are accessed in C++ via:
```cpp
drone::Config cfg;
cfg.load("config/default.json");
int w = cfg.get<int>("video_capture.mission_cam.width", 1920);  // dot-path access with default
auto section = cfg.section("mission_planner");                   // sub-tree access
```

Scenario configs use a `config_overrides` block that is deep-merged over the base config at runtime.

---

## Top-Level Settings

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `log_level` | string | `"info"` | spdlog level: `trace`, `debug`, `info`, `warn`, `error`, `critical` |
| `ipc_backend` | string | `"zenoh"` | IPC backend — only `"zenoh"` is supported (POSIX SHM removed in Issue #126) |

---

## Zenoh IPC (`zenoh.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `zenoh.shm_pool_size_mb` | int | `32` | SHM pool size for zero-copy transport |
| `zenoh.network.enabled` | bool | `false` | Enable network transport (TCP/UDP) for drone-to-GCS communication |
| `zenoh.network.mode` | string | `"peer"` | Zenoh mode: `"peer"`, `"router"`, `"client"` |
| `zenoh.network.listen_port` | int | `7447` | TCP/UDP listen port |
| `zenoh.network.listen_address` | string | `"127.0.0.1"` | Bind address (use `"0.0.0.0"` for remote access) |
| `zenoh.network.protocol` | string | `"tcp"` | Transport protocol: `"tcp"`, `"udp"`, `"quic"` |
| `zenoh.network.connect_endpoints` | array | `[]` | Remote endpoints to connect to (e.g., `["tcp/192.168.1.100:7447"]`) |
| `zenoh.network.multicast_scouting` | bool | `true` | Enable Zenoh multicast discovery |
| `zenoh.network.gossip_scouting` | bool | `true` | Enable Zenoh gossip discovery |

---

## Video Capture (`video_capture.*`)

### Mission Camera (`video_capture.mission_cam.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Camera backend: `"simulated"`, `"gazebo"`, `"v4l2"` |
| `width` | int | `1920` | Frame width in pixels |
| `height` | int | `1080` | Frame height in pixels |
| `fps` | int | `30` | Capture frame rate |
| `format` | string | `"RGB24"` | Pixel format |

### Stereo Camera (`video_capture.stereo_cam.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Camera backend: `"simulated"`, `"gazebo"` |
| `width` | int | `640` | Frame width in pixels |
| `height` | int | `480` | Frame height in pixels |
| `fps` | int | `30` | Capture frame rate |
| `format` | string | `"GRAY8"` | Pixel format (grayscale for VIO) |

---

## Perception (`perception.*`)

### Detector (`perception.detector.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Detector backend: `"simulated"`, `"color_contour"`, `"yolov8"` |
| `confidence_threshold` | float | `0.5` | Minimum detection confidence to pass through |
| `nms_threshold` | float | `0.4` | NMS IoU threshold (YOLOv8 only) |
| `max_detections` | int | `64` | Hard cap on detections per frame |
| `min_contour_area` | int | `80` | Minimum contour area in px² (color_contour only) |
| `subsample` | int | `2` | Spatial subsampling stride — `1` = full resolution, `2` = half each axis (color_contour only) |
| `max_fps` | int | `0` | Max detection rate in Hz; `0` = unlimited (color_contour only) |
| `confidence_max` | float | `0.95` | Maximum confidence output (color_contour) |
| `confidence_base` | float | `0.5` | Base confidence for detected contours (color_contour) |
| `confidence_area_scale` | float | `50.0` | Area-based confidence scaling factor (color_contour) |

#### Simulated Detector (`perception.detector.sim.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `min_detections` | int | `1` | Min random detections per frame |
| `max_detections` | int | `5` | Max random detections per frame |
| `margin_px` | float | `50.0` | Pixel margin from frame edges |
| `size_min_px` | float | `40.0` | Min bbox side length |
| `size_max_px` | float | `200.0` | Max bbox side length |
| `confidence_min` | float | `0.4` | Min random confidence |
| `confidence_max` | float | `0.99` | Max random confidence |
| `num_classes` | int | `5` | Number of class IDs to generate |

### Tracker (`perception.tracker.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"bytetrack"` | Tracker backend (only `"bytetrack"` supported) |
| `max_age` | int | `10` | Max consecutive misses before track deletion |
| `min_hits` | int | `3` | Consecutive matches required for track confirmation |
| `max_association_cost` | float | `100.0` | Max cost for valid association (pixels) |
| `high_conf_threshold` | float | `0.5` | Stage 1 confidence threshold (high-confidence detections) |
| `low_conf_threshold` | float | `0.1` | Stage 2 confidence threshold (low-confidence recovery) |
| `max_iou_cost` | float | `0.7` | Max IoU cost for valid association |

### Radar HAL (`perception.radar.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Radar backend: `"simulated"`, `"gazebo"` |
| `enabled` | bool | `false` | Enable radar sensor |
| `update_rate_hz` | int | `20` | Radar update rate |
| `max_range_m` | float | `100.0` | Maximum detection range |
| `fov_azimuth_rad` | float | `1.047` | Horizontal FOV (~60 degrees) |
| `fov_elevation_rad` | float | `0.698` | Vertical FOV (~40 degrees) |
| `ground_filter_alt_m` | float | `0.5` | Reject radar returns below this AGL (HAL-level ground filter) |
| `false_alarm_rate` | float | `0.02` | False alarm probability (simulated only) |
| `num_targets` | int | `3` | Number of simulated targets |

#### Radar Noise (`perception.radar.noise.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `range_std_m` | float | `0.3` | Range noise standard deviation (meters) |
| `azimuth_std_rad` | float | `0.026` | Azimuth noise standard deviation (radians, ~1.5 degrees) |
| `elevation_std_rad` | float | `0.026` | Elevation noise standard deviation (radians) |
| `velocity_std_mps` | float | `0.1` | Radial velocity noise standard deviation (m/s) |

### Fusion Engine (`perception.fusion.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `rate_hz` | int | `30` | Fusion update rate |
| `camera_weight` | float | `1.0` | Camera measurement weight |
| `camera_height_m` | float | `1.5` | Camera height above ground (for ground-plane depth) |

#### Depth Estimation (`perception.fusion.depth.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `covariance_init` | float | `5.0` | Initial position covariance |
| `bbox_h_threshold` | float | `10.0` | Minimum bbox height for apparent-size depth (pixels) |
| `depth_min_m` | float | `1.0` | Minimum depth clamp |
| `depth_max_m` | float | `40.0` | Maximum depth clamp |
| `ray_down_threshold` | float | `0.01` | Min downward ray component for ground-plane depth |
| `fallback_depth_m` | float | `8.0` | Near-horizon fallback depth |

#### Radar Fusion (`perception.fusion.radar.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `false` | Enable radar measurement updates in the UKF |
| `range_std_m` | float | `0.3` | Radar range noise std for UKF R matrix |
| `azimuth_std_rad` | float | `0.026` | Radar azimuth noise std for UKF R matrix |
| `elevation_std_rad` | float | `0.026` | Radar elevation noise std for UKF R matrix |
| `velocity_std_mps` | float | `0.1` | Radar radial velocity noise std for UKF R matrix |
| `gate_threshold` | float | `9.21` | Mahalanobis gate — chi-squared(4) at 95% confidence |
| `ground_filter_enabled` | bool | `true` | Enable UKF-level ground filter |
| `min_object_altitude_m` | float | `0.3` | Min altitude AGL for radar detections to enter UKF |
| `altitude_gate_m` | float | `2.0` | Max body-frame Z difference for radar-track association |

---

## SLAM / VIO (`slam.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `vio_rate_hz` | int | `100` | Pose publication rate |
| `visual_frontend_rate_hz` | int | `30` | Visual frontend processing rate |
| `imu_rate_hz` | int | `400` | IMU sampling rate |
| `imu.backend` | string | `"simulated"` | IMU backend: `"simulated"`, `"gazebo"` |

### Keyframe Selection (`slam.keyframe.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `min_parallax_px` | float | `15.0` | Minimum parallax for keyframe selection (pixels) |
| `min_tracked_ratio` | float | `0.6` | Minimum tracked feature ratio for keyframe |
| `max_time_sec` | float | `0.5` | Maximum time between keyframes |

---

## Mission Planner (`mission_planner.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `update_rate_hz` | int | `10` | Main loop tick rate |
| `takeoff_altitude_m` | float | `5.0` | Target takeoff altitude |
| `acceptance_radius_m` | float | `1.0` | Waypoint acceptance radius (meters) |
| `cruise_speed_mps` | float | `2.0` | Cruise speed between waypoints |
| `rtl_acceptance_radius_m` | float | `1.5` | RTL target acceptance radius |
| `landed_altitude_m` | float | `0.5` | Altitude threshold for "landed" detection |
| `rtl_min_dwell_seconds` | int | `5` | Minimum dwell at RTL position before landing |
| `survey_duration_s` | float | — | SURVEY phase duration (if present; omit to skip survey) |
| `survey_yaw_rate` | float | — | SURVEY phase yaw sweep rate (rad/s) |
| `overshoot_proximity_factor` | float | — | Waypoint overshoot proximity threshold multiplier |

### Path Planner (`mission_planner.path_planner.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"dstar_lite"` | Path planner backend |
| `ramp_dist_m` | float | `3.0` | Distance from waypoint to start speed ramping |
| `min_speed_mps` | float | `1.0` | Minimum speed during approach ramp |
| `snap_search_radius` | int | `8` | Grid cells to search for nearest free cell when start/goal is in obstacle |
| `max_search_time_ms` | int | `1500` | Maximum D* Lite search time before fallback |
| `max_iterations` | int | `200000` | Maximum D* Lite search iterations |
| `z_band_cells` | int | — | Z-band constraint for 3D search (0 = full 3D) |
| `replan_interval_s` | float | `0.5` | Time between D* Lite replans |
| `look_ahead_m` | float | — | Carrot look-ahead distance (0 = cell-by-cell) |
| `smoothing_alpha` | float | `0.5` | EMA smoothing factor for velocity output |

### Occupancy Grid (`mission_planner.occupancy_grid.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `resolution_m` | float | `0.5` | Grid cell size in meters |
| `inflation_radius_m` | float | `1.5` | Obstacle inflation radius for path planning safety margin |
| `dynamic_obstacle_ttl_s` | float | `3.0` | Time-to-live for dynamic obstacle cells before expiry |
| `min_confidence` | float | `0.3` | Minimum detection confidence to insert into grid |
| `promotion_hits` | int | `8` | Number of confirmations to promote dynamic cell to static |
| `radar_promotion_hits` | int | `3` | Radar confirmations needed for radar-confirmed promotion |

### Obstacle Avoider (`mission_planner.obstacle_avoidance.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `min_distance_m` | float | `2.0` | Minimum safe distance to obstacles |
| `influence_radius_m` | float | `5.0` | Radius within which obstacles exert repulsive force |
| `repulsive_gain` | float | `2.0` | Repulsive force gain multiplier |
| `max_correction_mps` | float | `3.0` | Maximum velocity correction per axis (m/s) |
| `min_confidence` | float | `0.3` | Minimum confidence to consider an obstacle |
| `prediction_dt_s` | float | `0.5` | Time horizon for velocity-based obstacle prediction |
| `max_age_ms` | int | `500` | Maximum obstacle age before it's considered stale |
| `vertical_gain` | float | — | Z-axis repulsion multiplier (0 = 2D avoidance only) |
| `path_aware` | bool | — | Strip backward repulsion along planned path direction |

### Geofence (`mission_planner.geofence.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `enabled` | bool | `true` | Enable geofence checks (can be disabled per-scenario) |
| `polygon` | array | 100m square | Array of `{x, y}` vertices defining the geofence boundary |
| `altitude_floor_m` | float | `0.0` | Minimum allowed altitude |
| `altitude_ceiling_m` | float | `120.0` | Maximum allowed altitude |
| `warning_margin_m` | float | `5.0` | Warning distance from boundary |
| `altitude_tolerance_m` | float | `0.5` | Altitude tolerance for floor/ceiling checks |

### Waypoints (`mission_planner.waypoints`)

Array of waypoint objects:

| Field | Type | Description |
|-------|------|-------------|
| `x` | float | North position (meters, ENU) |
| `y` | float | East position (meters, ENU) |
| `z` | float | Altitude (meters, ENU) |
| `yaw` | float | Target heading (radians) |
| `speed` | float | Cruise speed for this leg (m/s) |
| `payload_trigger` | bool | Trigger payload action on arrival |

### Static Obstacles (`mission_planner.static_obstacles`)

Array of known obstacle positions (HD-map):

| Field | Type | Description |
|-------|------|-------------|
| `x` | float | North position (meters, ENU) |
| `y` | float | East position (meters, ENU) |
| `z` | float | Altitude (meters, ENU) |
| `radius` | float | Obstacle radius (meters) |

Set to `[]` to disable HD-map and rely entirely on perception.

---

## Comms (`comms.*`)

### Flight Controller (`comms.mavlink.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | FC backend: `"simulated"`, `"mavlink"` |
| `serial_port` | string | `"/dev/ttyTHS1"` | Serial port for MAVLink (hardware only) |
| `baud_rate` | int | `921600` | Serial baud rate |
| `heartbeat_rate_hz` | int | `1` | Heartbeat frequency |
| `tx_rate_hz` | int | `20` | Trajectory command send rate |
| `rx_rate_hz` | int | `10` | FC state receive rate |

### Ground Station (`comms.gcs.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | GCS backend: `"simulated"` |
| `udp_port` | int | `14550` | UDP port for GCS communication |
| `telemetry_rate_hz` | int | `2` | Telemetry send rate |

---

## Payload Manager (`payload_manager.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `update_rate_hz` | int | `50` | Gimbal control loop rate |

### Gimbal (`payload_manager.gimbal.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Gimbal backend: `"simulated"` |
| `max_slew_rate_dps` | float | `60.0` | Maximum slew rate (degrees/second) |
| `pitch_min_deg` | float | `-90.0` | Minimum pitch (nadir) |
| `pitch_max_deg` | float | `30.0` | Maximum pitch |
| `yaw_min_deg` | float | `-180.0` | Minimum yaw |
| `yaw_max_deg` | float | `180.0` | Maximum yaw |

---

## Fault Manager (`fault_manager.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `pose_stale_timeout_ms` | int | `500` | Pose data staleness threshold — triggers LOITER |
| `battery_warn_percent` | float | `30.0` | Battery level for WARN action |
| `battery_rtl_percent` | float | `20.0` | Battery level for RTL action |
| `battery_crit_percent` | float | `10.0` | Battery level for EMERGENCY_LAND action |
| `fc_link_lost_timeout_ms` | int | `3000` | FC link loss threshold — triggers LOITER |
| `fc_link_rtl_timeout_ms` | int | `15000` | FC link RTL timeout |
| `loiter_escalation_timeout_s` | int | `30` | Time in LOITER before auto-escalation to RTL |

---

## System Monitor (`system_monitor.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `update_rate_hz` | int | `1` | Health check frequency |
| `disk_check_interval_s` | int | `10` | Disk usage check interval (avoids popen overhead) |

### Thresholds (`system_monitor.thresholds.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `cpu_warn_percent` | float | `90.0` | CPU usage warning threshold |
| `mem_warn_percent` | float | `90.0` | Memory usage warning threshold |
| `temp_warn_c` | float | `105.0` | Temperature warning threshold (dev-safe; use 80.0 for hardware) |
| `temp_crit_c` | float | `120.0` | Temperature critical threshold (dev-safe; use 95.0 for hardware) |
| `battery_warn_percent` | float | `20.0` | Battery warning threshold |
| `battery_crit_percent` | float | `10.0` | Battery critical threshold |
| `disk_crit_percent` | float | `98.0` | Disk usage critical threshold |

---

## Watchdog (`watchdog.*`)

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `thread_stuck_threshold_ms` | int | `5000` | Thread stuck detection timeout |
| `thread_scan_interval_ms` | int | `1000` | Watchdog scan interval |

### Per-Process Config (`watchdog.processes.<name>.*`)

Each process (video_capture, perception, slam_vio_nav, comms, mission_planner, payload_manager) has:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `critical` | bool | varies | If true, death triggers LOITER in fault manager |
| `max_restarts` | int | `3`-`5` | Max restart attempts before marking FAILED |
| `cooldown_s` | int | `60` | Stable time before restart counter resets |
| `backoff_ms` | int | `500` | Initial restart backoff (doubles each attempt) |
| `max_backoff_ms` | int | `30000` | Maximum backoff cap |
| `thermal_gate` | int | `3` | Thermal zone above which restarts are blocked |
| `launch_after` | array | varies | Dependencies that must launch first |
| `restart_cascade` | array | varies | Processes to restart when this one crashes |

**Critical processes:** `comms` and `slam_vio_nav` are critical by default. Their death triggers LOITER via fault manager.

**Launch order:** Defined by `launch_after` edges — topologically sorted for startup sequencing.

**Cascade restarts:** `restart_cascade` edges define failure propagation — e.g., comms crash cascades to mission_planner and payload_manager.

---

## Scenario Configuration

Scenario configs in `config/scenarios/` follow this structure:

```json
{
    "scenario": {
        "name": "scenario_name",
        "description": "What this scenario tests",
        "tier": 1,
        "timeout_s": 120,
        "requires_gazebo": false
    },
    "config_overrides": {
        "mission_planner": { ... },
        "perception": { ... }
    },
    "fault_sequence": {
        "steps": [
            {"delay_s": 30, "action": "set_battery", "value": 15}
        ]
    },
    "pass_criteria": {
        "log_contains": ["Mission complete"],
        "log_must_not_contain": ["EMERGENCY_LAND", "OBSTACLE COLLISION"],
        "processes_alive": ["video_capture", "perception", ...]
    }
}
```

- **`config_overrides`** — deep-merged over the base config (default.json + gazebo_sitl.json for Tier 2)
- **`fault_sequence`** — timed fault injection steps executed by `fault_injector`
- **`pass_criteria`** — automated verification checks (log patterns, process liveness)
- **`tier`** — 1 = simulated (no Gazebo), 2 = Gazebo SITL required

### Available Scenarios

| # | Name | Tier | What it tests |
|---|------|------|---------------|
| 01 | nominal_mission | 1 | 4-waypoint flight, no faults |
| 02 | obstacle_avoidance | 2 | HD-map D* Lite through obstacle field |
| 03 | battery_degradation | 1 | 3-tier battery escalation |
| 04 | fc_link_loss | 1 | FC disconnect contingency |
| 05 | geofence_breach | 1 | Geofence violation triggers RTL |
| 06 | mission_upload | 1 | Mid-flight waypoint upload via GCS |
| 07 | thermal_throttle | 1 | Thermal zone escalation |
| 08 | full_stack_stress | 1 | Concurrent faults + high-rate stress |
| 09 | perception_tracking | 1 | ByteTrack backend smoke test |
| 10 | gcs_pause_resume | 1 | GCS pause + resume during NAVIGATE |
| 11 | gcs_abort | 1 | GCS abort triggers immediate LAND |
| 12 | gcs_rtl | 1 | GCS RTL command |
| 13 | gcs_land | 1 | GCS LAND command |
| 14 | altitude_ceiling | 1 | Waypoint above ceiling triggers RTL |
| 15 | fc_quick_recovery | 1 | FC link reconnect after brief loss |
| 16 | vio_failure | 2 | VIO degradation fault escalation |
| 17 | radar_gazebo | 2 | Gazebo radar HAL + UKF fusion validation |
| 18 | perception_avoidance | 2 | Camera + radar obstacle avoidance (no HD-map) |
