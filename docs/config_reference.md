# Configuration Parameter Reference

Complete reference for all runtime parameters in `config/default.json`.
Values are read by `drone::Config::get<T>(key, default)` throughout the
codebase.

---

## Notation

| Column | Description |
|--------|-------------|
| **Key** | JSON dot-path from the root object |
| **Type** | C++ type used by `cfg.get<T>()` |
| **Default** | Value in `config/default.json` |
| **Range** | Valid input range or enumeration |
| **Process** | Which process(es) read this key |
| **Description** | Purpose and units |

---

## 1. Global

| Key | Type | Default | Range | Process | Description |
|-----|------|---------|-------|---------|-------------|
| `log_level` | `string` | `"info"` | `"debug"`, `"info"`, `"warn"`, `"error"` | All | Minimum log severity printed to stderr |
| `ipc_backend` | `string` | `"shm"` | `"shm"`, `"zenoh"` | All | IPC transport backend (`"shm"` in the default config; set to `"zenoh"` for production Zenoh sessions) |

---

## 2. Zenoh Transport (`zenoh`)

| Key | Type | Default | Range | Process | Description |
|-----|------|---------|-------|---------|-------------|
| `zenoh.shm_pool_size_mb` | `int` | `32` | 16–512 | All | Zenoh shared-memory pool allocated per process |
| `zenoh.network.enabled` | `bool` | `false` | — | All | Enable WAN/LAN networking (false = loopback only) |
| `zenoh.network.mode` | `string` | `"peer"` | `"peer"`, `"client"`, `"router"` | All | Zenoh session role |
| `zenoh.network.listen_port` | `int` | `7447` | 1024–65535 | All | UDP/TCP listen port for peer discovery |
| `zenoh.network.listen_address` | `string` | `"0.0.0.0"` | Valid IPv4/IPv6 | All | Bind address for incoming connections |
| `zenoh.network.protocol` | `string` | `"tcp"` | `"udp"`, `"tcp"` | All | Underlying transport protocol |
| `zenoh.network.connect_endpoints` | `[]string` | `[]` | URIs | All | Static peer addresses, e.g. `["udp/192.168.1.10:7447"]` |
| `zenoh.network.multicast_scouting` | `bool` | `true` | — | All | Auto-discovery via multicast on LAN |
| `zenoh.network.gossip_scouting` | `bool` | `true` | — | All | Gossip-based peer discovery |

> **Note:** For GCS-connected deployments set `network.enabled=true`, configure `listen_address` and disable multicast scouting for security.

---

## 3. Video Capture (`video_capture`) — Process 1

### 3.1 Mission Camera

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `video_capture.mission_cam.backend` | `string` | `"simulated"` | `"simulated"`, `"gazebo"` | HAL backend for the primary colour camera |
| `video_capture.mission_cam.width` | `int` | `1920` | 320–3840 | Frame width in pixels |
| `video_capture.mission_cam.height` | `int` | `1080` | 240–2160 | Frame height in pixels |
| `video_capture.mission_cam.fps` | `int` | `30` | 1–120 | Target frame rate |
| `video_capture.mission_cam.format` | `string` | `"RGB24"` | `"RGB24"` | Pixel format; only RGB24 supported currently |

### 3.2 Stereo Camera

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `video_capture.stereo_cam.backend` | `string` | `"simulated"` | `"simulated"`, `"gazebo"` | HAL backend for the stereo pair |
| `video_capture.stereo_cam.width` | `int` | `640` | 160–1280 | Frame width in pixels |
| `video_capture.stereo_cam.height` | `int` | `480` | 120–960 | Frame height in pixels |
| `video_capture.stereo_cam.fps` | `int` | `30` | 1–60 | Target frame rate |
| `video_capture.stereo_cam.format` | `string` | `"GRAY8"` | `"GRAY8"` | Pixel format; 8-bit monochrome |

---

## 4. Perception (`perception`) — Process 2

### 4.1 Detector

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `perception.detector.backend` | `string` | `"simulated"` | `"simulated"`, `"color_contour"` | Object detection backend |
| `perception.detector.confidence_threshold` | `float` | `0.5` | 0.0–1.0 | Minimum detection confidence to accept |
| `perception.detector.nms_threshold` | `float` | `0.4` | 0.0–1.0 | Non-maximum suppression IoU threshold |
| `perception.detector.max_detections` | `int` | `64` | 1–256 | Maximum detections per frame |

### 4.2 Tracker

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `perception.tracker.max_age` | `int` | `10` | 1–50 | Frames to retain an unmatched track before dropping |
| `perception.tracker.min_hits` | `int` | `3` | 1–10 | Minimum consecutive hits before a track is confirmed |
| `perception.tracker.max_association_cost` | `double` | `100.0` | > 0 | Hungarian assignment cost gate |

### 4.3 Sensor Fusion

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `perception.fusion.camera_weight` | `float` | `1.0` | 0.0–1.0 | Contribution of camera detections to fused position |
| `perception.fusion.camera_height_m` | `float` | `1.5` | 0.1–10.0 | Assumed height of detected objects above ground (m), used for depth estimation without lidar |

---

## 5. SLAM / VIO Navigation (`slam`) — Process 3

### 5.1 Rate Control

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `slam.vio_rate_hz` | `int` | `100` | 10–1000 | Visual-inertial odometry state update rate |
| `slam.visual_frontend_rate_hz` | `int` | `30` | 5–60 | Feature extraction / tracking rate |
| `slam.imu_rate_hz` | `int` | `400` | 100–1000 | IMU integration rate |

### 5.2 IMU Source

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `slam.imu.backend` | `string` | `"simulated"` | `"simulated"`, `"gazebo"`, `"bmi088"` | HAL backend for the inertial sensor |

### 5.3 Keyframe Selection

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `slam.keyframe.min_parallax_px` | `float` | `15.0` | 1.0–100.0 | Minimum feature parallax to trigger keyframe insertion |
| `slam.keyframe.min_tracked_ratio` | `float` | `0.6` | 0.1–1.0 | Minimum fraction of features to maintain re-use (drop keyframe if below) |
| `slam.keyframe.max_time_sec` | `float` | `0.5` | 0.05–5.0 | Force keyframe insertion after this interval regardless of parallax |

---

## 6. Mission Planner (`mission_planner`) — Process 4

### 6.1 Core

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `mission_planner.update_rate_hz` | `int` | `10` | 1–50 | FSM + planner update loop rate |
| `mission_planner.takeoff_altitude_m` | `float` | `5.0` | 1.0–120.0 | Commanded takeoff altitude above home (m) |
| `mission_planner.acceptance_radius_m` | `float` | `1.0` | 0.1–20.0 | Waypoint arrival acceptance sphere radius (m) |
| `mission_planner.cruise_speed_mps` | `float` | `2.0` | 0.1–15.0 | Default cruise speed (m/s) |
| `mission_planner.rtl_acceptance_radius_m` | `float` | `1.5` | 0.1–20.0 | RTL home acceptance radius (m) |
| `mission_planner.landed_altitude_m` | `float` | `0.5` | 0.0–5.0 | Altitude below which the vehicle is considered landed (m) |
| `mission_planner.rtl_min_dwell_seconds` | `int` | `5` | 0–60 | Minimum seconds to loiter at home before landing on RTL |

### 6.2 Path Planner

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `mission_planner.path_planner.backend` | `string` | `"potential_field"` | `"potential_field"`, `"astar"` | Path planning algorithm |

### 6.3 Obstacle Avoider

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `mission_planner.obstacle_avoider.backend` | `string` | `"potential_field"` | `"potential_field"`, `"3d"` | Online obstacle avoidance algorithm |
| `mission_planner.obstacle_avoidance.min_distance_m` | `float` | `2.0` | 0.5–20.0 | Minimum clearance maintained from detected obstacles (m) |
| `mission_planner.obstacle_avoidance.influence_radius_m` | `float` | `5.0` | 1.0–50.0 | Repulsive field activation radius (m) |
| `mission_planner.obstacle_avoidance.repulsive_gain` | `float` | `2.0` | 0.1–10.0 | Repulsive potential gain coefficient |

### 6.4 Geofence

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `mission_planner.geofence.polygon` | `[]object{x,y}` | 50 m square around origin | — | Ordered polygon vertices in local horizontal frame (x = East m, y = North m) defining the allowed operating area |
| `mission_planner.geofence.altitude_floor_m` | `float` | `0.0` | -50–0 | Minimum allowed altitude AGL (m) |
| `mission_planner.geofence.altitude_ceiling_m` | `float` | `120.0` | 0–600 | Maximum allowed altitude AGL (m, ICAO Class G max) |
| `mission_planner.geofence.warning_margin_m` | `float` | `5.0` | 0–20 | Distance inside geofence boundary that triggers a warning before enforcement |

### 6.5 Waypoints

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `mission_planner.waypoints` | `[]object` | 4 waypoints | — | Ordered mission waypoints |
| `mission_planner.waypoints[].x` | `float` | varies | — | X offset from home (m) |
| `mission_planner.waypoints[].y` | `float` | varies | — | Y offset from home (m) |
| `mission_planner.waypoints[].z` | `float` | varies | — | Altitude AGL (m) |
| `mission_planner.waypoints[].yaw` | `float` | `0.0` | -π–π | Heading at waypoint (rad) |
| `mission_planner.waypoints[].speed` | `float` | `2.0` | 0.1–15.0 | Speed to this waypoint (m/s); 0 = use cruise_speed |
| `mission_planner.waypoints[].payload_trigger` | `bool` | `false` | — | If true, trigger payload action (image capture) on arrival |

---

## 7. Communications (`comms`) — Process 5

### 7.1 MAVLink / Flight Controller

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `comms.mavlink.backend` | `string` | `"simulated"` | `"simulated"`, `"mavlink"` | FC link HAL backend |
| `comms.mavlink.serial_port` | `string` | `"/dev/ttyTHS1"` | valid path | Serial device connecting to FC (Jetson Orin UART) |
| `comms.mavlink.baud_rate` | `int` | `921600` | 9600–3000000 | Serial baud rate |
| `comms.mavlink.heartbeat_rate_hz` | `int` | `1` | 1–10 | MAVLink heartbeat transmit rate |
| `comms.mavlink.tx_rate_hz` | `int` | `20` | 1–100 | Trajectory command transmit rate |
| `comms.mavlink.rx_rate_hz` | `int` | `10` | 1–100 | FC state poll rate |

### 7.2 Ground Control Station

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `comms.gcs.backend` | `string` | `"simulated"` | `"simulated"`, `"udp"` | GCS link HAL backend |
| `comms.gcs.udp_port` | `int` | `14550` | 1024–65535 | UDP port for GCS communication |
| `comms.gcs.telemetry_rate_hz` | `int` | `2` | 1–10 | Telemetry transmit rate to GCS |

---

## 8. Payload Manager (`payload_manager`) — Process 6

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `payload_manager.update_rate_hz` | `int` | `50` | 10–200 | Gimbal slew control loop rate |
| `payload_manager.gimbal.backend` | `string` | `"simulated"` | `"simulated"`, `"siyi"` | Gimbal HAL backend |
| `payload_manager.gimbal.max_slew_rate_dps` | `float` | `60.0` | 1–360 | Maximum angular slew rate (degrees/second) |
| `payload_manager.gimbal.pitch_min_deg` | `float` | `-90.0` | -90–0 | Minimum pitch limit (degrees, negative = down) |
| `payload_manager.gimbal.pitch_max_deg` | `float` | `30.0` | 0–90 | Maximum pitch limit (degrees) |
| `payload_manager.gimbal.yaw_min_deg` | `float` | `-180.0` | -360–0 | Minimum yaw limit (degrees) |
| `payload_manager.gimbal.yaw_max_deg` | `float` | `180.0` | 0–360 | Maximum yaw limit (degrees) |

---

## 9. Fault Manager (`fault_manager`) — Process 4

These are read by the mission planner FSM to drive fault-recovery
transitions (LOITER → RTL → LAND).

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `fault_manager.pose_stale_timeout_ms` | `int` | `500` | 100–5000 | ms since last SLAM pose before entering LOITER |
| `fault_manager.battery_warn_percent` | `float` | `30.0` | 0–100 | Battery level at which a warning is raised |
| `fault_manager.battery_rtl_percent` | `float` | `20.0` | 0–100 | Battery level that triggers automatic RTL |
| `fault_manager.battery_crit_percent` | `float` | `10.0` | 0–100 | Battery level that triggers immediate LAND |
| `fault_manager.fc_link_lost_timeout_ms` | `int` | `3000` | 500–30000 | ms FC link silent before heartbeat-loss fault |
| `fault_manager.fc_link_rtl_timeout_ms` | `int` | `15000` | 1000–60000 | ms FC link lost before escalation to RTL |
| `fault_manager.loiter_escalation_timeout_s` | `int` | `30` | 5–300 | s in LOITER before auto-escalating to RTL |

---

## 10. System Monitor (`system_monitor`) — Process 7

### 10.1 Update Rates

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `system_monitor.update_rate_hz` | `int` | `1` | 1–10 | Health status publish rate |
| `system_monitor.disk_check_interval_s` | `int` | `10` | 1–60 | How often to re-sample disk usage (expensive `statvfs` call) |

### 10.2 Thresholds

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `system_monitor.thresholds.cpu_warn_percent` | `float` | `90.0` | 0–100 | CPU usage warning threshold |
| `system_monitor.thresholds.mem_warn_percent` | `float` | `90.0` | 0–100 | System memory usage warning threshold |
| `system_monitor.thresholds.temp_warn_c` | `float` | `80.0` | 0–150 | Die temperature warning threshold (°C) |
| `system_monitor.thresholds.temp_crit_c` | `float` | `95.0` | 0–150 | Die temperature critical threshold; triggers process shutdown (°C) |
| `system_monitor.thresholds.battery_warn_percent` | `float` | `20.0` | 0–100 | Battery warning threshold (mirrors fault_manager; both must agree) |
| `system_monitor.thresholds.battery_crit_percent` | `float` | `10.0` | 0–100 | Battery critical threshold |
| `system_monitor.thresholds.disk_crit_percent` | `float` | `98.0` | 0–100 | Disk full critical threshold |

---

## 11. Watchdog (`watchdog`) — P7 ThreadWatchdog + ProcessManager

### 11.1 Thread Watchdog

| Key | Type | Default | Range | Description |
|-----|------|---------|-------|-------------|
| `watchdog.thread_stuck_threshold_ms` | `int` | `5000` | 500–60000 | ms without a heartbeat before a thread is considered stuck |
| `watchdog.thread_scan_interval_ms` | `int` | `1000` | 100–10000 | How often ThreadWatchdog scans all registered threads |

### 11.2 Process Restart Policy

Keyed by process short name (e.g., `"video_capture"`, `"perception"`,
`"slam_vio_nav"`, `"mission_planner"`, `"comms"`, `"payload_manager"`,
`"system_monitor"`).

| Sub-key | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `critical` | `bool` | varies | — | If true, losing this process triggers a system-wide restart cascade |
| `max_restarts` | `int` | varies | 0–20 | Maximum restart attempts before giving up (-1 = unlimited) |
| `cooldown_s` | `int` | `5` | 0–300 | Minimum seconds between restarts |
| `backoff_ms` | `int` | `500` | 0–5000 | Initial exponential backoff delay (ms) |
| `max_backoff_ms` | `int` | `30000` | 0–300000 | Maximum backoff cap (ms) |
| `thermal_gate` | `bool` | `false` | — | If true, prevent restart when `temp_crit_c` is exceeded |
| `launch_after` | `[]string` | `[]` | process names | Dependencies that must be healthy before this process starts |
| `restart_cascade` | `[]string` | `[]` | process names | Processes to also restart when this process is restarted |

---

## 12. Cross-Reference: Config Key → HAL Backend

| Config key | HAL factory call | Interface |
|------------|-----------------|-----------|
| `video_capture.mission_cam.backend` | `create_camera(cfg, "video_capture.mission_cam")` | `ICamera` |
| `video_capture.stereo_cam.backend` | `create_camera(cfg, "video_capture.stereo_cam")` | `ICamera` |
| `slam.imu.backend` | `create_imu_source(cfg, "slam.imu")` | `IIMUSource` |
| `comms.mavlink.backend` | `create_fc_link(cfg, "comms.mavlink")` | `IFCLink` |
| `comms.gcs.backend` | `create_gcs_link(cfg, "comms.gcs")` | `IGCSLink` |
| `payload_manager.gimbal.backend` | `create_gimbal(cfg, "payload_manager.gimbal")` | `IGimbal` |
| `mission_planner.path_planner.backend` | `create_path_planner(backend)` | `IPathPlanner` |
| `mission_planner.obstacle_avoider.backend` | `create_obstacle_avoider(backend, ...)` | `IObstacleAvoider` |

---

## 13. Adding a New Parameter

1. Add the key to `config/default.json` with a sensible default.
2. Read it with `cfg.get<T>("section.key", default_value)` — never hard-code the magic number.
3. Document it in this file under the appropriate section.
4. Add a unit test that verifies the new parameter is loaded and applied.

*See [CPP_PATTERNS_GUIDE.md](CPP_PATTERNS_GUIDE.md) for `Config` API usage.*
