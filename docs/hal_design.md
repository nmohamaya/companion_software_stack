# Hardware Abstraction Layer (HAL) Design

This document describes the HAL interfaces that decouple process business
logic from physical hardware.  The architecture decision is captured in
[ADR-006](adr/ADR-006-hal-hardware-abstraction-strategy.md).

---

## 1. Overview

The HAL is a set of pure-virtual C++ interface classes in
`common/hal/include/hal/`.  Every process depends only on the interface;
the concrete backend (simulated, Gazebo, real hardware) is selected at
runtime via the `"backend"` key in `config/default.json` and wired up
through a factory function.

```
┌──────────────────────────────────────┐
│          Process code                │
│   std::unique_ptr<ICamera> cam_      │
│   cam_->capture()                    │
└───────────────┬──────────────────────┘
                │ virtual dispatch
    ┌───────────┼──────────────┐
    ▼           ▼              ▼
SimulatedCamera  GazeboCamera  V4L2Camera
(always)        (HAVE_GAZEBO) (future)
```

---

## 2. Common HAL Interfaces (`common/hal/include/hal/`)

### 2.1 ICamera

**Header:** `common/hal/include/hal/icamera.h`

Represents a single video stream (mission camera or stereo camera).

#### Data Structures

```cpp
struct CapturedFrame {
    uint64_t timestamp_ns;   // monotonic clock nanoseconds
    uint64_t sequence;       // monotonically increasing frame index
    uint32_t width;          // pixels
    uint32_t height;         // pixels
    uint32_t channels;       // 1 = GRAY, 3 = RGB
    uint32_t stride;         // bytes per row (may include padding)
    const uint8_t* data;     // raw pixel data; valid only for calling thread
    bool valid;              // false if capture failed or device not open
};
```

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `open` | `bool open(uint32_t w, uint32_t h, int fps)` | Open camera at requested resolution/frame rate.  Returns false on error. |
| `close` | `void close()` | Release device + buffers. |
| `capture` | `CapturedFrame capture()` | Blocking capture.  Returns frame with `valid=false` on timeout/error. |
| `is_open` | `bool is_open() const` | True only after successful `open()`. |
| `name` | `std::string name() const` | Human-readable backend identifier (e.g., `"SimulatedCamera"`). |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedCamera` | Generates a gradient test pattern at 30 fps. |
| `"gazebo"` | `GazeboCameraBackend` | Subscribes to a gz-transport topic (requires `HAVE_GAZEBO`). |
| `"v4l2"` | *(future)* | Video4Linux2 for USB/CSI cameras. |

**Config section:** `video_capture.mission_cam` or `video_capture.stereo_cam`

---

### 2.2 IFCLink

**Header:** `common/hal/include/hal/ifc_link.h`

Bidirectional link to the flight controller.  Used by P5 (comms).

#### Data Structures

```cpp
struct FCState {
    uint64_t timestamp_ns;
    float battery_voltage;   // Volts
    float battery_current;   // Amps
    float battery_percent;   // 0–100
    float altitude_rel;      // metres above takeoff point
    float ground_speed;      // m/s
    uint8_t satellites;      // GPS satellites in view
    uint8_t flight_mode;     // 0=STAB 1=GUIDED 2=AUTO 3=RTL
    bool armed;
};
```

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `open` | `bool open(const std::string& port, int baud)` | Open serial/UDP transport. |
| `close` | `void close()` | Close transport. |
| `is_connected` | `bool is_connected() const` | True if the link is healthy. |
| `send_trajectory` | `bool send_trajectory(float vx, float vy, float vz, float yaw)` | Send velocity setpoint (m/s, rad). |
| `send_arm` | `bool send_arm(bool arm)` | Arm or disarm motors. |
| `send_mode` | `bool send_mode(uint8_t mode)` | Set flight mode (see `FCState::flight_mode` codes). |
| `send_takeoff` | `bool send_takeoff(float altitude_m)` | Initiate guided takeoff. |
| `receive_state` | `FCState receive_state()` | Poll latest telemetry; non-blocking. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedFCLink` | In-memory state, always "armed/guided". |
| `"mavlink"` | `MavlinkFCLink` | MAVSDK over serial/UDP (requires `HAVE_MAVSDK`). |
| `"mavlink_v2"` | *(future)* | Direct MAVLink v2 implementation. |

**Config section:** `comms.mavlink`

---

### 2.3 IGCSLink

**Header:** `common/hal/include/hal/igcs_link.h`

Bidirectional link to the Ground Control Station.  Used by P5 (comms).

#### Data Structures

```cpp
enum class GCSCommandType : uint8_t {
    NONE      = 0,
    RTL       = 1,
    LAND      = 2,
    MISSION   = 3,
    PARAM_SET = 4,
};

struct GCSCommand {
    GCSCommandType type;
    float param1;
    float param2;
    uint64_t timestamp_ns;
    bool valid;
};
```

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `open` | `bool open(const std::string& addr, int port)` | Bind UDP/TCP socket. |
| `close` | `void close()` | Close socket. |
| `is_connected` | `bool is_connected() const` | True while socket is bound. |
| `send_telemetry` | `bool send_telemetry(float lat, float lon, float alt, float battery, uint8_t state)` | Push telemetry to GCS. |
| `poll_command` | `GCSCommand poll_command()` | Non-blocking read of latest GCS command. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedGCSLink` | In-memory loopback; commands injected via `inject_command()`. |
| `"udp"` | *(future)* | Direct MAVLink-over-UDP. |

**Config section:** `comms.gcs`

---

### 2.4 IGimbal

**Header:** `common/hal/include/hal/igimbal.h`

Rate-controlled gimbal with camera trigger.  Used by P6 (payload manager).

#### Data Structures

```cpp
struct GimbalState {
    float pitch;       // degrees, positive = up
    float yaw;         // degrees, positive = clockwise
    float roll;        // degrees
    bool stabilised;   // true when at target within tolerance
};
```

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init()` | Initialise hardware / reset to home position. |
| `is_initialised` | `bool is_initialised() const` | True after successful `init()`. |
| `set_target` | `void set_target(float pitch_deg, float yaw_deg)` | Set target orientation. |
| `update` | `void update(float dt_s)` | Advance slew control by `dt_s` seconds. |
| `state` | `GimbalState state() const` | Current orientation. |
| `capture_image` | `bool capture_image()` | Trigger stills capture. |
| `start_recording` | `bool start_recording()` | Begin video recording. |
| `stop_recording` | `bool stop_recording()` | End video recording. |
| `is_recording` | `bool is_recording() const` | True while recording is active. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedGimbal` | Slew-rate limited software model. |
| `"siyi"` | *(future)* | SIYI A8 Mini via UART. |

**Config section:** `payload_manager.gimbal`

---

### 2.5 IIMUSource

**Header:** `common/hal/include/hal/iimu_source.h`

Inertial measurement unit.  Used by P3 (SLAM/VIO).

#### Data Structures

```cpp
struct ImuReading {
    double timestamp;          // seconds, monotonic
    Eigen::Vector3d accel;     // m/s²
    Eigen::Vector3d gyro;      // rad/s
    bool valid;
};
```

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init(int rate_hz)` | Configure sampling rate and start acquisition. |
| `read` | `ImuReading read()` | Non-blocking read of latest sample. |
| `is_active` | `bool is_active() const` | True after successful `init()`. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedIMU` | Band-limited white noise at requested rate. |
| `"gazebo"` | `GazeboIMUBackend` | gz-transport IMU topic (requires `HAVE_GAZEBO`). |
| `"bmi088"` | *(future)* | Bosch BMI088 over SPI/I2C on Jetson Orin. |

**Config section:** `slam.imu`

---

## 3. Factory Functions

All factory calls live in `common/hal/include/hal/hal_factory.h`.  Each
returns a `std::unique_ptr` to the interface, transferring ownership to
the calling process.

```cpp
namespace drone::hal {

// Video cameras
std::unique_ptr<ICamera>   create_camera(const Config& cfg, const std::string& section);

// Flight controller / GCS links
std::unique_ptr<IFCLink>   create_fc_link(const Config& cfg, const std::string& section);
std::unique_ptr<IGCSLink>  create_gcs_link(const Config& cfg, const std::string& section);

// Payload
std::unique_ptr<IGimbal>   create_gimbal(const Config& cfg, const std::string& section);

// Inertial sensor
std::unique_ptr<IIMUSource> create_imu_source(const Config& cfg, const std::string& section);

} // namespace drone::hal
```

Error if requested backend is unavailable at compile time: throws
`std::runtime_error` at startup before any IPC channels open.

---

## 4. Process-Local Strategy Interfaces

The following interfaces follow the same Strategy + factory pattern but
are scoped to a single process.  They are not in `common/hal/` because
their data types reference IPC message structs that are not available
globally.

### 4.1 IPathPlanner

**Header:** `process4_mission_planner/include/planner/ipath_planner.h`
**Consumer:** P4 (mission planner)

```cpp
namespace drone::planner {
class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;
    virtual drone::ipc::ShmTrajectoryCmd plan(const drone::ipc::ShmPose& pose,
                                              const Waypoint& target) = 0;
    virtual std::string name() const = 0;
};
}  // namespace drone::planner
```

| Key | Class | Notes |
|-----|-------|-------|
| `"potential_field"` | `PotentialFieldPlanner` | 2D gradient descent, smoothing=0.35 |
| `"astar"` | `AStarPlanner` | Grid-based A* with HD-map static obstacles |

**Config:** `mission_planner.path_planner.backend`

### 4.2 IObstacleAvoider

**Header:** `process4_mission_planner/include/planner/iobstacle_avoider.h`
**Consumer:** P4 (mission planner)

```cpp
class IObstacleAvoider {
    virtual ShmTrajectoryCmd avoid(const ShmTrajectoryCmd& planned,
                                   const ShmPose& pose,
                                   const ShmDetectedObjectList& objects) = 0;
    virtual std::string name() const = 0;
};
```

| Key | Class | Notes |
|-----|-------|-------|
| `"potential_field"` | `PotentialFieldAvoider` | 2D repulsive field, influence_radius=5 m |
| `"3d"` | `ObstacleAvoider3D` | Full 3-D potential field |

**Config:** `mission_planner.obstacle_avoider.backend`

### 4.3 IProcessMonitor

**Header:** `process7_system_monitor/include/monitor/iprocess_monitor.h`
**Consumer:** P7 (system monitor)

```cpp
class IProcessMonitor {
    virtual ShmSystemHealth collect() = 0;
    virtual void set_battery_percent(float pct) = 0;
    virtual std::string name() const = 0;
};
```

| Class | Notes |
|-------|-------|
| `LinuxProcessMonitor` | Reads `/proc` for CPU / memory / temperature; calls `external_battery_percent_` |

Default thresholds: `cpu_warn=90`, `mem_warn=90`, `temp_warn=80°C`,
`temp_crit=95°C`, `disk_crit=98%`, `batt_warn=20%`, `batt_crit=10%`,
`disk_interval=10 s`.

---

## 5. Backend Availability Matrix

| Interface | Simulated | Gazebo | Real |
|-----------|-----------|--------|------|
| `ICamera` | ✓ | ✓ (HAVE_GAZEBO) | V4L2 (future) |
| `IFCLink` | ✓ | ✓ via `mavlink` + PX4 SITL | MAVSDK (`HAVE_MAVSDK`) |
| `IGCSLink` | ✓ | ✓ (via FC) | UDP (future) |
| `IGimbal` | ✓ | — | SIYI (future) |
| `IIMUSource` | ✓ | ✓ (HAVE_GAZEBO) | BMI088 (future) |

---

## 6. Adding a New Backend

1. Create `common/hal/include/hal/my_backend_camera.h` implementing `ICamera`.
2. Add a compile guard if it requires an optional library:
   ```cmake
   find_package(MyLib QUIET)
   if(MyLib_FOUND)
       target_compile_definitions(drone_hal PUBLIC HAVE_MYLIB)
   endif()
   ```
3. Add one `else if (backend == "mybackend")` branch in `hal_factory.h`:
   ```cpp
   #ifdef HAVE_MYLIB
   else if (backend == "mybackend") {
       return std::make_unique<MyBackendCamera>(cfg, section);
   }
   #endif
   ```
4. Add unit tests in `tests/hal/test_my_backend_camera.cpp`.
5. Zero process code changes required.

---

## 7. Related Documents

- [ADR-006 — HAL Strategy](adr/ADR-006-hal-hardware-abstraction-strategy.md)
- [ADR-002 — Modular IPC Backend](adr/ADR-002-modular-ipc-backend-architecture.md)
- [config_reference.md](config_reference.md) — backend keys per section
- [observability.md](observability.md) — how to trace latency across HAL boundaries
