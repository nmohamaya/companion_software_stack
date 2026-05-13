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

**Current interfaces:** see `common/hal/include/hal/` for the authoritative list
(every `i*.h` is an interface; non-prefixed headers are concrete backends or
support types).  Newer interfaces (`IDepthEstimator`, `IEventCamera`,
`IInferenceBackend`, `ISemanticProjector`, `IVolumetricMap`) were added by
Epics E1/E5 to support the Perception v2 PATH A pipeline.

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

### 2.6 IRadar

**Header:** `common/hal/include/hal/iradar.h`

Radar sensor interface for short-range obstacle detection and velocity measurement. Used by P2 (perception) or a dedicated radar process. Returns `RadarDetectionList` IPC structs on `/radar_detections`.

#### Data Structures

See `common/ipc/include/ipc/ipc_types.h` for `RadarDetection` and `RadarDetectionList` (capacity: `MAX_RADAR_DETECTIONS = 128`).

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init()` | Initialise hardware / start acquisition. Returns false on error. |
| `read` | `RadarDetectionList read()` | Non-blocking read of latest detections. Before `init()` returns a default-constructed list (`num_detections == 0`). |
| `is_active` | `bool is_active() const` | True after successful `init()`. |
| `name` | `std::string name() const` | Human-readable backend identifier (e.g., `"SimulatedRadar"`). |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedRadar` | Configurable FoV, range, target count, and Gaussian noise model. |
| `"gazebo"` | `GazeboRadarBackend` | gpu_lidar + odometry via gz-transport; noise injected in HAL (requires `HAVE_GAZEBO`). |

**Config section:** `perception.radar`

---

### 2.7 IDepthEstimator

**Header:** `common/hal/include/hal/idepth_estimator.h`

Per-frame dense depth estimation from a single RGB image. Consumed by P2's Perception v2 PATH A pipeline to back-project segmentation masks into 3D voxel updates.

#### Data Structures

`DepthMap` — row-major dense float depth (metres) plus metadata (timestamp, scale, confidence, source-frame dimensions for bbox→depth coordinate mapping). See `common/hal/include/hal/idepth_estimator.h` for the canonical struct definition.

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `estimate` | `[[nodiscard]] Result<DepthMap, std::string> estimate(const uint8_t* frame, w, h, channels, stride=0)` | Estimate depth from one RGB frame. `stride=0` means tightly packed. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedDepthEstimator` | Synthetic depth from a configurable scene model. |
| `"depth_anything_v2"` | `DepthAnythingV2Estimator` | OpenCV DNN — monocular metric depth. |
| `"cosys_airsim"` | `CosysDepthBackend` | Cosys-AirSim depth camera passthrough (Tier 3 sim only; requires `HAVE_COSYS_AIRSIM`). |

**Config section:** `perception.depth_estimator` (canonical keys in `common/util/include/util/config_keys.h`; factory in `common/hal/include/hal/hal_factory.h::create_depth_estimator`).

---

### 2.8 IEventCamera

**Header:** `common/hal/include/hal/ievent_camera.h`

Event-based (DVS) camera interface — produces `EventCD` (x, y, polarity, timestamp) streams instead of dense frames. Forward-looking interface for low-latency motion-only perception; not yet on the flight-critical path.

#### Data Structures

`EventCD` and `EventBatch` — see header.

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `open` | `bool open(uint32_t width, uint32_t height)` | Open the sensor at the given resolution. |
| `close` | `void close()` | Release resources. |
| `read_events` | `EventBatch read_events()` | Read the next batch. Check `EventBatch::valid`. |
| `is_open` | `bool is_open() const` | Sensor state. |
| `pixel_format` | `PixelFormat pixel_format() const` | Reported by the backend. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedEventCamera` | Synthetic events generated from a moving scene. |

**Config section:** `perception.event_camera` (reserved; consumers TBD).

---

### 2.9 IInferenceBackend

**Header:** `common/hal/include/hal/iinference_backend.h`

Generic inference interface for vision models (object detection, segmentation). Returns `InferenceOutput` containing `InferenceDetection`s with optional mask data — used by both YOLO-style detectors and SAM-style mask producers.

#### Data Structures

`BoundingBox2D`, `InferenceDetection`, `InferenceOutput` — see header.

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init(const std::string& model_path, int input_size)` | Load model; `input_size` is the model's square input dimension (e.g. 640 for YOLOv8). |
| `infer` | `[[nodiscard]] Result<InferenceOutput, std::string> infer(const uint8_t* frame, w, h, channels, stride=0)` | Run inference on one frame. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

Keys served by `common/hal/include/hal/hal_factory.h::create_inference_backend()`:

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedInferenceBackend` | Synthetic detections from a configurable scene model. |
| `"sam_simulated"` | `SimulatedSAMBackend` | Class-agnostic SAM dev/test scaffold. |
| `"edge_contour_sam"` | `EdgeContourSAMBackend` | OpenCV edge+contour mask producer (no ML dependency); interim fill between `sam_simulated` and a real SAM ONNX. |
| `"fastsam"` | `FastSamInferenceBackend` | Real FastSAM via ONNX (YOLOv8-seg architecture trained on SA-1B). Requires `models/fastsam_s.onnx`. |
| `"cosys_airsim"` | `CosysSegmentationBackend` | Sim-only ground-truth instance segmentation (requires `HAVE_COSYS_AIRSIM`); pair with `depth_estimator.backend = cosys_airsim` for GT PATH A. |
| `"plugin"` | (dlopened) | Out-of-tree backend (requires `HAVE_PLUGINS`). |

> **Note:** `YoloSegInferenceBackend` is part of the perception pipeline but is **not** constructed by this HAL factory — it's wired in `process2_perception` directly. The HAL factory above is for SAM-style mask producers + the simulated detector.

**Config section:** `perception.inference_backend` (canonical key path; the `MaskClassAssigner` in PATH A pairs the SAM mask producer with a class-aware detector — see `process2_perception` for how the two are composed).

---

### 2.10 ISemanticProjector

**Header:** `common/hal/include/hal/isemantic_projector.h`

Back-projects 2D masked detections into 3D voxel updates using a depth map and camera pose. Consumed by P2's PATH A orchestrator (`MaskDepthProjector`); output flows to P4's volumetric map via `/semantic_voxels`.

#### Data Structures

`CameraIntrinsics`, `VoxelUpdate` (from `ivolumetric_map.h`).

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init(const CameraIntrinsics&)` | Configure intrinsics. |
| `project` | `[[nodiscard]] Result<std::vector<VoxelUpdate>, std::string> project(const std::vector<InferenceDetection>&, const DepthMap&, const Eigen::Affine3f& camera_pose)` | Produce voxel updates for a single frame. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"cpu"` | `CpuSemanticProjector` | Header-only CPU back-projection; samples mask at a 4×4 grid per detection. |

**Config section:** `perception.semantic_projector` (canonical keys in `common/util/include/util/config_keys.h`; factory in `common/hal/include/hal/hal_factory.h::create_semantic_projector`).

---

### 2.11 IVolumetricMap

**Header:** `common/hal/include/hal/ivolumetric_map.h`

3D voxel map abstraction consumed by P4's obstacle avoider. `VoxelUpdate`s come from PATH A (P2 → `/semantic_voxels` → P4) and from depth-only inputs.

#### Data Structures

`VoxelKey`, `VoxelData`, `VoxelUpdate` (with `semantic_label`).

#### Interface

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init(float resolution_m)` | Initialise with voxel side length in metres. |
| `insert` | `bool insert(const std::vector<VoxelUpdate>&)` | Insert updates. |
| `query` | `std::optional<VoxelData> query(const Eigen::Vector3f& position_m)` | Lookup at a world-frame position. |
| `size` | `size_t size() const` | Number of occupied voxels. |
| `clear` | `void clear()` | Empty the map. |
| `resolution` | `float resolution() const` | Voxel size in metres. |
| `name` | `std::string name() const` | Backend identifier. |

#### Backends

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedVolumetricMap` | In-memory hash-map keyed by `VoxelKey`. |

**Config section:** `perception.volumetric_map` (canonical keys in `common/util/include/util/config_keys.h`; factory in `common/hal/include/hal/hal_factory.h::create_volumetric_map`).

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

// Radar
std::unique_ptr<IRadar>     create_radar(const Config& cfg, const std::string& section);

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
| `"dstar_lite"` | `DStarLitePlanner` | 3D incremental D* Lite search with obstacle awareness |

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
| `"potential_field_3d"` | `ObstacleAvoider3D` | Full 3D repulsive field + velocity prediction |
| `"3d"` | `ObstacleAvoider3D` | Alias for `potential_field_3d` |
| `"obstacle_avoider_3d"` | `ObstacleAvoider3D` | Alias for `potential_field_3d` |

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

### 4.4 IDetector (Perception)

**Header:** `process2_perception/include/perception/detector_interface.h`
**Consumer:** P2 (perception)

Strategy interface for object detection.  `SimulatedDetector` was originally
defined inline in `detector_interface.h` and has been extracted into its own
file for clarity.

| Key | Class | Notes |
|-----|-------|-------|
| `"simulated"` | `SimulatedDetector` | Config-driven synthetic detections (extracted from `detector_interface.h`). |
| `"color_contour"` | `ColorContourDetector` | HSV colour-space contour detector. |
| `"opencv_yolo"` | `OpenCvYoloDetector` | YOLOv8-nano via OpenCV DNN (optional, requires `HAS_OPENCV`). |

**Config:** `perception.detector.backend`

---

## 5. Backend Availability Matrix

| Interface | Simulated | Gazebo | Real | Plugin |
|-----------|-----------|--------|------|--------|
| `ICamera` | ✓ | ✓ (HAVE_GAZEBO) | V4L2 (future) | ✓ (HAVE_PLUGINS) |
| `IFCLink` | ✓ | ✓ via `mavlink` + PX4 SITL | MAVSDK (`HAVE_MAVSDK`) | ✓ (HAVE_PLUGINS) |
| `IGCSLink` | ✓ | ✓ (via FC) | UDP (future) | ✓ (HAVE_PLUGINS) |
| `IGimbal` | ✓ | — | SIYI (future) | ✓ (HAVE_PLUGINS) |
| `IIMUSource` | ✓ | ✓ (HAVE_GAZEBO) | BMI088 (future) | ✓ (HAVE_PLUGINS) |
| `IRadar` | ✓ | ✓ (HAVE_GAZEBO) | TI AWR1843 (future) | ✓ (HAVE_PLUGINS) |

---

## 5.5 Plugin Backend Support — Epic #284, Issue #295

> **Gate:** `ENABLE_PLUGINS=ON` → `HAVE_PLUGINS` compile definition
> **File:** `common/hal/include/hal/hal_factory.h` (plugin dispatch), `common/util/include/util/plugin_loader.h` (loader)

When `ENABLE_PLUGINS` is enabled, all 6 HAL factory functions gain a `"plugin"` backend that loads a `.so` at runtime via `dlopen`. This enables third-party or hardware-specific backends without recompiling the stack.

**Config example:**

```json
{
  "video_capture.mission_cam.backend": "plugin",
  "video_capture.mission_cam.plugin_path": "/opt/drone/plugins/libv4l2_camera.so",
  "video_capture.mission_cam.plugin_factory": "create_camera"
}
```

**Plugin .so contract:**

- Factory function must use `extern "C"` linkage (no name mangling)
- Factory must have `__attribute__((visibility("default")))`
- Signature: `Interface* symbol_name();` — caller takes ownership
- Default factory symbol: `"create_instance"` (configurable via `plugin_factory` config key)

**Lifetime management:** `PluginHandle<I>` (RAII) ensures the dlopen handle outlives the instance. `PluginRegistry` provides process-lifetime storage for handles when the HAL factory needs to return `unique_ptr<Interface>`.

**Security:** `ENABLE_PLUGINS` is OFF by default. Arbitrary .so loading can execute any code — only enable in trusted environments.

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
