# Test Documentation — Companion Software Stack

> Comprehensive index of all unit tests, integration tests, and end-to-end
> smoke tests.  **Keep this file up to date** when adding or modifying tests.
>
> **Framework:** Google Test 1.14.0 (`TEST()` / `TEST_F()`)  
> **Standard:** C++17  
> **Run all:** `ctest --test-dir build --output-on-failure -j$(nproc)`

---

## Summary

| Category | Files | Tests | Description |
|----------|-------|-------|-------------|
| [IPC — SHM](#ipc--shm) | 2 | 25 | POSIX shared-memory primitives and message bus |
| [IPC — Zenoh](#ipc--zenoh) | 3 | 121 | Zenoh pub/sub, services, SHM zero-copy, liveliness, network/wire format |
| [HAL — Simulated](#hal--simulated) | 1 | 30 | Simulated hardware backends and HAL factory |
| [HAL — Gazebo](#hal--gazebo) | 2 | 25 | Gazebo camera and IMU backends |
| [HAL — MAVLink](#hal--mavlink) | 1 | 14 | MavlinkFCLink (MAVSDK-based flight controller) |
| [P2 — Perception](#p2--perception) | 3 | 83 | Kalman tracker, fusion engine, color contour, YOLOv8 |
| [P4 — Mission Planner](#p4--mission-planner) | 2 | 31 | Mission FSM state machine, FaultManager degradation |
| [P5 — Comms](#p5--comms) | 1 | 13 | MavlinkSim and GCSLink |
| [P6 — Payload Manager](#p6--payload-manager) | 1 | 9 | GimbalController servo simulation |
| [P7 — System Monitor](#p7--system-monitor) | 1 | 11 | CPU/memory/thermal monitoring via `/proc` |
| [Utility](#utility) | 3 | 77 | Config system, Result<T,E>, config schema validator |
| [Cross-Cutting Interfaces](#cross-cutting-interfaces) | 1 | 21 | IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor |
| [Integration (shell)](#integration-tests) | 2 | 42+ | Full-stack E2E: Zenoh smoke test, Gazebo SITL integration |
| **Total** | **23 C++ + 2 shell** | **464 + 42** | |

---

## IPC — SHM

### test_shm_ipc.cpp — 8 tests

**What it tests:** Low-level `ShmWriter` / `ShmReader` with SeqLock-based
shared memory.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ShmIPCTest` | 8 | Write ↔ read round-trip for `ShmPose`, concurrent producer–consumer access, move semantics (no double `shm_unlink`), segment creation/cleanup |

**Key files under test:** `ipc/shm_reader.h`, `ipc/shm_writer.h`, `ipc/shm_types.h`

---

### test_message_bus.cpp — 17 tests

**What it tests:** The `ShmMessageBus` abstraction layer and
`IPublisher` / `ISubscriber` / `IServiceChannel` interfaces over SHM.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ShmMessageBusTest` | 4 | Bus creation, advertise/subscribe, pub/sub round-trip with unique segment names |
| `ShmPublisherTest` | 4 | SHM publisher topic name, readiness, publish mechanics |
| `ShmSubscriberTest` | 5 | SHM subscriber connection, receive, topic name |
| `ServiceChannelInterface` | 4 | Service request/response types, poll semantics |

**Key files under test:** `ipc/shm_message_bus.h`, `ipc/shm_publisher.h`, `ipc/shm_subscriber.h`, `ipc/iservice_channel.h`

---

## IPC — Zenoh

### test_zenoh_ipc.cpp — 70 tests

**What it tests:** The Zenoh IPC backend — message bus, pub/sub, services,
SHM zero-copy, and session management.  Tests are split into *always-compiled*
(logic/mapping) and `HAVE_ZENOH`-guarded (live Zenoh session).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ZenohSession` | 7 | Singleton lifecycle, SHM pool allocation, `is_open()` |
| `ZenohPublisher` | 8 | Topic name, readiness, SHM vs bytes publish paths |
| `ZenohSubscriber` | 6 | Connection, receive, topic name, callback delivery |
| `ZenohPubSub` | 9 | End-to-end pub/sub round-trips for all IPC message types |
| `ZenohMessageBus` | 10 | Bus factory, advertise, subscribe, lazy subscribe |
| `ZenohTopicMapping` | 6 | Key-expression generation from IPC keys |
| `ZenohMigration` | 4 | SHM ↔ Zenoh migration compatibility |
| `ZenohShmProvider` | 6 | SHM buffer allocation, zero-copy semantics |
| `ZenohShmPublish` | 5 | Zero-copy SHM publish path (>64 KB payloads) |
| `ZenohServiceChannel` | 5 | Zenoh queryable service request/response |
| `MessageBusFactory` | 4 | Config-driven bus creation (`"shm"` vs `"zenoh"`) |

**Key files under test:** `ipc/zenoh_message_bus.h`, `ipc/zenoh_publisher.h`, `ipc/zenoh_subscriber.h`, `ipc/zenoh_session.h`, `ipc/zenoh_service_client.h`, `ipc/zenoh_service_server.h`, `ipc/message_bus_factory.h`

---

### test_zenoh_liveliness.cpp — 28 tests

**What it tests:** Zenoh Phase F liveliness tokens for process health
monitoring.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `LivelinessToken` | 6 | Token declaration, key expression format, validity |
| `LivelinessMonitor` | 6 | Monitoring alive/dead processes, callback dispatch |
| `LivelinessConstants` | 4 | Prefix and wildcard string constants |
| `LivelinessExtract` | 4 | `extract_process_name()` utility parsing |
| `ProcessHealthEntry` | 4 | `ProcessHealthEntry` struct defaults and transitions |
| `ShmSystemHealth` | 4 | `ShmSystemHealth` struct layout, active process bitmask |

**Key files under test:** `ipc/zenoh_liveliness.h`, `ipc/shm_types.h`

---

### test_zenoh_network.cpp — 23 tests

**What it tests:** Zenoh Phase E network transport — binary wire format and
network configuration.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `WireFormat` | 13 | 24-byte header layout, magic bytes, serialize/deserialize round-trip, validation (truncated, bad version, too small), `key_to_wire_type()` mapping |
| `ZenohNetworkConfig` | 6 | Config generation for peer/router/client modes, JSON output |
| `ConfigAwareFactory` | 4 | Config-driven ZenohMessageBus creation with network settings |

**Key files under test:** `ipc/wire_format.h`, `ipc/zenoh_network_config.h`, `ipc/message_bus_factory.h`

---

## HAL — Simulated

### test_hal.cpp — 30 tests

**What it tests:** All 5 simulated hardware backends and the HAL factory.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `SimulatedCameraTest` | 5 | Open/close lifecycle, frame capture with valid dimensions, double-open rejection |
| `SimulatedIMUTest` | 5 | Init/read lifecycle, accelerometer + gyroscope data plausibility, `is_active()` |
| `SimulatedFCLinkTest` | 5 | Open/close, trajectory send, arm/disarm, mode switch, state receive |
| `SimulatedGCSLinkTest` | 5 | Open/close, telemetry send, command poll |
| `SimulatedGimbalTest` | 5 | Init, target set + update, image capture, recording toggle |
| `HALFactoryTest` | 5 | Factory creates correct backend from `config.hal_backend` string |

**Key files under test:** `hal/hal_factory.h`, `hal/icamera.h`, `hal/iimu_source.h`, `hal/ifc_link.h`, `hal/igcs_link.h`, `hal/igimbal.h`

---

## HAL — Gazebo

### test_gazebo_camera.cpp — 13 tests

**What it tests:** `GazeboCameraBackend` — the Gazebo Transport camera HAL.
Tests compile with `HAVE_GAZEBO`.  Live-session tests gracefully handle
missing Gazebo runtime.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `GazeboCameraTest` | 13 | Factory creation, topic-based naming, `is_open()/close` lifecycle, capture timeout without Gazebo, double-open rejection, pixel format helpers |

**Key files under test:** `hal/gazebo_camera_backend.h`, `hal/hal_factory.h`

---

### test_gazebo_imu.cpp — 12 tests

**What it tests:** `GazeboIMUBackend` — the Gazebo Transport IMU HAL.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `GazeboIMUTest` | 8 | Factory creation, topic-based naming, subscription via `init()`, double-init rejection, graceful behavior without Gazebo |
| `GazeboIMUFallbackTest` | 4 | Fallback to simulated IMU when Gazebo is unavailable |

**Key files under test:** `hal/gazebo_imu_backend.h`, `hal/hal_factory.h`

---

## HAL — MAVLink

### test_mavlink_fc_link.cpp — 14 tests

**What it tests:** `MavlinkFCLink` — MAVSDK-based flight controller backend.
Compiled with `HAVE_MAVSDK`.  Tests gracefully handle missing PX4 SITL.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `MavlinkFCLinkTest` | 14 | Factory creation, `name()`, graceful connection failure, flight mode string mapping, arm/disarm commands, trajectory send, interface compliance |

**Key files under test:** `hal/mavlink_fc_link.h`, `hal/hal_factory.h`

---

## P2 — Perception

### test_kalman_tracker.cpp — 17 tests

**What it tests:** Multi-object tracking pipeline — Kalman filter, Hungarian
assignment, and multi-object tracker lifecycle.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `KalmanBoxTrackerTest` | 7 | Init from detection, predict step, update step, age increment, `is_confirmed()` after N updates, `is_stale()` after M misses |
| `HungarianSolverTest` | 5 | Cost matrix assignment, empty input, perfect match, unequal rows/cols, edge cases |
| `MultiObjectTrackerTest` | 5 | Track creation from detections, track pruning after staleness, continuous tracking across frames |

**Key files under test:** `perception/kalman_tracker.h`

---

### test_fusion_engine.cpp — 5 tests

**What it tests:** Multi-sensor fusion engine — camera, LiDAR, radar data fusion.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FusionEngineTest` | 5 | Empty inputs → empty output, camera-only fusion, LiDAR confidence boosting, unmatched LiDAR clusters, radar velocity integration |

**Key files under test:** `perception/fusion_engine.h`

---

### test_color_contour_detector.cpp — 42 tests

**What it tests:** `ColorContourDetector` — HSV-based object detection using
synthetic test images with known colored rectangles.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `HsvRangeTest` | 6 | HSV range struct, containment checks, wrap-around hue |
| `RgbToHsvTest` | 6 | RGB→HSV conversion accuracy for pure colors and edge cases |
| `UnionFindTest` | 6 | Connected-component labeling (union-find data structure) |
| `ComponentBBoxTest` | 6 | Bounding box extraction from labeled components |
| `ColorContourDetectorTest` | 12 | Detection on synthetic images: single/multi object, config-driven thresholds, minimum area filtering |
| `DetectorFactoryTest` | 6 | Factory creates `ColorContourDetector` from config |

**Key files under test:** `perception/color_contour_detector.h`, `perception/detector_interface.h`

---

### test_opencv_yolo_detector.cpp — 24 tests

**What it tests:** `OpenCvYoloDetector` (YOLOv8-nano via OpenCV DNN) and the
YOLO factory.  COCO mapping tests always compile; inference tests require a
model file.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CocoMappingTest` | 6 | COCO class ID → `ObjectClass` enum mapping (person, car, truck, etc.) |
| `YoloModelTest` | 4 | Model loading, input tensor shape, backend selection |
| `OpenCvYoloDetectorTest` | 8 | Inference on synthetic images, NMS, confidence thresholds |
| `YoloFactoryTest` | 6 | Factory creation for `"yolov8"` backend with config |

**Key files under test:** `perception/opencv_yolo_detector.h`, `perception/detector_interface.h`

---

## P4 — Mission Planner

### test_mission_fsm.cpp — 7 tests

**What it tests:** `MissionFSM` state machine — the core flight mission
lifecycle.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `MissionFSMTest` | 7 | Start state (`IDLE`), full lifecycle (arm → preflight → takeoff → navigate → loiter → RTL → land → idle), waypoint load/advance/reached, emergency transition from any state |

**Key files under test:** `planner/mission_fsm.h`

---

### test_fault_manager.cpp — 24 tests

**What it tests:** `FaultManager` graceful degradation engine — config-driven
fault evaluation with escalation-only policy.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FaultManagerTest` | 24 | Nominal health → `NONE`, battery low → `WARN`, battery critical → `RTL`, thermal warning → `LOITER`, thermal critical → `EMERGENCY_LAND`, critical process death, pose staleness, FC link lost, escalation-only (never downgrade), loiter → RTL auto-escalation, high-water mark tracking, config overrides |

**Key files under test:** `planner/fault_manager.h`

---

## P5 — Comms

### test_comms.cpp — 13 tests

**What it tests:** `MavlinkSim` (simulated MAVLink FC link) and `GCSLink`
(ground control station link).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `MavlinkSim` | 8 | Connection lifecycle, trajectory send requires connection, heartbeat battery drain simulation, arm/disarm, mode change, ground speed from trajectory |
| `GCSLink` | 5 | Connection lifecycle, telemetry send requires connection, initial poll → no command, `GCSMessageType` enum values |

**Key files under test:** `comms/mavlink_sim.h`, `comms/gcs_link.h`

---

## P6 — Payload Manager

### test_payload_manager.cpp — 9 tests

**What it tests:** `GimbalController` — simulated 2-axis gimbal servo.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `GimbalController` | 9 | Initial state (zeroed), init success, pitch clamp [-90°, 30°], yaw clamp [-180°, 180°], smooth motion (rate-limited, not instant jump), recording start/stop toggle, image capture returns true when initialised |

**Key files under test:** `payload/gimbal_controller.h`

---

## P7 — System Monitor

### test_system_monitor.cpp — 11 tests

**What it tests:** System resource monitoring via Linux `/proc` filesystem.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `SysInfo` | 11 | `CpuTimes` struct arithmetic (total/active), parsing `/proc/stat`, CPU usage percentage, zero-delta edge case, memory info, thermal zone reading, process enumeration |

**Key files under test:** `monitor/sys_info.h`

---

## Utility

### test_config.cpp — 23 tests

**What it tests:** `drone::Config` JSON configuration system.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ConfigTest` | 23 | Load valid/invalid/missing files, typed access (`get<string>`, `get<int>`, `get<float>`, `get<bool>`), nested dot-path keys, default values, `has()` check, `section()` sub-tree, `raw()` JSON access, `load_config()` → `Result`, `require<T>()` → `Result`, `default.json` integration |

**Key files under test:** `util/config.h`

---

### test_result.cpp — 32 tests

**What it tests:** `Result<T,E>` monadic error-handling type.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ErrorCodeTest` | 4 | `ErrorCode` enum values, string conversion |
| `ErrorTest` | 4 | `Error` construction, equality, message access |
| `ResultTest` | 16 | Success/error construction, `ok()`/`value()`/`error()`, `value_or()`, `map()`, `and_then()`, `map_error()`, move semantics, `Result<string>`, `Result<vector>` |
| `VoidResultTest` | 8 | `Result<void, E>` specialisation — success/error, `and_then()`, `map_error()` |

**Key files under test:** `util/result.h`

---

### test_config_validator.cpp — 22 tests

**What it tests:** `ConfigSchema` startup-time config validation with
builder-pattern constraints.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ConfigValidatorTest` | 22 | Required field missing → error, type mismatch → error, range constraint (`.range()`), one-of constraint (`.one_of()`), custom predicate (`.satisfies()`), optional fields, required sections, valid config passes, multiple errors collected in single pass, pre-built process schemas |

**Key files under test:** `util/config_validator.h`

---

## Cross-Cutting Interfaces

### test_process_interfaces.cpp — 21 tests

**What it tests:** Internal strategy interfaces used across multiple
processes — tested via their simulated backends.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `VisualFrontendTest` | 5 | `IVisualFrontend` — simulated visual odometry init, pose estimation, feature count |
| `PathPlannerTest` | 5 | `IPathPlanner` — potential field planner, obstacle avoidance path generation |
| `ObstacleAvoiderTest` | 6 | `IObstacleAvoider` — collision check, safe velocity computation |
| `ProcessMonitorTest` | 5 | `IProcessMonitor` — Linux process monitoring interface, CPU/memory query |

**Key files under test:** `slam/ivisual_frontend.h`, `planner/ipath_planner.h`, `planner/iobstacle_avoider.h`, `monitor/iprocess_monitor.h`

---

## Integration Tests

### test_zenoh_e2e.sh — 42 checks

**What it tests:** Full-stack end-to-end smoke test with all 7 processes
running the Zenoh IPC backend (simulated hardware).

| Phase | Description | Checks |
|-------|-------------|--------|
| 1 | Launch all 7 processes (staggered 0.5s) | — |
| 2 | Configurable verification window (default 15s) | — |
| 3 | Process liveness — all 7 still running | 7 |
| 4 | Zenoh backend selection (log markers) | 7 |
| 5 | Liveliness tokens declared + monitor active | 8 |
| 6 | Data flow verification (log markers per process) | 7 |
| 7 | Death detection — kill payload_manager → P7 detects, others survive | 7 |
| 8 | Graceful shutdown via SIGINT (exit code 0 or 130) | 6 |

**Run:** `bash tests/test_zenoh_e2e.sh`

---

### test_gazebo_integration.sh

**What it tests:** Full-stack smoke test with PX4 SITL + Gazebo Harmonic.
Launches all 7 processes and verifies liveness and IPC data flow over SHM,
gz-transport, and MAVLink.

**Requires:** PX4 SITL + Gazebo Harmonic running.  
**Run:** `bash tests/test_gazebo_integration.sh`

---

## Conditional Compilation Guards

Some tests require optional dependencies.  The build system uses compile-time
guards so tests always compile but skip live-session checks when the dependency
is not available.

| Guard | Tests affected | Dependency |
|-------|---------------|------------|
| `HAVE_ZENOH` | `test_zenoh_ipc`, `test_zenoh_liveliness`, `test_zenoh_network` | Zenoh C++ 1.7.2 |
| `HAVE_GAZEBO` | `test_gazebo_camera`, `test_gazebo_imu` | Gazebo Harmonic + gz-transport |
| `HAVE_MAVSDK` | `test_mavlink_fc_link` | MAVSDK 2.12.12 |
| `HAVE_OPENCV` | `test_opencv_yolo_detector` (inference tests) | OpenCV 4.10.0 + YOLOv8 ONNX model |

---

## Adding New Tests

1. Create `tests/test_<component>.cpp`
2. Include `<gtest/gtest.h>` and the header(s) under test
3. Register in `tests/CMakeLists.txt` using `add_drone_test(<name> <source> <link_libs>)`
4. Run: `ctest --test-dir build --output-on-failure -j$(nproc)`
5. **Update this file** — add an entry in the appropriate section above

---

*Last updated: March 2026 — 464 unit tests (26 suites across 23 files) + 42 E2E checks (2 shell scripts).*
