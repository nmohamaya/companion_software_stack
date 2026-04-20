# Test Documentation — Companion Software Stack

> Comprehensive index of all unit tests, integration tests, and end-to-end
> smoke tests.  **Keep this file up to date** when adding or modifying tests.
>
> **Framework:** Google Test 1.14.0 (`TEST()` / `TEST_F()`)  
> **Standard:** C++17  
> **Run all:** `./tests/run_tests.sh` or `ctest --test-dir build --output-on-failure -j$(nproc)`

---

## Running Tests

### Quick Start

```bash
# Run all tests
./tests/run_tests.sh

# Run a specific module
./tests/run_tests.sh watchdog
./tests/run_tests.sh perception
./tests/run_tests.sh ipc

# Fast tests only (skip fork/exec, Zenoh sessions, E2E)
./tests/run_tests.sh quick

# Zenoh end-to-end integration (launches all 7 processes)
./tests/run_tests.sh zenoh-e2e

# Gazebo SITL integration (requires PX4 + Gazebo Harmonic)
./tests/run_tests.sh gazebo-e2e

# Gazebo SITL scenario runner (all 25 scenarios with PX4 + Gazebo + Zenoh)
./tests/run_scenario_gazebo.sh --all
./tests/run_scenario_gazebo.sh --all --gui   # with 3D window
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json

# Run Zenoh E2E
./tests/run_tests.sh zenoh-e2e

# List available modules
./tests/run_tests.sh list
```

### Module Filters

| Module | Description | Approx. Tests |
|--------|-------------|---------------|
| `ipc` | Zenoh IPC primitives, message bus, wire format, serializer | ~296 |
| `watchdog` | Thread heartbeat, health publisher, restart policy, process graph, supervisor | ~86 |
| `perception` | Kalman tracker, fusion engine (UKF+camera+radar), color contour, YOLOv8 | ~189 |
| `mission` | Mission FSM, FaultManager, D* Lite, ObstacleAvoider3D, geofence, event bus | ~287 |
| `comms` | MavlinkSim and GCSLink | ~13 |
| `hal` | Simulated, Gazebo, MAVLink, and plugin HAL backends | ~137 |
| `payload` | GimbalController servo simulation | ~34 |
| `monitor` | P7 system monitor (CPU/memory/thermal, ISysInfo, process context) | ~64 |
| `util` | Config, Result, latency tracker, JSON log, correlation, replay dispatch | ~251 |
| `interfaces` | IProcessMonitor interface tests | ~7 |
| `zenoh` | All Zenoh-specific tests | ~154 |
| `network` | Network transport, wire format, liveliness | ~51 |
| `quick` | All fast unit tests (excludes slow/resource-heavy) | ~1200 |
| `zenoh-e2e` | Zenoh end-to-end integration smoke test (shell script) | N/A |
| `gazebo-e2e` | Gazebo SITL integration smoke test (shell script) | N/A |

### Options

```bash
./tests/run_tests.sh watchdog --verbose       # Show individual test output
./tests/run_tests.sh --parallel 4             # Limit parallelism
./tests/run_tests.sh ipc --repeat 5           # Stress-test (run 5x)
./tests/run_tests.sh --build                  # Rebuild before testing
./tests/run_tests.sh --asan                   # Rebuild with AddressSanitizer
./tests/run_tests.sh --tsan                   # Rebuild with ThreadSanitizer
./tests/run_tests.sh --coverage               # Generate coverage report after tests
./tests/run_tests.sh --coverage --build       # Rebuild with coverage, test + report
./tests/run_tests.sh --no-build --coverage    # Coverage report only (skip rebuild)
```

### Build + Test Shortcut

```bash
# Build and run all tests in one command
bash deploy/build.sh --test

# Build and run only watchdog tests
bash deploy/build.sh --test-filter watchdog
```

---

## Summary

| Category | Files | Tests | Description |
|----------|-------|-------|-------------|
| [IPC — Core](#ipc--core) | 2 | 24 | SPSC ring buffer, message bus abstraction |
| [IPC — Zenoh](#ipc--zenoh) | 3 | 121 | Zenoh pub/sub, services, SHM zero-copy, liveliness, network/wire format |
| [IPC — Cross-Process Correlation](#ipc--cross-process-correlation) | 1 | 33 | Correlation IDs, ScopedCorrelation, WireHeader v2, SHM + log integration |
| [HAL — Simulated](#hal--simulated) | 1 | 30 | Simulated hardware backends and HAL factory |
| [HAL — Gazebo](#hal--gazebo) | 2 | 25 | Gazebo camera and IMU backends |
| [HAL — MAVLink](#hal--mavlink) | 1 | 14 | MavlinkFCLink (MAVSDK-based flight controller) |
| [HAL — Radar](#hal--radar) | 1 | 29 | IRadar interface, SimulatedRadar, factory, config, topic |
| [P2 — Perception](#p2--perception) | 6 | 216 | Kalman filter + Hungarian solver, ByteTrack (two-stage IoU), fusion (UKF+camera+radar+dormant re-ID+covariance depth+height priors+radar-learned heights), color contour, YOLOv8, world transform |
| [P4 — Mission Planner](#p4--mission-planner) | 8 | 220 | Mission FSM, FaultManager, StaticObstacleLayer, GCSCommandHandler, FaultResponseExecutor, MissionStateTick, D* Lite planner, ObstacleAvoider3D |
| [P5 — Comms](#p5--comms) | 1 | 13 | MavlinkSim and GCSLink |
| [P6 — Payload Manager](#p6--payload-manager) | 1 | 9 | GimbalController servo simulation |
| [P7 — System Monitor](#p7--system-monitor) | 2 | 53 | CPU/memory/thermal monitoring, ISysInfo abstraction, ProcessManager supervisor |
| [Watchdog — Thread Heartbeat](#watchdog--thread-heartbeat) | 1 | 25 | ThreadHeartbeatRegistry, ScopedHeartbeat, ThreadWatchdog |
| [Watchdog — Thread Health Publisher](#watchdog--thread-health-publisher) | 1 | 15 | ShmThreadHealth struct, ThreadHealthPublisher bridge |
| [Watchdog — Restart Policy](#watchdog--restart-policy) | 1 | 17 | RestartPolicy backoff/thermal, StackStatus, ProcessConfig from_json |
| [Watchdog — Process Graph](#watchdog--process-graph) | 1 | 27 | Dual-edge dependency graph, topo sort, cascade targets, cycle detection |
| [Utility](#utility) | 5 | 147 | Config, Result<T,E>, config validator, JSON log sink, latency tracker |
| [P3 — SLAM / VIO](#p3--slam--vio) | 3 | 49 | Feature extractor, stereo matcher, IMU pre-integrator, VIO backend (covariance quality) |
| [Utility — Diagnostics](#utility--diagnostics) | 1 | 12 | FrameDiagnostics collector, ScopedDiagTimer, merge, severity |
| [P4 — Collision Recovery](#p4--collision-recovery) | 1 | 14 | Post-collision FSM state, waypoint skip, recovery logic |
| [P4 — Obstacle Prediction](#p4--obstacle-prediction) | 1 | 10 | Dynamic obstacle prediction via UKF velocity vectors |
| [P6 — Gimbal Auto-Tracker](#p6--gimbal-auto-tracker) | 1 | 25 | Gimbal auto-tracking of highest-priority tracked object |
| [P3 — Rate Clamping](#p3--rate-clamping) | 1 | 27 | Config-driven loop rate clamping for P3 threads |
| [P2 — Simulated Detector](#p2--simulated-detector) | 1 | 13 | SimulatedDetector extraction, detection generation |
| [Flight Data Recorder](#flight-data-recorder) | 1 | 19 | Ring-buffer flight recorder, IPC replay |
| [Cross-Cutting Interfaces](#cross-cutting-interfaces) | 1 | 7 | IProcessMonitor, ISysInfo |
| [Integration (shell)](#integration-tests) | 2 | 42+ | Full-stack E2E: Zenoh smoke test, Gazebo SITL integration |
| [IPC — Validation](#ipc--validation) | 1 | 56 | IPC struct validation (dimensions, NaN/Inf, oversized) |
| [Utility — Triple Buffer](#utility--triple-buffer) | 1 | 10 | Lock-free triple buffer latest-value handoff |
| [Utility — sd_notify](#utility--sd_notify) | 1 | 9 | systemd sd_notify wrapper (ready, watchdog, stopping, status) |
| [Scenario Integration](#run_scenariosh--scenario-driven-integration-runner) | 2 | 250+ | 25 scenarios via `run_scenario.sh` + `run_scenario_gazebo.sh` (20 Tier 1 + 5 Tier 2) |
| [IPC — TopicResolver](#ipc--topicresolver) | 1 | 17 | Vehicle_id namespace resolution, validation, Zenoh pub/sub round-trip |
| [IPC — Serializer](#ipc--serializer) | 1 | 21 | ISerializer<T> interface, RawSerializer round-trip, wire-format compat, null safety |
| [HAL — PluginLoader](#hal--pluginloader) | 2 | 13 | PluginHandle RAII, PluginLoader dlopen/dlsym, PluginRegistry (HAVE_PLUGINS only) |
| [HAL — Depth Anything V2](#hal--depth-anything-v2) | 2 | 16 | DA V2 OpenCV DNN backend: model load, input validation, known-scene golden test, depth range (OPENCV_FOUND only) |
| [HAL — Camera Lifetime](#test_hal_camera_lifetimecpp--7-tests) | 1 | 7 | CapturedFrame owned data lifetime safety: survives next capture, dimension match, close survival |
| [HAL — Cosys-AirSim Camera Config](#test_cosys_camera_configcpp--6-tests) | 1 | 6 | CosysCameraBackend name-resolution precedence: per-section → top-level → default, plus symmetric empty-value-not-shadowing for vehicle_name (gated on `HAVE_COSYS_AIRSIM`) |
| [Benchmark — Perception Metrics](#test_perception_metricscpp--23-tests) | 1 | 23 | TP/FP/FN, per-class AP (PASCAL VOC 11-point), MOTA/MOTP, ID switches, fragmentations, confusion matrix, IoU math, N=1000×1000 perf budget |
| **Total** | **72 C++ + 5 shell** | **1589 (no SDK) / 1628 (+SDK) + 42 + 250+** | |

---

## IPC — Core

> The legacy `test_shm_ipc.cpp` (9 tests for `ShmWriter`/`ShmReader`) was removed in Issue #126
> when the POSIX SHM backend was deleted. The SPSC ring buffer and message bus tests remain.

### test_spsc_ring.cpp — 7 tests

**What it tests:** `SPSCRing<T, N>` — a lock-free single-producer
single-consumer fixed-capacity ring buffer used for zero-allocation IPC
message queues.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `SPSCRingTest` | 7 | Push/pop round-trip, pop on empty returns `nullopt`, full ring rejects push, index wrap-around on capacity boundary, `available()` count, concurrent producer–consumer stress test (10k items), struct payload (non-trivial types) |

**Why these tests matter:** The SPSC ring is a foundational lock-free
primitive.  A single off-by-one in the head/tail indices would silently
corrupt messages or stall a pipeline.  The concurrent test runs a real
producer and consumer thread to flush out memory-ordering bugs under TSan.

**Key files under test:** `util/spsc_ring.h`

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

**Key files under test:** `ipc/zenoh_liveliness.h`, `ipc/ipc_types.h`

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

## IPC — Validation

### test_ipc_validation.cpp — 54 tests

**What it tests:** `validate()` methods on all IPC structs — boundary checks for dimensions, NaN/Inf rejection, oversized payloads, and quality field range.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `IpcValidation` | 54 | VideoFrame (valid dims, zero width/height/channels, max exceeded), StereoFrame (valid, zero/max dims), DetectedObject (valid, NaN fields, negative confidence, label overflow), DetectedObjectList (valid, count exceeded, invalid nested object), Pose (valid, NaN position/quaternion, quality range), SystemHealth (valid fields), FCState (valid, NaN battery), TrajectoryCmd (valid, NaN velocity), ThreadHealth (valid, name overflow) |

**Key files under test:** `ipc/ipc_types.h`

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

### test_hal_camera_lifetime.cpp — 7 tests

**What it tests:** CapturedFrame owned data lifetime safety (Issue #452).
Verifies that frame data survives beyond subsequent capture() calls, matches
expected dimensions, and remains valid after camera close.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CameraLifetimeTest` | 6 | Frame data survives next capture, size matches w*h*c, invalid frame empty, multiple frames independent, grayscale size, data survives close |
| `CapturedFrameDefault` | 1 | Default-constructed CapturedFrame has empty data |

**Key files under test:** `hal/icamera.h`, `hal/simulated_camera.h`

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

## HAL — Cosys-AirSim Flight Controller

### test_cosys_fc_link.cpp — 8 tests

**What it tests:** `CosysFCLink` — SimpleFlight flight controller driven through AirSim RPC (Issue #490). Compiled under `HAVE_COSYS_AIRSIM`.  Tests run WITHOUT a live AirSim RPC server: they construct the backend against an unconnected `CosysRpcClient` and verify that every command path returns false rather than crashing or hanging.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CosysFCLinkTest` | 8 | Construction with a disconnected client; `name() == "CosysFCLink"`; default `FCState{}` before `open()`; `is_connected()` false before open; `open("", 0)` returns false when the RPC server is unreachable; `send_arm/send_mode/send_takeoff/send_trajectory` all return false when disconnected; unknown flight-mode codes (4, 255) rejected without RPC; `close()` is idempotent on a never-opened link |

**Key files under test:** `hal/cosys_fc_link.h`, `hal/cosys_rpc_client.h`, `hal/hal_factory.h`

**Why:** Tier 3 (Cosys-AirSim) uses SimpleFlight instead of PX4 because PX4+Cosys HIL is broken upstream (PX4 #24033, AirSim #5018). This backend is the Tier-3 counterpart to `MavlinkFCLink`. The tests verify the failure paths that flight-critical code depends on (no crash, no hang, no arbitrary truthy result when the simulator isn't running).

---

## HAL — Cosys-AirSim Camera Config

### test_cosys_camera_config.cpp — 6 tests

**What it tests:** `CosysCameraBackend::resolve_camera_name` / `resolve_vehicle_name` — the name-resolution precedence ladder introduced to fix Issue #499, Bug #1. Whole TU is gated on `HAVE_COSYS_AIRSIM`; machines without the SDK compile it to empty.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CosysCameraConfigTest` | 6 | `<section>.camera_name` overrides `cosys_airsim.camera_name`; top-level used when per-section absent; defaults `"front_center"` / `"Drone0"` used when neither set; empty section falls through to top-level; empty per-section value treated as absent for both camera_name and vehicle_name (does not shadow top-level) |

**Key files under test:** `hal/cosys_name_resolver.h` (free functions `resolve_camera_name`, `resolve_vehicle_name`, `resolve_airsim_name`), `hal/cosys_camera.h` (thin static wrappers).

**Why:** Regression test for the silent 256×144 downgrade that made scenario 30 produce zero detections for 39 YOLO inference frames. No RPC required — the helpers are `static` by design so the resolution logic can be unit-tested without instantiating the backend (which would need a live AirSim server). See BUG_FIXES.md → Fix #499a.

---

### test_perception_metrics.cpp — 23 tests

**What it tests:** `drone::benchmark::perception_metrics` — pure-C++ metrics library that scores the perception pipeline against ground truth. Foundation layer for the Epic #523 benchmark harness; consumed by later sub-issues (latency profiler, CI gating, dashboard).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `BBox2D` | 4 | `area()` and identity IoU=1; disjoint boxes → IoU 0; zero-area box → IoU 0; half-overlap IoU = 50/150 |
| `ClassMetrics` | 2 | `precision/recall/f1` math; zero denominators don't NaN |
| `DetectionMetrics` | 9 | Empty frames; all-FP when no GT; all-FN when no predictions; perfect overlap → all TP, mAP=1; class mismatch → FP+FN with confusion-matrix cell filled; below-IoU-threshold pair is FP+FN (same pair passes at lower threshold); greedy matches highest-confidence prediction first; confusion-matrix dimensions are `[num_classes+1]²` with background slot; multi-IoU returns one `DetectionMetrics` per threshold |
| `AP` | 2 | Empty preds or empty GT → AP 0; perfect preds across multiple classes → AP 1 |
| `Tracking` | 5 | Empty frames → zero metrics; perfect track over 5 frames → MOTA=MOTP=1, no switches/fragmentations; ID swap between frames detected as id_switch=1; 3-frame sequence with a 1-frame gap detected as fragmentation=1; unmatched prediction counts as FP with total_gt=0 yielding MOTA=0 (defined, not -∞) |
| `Performance` | 1 | N=1000 preds × 1000 GT in < 100 ms (measured ≈5 ms) |

**Why these tests matter:** Every subsequent PR in the perception-v2 rewrite (Epics E2-E11) has to prove it doesn't regress detection/tracking quality. The scorer is the shared contract — if this math is wrong, every downstream comparison is wrong. Corner-case tests enforce the "no zero-denominator NaN" invariant used by CI gating; the perf test keeps the scorer fast enough to run on every scenario run without bloating wall-clock.

**Key files under test:** `tests/benchmark/perception_metrics.h`, `tests/benchmark/perception_metrics.cpp`

---

## HAL — Radar

### test_radar_hal.cpp — 30 tests

**What it tests:** `IRadar` interface, `SimulatedRadar` backend, HAL factory radar path, config-driven construction, and the `/radar_detections` IPC topic constant.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `RadarDetectionValidation` | 6 | Valid detection accepted; negative range rejected; confidence above 1.0 rejected; confidence below 0.0 rejected; zero range valid; boundary confidence (0.0 and 1.0) valid |
| `RadarDetectionListValidation` | 4 | Valid list accepted; empty list (count=0) valid; max-capacity list (count=MAX_RADAR_DETECTIONS) valid; overflow (count > MAX_RADAR_DETECTIONS) rejected |
| `RadarDetectionTriviallyCopyable` | 1 | `static_assert(std::is_trivially_copyable_v<RadarDetection>)` and `RadarDetectionList` pass |
| `SimulatedRadarTest` | 11 | `init()` returns true; `is_active()` false before init; `name()` returns `"SimulatedRadar"`; `read()` returns empty list before init; `read()` returns valid list after init; target count matches config; FoV limits clamp azimuth; range limits clamp detections; all detections have confidence in [0, 1]; noise distribution is non-trivial over many samples; consecutive reads have non-decreasing timestamps |
| `RadarFactoryTest` | 3 | `create_radar("simulated")` returns `SimulatedRadar`; default backend is `"simulated"`; unknown backend throws `std::runtime_error` |
| `SimulatedRadarConfigTest` | 1 | Custom `fov_azimuth_rad`, `fov_elevation_rad`, `max_range_m`, `num_targets`, `noise.*` from config are applied |
| `RadarTopicTest` | 1 | `ipc::topics::RADAR_DETECTIONS` constant equals `"/radar_detections"` |

**Why these tests matter:** Radar is a safety-critical sensor path — incorrect range or confidence values flowing into the obstacle avoidance stack could suppress or trigger avoidance incorrectly. The validation tests enforce the wire-format invariants that downstream processes depend on. The noise distribution test guards against bias that would systematically shift range estimates.

**Key files under test:** `hal/iradar.h`, `hal/simulated_radar.h`, `hal/hal_factory.h`, `ipc/ipc_types.h`

---

### test_gazebo_radar.cpp — 17 tests

**What it tests:** `GazeboRadarBackend` — Gazebo radar HAL that converts gpu_lidar rays + odometry into `RadarDetectionList` with noise and Doppler.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `GazeboRadarTest` | 15 | Factory creation, topic-based naming, subscription via `init()`, double-init rejection, empty read before data, message counts, `ray_to_detection()` conversion (zero velocity, forward Doppler, oblique Doppler, vertical Doppler), SNR vs range, FOV mapping (single ray, multi-ray horizontal, vertical), factory gazebo/simulated backends |
| `GazeboRadarFallbackTest` | 2 | Fallback to simulated when `HAVE_GAZEBO` is not defined; gazebo backend throws `std::runtime_error` |

**Key files under test:** `hal/gazebo_radar.h`, `hal/hal_factory.h`

---

## HAL — Depth Anything V2

### test_depth_anything_v2.cpp — 16 tests

**What it tests:** `DepthAnythingV2Estimator` — ML monocular depth estimation via OpenCV DNN. Tests run with and without the ONNX model file (model-dependent tests use `GTEST_SKIP`). Requires `OPENCV_FOUND` at build time.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `DepthAnythingV2Test` | 9 | Name, model-not-found, estimate-returns-error, path traversal (direct + middle), null frame, zero dimensions, invalid channels (0/1/2/5), config construction with assert-load |
| `DAv2ModelTest` | 7 | Model loads, valid frame produces depth map, source dimensions set correctly, **depth values positive/bounded with >1m variation**, **known-scene left/right depth differs by >2m**, RGBA frame, max_depth config affects output |

**Key assertions:**
- `DepthValuesPositiveAndBounded`: gradient input must produce `max_d - min_d > 1.0m` (catches conversion formulas that collapse to uniform output — see Fix #51)
- `KnownSceneLeftRightDepthDiffers`: half-black/half-white frame must produce >2m mean depth difference between halves (golden test for formula correctness)

**Key files under test:** `hal/depth_anything_v2.h`, `hal/depth_anything_v2.cpp`, `hal/hal_factory.h`

---

## P2 — Perception

### test_kalman_tracker.cpp — 15 tests

**What it tests:** Tracking building blocks — Kalman filter (8D state, constant
velocity model) and O(n³) Munkres Hungarian assignment solver. These are shared
infrastructure used by ByteTrackTracker.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `KalmanBoxTrackerTest` | 7 | Init from detection, predict step, update step, age increment, `is_confirmed()` after N updates, `is_stale()` after M misses, velocity initially zero |
| `HungarianSolverTest` | 8 | Empty input, single match, perfect diagonal, max-cost gating, all-too-expensive, rectangular matrices, Munkres optimality (greedy-beats cases) |

**Key files under test:** `perception/kalman_tracker.h`

---

### test_fusion_engine.cpp — 74 tests

**What it tests:** CameraOnlyFusionEngine, UKFFusionEngine (per-object UKF with radar),
IFusionEngine factory, altitude gate, ground filter, dormant re-identification (Issue #237),
radar-primary architecture (radar detection reservation, orphan output gating, radar-confirmed
depth promotion — Issue #237 Phase D), covariance-weighted depth confidence (Issue #420),
multi-class height priors (Issue #423), radar-learned object heights (Issue #422),
depth edge cases (Issue #419).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FusionEngineTest` | 4 | Empty inputs → empty output, camera-only fusion, depth estimation from bbox height, multiple tracked objects |
| `FusionFactoryTest` | 3 | Factory creates `camera_only` and `ukf` backends, unknown backend throws |
| `UKFFusionEngineTest` | 6 | Empty input, 3D position estimate, covariance convergence, reset clears state, name, radar range adopts PERSON height prior |
| `CameraOnlyFusionEngineTest` | 1 | Name returns "camera_only" |
| `DormantReIDTest` | 8 | Radar-confirmed track creates dormant entry, re-ID merges at similar world position, world-frame flag output, pool cap respected, reset clears state, no dormant without pose, camera-only excluded from pool, distant tracks get separate entries |
| `RadarFusionTest` | 16 | Radar measurement model, covariance reduction, azimuth sign convention, radar-initialized depth override, camera+radar tighter than either, gate rejects outlier, noise config, disabled-by-default, set_radar_detections+fuse, has_radar flag, ground filter (reject/pass/disabled), altitude gate (reject/accept/configurable), radar detection reservation prevents double-init, orphan output gating by radar_orphan_min_hits |
| `RadarPrimaryTest` | 11 | Radar-primary depth override, reservation prevents double-init, orphan output gating, radar update count, multi-radar association |
| `RadarOnlyTrackTest` | 8 | Radar-only track creation, output gating, camera adoption, depth confidence |
| `CovarianceConfidenceTest` | 4 | Monotonic confidence decay with range, close range high confidence, far range low confidence, kDepthMaxM penalty preserved |
| `HeightPriorsTest` | 3 | Class-specific depth differs by class, UNKNOWN uses default 3.0m, config override applied |
| `RadarLearnedHeightTest` | 4 | Radar back-calculates height, EMA converges, learned height overrides class prior, new track uses prior |
| `DepthConfidenceTest` | 4 | Covariance-based confidence output, close range near 1.0, far range near 0.0, smooth degradation |
| `DepthEdgeCaseTest` | 2 | Zero bbox_h does not crash, zero bbox_noise_px produces full confidence |

**Key files under test:** `perception/fusion_engine.h`, `perception/ifusion_engine.h`, `perception/ukf_fusion_engine.h`

---

### test_world_transform.cpp — 5 tests

**What it tests:** Camera-to-world coordinate transform using full VIO quaternion rotation
(Issue #421). Validates that pitch/roll from the IMU are properly applied when converting
camera-frame 3D positions to world-frame NEU coordinates.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `WorldTransformTest` | 5 | Identity quaternion matches z-flip, pure yaw 90° rotates forward to east, 10° pitch nose-down shifts vertical, combined yaw+pitch composition, velocity not affected by drone position offset |

**Key files under test:** `process2_perception/src/main.cpp` (camera→world transform block)

---

### test_color_contour_detector.cpp — 53 tests

**What it tests:** `ColorContourDetector` — HSV-based object detection using
synthetic test images with known colored rectangles.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `HsvRangeTest` | 6 | HSV range struct, containment checks, wrap-around hue |
| `RgbToHsvTest` | 6 | RGB→HSV conversion accuracy for pure colors and edge cases |
| `UnionFindTest` | 6 | Connected-component labeling (union-find data structure) |
| `ComponentBBoxTest` | 6 | Bounding box extraction from labeled components |
| `ColorContourDetectorTest` | 23 | Detection on synthetic images: single/multi object, config thresholds, area filtering, subsampling (stride 1 + 2), single-pass multi-color, max_fps config, `subsample=0` clamping, `kNoColor` sentinel, odd-dimension bbox clamping |
| `DetectorFactoryTest` | 6 | Factory creates `ColorContourDetector` from config |

**Key files under test:** `perception/color_contour_detector.h`, `perception/detector_interface.h`

---

### test_opencv_yolo_detector.cpp — 24 tests

**What it tests:** `OpenCvYoloDetector` (YOLOv8-nano via OpenCV DNN) and the
YOLO factory.  COCO mapping tests always compile; inference tests now enabled
with model file `models/yolov8n.onnx` (13 MB, copied from companion_software_stack).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CocoMappingTest` | 6 | COCO class ID → `ObjectClass` enum mapping (person, car, truck, etc.) |
| `YoloModelTest` | 6 | Model loading, black/colored/gradient image detection, high confidence threshold reduces detections |
| `OpenCvYoloDetectorTest` | 8 | Inference on synthetic images, NMS, confidence thresholds, config construction |
| `YoloFactoryTest` | 4 | Factory creation for `"yolov8"` backend with config |

**Key files under test:** `perception/opencv_yolo_detector.h`, `perception/detector_interface.h`

---

### test_bytetrack_tracker.cpp — 18 tests

**What it tests:** ByteTrack two-stage association tracker — IoU computation,
cost matrix, two-stage matching (high-conf then low-conf), track lifecycle,
occlusion recovery, config/factory integration.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ByteTrackIoU` | 3 | Perfect overlap (IoU=1.0), no overlap (IoU=0.0), partial overlap (known geometry) |
| `ByteTrackCostMatrix` | 2 | Single track/det (1×1 matrix), multiple tracks/dets (2×3 matrix with expected costs) |
| `ByteTrackAssociation` | 4 | High-conf matched first, low-conf recovers unmatched track, low-conf doesn't create new track, only high-conf creates new tracks |
| `ByteTrackLifecycle` | 3 | Empty detections, single detection becomes confirmed after `min_hits`, stale tracks pruned after `max_age` |
| `ByteTrackOcclusion` | 1 | Occlusion recovery (high→low→high conf preserves track ID) |
| `ByteTrackConfig` | 5 | Default params, factory with null config, `name()` returns "bytetrack", factory creates working tracker, unknown backend throws |

**Key files under test:** `perception/bytetrack_tracker.h`, `perception/itracker.h`, `perception/kalman_tracker.h`

---

## P4 — Mission Planner

### test_mission_fsm.cpp — 27 tests

**What it tests:** `MissionFSM` state machine + `StuckDetector` — the core
flight mission lifecycle plus the Issue #503 stuck detector.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `MissionFSMTest` | 20 | Start state (`IDLE`), full lifecycle (arm → preflight → takeoff → survey → navigate → loiter → RTL → land → idle), waypoint load/advance/reached, emergency transition from any state, overshoot detection (past WP, before WP, last WP, lateral offset, far from WP), next_waypoint accessor, SURVEY state transition, snap offset acceptance (Issue #394: snapped position, far-from-both, no-snap fallback, within-radius boundary), NAVIGATE ↔ NAVIGATE_UNSTUCK transitions (#503) |
| `StuckDetectorTest` | 7 | Triggers when pose stationary over full window (no avoider-activity gate — the gate was removed in #503 live-flight debugging because LiDAR loses returns on collision and a gate would suppress firing in exactly the scenario it was built to catch), triggers even when avoider was never active, resets on movement > min_movement_m, disabled never fires, window-not-full does not fire, span-gate boundary below threshold (0.75 × window) does not fire, span-gate boundary above threshold (0.85 × window) fires (#503) |

**Key files under test:** `planner/mission_fsm.h`

---

### test_fault_manager.cpp — 41 tests

**What it tests:** `FaultManager` graceful degradation engine — config-driven
fault evaluation with escalation-only policy.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FaultManagerTest` | 41 | Nominal health → `NONE`, battery low → `WARN`, battery critical → `RTL`, thermal warning → `LOITER`, thermal critical → `EMERGENCY_LAND`, critical process death, pose staleness, FC link lost, escalation-only (never downgrade), loiter → RTL auto-escalation, high-water mark tracking, config overrides, geofence polygon/altitude checks, VIO quality degradation |

**Key files under test:** `planner/fault_manager.h`

---

### test_static_obstacle_layer.cpp — 12 tests

**What it tests:** `StaticObstacleLayer` — HD-map static obstacle management
including loading, camera cross-check confirmation, collision detection, and
unconfirmed approach warnings.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `StaticObstacleLayerTest` | 12 | Load empty/single/multi, cross-check 2-hit confirmation, low-quality pose skip, distant detection skip, collision within margin, no collision when far, cooldown throttle, height check, unconfirmed approach warning, confirmed obstacle no-warn |

**Key files under test:** `planner/static_obstacle_layer.h`

---

### test_gcs_command_handler.cpp — 25 tests

**What it tests:** `GCSCommandHandler` — GCS command dispatch (RTL, LAND,
MISSION_UPLOAD) with deduplication by timestamp and correlation ID propagation.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `GCSCommandHandlerTest` | 25 | RTL dispatch + FSM transition, LAND dispatch + land_sent flag, duplicate timestamp ignored, correlation ID persists, mission upload loads waypoints, disconnected subscriber ignored, PAUSE/RESUME commands, ABORT command, waypoint upload validation, geofence upload |

**Key files under test:** `planner/gcs_command_handler.h`

---

### test_fault_response_executor.cpp — 7 tests

**What it tests:** `FaultResponseExecutor` — fault response execution with
escalation-only policy (never downgrade from a previously applied action).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FaultResponseExecutorTest` | 7 | WARN action (no FC command), LOITER stops trajectory + transitions, RTL sends FC + transitions, EMERGENCY_LAND sends LAND, escalation-only (never downgrade), non-airborne states skipped, reset clears state |

**Key files under test:** `planner/fault_response_executor.h`

---

### test_mission_state_tick.cpp — 15 tests

**What it tests:** `MissionStateTick` — per-tick FSM logic for all mission
states (PREFLIGHT, TAKEOFF, NAVIGATE, NAVIGATE_UNSTUCK, RTL, LAND) with
tracking variables.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `MissionStateTickTest` | 15 | PREFLIGHT ARM retry, armed → TAKEOFF transition, takeoff altitude threshold, SURVEY yaw sweep + grid promotion, waypoint reached + payload trigger, mission complete → RTL, disarm detection during NAVIGATE, NAVIGATE_UNSTUCK disarm-abort (#503), RTL disarm → IDLE, landed transition → IDLE + fault reset, land_sent guard, waypoint overshoot advances to next, survey target_yaw wrapping to [-π,π] |

**Key files under test:** `planner/mission_state_tick.h`

---

### test_obstacle_avoider_3d.cpp — 24 tests

**What it tests:** ObstacleAvoider3D — 3D repulsive field with velocity prediction,
factory registration (including `"potential_field_3d"` alias), name accessor, vertical gain,
path-aware mode, Euclidean-magnitude clamp and close-regime path-aware bypass (Issue #503).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ObstacleAvoider3DTest` | 7 | No objects pass-through, stale objects ignored, close object repels in XYZ, low confidence ignored, correction clamped, prediction shifts repulsion, NaN pose pass-through |
| `ObstacleAvoiderFactory` | 4 | `"3d"` registered, `"obstacle_avoider_3d"` registered, `"potential_field_3d"` registered, unknown throws |
| `ObstacleAvoider3DTest` | 2 | Name is correct, convenience constructor |
| `ObstacleAvoider3DTest` | 1 | Very close object (< 0.1m) produces maximum repulsion (dead zone fix) |
| `ObstacleAvoider3DTest` | 2 | `vertical_gain=0` eliminates Z repulsion, `vertical_gain=1` produces Z repulsion |
| `ObstacleAvoider3DTest` | 4 | Path-aware mode: strips backward repulsion, preserves lateral, preserves same-direction, disabled fallback |
| `ObstacleAvoider3DTest` | 3 | Euclidean-magnitude clamp (not per-axis), close-regime bypass pushes opposite planner, bypass hysteresis prevents flip-flop (Issue #503) |

**Key files under test:** `planner/obstacle_avoider_3d.h`, `planner/iobstacle_avoider.h`

---

### test_dstar_lite_planner.cpp — 47 tests

**What it tests:** D* Lite incremental path planner — occupancy grid basics, change tracking,
D* Lite search algorithm, incremental replanning, wall-clock timeout, Z-band constraint,
km reinit, `DStarLitePlanner` (IPathPlanner implementation) integration, grid cell hashing,
factory registration.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `OccupancyGrid3DTest` | 10 | Empty grid, world↔grid round-trip, in-bounds check, obstacle inflation, low-confidence skip, clear resets, configurable min_confidence threshold, default min_confidence backward compat, radar-confirmed promotion with configurable radar_promotion_hits, promotion suppression near static cells |
| `GridCellHashTest` | 2 | Different cells → different hashes, same cell → same hash |
| `ChangeTrackingTest` | 4 | New cell insertions recorded, expired cells recorded, drain clears buffer, static obstacle changes tracked |
| `DStarLiteSearchTest` | 6 | Trivial start=goal, straight line path, 3D obstacle detour, unreachable goal → direct fallback, blocked start BFS escape, out-of-bounds goal |
| `DStarLiteIncrementalTest` | 4 | New obstacle on path triggers replan (no occupied waypoints), obstacle removed shortens path, goal change reinitialises, drone movement updates km |
| `DStarLiteTimeoutTest` | 2 | Max search time enforced, fallback on timeout |
| `DStarLiteIntegrationTest` | 6 | Plan returns valid cmd, goal snapping works, EMA smoothing, speed ramping near target, update obstacles integration, factory registered |
| `DStarLiteNameTest` | 1 | Name is "DStarLitePlanner" |
| `GridPlannerConfigTest` | 1 | cell_ttl_s propagates through config to OccupancyGrid3D (cells expire after configured TTL) |
| `DStarLiteQueueTest` | 3 | Large grid with obstacles completes within timeout, incremental replan after obstacle insertion, planner returns valid cmd while meeting timing budget |
| `DStarLiteZBandTest` | 4 | Z-band constraint reduces search space, disabled Z-band searches full 3D, different start/goal altitudes compute correct band, km reinit prevents key churn after drone movement |
| `PathPlannerFactory` | 2 | Factory creates D* Lite, unknown backend throws |

**Key files under test:** `planner/dstar_lite_planner.h`, `planner/occupancy_grid_3d.h`, `planner/grid_planner_base.h`, `planner/planner_factory.h`

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

### test_system_monitor.cpp — 36 tests

**What it tests:** System resource monitoring via Linux `/proc` filesystem.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `SysInfo` | 11 | `CpuTimes` struct arithmetic (total/active), parsing `/proc/stat`, CPU usage percentage, zero-delta edge case, memory info, thermal zone reading, process enumeration |
| `ISysInfo` | 15 | ISysInfo interface abstraction — LinuxSysInfo (name, CPU, memory, thermal, process alive, repeated reads), MockSysInfo (name, injected values), JetsonSysInfo (name, thermal zones, millideg conversion), SysInfoFactory (platform detection, config override) |
| `ProcessMonitor` | 10 | IProcessMonitor threshold monitoring — default/custom thresholds, CPU/memory/thermal alerts, battery voltage/remaining, critical battery, overlapping thresholds |

**Key files under test:** `monitor/sys_info.h`, `util/isys_info.h`, `util/linux_sys_info.h`, `util/mock_sys_info.h`, `util/jetson_sys_info.h`, `util/sys_info_factory.h`, `monitor/iprocess_monitor.h`

---

### test_process_manager.cpp — 17 tests (+ test_crasher helper binary)

**What it tests:** `ProcessManager` — the Phase 3 process supervisor that
fork+exec's child processes, detects crashes via `waitpid(WNOHANG)`, and
restarts them with exponential backoff.  This is the most complex test file
in the project because it exercises *real OS-level process lifecycle*:
fork, exec, signals, zombie reaping, and PID management.

**test_crasher binary:**  A purpose-built ~50-line binary that behaves
differently based on its CLI argument: `exit0` (clean exit), `exit1`
(error exit), `crash` (raises SIGSEGV), `hang` (sleeps forever), or
`sleep_N` (sleep N seconds then exit).  This avoids testing the
supervisor against the real stack — every crash/exit mode is
deterministic and fast.

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `ProcessManagerTest` | `AddProcess` | Process registration, name + binary path stored correctly | Catches `safe_name_copy` truncation or buffer layout errors |
| | `FindByName` | Name-based lookup returns correct pointer; missing name → `nullptr` | Validates the linear scan doesn't go OOB or match substrings |
| | `NameTruncation` | 64-char name truncated to 31 + `\0` | Prevents buffer overflows in the 32-byte `name` field |
| | `LaunchSingleCleanExit` | fork+exec, PID > 0, state RUNNING; after exit → reaped, exit code 0 | Validates the core fork+exec+waitpid path works end-to-end |
| | `LaunchAllProcesses` | Two processes launched, distinct PIDs, both RUNNING | Catches iterator bugs when launching multiple children |
| | `LaunchUnknownProcessFails` | `launch("nonexistent")` returns false | Prevents silent failure on typo'd process names |
| | `StopGraceful` | SIGTERM → process exits → state STOPPED, pid reset to -1 | Validates graceful shutdown path; catches stale PID bugs |
| | `StopAllMultipleProcesses` | `stop_all()` stops both children; all pids -1 | Catches partial-stop bugs (e.g. break after first child) |
| | `DetectErrorExit` | Child exits with code 1 → death callback fires with code=1, state=RESTARTING | Validates `WIFEXITED` / `WEXITSTATUS` decoding in `reap_children()` |
| | `DetectCrash` | Child raises SIGSEGV → callback fires with signal=SIGSEGV, `was_signaled=true` | Validates `WIFSIGNALED` / `WTERMSIG` decoding.  Uses polling loop (100ms × 20) because SIGSEGV + core dump can take 500ms–1.5s |
| | `TickIgnoresAliveChild` | Long-running child still RUNNING after `tick()` | Ensures `waitpid(WNOHANG)` doesn't spuriously reap live children |
| | `MaxRestartsExhausted` | After `max_restarts` restart attempts → state FAILED, no more restarts | Catches the bug where `restart_count` never increments (fixed: increment before state transition) |
| | `BackoffIncreases` | After first crash → state RESTARTING; immediate `tick()` does NOT re-launch (backoff not elapsed) | Validates exponential backoff timing; catches immediate-restart storms |
| | `ExternalKillDetected` | External `kill(pid, SIGKILL)` → supervisor detects death, records `SIGKILL` | Simulates OOM-killer or manual intervention; validates signal recording |
| | `FullSupervisorCycle` | Launch 2 children → kill one → tick detects + schedules restart → backoff elapses → tick re-launches with new PID → stop all | End-to-end integration: exercises the full supervisor loop including restart with fresh PID |
| | `StateToString` | `to_string()` for all 4 `ProcessState` enum values | Guards against missing cases after adding new states |
| | `NoZombieProcesses` | Launch + reap 3 times → `pid == -1` after each reap | Validates `waitpid` is called for every child; prevents zombie accumulation |

**Bugs found during development (caught by these tests):**

1. **`restart_count` never incremented** — `MaxRestartsExhausted` failed
   because `launch_one()` set state to `RUNNING` *before* checking
   `if (state == RESTARTING)` to increment the counter.  The fix: move
   the increment before the state transition.

2. **uint64_t underflow in cooldown calculation** — `now_ns` was captured
   once at the top of `tick()`, but `started_at_ns` was set later by
   `launch_one()`.  The subtraction `now_ns - started_at_ns` wrapped to
   a huge positive value, falsely triggering "stable for 60s" and
   resetting the restart counter.  Fixed by adding a `continue` after
   the RESTARTING→launch path and capturing `now_ns` fresh per-process.

3. **DetectCrash flaky at 300ms** — SIGSEGV + core dump latency
   exceeded the original 300ms sleep.  Fixed with a polling loop
   (100ms × 20 = up to 2s), which is both more robust and exits early
   in the common case (~200ms for clean SIGSEGV, ~1.3s with core dump).

**Key files under test:** `monitor/process_manager.h`  
**Test support binary:** `tests/test_crasher.cpp` (built as `bin/test_crasher`)

---

## Watchdog — Thread Heartbeat

### test_thread_heartbeat.cpp — 25 tests

**What it tests:** Phase 1 of the Process & Thread Watchdog (Epic #88,
Issue #89).  Three layers:

1. **`ThreadHeartbeat`** struct — the per-thread data record
2. **`ThreadHeartbeatRegistry`** — singleton that manages up to
   `kMaxThreads` heartbeat slots with atomic touch timestamps
3. **`ScopedHeartbeat`** — RAII wrapper for register + auto-touch
4. **`ThreadWatchdog`** — background scanner that detects stuck threads
   (no heartbeat within `stuck_threshold`) and fires callbacks

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `ThreadHeartbeatTest` | `DefaultValues` | Struct zero-initialised: name empty, timestamp 0, not critical | Catches uninitialised memory in the heartbeat slot |
| | `CopyPreservesFields` | Copy constructor copies name, atomic timestamp, critical flag | Validates the custom copy ctor for `std::atomic<uint64_t>` (atomics are not default-copyable) |
| | `RegisterSingle` | Returns valid handle, count = 1 | Basic registration path |
| | `RegisterMultiple` | 3 registrations → 3 unique handles, count = 3 | Catches slot index collision |
| | `RegisterOverflowReturnsSentinel` | After `kMaxThreads` registrations → next returns `kInvalidHandle` | Prevents OOB write when registry is full |
| | `TouchUpdatesTimestamp` | Touch sets `last_touch_ns` > 0 | Validates the `steady_clock` timestamp path |
| | `TouchMonotonicallyIncreases` | Two touches 1ms apart → ts2 > ts1 | Catches stale/cached clock reads |
| | `TouchWithGraceBumpsTimestampForward` | `touch_with_grace(10s)` → timestamp jumps ~10s ahead | Validates the grace period mechanism (used for slow init phases like model loading) |
| | `SnapshotReturnsAll` | Snapshot contains all registered threads with correct names/flags | Validates the deep-copy snapshot path |
| | `SnapshotIsDeepCopy` | Touching after snapshot doesn't alter the returned snapshot | Catches aliased references to the live registry |
| | `TouchInvalidHandleIsSafe` | Touch with `kInvalidHandle`, `kMaxThreads`, `kMaxThreads+100` → no crash | Bounds-check validation; run under ASan to catch OOB |
| | `NameTruncation` | 50-char name → stored as 31 + `\0` | Prevents buffer overflow in the 32-byte name field |
| | `CriticalFlagPreserved` | Critical vs non-critical threads retain their flag after registration | Catches flag confusion in slot assignment |
| | `ScopedHeartbeatRegisters` | RAII wrapper registers + reports valid handle | Tests the convenience wrapper |
| | `ScopedHeartbeatCritical` | `ScopedHeartbeat("crit", true)` → critical flag set | Validates critical-flag forwarding |
| | `ScopedHeartbeatTouch` | `hb.touch()` → timestamp > 0 | Tests the RAII touch path |
| | `ScopedHeartbeatTouchWithGrace` | `hb.touch_with_grace(5s)` → timestamp jumps ~5s | Tests grace period through RAII wrapper |
| | `WatchdogHealthyThreadNotFlagged` | Actively touching thread for 500ms → callback never fires, `get_stuck_threads()` empty | Core negative test: healthy threads must not trigger alerts |
| | `WatchdogStuckThreadTriggersCallback` | Thread not touched for 500ms (threshold=200ms) → callback fires with correct name | Core positive test: stuck thread detection |
| | `WatchdogGracePeriodSuppressesCallback` | `touch_with_grace(5s)` → 500ms wait → no callback despite 200ms threshold | Validates that grace periods suppress false positives during slow operations |
| | `WatchdogCriticalFlagPropagated` | Stuck critical thread → callback receives `is_critical=true` | Ensures criticality reaches restart/alert logic |
| | `WatchdogIgnoresUnstartedThread` | Registered but never touched (ts=0) → no callback | Prevents false positives for threads that haven't started yet |
| | `WatchdogMultipleThreadsMixed` | 1 healthy + 1 stuck → only stuck fires callback | Validates per-thread independent evaluation |
| | `ConcurrentRegisterAndTouch` | 8 threads register + touch 100× concurrently → all registered, all timestamps > 0 | TSan-targeted: catches data races in the atomic slot array |
| | `ConcurrentTouchAndSnapshot` | Writer thread touches rapidly while reader takes 1000 snapshots → no torn reads | TSan-targeted: validates that snapshot copies are safe under contention |

**Key files under test:** `util/thread_heartbeat.h`, `util/thread_watchdog.h`

---

## Watchdog — Thread Health Publisher

### test_thread_health_publisher.cpp — 15 tests

**What it tests:** Phase 2 of the Process & Thread Watchdog (Epic #88,
Issue #90).  Two components:

1. **`ThreadHealth`** struct — fixed-layout IPC-transportable snapshot
   of up to 16 thread health entries per process
2. **`ThreadHealthPublisher`** — bridge that reads the heartbeat registry
   + watchdog stuck list and produces `ThreadHealth` for cross-process
   visibility

Uses a `MockPublisher` that captures the last published `ThreadHealth`
without needing a real Zenoh backend.

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `ThreadHealthStruct` | `TrivialCopyable` | `static_assert` that struct is trivially copyable | Guarantees safe IPC serialisation |
| | `ThreadHealthEntryTrivialCopyable` | `static_assert` for `ThreadHealthEntry` | Same guarantee for the per-thread sub-struct |
| | `DefaultValues` | Zero-initialised: `num_threads=0`, `timestamp_ns=0`, all entries healthy, names empty | Catches uninitialised SHM reads |
| | `MaxTrackedThreadsIs16` | `kMaxTrackedThreads == 16` | Documents the compile-time contract |
| | `ProcessNameFits31Chars` | 30-char name stored correctly in 32-byte field | Buffer size validation |
| `ThreadHealthPublisherTest` | `ZeroThreadsPublishesEmptySnapshot` | With no registered threads → publishes `num_threads=0`, process name set, timestamp > 0 | Validates the empty path doesn't crash or leave garbage |
| | `SingleThreadPopulatesCorrectly` | 1 critical thread → snapshot has correct name, critical flag, healthy=true, non-zero timestamp | End-to-end single-thread path |
| | `MultipleThreadsPopulateCorrectly` | 3 threads with mixed criticality → all 3 appear in snapshot with correct flags | Validates iteration over multiple heartbeat slots |
| | `StuckThreadMarkedUnhealthy` | Thread not touched for 120ms (threshold=50ms) → `healthy=false` in snapshot | Core integration: watchdog stuck list → publisher output |
| | `HealthyThreadMarkedHealthy` | Thread touched recently (within 200ms threshold) → `healthy=true` | Negative case: ensures healthy threads aren't falsely flagged |
| | `MaxThreadsSaturates` | 16 threads registered → `num_threads=16`, all timestamps set | Boundary test at `kMaxTrackedThreads` |
| | `TimestampIsMonotonic` | Two publishes 1ms apart → `ts2 > ts1` | Catches stale snapshot timestamps |
| | `MultiplePublishCallsWork` | 5 sequential publishes → count = 5, no crash | Validates idempotent publish path |
| | `TwoThreadIntegration` | 1 active + 1 stuck thread → snapshot shows healthy + unhealthy correctly | Full integration: heartbeat registry + watchdog + publisher in one test |
| | `ProcessNameTruncatesGracefully` | 52-char process name → truncated to 31 + `\0`, no overflow | Prevents buffer overflow in the SHM struct |

**Key files under test:** `ipc/ipc_types.h` (`ThreadHealth`, `ThreadHealthEntry`), `util/thread_health_publisher.h`

---

## Watchdog — Restart Policy

### test_restart_policy.cpp — 17 tests

**What it tests:** Phase 4 of the Process & Thread Watchdog (Epic #88,
Issue #92).  Three components:

1. **`RestartPolicy`** struct — per-process restart parameters with
   exponential backoff calculation and thermal gate logic
2. **`StackStatus`** enum — NOMINAL / DEGRADED / CRITICAL with string
   conversion
3. **`ProcessConfig`** struct — JSON-deserialisable per-process configuration
   including policy overrides, launch dependencies, and cascade targets

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `RestartPolicy` | `BackoffDoublesEachAttempt` | Backoff doubles: 500 → 1000 → 2000 → 4000ms | Validates exponential scaling |
| | `BackoffCapsAtMax` | Backoff at attempt 20 capped at `max_backoff_ms` | Catches unbounded backoff overflow |
| | `BackoffWithLargeInitial` | 10000ms initial → caps at 30000ms correctly | Edge case for large starting values |
| | `BackoffFirstAttemptEqualsInitial` | Attempt 0 returns `initial_backoff_ms` | Validates off-by-one in shift |
| | `ThermalGateBlocksAtThreshold` | `thermal_gate=3`, zone=3 → blocked | Boundary condition: "at threshold means blocked" |
| | `ThermalGateZeroAlwaysBlocks` | `thermal_gate=0`, zone=0 → blocked | Ensures gate=0 means "always block" |
| | `ThermalGateFourNeverBlocks` | `thermal_gate=4`, zone=3 → not blocked | Gate=4 means "never block" (max zone is 3) |
| | `ThermalGateAtHot` | `thermal_gate=2`, zone=3 → blocked | Higher zone always exceeds lower gate |
| | `DefaultValues` | Default-constructed policy has expected field values | Documents the compile-time defaults |
| `StackStatus` | `ToStringCoversAllStates` | 3 enum values → correct strings | Guards against missing `to_string` cases |
| | `EnumValues` | NOMINAL=0, DEGRADED=1, CRITICAL=2 | Stable enum encoding for SHM transmission |
| `ProcessConfig` | `FromJsonFullConfig` | Full JSON → all fields parsed correctly | End-to-end JSON deserialization |
| | `FromJsonMissingFieldsUseDefaults` | Empty JSON → uses `RestartPolicy` defaults | Validates fallback behaviour |
| | `FromJsonPartialConfig` | Partial JSON with only some fields → correct merge | Selective override without breaking defaults |
| | `FromJsonEmptyArrays` | Empty `launch_after` / `restart_cascade` arrays → empty vectors | Edge case: explicit empty arrays |
| | `FromJsonInvalidArrayElementsIgnored` | Non-string elements in arrays → silently skipped | Robustness against bad config data |
| | `LoadFromDefaultJsonFile` | Loads `config/default.json`, validates comms=critical, slam=critical, video=non-critical | Integration: real config matches expectations (absolute path via PROJECT_CONFIG_DIR compile definition) |

**Key files under test:** `util/restart_policy.h`

---

## Watchdog — Process Graph

### test_process_graph.cpp — 27 tests

**What it tests:** Phase 4 of the Process & Thread Watchdog (Epic #88,
Issue #92).  The `ProcessGraph` class implements a dual-edge directed
graph where:
- **`launch_after`** edges define startup ordering (topological sort)
- **`restart_cascade`** edges define failure propagation (BFS transitive closure)

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `ProcessGraph` | `EmptyGraphHasNoProcesses` | Empty graph → size 0, empty launch order | Validates zero-state |
| | `SingleProcessLaunchOrder` | Single node → appears in launch order | Minimal graph works |
| | `LinearChainLaunchOrder` | A→B→C chain → [A, B, C] order | Basic topological sort |
| | `DiamondDependencyLaunchOrder` | Diamond (A→C, B→C) → A,B before C, alphabetical tiebreak | Non-trivial DAG with multiple valid orderings |
| | `IndependentProcessesAlphabetical` | 3 independent processes → alphabetical order | Deterministic tie-breaking via `std::set` |
| | `DirectCascadeTarget` | A cascades to B → `cascade_targets("A")` = {"B"} | Basic cascade edge query |
| | `TransitiveCascadeTarget` | A→B→C cascade chain → targets(A) = {B, C} | BFS transitive closure |
| | `DeepTransitiveCascade` | 3-level chain A→B→C→D → targets(A) = {B, C, D} | Deep BFS doesn't stop early |
| | `CascadeExcludesSelf` | Source never appears in its own cascade set | Prevents infinite restart loop |
| | `NoCascadeTargets` | Process with no cascade edges → empty set | Clean "no-op" path |
| | `CascadeDoesNotFollowLaunchEdges` | launch_after edges not traversed during cascade | Edge-type separation: launch ≠ cascade |
| | `LaunchDoesNotFollowCascadeEdges` | cascade edges don't affect launch ordering | Edge-type separation: cascade ≠ launch |
| | `PerceptionCrashIsolation` | Perception crash → no cascade targets (non-critical) | ADR-004 scenario: perception isolated |
| | `CommsCrashCascade` | Comms cascades to mission_planner + payload_manager | ADR-004 scenario: comms is critical |
| | `SlamCrashCascade` | SLAM cascades to perception | ADR-004 scenario: SLAM→perception dependency |
| | `LaunchCycleDetected` | A→B→A cycle → empty launch order (logs error) | Prevents infinite loop in Kahn's algorithm |
| | `LaunchCycleValidationFails` | validate() returns false on cycles, true on DAG | Surface cycle errors to operator |
| | `DanglingReferenceFails` | Reference to non-existent process → validate() fails | Catches typos in config |
| | `DefaultEdgeTableLaunchOrder` | `populate_defaults()` → comms and mission_planner come after video_capture | ADR-004 §2.3 default wiring |
| | `DefaultEdgeTableValidates` | Default graph passes `validate()` | Ensures built-in graph is acyclic |
| | `DefaultEdgeTableHasAllProcesses` | 6 processes registered by `populate_defaults()` | Documents expected process set |
| | `CascadeTargetsSorted` | Cascade output is alphabetically sorted | Deterministic output for testing |
| | `MultipleCascadeSourcesSameTarget` | Two sources both cascade to same target → each source lists it | Fan-in cascade correctness |
| | `ProcessListSorted` | `processes()` returns sorted vector | Deterministic enumeration |
| | `LaunchDepsQuery` | `launch_deps("X")` returns correct parent set | Direct predecessor query |
| | `HasProcess` | Registered → true, missing → false | Basic membership check |
| | `SizeMatchesProcessCount` | `size()` equals number of `add_process` calls | Consistency check |

**Key files under test:** `util/process_graph.h`

---

## Utility

### test_config.cpp — 23 tests

**What it tests:** `drone::Config` JSON configuration system.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ConfigTest` | 23 | Load valid/invalid/missing files, typed access (`get<string>`, `get<int>`, `get<float>`, `get<bool>`), nested dot-path keys, default values, `has()` check, `section()` sub-tree, `raw()` JSON access, `load_config()` → `Result`, `require<T>()` → `Result`, `default.json` integration |

**Key files under test:** `util/config.h`

---

### test_result.cpp — 35 tests

**What it tests:** `Result<T,E>` monadic error-handling type.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ErrorCodeTest` | 1 | `ErrorCode` enum values, string conversion |
| `ErrorTest` | 4 | `Error` construction, equality, message access |
| `ResultTest` | 26 | Success/error construction, `ok()`/`value()`/`error()`, `value_or()`, `map()`, `and_then()`, `map_error()`, move semantics, `Result<string>`, `Result<vector>`, custom error types, `map` with void return |
| `VoidResultTest` | 4 | `Result<void, E>` specialisation — success/error, `and_then()`, `map_error()` |

**Key files under test:** `util/result.h`

---

### test_config_validator.cpp — 22 tests

**What it tests:** `ConfigSchema` startup-time config validation with
builder-pattern constraints. Includes real config validation test
(`DefaultConfigPassesAllSchemas`) using absolute path resolution.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ConfigValidatorTest` | 22 | Required field missing → error, type mismatch → error, range constraint (`.range()`), one-of constraint (`.one_of()`), custom predicate (`.satisfies()`), optional fields, required sections, **default.json validates against all schemas** (absolute path via PROJECT_CONFIG_DIR), multiple errors collected in single pass, pre-built process schemas |

**Key files under test:** `util/config_validator.h`

---

### test_json_log_sink.cpp — 36 tests

**What it tests:** `JsonLogSink` — structured JSON logging sink for
machine-parseable log output.  Each log line is a self-contained JSON
object with ISO 8601 timestamp, level, logger name, PID, thread ID,
and message.  Tests validate format correctness, special-character
escaping, and integration with `LogConfig` and `ArgParser`.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `JsonSinkTest` | 23 | Output is valid JSON line (single `{…}`), required fields present (`timestamp`, `level`, `logger`, `message`, `pid`, `tid`), message content preserved, all 6 spdlog levels mapped correctly, ISO 8601 format, thread ID is numeric, PID matches `getpid()`, JSON escaping for double quotes / backslash / newline / tab / CR / control chars, file output, multi-line produces multi-line NDJSON, empty + long message handling |
| `JsonSinkMtTest` | 1 | Multi-threaded sink variant produces output without corruption |
| `JsonEscapeTest` | 4 | Escape utility: plain string unchanged, all special chars escaped, empty string, Unicode passthrough |
| `FormatTimestampTest` | 2 | ISO 8601 timestamp has correct length and contains `T` separator |
| `LevelToStrTest` | 1 | All spdlog levels → human-readable string mapping |
| `LogConfigJsonTest` | 2 | `LogConfig::init()` with JSON mode doesn't crash; human mode still works |
| `ArgParserJsonTest` | 3 | `--json-logs` flag parsed correctly, default is `false`, works alongside other flags |

**Why these tests matter:** Log output is consumed by ELK/Grafana pipelines
in production.  A single missing escape (e.g., a newline in a message)
breaks every downstream JSON parser.  The escaping tests exercise each
control character individually to prevent silent pipeline failures.

**Key files under test:** `util/json_log_sink.h`, `util/log_config.h`, `util/arg_parser.h`

---

### test_latency_tracker.cpp — 20 tests

**What it tests:** `LatencyTracker` — a fixed-capacity ring-buffer
histogram for measuring IPC and processing latencies in real-time
without heap allocation.  Computes P50/P90/P99 percentiles, mean,
and min/max.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `LatencyTrackerTest` | 20 | Default construction (empty summary), custom capacity rounded to power-of-two, record increments count, ring wraps at capacity, empty tracker returns zeroed summary, single sample → all percentiles equal, uniform data → correct P50/P90/P99, outlier shifts P99, percentiles are monotonically non-decreasing, `reset()` clears state, reset allows reuse, nanosecond → µs / ms conversion helpers, `now_ns()` returns increasing values, `log_summary_due()` with too few samples → false, enough samples → true, custom `min_samples`, wrapped buffer reports most recent data, stress test with 100k records, integration with real `steady_clock` latency |

**Why these tests matter:** Latency measurement is used in the main loop
of every process to detect performance degradation.  A bug in the
ring-buffer indexing would produce nonsensical percentiles (e.g., P50 >
P99), misguiding performance investigations.  The power-of-two
capacity test validates the bitwise-mask wrap-around that avoids
expensive modulo operations.

**Key files under test:** `util/latency_tracker.h`

---

## IPC — Cross-Process Correlation

### test_correlation.cpp — 33 tests

**What it tests:** Cross-process correlation ID support for end-to-end
request tracing across the 7-process stack.  Tests cover the full
vertical: ID generation → thread-local context → SHM message types →
wire format → JSON log output.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `CorrelationContext` | 7 | Initial value is 0, set/get/clear, `generate()` returns non-zero, lower 32 bits monotonically increase, upper 32 bits contain PID, 1000 generated IDs are all unique |
| `ScopedCorrelation` | 3 | RAII guard sets ID on construction and restores previous on destruction, nested guards restore correctly, restores zero when outer had none |
| `CorrelationContext` (threads) | 2 | Thread isolation: each thread has independent correlation ID, multi-threaded `generate()` produces globally unique IDs |
| `IpcCorrelation` | 6 | `correlation_id` field exists and defaults to 0 in all command/status IPC types: `GCSCommand`, `FCCommand`, `TrajectoryCmd`, `PayloadCommand`, `MissionStatus` |
| `WireHeaderV2` | 5 | Header is 32 bytes, version = 3, `correlation_id` defaults to 0, correlation round-trips through set/get, full serialize → deserialize with correlation preserved |
| `WireHeaderBackcompat` | 5 | V1 (24-byte) headers still validate, V1 correlation ID = 0, V0 rejected, future version rejected, truncated V1/V2 rejected |
| `JsonCorrelation` | 4 | Correlation ID omitted from JSON log when 0, present as hex string when non-zero, `ScopedCorrelation` affects live log output, hex format is correct |

**Why these tests matter:** Correlation IDs are the primary mechanism for
tracing a single GCS command through all 7 processes.  Bugs here are
insidious: a broken ID generator produces duplicate IDs (cross-contaminated
traces), a missing field silently drops tracing for an entire message type,
and a wire format incompatibility between V1 and V2 would crash older
processes during rolling upgrades.  The backward-compatibility tests
explicitly guard against this.

**Key files under test:** `util/correlation.h`, `ipc/ipc_types.h`, `ipc/wire_format.h`, `util/json_log_sink.h`

---

## Cross-Cutting Interfaces

### test_process_interfaces.cpp — 7 tests

**What it tests:** Internal strategy interfaces used across multiple
processes — tested via their simulated backends.
Path planner and obstacle avoider tests removed in Issue #207 (covered by
`test_dstar_lite_planner.cpp` and `test_obstacle_avoider_3d.cpp`).

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ProcessMonitorTest` | 7 | `IProcessMonitor` — Linux process monitoring interface, CPU/memory query, ISysInfo integration |

**Key files under test:** `monitor/iprocess_monitor.h`, `util/isys_info.h`

---

## P3 — SLAM / VIO

### test_feature_extractor.cpp — 11 tests

**What it tests:** `IFeatureExtractor` interface and `SimulatedFeatureExtractor` implementation.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FeatureExtractorTest` | 11 | Feature count, image bounds, unique IDs, deterministic replay, different-frame variation, zero-dim error path, not-triangulated initial state, diagnostics recording, factory create/throw, frame_id propagation |

**Key files under test:** `slam/ifeature_extractor.h`

### test_stereo_matcher.cpp — 14 tests

**What it tests:** `IStereoMatcher` interface, `SimulatedStereoMatcher`, and `StereoCalibration` depth model.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `StereoMatcherTest` | 11 | Match production, match rate, depth range, empty features error, positive disparity, mean depth + stddev, diagnostics, calibration effect on depth, factory, feature ID references |
| `StereoCalibrationTest` | 3 | `depth_from_disparity()` happy path, zero-disparity → negative-depth sentinel, negative-disparity → negative-depth sentinel |

**Key files under test:** `slam/istereo_matcher.h`, `slam/vio_types.h`

### test_imu_preintegrator.cpp — 12 tests

**What it tests:** `ImuPreintegrator` — Forster et al. on-manifold pre-integration.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ImuPreintegratorTest` | 12 | No/single sample errors, stationary IMU → zero velocity, constant accel → correct velocity, constant rotation → correct angle, PSD covariance, finite bias Jacobian, reset clears state, gap detection, gyro saturation warning, integration interval, sample count tracking |

**Key files under test:** `slam/imu_preintegrator.h`, `imu_preintegrator.cpp`

### test_vio_backend.cpp — 15 tests

**What it tests:** `IVIOBackend` and `SimulatedVIOBackend` — full VIO pipeline and health state machine.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `VIOBackendTest` | 13 | Frame processing, INITIALIZING→NOMINAL health transition, valid pose, IMU consumption, zero-IMU fallback, frame_id propagation, feature/match counts, advancing trajectory, timing recorded, factory create/throw, invalid frame dimensions |
| `VIOHealthTest` | 1 | Health enum name strings |
| `VIOErrorTest` | 1 | `VIOError::to_string()` format |

**Key files under test:** `slam/ivio_backend.h`, `slam/vio_types.h`

---

## Utility — Triple Buffer

### test_triple_buffer.cpp — 10 tests

**What it tests:** `TripleBuffer<T>` lock-free latest-value handoff between producer and consumer threads.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `TripleBufferTest` | 10 | Default construction, single write/read, multiple writes (reader gets latest), no-new-data returns false, concurrent producer-consumer stress, move-only types, large payloads, write-never-blocks guarantee, atomic CAS correctness (no torn reads) |

**Key files under test:** `util/triple_buffer.h`

---

## Utility — Diagnostics

### test_diagnostic.cpp — 12 tests

**What it tests:** `FrameDiagnostics` pipeline-level error/metric collector and `ScopedDiagTimer`.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `DiagnosticTest` | 12 | Empty state, add_error/warning/fatal counts, add_metric value, add_timing entry, worst_severity ordering, merge combines entries, reset clears state, ScopedDiagTimer RAII elapsed, log_summary no-throw, severity strings |

**Key files under test:** `util/diagnostic.h`

---

## Utility — sd_notify

### test_sd_notify.cpp — 9 tests

**What it tests:** `drone::systemd::*` sd_notify wrapper functions — verifies they don't crash regardless of build mode (with or without `-DENABLE_SYSTEMD=ON`), and validates watchdog environment variable parsing.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `SdNotifyWrapper` | 9 | `notify_ready()` no-crash, `notify_stopping()` no-crash, `notify_watchdog()` no-crash, `notify_status()` no-crash, `watchdog_enabled()` returns false outside systemd, `watchdog_usec()` returns 0 outside systemd, `WATCHDOG_USEC` env var parsing, `WATCHDOG_PID` mismatch handling, status string with special characters |

**Key files under test:** `util/sd_notify.h`

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

### run_scenario.sh — Scenario-Driven Integration Runner

**What it tests:** Parameterized scenario execution with automated fault injection
and verification. Launches the full stack, merges scenario-specific config overrides,
injects timed faults via the `fault_injector` CLI tool, and verifies pass criteria
(log patterns, process liveness, SHM segment existence).

| Scenario | Tier | Gazebo | Checks | Description |
|----------|------|--------|--------|-------------|
| 01 — Nominal Mission | 1 | No | 22 | 4-waypoint rectangular flight, no faults; verifies all 7 processes alive + 5 SHM segments |
| 02 — Obstacle Avoidance | 2 | Yes | 15 | HD-map two-layer D* Lite planner navigates 7-WP obstacle field with ByteTrack tracker + proximity collision detection |
| 03 — Battery Degradation | 1 | No | 5 | 3-tier battery escalation (WARN→RTL→CRIT) |
| 04 — FC Link Loss | 1 | No | 5 | FC disconnect → LOITER → RTL contingency |
| 05 — Geofence Breach | 1 | No | 5 | Tight geofence polygon → RTL on violation (WP4 exits eastern boundary) |
| 06 — Mission Upload | 1 | No | 6 | Mid-flight 3-waypoint upload via GCS command |
| 07 — Thermal Throttle | 1 | No | 8 | Thermal zone escalation with 4 critical processes alive check |
| 08 — Full Stack Stress | 1 | No | 15 | Concurrent faults, high-rate stress; 4 procs alive + 7 SHM segments |
| 09 — Perception Tracking | 1 | No | 16 | ByteTrack backend-switching smoke test; 4 log checks + 5 forbidden + 7 procs alive |
| 10 — GCS Pause/Resume | 1 | No | — | GCS pause + resume command during NAVIGATE |
| 11 — GCS Abort | 1 | No | — | GCS abort triggers immediate LAND |
| 12 — GCS RTL | 1 | No | — | GCS RTL command mid-mission |
| 13 — GCS Land | 1 | No | — | GCS LAND command mid-mission |
| 14 — Altitude Ceiling Breach | 1 | No | — | Waypoint above geofence ceiling → RTL |
| 15 — FC Quick Recovery | 1 | No | — | FC link lost + quick reconnect → LOITER then resume |
| 16 — VIO Failure | 2 | Yes | 6 | VIO degradation fault → LOITER/RTL escalation (geofence disabled — VIO drift crosses boundary in SITL) |
| 17 — Radar Gazebo | 2 | Yes | 8 | GazeboRadarBackend subscribes to gpu_lidar scan topic, converts to RadarDetectionList, UKF fusion consumes detections |
| 18 — Perception Avoidance | 2 | Yes | 8 | Camera + radar obstacle avoidance with UKF fusion — no HD-map, D* Lite dynamic layer replans from color_contour detections and radar tracks |
| 29 — Cosys Perception (mixed) | 3 | No (UE5) | 6 | Tier 3 Cosys-AirSim + PX4 HIL. Plumbing-fix only in Phase A; populated world lands via epic #480 Phase B (issue #482). |
| 30 — Cosys Static | 3 | No (UE5) | 7 | Tier 3 Cosys-AirSim + PX4 HIL. 9 RPC-spawned static obstacles (pillars, walls, chair, couch, bush, 2 mannequins). Exercises radar→grid→D* Lite→avoider + YOLOv8 + DA V2. Run via `tests/run_scenario_cosys.sh`. |

**Run (Tier 1 — simulated, no Gazebo):**
```bash
./tests/run_scenario.sh --list                          # list all scenarios
./tests/run_scenario.sh config/scenarios/01_nominal_mission.json  # run one
./tests/run_scenario.sh --all                           # run all scenarios
./tests/run_scenario.sh --all --tier 1                  # Tier 1 only (no Gazebo)
./tests/run_scenario.sh --dry-run config/scenarios/03_battery_degradation.json
```

**Manual controls:** Each scenario JSON defines adjustable parameters (battery levels,
timeouts, thermal zones, waypoints) in a `manual_controls` section. Edit these values
before running to explore edge cases.

**Requires:** `fault_injector` built (`cmake --build build --target fault_injector`);
Tier 2 scenarios additionally require Gazebo Harmonic + PX4 SITL.

See [docs/SIMULATION_ARCHITECTURE.md](../docs/architecture/SIMULATION_ARCHITECTURE.md) for architecture
diagrams and detailed setup instructions.

---

### run_scenario_gazebo.sh — Gazebo SITL Scenario Runner

**What it tests:** All 18 scenarios (15 Tier 1 + 3 Tier 2) running on PX4 SITL + Gazebo
Harmonic with real MAVLink telemetry, Gazebo camera/IMU sensors, and Zenoh IPC.
Launches PX4 + Gazebo + the full companion stack per scenario, injects timed faults
via `fault_injector`, and verifies pass criteria against actual process logs.

**Pass criteria per scenario (250+ total checks across 25 scenarios):**
- `log_contains` — required log patterns (FSM states, fault flags)
- `log_must_not_contain` — forbidden patterns (collision, unexpected faults)
- `processes_alive` — processes that must survive to end of scenario
- `shm_segments_exist` — SHM segments that must be present at verification time (legacy; skipped when `EFFECTIVE_IPC == zenoh`, which is the sole backend)

All 18 scenarios also include an `OBSTACLE COLLISION` guard in `log_must_not_contain`
to catch unexpected collisions (Fix #40).

**Run (Tier 2 — Gazebo SITL):**
```bash
./tests/run_scenario_gazebo.sh --list                                    # list
./tests/run_scenario_gazebo.sh config/scenarios/01_nominal_mission.json   # one
./tests/run_scenario_gazebo.sh --all                                     # all 18 scenarios
./tests/run_scenario_gazebo.sh --all --gui                               # with 3D window
./tests/run_scenario_gazebo.sh --dry-run config/scenarios/02_obstacle_avoidance.json
```

**Options:**
- `--ipc <shm|zenoh>` — override IPC backend (default: from `config/gazebo_sitl.json`)
- `--gui` — launch Gazebo GUI (3D visualisation)
- `--base-config <path>` — override base config
- `--timeout <seconds>` — override scenario timeout (default: 120s minimum)
- `--verbose` — extra verbose output

**Requires:** PX4 SITL (`$PX4_DIR`, default `~/PX4-Autopilot`), Gazebo Harmonic,
`fault_injector` built. Logs output to `drone_logs/scenarios_gazebo/<scenario_name>/`.

**Status (March 2026):** 18/18 scenarios passing in Tier 1; 18/18 passing in Tier 2 Gazebo SITL. Scenario 17 (radar_gazebo) added for Issue #212. Scenario 18 (perception_avoidance) added for Issue #222. See ADR-009 for Tier 1 vs Tier 2 test strategy.

---

## Conditional Compilation Guards

Some tests require optional dependencies.  The build system uses compile-time
guards so tests always compile but skip live-session checks when the dependency
is not available.

| Guard | Tests affected | Dependency |
|-------|---------------|------------|
| `HAVE_ZENOH` | `test_zenoh_ipc`, `test_zenoh_liveliness`, `test_zenoh_network` | Zenoh C++ 1.7.2 |
| `HAVE_GAZEBO` | `test_gazebo_camera`, `test_gazebo_imu`, `test_gazebo_radar` | Gazebo Harmonic + gz-transport |
| `HAVE_MAVSDK` | `test_mavlink_fc_link` | MAVSDK 2.12.12 |
| `HAVE_OPENCV` | `test_opencv_yolo_detector` (inference tests) | OpenCV 4.10.0 + YOLOv8 ONNX model ✓ (available at `models/yolov8n.onnx`) |

---

## Adding New Tests

1. Create `tests/test_<component>.cpp`
2. Include `<gtest/gtest.h>` and the header(s) under test
3. Register in `tests/CMakeLists.txt` using `add_drone_test(<name> <source> <link_libs>)`
4. Run: `ctest --test-dir build --output-on-failure -j$(nproc)`
5. **Update this file** — add an entry in the appropriate section above

---

## Python Orchestrator Tests (pytest)

The multi-agent pipeline orchestrator (`scripts/orchestrator/`) has its own
pytest test suite, separate from the C++ GTest suite tracked by ctest.

**Run:** `PYTHONPATH=scripts python -m pytest tests/test_orchestrator/ -v`

| File | Tests | Description |
|------|-------|-------------|
| `tests/test_orchestrator/test_notifications.py` | 37 | ntfy.sh notification module — config, validation, send, auth token, graceful degradation |
| `tests/test_orchestrator/test_tmux.py` | 29 | tmux session management — launch, attach, exec_attach, list, status, TOCTOU handling |
| `tests/test_orchestrator/test_checkpoint_notifications.py` | 11 | Checkpoint ↔ notification integration — CP1–CP5 with/without notifier, sanitized CP4 |
| **Total** | **77** | |

---

## IPC — TopicResolver

### test_topic_resolver.cpp — 17 tests

| Suite | Tests | What it covers |
|-------|-------|----------------|
| `TopicResolverTest` | 13 | Empty/non-empty vehicle_id, prefix behavior, leading slash, move construction, **vehicle_id validation** (rejects slashes, spaces, dots; accepts dash/underscore) |
| `TopicResolverBusTest` | 4 | Default resolver, set_topic_resolver persistence, namespaced Zenoh pub/sub round-trip, multi-namespace isolation |

**RESOURCE_LOCK:** `zenoh_session` (bus tests open Zenoh sessions)

---

*Last updated: April 2026 — 1487 C++ unit tests across 67 files + 77 Python orchestrator tests across 3 files + 42 E2E checks (5 shell scripts) + 250+ scenario checks across 25 scenarios (20 Tier 1 + 5 Tier 2). All Tier 1 scenarios passing. Epic #419 Wave 1: covariance-weighted depth fusion (4 tests), multi-class height priors (3 tests), radar-learned object heights (4 tests), depth confidence (4 tests), depth edge cases (2 tests), world transform quaternion rotation (5 tests). Issue #490: CosysFCLink SimpleFlight-via-AirSim-RPC (+8 tests). All 1487 C++ tests passing.*
