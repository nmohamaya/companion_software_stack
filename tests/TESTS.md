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

# Gazebo SITL scenario runner (all 8 scenarios with PX4 + Gazebo + Zenoh)
./tests/run_scenario_gazebo.sh --all --ipc zenoh
./tests/run_scenario_gazebo.sh --all --ipc zenoh --gui   # with 3D window
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json --ipc zenoh

# Build with Zenoh and run E2E
./tests/run_tests.sh zenoh-e2e --zenoh

# List available modules
./tests/run_tests.sh list
```

### Module Filters

| Module | Description | Approx. Tests |
|--------|-------------|---------------|
| `ipc` | SHM + Zenoh IPC primitives, message bus, wire format | ~150 |
| `watchdog` | Thread heartbeat, health publisher, restart policy, process graph, supervisor | ~85 |
| `perception` | Kalman tracker, fusion engine (UKF+camera), color contour, YOLOv8 | ~113 |
| `mission` | Mission FSM, FaultManager degradation | ~31 |
| `comms` | MavlinkSim and GCSLink | ~13 |
| `hal` | Simulated, Gazebo, and MAVLink HAL backends | ~44 |
| `payload` | GimbalController servo simulation | ~9 |
| `monitor` | P7 system monitor (CPU/memory/thermal) | ~28 |
| `util` | Config, Result, latency tracker, JSON log, correlation | ~136 |
| `interfaces` | IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor | ~21 |
| `zenoh` | All Zenoh-specific tests | ~121 |
| `network` | Network transport, wire format, liveliness | ~50 |
| `quick` | All fast unit tests (excludes slow/resource-heavy) | ~600 |
| `zenoh-e2e` | Zenoh end-to-end integration smoke test (shell script) | N/A |
| `gazebo-e2e` | Gazebo SITL integration smoke test (shell script) | N/A |

### Options

```bash
./tests/run_tests.sh watchdog --verbose       # Show individual test output
./tests/run_tests.sh --parallel 4             # Limit parallelism
./tests/run_tests.sh ipc --repeat 5           # Stress-test (run 5x)
./tests/run_tests.sh --build                  # Rebuild before testing
./tests/run_tests.sh --zenoh                   # Build with Zenoh backend + run all
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
| [IPC — SHM](#ipc--shm) | 3 | 32 | POSIX shared-memory primitives, message bus, SPSC ring buffer |
| [IPC — Zenoh](#ipc--zenoh) | 3 | 121 | Zenoh pub/sub, services, SHM zero-copy, liveliness, network/wire format |
| [IPC — Cross-Process Correlation](#ipc--cross-process-correlation) | 1 | 33 | Correlation IDs, ScopedCorrelation, WireHeader v2, SHM + log integration |
| [HAL — Simulated](#hal--simulated) | 1 | 30 | Simulated hardware backends and HAL factory |
| [HAL — Gazebo](#hal--gazebo) | 2 | 25 | Gazebo camera and IMU backends |
| [HAL — MAVLink](#hal--mavlink) | 1 | 14 | MavlinkFCLink (MAVSDK-based flight controller) |
| [P2 — Perception](#p2--perception) | 4 | 113 | Kalman tracker (Munkres), fusion (UKF+camera), color contour, YOLOv8 |
| [P4 — Mission Planner](#p4--mission-planner) | 2 | 31 | Mission FSM state machine, FaultManager degradation |
| [P5 — Comms](#p5--comms) | 1 | 13 | MavlinkSim and GCSLink |
| [P6 — Payload Manager](#p6--payload-manager) | 1 | 9 | GimbalController servo simulation |
| [P7 — System Monitor](#p7--system-monitor) | 2 | 28 | CPU/memory/thermal monitoring, ProcessManager supervisor |
| [Watchdog — Thread Heartbeat](#watchdog--thread-heartbeat) | 1 | 25 | ThreadHeartbeatRegistry, ScopedHeartbeat, ThreadWatchdog |
| [Watchdog — Thread Health Publisher](#watchdog--thread-health-publisher) | 1 | 15 | ShmThreadHealth struct, ThreadHealthPublisher bridge |
| [Watchdog — Restart Policy](#watchdog--restart-policy) | 1 | 17 | RestartPolicy backoff/thermal, StackStatus, ProcessConfig from_json |
| [Watchdog — Process Graph](#watchdog--process-graph) | 1 | 27 | Dual-edge dependency graph, topo sort, cascade targets, cycle detection |
| [Utility](#utility) | 5 | 136 | Config, Result<T,E>, config validator, JSON log sink, latency tracker |
| [P3 — SLAM / VIO](#p3--slam--vio) | 3 | 41 | Feature extractor, stereo matcher, IMU pre-integrator, VIO backend |
| [Utility — Diagnostics](#utility--diagnostics) | 1 | 12 | FrameDiagnostics collector, ScopedDiagTimer, merge, severity |
| [Cross-Cutting Interfaces](#cross-cutting-interfaces) | 1 | 21 | IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor |
| [Integration (shell)](#integration-tests) | 2 | 42+ | Full-stack E2E: Zenoh smoke test, Gazebo SITL integration |
| [Scenario Integration](#run_scenariosh--scenario-driven-integration-runner) | 2 | 80 | 8 scenarios via `run_scenario.sh` + `run_scenario_gazebo.sh` (Tier 1 + Tier 2) |
| **Total** | **37 C++ + 4 shell** | **746 + 42 + 80** | |

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

### test_kalman_tracker.cpp — 22 tests

**What it tests:** Multi-object tracking pipeline — Kalman filter, O(n³) Munkres
Hungarian assignment, SortTracker lifecycle, ITracker factory.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `KalmanBoxTrackerTest` | 7 | Init from detection, predict step, update step, age increment, `is_confirmed()` after N updates, `is_stale()` after M misses |
| `HungarianSolverTest` | 8 | Empty input, single match, perfect diagonal, max-cost gating, all-too-expensive, rectangular matrices, Munkres optimality (greedy-beats cases) |
| `MultiObjectTrackerTest` | 3 | Track creation from detections, track pruning after staleness, continuous tracking across frames |
| `SortTrackerTest` | 2 | `name()` returns "sort", `reset()` clears tracks and resets ID counter |
| `TrackerFactoryTest` | 2 | Factory creates `SortTracker`, unknown backend throws |

**Key files under test:** `perception/kalman_tracker.h`, `perception/itracker.h`

---

### test_fusion_engine.cpp — 14 tests

**What it tests:** CameraOnlyFusionEngine, UKFFusionEngine (per-object UKF),
IFusionEngine factory, thermal measurement update.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `FusionEngineTest` | 4 | Empty inputs → empty output, camera-only fusion, depth estimation from bbox height, multiple tracked objects |
| `FusionFactoryTest` | 3 | Factory creates `camera_only` and `ukf` backends, unknown backend throws |
| `UKFFusionEngineTest` | 6 | Empty input, 3D position estimate, covariance convergence, thermal flag, reset clears state, name |
| `CameraOnlyFusionEngineTest` | 1 | Name returns "camera_only" |

**Key files under test:** `perception/fusion_engine.h`, `perception/ifusion_engine.h`, `perception/ukf_fusion_engine.h`

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

1. **`ShmThreadHealth`** struct — fixed-layout SHM-transportable snapshot
   of up to 16 thread health entries per process
2. **`ThreadHealthPublisher`** — bridge that reads the heartbeat registry
   + watchdog stuck list and produces `ShmThreadHealth` for cross-process
   visibility

Uses a `MockPublisher` that captures the last published `ShmThreadHealth`
without needing a real SHM or Zenoh backend.

| Suite | Tests | What is validated | Why it catches bugs |
|-------|-------|-------------------|--------------------|
| `ShmThreadHealthStruct` | `TrivialCopyable` | `static_assert` that struct is trivially copyable | Guarantees safe `memcpy` across SHM boundaries |
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

**Key files under test:** `ipc/shm_types.h` (`ShmThreadHealth`, `ThreadHealthEntry`), `util/thread_health_publisher.h`

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
| | `LoadFromDefaultJsonFile` | Loads `config/default.json`, validates comms=critical, slam=critical, video=non-critical | Integration: real config matches expectations (skips if file not found) |

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
builder-pattern constraints.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `ConfigValidatorTest` | 22 | Required field missing → error, type mismatch → error, range constraint (`.range()`), one-of constraint (`.one_of()`), custom predicate (`.satisfies()`), optional fields, required sections, valid config passes, multiple errors collected in single pass, pre-built process schemas |

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
| `ShmCorrelation` | 6 | `correlation_id` field exists and defaults to 0 in all command/status SHM types: `ShmGCSCommand`, `ShmFCCommand`, `ShmTrajectoryCmd`, `ShmPayloadCommand`, `ShmMissionStatus` |
| `WireHeaderV2` | 5 | Header is 32 bytes, version = 2, `correlation_id` defaults to 0, correlation round-trips through set/get, full serialize → deserialize with correlation preserved |
| `WireHeaderBackcompat` | 5 | V1 (24-byte) headers still validate, V1 correlation ID = 0, V0 rejected, future version rejected, truncated V1/V2 rejected |
| `JsonCorrelation` | 4 | Correlation ID omitted from JSON log when 0, present as hex string when non-zero, `ScopedCorrelation` affects live log output, hex format is correct |

**Why these tests matter:** Correlation IDs are the primary mechanism for
tracing a single GCS command through all 7 processes.  Bugs here are
insidious: a broken ID generator produces duplicate IDs (cross-contaminated
traces), a missing field silently drops tracing for an entire message type,
and a wire format incompatibility between V1 and V2 would crash older
processes during rolling upgrades.  The backward-compatibility tests
explicitly guard against this.

**Key files under test:** `util/correlation.h`, `ipc/shm_types.h`, `ipc/wire_format.h`, `util/json_log_sink.h`

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

## Utility — Diagnostics

### test_diagnostic.cpp — 12 tests

**What it tests:** `FrameDiagnostics` pipeline-level error/metric collector and `ScopedDiagTimer`.

| Suite | Tests | What is validated |
|-------|-------|-------------------|
| `DiagnosticTest` | 12 | Empty state, add_error/warning/fatal counts, add_metric value, add_timing entry, worst_severity ordering, merge combines entries, reset clears state, ScopedDiagTimer RAII elapsed, log_summary no-throw, severity strings |

**Key files under test:** `util/diagnostic.h`

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
| 02 — Obstacle Avoidance | 2 | Yes | 14 | HD-map two-layer A* planner navigates 7-WP obstacle field with proximity collision detection |
| 03 — Battery Degradation | 1 | No | 5 | 3-tier battery escalation (WARN→RTL→CRIT) |
| 04 — FC Link Loss | 1 | No | 5 | FC disconnect → LOITER → RTL contingency |
| 05 — Geofence Breach | 1 | No | 5 | Tight geofence polygon → RTL on violation (WP4 exits eastern boundary) |
| 06 — Mission Upload | 1 | No | 6 | Mid-flight 3-waypoint upload via GCS command |
| 07 — Thermal Throttle | 1 | No | 8 | Thermal zone escalation with 4 critical processes alive check |
| 08 — Full Stack Stress | 1 | No | 15 | Concurrent faults, high-rate stress; 4 procs alive + 7 SHM segments |

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

See [docs/SIMULATION_ARCHITECTURE.md](../docs/SIMULATION_ARCHITECTURE.md) for architecture
diagrams and detailed setup instructions.

---

### run_scenario_gazebo.sh — Gazebo SITL Scenario Runner

**What it tests:** All 8 scenarios running on PX4 SITL + Gazebo Harmonic with real
MAVLink telemetry, Gazebo camera/IMU sensors, and configurable IPC backend (SHM or Zenoh).
Launches PX4 + Gazebo + the full companion stack per scenario, injects timed faults
via `fault_injector`, and verifies pass criteria against actual process logs.

**Pass criteria per scenario (80 total checks across 8 scenarios):**
- `log_contains` — required log patterns (FSM states, fault flags)
- `log_must_not_contain` — forbidden patterns (collision, unexpected faults)
- `processes_alive` — processes that must survive to end of scenario
- `shm_segments_exist` — SHM segments that must be present at verification time

All 8 scenarios also include an `OBSTACLE COLLISION` guard in `log_must_not_contain`
to catch unexpected collisions (Fix #40).

**Run (Tier 2 — Gazebo SITL):**
```bash
./tests/run_scenario_gazebo.sh --list                                    # list
./tests/run_scenario_gazebo.sh config/scenarios/01_nominal_mission.json   # one
./tests/run_scenario_gazebo.sh --all --ipc zenoh                         # all 8 with Zenoh
./tests/run_scenario_gazebo.sh --all --ipc zenoh --gui                   # with 3D window
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

**Status (March 2026):** 8/8 scenarios passing, 80 checks green (Gazebo SITL + Zenoh).

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

*Last updated: March 2026 — 746 unit tests (SHM) / 746 (SHM+Zenoh) across 49+ suites in 37 files + 42 E2E checks (2 shell scripts) + 80 scenario checks across 8 scenarios (run_scenario.sh + run_scenario_gazebo.sh). All 8 Gazebo SITL + Zenoh scenarios green. PR #135 (rev 2): bbox clamping, uint8_t overflow guard, area_ratio float cast, max_fps sleep-at-end, odd-dimension regression test.*
