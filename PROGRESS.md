# Progress Log

Tracking all improvements, features, and infrastructure additions to the Drone Companion Software Stack.

---

## Phase 1 — Foundation (Bug Fixes, Testing, Config System)

### Improvement #1 — Testing Infrastructure (Google Test)

**Date:** 2026-02-23  
**Category:** Testing  
**Files Added:**
- `tests/CMakeLists.txt`
- `tests/test_shm_ipc.cpp`
- `tests/test_spsc_ring.cpp`
- `tests/test_kalman_tracker.cpp`
- `tests/test_mission_fsm.cpp`
- `tests/test_fusion_engine.cpp`
- `tests/test_config.cpp`

**What:** Introduced Google Test (GTest 1.14.0) framework with an initial 58 unit tests across 6 suites covering the core subsystems:

| Suite | Tests | Coverage |
|---|---|---|
| `test_shm_ipc` | 8 | ShmWriter/ShmReader lifecycle, concurrent read/write, move semantics, ShmPose round-trip |
| `test_spsc_ring` | 7 | Push/pop, full ring rejection, wrap-around, struct payloads, concurrent producer-consumer |
| `test_kalman_tracker` | 15 | KalmanBoxTracker init/predict/update, HungarianSolver assignment/edge cases, MultiObjectTracker creation/pruning |
| `test_mission_fsm` | 7 | FSM transitions, waypoint load/advance/reached, emergency from any state |
| `test_fusion_engine` | 5 | Camera-only fusion, LiDAR confidence boost, unmatched LiDAR clusters, radar velocity, empty inputs |
| `test_config` | 14 | Load/parse, nested dot-paths, type coercion, sections, missing keys, bool/float values, default.json integration |

**Build Integration:** Added `enable_testing()` and `add_subdirectory(tests)` to root CMakeLists.txt. Created `add_drone_test()` CMake helper function. Handled the Anaconda/system GTest conflict by forcing `GTest_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTest` and adding a `CMAKE_CROSSCOMPILING_EMULATOR` workaround for test discovery.

---

### Improvement #2 — JSON Configuration System

**Date:** 2026-02-23  
**Category:** Architecture  
**Files Added:**
- `common/util/include/util/config.h`
- `config/default.json`

**Files Modified:**
- `common/util/CMakeLists.txt` — linked `nlohmann_json::nlohmann_json`
- `CMakeLists.txt` — added `find_package(nlohmann_json REQUIRED)`
- `common/util/include/util/arg_parser.h` — default config path → `config/default.json`

**What:** Created a `drone::Config` class backed by nlohmann/json that provides:
- `load(path)` — loads JSON with error handling
- `get<T>(key, default)` — dot-separated key navigation (e.g., `"video_capture.mission_cam.width"`)
- `has(key)` — existence check
- `section(key)` — raw JSON sub-tree access
- `raw()` — full JSON access

All 7 processes plus every tunable parameter are represented in [config/default.json](config/default.json) with a clear hierarchical structure.

---

### Improvement #3 — Bug Fixes (6 Total)

**Date:** 2026-02-23  
**Category:** Correctness  
**Documentation:** See [BUG_FIXES.md](BUG_FIXES.md) for full details.

| # | Severity | Component | Issue |
|---|---|---|---|
| 1 | Critical | P3 SLAM | Use-after-free race condition (raw `new/delete` + `atomic<Pose*>`) |
| 2 | High | ShmWriter | Broken move constructor → double `shm_unlink` |
| 3 | High | ShmReader | Broken move constructor → double `munmap`/`close` |
| 4 | Medium | HungarianSolver | Uninitialized `total_cost` field |
| 5 | High | MultiObjectTracker | Never created tracks from empty state |
| 6 | Medium | LiDARCluster | Uninitialized `Eigen::Vector3f` members |

---

## Phase 2 — Near-Term Improvements

### Improvement #4 — Config Wired into All 7 Processes

**Date:** 2026-02-23  
**Category:** Architecture  
**Files Modified:**
- `process1_video_capture/src/main.cpp`
- `process2_perception/src/main.cpp`
- `process3_slam_vio_nav/src/main.cpp`
- `process4_mission_planner/src/main.cpp`
- `process5_comms/src/main.cpp`
- `process6_payload_manager/src/main.cpp`
- `process7_system_monitor/src/main.cpp`

**What:** Every process now loads `drone::Config` from `args.config_path` at startup and replaces all hardcoded magic numbers with `cfg.get<>()` calls. All values fall back to sane defaults so the stack runs identically without a config file.

| Process | Parameters Made Configurable |
|---|---|
| P1 video_capture | Mission camera W/H/FPS, stereo camera W/H/FPS |
| P2 perception | LiDAR sim rate (ms), radar sim rate (ms), camera intrinsics (fx, fy, cx, cy) |
| P3 slam_vio_nav | IMU rate (Hz), VIO publish rate (Hz) |
| P4 mission_planner | Waypoints from JSON array, acceptance radius, cruise speed, obstacle avoidance influence radius & repulsive gain, update rate |
| P5 comms | MAVLink serial port & baud rate, GCS UDP port |
| P6 payload_manager | Gimbal update rate (→ control loop dt) |
| P7 system_monitor | CPU/memory/temp/battery/disk warning & critical thresholds, disk check interval, update rate |

**Impact:** Operators can now tune the entire stack by editing a single JSON file instead of recompiling. Simulation and field deployment can share the same binary with different configs.

---

### Improvement #5 — Expanded Test Coverage (58 → 91 tests)

**Date:** 2026-02-23  
**Category:** Testing  
**Files Added:**
- `tests/test_comms.cpp` — 13 tests
- `tests/test_payload_manager.cpp` — 9 tests
- `tests/test_system_monitor.cpp` — 11 tests

**Files Modified:**
- `tests/CMakeLists.txt` — added 3 new test targets, expanded include dirs for P5/P6/P7

**New Test Coverage:**

| Suite | Tests | Coverage |
|---|---|---|
| `test_comms` | 13 | `MavlinkSim` connect/disconnect, send_trajectory, arm/disarm, mode change, heartbeat battery drain, ground speed; `GCSLink` connect, telemetry, poll_command, message types, defaults |
| `test_payload_manager` | 9 | `GimbalController` init, set_target clamping (pitch/yaw limits), smooth motion (slew rate), convergence to target, capture image, start/stop recording, zero-dt no-movement |
| `test_system_monitor` | 11 | `CpuTimes` defaults & arithmetic, `/proc/stat` reader, CPU usage computation, `/proc/meminfo` reader, temperature reader, disk usage, process watchdog (`is_process_alive`) |

**Total:** 91 tests across 9 suites, all passing, zero warnings.

---

### Improvement #6 — CI Pipeline (GitHub Actions)

**Date:** 2026-02-23  
**Category:** Infrastructure  
**Files Added:**
- `.github/workflows/ci.yml`

**What:** Created a GitHub Actions CI workflow that runs on every push to `main`/`develop` and on pull requests to `main`. The pipeline:

1. Checks out the repository
2. Installs system dependencies (`build-essential`, `cmake`, `libspdlog-dev`, `libeigen3-dev`, `nlohmann-json3-dev`, `libgtest-dev`)
3. Configures CMake in Release mode with `-Werror -Wall -Wextra`
4. Builds all 7 binaries + 9 test targets
5. Runs all 91 tests via `ctest --output-on-failure`

**Impact:** Every PR is automatically validated for compilation (with warnings-as-errors) and full test pass before merge.

---

## Phase 3 — Hardware Abstraction Layer (HAL)

### Improvement #7 — HAL Interfaces, Simulated Backends & Factory (Issue #2)

**Date:** 2026-02-24  
**Category:** Architecture / Abstraction  
**Issue:** [#2 — Hardware Abstraction Layer](https://github.com/nmohamaya/companion_software_stack/issues/2)

**Files Added:**
- `common/hal/CMakeLists.txt` — Header-only HAL library
- `common/hal/include/hal/icamera.h` — `ICamera` interface
- `common/hal/include/hal/ifc_link.h` — `IFCLink` interface
- `common/hal/include/hal/igcs_link.h` — `IGCSLink` interface
- `common/hal/include/hal/igimbal.h` — `IGimbal` interface
- `common/hal/include/hal/iimu_source.h` — `IIMUSource` interface
- `common/hal/include/hal/simulated_camera.h` — `SimulatedCamera` backend
- `common/hal/include/hal/simulated_fc_link.h` — `SimulatedFCLink` backend
- `common/hal/include/hal/simulated_gcs_link.h` — `SimulatedGCSLink` backend
- `common/hal/include/hal/simulated_gimbal.h` — `SimulatedGimbal` backend
- `common/hal/include/hal/simulated_imu.h` — `SimulatedIMU` backend
- `common/hal/include/hal/hal_factory.h` — Factory functions with config-driven backend selection
- `tests/test_hal.cpp` — 30 HAL unit tests

**Files Modified:**
- `CMakeLists.txt` — Added `common/hal` subdirectory
- `process1_video_capture/CMakeLists.txt` — Linked `drone_hal`
- `process1_video_capture/src/main.cpp` — Uses `ICamera` via factory (mission + stereo)
- `process3_slam_vio_nav/CMakeLists.txt` — Linked `drone_hal`
- `process3_slam_vio_nav/src/main.cpp` — Uses `IIMUSource` via factory
- `process5_comms/CMakeLists.txt` — Linked `drone_hal`
- `process5_comms/src/main.cpp` — Uses `IFCLink` + `IGCSLink` via factory
- `process6_payload_manager/CMakeLists.txt` — Linked `drone_hal`
- `process6_payload_manager/src/main.cpp` — Uses `IGimbal` via factory
- `config/default.json` — Added `"backend": "simulated"` to all HAL-managed sections
- `tests/CMakeLists.txt` — Added `test_hal` target, linked `drone_hal` to all tests

**What:** Introduced a complete Hardware Abstraction Layer with 5 interfaces (`ICamera`, `IFCLink`, `IGCSLink`, `IGimbal`, `IIMUSource`), 5 simulated backends, and a config-driven factory. Processes P1, P3, P5, P6 now use HAL instead of hardcoded concrete classes. Backend selection is controlled by the `"backend"` key in each config section (currently only `"simulated"` available; future backends like `v4l2`, `mavlink_v2`, `siyi`, `bmi088` can be added without modifying process code).

**Impact:**
- Hardware and simulation are now fully decoupled — switching backends requires only a config change
- 30 new unit tests covering factory creation, interface contracts, and simulated behaviour
- Total test count: 91 → 121 (10 suites)
- Zero compile warnings, all 121 tests passing
- Foundation for adding real hardware backends without touching process code

---

## Phase 4 — Gazebo SITL Integration (Issues #7–#11, PR #12–#16)

### Improvement #8 — Environment Setup for Gazebo + PX4 SITL (Issue #7, PR #12)

**Date:** 2026-02-24  
**Category:** Infrastructure / Simulation  

**Files Added:**
- `docs/gazebo_setup.md` — Detailed installation guide for Gazebo Harmonic, PX4-Autopilot, and MAVSDK
- `.github/workflows/ci.yml` — Updated to conditionally build with Gazebo/MAVSDK when available
- `sim/` directory structure — Models, worlds, GUI configs

**What:** Established the foundational environment for closed-loop simulation:
- Gazebo Harmonic 8.10.0 installation and verification
- PX4-Autopilot SITL build (`make px4_sitl_default`)
- MAVSDK 2.12.12 C++ library from source
- CMake compile guards (`HAVE_MAVSDK`, `HAVE_GAZEBO`) for optional dependencies

---

### Improvement #9 — MavlinkFCLink — MAVLink HAL Backend (Issue #8, PR #13)

**Date:** 2026-02-24  
**Category:** HAL / Flight Control  

**Files Added:**
- `common/hal/include/hal/mavlink_fc_link.h` — Full MAVSDK-based `IFCLink` implementation
- `tests/test_mavlink_fc_link.cpp` — 10 unit tests

**What:** Created `MavlinkFCLink`, the real MAVLink backend for the flight controller HAL interface:
- Connects to PX4 via MAVSDK over `udp://:14540`
- Supports arm/disarm, takeoff, land, RTL mode commands
- Offboard velocity control (NED body frame)
- Telemetry subscriptions: position, velocity, battery, flight mode, armed state
- Configurable RTL return altitude (set to flight altitude to avoid PX4's default 30 m climb)
- 10 new unit tests (connection, timeout, double-open, arm, telemetry)

---

### Improvement #10 — GazeboCameraBackend (Issue #9, PR #14)

**Date:** 2026-02-24  
**Category:** HAL / Sensors  

**Files Added:**
- `common/hal/include/hal/gazebo_camera.h` — gz-transport camera subscriber
- `tests/test_gazebo_camera.cpp` — 5 unit tests

**What:** Created `GazeboCameraBackend` implementing `ICamera` via Gazebo gz-transport:
- Subscribes to Gazebo image topics (`/camera`, `/stereo_left`)
- Receives live rendered frames from Gazebo (640×480 RGB/GRAY8)
- Double-buffered thread-safe frame delivery
- Factory integration: `"backend": "gazebo"` in config

---

### Improvement #11 — GazeboIMUBackend (Issue #10, PR #15)

**Date:** 2026-02-24  
**Category:** HAL / Sensors  

**Files Added:**
- `common/hal/include/hal/gazebo_imu.h` — gz-transport IMU subscriber
- `tests/test_gazebo_imu.cpp` — 5 unit tests

**What:** Created `GazeboIMUBackend` implementing `IIMUSource` via Gazebo gz-transport:
- Subscribes to `/imu` topic for accelerometer and gyroscope data
- Thread-safe sample delivery with mutex protection
- Configurable topic name via config

---

### Improvement #12 — Closed-Loop Integration Test (Issue #11, PR #16)

**Date:** 2026-02-24  
**Category:** Integration / Deployment  

**Files Added:**
- `sim/worlds/test_world.sdf` — Gazebo world with ground plane, landing pad, obstacles, sun
- `sim/models/x500_companion/model.sdf` — Custom x500 drone with 3 companion sensors
- `sim/models/x500_companion/model.config` — Gazebo model metadata
- `deploy/launch_gazebo.sh` — One-command launch: PX4 + Gazebo + 7 companion processes
- `deploy/build.sh` — Build script with Release/Debug mode
- `config/gazebo.json` — Full simulation config with Gazebo/MAVLink backends
- `tests/test_process_interfaces.cpp` — 25 IPC interface tests

**What:** Full closed-loop integration:
- Launch script handles SHM cleanup, PX4 boot, MAVLink heartbeat wait, GUI launch, process ordering
- Custom drone model extends x500 with mission camera, stereo camera, and companion IMU
- World includes static obstacles for avoidance testing
- 25 new IPC interface tests for all shared memory message types
- Total tests: 121 → 196 (all passing)

---

## Phase 5 — End-to-End Flight Fixes (Issue #17, PR #18)

### Improvement #13 — End-to-End Autonomous Flight Bug Fixes

**Date:** 2026-02-24  
**Category:** Bug Fixes / Flight  
**Issue:** [#17 — End-to-end flight testing](https://github.com/nmohamaya/companion_software_stack/issues/17)

**What:** Deployed the full stack with PX4 SITL and identified/fixed multiple issues preventing autonomous flight:

| # | Issue | Root Cause | Fix |
|---|---|---|---|
| 1 | MAVSDK connection failed | `ComponentType::Autopilot` rejected by PX4 | Changed to `ComponentType::GroundStation` |
| 2 | ARM command ignored | Sent before PX4 health checks pass | Added pre-arm health polling loop |
| 3 | Takeoff command rejected | Sent while still in HOLD mode after arm | Added post-arm mode stabilisation delay |
| 4 | Offboard mode rejected | Sent without prior setpoint | Pre-send zero velocity setpoint before mode switch |
| 5 | Velocity commands ignored | Body-frame NED vs local-frame mismatch | Switched to `set_velocity_ned()` local frame |
| 6 | Process ordering crash | comms started before PX4 ready | Added MAVLink port wait + process startup ordering |
| 7 | Launch script env vars | PX4 couldn't find Gazebo plugins | Fixed `GZ_SIM_RESOURCE_PATH` and working directory |

**Impact:** Complete autonomous flight: ARM → Takeoff → Navigate 3 waypoints → RTL → Land → Disarm.

---

## Phase 6 — Simulation Visualization & Flight Tuning

### Improvement #14 — Gazebo GUI with Chase-Cam Follow

**Date:** 2026-02-24  
**Category:** Visualization  

**Files Added:**
- `sim/gui.config` — Gazebo GUI layout with CameraTracking plugin, entity tree, component inspector

**Files Modified:**
- `deploy/launch_gazebo.sh` — `--gui` flag launches Gazebo GUI client with auto chase-cam follow
- `sim/worlds/test_world.sdf` — Added CameraTracking plugin to world plugins

**What:** Added 3D visualization support:
- PX4 runs Gazebo server-only (`-s` flag); GUI client launched separately via `gz sim -g`
- Chase-cam automatically follows `x500_companion_0` with 6 m behind / 3 m above offset
- Clean environment (`env -i`) avoids Conda/Snap LD_LIBRARY_PATH conflicts
- Camera follow command sent 8 s after GUI launch to allow initialization

---

### Improvement #15 — GazeboVisualFrontend — Real Odometry from Gazebo

**Date:** 2026-02-24  
**Category:** SLAM / Sensors  

**Files Modified:**
- `process3_slam_vio_nav/include/slam/ivisual_frontend.h` — Added `GazeboVisualFrontend` class
- `process3_slam_vio_nav/src/main.cpp` — Factory selects `"gazebo"` or `"simulated"` backend
- `sim/models/x500_companion/model.sdf` — Added `OdometryPublisher` plugin

**What:** The `SimulatedVisualFrontend` generated fake circular orbit data, making the drone's position meaningless during simulation. Created `GazeboVisualFrontend`:
- Subscribes to `/model/x500_companion_0/odometry` via gz-transport
- Returns ground-truth pose from Gazebo physics engine
- Coordinate frame mapping: Gazebo ENU (X=East, Y=North) → Internal (X=North, Y=East) via X↔Y swap
- SLAM now publishes real drone position to the mission planner

---

### Improvement #16 — Flight Plan Tuning for Visible Flight

**Date:** 2026-02-24  
**Category:** Configuration / Flight  

**Files Modified:**
- `config/gazebo.json` — Updated waypoints: 15 m apart at 5 m AGL, 5 m/s cruise, 2.0 m acceptance
- `process4_mission_planner/include/planner/ipath_planner.h` — Speed ramp: linear blend from cruise to 1.0 m/s min in last 2 m
- `process4_mission_planner/src/main.cpp` — Mission completion: RTL (returns to takeoff point)
- `common/hal/include/hal/mavlink_fc_link.h` — Set `RTL_RETURN_ALT` to 5 m via MAVSDK
- `tests/test_process_interfaces.cpp` — Updated `WithObstacles` test for new planner behavior

**What:** Multiple iterations to achieve a clearly visible, well-behaved demo flight:

1. **Waypoints expanded:** 4 m apart → 15 m apart so flight path is clearly visible in Gazebo GUI
2. **Speed reduced:** 8 m/s → 5 m/s for better visual tracking
3. **Altitude increased:** 3 m → 5 m for more visible flight
4. **Speed ramp added:** Planner ramped speed from cruise to 1.0 m/s floor in last 2 m to prevent crawling near waypoints (previously `speed = min(target.speed, dist)` made the drone nearly stop)
5. **LAND → RTL:** Changed mission completion from LAND (descends in place) to RTL so drone returns to takeoff point
6. **RTL altitude configured:** Set `action->set_return_to_launch_altitude(5.0f)` so PX4 doesn't climb to default 30 m before returning home — RTL now stays at flight altitude

**Result:** ARM (1 s) → Takeoff (13 s) → WP1 (6 s) → WP2 (6 s) → WP3 (9 s) → RTL+Land (14 s) = ~50 s total. Drone flies a clearly visible triangle and lands back at the takeoff point.

---

### Improvement #17 — Documentation

**Date:** 2026-02-24  
**Category:** Documentation  

**Files Added:**
- `docs/Gazebo_sim_run.md` — 14-section comprehensive simulation guide (setup, launch, flight plan, GUI, obstacles, logs, troubleshooting)
- `docs/sim_run_commands.md` — Quick-reference command sheet for clean build + run
- `deploy/clean_build_and_run.sh` — One-command automated script: cleanup → build → test → launch

**What:** Created complete documentation for running the Gazebo SITL simulation:
- Step-by-step installation guide (Gazebo Harmonic, PX4, MAVSDK)
- Custom model and world symlink instructions
- Flight plan customization with waypoint field reference and examples
- Gazebo GUI controls and camera follow instructions
- Troubleshooting table for common issues
- Automated `clean_build_and_run.sh` script

---

## Summary

| Metric | Phase 1–3 | After Phase 6 |
|---|---|---|
| Bug fixes | 6 | 13 (+ 7 end-to-end flight fixes) |
| Unit tests | 121 (10 suites) | 196 (14 suites) |
| Config system | JSON with 45+ tunables | JSON with 70+ tunables (+ Gazebo config) |
| Processes using config | 7/7 | 7/7 |
| Processes using HAL | 4/4 | 4/4 (+ 3 Gazebo backends) |
| HAL interfaces | 5 | 5 (8 backend implementations) |
| HAL backends — Simulated | 5 | 5 |
| HAL backends — Gazebo/MAVLink | 0 | 3 (GazeboCam, GazeboIMU, MavlinkFC) |
| CI pipeline | GitHub Actions | GitHub Actions (build + 196 tests) |
| Simulation | None | Full closed-loop Gazebo SITL with PX4 |
| Visualization | None | Gazebo GUI with chase-cam follow |
| Documentation | README + BUG_FIXES | + Gazebo guide, command ref, setup docs |
| Autonomous flight | Not possible | ARM → Takeoff → Navigate → RTL → Land |
| Compiler warnings | 0 | 0 |

### Process Activity During Simulation

| # | Process | Backend | Real Gazebo Data | Activity |
|---|---|---|---|---|
| 1 | Video Capture | Gazebo camera | Yes — rendered frames at 30 Hz | High |
| 2 | Perception | Simulated detector | No — random fake detections | Medium (real algorithms, synthetic data) |
| 3 | SLAM/VIO/Nav | Gazebo odometry + IMU | Yes — ground-truth pose | High |
| 4 | Mission Planner | Pure logic | Consumes real pose | High — orchestrates flight |
| 5 | Comms | MAVLink (MAVSDK) | Yes — real PX4 link | High — controls PX4 |
| 6 | Payload Manager | Simulated gimbal | No | Low (occasional triggers) |
| 7 | System Monitor | Linux /proc | Host metrics only | Low (1 Hz) |
