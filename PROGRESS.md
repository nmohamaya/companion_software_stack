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

---

## Phase 8 — Deployment Tooling & RTL Safety (PRs #21, #23)

### Improvement #9 — Automated Dependency Installer (PR #21)

**Date:** 2026-02-25  
**Category:** Infrastructure / Deployment  
**Branch:** `feature/install-script`

**Files Added:**
- `deploy/install_dependencies.sh` — One-command automated installer (~605 lines)

**Files Modified:**
- `process7_system_monitor/CMakeLists.txt` — Added missing `drone_hal` link
- `common/ipc/include/ipc/shm_writer.h` — Fixed `NO_DISCARD` attribute typo
- `common/ipc/include/ipc/shm_reader.h` — Fixed `msg` variable shadowing

**What:** Created a comprehensive dependency installation script that sets up a fresh Ubuntu machine from scratch:
- System packages (build-essential, cmake, spdlog, eigen3, nlohmann-json, gtest)
- OpenCV 4.10.0 from source (core + imgproc + dnn modules)
- MAVSDK 2.12.12 from source
- PX4-Autopilot SITL
- Gazebo Harmonic

**Bug Fixes Found During Fresh-Machine Testing:**
1. `process7_system_monitor` missing `drone_hal` link (linker error)
2. ShmWriter `NO_DISCARD` → `[[nodiscard]]` attribute fix
3. ShmReader `msg` variable shadowed outer scope

**Review Comment Fixes (6 total):**
- ShellCheck compliance improvements
- Error handling for failed downloads
- Idiomatic bash patterns

---

### Improvement #10 — Development Workflow Documentation

**Date:** 2026-02-25  
**Category:** Documentation / Process  

**Files Modified:**
- `README.md` — Added Development Workflow section

**What:** Documented the project's development workflow in README.md:
- **Branch naming:** `feature/issue-N-description`, `fix/issue-N-description`
- **Commit format:** `feat(#N):`, `fix(#N):`, `docs(#N):`
- **PR checklist:** Tests pass, zero warnings, descriptive title, linked issue
- **Bug fix workflow:** File issue → branch → fix → test → PR → verify → close
- **Post-merge lessons:** Lessons learned from PRs #12–#21

---

### Improvement #11 — RTL Landing Fix (PR #23)

**Date:** 2026-02-25  
**Category:** Flight Safety / Bug Fix  
**Issue:** #22, #24  
**Branch:** `fix/rtl-landing-delay`

**Files Modified:**
- `process4_mission_planner/src/main.cpp` — RTL logic rewrite
- `tests/test_mission_fsm.cpp` — Updated RTL tests

**What:** Fixed the drone not returning to the starting point before landing. Two PRs worth of fixes:

**PR #23 — Review Comment Fixes:**

| Fix | Problem | Solution |
|-----|---------|----------|
| Home position check | `!= 0.0` fails if home is at x=0 or y=0 | `std::isfinite()` gating |
| RTL guard | `home_x/home_y` used without checking `home_recorded` | `if (home_recorded)` guard |

**PR #23 — RTL Dwell Time Fix (found during simulation testing):**

| Fix | Problem | Solution |
|-----|---------|----------|
| Premature LAND | LAND sent within 100 ms of RTL before PX4 could fly back | `rtl_min_dwell_seconds` (5 s default) |
| Loose acceptance | 3.0 m acceptance radius too wide | Tightened to 1.5 m |
| RTL timing | No minimum time in RTL state | `rtl_start_time` tracked on both mission-complete and GCS RTL paths |

**Simulation Verification:**
- Before fix: Drone landed 1.7 m from starting point
- After fix: Drone returned to home (0.0 m distance, 52 s in RTL state), then landed at starting point

**Issues Closed:** #22, #24

---

### Improvement #12 — Real Drone Deployment Roadmap (Epic #25)

**Date:** 2026-02-25  
**Category:** Project Management  

**Files Added:**
- `ROADMAP.md` — Unified roadmap combining completed phases + future deployment plan

**GitHub Issues Created:**
- **Epic #25** — [Real Drone Deployment — From Simulation to Flight](https://github.com/nmohamaya/companion_software_stack/issues/25)
- **17 sub-issues** (#26–#42) across 4 planned phases:
  - Phase 9 (First Safe Flight): #26–#31 — hardware config, battery RTL, heartbeat timeout, geofencing, pre-flight checks, systemd services
  - Phase 10 (Real Cameras & Perception): #32–#36 — V4L2, TensorRT, UDPGCSLink, video streaming, cross-compilation
  - Phase 11 (Autonomous Navigation): #37–#39 — VIO backend, stereo calibration, VIO/GPS fusion
  - Phase 12 (Production Hardening): #40–#42 — flight data recorder, fault tree, gimbal driver

---

## Phase 9 — First Safe Flight (Epic #25)

### Improvement #13 — Hardware Config & Launch Script (Issue #26, PR #43)

**Date:** 2026-02-28  
**Category:** Deployment / Hardware  
**Issue:** [#26](https://github.com/nmohamaya/companion_software_stack/issues/26)  
**Branch:** `feature/issue-26-hardware-config`

**Files Added:**
- `config/hardware.json` — Real drone configuration targeting Pixhawk FC over serial
- `deploy/launch_hardware.sh` — Hardware launch script with pre-flight checks

**What:** First deliverable of Phase 9 (First Safe Flight). Created the hardware configuration and launch infrastructure for deploying the companion stack on a real drone.

**config/hardware.json:**
- FC connection via `serial:///dev/ttyACM0:921600` (MavlinkFCLink / MAVSDK)
- Conservative first-flight waypoints: 5 m triangle at 3 m AGL, 1.5 m/s cruise
- Tighter safety thresholds: battery warn 25% / crit 15%, temp warn 75°C / crit 90°C
- URI examples for USB, Jetson UART, RPi UART, UDP
- Future backend placeholders with issue references (V4L2 #32, TensorRT #33, UDPGCSLink #34, VIO #37, gimbal #42)

**deploy/launch_hardware.sh:**
- Pre-flight checks: JSON syntax validation, serial device auto-detection, permissions check, disk/temp/memory
- `--dry-run` mode: run all checks without starting processes
- `--config` override: use any JSON config file
- `FC_DEVICE` env override: override serial device from config
- aarch64 (Jetson) `LD_LIBRARY_PATH` support
- Process health monitor: periodic alive-check of all 7 processes
- Comms startup verification: early detection of FC connection failure with diagnostics
- Clean shutdown: SIGINT → wait → SIGKILL + SHM cleanup

**Metrics:**
- All 262 tests pass, 0 compiler warnings
- `--dry-run` validates correctly on dev machine

---

## Updated Summary

| Metric | Phase 1–3 | Phase 6 | Phase 7 | Phase 8 | Phase 9 |
|---|---|---|---|---|---|
| Bug fixes | 6 | 13 | 13 | 15 | **15** |
| Unit tests | 121 (10 suites) | 196 (14 suites) | 262 (18 suites) | 262 (18 suites) | **262** (18 suites) |
| Config system | 45+ tunables | 70+ tunables | 75+ tunables | 75+ tunables | **80+** (+ hardware config) |
| Processes using real Gazebo data | 0/7 | 4/7 | 5/7 | 5/7 | **5/7** |
| Detection backend | Simulated only | Simulated only | YOLOv8-nano (80 COCO classes) | YOLOv8-nano | **YOLOv8-nano** |
| OpenCV | Not used | Not used | 4.10.0 | 4.10.0 | **4.10.0** |
| Compiler warnings | 0 | 0 | 0 | 0 | **0** |
| Deployment | Manual | Manual | Manual | Automated (install script) | **+ Hardware launch script** |
| RTL verified | — | — | — | Yes (0.0 m from home) | **Yes** |
| GitHub issues | — | — | — | Epic #25 + 17 sub-issues | **#26 complete (PR #43)** |
| Documentation | README + BUG_FIXES | + Gazebo docs | + Perception docs | + ROADMAP.md + Dev Workflow | **+ hardware.json** |

### Process Activity During Simulation (Updated)

| # | Process | Backend | Real Gazebo Data | Activity |
|---|---|---|---|---|
| 1 | Video Capture | Gazebo camera | Yes — rendered frames at 30 Hz | High |
| 2 | Perception | **YOLOv8-nano (OpenCV DNN)** | **Yes — real object detection on camera frames** | **High** |
| 3 | SLAM/VIO/Nav | Gazebo odometry + IMU | Yes — ground-truth pose | High |
| 4 | Mission Planner | Pure logic | Consumes real pose | High — orchestrates flight |
| 5 | Comms | MAVLink (MAVSDK) | Yes — real PX4 link | High — controls PX4 |
| 6 | Payload Manager | Simulated gimbal | No | Low (occasional triggers) |
| 7 | System Monitor | Linux /proc | Host metrics only | Low (1 Hz) |

---

## Phase 7 — Real Perception Pipeline (Issue #19)

### Improvement #7 — ColorContourDetector (HSV Segmentation + Connected-Component Labeling)

**Date:** 2026-02-25
**Category:** Perception / Computer Vision
**Issue:** #19
**Branch:** `feature/issue-19-real-perception`

**Files Added:**
- `process2_perception/include/perception/color_contour_detector.h` — Header-only real detector (~320 lines)
- `tests/test_color_contour_detector.cpp` — 42 unit tests across 6 suites

**Files Modified:**
- `process2_perception/include/perception/detector_interface.h` — Added factory function `create_detector(backend, cfg)`
- `process2_perception/src/main.cpp` — Wired factory into inference thread
- `config/gazebo.json` — Backend `"color_contour"`, fixed camera intrinsics (cx=320, cy=240)
- `sim/worlds/test_world.sdf` — Replaced 3 muted obstacles with 6 brightly colored ones along flight path
- `tests/CMakeLists.txt` — Added `test_color_contour_detector` target

**What:** Replaced `SimulatedDetector` (random fake bounding boxes) with `ColorContourDetector` that processes actual Gazebo RGB camera frames using a pure-C++ computer vision pipeline (no OpenCV dependency):

1. **RGB → HSV Conversion** — Per-pixel inline conversion handling all edge cases
2. **Binary Mask Generation** — For each of 6 configured color ranges (with hue wrap-around support for red)
3. **Connected-Component Labeling** — Union-Find with path compression, 4-connectivity
4. **Bounding Box Extraction** — Per-component min/max coordinates + pixel count
5. **Confidence Scoring** — Area-ratio based, sorted descending, capped at `max_detections`

**Color → Class Mapping:**

| Color | Hue Range | Saturation | Object Class |
|---|---|---|---|
| Red | 340°–20° (wrap) | ≥ 0.3 | PERSON |
| Blue | 200°–260° | ≥ 0.3 | VEHICLE_CAR |
| Yellow | 40°–70° | ≥ 0.4 | VEHICLE_TRUCK |
| Green | 80°–160° | ≥ 0.3 | DRONE |
| Orange | 15°–40° | ≥ 0.5 | ANIMAL |
| Magenta | 270°–330° | ≥ 0.3 | BUILDING |

**Gazebo Obstacles (6 total along flight path):**

| # | Color | Shape | Position (x,y) | Size | Flight Leg |
|---|---|---|---|---|---|
| 1 | Red | Box | (7, 1) | 1.5×1.5×5m | Origin → WP1 |
| 2 | Blue | Cylinder | (13, 2) | r=0.8m, h=4m | Near WP1 |
| 3 | Yellow | Box | (15, 7) | 2×1×4m | WP1 → WP2 |
| 4 | Green | Cylinder | (8, 8) | r=1.0m, h=5m | WP2 → WP3 |
| 5 | Orange | Box | (4, 4) | 1.5×1.5×3m | Return leg |
| 6 | Magenta | Cylinder | (14, 13) | r=0.6m, h=6m | Near WP2 |

**Design Decisions:**
- **Header-only** — No new .cpp files, zero CMake changes for the detector itself
- **No OpenCV** — Pure C++ avoids a heavy dependency; algorithm is straightforward for solid-color objects
- **Config-driven** — Color ranges, min_contour_area, and max_detections all configurable via JSON
- **Factory pattern** — `create_detector("color_contour", &cfg)` for clean backend switching
- **Backward-compatible** — `config/default.json` still uses `"simulated"` backend

**Test Coverage (42 new tests):**

| Suite | Tests | Covers |
|---|---|---|
| RgbToHsvTest | 7 | Pure colors, white, black, yellow, grey |
| HsvRangeTest | 3 | Normal range, hue wrap-around (red), boundaries |
| UnionFindTest | 4 | Disjoint, unite, transitive, self-unite |
| ComponentBBoxTest | 1 | Width/height/area calculations |
| ColorContourDetectorTest | 22 | Null/zero/black/grey/white, solid colors, multi-rect, area filtering, confidence sort, config overrides, 4-channel, edge cases |
| DetectorFactoryTest | 5 | Simulated, color_contour, config, empty string, unknown throws |

**Metrics:**
- Total tests: 196 → **238** (+42)
- Build targets: 51 (all zero warnings with `-Werror -Wall -Wextra`)
- Processes with real Gazebo data: 4/7 → **5/7** (Perception upgraded)

---

### Improvement #8 — OpenCV 4.10 + YOLOv8-nano DNN Detection Pipeline

**Date:** 2026-02-25
**Category:** Perception / Computer Vision / Deep Learning
**Issue:** #19
**Branch:** `feature/issue-19-real-perception`

**Files Added:**
- `process2_perception/include/perception/opencv_yolo_detector.h` — YOLOv8 detector class declaration
- `process2_perception/src/opencv_yolo_detector.cpp` — Full inference implementation (~200 lines)
- `process2_perception/src/detector_factory.cpp` — Extracted factory (avoids circular includes)
- `tests/test_opencv_yolo_detector.cpp` — 24 unit tests across 4 suites
- `models/download_yolov8n.sh` — Model download/export script
- `models/yolov8n.onnx` — YOLOv8-nano ONNX model (12.8 MB, 80 COCO classes)

**Files Modified:**
- `CMakeLists.txt` — Added `find_package(OpenCV QUIET COMPONENTS core imgproc dnn)`
- `process2_perception/CMakeLists.txt` — Added new sources, conditional OpenCV linking, `HAS_OPENCV` definition
- `process2_perception/include/perception/detector_interface.h` — Factory moved to declaration-only (non-inline)
- `config/gazebo.json` — Backend changed to `"yolov8"`, added `model_path`, `input_size`, thresholds
- `tests/CMakeLists.txt` — Added `test_opencv_yolo_detector` target with `YOLO_MODEL_PATH`
- `tests/test_color_contour_detector.cpp` — Explicit include of `color_contour_detector.h`
- `.gitignore` — Added `*.onnx`, `*.pt`, `.venv/`

**What:** Added a production-grade YOLOv8-nano object detection pipeline using OpenCV DNN, providing real 80-class COCO detection on Gazebo camera frames:

**OpenCV Upgrade (4.6.0 → 4.10.0):**
- System OpenCV 4.6.0 could not parse YOLOv8 ONNX model (`parseBias` assertion on `Add` node)
- Built OpenCV 4.10.0 from source with minimal modules (core, imgproc, dnn) → `/usr/local`
- CMake configured with `-DOpenCV_DIR=/usr/local/lib/cmake/opencv4`

**YOLOv8-nano Inference Pipeline:**
1. **Input Preprocessing** — Raw pixel buffer → `cv::Mat` (RGBA→RGB if needed) → `cv::dnn::blobFromImage` (640×640, 1/255 scale, swapRB)
2. **DNN Forward Pass** — `cv::dnn::readNetFromONNX("yolov8n.onnx")` → `net.forward()` → output shape [1, 84, 8400]
3. **Output Parsing** — Transpose to [8400, 84], extract 4 bbox coords + 80 class scores per proposal
4. **NMS** — `cv::dnn::NMSBoxes()` with configurable confidence (0.25) and NMS (0.45) thresholds
5. **Result Mapping** — Scale boxes back to original frame, map COCO class → `ObjectClass` enum

**COCO → ObjectClass Mapping (key classes):**

| COCO Class | ObjectClass |
|---|---|
| person (0) | PERSON |
| car (2), bus (5), truck (7) | VEHICLE_CAR / VEHICLE_TRUCK |
| bicycle (1), motorcycle (3) | VEHICLE_CAR |
| airplane (4) | DRONE |
| bird (14), cat (15), dog (16) | ANIMAL |
| boat (8), train (6) | VEHICLE_CAR |
| All others | UNKNOWN |

**Architecture Decisions:**
- **Factory refactored** — Moved from inline header to `detector_factory.cpp` to resolve circular include between `detector_interface.h` ↔ `opencv_yolo_detector.h`
- **`HAS_OPENCV` guard** — All OpenCV code conditionally compiled; graceful fallback to color_contour or simulated when OpenCV unavailable
- **`YOLO_MODEL_PATH`** — Compile-time absolute path definition for test targets ensures ctest finds the model regardless of working directory
- **Three backends coexist** — `"yolov8"` (OpenCV DNN), `"color_contour"` (pure C++), `"simulated"` (random) — selected via config

**Test Coverage (24 new tests):**

| Suite | Tests | Covers |
|---|---|---|
| CocoMappingTest | 7 | COCO ID → ObjectClass for person, car, truck, airplane, animal, unknown, out-of-range |
| OpenCvYoloDetectorTest | 6 | Construction, default params, config construction, detect-without-load, detect-null-data, zero-dimension |
| YoloModelTest | 6 | Model loading, inference on black/color images, empty-on-tiny, confidence threshold, nonexistent model |
| YoloFactoryTest | 5 | Factory creates yolov8 backend, config-based creation, is-loaded check, fallback backends |

**Metrics:**
- Total tests: 238 → **262** (+24)
- Build: 0 warnings with `-Werror -Wall -Wextra -Wpedantic`
- OpenCV: 4.6.0 → **4.10.0** (built from source, dnn module)
- Model: YOLOv8-nano (6.2M params, 12.8 MB ONNX, 80 COCO classes)

---

## Updated Summary

| Metric | Phase 1–3 | Phase 6 | Phase 7 |
|---|---|---|---|
| Bug fixes | 6 | 13 | 13 |
| Unit tests | 121 (10 suites) | 196 (14 suites) | **262** (18 suites) |
| Config system | 45+ tunables | 70+ tunables | 75+ tunables (+ YOLO params) |
| Processes using real Gazebo data | 0/7 | 4/7 | **5/7** (+ Perception) |
| Detection backend | Simulated only | Simulated only | **YOLOv8-nano (80 COCO classes)** |
| OpenCV | Not used | Not used | **4.10.0 (core + imgproc + dnn)** |
| Compiler warnings | 0 | 0 | **0** |

### Process Activity During Simulation (Updated)

| # | Process | Backend | Real Gazebo Data | Activity |
|---|---|---|---|---|
| 1 | Video Capture | Gazebo camera | Yes — rendered frames at 30 Hz | High |
| 2 | Perception | **YOLOv8-nano (OpenCV DNN)** | **Yes — real object detection on camera frames** | **High** |
| 3 | SLAM/VIO/Nav | Gazebo odometry + IMU | Yes — ground-truth pose | High |
| 4 | Mission Planner | Pure logic | Consumes real pose | High — orchestrates flight |
| 5 | Comms | MAVLink (MAVSDK) | Yes — real PX4 link | High — controls PX4 |
| 6 | Payload Manager | Simulated gimbal | No | Low (occasional triggers) |
| 7 | System Monitor | Linux /proc | Host metrics only | Low (1 Hz) |

---

## Zenoh IPC Migration — Phase A (Issue #46, PR #52)

### Improvement — Zenoh Phase A Foundation: CMake, ZenohMessageBus, Factory, Tests, CI

**Date:** 2026-02-28
**Category:** IPC / Architecture
**Issue:** [#46](https://github.com/nmohamaya/companion_software_stack/issues/46)
**PR:** [#52](https://github.com/nmohamaya/companion_software_stack/pull/52)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)
**ADR:** [ADR-001 — IPC Framework Selection](docs/adr/ADR-001-ipc-framework-selection.md)

**Files Added:**
- `common/ipc/include/ipc/zenoh_session.h` — Singleton Zenoh session manager (intentionally-leaked, peer mode)
- `common/ipc/include/ipc/zenoh_publisher.h` — `IPublisher<T>` backed by `zenoh::Publisher`
- `common/ipc/include/ipc/zenoh_subscriber.h` — `ISubscriber<T>` with latest-value cache (callback + mutex)
- `common/ipc/include/ipc/zenoh_message_bus.h` — Factory with 12-segment SHM→Zenoh topic mapping
- `common/ipc/include/ipc/message_bus_factory.h` — Config-driven `create_message_bus("shm"|"zenoh")` + variant helpers
- `tests/test_zenoh_ipc.cpp` — 31 tests (5 factory + 15 topic mapping + 11 Zenoh pub/sub)

**Files Modified:**
- `CMakeLists.txt` — `ENABLE_ZENOH`, `ZENOH_CONFIG_PATH`, `ALLOW_INSECURE_ZENOH` options
- `common/ipc/CMakeLists.txt` — Conditional `zenohc::lib` link, `ZENOHCXX_ZENOHC` define
- `.github/workflows/ci.yml` — Build matrix `{shm, zenoh}`, zenohc/zenoh-cpp 1.7.2 install
- `config/default.json` — Added `"ipc_backend": "shm"`
- `docs/API.md` — Updated with all Zenoh wrapper class documentation
- `BUG_FIXES.md` — Added Fix #7 (atexit panic) and Fix #8 (stack overflow)
- `DEVELOPMENT_WORKFLOW.md` — Added API.md and BUG_FIXES.md to documentation checklists

**What:** Implemented the Zenoh IPC backend behind a `HAVE_ZENOH` compile guard:

1. **CMake Integration** — `ENABLE_ZENOH` option (OFF by default) with security gate: builds require either `ZENOH_CONFIG_PATH` (TLS + auth config) or `-DALLOW_INSECURE_ZENOH=ON` (dev/test only)
2. **ZenohSession** — Process-wide singleton. Peer mode, no daemon needed. Intentionally leaked to avoid zenohc Rust 1.85 atexit() panic (Bug #7)
3. **ZenohPublisher<T>** — Serializes trivially-copyable T to `zenoh::Bytes` via `vector<uint8_t>`. Enforces `static_assert(is_trivially_copyable_v<T>)`
4. **ZenohSubscriber<T>** — Callback-based with latest-value cache. Thread-safe via mutex + atomic flag
5. **ZenohMessageBus** — Factory with `advertise<T>()`, `subscribe<T>()`, `subscribe_lazy<T>()` + 12-segment SHM→Zenoh key-expression mapping
6. **MessageBusFactory** — `create_message_bus("shm"|"zenoh")` returns `std::variant<ShmMessageBus, ZenohMessageBus>` with type-erased `bus_advertise<T>()` / `bus_subscribe<T>()` helpers
7. **CI Dual Build** — Build matrix with `{shm, zenoh}` legs; Zenoh leg installs zenohc + zenoh-cpp 1.7.2 from GitHub releases

**Topic Mapping (all 12 SHM segments):**

| SHM Segment | Zenoh Key Expression |
|---|---|
| `/drone_mission_cam` | `drone/video/frame` |
| `/drone_stereo_cam` | `drone/video/stereo_frame` |
| `/detected_objects` | `drone/perception/detections` |
| `/slam_pose` | `drone/slam/pose` |
| `/mission_status` | `drone/mission/status` |
| `/trajectory_cmd` | `drone/mission/trajectory` |
| `/payload_commands` | `drone/mission/payload_command` |
| `/fc_commands` | `drone/comms/fc_command` |
| `/fc_state` | `drone/comms/fc_state` |
| `/gcs_commands` | `drone/comms/gcs_command` |
| `/payload_status` | `drone/payload/status` |
| `/system_health` | `drone/monitor/health` |

**Bug Fixes (2):**
- **Fix #7 (Critical):** zenohc Rust 1.85 atexit() panic — fixed with intentionally-leaked singleton
- **Fix #8 (High):** ~6 MB ShmVideoFrame stack overflow in test — fixed with heap allocation

**PR Review Fixes (9):**
- R1: `subscribe_lazy()` now accepts topic parameter and returns functional subscriber (was returning nullptr)
- R2: `to_key_expr()` guards against empty string input (was calling `substr(1)` on empty → `out_of_range`)
- R3: `message_bus_factory.h` header comment updated to match actual API (was documenting wrong signature)
- R4: Added `#include <unistd.h>` for `getpid()` (was relying on transitive include)
- R5: Fixed test header comment: topic mapping tests are HAVE_ZENOH-guarded, not "always compiled"
- R6: Replaced all fixed `sleep_for()` delays with `publish_until_received()` / `poll_receive()` polling loops (5 s bounded timeout)
- R7: Verified `zenoh_publisher.h` doesn't need `<memory>` (uses `optional`, not `unique_ptr`)
- R8: Added `#include <chrono>` to `zenoh_subscriber.h` for `steady_clock`/`duration_cast`
- R9: Added `ZENOH_CONFIG_PATH` + `ALLOW_INSECURE_ZENOH` CMake security options — builds fail without secure config or explicit opt-in

**CI Fix:**
- Updated zenohc from 1.1.0 → 1.7.2 with correct GitHub release URL (`libzenohc-VERSION-x86_64-unknown-linux-gnu-debian.zip`)
- Pinned zenoh-cpp to matching 1.7.2 tag
- Build zenoh-cpp with `ZENOHCXX_ZENOHC=ON`
- Pass `-DALLOW_INSECURE_ZENOH=ON` in CI configure step

**Test Coverage:**

| Category | Tests | Guard |
|---|---|---|
| MessageBusFactory | 5 | Always compiled |
| ZenohTopicMapping | 15 | `HAVE_ZENOH` |
| Zenoh pub/sub round-trips | 11 | `HAVE_ZENOH` |
| **Total new** | **33** | |

**Metrics:**
- Tests: 262 → **295** (+33)
- Test suites: 18 → **19** (+1: `test_zenoh_ipc`)
- Bug fixes: 15 → **17** (+2)
- Build: 0 warnings (`-Werror -Wall -Wextra`)
- zenohc: **1.7.2** (installed from GitHub releases)
- zenoh-cpp: **1.7.2** (installed from source)

---

## Updated Summary (Post Zenoh Phase A)

| Metric | Phase 7 | Phase 8 | Phase 9 | Zenoh Phase A |
|---|---|---|---|---|
| Bug fixes | 13 | 15 | 15 | **17** |
| Unit tests | 262 | 262 | 262 | **295** |
| Test suites | 18 | 18 | 18 | **19** |
| Config tunables | 75+ | 75+ | 80+ | **80+** |
| Compiler warnings | 0 | 0 | 0 | **0** |
| IPC backends | SHM only | SHM only | SHM only | **SHM + Zenoh** |
| CI matrix | 1 build | 1 build | 1 build | **2 builds (shm, zenoh)** |

*Last updated after Zenoh Phase A — PR #52 (#46 foundation), 295 tests, SHM + Zenoh IPC backends.*