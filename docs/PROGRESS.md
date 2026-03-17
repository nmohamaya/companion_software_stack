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
| `test_fusion_engine` | 5 | Camera-only fusion, depth estimation, multiple objects, factory tests |
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
| P2 perception | Camera intrinsics (fx, fy, cx, cy), camera_height_m |
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

## Zenoh IPC Migration — Phase B (Issue #47, PR #53)

### Improvement — Zenoh Phase B: Low-Bandwidth Channel Migration

**Date:** 2026-02-28
**Category:** IPC / Architecture
**Issue:** [#47](https://github.com/nmohamaya/companion_software_stack/issues/47)
**PR:** [#53](https://github.com/nmohamaya/companion_software_stack/pull/53)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)

**Files Added:**
- `docs/ipc-key-expressions.md` — Naming convention, full channel table, wildcard examples, backend selection guide

**Files Modified:**
- `common/ipc/include/ipc/message_bus_factory.h` — Added `bus_subscribe_optional<T>()` helper (single attempt, no retries)
- `process2_perception/src/main.cpp` — Migrated from `ShmMessageBus` to `MessageBusFactory`
- `process4_mission_planner/src/main.cpp` — Migrated from `ShmMessageBus` to `MessageBusFactory`; replaced `subscribe_lazy` with `bus_subscribe_optional`
- `process5_comms/src/main.cpp` — Migrated from `ShmMessageBus` to `MessageBusFactory`
- `process6_payload_manager/src/main.cpp` — Migrated from `ShmMessageBus` to `MessageBusFactory`
- `process7_system_monitor/src/main.cpp` — Migrated from `ShmMessageBus` to `MessageBusFactory`
- `tests/test_zenoh_ipc.cpp` — Added 13 new ZenohMigration tests

**What:** Migrated all 10 low-bandwidth IPC channels (control & status) from direct `ShmMessageBus` usage to the config-driven `MessageBusFactory`. Process code now calls `create_message_bus(cfg)` + `bus_advertise<T>()` / `bus_subscribe<T>()` — the IPC backend is selected at runtime from `config/default.json`.

**Channels Migrated (10):**

| Channel | Type | Producer | Consumer | Hz |
|---------|------|----------|----------|----|
| `ShmPose` | pose | P3 SLAM | P4 Mission, P5 Comms | 200 |
| `ShmFCState` | status | P5 Comms | P4 Mission | 50 |
| `ShmFCCommand` | command | P4 Mission | P5 Comms | 50 |
| `ShmMissionStatus` | status | P4 Mission | P5 Comms, P7 Monitor | 10 |
| `ShmTrajectoryCmd` | command | P4 Mission | P5 Comms | 50 |
| `ShmGCSCommand` | command | P5 Comms | P4 Mission | ~1 |
| `ShmDetectedObjectList` | data | P2 Perception | P4 Mission | 10 |
| `ShmPayloadCommand` | command | P4 Mission | P6 Payload | ~1 |
| `ShmPayloadStatus` | status | P6 Payload | P4 Mission | 1 |
| `ShmSystemHealth` | status | P7 Monitor | P5 Comms | 1 |

**New Factory Helper:**
```cpp
/// bus_subscribe_optional<T>(bus, topic)
/// Single attempt, no retries. For SHM: is_connected() false if segment
/// doesn't exist. For Zenoh: is_connected() always true (async discovery).
```

**PR Review Fixes (7):** See [BUG_FIXES.md](BUG_FIXES.md) for details.
- R1: Backend-agnostic error message in P2
- R2: GCS subscribe reordered after FC-state blocking subscribe
- R3: Expanded `bus_subscribe_optional` doc comment
- R4: Fixed infinite loop in HighRate_Pose test (non-consuming receive)
- R5: PID-unique key in FactorySubscribeOptional test
- R6: Added explicit `#include <algorithm>`
- R7: Fixed invalid duplicate JSON in docs

**Test Coverage:**

| Category | Tests | Guard |
|---|---|---|
| Per-channel round-trips | 10 | `HAVE_ZENOH` |
| Multi-channel simultaneous | 1 | `HAVE_ZENOH` |
| High-rate pose (200 Hz × 2s) | 1 | `HAVE_ZENOH` |
| Factory subscribe_optional | 1 | `HAVE_ZENOH` |
| **Total new** | **13** | |

**Metrics:**
- Tests: 295 → **308** (+13)
- Processes using MessageBusFactory: 2 → **7** (all processes)
- SHM-only build: 267/267 tests pass (no regression)
- Zenoh build: 308/308 tests pass
- Build: 0 warnings (`-Werror -Wall -Wextra`)

---

## Updated Summary (Post Zenoh Phase B)

*See [Updated Summary (Post Zenoh Migration Complete)](#updated-summary-post-zenoh-migration-complete) at end of document for latest metrics (377 tests, Zenoh Epic #45 complete).*

---

## Zenoh IPC Migration — Phase C (Issue #48, PR #54)

### Improvement — Zenoh Phase C: Zero-Copy SHM Video Publishing

**Date:** 2026-03-01
**Category:** IPC / Performance
**Issue:** [#48](https://github.com/nmohamaya/companion_software_stack/issues/48)
**PR:** [#54](https://github.com/nmohamaya/companion_software_stack/pull/54)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)

**Files Added:**
- `CI_ISSUES.md` — CI issues log (248 lines, CI-001 through CI-003)

**Files Modified:**
- `common/ipc/include/ipc/zenoh_publisher.h` — Added SHM publish path via `PosixShmProvider` for large messages
- `common/ipc/include/ipc/zenoh_session.h` — Added `shm_provider()` accessor, configurable SHM pool size
- `common/ipc/include/ipc/message_bus_factory.h` — Added `shm_pool_mb` parameter, `(void)` cast for SHM-only builds
- `process1_video_capture/src/main.cpp` — Migrated video frame publishing to MessageBusFactory
- `process3_slam_vio_nav/src/main.cpp` — Migrated stereo frame + pose publishing to MessageBusFactory
- `config/default.json` — Added `zenoh.shm_pool_size_mb` config
- `tests/test_zenoh_ipc.cpp` — Added 21 SHM zero-copy tests
- `.github/workflows/ci.yml` — CI cache for zenohc build

**What:** Migrated the 2 high-bandwidth video channels (mission camera ~185 MB/s, stereo camera ~5 MB/s) to Zenoh with zero-copy SHM publishing:

1. **PosixShmProvider** — Creates a shared memory pool (default 64 MB) for zero-copy buffer allocation
2. **SHM publish path** — When SHM provider is available and buffer > threshold, allocates from SHM pool; receiver gets direct pointer (no serialise/deserialise)
3. **Bytes fallback** — If SHM unavailable (CI) or alloc fails, falls back to byte-copy publish
4. **Configurable** — `zenoh.shm_pool_size_mb` in JSON config

**CI Issues Discovered:**
- **CI-001:** Pre-built zenohc debs lack `shared-memory` cargo feature → 4 SHM tests failed → added `GTEST_SKIP()` guards
- **CI-002:** Unused `shm_pool_mb` parameter in SHM-only build → `-Werror` failure → added `(void)` cast
- **CI-003:** Building zenohc from source with SHM feature causes opaque-type size mismatches → reverted to pre-built debs, SHM tests skip on CI

**Metrics:**
- Tests: 308 → **329** (+21)
- Channels migrated: 10 → **12** (all channels on Zenoh)
- Video publish: byte-copy → **zero-copy SHM** (when provider available)

---

## Zenoh IPC Migration — Phase D (Issue #49, PR #55)

### Improvement — Zenoh Phase D: Service Channels + Legacy SHM Cleanup

**Date:** 2026-03-01
**Category:** IPC / Architecture
**Issue:** [#49](https://github.com/nmohamaya/companion_software_stack/issues/49)
**PR:** [#55](https://github.com/nmohamaya/companion_software_stack/pull/55)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)

**Files Added:**
- `common/ipc/include/ipc/zenoh_service_client.h` — `IServiceClient` implementation using Zenoh queryable
- `common/ipc/include/ipc/zenoh_service_server.h` — `IServiceServer` implementation using Zenoh queryable
- `docs/adr/ADR-002-modular-ipc-backend-architecture.md` — Architecture decision record for dual-backend IPC

**Files Removed:**
- `common/ipc/include/ipc/shm_service_channel.h` — Legacy SHM-based service channel (176 lines)

**Files Modified:**
- `common/ipc/include/ipc/message_bus_factory.h` — Added `bus_create_client<Req,Resp>()` and `bus_create_server<Req,Resp>()` factory helpers
- `common/ipc/include/ipc/zenoh_message_bus.h` — Added `create_client()` and `create_server()` methods
- `deploy/launch_all.sh`, `deploy/launch_gazebo.sh`, `deploy/launch_hardware.sh` — Updated for service channel changes
- `tests/test_message_bus.cpp` — Simplified (removed 133 lines of legacy SHM service tests)
- `tests/test_zenoh_ipc.cpp` — Added 19 Zenoh service channel tests

**What:** Replaced the legacy `ShmServiceChannel` (polling-based shared memory request/response) with Zenoh queryable-based service channels:

1. **ZenohServiceServer** — Registers a Zenoh queryable; receives requests as queries, sends responses as replies
2. **ZenohServiceClient** — Sends a query to the service key expression, waits for reply with configurable timeout
3. **Factory integration** — `bus_create_client<Req,Resp>()` / `bus_create_server<Req,Resp>()` in `message_bus_factory.h`
4. **ADR-002** — Documented the dual-backend architecture: SHM pub/sub retained for backward compatibility, Zenoh provides pub/sub + services + network transport

**Metrics:**
- Tests: 329 → **348** (+19 new, -133 legacy removed)
- Legacy code removed: 176 lines (`shm_service_channel.h`)

---

## Zenoh IPC Migration — Phase E (Issue #50, PR #56)

### Improvement — Zenoh Phase E: Network Transport (Drone↔GCS)

**Date:** 2026-03-01
**Category:** IPC / Networking
**Issue:** [#50](https://github.com/nmohamaya/companion_software_stack/issues/50)
**PR:** [#56](https://github.com/nmohamaya/companion_software_stack/pull/56)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)

**Files Added:**
- `common/ipc/include/ipc/wire_format.h` — Platform-independent serialisation (little-endian, CRC32 integrity)
- `common/ipc/include/ipc/zenoh_network_config.h` — Network configuration builder (peer/client/router modes, TLS, endpoints)
- `docs/network-transport.md` — Network transport architecture and deployment guide
- `PRODUCTION_READINESS.md` — Production deployment checklist
- `tests/test_zenoh_network.cpp` — 11 network transport tests
- `tools/gcs_client/gcs_client.py` — Python GCS client for receiving telemetry and sending commands
- `tools/gcs_client/requirements.txt` — Python dependencies (eclipse-zenoh, protobuf)
- `tools/gcs_client/README.md` — GCS client usage guide

**Files Modified:**
- `common/ipc/include/ipc/zenoh_session.h` — Router/peer/client mode support, custom endpoints
- `common/ipc/include/ipc/message_bus_factory.h` — Network config integration
- `config/default.json` — Added `zenoh.network` config section (mode, connect endpoints, listen endpoints)
- `config/hardware.json` — Added network config with GCS endpoint
- All 7 process mains — Updated to pass network config to session initialisation

**What:** Extended Zenoh pub/sub to work across the network, enabling drone↔GCS communication:

1. **Wire Format** — `WireFormat<T>` wraps messages with little-endian header + CRC32 for cross-platform integrity
2. **Network Configuration** — `ZenohNetworkConfig` builder supports peer/client/router modes, TCP/UDP endpoints, TLS mutual auth
3. **Session Modes** — `ZenohSession` now supports `peer` (default, local), `client` (connect to router), `router` (GCS side)
4. **GCS Client** — Python tool subscribes to all drone topics, displays telemetry, sends commands
5. **Key Expression Filtering** — Network-exposed topics filtered via `allow_key_exprs` to prevent leaking internal high-bandwidth channels

**Metrics:**
- Tests: 348 → **359** (+11)
- New docs: `network-transport.md`, `PRODUCTION_READINESS.md`, GCS client README

---

## Zenoh IPC Migration — Phase F (Issue #51, PR #57)

### Improvement — Zenoh Phase F: Process Health via Liveliness Tokens

**Date:** 2026-03-01
**Category:** IPC / Health Monitoring
**Issue:** [#51](https://github.com/nmohamaya/companion_software_stack/issues/51)
**PR:** [#57](https://github.com/nmohamaya/companion_software_stack/pull/57)
**Epic:** [#45 — Zenoh IPC Migration](https://github.com/nmohamaya/companion_software_stack/issues/45)

**Files Added:**
- `common/ipc/include/ipc/zenoh_liveliness.h` — `LivelinessToken` (RAII) + `LivelinessMonitor` (subscriber)
- `docs/process-health-monitoring.md` — Architecture doc for liveliness-based health monitoring
- `tests/test_zenoh_liveliness.cpp` — 11 liveliness token tests

**Files Modified:**
- `common/ipc/include/ipc/shm_types.h` — Added `ProcessHealth` to `ShmSystemHealth` for per-process status
- `process1_video_capture/src/main.cpp` through `process6_payload_manager/src/main.cpp` — Each process declares a liveliness token on startup
- `process7_system_monitor/src/main.cpp` — Added `LivelinessMonitor` that subscribes to `drone/alive/**`, detects process joins/leaves, publishes per-process status in `ShmSystemHealth`

**What:** Implemented automatic process health monitoring using Zenoh's liveliness protocol:

1. **LivelinessToken** — RAII wrapper around `zenoh::Session::liveliness_declare_token()`. Each process declares `drone/alive/{process_name}` on startup; token is automatically undeclared on process exit/crash
2. **LivelinessMonitor** — Subscribes to `drone/alive/**` with `LIVELINESS_GET` + `LIVELINESS_SUB`. Detects process joins (token declared) and leaves (token undeclared). Reports per-process health status
3. **P7 Integration** — System monitor now includes per-process alive/dead status in `ShmSystemHealth`, enabling downstream consumers (comms → GCS) to report process failures

**Key Features:**
- **Zero-config** — No heartbeat intervals to tune; Zenoh's built-in liveliness protocol handles timing
- **Crash detection** — Process crash automatically undeclares token (session drops)
- **No polling** — Event-driven callbacks on join/leave

**PR Review Fixes (8):**
- Token naming collision guards, RAII move semantics, subscriber error handling, documentation improvements, test isolation, monitor thread safety, graceful shutdown ordering, log level adjustments

**Metrics:**
- Tests: 359 → **370** (+11)
- Liveliness tokens: 7 (one per process)

---

## E2E Zenoh Smoke Test + is_connected() Bug Fix (PR #58)

### Improvement — End-to-End Zenoh Smoke Test

**Date:** 2026-03-01
**Category:** Testing / Integration / Bug Fix
**PR:** [#58](https://github.com/nmohamaya/companion_software_stack/pull/58)

**Files Added:**
- `tests/test_zenoh_e2e.sh` — 8-phase E2E smoke test (42 checks)
- `config/zenoh_e2e.json` — Dedicated E2E config (`ipc_backend: "zenoh"`, all simulated backends)

**Files Modified:**
- `common/ipc/include/ipc/zenoh_subscriber.h` — Fixed `is_connected()` to return `subscriber_.has_value()` instead of `has_data_`
- `common/ipc/include/ipc/zenoh_message_bus.h` — Updated docs for subscribe retry params
- `tests/test_zenoh_ipc.cpp` — Updated 2 unit tests for corrected `is_connected()` semantics
- `BUG_FIXES.md` — Added Fix #9

**What:** Created a comprehensive end-to-end smoke test that launches all 7 processes with the Zenoh backend and validates the full stack:

**E2E Test Phases (42 checks):**

| Phase | Description | Checks |
|-------|-------------|--------|
| 1 | Launch all 7 processes (staggered 0.5s) | — |
| 2 | Verify window (configurable, default 15s) | — |
| 3 | Process liveness — all 7 still running | 7 |
| 4 | Zenoh backend selection (log markers) | 7 |
| 5 | Liveliness tokens declared + monitor active | 8 |
| 6 | Data flow verification (log markers per process) | 7 |
| 7 | Death detection — kill payload_manager → P7 detects, others survive | 7 |
| 8 | Graceful shutdown via SIGINT (exit code 0 or 130) | 6 |

**Bug Fix — `ZenohSubscriber::is_connected()` (Fix #9, Critical):**

The E2E test exposed a real integration bug: processes 2, 4, and 6 crashed immediately on Zenoh because `is_connected()` returned `has_data_` (false until first message), but process mains treat false as a fatal error.

- **Root Cause:** SHM vs Zenoh semantics mismatch — SHM checks "does segment exist?" but Zenoh subscriptions are always valid immediately
- **Failed Fix:** Retry polling in `subscribe()` caused circular deadlock (comms ↔ mission_planner)
- **Correct Fix:** `is_connected()` → `subscriber_.has_value()` (true once subscription declared)

See [BUG_FIXES.md](BUG_FIXES.md) Fix #9 for full details.

**Metrics:**
- Tests: 370 → **377** (+7: 2 updated unit tests + E2E script)
- E2E checks: **42/42** passing
- Bug fixes: 17 → **19** (+2: Fix #9 is_connected semantics, Fix #8 was stack overflow from Phase A)

---

### Improvement #20 — FaultManager: Graceful Degradation (Issue #61, PR #63)

**Date:** 2026-03-02  
**Category:** Feature — Safety / Fault Tolerance  
**Issue:** [#61](https://github.com/nmohamaya/companion_software_stack/issues/61)  
**PR:** [#63](https://github.com/nmohamaya/companion_software_stack/pull/63)

**Files Added:**
- `process4_mission_planner/include/planner/fault_manager.h` — FaultManager class, FaultAction/FaultType enums, FaultConfig
- `tests/test_fault_manager.cpp` — 23 unit tests

**Files Modified:**
- `process4_mission_planner/include/planner/mission_fsm.h` — Added `is_in_fault_state()`, `set_fault_triggered()`, `fault_triggered_` member
- `process4_mission_planner/src/main.cpp` — Wired health subscription + fault evaluation loop (reads `ShmSystemHealth`, calls `evaluate()`, triggers FSM transitions + FC commands)
- `common/ipc/include/ipc/shm_types.h` — Added `active_faults` (uint32_t bitmask) and `fault_action` (uint8_t) to `ShmMissionStatus`
- `config/gazebo.json`, `config/gazebo_zenoh.json`, `config/hardware.json`, `config/default.json`, `config/gazebo_sitl.json`, `config/zenoh_e2e.json` — Added `fault_manager` config section
- `tests/CMakeLists.txt` — Registered `test_fault_manager`

**What:** Implemented a config-driven FaultManager library in Process 4 that evaluates system health each loop tick and returns graduated response actions (NONE < WARN < LOITER < RTL < EMERGENCY_LAND). Key design: library in P4 (not separate process) for zero IPC latency. Escalation-only policy ensures actions never downgrade within a flight. Loiter auto-escalates to RTL after configurable timeout.

**8 fault conditions monitored:** critical process death, pose staleness, battery low/critical, thermal warning/critical, perception death, FC link lost.

**Metrics:**
- Tests: 377 → **400** (+23 FaultManager tests)
- Test suites: 22 → **23**
- Config tunables: 90+ → **95+** (+5 fault_manager thresholds)

---

---

## Phase 10 — Foundation Hardening (Epic #64)

### Improvement #21 — Tier 1: CI Infrastructure (Issues #65, #66, #67)

**Date:** 2026-03-03  
**Category:** Infrastructure / CI  
**Issues:** [#65](https://github.com/nmohamaya/companion_software_stack/issues/65), [#66](https://github.com/nmohamaya/companion_software_stack/issues/66), [#67](https://github.com/nmohamaya/companion_software_stack/issues/67)  
**PRs:** [#71](https://github.com/nmohamaya/companion_software_stack/pull/71) (sanitizers), [#72](https://github.com/nmohamaya/companion_software_stack/pull/72) (clang-tidy/format), [#73](https://github.com/nmohamaya/companion_software_stack/pull/73) (coverage)

**What:**
- **Sanitizer support (#67):** CMake `SANITIZER` option supporting ASan, TSan, UBSan. CI expanded from 2 legs to 7 (shm/zenoh × ASan/TSan/UBSan + baseline).
- **clang-tidy + clang-format (#65):** `.clang-tidy` config, `.clang-format` config, `format-check` CI job that enforces style on every PR.
- **Code coverage (#66):** CMake `ENABLE_COVERAGE` option, `coverage` CI job that generates lcov reports.

**CI pipeline post-Tier 1:** 9 jobs (format-check → 7-leg build matrix → coverage).

---

### Improvement #22 — Tier 2: Result<T,E> Error Type (Issue #68, PR #75)

**Date:** 2026-03-04  
**Category:** Architecture — Error Handling  
**Issue:** [#68](https://github.com/nmohamaya/companion_software_stack/issues/68)  
**PR:** [#75](https://github.com/nmohamaya/companion_software_stack/pull/75)

**Files Added:**
- `common/util/include/util/result.h` — `Result<T,E>` class template, `ErrorCode` enum, `Error` class
- `tests/test_result.cpp` — 32 unit tests

**Files Modified:**
- `common/util/include/util/config.h` — Added `load_config()` → `VoidResult`, `require<T>()` → `Result<T>`

**What:** Lightweight monadic `Result<T,E>` type for structured error handling without exceptions. Based on `std::variant<T, E>` storage with monadic API:
- `map()`, `and_then()`, `map_error()`, `value_or()`
- `Result<void, E>` specialization for operations with no return value
- `ErrorCode` enum with 10 codes (INVALID_ARGUMENT, NOT_FOUND, IO_ERROR, etc.)

Design constraint: **no `shared_ptr`** — zero-cost value semantics throughout, suitable for real-time drone software.

**Metrics:**
- Tests: 400 → **442** (+32 Result tests, +10 config Result tests)
- Test suites: 23 → **25**

---

### Improvement #23 — Tier 2: Config Schema Validation (Issue #69, PR #76)

**Date:** 2026-03-04  
**Category:** Architecture — Config Validation  
**Issue:** [#69](https://github.com/nmohamaya/companion_software_stack/issues/69)  
**PR:** [#76](https://github.com/nmohamaya/companion_software_stack/pull/76)

**Files Added:**
- `common/util/include/util/config_validator.h` — `ConfigSchema`, `FieldRule<T>`, 7 pre-built process schemas
- `tests/test_config_validator.cpp` — 22 unit tests

**What:** Startup-time JSON config schema validation with builder-pattern API:
- `ConfigSchema` with `.required<T>()`, `.optional<T>()`, `.required_section()`, `.custom()`
- `FieldRule<T>` with `.range()`, `.one_of()`, `.satisfies()` constraints
- `validate()` collects all errors in one pass, returns `Result<void, vector<string>>`
- 7 pre-built schemas: `common_schema()`, `video_capture_schema()`, `perception_schema()`, `slam_schema()`, `mission_planner_schema()`, `comms_schema()`, `payload_manager_schema()`, `system_monitor_schema()`

**Design:** Uses virtual base class `IFieldRuleBase` + `unique_ptr` for type-erased rule storage — zero overhead via vtable dispatch, no `shared_ptr` (no atomic refcounting in real-time code).

**Metrics:**
- Tests: 442 → **464** (+22 config validator tests)
- Test suites: 25 → **26**

---

### Improvement #24 — Tier 2: [[nodiscard]] Audit (Issue #70, PR #77)

**Date:** 2026-03-04  
**Category:** Code Quality — Correctness  
**Issue:** [#70](https://github.com/nmohamaya/companion_software_stack/issues/70)  
**PR:** [#77](https://github.com/nmohamaya/companion_software_stack/pull/77)

**Files Modified:** 41 files (26 headers + 7 process mains + 8 test files)

**What:** Comprehensive `[[nodiscard]]` annotation of all public methods returning values across the entire codebase:
- IPC interfaces & implementations (7 files)
- Wire format free functions (1 file)
- HAL interfaces (5 files)
- Process headers (3 files)
- Concrete implementations (3 files)
- Zenoh layer (7 files)
- Config utility (1 file, `get()` and `section()`)

All call sites fixed to properly handle return values:
- 7 process mains: `cfg.load()` → `if (!cfg.load()) { warn }`
- Test files: bare calls → `ASSERT_TRUE()` or `(void)` cast with explanation

**Metrics:**
- `[[nodiscard]]` warnings: **0** (compiler-enforced)
- Tests: **464/464** pass (unchanged count)

---

## Updated Summary (Post Foundation Hardening)

| Metric | Phase 7 | Phase 8 | Zenoh F | E2E | FaultMgr | **Hardening** |
|---|---|---|---|---|---|---|
| Bug fixes | 13 | 15 | 17 | **19** | **19** | **19** |
| Unit tests | 262 | 262 | 370 | 377 | 400 | **464** |
| Test suites | 18 | 18 | 21 | 22 | 23 | **26** |
| Compiler warnings | 0 | 0 | 0 | 0 | 0 | **0** |
| CI matrix legs | 1 | 1 | 2 | 2 | 2 | **9** |
| `[[nodiscard]]` headers | — | — | — | — | — | **26** |
| Config schemas | — | — | — | — | — | **7** |
| Sanitizers | — | — | — | — | — | **ASan+TSan+UBSan** |
| Fault conditions | — | — | — | — | **8** | **8** |
| E2E checks | — | — | — | **42/42** | **42/42** | **42/42** |

*Last updated after Foundation Hardening Epic #64 — 464 tests, 26 suites, 42/42 E2E checks.*

---

## Phase 10 — Foundation Hardening (Epic #64)

### Improvement #21 — Sanitizer CI Support (Issue #67, PR #71)

**Date:** 2026-03-03  
**Category:** Infrastructure — Build & CI  
**Issue:** [#67](https://github.com/nmohamaya/companion_software_stack/issues/67)  
**PR:** [#71](https://github.com/nmohamaya/companion_software_stack/pull/71)

**Files Modified:**
- `CMakeLists.txt` — Added `ENABLE_ASAN`, `ENABLE_TSAN`, `ENABLE_UBSAN` options with mutual exclusivity + frame pointer flags
- `.github/workflows/ci.yml` — Expanded matrix from 2→7 legs (shm, zenoh, shm+asan, shm+tsan, shm+ubsan, zenoh+asan, zenoh+ubsan)
- `BUG_FIXES.md` — Fix #11 (TSan ASLR crash on kernel 6.17)
- `CI_ISSUES.md` — CI-005 (TSan `-Wtsan` warning under `-Werror`)

**What:** Added first-class sanitizer support to CMake and CI. Three mutually exclusive options (`ENABLE_ASAN`, `ENABLE_TSAN`, `ENABLE_UBSAN`) each add the appropriate compile/link flags and automatically switch to Debug mode. The CI matrix expanded from 2 legs (shm, zenoh) to 7 legs covering all sanitizer+backend combinations (TSan excluded for Zenoh due to uninstrumented zenohc library). Fixed TSan ASLR crash on kernel 6.17 (`vm.mmap_rnd_bits=28`) and GCC 13 `-Wtsan` warning.

**Metrics:**
- CI matrix: 2 → **7 legs**
- All 400 tests pass under ASan, TSan, and UBSan

---

### Improvement #22 — clang-format + clang-tidy + .editorconfig (Issue #65, PR #72)

**Date:** 2026-03-03  
**Category:** Infrastructure — Code Quality  
**Issue:** [#65](https://github.com/nmohamaya/companion_software_stack/issues/65)  
**PR:** [#72](https://github.com/nmohamaya/companion_software_stack/pull/72)

**Files Added:**
- `.clang-format` — 4-space indent, K&R braces, 100-col limit, left pointer alignment
- `.clang-tidy` — bugprone/cert/cppcoreguidelines/misc/modernize/performance/readability checks with naming conventions
- `.editorconfig` — UTF-8, LF, language-appropriate indent sizes
- `.git-blame-ignore-revs` — ignores the one-time format pass commit in `git blame`

**Files Modified:**
- 91 C++ source files — one-time automated format pass
- `.github/workflows/ci.yml` — Added `format-check` fast gate job (pinned `clang-format-18`)

**What:** Established consistent code style across the entire codebase. The `.clang-format` config was tuned to match the project's existing style (CamelCase classes, lower_case methods, trailing `_` for private members). A `format-check` CI job runs before the build matrix as a fast gate — any formatting drift fails the pipeline immediately. The `.git-blame-ignore-revs` file ensures `git blame` skips the bulk format commit.

---

### Improvement #23 — Code Coverage Reporting (Issue #66, PR #73)

**Date:** 2026-03-03  
**Category:** Infrastructure — Testing  
**Issue:** [#66](https://github.com/nmohamaya/companion_software_stack/issues/66)  
**PR:** [#73](https://github.com/nmohamaya/companion_software_stack/pull/73)

**Files Modified:**
- `CMakeLists.txt` — Added `ENABLE_COVERAGE` option (`--coverage -fprofile-arcs -ftest-coverage`)
- `.github/workflows/ci.yml` — Added `coverage (shm)` CI job: build → test → lcov capture → filter → genhtml → upload artifact (14-day retention)
- `.gitignore` — Added `build-*/`, `*.gcno`, `*.gcda`, `coverage-report/`

**What:** Added gcov/lcov code coverage instrumentation. The `ENABLE_COVERAGE` CMake option adds coverage flags; a dedicated CI job builds with coverage, runs all tests, captures lcov data, filters out system headers and test code, generates an HTML report, and uploads it as a GitHub Actions artifact (14-day retention). Local verification showed **75.1% line coverage** and **84.9% function coverage**.

---

## Updated Summary (Post Foundation Hardening + Watchdog — Epics #64 & #88 COMPLETE)

| Metric | FaultMgr | Tier 1 | Tier 2 | **Watchdog + systemd (Final)** |
|---|---|---|---|---|
| Bug fixes | 19 | 21 | 21 | **21** |
| Automated tests (unit+E2E) | 400 | 400 | 464 | **701** |
| Test suites | 23 | 23 | 26 | **31+** |
| Compiler warnings | 0 | 0 | 0 | **0** |
| CI matrix legs | 2 | 9 | 9 | **9** |
| Line coverage | — | 75.1% | 75.1% | **75.1%** |
| Function coverage | — | 84.9% | 84.9% | **84.9%** |
| Code style | — | enforced | enforced | **enforced** |
| Sanitizers | — | ASan+TSan+UBSan | ASan+TSan+UBSan | **ASan+TSan+UBSan** |
| Error handling | exceptions | exceptions | Result<T,E> | **Result<T,E>** |
| Config validation | — | — | 7 schemas | **7 schemas** |
| `[[nodiscard]]` | — | — | 26 headers | **26 headers** |
| Thread watchdog | — | — | — | **ThreadHeartbeat + ThreadWatchdog** |
| Process supervision | — | — | — | **systemd (production) / ProcessManager (`--supervised`)** |

---

## Phase 11 — Process & Thread Watchdog (Epic #88)

### Improvement #24 — ThreadHeartbeat + ThreadWatchdog Library (Issue #89, PR #94)

**Date:** 2026-03-04  
**Category:** Safety — Stuck Thread Detection  
**Issue:** [#89](https://github.com/nmohamaya/companion_software_stack/issues/89)  
**PR:** [#94](https://github.com/nmohamaya/companion_software_stack/pull/94)

**Files Added:**
- `common/util/include/util/thread_heartbeat.h` — `ThreadHeartbeat` (atomic per-thread), `ThreadHeartbeatRegistry` (global registry), `ThreadSnapshot` (stack-allocated snapshot)
- `common/util/include/util/thread_watchdog.h` — `ThreadWatchdog` scans all heartbeats, reports stuck threads

**What:** Layer 1 of the three-layer watchdog architecture. Each thread gets a `ThreadHeartbeat` that calls `beat()` (a single `atomic_store(relaxed)`, ~1 ns on ARM) every loop iteration. `ThreadWatchdog::scan_once()` compares timestamps against a configurable timeout and reports stuck threads. Entirely lock-free and backend-independent (works with both SHM and Zenoh). See [ADR-004](docs/adr/ADR-004-process-thread-watchdog-architecture.md).

**Tests:** ~40 new tests covering heartbeat lifecycle, registry, snapshot, watchdog scanning, timeout detection

---

### Improvement #25 — ShmThreadHealth + Process Integration (Issue #90, PR #96)

**Date:** 2026-03-05  
**Category:** Safety — Health Publishing  
**Issue:** [#90](https://github.com/nmohamaya/companion_software_stack/issues/90)  
**PR:** [#96](https://github.com/nmohamaya/companion_software_stack/pull/96)

**Files Added:**
- `common/util/include/util/thread_health_publisher.h` — publishes `ShmThreadHealth` to shared memory for P7 consumption
- `common/util/include/util/safe_name_copy.h` — `strncpy` replacement that null-terminates safely

**What:** Layer 1–2 bridge. `ThreadHealthPublisher` takes a snapshot of all thread heartbeats and publishes them as `ShmThreadHealth` for the system monitor (P7) to consume. All 7 process `main.cpp` files were updated to register their worker threads with `ThreadHeartbeat` and call `beat()` in hot loops. Added `safe_name_copy()` utility to replace all raw `strncpy` calls.

**Tests:** ~20 new tests for health publisher, safe_name_copy, ShmThreadHealth struct

---

### Improvement #26 — Process Supervisor (Issue #91, ADR in PR #100)

**Date:** 2026-03-05  
**Category:** Safety — Crash Recovery  
**Issue:** [#91](https://github.com/nmohamaya/companion_software_stack/issues/91)  
**PR (ADR/design):** [#100](https://github.com/nmohamaya/companion_software_stack/pull/100)  
**Implementation PRs:** [#101](https://github.com/nmohamaya/companion_software_stack/pull/101), [#102](https://github.com/nmohamaya/companion_software_stack/pull/102)

**Files Added:**
- `process7_system_monitor/include/monitor/process_manager.h` — fork+exec supervisor with PID tracking
- `process7_system_monitor/include/monitor/iprocess_monitor.h` — interface for process lifecycle

**What:** Layer 2 of the watchdog. P7 can now fork+exec child processes (P1–P6), track their PIDs, detect crashes via `waitpid()`, and restart them. When run with `--supervised` flag, P7 launches all other processes as children. Without the flag, P7 runs in monitor-only mode (for systemd deployments where systemd manages individual processes).

**Tests:** ~30 tests for ProcessManager (fork, exec, PID tracking, crash detection, supervised mode)

---

### Improvement #27 — Restart Policies + Dependency Graph (Issues #92, PRs #101, #102)

**Date:** 2026-03-05  
**Category:** Safety — Intelligent Recovery  
**Issues:** [#92](https://github.com/nmohamaya/companion_software_stack/issues/92)  
**PRs:** [#101](https://github.com/nmohamaya/companion_software_stack/pull/101), [#102](https://github.com/nmohamaya/companion_software_stack/pull/102)

**Files Added:**
- `common/util/include/util/restart_policy.h` — per-process restart policy with exponential backoff, max retries, cooldown
- `common/util/include/util/process_graph.h` — dependency graph (e.g., perception depends on video_capture; restarting video_capture also restarts perception)

**What:** Layer 2 policies. `RestartPolicy` controls when and how often a crashed process is restarted — configurable max retries, initial delay, backoff multiplier, and cooldown period. `ProcessGraph` models inter-process dependencies so that restarting a dependency cascades to all dependents. Stack status is computed from process health: NOMINAL (all alive), DEGRADED (non-critical process down), CRITICAL (critical process exhausted retries).

**Tests:** ~50 tests for restart policy (backoff, cooldown, max retries), process graph (dependency cascade, critical process detection), stack status computation

---

### Improvement #28 — systemd Service Units (Issue #83, PR #107)

**Date:** 2026-03-06  
**Category:** Deployment — OS-Level Supervision  
**Issue:** [#83](https://github.com/nmohamaya/companion_software_stack/issues/83)  
**PR:** [#107](https://github.com/nmohamaya/companion_software_stack/pull/107)

**Files Added:**
- `deploy/systemd/drone-video-capture.service` — P1
- `deploy/systemd/drone-perception.service` — P2, `BindsTo=drone-video-capture.service`
- `deploy/systemd/drone-slam-vio-nav.service` — P3, `BindsTo=drone-perception.service`
- `deploy/systemd/drone-comms.service` — P5
- `deploy/systemd/drone-mission-planner.service` — P4, `BindsTo=drone-comms.service drone-slam-vio-nav.service`
- `deploy/systemd/drone-payload-manager.service` — P6, `BindsTo=drone-comms.service`
- `deploy/systemd/drone-system-monitor.service` — P7, `Type=notify`, `WatchdogSec=10s`
- `deploy/systemd/drone-stack.target` — groups all 7 units
- `deploy/install_systemd.sh` — installs service files, config, binaries to system paths
- `deploy/uninstall_systemd.sh` — removes installed service files and config
- `common/util/include/util/sd_notify.h` — thin `sd_notify()` wrapper (no-op without `-DENABLE_SYSTEMD=ON`)
- `tests/test_sd_notify.cpp` — 11 tests for sd_notify wrapper

**What:** Layer 3 of the watchdog. Seven independent systemd service units (Option B architecture) with `BindsTo=` dependency semantics — when a dependency stops or crashes, systemd also stops all bound dependents. Each unit has `Restart=on-failure`, so systemd restarts them independently. Unlike `Requires=`, `BindsTo=` also arranges for dependents to be stopped when the dependency disappears. P7 uses `Type=notify` with `sd_notify(READY=1)` and `WatchdogSec=10s`. P1–P6 use `Type=simple`. Security hardening includes `NoNewPrivileges`, `ProtectSystem=strict`, `PrivateTmp`, resource limits.

**Key design decision:** Option B (7 independent units) chosen over Option A (single P7 unit with fork+exec) to avoid the orphan re-adoption problem when systemd restarts P7 — children would become orphans adopted by PID 1 and invisible to the new P7 instance.

**Tests:** 11 new sd_notify tests

---

### Improvement #29 — Tech Debt Cleanup (Issues #97, #98, PR #108)

**Date:** 2026-03-06  
**Category:** Code Quality — Tech Debt  
**Issues:** [#97](https://github.com/nmohamaya/companion_software_stack/issues/97), [#98](https://github.com/nmohamaya/companion_software_stack/issues/98)  
**PR:** [#108](https://github.com/nmohamaya/companion_software_stack/pull/108)

**What:**
- #97: Changed `ThreadHeartbeatRegistry::snapshot()` from returning `std::vector<ThreadHeartbeat>` (heap-allocated) to `ThreadSnapshot` struct with `std::array<ThreadHeartbeat, kMaxThreads>` + count (stack-allocated). Zero heap allocations in the monitoring hot path.
- #98: Added `std::max(1, ...)` clamp to P7's `loop_sleep_ms` to prevent busy-spin when `update_rate > 1000`. Replaced remaining raw `strncpy` with `drone::util::safe_name_copy()`.

---

### Improvement #30 — Deploy Scripts + Test Runners (Issue #103, PR #104)

**Date:** 2026-03-05  
**Category:** Infrastructure — Developer Experience  
**Issue:** [#103](https://github.com/nmohamaya/companion_software_stack/issues/103)  
**PR:** [#104](https://github.com/nmohamaya/companion_software_stack/pull/104)

**What:** Updated `deploy/build.sh` with `--test` and `--test-filter` flags. Created `tests/run_tests.sh` modular test runner with module-based filtering (`watchdog`, `ipc`, `perception`, `hal`, `util`, `quick`). Created `deploy/run_ci_local.sh` for local CI simulation. Updated `TESTS.md` and `DEVELOPMENT_WORKFLOW.md`.

---

## Updated Summary (Post Watchdog + systemd — All Hardening Complete)

| Metric | Hardening (Tiers 1-2) | **Watchdog + systemd (Final)** |
|---|---|---|
| Bug fixes | 21 | **21** |
| Unit tests | 464 | **701** |
| Test suites | 26 | **31+** |
| Compiler warnings | 0 | **0** |
| CI matrix legs | 9 | **9** |
| Line coverage | 75.1% | **75.1%** |
| Code style | enforced | **enforced** |
| Sanitizers | ASan+TSan+UBSan | **ASan+TSan+UBSan** |
| Error handling | Result<T,E> | **Result<T,E>** |
| Config schemas | 7 | **7** |
| `[[nodiscard]]` headers | 26 | **26** |
| Thread watchdog | — | **ThreadHeartbeat + ThreadWatchdog** |
| Process supervision | — | **systemd + ProcessManager** |
| Stuck thread detection | — | **Configurable timeout, per-thread atomic heartbeats** |
| Crash recovery | — | **Exponential backoff, dependency-aware restart cascade** |
| OS supervisor | — | **systemd BindsTo + sd_notify watchdog** |

*Last updated after Process & Thread Watchdog (Epic #88) + systemd (#83). See [tests/TESTS.md](../tests/TESTS.md) for current test counts. Three-layer watchdog architecture, systemd service units.*

---

### Improvement #31 — Integration Testing — Scenario-Driven Simulation Harness (Issue #120)

**Date:** 2026-03  
**Category:** Testing & Simulation Infrastructure  
**Issue:** [#120](https://github.com/nmohamaya/companion_software_stack/issues/120)

**Files Added:**
- `tools/fault_injector/main.cpp` — CLI tool for runtime fault injection via SHM IPC
- `tools/fault_injector/CMakeLists.txt` — Build config for fault injector
- `config/scenarios/01_nominal_mission.json` — Nominal 4-waypoint flight scenario
- `config/scenarios/02_obstacle_avoidance.json` — A* planner obstacle avoidance (Tier 2, Gazebo)
- `config/scenarios/03_battery_degradation.json` — 3-tier battery escalation scenario
- `config/scenarios/04_fc_link_loss.json` — FC link loss LOITER→RTL contingency
- `config/scenarios/05_geofence_breach.json` — Geofence polygon breach → RTL
- `config/scenarios/06_mission_upload.json` — Mid-flight waypoint upload
- `config/scenarios/07_thermal_throttle.json` — Thermal zone escalation 0→1→2→3→0
- `config/scenarios/08_full_stack_stress.json` — Concurrent faults, high rate stress test
- `config/scenarios/data/upload_waypoints.json` — Waypoint data for scenario 6
- `tests/run_scenario.sh` — Scenario runner (list, dry-run, single, all, merge, verify)
- `docs/SIMULATION_ARCHITECTURE.md` — Architecture diagram (Mermaid), setup/run docs

**Files Modified:**
- `config/default.json` — Added Phase 3 knobs (geofence polygon, path_planner/obstacle_avoider backend, RTL params, battery thresholds, FC link RTL timeout)
- `config/gazebo_sitl.json` — Same Phase 3 knobs
- `CMakeLists.txt` — Added `add_subdirectory(tools/fault_injector)`

**What:**
- Created a fault injection CLI tool (`fault_injector`) that writes directly to POSIX SHM IPC channels to simulate battery drain, FC link loss/recovery, GCS commands, thermal zone escalation, and mission upload — without requiring Gazebo or PX4.
- Designed 8 parameterized test scenarios (JSON configs) covering nominal flight, obstacle avoidance, battery degradation, FC link loss, geofence breach, mission upload, thermal throttle, and full-stack stress. Each scenario includes config overrides, timed fault sequences, pass criteria (log patterns, process liveness, SHM segments), and manual control knobs.
- Built a scenario runner script (`run_scenario.sh`) that orchestrates launch → config merge → fault injection → verification → report, supporting `--list`, `--dry-run`, `--all`, `--tier`, `--verbose`, and `--timeout`.
- Introduced a two-tier testing model: Tier 1 (SHM-only, no simulator required) and Tier 2 (Gazebo SITL, full closed-loop).
- Created architecture documentation with Mermaid diagrams showing simulated vs. actual components and the full test flow.
- Updated default and Gazebo SITL configs with all Phase 3 planning & safety knobs (geofence, path planner, obstacle avoidance, RTL parameters).

**Test impact:** No new unit tests (tool is an integration/manual test harness). 844 existing tests unchanged.

---

### Improvement #32 — Integration Scenario Bug Fixes — 8 Root Causes (Issue #122, PR #123)

**Date:** 2026-03-08  
**Category:** Bug Fix / Integration Testing  
**Issue:** [#122](https://github.com/nmohamaya/companion_software_stack/issues/122)  
**PR:** [#123](https://github.com/nmohamaya/companion_software_stack/pull/123)

**Files Modified:**
- `common/ipc/include/ipc/shm_types.h` — `ShmFaultOverrides` struct, `FAULT_BATTERY_RTL`, `fault_flags_string()`, `/fault_overrides` SHM name
- `process3_slam_vio_nav/include/slam/ivio_backend.h` — VIO timestamp fix (`steady_clock`)
- `process3_slam_vio_nav/include/slam/ivisual_frontend.h` — Visual frontend timestamp fix
- `process4_mission_planner/src/main.cpp` — Fault flag logging, EXECUTING log, geofence violation log, new-fault-flags tracker
- `process4_mission_planner/include/planner/fault_manager.h` — `FAULT_BATTERY_RTL` usage
- `process4_mission_planner/include/planner/obstacle_avoider_3d.h` — `potential_field_3d` factory alias
- `process5_comms/src/main.cpp` — Mission upload publisher, fault override reader, FC timestamp freeze
- `process7_system_monitor/src/main.cpp` — Thermal zone log, fault override reader
- `tools/fault_injector/main.cpp` — Sideband `/fault_overrides` writer, create-or-attach pattern
- `tests/run_scenario.sh` — `--ipc` flag, transport-aware startup, log path consolidation
- `tests/test_fault_manager.cpp` — Updated for `FAULT_BATTERY_RTL`
- `docs/BUG_FIXES.md` — Fix #17–#24
- `docs/SIMULATION_ARCHITECTURE.md` — Sideband fault injection architecture, transport-aware docs
- `docs/ipc-key-expressions.md` — Added `/fault_overrides` and `/mission_upload` channels

**Root Causes Fixed:**

| RC | Severity | Component | Issue | Fix |
|---|---|---|---|---|
| 1 | Critical | P3 VIO backend | Pose timestamp used frame counter (epoch ~0), causing 4.7M ms stale-pose rejection | Use `steady_clock::now()` in `generate_simulated_pose()` |
| 2 | Medium | P4 Mission Planner | Fault flag bitmask names never logged — impossible to verify fault handling | Added `fault_flags_string()` and active_faults logging |
| 3 | Medium | P5 Comms | `/mission_upload` SHM never published — mission upload scenario fails | Added mission_upload publisher in comms |
| 4 | Low | P7 System Monitor | `"thermal_zone"` string never logged — thermal scenario grep fails | Added thermal_zone to log output |
| 5 | Low | Scenario runner | Missing closing `fi` in `run_scenario.sh` | Fixed syntax |
| 6 | Critical | Fault injector | Race condition: injector writes directly to `/fc_state` and `/system_health`, but comms/sysmon overwrite every 100 ms | Sideband `/fault_overrides` channel with `ShmFaultOverrides` struct; producers merge overrides |
| 7 | High | P5 Comms | FC link-loss: comms sets `connected=false` but keeps publishing fresh timestamps — FaultManager stale-heartbeat never fires | Freeze timestamp when `fc_connected` override is 0 |
| 8 | Medium | P4 Mission Planner | `create_obstacle_avoider("potential_field_3d")` throws `std::runtime_error` — stress test crashes | Added `potential_field_3d` as accepted factory alias |

**Additional Fixes:**
- `FAULT_BATTERY_RTL` enum (distinct from `FAULT_BATTERY_LOW`) for RTL-level escalation
- FSM logs "EXECUTING" state after takeoff (previously only "NAVIGATE")
- Explicit "Geofence: VIOLATED" log line for scenario verification
- New-fault-flags log when flags change without action-level change
- Transport-aware scenario runner: Zenoh uses log grep for startup, SHM segment checks skipped
- Fault injector create-or-attach pattern for all SHM writers (works with Zenoh transport)
- Scenario logs moved from `/tmp/drone_scenario_logs/` to `drone_logs/scenarios/`

**Result:** 7/7 Tier 1 scenarios passing on SHM, 7/7 on Zenoh. 844/844 unit tests unchanged.

---

## Updated Summary (Post Issue #122 Fix — Integration Scenarios Passing)

| Metric | Watchdog + systemd | Epic #110 | **#122 Fix (Current)** |
|---|---|---|---|
| Bug fixes | 21 | 21 | **29** |
| Unit tests (SHM) | 701 | 735 | **735** |
| Unit tests (SHM+Zenoh) | — | 844 | **844** |
| Test suites | 31+ | 42 | **42** |
| Compiler warnings | 0 | 0 | **0** |
| CI matrix legs | 9 | 9 | **9** |
| Line coverage | 75.1% | 75.1% | **75.1%** |
| Code style | enforced | enforced | **enforced** |
| Sanitizers | ASan+TSan+UBSan | ASan+TSan+UBSan | **ASan+TSan+UBSan** |
| Error handling | Result<T,E> | Result<T,E> | **Result<T,E>** |
| Config tunables | 95+ | 110+ | **110+** |
| Config schemas | 7 | 7 | **7** |
| `[[nodiscard]]` headers | 26 | 26 | **26** |
| HAL backends | 8 | 9 | **9** |
| Thread watchdog | ThreadHeartbeat + ThreadWatchdog | ThreadHeartbeat + ThreadWatchdog | **ThreadHeartbeat + ThreadWatchdog** |
| Process supervision | systemd + ProcessManager | systemd + ProcessManager | **systemd + ProcessManager** |
| Planning | — | A* 3D grid + potential field fallback | **A* 3D + potential field + potential_field_3d** |
| Safety subsystems | — | Geofence + 3-tier battery RTL + FC link contingency | **Geofence + FAULT_BATTERY_RTL + FC link freeze** |
| Perception fusion | weighted merge | UKF (RGB + thermal) | **UKF (RGB + thermal)** |
| Tracker | greedy IOU | O(n³) Hungarian (Kuhn-Munkres) | **O(n³) Hungarian (Kuhn-Munkres)** |
| VIO infrastructure | — | Feature extraction + stereo matching + IMU pre-integration | **+ steady_clock timestamps** |
| Fault conditions | 8 | 11 | **12** (+ FAULT_BATTERY_RTL) |
| Integration scenarios | — | 8 (0/7 passing) | **8 (7/7 SHM + 7/7 Zenoh passing)** |
| Fault injector | — | CLI tool (direct SHM write) | **Sideband /fault_overrides channel** |

---

## Improvement #33 — Scenario 02 Obstacle Avoidance: Full Mission Pass with HD-Map + Proximity Collision Detection

**Date:** 2026-03-09
**Context:** Gazebo SITL scenario `02_obstacle_avoidance.json` with GUI.
6 cylindrical obstacles (RED, BLUE, YELLOW, MAGENTA, GREEN, ORANGE) placed in a
30 × 30 m arena. Mission requires visiting 7 waypoints in sequence without collision.
Prior to this work: 5/7 waypoints reached at best; the drone oscillated near WP2
(blue cylinder) and eventually was ejected by a geofence fault.

**Root Causes Resolved:**
Six distinct bugs were fixed to reach a full 7/7 pass (Fixes #30–35 in BUG_FIXES.md):

| # | Root Cause | Fix Summary |
|---|---|---|
| #30 | A* grid cleared every frame (no obstacle memory) | 3-second TTL expiry map per cell |
| #31 | Goal snap oscillation (BFS non-determinism) | Per-waypoint snap cache + lateral preference |
| #32 | Thermal fault at takeoff (temp_crit 95 °C on 100 °C host) | Raised to 120 °C in SITL configs |
| #33 | Geofence breach after WP3 (potential-field pushes to boundary) | Disabled in scenario config + wired flag |
| #34 | A* empty path when start cell in inflated obstacle zone | BFS shell expansion (radius 1–6) to find eff_start |
| #35 | Reactive-only grid misses obstacles before camera range | Two-layer grid: HD-map static + camera TTL |

**Additional Improvements:**
- **Proximity collision detection (Fix #36):** NAVIGATE loop now checks drone ENU
  position against all HD-map obstacles on every tick. Fires `"OBSTACLE COLLISION
  detected"` warn if drone centre enters `radius_m + 0.5 m` of any obstacle while
  at or below obstacle height. Throttled to 2 s cooldown. Supplements the
  existing disarm-based check for scenario pass/fail gating.
- **HD-map obstacle pre-population:** `mission_planner.static_obstacles` JSON array
  loaded at startup; each entry marks a permanent inflated cylinder footprint in the
  A* `static_occupied_` layer. Camera TTL layer confirms them independently.

**Verified Run:**
- All 7/7 waypoints reached, "Mission complete" logged.
- Mission wall-clock: ≈ 191 s from takeoff.
- Zero obstacle collisions, zero thermal faults, zero geofence trips.
- Observed A* re-routing around blue cylinder (WP2 approach) and red cylinder
  (WP4 approach) with visible detours, then converging on waypoint.

## Updated Summary (Post Improvement #33 — Scenario 02 Obstacle Avoidance Passing)

| Metric | #122 Fix | **#33 (Current)** |
|---|---|---|
| Bug fixes | 29 | **36** |
| Unit tests (SHM) | 735 | **735** |
| Compiler warnings | 0 | **0** |
| CI matrix legs | 9 | **9** |
| Integration scenarios (7-node SHM) | 7/7 | **7/7** |
| Integration scenarios (Zenoh) | 7/7 | **7/7** |
| Gazebo SITL scenario 02 (obstacle avoidance) | — | **7/7 WP reached, 0 collisions** |
| A* occupancy layers | 1 (camera TTL) | **2 (HD-map static + camera TTL)** |
| Static obstacles in HD-map | 0 | **6** |
| Collision detection methods | 1 (crash-disarm) | **2 (proximity + crash-disarm)** |
| Config tunables | 110+ | **115+** |

_Last updated after Improvement #33 (Scenario 02 pass, HD-map, proximity collision detection)_

*Last updated after Epic #110 (Core Autonomy & Safety). All 7 sub-issues closed.

---

## Improvement #34 — Full 8-Scenario Suite Green (Gazebo SITL + Zenoh)

**Branch:** `feat/scenario-02-obstacle-avoidance-pass`  
**PR:** [#130](https://github.com/nmohamaya/companion_software_stack/pull/130)  
**Date:** 2026-03-10

### What Changed

This session closed the remaining scenario reliability gaps and ran the first full **8/8 scenario green suite** on Gazebo SITL with Zenoh IPC.

#### RTL Disarm Detection — `nav_was_armed_` Gap (Fix #39)

`nav_was_armed_` was only updated inside `MissionState::NAVIGATE`. If RTL was triggered from `TAKEOFF` (e.g. fast battery drain in scenario 03, thermal fault in scenario 07), the flag stayed `false` and the disarm detection added in the previous session silently never fired — the FSM would stay locked in `MissionState::RTL` indefinitely after the drone grounded and PX4 disarmed.

**Fix:** `nav_was_armed_ = true` seeded immediately before every `fsm.on_rtl()` call — all three entry points:
- Fault escalation (`FaultAction::RTL`) — can fire from TAKEOFF or NAVIGATE
- GCS RTL command (`GCSCommandType::RTL`) — can fire from any state
- Mission complete (inside NAVIGATE) — already safe, made explicit for symmetry

#### OBSTACLE COLLISION Guard — All Scenarios (Fix #40)

All 8 scenarios run in `test_world.sdf` which contains 6 physical obstacles. Previously only scenarios 02 and 05 had `"OBSTACLE COLLISION"` in `log_must_not_contain`. A collision in any scenario would silently not fail the test. Added to all 8.

#### Scenario 05 Geofence Breach — Three Root Causes Fixed (this session — Fix #37 partial)

See BUG_FIXES.md Fix #37 (previously documented). Three root causes: WP4(15,15) clipped magenta cylinder, `altitude_floor_m: 0.0` flooded log, RTL disarm check never fired. All three resolved.

#### Scenario 07 Thermal Throttle — Temp Threshold Regression (Fix #38)

Scenario 07 had `temp_crit_c: 95°C` override. Host CPU runs at ~100°C under simulation. `FAULT_THERMAL_CRITICAL` fired at PREFLIGHT before arming → RTL → unconverged SLAM pose → spurious `FAULT_GEOFENCE_BREACH` → false FAIL. Fixed by removing the temp override — thermal escalation is exercised via the zone-override injector which bypasses temperature calculation entirely. See BUG_FIXES.md Fix #38.

### Full Suite Results

| # | Scenario | Checks | Result |
|---|---|---|---|
| 01 | nominal_mission | 19/19 | ✅ PASS |
| 02 | obstacle_avoidance | 16/16 | ✅ PASS |
| 03 | battery_degradation | 8/8 | ✅ PASS |
| 04 | fc_link_loss | 8/8 | ✅ PASS |
| 05 | geofence_breach | 7/7 | ✅ PASS |
| 06 | mission_upload | 9/9 | ✅ PASS |
| 07 | thermal_throttle | 11/11 | ✅ PASS |
| 08 | full_stack_stress | 11/11 | ✅ PASS |
| **Total** | | **89/89** | **✅ 8/8 GREEN** |

### Documents Updated

- `docs/BUG_FIXES.md` — Fix #38, Known Issue #30 added
- `docs/PROGRESS.md` — this entry
- `docs/ROADMAP.md` — metrics updated, new issues listed

---

## Updated Summary (Post Improvement #34 — Full Suite Green)

| Metric | Improvement #33 | **#34 (Current)** |
|---|---|---|
| Bug fixes | 36 | **38 + 2 known issues** |
| Integration scenarios (Gazebo SITL + Zenoh) | 7/8 (scenario 07 failing) | **8/8 ✅ all green** |
| Total scenario checks passing | ~72/89 | **89/89** |
| RTL disarm detection coverage | NAVIGATE only | **All 3 entry points** |
| OBSTACLE COLLISION guard | 2/8 scenarios | **8/8 scenarios** |
| Geofence scenario correctness | False-pass (crash hidden) | **True-pass (RTL + clean land)** |
| Thermal scenario flakiness | Hot-hardware false-fail | **Stable** |
| Compiler warnings | 0 | **0** |

---

## Improvement #35 — Complete sd_notify / WatchdogSec for All 7 Processes (Issue #131, PR #132)

**Date:** 2026-03-12  
**Category:** Deployment / Reliability  
**Branch:** `fix/issue-131-systemd-notify-watchdog`  
**Commit:** `7ace87c`

**What:** Two checklist items from issue #31 (systemd service files) were left unimplemented when that issue was closed:
1. `sd_notify` / watchdog keepalive calls were only wired for `process7_system_monitor`; processes 1–6 sent no `READY=1` or watchdog pings.
2. `StartLimitBurst=5` was absent from all 7 service files; 6 of 7 also used `Type=simple` instead of `Type=notify`.

**Files Modified (13):**
- `process1_video_capture/src/main.cpp` — `notify_ready()`, `notify_watchdog()`, `notify_stopping()`
- `process2_perception/src/main.cpp` — same
- `process3_slam_vio_nav/src/main.cpp` — same
- `process4_mission_planner/src/main.cpp` — same
- `process5_comms/src/main.cpp` — same
- `process6_payload_manager/src/main.cpp` — same
- `deploy/systemd/drone-video-capture.service` — `Type=notify`, `NotifyAccess=main`, `WatchdogSec=10s`, `StartLimitBurst=5`
- `deploy/systemd/drone-perception.service` — same
- `deploy/systemd/drone-slam-vio-nav.service` — same
- `deploy/systemd/drone-mission-planner.service` — same
- `deploy/systemd/drone-comms.service` — same
- `deploy/systemd/drone-payload-manager.service` — same
- `deploy/systemd/drone-system-monitor.service` — `StartLimitBurst=5` only (already had `Type=notify + WatchdogSec=10s`)

**CI Results:**
- SHM backend: 735/735 tests passed
- Zenoh backend: 844/844 tests passed
- clang-format-18: clean

---

_Last updated after Improvement #35 (sd_notify/WatchdogSec for all 7 processes, PR #132/#133 — 735/735 SHM, 844/844 Zenoh)_

## Improvement #36 — ColorContourDetector Pipeline Optimisation (Issue #128, PR #135)

**Date:** 2026-03-12
**Category:** Performance / Perception
**Branch:** `feature/issue-128-perception-pipeline-optimisation`
**Commit:** `844bafc`

**What:** Eliminated the per-color-class redundant pixel scan bottleneck in `ColorContourDetector`. Previously, detecting N color classes required N independent W×H image passes (one full RGB→HSV conversion per color per pixel). Three optimisations applied:

1. **Single-pass classification (Opt 1):** `build_color_map()` converts each pixel to HSV exactly once and stores the winning color index (or `kNoColor = 0xFF`) in a compact `uint8_t color_map_`. `extract_bboxes()` then runs union-find on `color_map_` per color class — O(W·H) total regardless of N.
2. **Spatial subsampling (Opt 2):** `subsample_` stride (default 2, config key `perception.detector.subsample`) halves both image dimensions before classification, reducing pixel work by up to 4×. Bounding box coordinates are scaled back to full resolution on output.
3. **Frame-rate cap (Opt 6):** Optional `max_fps_` sleep throttle (config key `perception.detector.max_fps`, default 0 = unlimited) via `std::this_thread::sleep_for` to free CPU headroom on hardware.

**Files Modified (6):**
- `process2_perception/include/perception/color_contour_detector.h` — single-pass `build_color_map()` + `extract_bboxes()`, `color_map_` replaces `mask_buf_`, `subsample_`/`max_fps_` members, `<thread>` added
- `tests/test_color_contour_detector.cpp` — 10 new tests added (42 → 52 total)
- `config/gazebo_sitl.json` — `"subsample": 2, "max_fps": 0` added to `perception.detector`
- `config/gazebo.json` — same
- `config/gazebo_zenoh.json` — same
- `config/hardware.json` — same

**CI Results:**
- SHM backend: 746/746 tests passed
- Zenoh backend: 746/746 tests passed
- clang-format-18: clean

---

---

## Improvement #37 — Fix fault_injector IPC: Non-Owning SHM + Zenoh Support (Issues #124, #125)

**Date:** 2026-03-12
**Category:** Bug Fix / IPC
**Branch:** `fix/issue-124-125-fault-injector-ipc`

**What:** Two fault_injector bugs fixed in one PR:

1. **#125 — shm_unlink on exit:** `get_override_writer()` fell back to `ShmWriter::create()` which set `owns_=true`, causing `shm_unlink("/fault_overrides")` on destructor. Fixed by adding `ShmWriter::create_non_owning()` method and using it in the fallback path.

2. **#124 — Zenoh-invisible writes:** `cmd_gcs_command()` and `cmd_mission_upload()` wrote directly via `ShmWriter::attach()`, invisible to Zenoh subscribers. Added `--ipc <shm|zenoh>` CLI flag that routes writes through the transport-agnostic `MessageBus` when using Zenoh. Also added missing `/mission_upload` → `drone/mission/upload` Zenoh key mapping.

**Files Modified (5):**
- `common/ipc/include/ipc/shm_writer.h` — added `create_non_owning()` method
- `common/ipc/include/ipc/zenoh_message_bus.h` — added `/mission_upload` key mapping
- `tools/fault_injector/main.cpp` — non-owning create, `--ipc` flag, `publish_via_bus<T>()`
- `tests/run_scenario.sh` — passes `--ipc` to fault_injector when not using SHM
- `tests/test_shm_ipc.cpp` — added `ShmWriterNonOwning_SegmentSurvivesDestruction` test

**Test additions:** 1 new test (`ShmWriterNonOwning_SegmentSurvivesDestruction`)

---

## Improvement #38 — Upgrade Gazebo Configs to Full Avoidance Stack (Issue #137)

**Date:** 2026-03-12
**Category:** Configuration / Simulation
**Branch:** `feature/issue-137-gazebo-full-avoidance-stack`

**What:** The Gazebo SITL world (`test_world.sdf`) contains 6 physical obstacles, but all three Gazebo configs used the simple 2D `potential_field` planner/avoider — meaning `launch_gazebo.sh` flew through obstacles without avoidance. Updated all Gazebo configs to use the A* planner + 3D potential-field avoider + HD-map static obstacles, so every Gazebo run exercises the full avoidance stack (previously only Scenario 02 did).

**Why:** Only Scenario 02 previously exercised the full avoidance stack. Standard Gazebo launches flew through obstacles, leaving the core planning/avoidance code untested in normal simulation runs.

**Files Modified (3):**
- `config/gazebo_sitl.json` — A* planner, 3D avoider, 6 static obstacles, obstacle-threading waypoints, cruise speed 2.0→3.0
- `config/gazebo.json` — Added `path_planner`/`obstacle_avoider` backends (previously missing), same avoidance upgrades, cruise speed 5.0→3.0
- `config/gazebo_zenoh.json` — Same mission_planner upgrades as `gazebo.json`, cruise speed 5.0→3.0

**Key parameter changes (all three configs):**
- `path_planner.backend`: `potential_field` → `astar`
- `obstacle_avoider.backend`: `potential_field` → `potential_field_3d`
- `repulsive_gain`: 2.0 → 3.0
- `influence_radius_m`: 5.0 → 4.0
- `acceptance_radius_m`: 1.0/2.0 → 2.0 (A* paths don't hit exact waypoint centers)
- Added `static_obstacles[]` with 6 entries corresponding to `test_world.sdf` (coords in internal nav frame: X=North=Gazebo Y, Y=East=Gazebo X; radii are conservative ~0.75m footprints)
- Waypoints updated to thread through/near obstacles (reused Scenario 02's tuned waypoints)

**Test additions:** None (config-only change). All existing tests pass.

---

## Improvement #39 — Zenoh-Only IPC: Remove Legacy SHM Backend (Issue #126)

**Date:** 2026-03-13
**Category:** Refactor / IPC Architecture
**Issue:** [#126](https://github.com/nmohamaya/companion_software_stack/issues/126) — [Epic] Zenoh-Only IPC — Remove Legacy SHM, Keep Middleware-Swappable
**Branch:** `feature/issue-126-zenoh-only-ipc`

**What:** Removed the entire POSIX SHM IPC backend (~1,500 lines of code). Zenoh is now the sole IPC transport. The `ENABLE_ZENOH` / `HAVE_ZENOH` compile guards were removed — Zenoh is always built. The `MessageBus` variant now wraps only `ZenohMessageBus`. Factory defaults to `"zenoh"`; requesting `"shm"` logs an error and falls back to Zenoh. Renamed `shm_types.h` to `ipc_types.h` and all `Shm*` type prefixes to transport-agnostic names. Renamed `shm_names::` namespace to `topics::`. Migrated `fault_injector`, P5, and P7 from direct `ShmReader` usage to `MessageBus` subscribers.

**Why:** Maintaining two IPC backends (SHM + Zenoh) doubled the CI matrix, added compile guards throughout the codebase, and bloated header includes. Since Zenoh covers both local (zero-copy SHM) and network transport, the legacy POSIX SHM backend was dead weight. Removing it simplifies the build, cuts CI from 9 to 5 jobs, and eliminates an entire class of "did you build with Zenoh?" developer pitfalls.

**Files Deleted (4):**
- `common/ipc/include/ipc/shm_writer.h` — SeqLock-based POSIX SHM writer
- `common/ipc/include/ipc/shm_reader.h` — SeqLock-based POSIX SHM reader
- `common/ipc/include/ipc/shm_publisher.h` — ShmPublisher wrapper
- `common/ipc/include/ipc/shm_subscriber.h` — ShmSubscriber wrapper
- `common/ipc/include/ipc/shm_message_bus.h` — ShmMessageBus factory
- `tests/test_shm_ipc.cpp` — 9 SHM-specific unit tests
- `deploy/clean_build_and_run_shm.sh` — SHM-specific launch script

**Files Modified (key):**
- `common/ipc/include/ipc/shm_types.h` → renamed to `ipc_types.h`; all `Shm*` structs renamed to transport-agnostic names
- `common/ipc/include/ipc/shm_names.h` → `shm_names::` namespace renamed to `topics::`
- `common/ipc/include/ipc/message_bus_factory.h` — `MessageBus` variant wraps only `ZenohMessageBus`; `"shm"` falls back with error log
- `CMakeLists.txt` — `find_package(zenohc REQUIRED)`, removed `ENABLE_ZENOH` option
- `tests/CMakeLists.txt` — removed `test_shm_ipc.cpp`, removed `HAVE_ZENOH` guards from Zenoh tests
- `deploy/build.sh` — `--zenoh` flag deprecated (now a no-op)
- `deploy/clean_build_and_run_zenoh.sh` → renamed to `deploy/clean_build_and_run.sh`
- `.github/workflows/ci.yml` — 9 → 5 jobs (removed SHM/Zenoh build matrix)
- `deploy/run_ci_local.sh` — 10 → 6 local CI jobs
- `config/default.json` — `ipc_backend` changed from `"shm"` to `"zenoh"`
- `tools/fault_injector/main.cpp` — migrated from `ShmReader` to `MessageBus` subscriber
- All 7 process `main.cpp` files — updated type names and includes

**Net change:** ~1,500 lines removed, ~289 lines added.

**Test results:** 845 tests pass, 0 failures, 0 compiler warnings.

**Summary table update:**

| Metric | Before (Improvement #38) | After (Improvement #39) |
|--------|--------------------------|------------------------|
| IPC backends | SHM + Zenoh | Zenoh only |
| CI jobs | 9 | 5 |
| Local CI jobs | 10 | 6 |
| Compile guards | `ENABLE_ZENOH` / `HAVE_ZENOH` | None (Zenoh always on) |
| Unit tests | 844 | 845 |
| SHM code | ~1,500 lines | 0 lines |

---

_Last updated after Improvement #39 (Zenoh-Only IPC, Issue #126). See [tests/TESTS.md](../tests/TESTS.md) for current test counts. 845 tests, 5 CI jobs, Zenoh sole IPC backend._
## Improvement #39 — Documentation Completeness: HAL, Config Reference, Error Handling, Observability Cross-referencing (Issue #149)

**Date:** 2026-03-13
**Category:** Documentation
**Branch:** `docs/issue-149-hal-config-error-handling-observability`

**What:** Filled four major documentation gaps identified after Phase 11:

1. **HAL Design Documentation** — `docs/hal_design.md` covering all 5 common HAL
   interfaces (`ICamera`, `IFCLink`, `IGCSLink`, `IGimbal`, `IIMUSource`) + factory
   function reference + process-local Strategy interfaces (`IPathPlanner`,
   `IObstacleAvoider`, `IProcessMonitor`) + backend availability matrix.
2. **ADR-006: HAL Strategy** — `docs/adr/ADR-006-hal-hardware-abstraction-strategy.md`
   documenting the Strategy + factory pattern decision, alternatives considered
   (ROS2 HAL, template policies), and the three-tier backend hierarchy.
3. **Config Parameter Reference** — `docs/config_reference.md` mapping all 95+
   `config/default.json` keys with type, default, valid range, consuming process, and
   description. Includes a cross-reference table from `"backend"` config keys to HAL
   factory calls.
4. **ADR-007: Error Handling** — `docs/adr/ADR-007-error-handling.md` documenting
   the `Result<T,E>` adoption decision over exceptions, alternatives evaluated
   (`std::expected`, raw `bool`, `std::optional`), and the domain-alias pattern.
5. **Error Handling Design Doc** — `docs/error_handling_design.md` with the full
   `ErrorCode` enum, `Error` class, `Result<T,E>` / `Result<void,E>` / `VoidResult`
   API reference, monadic combinator examples, domain alias pattern (`VIOResult`),
   and usage rules.
6. **Observability Cross-referencing** — Added `## Observability` sections to all 7
   process design docs with per-process structured logging field tables, correlation
   ID flow, and latency tracking channel tables — all cross-linked to
   `docs/observability.md`.

**Files Added (5):**
- `docs/adr/ADR-006-hal-hardware-abstraction-strategy.md`
- `docs/adr/ADR-007-error-handling.md`
- `docs/hal_design.md`
- `docs/config_reference.md`
- `docs/error_handling_design.md`

**Files Modified (7):**
- `docs/video_capture_design.md` — added `## Observability` section
- `docs/perception_design.md` — added `## Observability` section
- `docs/slam_vio_nav_design.md` — added `## Observability` section
- `docs/mission_planner_design.md` — added `## Observability` section
- `docs/comms_design.md` — added `## Observability` section
- `docs/payload_manager_design.md` — added `## Observability` section
- `docs/system_monitor_design.md` — added `## Observability` section

**Test additions:** None (documentation only). All 856 tests pass.

---

_Last updated after Improvement #39 (Documentation gaps #149). See [tests/TESTS.md](../tests/TESTS.md) for current test counts._

## Improvement #40 — Post-merge SHM Remnant Cleanup (Issue #153)

**Date:** 2026-03-13
**Category:** Bug Fix / Tech Debt
**Commit:** `ad7dfd7` (direct to main)

**What:** Post-merge audit of PR #151 (Zenoh-only IPC) found two functional bugs
and several naming/comment remnants not caught during review:

**Functional bugs fixed:**
1. `process3_slam_vio_nav/src/main.cpp` — `ipc_backend` default was `"shm"`
   instead of `"zenoh"`. If `config/default.json` ever omitted the key, P3 would
   default to `"shm"`, hit `is_connected() == false` on the Zenoh subscriber
   (Zenoh doesn't set this at startup), and **exit with error code 1** —
   killing the SLAM/VIO stack silently.
2. `common/util/include/util/config_validator.h` — `one_of({"shm", "zenoh"})`
   still accepted `"shm"` as valid without error. Now `one_of({"zenoh"})` only.

**Naming/comment cleanup:**
- P5 `comms`: local variable `shm_cmd` → `gcs_cmd`
- `ipublisher.h`, `isubscriber.h`: doc comments updated (`ShmPublisher` → `ZenohPublisher`)
- `zenoh_message_bus.h`: removed "Mirrors ShmMessageBus API" phrasing
- `wire_format.h`: `SLAM_POSE` comment `ShmSlamPose` → `Pose`
- `deploy/launch_*.sh`: stale `shm_types.h` and `ipc_backend=shm` comments updated
- `tests/test_correlation.cpp`: suite `ShmCorrelation` → `IpcCorrelation`
- `tests/test_thread_health_publisher.cpp`: suite `ShmThreadHealthStruct` → `ThreadHealthStruct`
- `tests/test_zenoh_network.cpp`: stale comment updated
- `tests/TESTS.md`: updated all suite names and `shm_types.h` file references

**Files modified (13):** `process3_slam_vio_nav/src/main.cpp`,
`process5_comms/src/main.cpp`, `common/ipc/include/ipc/ipublisher.h`,
`common/ipc/include/ipc/isubscriber.h`, `common/ipc/include/ipc/wire_format.h`,
`common/ipc/include/ipc/zenoh_message_bus.h`,
`common/util/include/util/config_validator.h`,
`deploy/launch_all.sh`, `deploy/launch_gazebo.sh`, `deploy/launch_hardware.sh`,
`tests/test_correlation.cpp`, `tests/test_thread_health_publisher.cpp`,
`tests/test_zenoh_network.cpp`

**Test additions:** None (all existing 845 tests pass; two test suites renamed).

---

_Last updated after Improvement #40 (SHM remnant cleanup, Issue #153). See [tests/TESTS.md](../tests/TESTS.md) for current test counts. 845 tests, 6 CI jobs, Zenoh sole IPC backend._

## Improvement #41 — P4 Mission Planner Refactor: Extract 4 Classes from main.cpp (Issue #154, PR #157)

**Date:** 2026-03-14
**Category:** Refactor / Code Quality
**Issue:** [#154](https://github.com/nmohamaya/companion_software_stack/issues/154)
**PR:** [#157](https://github.com/nmohamaya/companion_software_stack/pull/157)
**Branch:** `refactor/issue-154-mission-planner-extract`

**What:** Extracted ~530 lines of inline logic from `process4_mission_planner/src/main.cpp` into 4 focused, header-only classes with 35 new unit tests. Reduced main.cpp from 809 to 366 lines while preserving the 9-step execution order contract.

**Files Added (8):**
- `process4_mission_planner/include/planner/static_obstacle_layer.h` — HD-map obstacle loading, camera cross-check confirmation, collision detection, unconfirmed approach warnings (~150 lines)
- `process4_mission_planner/include/planner/gcs_command_handler.h` — GCS command dispatch (RTL/LAND/MISSION_UPLOAD), dedup by timestamp, correlation ID propagation (~130 lines)
- `process4_mission_planner/include/planner/fault_response_executor.h` — Fault response execution (WARN/LOITER/RTL/EMERGENCY_LAND), escalation-only policy (~105 lines)
- `process4_mission_planner/include/planner/mission_state_tick.h` — Per-tick FSM logic (PREFLIGHT/TAKEOFF/NAVIGATE/RTL/LAND), owns tracking variables (~280 lines)
- `tests/test_static_obstacle_layer.cpp` — 12 tests
- `tests/test_gcs_command_handler.cpp` — 6 tests
- `tests/test_fault_response_executor.cpp` — 7 tests
- `tests/test_mission_state_tick.cpp` — 10 tests

**Files Modified (2):**
- `process4_mission_planner/src/main.cpp` — Replaced inline logic with extracted class calls (809 → 366 lines)
- `tests/CMakeLists.txt` — Added 4 new `add_drone_test()` entries

**Design Decisions:**
- **SharedFlightState struct** — Mutable state (`land_sent`, `nav_was_armed`, `rtl_start_time`) shared between GCSCommandHandler, FaultResponseExecutor, and MissionStateTick. Owned by MissionStateTick, passed by reference to avoid circular dependencies.
- **consume_fault_reset() pattern** — MissionStateTick signals landing via a flag, main.cpp reads and clears it to reset FaultManager. Avoids coupling MissionStateTick to FaultManager.
- **Header-only** — Matches existing codebase pattern; no CMakeLists.txt changes needed for production code.
- **9-step execution order preserved** — Inputs → staleness → obstacles → geofence → fault eval → fault exec → GCS → state tick → publish status.

**Test Coverage (35 new tests):**

| Suite | Tests | Covers |
|---|---|---|
| `StaticObstacleLayer` | 12 | Load empty/single/multi, cross-check 2-hit confirmation, low-quality skip, distant detection skip, collision margin, cooldown throttle, height check, unconfirmed approach |
| `GCSCommandHandler` | 6 | RTL/LAND/MISSION_UPLOAD dispatch, dedup by timestamp, correlation ID, disconnected subscriber |
| `FaultResponseExecutor` | 7 | WARN/LOITER/RTL/EMERGENCY_LAND actions, escalation-only, non-airborne skip, reset |
| `MissionStateTick` | 10 | ARM retry, armed transition, takeoff altitude, waypoint+payload, mission complete RTL, disarm detection, RTL disarm→IDLE, landed transition, land_sent guard |

**Test count:** 845 → 880 (+35 tests, 4 new suites)

---

_Last updated after Improvement #41 (P4 Mission Planner refactor, Issue #154, PR #157). 880 tests, 46 test suites._
## Improvement #42 — Stale SHM Reference Cleanup in API.md and ROADMAP.md (Issue #155)

**Date:** 2026-03-13
**Category:** Documentation / Tech Debt
**Branch:** `docs/issue-155-api-shm-cleanup`
**PR:** [#156](https://github.com/nmohamaya/companion_software_stack/pull/156)
**Commit:** `f73c863`

**Files modified (2):** `docs/API.md`, `docs/ROADMAP.md`

**What:** Removed all remaining stale POSIX SHM references from `docs/API.md` and `docs/ROADMAP.md` that were missed during the Zenoh-only IPC migration (Issue #126, PR #151).

**API.md changes (14 fixes):**
- `ZenohSubscriber` "Why a wrapper?": removed "same `receive(T&)` API as the SHM backend" comparison
- `ZenohMessageBus` description: replaced "Drop-in replacement for `ShmMessageBus`" with standalone description
- `ZenohMessageBus` `to_key_expr`: param `shm_name` → `channel_name`; description updated
- `ZenohMessageBus` "Why a wrapper?": removed all `ShmMessageBus` references; reflects Zenoh-only world
- Key-expression table: column header `SHM Name` → `Channel Name`
- `MessageBusFactory` `create_message_bus`: default `backend = "shm"` → `"zenoh"`
- `MessageBusFactory` description: removed "Requesting `\"shm\"` logs error" — validator now rejects it
- `MessageBusFactory` `bus_subscribe_optional`: removed SHM-specific `is_connected()` note
- `MessageBusFactory` "Why a wrapper?": removed `"shm"` option reference
- Section 5 code comment: `ShmMessageBus or ZenohMessageBus` → `Returns ZenohMessageBus (sole backend)`
- `safe_name_copy` description: removed "SHM" from "fixed-size SHM name buffers"
- Section 8 migration table: all phases A–F updated to **Done** with correct PR numbers
- Section 8 intro: future tense → past tense (migration complete)
- Zenoh Types table: `12-segment SHM→Zenoh topic mapping` → `12-channel Zenoh topic mapping`

**ROADMAP.md changes (2 fixes):**
- Issue #126 epic entry: `**In Progress**` → `~~strikethrough~~ **Closed** ✅ (PR #151)`
- Footer: `Improvement #39` → `#40`; `5 CI jobs` → `6 CI jobs`

**Why:** These were purely documentation gaps — no functional impact — but misleading to anyone reading the API reference. Discovered during post-merge doc audit after Issue #153.

**Test additions:** None (docs-only change; 845 tests unchanged).

---

_Last updated after Improvement #42 (API.md/ROADMAP.md SHM cleanup, Issue #155). See [tests/TESTS.md](../tests/TESTS.md) for current test counts. 880 tests, 46 test suites, 6 CI jobs, Zenoh sole IPC backend._

## Improvement #43 — D* Lite Incremental Path Planner + Grid Planner Refactor (Issue #158)

**Date:** 2026-03-14
**Category:** Feature / Refactor / Performance
**Issue:** [#158](https://github.com/nmohamaya/companion_software_stack/issues/158)
**Branch:** `feature/issue-158-dstar-lite-planner`

**What:** Implemented D* Lite (Koenig & Likhachev, 2002) incremental path planner and refactored the grid-based planner infrastructure. D* Lite searches backward from goal and maintains its priority queue across frames — replanning is O(changed cells) instead of O(grid), significantly reducing replan cost in obstacle-dense environments.

**Phase 1 — Extract shared infrastructure (refactor, no behavior change):**
- Extracted `OccupancyGrid3D`, `GridCell`, `GridCellHash`, neighbor tables from `astar_planner.h` into `occupancy_grid_3d.h`
- Added change tracking to `OccupancyGrid3D`: `changed_cells_` vector, `drain_changes()`, `pending_changes()` for incremental planners
- Created `GridPlannerBase` (template method pattern) with shared logic: goal snapping, path following, EMA velocity smoothing, speed ramping, replan timing
- `AStarPathPlanner` now extends `GridPlannerBase` instead of implementing `IPathPlanner` directly
- Updated integration points: `static_obstacle_layer.h`, `mission_state_tick.h`, `main.cpp` use `IGridPlanner*` instead of `AStarPathPlanner*`

**Phase 2 — D* Lite planner:**
- Backward search from goal with `g(s)` / `rhs(s)` consistency tracking
- `std::set<QueueEntry>` priority queue with two-key ordering for O(log n) decrease-key
- `km_` correction factor for drone movement between replans
- Incremental replanning: `drain_changes()` → update affected vertices → `compute_shortest_path()` (only re-expands changed nodes)
- Large change threshold (>500 cells) triggers full reinitialisation instead of incremental update
- Wall-clock timeout guard: `steady_clock` checked every 64 iterations in both A* and D* Lite

**Phase 3 — Tests, configs, documentation:**
- 23 new D* Lite tests + 1 new A* timeout test = 24 new tests
- Updated Gazebo configs: `"astar"` → `"dstar_lite"`, added `"max_search_time_ms": 50`
- New `planner_factory.h` supporting `"potential_field"`, `"astar"`, `"dstar_lite"` backends

**Files Added (4):**
- `process4_mission_planner/include/planner/occupancy_grid_3d.h` — extracted grid + change tracking
- `process4_mission_planner/include/planner/grid_planner_base.h` — `IGridPlanner` interface + `GridPlannerBase` shared logic
- `process4_mission_planner/include/planner/dstar_lite_planner.h` — D* Lite implementation
- `tests/test_dstar_lite_planner.cpp` — 23 tests (change tracking, search, incremental replan, timeout, integration)

**Files Modified (9):**
- `process4_mission_planner/include/planner/astar_planner.h` — extends `GridPlannerBase`, added timeout support
- `process4_mission_planner/include/planner/planner_factory.h` — relocated factory, added `"dstar_lite"` backend + config param
- `process4_mission_planner/include/planner/static_obstacle_layer.h` — `AStarPathPlanner*` → `IGridPlanner*`
- `process4_mission_planner/include/planner/mission_state_tick.h` — `AStarPathPlanner*` → `IGridPlanner*`
- `process4_mission_planner/src/main.cpp` — factory wiring with `GridPlannerConfig` from JSON
- `tests/test_astar_planner.cpp` — added `MaxSearchTimeEnforced` timeout test
- `tests/test_process_interfaces.cpp` — updated include for relocated factory
- `tests/test_mission_state_tick.cpp` — updated include for relocated factory
- `tests/CMakeLists.txt` — added `test_dstar_lite_planner` target

**Config Changes (3):**
- `config/gazebo_sitl.json` — `"backend": "dstar_lite"`, `"max_search_time_ms": 50`
- `config/gazebo.json` — same
- `config/gazebo_zenoh.json` — same

**Test count:** 880 → 904 (+24 tests, 2 new suites)

---

## Improvement #44 — ByteTrack Multi-Object Tracker: Two-Stage Association (Issue #163, PR #165)

**Date:** 2026-03-14
**Category:** Feature / Perception
**Issue:** [#163](https://github.com/nmohamaya/companion_software_stack/issues/163)
**PR:** [#165](https://github.com/nmohamaya/companion_software_stack/pull/165)

**What:** Implemented ByteTrack (Zhang et al., ECCV 2022) two-stage association tracker to reduce ID switches during partial occlusion. Stage 1 matches all tracks against high-confidence detections using IoU cost + Hungarian assignment. Stage 2 recovers unmatched tracks using low-confidence detections — no CNN features required. Reuses existing `KalmanBoxTracker` and `HungarianSolver` with zero new dependencies.

**Key Design:**
- **Two-stage association:** High-conf detections match first (Stage 1), then low-conf detections recover unmatched tracks (Stage 2). Only high-conf detections create new tracks.
- **IoU cost matrix:** Replaces SORT's center-distance cost with intersection-over-union for more robust bbox matching.
- **Config-driven params:** `high_conf_threshold`, `low_conf_threshold`, `max_iou_cost` in `config/default.json`.
- **No raw/shared pointers:** Value vector of `KalmanBoxTracker`, RAII throughout.

**Files Added (2):**
- `process2_perception/include/perception/bytetrack_tracker.h` — `ByteTrackTracker` class with `ByteTrackParams`
- `process2_perception/src/bytetrack_tracker.cpp` — IoU computation, cost matrix, two-stage update
- `tests/test_bytetrack_tracker.cpp` — 18 tests across 6 suites

**Files Modified (5):**
- `process2_perception/src/kalman_tracker.cpp` — Factory: added `"bytetrack"` backend with config-driven params
- `process2_perception/include/perception/itracker.h` — Updated comment: supported backends
- `process2_perception/CMakeLists.txt` — Added `bytetrack_tracker.cpp` source
- `tests/CMakeLists.txt` — Added test target + linked bytetrack to existing test targets
- `config/default.json` — Added `high_conf_threshold`, `low_conf_threshold`, `max_iou_cost` keys

**Test count:** 904 → 922 (+18 tests, 6 new suites)

---

## Improvement #45 — Scenario Config Updates for ByteTrack Tracker (Issue #167)

**Date:** 2026-03-14
**Category:** Bug Fix / Testing / Config
**Files Modified:**
- `config/scenarios/02_obstacle_avoidance.json`
- `config/scenarios/09_perception_tracking.json` (new)

**What:** Fixed a bug in Scenario 02 pass_criteria where `"Path planner: astar"` didn't match the actual D* Lite backend log output (`"Path planner: DStarLitePlanner"`). Updated Scenario 02 to exercise ByteTrack tracker instead of default SORT — this is the perception-heavy obstacle avoidance scenario, ideal for ByteTrack's two-stage association (obstacles partially occlude each other). Created new Scenario 09 (Perception Tracking) as a dedicated Tier 1 scenario validating ByteTrack backend switching and two-stage association without Gazebo.

**Why:** After implementing ByteTrack (#163), no scenario exercised the new tracker backend. The pass_criteria bug would cause false failures in Gazebo SITL runs.

**Test additions:** +17 scenario checks (1 new check in scenario 02, 16 checks in new scenario 09). Total scenario checks: 97 across 9 scenarios (8 Tier 1 + 1 Tier 2).

---

## Improvement #46 — Responsive Simulated VIO + Thermal Threshold Fix (Issue #167)

**Date:** 2026-03-14
**Category:** Feature / Bug Fix / Config
**Files Modified:**
- `process3_slam_vio_nav/include/slam/ivio_backend.h` — SimulatedVIOBackend target-following dynamics
- `process3_slam_vio_nav/src/main.cpp` — trajectory command forwarding to VIO
- `tests/run_scenario.sh` — dynamic collection window (remaining timeout budget)
- `config/default.json` — raised thermal thresholds from 80/95°C to 105/120°C
- `config/hardware.json` — added thermal calibration documentation note
- `config/scenarios/09_perception_tracking.json` — removed incorrect thermal override
- `docs/ROADMAP.md` — added predictive thermal trend monitoring to Phase 12
- `docs/config_reference.md` — updated thermal threshold defaults and documentation

**What:** Made `SimulatedVIOBackend` respond to trajectory commands via first-order target-following dynamics, enabling Tier 1 scenarios to validate actual waypoint navigation, mission completion, and RTL logic without Gazebo. Fixed the scenario runner's hardcoded 5-second collection window to use remaining timeout budget. Fixed thermal threshold override bug where scenario 09's `system_monitor.temp_warn_c`/`temp_crit_c` was at the wrong config nesting level (needed `system_monitor.thresholds.temp_warn_c`). Raised default thermal thresholds to 105/120°C for dev machines and documented that real hardware thresholds must be calibrated per-platform based on thermal runaway characteristics.

**Why:** SimulatedVIOBackend previously traced a fixed circular path ignoring trajectory commands, so no Tier 1 scenario could verify waypoint acceptance or mission completion. The 5-second collection window was too short for missions to complete. The thermal override used the wrong config path, causing `FAULT_THERMAL_CRITICAL` → RTL on hot dev machines (CPU at 99°C exceeded default 95°C threshold).

**Test impact:** Scenario 09 now validates full mission completion ("Mission complete" in pass_criteria). All 8 Tier 1 scenarios pass on hot dev machines (scenario 02 is Tier 2, requiring Gazebo).

---

## Improvement #47 — Fix Gazebo VIO Config Key Mismatch + Debugging Workflow Improvements (Issue #170)

**Date:** 2026-03-17
**Category:** Bug Fix / Documentation
**Issue:** [#170](https://github.com/nmohamaya/companion_software_stack/issues/170)

**What:** Fixed a critical config key mismatch in `config/gazebo.json` where `slam.visual_frontend` (a stale key from a prior refactor) should have been `slam.vio`. This caused P3 to silently fall back to `SimulatedVIOBackend` during manual Gazebo SITL runs (`launch_gazebo.sh`), making the drone unable to navigate toward obstacles or follow waypoints. Same class of bug as Fix #25, which fixed the identical issue in `gazebo_sitl.json` but didn't propagate to `gazebo.json`.

**Why:** Scenario 02 (obstacle avoidance) regressed in manual Gazebo testing — the drone no longer flew toward obstacles. Root cause: P3 reads `slam.vio.backend` but `gazebo.json` had the value under the stale `slam.visual_frontend` key, so the VIO factory returned `SimulatedVIOBackend` (synthetic pose) instead of `GazeboVIOBackend` (ground-truth odometry).

**Files Modified (3):**
- `config/gazebo.json` — renamed `slam.visual_frontend` → `slam.vio`
- `docs/BUG_FIXES.md` — added Fix #41 (stale config key, root cause, prevention)
- `docs/sim_debugging_workflow.md` — added config verification step, copy-pastable commands with `LOG_DIR` variable, Pattern 5 (config key mismatch), new worked example, updated troubleshooting flowchart

**Test impact:** No new tests. Existing Tier 2 scenario 02 should now pass in manual Gazebo SITL.

---

## Improvement #48 — Consolidate gazebo.json into gazebo_sitl.json (Issue #172)

**Date:** 2026-03-17
**Category:** Refactor / Config / Documentation
**Issue:** [#172](https://github.com/nmohamaya/companion_software_stack/issues/172)

**What:** Eliminated config drift between manual and automated Gazebo SITL runs by deleting `config/gazebo.json` and consolidating into `config/gazebo_sitl.json`. Added missing camera intrinsics (`fx=277, fy=277, cx=320, cy=240`), `min_contour_area`, YOLOv8 `model_path`/`input_size` comment, and other detector params to `gazebo_sitl.json`. Updated all scripts (`launch_gazebo.sh`, `clean_build_and_run.sh`, `test_gazebo_integration.sh`) and ~20 doc references across 9 files.

**Why:** Maintaining two near-identical Gazebo config files caused two separate bugs from config drift (Fix #25 and Fix #41 — both stale keys in one file that had been updated in the other). A single source of truth eliminates this class of bug entirely. The missing camera intrinsics in `gazebo_sitl.json` meant the fusion engine was using wrong defaults (`fx=500, cx=960`) instead of the correct values for the 640×480 Gazebo camera.

**Files Deleted (1):**
- `config/gazebo.json`

**Files Modified (11):**
- `config/gazebo_sitl.json` — added camera intrinsics, min_contour_area, model_path, input_size
- `deploy/launch_gazebo.sh` — default config `gazebo.json` → `gazebo_sitl.json`
- `deploy/clean_build_and_run.sh` — same
- `tests/test_gazebo_integration.sh` — same
- `docs/sim_debugging_workflow.md` — updated references, replaced config diff with single-file validation
- `docs/Gazebo_sim_run.md` — updated 7 references
- `docs/INSTALL.md` — updated launch reference
- `docs/SIMULATION_ARCHITECTURE.md` — removed gazebo.json row from config table
- `docs/perception_design.md` — removed gazebo.json from deployment profiles
- `docs/BUG_FIXES.md` — added consolidation note to Fix #41 prevention
- `README.md` — removed gazebo.json from directory tree
- `deploy/DEPLOY_USAGE.md` — updated config references

**Test impact:** No new tests. 927/927 pass. Zero warnings.

---

_Last updated after Improvement #48 (config consolidation, Issue #172). See [tests/TESTS.md](../tests/TESTS.md) for current test counts. 927 tests, 54 test suites, 97 scenario checks (8 Tier 1 + 1 Tier 2), 6 CI jobs._
