# Roadmap — Drone Companion Software Stack

> From first line of code to real drone deployment.  
> This document tracks **completed milestones** and **planned phases** in a single place.

---

## Table of Contents

- [Roadmap — Drone Companion Software Stack](#roadmap--drone-companion-software-stack)
  - [Table of Contents](#table-of-contents)
  - [Current State at a Glance](#current-state-at-a-glance)
  - [Completed Phases](#completed-phases)
    - [Phase 1 — Foundation (Bug Fixes, Testing, Config System)](#phase-1--foundation-bug-fixes-testing-config-system)
    - [Phase 2 — Near-Term Improvements](#phase-2--near-term-improvements)
    - [Phase 3 — Hardware Abstraction Layer (HAL)](#phase-3--hardware-abstraction-layer-hal)
    - [Phase 4 — Gazebo SITL Integration](#phase-4--gazebo-sitl-integration)
    - [Phase 5 — End-to-End Flight Fixes](#phase-5--end-to-end-flight-fixes)
    - [Phase 6 — Simulation Visualization \& Flight Tuning](#phase-6--simulation-visualization--flight-tuning)
    - [Phase 7 — Real Perception Pipeline](#phase-7--real-perception-pipeline)
    - [Phase 8 — Deployment Tooling \& RTL Safety](#phase-8--deployment-tooling--rtl-safety)
  - [Planned Phases (Real Drone Deployment)](#planned-phases-real-drone-deployment)
    - [Phase 9 — First Safe Flight (Safety + Hardware Config)](#phase-9--first-safe-flight-safety--hardware-config)
    - [Phase 10 — Real Cameras \& Perception](#phase-10--real-cameras--perception)
    - [Phase 11 — Autonomous Navigation (Real VIO/SLAM)](#phase-11--autonomous-navigation-real-vioslam)
    - [Phase 12 — Production Hardening](#phase-12--production-hardening)
  - [Issue Tracking](#issue-tracking)
    - [Epics](#epics)
    - [Phase 9 — First Safe Flight](#phase-9--first-safe-flight)
    - [Phase 10 — Real Cameras \& Perception](#phase-10--real-cameras--perception-1)
    - [Phase 11 — Autonomous Navigation](#phase-11--autonomous-navigation)
    - [Phase 12 — Production Hardening](#phase-12--production-hardening-1)
    - [Zenoh IPC Migration (Phase 12)](#zenoh-ipc-migration-phase-12)
  - [Metrics History](#metrics-history)
    - [Process Activity During Simulation](#process-activity-during-simulation)

---

## Current State at a Glance

| Metric | Value |
|--------|-------|
| Unit tests | **308** (19 suites, 0 failures) |
| Compiler warnings | **0** (`-Werror -Wall -Wextra`) |
| HAL interfaces | 5 (ICamera, IFCLink, IGCSLink, IGimbal, IIMUSource) |
| HAL backends | 8 (5 simulated + GazeboCam + GazeboIMU + MavlinkFCLink) |
| Perception backends | 3 (simulated, color_contour, YOLOv8-nano via OpenCV DNN) |
| Simulation | Full closed-loop Gazebo Harmonic + PX4 SITL |
| Autonomous flight | ARM → Takeoff → Navigate 3 WPs → RTL → Land → Disarm |
| CI | GitHub Actions — 2-leg matrix `{shm, zenoh}` (295 tests on every push/PR) |
| Config tunables | 80+ (JSON, dot-path access) |
| OpenCV | 4.10.0 from source (core + imgproc + dnn) |
| MAVSDK | 2.12.12 |
| Target hardware | **NVIDIA Jetson Orin** (Nano/NX/AGX, aarch64, JetPack 6.x) |
| IPC framework | **POSIX SHM (default) + Zenoh 1.7.2** — Phase A+B done ([ADR-001](docs/adr/ADR-001-ipc-framework-selection.md), PRs #52, #53) |

---

## Completed Phases

### Phase 1 — Foundation (Bug Fixes, Testing, Config System)

**Key Deliverables:**
- **Google Test framework** — 58 initial tests across 6 suites covering SHM IPC, SPSC ring, Kalman tracker, mission FSM, fusion engine, config
- **JSON config system** — `drone::Config` class with dot-path navigation (`cfg.get<T>("video_capture.mission_cam.width", 640)`), backed by nlohmann/json
- **6 critical bug fixes** — use-after-free in SLAM (raw `new/delete` + `atomic<Pose*>`), broken ShmWriter/ShmReader move constructors, uninitialized fields in HungarianSolver/MultiObjectTracker/LiDARCluster

**PRs:** Initial commits  
**Tests:** 0 → 58

---

### Phase 2 — Near-Term Improvements

**Key Deliverables:**
- **Config wired into all 7 processes** — every hardcoded magic number replaced with `cfg.get<>()` calls; operators tune the stack via a single JSON file
- **Expanded test coverage** — 3 new suites: comms (13), payload_manager (9), system_monitor (11)
- **CI pipeline** — GitHub Actions on push/PR: install deps → CMake Release `-Werror` → build → ctest

**Tests:** 58 → 91

---

### Phase 3 — Hardware Abstraction Layer (HAL)

**Key Deliverables:**
- **5 HAL interfaces** — `ICamera`, `IFCLink`, `IGCSLink`, `IGimbal`, `IIMUSource`
- **5 simulated backends** — `SimulatedCamera`, `SimulatedFCLink`, `SimulatedGCSLink`, `SimulatedGimbal`, `SimulatedIMU`
- **Config-driven factory** — `"backend": "simulated"` in JSON selects backend; processes P1, P3, P5, P6 use HAL via factory
- **30 HAL tests** — factory creation, interface contracts, simulated behaviour

**Issue:** #2  
**Tests:** 91 → 121

---

### Phase 4 — Gazebo SITL Integration

**Key Deliverables:**
- **MavlinkFCLink** — MAVSDK-based `IFCLink` implementation: arm/disarm, takeoff/land/RTL, offboard velocity, telemetry (10 tests)
- **GazeboCameraBackend** — gz-transport subscriber receiving live rendered frames from Gazebo (5 tests)
- **GazeboIMUBackend** — gz-transport subscriber for accelerometer + gyroscope data (5 tests)
- **Closed-loop integration** — Custom x500 drone model with 3 companion sensors, test world with obstacles, one-command launch script
- **25 IPC interface tests** — SHM message types for all inter-process links

**Issues:** #7–#11  
**PRs:** #12–#16  
**Tests:** 121 → 196

---

### Phase 5 — End-to-End Flight Fixes

**Key Deliverables:**
- **7 flight-critical bug fixes:**
  1. MAVSDK `ComponentType::Autopilot` → `GroundStation`
  2. Pre-arm health polling loop
  3. Post-arm mode stabilisation delay
  4. Pre-send zero velocity before offboard mode
  5. Local-frame NED velocity commands
  6. Process startup ordering + MAVLink port wait
  7. Gazebo plugin env vars

**Result:** First successful autonomous flight — ARM → Takeoff → Navigate 3 WPs → RTL → Land → Disarm

**Issue:** #17  
**PR:** #18

---

### Phase 6 — Simulation Visualization & Flight Tuning

**Key Deliverables:**
- **Gazebo GUI** — Chase-cam follow with CameraTracking plugin (6 m behind, 3 m above)
- **GazeboVisualFrontend** — Ground-truth pose via gz-transport odometry, replacing fake circular orbit
- **Flight plan tuning** — Waypoints 15 m apart at 5 m AGL, 5 m/s cruise, speed ramp in last 2 m, LAND → RTL
- **Documentation** — `docs/Gazebo_sim_run.md` (14 sections), `docs/sim_run_commands.md`, `deploy/clean_build_and_run.sh`

**Flight Profile:** ARM (1 s) → Takeoff (13 s) → WP1 (6 s) → WP2 (6 s) → WP3 (9 s) → RTL+Land (14 s) ≈ 50 s

---

### Phase 7 — Real Perception Pipeline

**Key Deliverables:**

**ColorContourDetector** (42 tests):
- Pure C++ (no OpenCV) — RGB→HSV, binary mask, Union-Find CCL, bbox extraction, confidence scoring
- 6 color→class mappings (Red→PERSON, Blue→VEHICLE_CAR, etc.)
- Config-driven color ranges, min area, max detections

**OpenCvYoloDetector** (24 tests):
- YOLOv8-nano (6.2M params, 12.8 MB ONNX, 80 COCO classes)
- OpenCV 4.10.0 DNN `blobFromImage` → `net.forward()` → NMS
- Conditional compilation via `HAS_OPENCV` — graceful fallback
- COCO→ObjectClass mapping (person, car, truck, airplane→drone, animals, etc.)

**Three detector backends coexist** — `"yolov8"`, `"color_contour"`, `"simulated"` — selected via config

**Issue:** #19  
**PR:** #20  
**Tests:** 196 → 262

---

### Phase 8 — Deployment Tooling & RTL Safety

**Key Deliverables:**

**Automated Dependency Installer** (PR #21):
- `deploy/install_dependencies.sh` — one-command setup for Ubuntu (apt deps, Eigen, nlohmann-json, spdlog, GTest, OpenCV 4.10, MAVSDK 2.12.12, PX4, Gazebo)
- 3 bug fixes found during fresh-machine testing: missing `drone_hal` link for P7 monitor, ShmWriter `NO_DISCARD` attribute typo, ShmReader `msg` shadowing
- 6 review comment fixes from PR review

**Development Workflow** (README.md):
- Branch naming: `feature/issue-N-description`, `fix/issue-N-description`
- Commit format: `feat(#N):`, `fix(#N):`, `docs(#N):`
- PR checklist, post-merge lessons, bug fix workflow

**RTL Landing Fix** (PR #23):
- `std::isfinite()` home-position gating (replaced `!= 0.0` check)
- `home_recorded` guard before RTL waypoint use
- RTL dwell time enforcement (`rtl_min_dwell_seconds` = 5 s default)
- Tightened `rtl_acceptance_radius_m` from 3.0 → 1.5 m
- Simulation verified: drone returns to home (0.0 m distance, 52 s in RTL) then lands

**PRs:** #21, #23  
**Issues closed:** #22, #24

---

## Planned Phases (Real Drone Deployment)

> **Epic:** [#25 — Real Drone Deployment — From Simulation to Flight](https://github.com/nmohamaya/companion_software_stack/issues/25)

### Phase 9 — First Safe Flight (Safety + Hardware Config)

> **Goal:** Fly a waypoint mission on a real flight controller with proper safety guardrails.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| ~~[#26](https://github.com/nmohamaya/companion_software_stack/issues/26)~~ | ~~Hardware config + launch script~~ | ~~P0~~ | ✅ Done (PR #43) — `config/hardware.json` + `deploy/launch_hardware.sh` |
| [#27](https://github.com/nmohamaya/companion_software_stack/issues/27) | Battery-critical auto-RTL + temp failsafe | P0 | Voltage/percentage thresholds trigger RTL; SoC thermal throttle monitoring |
| [#28](https://github.com/nmohamaya/companion_software_stack/issues/28) | FC heartbeat timeout + link-loss contingency | P0 | Detect MAVLink heartbeat loss → hold / RTL after timeout; reconnection logic |
| [#29](https://github.com/nmohamaya/companion_software_stack/issues/29) | Geofencing (polygon + altitude ceiling) | P1 | Config-defined polygon + altitude ceiling; auto-RTL on breach |
| [#30](https://github.com/nmohamaya/companion_software_stack/issues/30) | Pre-flight check script | P0 | Verify FC link, GPS fix, battery level, disk space, process health before ARM |
| [#31](https://github.com/nmohamaya/companion_software_stack/issues/31) | systemd service files + process supervisor | P1 | Auto-start on boot, auto-restart on crash, ordered dependencies, journal logging |

**Exit Criteria:** Safe tethered flight with real FC; manual override tested; battery RTL tested.

---

### Phase 10 — Real Cameras & Perception

> **Goal:** Real camera feed → real object detection → telemetry to ground station.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| [#32](https://github.com/nmohamaya/companion_software_stack/issues/32) | V4L2Camera HAL backend | P0 | `ICamera` implementation using V4L2 (`/dev/video*`); MJPEG/YUYV capture, zero-copy mmap |
| [#33](https://github.com/nmohamaya/companion_software_stack/issues/33) | TensorRT YOLOv8 detector backend | P1 | ONNX → TensorRT engine; FP16 inference on Jetson; `IDetector` backend |
| [#34](https://github.com/nmohamaya/companion_software_stack/issues/34) | UDPGCSLink — ground station telemetry | P1 | `IGCSLink` implementation with UDP socket; telemetry out + command in; protobuf framing |
| [#35](https://github.com/nmohamaya/companion_software_stack/issues/35) | Video streaming (H.265 → RTP/RTSP) | P2 | Hardware H.265 encode on Jetson; GStreamer RTP/RTSP pipeline; bitrate adaptation |
| [#36](https://github.com/nmohamaya/companion_software_stack/issues/36) | Cross-compilation / Jetson native build | P1 | Native aarch64 build on **NVIDIA Jetson Orin** (JetPack 6.x); CMake cross-compile toolchain |

**Target Hardware:** NVIDIA Jetson Orin (Nano 8 GB / NX 16 GB / AGX 32–64 GB), JetPack 6.x, CUDA 12.x

**Exit Criteria:** Real camera producing detected objects on Jetson Orin; ground station receives telemetry.

---

### Phase 11 — Autonomous Navigation (Real VIO/SLAM)

> **Goal:** GPS-denied navigation using onboard vision.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| [#37](https://github.com/nmohamaya/companion_software_stack/issues/37) | Real VIO backend (ORB-SLAM3 / VINS-Fusion) | P0 | Integrate ORB-SLAM3 or VINS-Fusion as `IVisualFrontend` backend; stereo or mono+IMU |
| [#38](https://github.com/nmohamaya/companion_software_stack/issues/38) | Stereo camera calibration pipeline | P1 | Checkerboard calibration tool; output intrinsics + extrinsics to config JSON |
| [#39](https://github.com/nmohamaya/companion_software_stack/issues/39) | VIO/GPS fusion via PX4 external vision | P1 | Publish VIO pose to PX4 via MAVLink `VISION_POSITION_ESTIMATE`; EKF2 fusion |

**Exit Criteria:** Indoor GPS-denied flight using VIO pose estimation.

---

### Phase 12 — Production Hardening

> **Goal:** Reliable enough for repeated real-world missions.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| [#40](https://github.com/nmohamaya/companion_software_stack/issues/40) | Flight data recorder + replay | P1 | Binary ring-buffer logger; all SHM channels + telemetry; offline replay tool |
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | P0 | Comm-loss, GPS-loss, SLAM divergence, motor failure detection → safe action matrix |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | P2 | `IGimbal` backend for SIYI A8 mini (UART) or PWM servo; stabilisation loop |
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | **[Epic] Zenoh IPC Migration** | P1 | Replace POSIX SHM with Zenoh zero-copy SHM + network transport ([ADR-001](docs/adr/ADR-001-ipc-framework-selection.md)) |
| ~~[#46](https://github.com/nmohamaya/companion_software_stack/issues/46)~~ | ~~Zenoh Phase A — Foundation~~ | ~~P0~~ | ✅ Done (PR #52) — CMake, ZenohMessageBus, security options, 33 tests, CI dual-build |
| [#47](https://github.com/nmohamaya/companion_software_stack/issues/47) | Zenoh Phase B — Low-bandwidth channels | P1 | Migrate 10 control/status channels to Zenoh pub/sub |
| [#48](https://github.com/nmohamaya/companion_software_stack/issues/48) | Zenoh Phase C — High-bandwidth video | P1 | Migrate video frames with Zenoh SHM provider (zero-copy) |
| [#49](https://github.com/nmohamaya/companion_software_stack/issues/49) | Zenoh Phase D — Service channels + cleanup | P1 | Replace `ShmServiceChannel` with Zenoh queryable; remove old SHM primitives |
| [#50](https://github.com/nmohamaya/companion_software_stack/issues/50) | Zenoh Phase E — Network transport | P1 | Enable drone↔GCS communication over same pub/sub API |
| [#51](https://github.com/nmohamaya/companion_software_stack/issues/51) | Zenoh Phase F — Liveliness tokens | P1 | Process health monitoring via Zenoh liveliness tokens |

**Exit Criteria:** Repeated outdoor missions on Jetson Orin with full telemetry logging; graceful degradation on sensor failures; Zenoh-based IPC with drone↔GCS network transport.

**Zenoh Migration Sub-Issues:** [Epic #45](https://github.com/nmohamaya/companion_software_stack/issues/45) — Phase A (#46) → B (#47) → C (#48) + D (#49) → E (#50), F (#51) in parallel.

---

## Issue Tracking

### Epics

| # | Title | State |
|---|-------|-------|
| [#25](https://github.com/nmohamaya/companion_software_stack/issues/25) | [Epic] Real Drone Deployment — From Simulation to Flight | Open |
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | [Epic] Zenoh IPC Migration — From POSIX SHM to Zero-Copy Network-Transparent IPC | Open |

### Phase 9 — First Safe Flight

| # | Title | State |
|---|-------|-------|
| [#26](https://github.com/nmohamaya/companion_software_stack/issues/26) | Hardware config + launch script | **Closed** (PR #43) |
| [#27](https://github.com/nmohamaya/companion_software_stack/issues/27) | Battery-critical auto-RTL + temperature failsafe | Open |
| [#28](https://github.com/nmohamaya/companion_software_stack/issues/28) | FC heartbeat timeout + link-loss contingency | Open |
| [#29](https://github.com/nmohamaya/companion_software_stack/issues/29) | Geofencing (polygon + altitude ceiling) | Open |
| [#30](https://github.com/nmohamaya/companion_software_stack/issues/30) | Pre-flight check script | Open |
| [#31](https://github.com/nmohamaya/companion_software_stack/issues/31) | systemd service files + process supervisor | Open |

### Phase 10 — Real Cameras & Perception

| # | Title | State |
|---|-------|-------|
| [#32](https://github.com/nmohamaya/companion_software_stack/issues/32) | V4L2Camera HAL backend | Open |
| [#33](https://github.com/nmohamaya/companion_software_stack/issues/33) | TensorRT YOLOv8 detector backend | Open |
| [#34](https://github.com/nmohamaya/companion_software_stack/issues/34) | UDPGCSLink — ground station telemetry | Open |
| [#35](https://github.com/nmohamaya/companion_software_stack/issues/35) | Video streaming (H.265 → RTP/RTSP) | Open |
| [#36](https://github.com/nmohamaya/companion_software_stack/issues/36) | Cross-compilation / Jetson native build | Open |

### Phase 11 — Autonomous Navigation

| # | Title | State |
|---|-------|-------|
| [#37](https://github.com/nmohamaya/companion_software_stack/issues/37) | Real VIO backend (ORB-SLAM3 / VINS-Fusion) | Open |
| [#38](https://github.com/nmohamaya/companion_software_stack/issues/38) | Stereo camera calibration pipeline | Open |
| [#39](https://github.com/nmohamaya/companion_software_stack/issues/39) | VIO/GPS fusion via PX4 external vision | Open |

### Phase 12 — Production Hardening

| # | Title | State |
|---|-------|-------|
| [#40](https://github.com/nmohamaya/companion_software_stack/issues/40) | Flight data recorder + replay | Open |
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | Open |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | Open |

### Zenoh IPC Migration (Phase 12)

| # | Title | State |
|---|-------|-------|
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | [Epic] Zenoh IPC Migration | Open |
| [#46](https://github.com/nmohamaya/companion_software_stack/issues/46) | Phase A — Foundation (CMake, ZenohMessageBus, CI) | **Closed** (PR #52) |
| [#47](https://github.com/nmohamaya/companion_software_stack/issues/47) | Phase B — Low-bandwidth channel migration | **In Review** (PR #53) |
| [#48](https://github.com/nmohamaya/companion_software_stack/issues/48) | Phase C — High-bandwidth video migration (zero-copy) | Open |
| [#49](https://github.com/nmohamaya/companion_software_stack/issues/49) | Phase D — Service channel migration + SHM removal | Open |
| [#50](https://github.com/nmohamaya/companion_software_stack/issues/50) | Phase E — Network transport (drone↔GCS) | Open |
| [#51](https://github.com/nmohamaya/companion_software_stack/issues/51) | Phase F — Liveliness tokens (process health) | Open |

---

## Metrics History

| Metric | Phase 1 | Phase 3 | Phase 6 | Phase 7 | Phase 8 | Phase 9 | Zenoh A | Zenoh B (Current) |
|--------|---------|---------|---------|---------|---------|---------|---------|-------------------|
| Unit tests | 58 | 121 | 196 | 262 | 262 | 262 | 295 | **308** |
| Test suites | 6 | 10 | 14 | 18 | 18 | 18 | 19 | **19** |
| Bug fixes | 6 | 6 | 13 | 13 | 15 | 15 | 17 | **17** |
| Config tunables | 45+ | 45+ | 70+ | 75+ | 75+ | 80+ | 80+ | **80+** |
| HAL backends | 0 | 5 | 8 | 8 | 8 | 8 | 8 | **8** |
| IPC backends | SHM | SHM | SHM | SHM | SHM | SHM | SHM + Zenoh | **SHM + Zenoh** |
| Perception backends | 0 | 0 | 1 | 3 | 3 | 3 | 3 | **3** |
| Compiler warnings | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **0** |
| Processes on factory | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 2/7 | **7/7** |
| Processes w/ real Gazebo data | 0/7 | 0/7 | 4/7 | 5/7 | 5/7 | 5/7 | 5/7 | **5/7** |
| OpenCV | — | — | — | 4.10.0 | 4.10.0 | 4.10.0 | 4.10.0 | **4.10.0** |
| MAVSDK | — | — | 2.12.12 | 2.12.12 | 2.12.12 | 2.12.12 | 2.12.12 | **2.12.12** |
| Autonomous flight | No | No | Yes | Yes | Yes | Yes | Yes | **Yes** |
| Hardware deploy | No | No | No | No | No | Yes | Yes | **Yes** |
| CI matrix legs | 1 | 1 | 1 | 1 | 1 | 1 | 2 | **2 (shm, zenoh)** |

### Process Activity During Simulation

| # | Process | Backend | Real Gazebo Data | Activity |
|---|---------|---------|------------------|----------|
| 1 | Video Capture | Gazebo camera | Yes — rendered frames at 30 Hz | High |
| 2 | Perception | YOLOv8-nano (OpenCV DNN) | Yes — real object detection | High |
| 3 | SLAM/VIO/Nav | Gazebo odometry + IMU | Yes — ground-truth pose | High |
| 4 | Mission Planner | Pure logic | Consumes real pose | High — orchestrates flight |
| 5 | Comms | MAVLink (MAVSDK) | Yes — real PX4 link | High — controls PX4 |
| 6 | Payload Manager | Simulated gimbal | No | Low |
| 7 | System Monitor | Linux /proc | Host metrics only | Low (1 Hz) |

---

*Last updated after Zenoh Phase B — PR #53 (#47 low-bandwidth migration), 308 tests, all 7 processes on MessageBusFactory.*
