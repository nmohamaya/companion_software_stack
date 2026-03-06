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
  - [Phase 9 — Process \& Thread Watchdog (Epic #88)](#phase-9--process--thread-watchdog-epic-88)
  - [Phase 10 — Foundation Hardening \& systemd (Epic #64)](#phase-10--foundation-hardening--systemd-epic-64)
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
| Unit tests | See [tests/TESTS.md](../tests/TESTS.md) for current counts |
| Compiler warnings | **0** (`-Werror -Wall -Wextra`, `[[nodiscard]]` enforced) |
| HAL interfaces | 5 (ICamera, IFCLink, IGCSLink, IGimbal, IIMUSource) |
| HAL backends | 8 (5 simulated + GazeboCam + GazeboIMU + MavlinkFCLink) |
| Perception backends | 3 (simulated, color_contour, YOLOv8-nano via OpenCV DNN) |
| Simulation | Full closed-loop Gazebo Harmonic + PX4 SITL |
| Autonomous flight | ARM → Takeoff → Navigate 3 WPs → RTL → Land → Disarm |
| CI | GitHub Actions — 9-job pipeline: format gate + 7-leg build matrix (shm/zenoh × sanitizers) + coverage ([docs/CI_SETUP.md](docs/CI_SETUP.md)) |
| Line coverage | **75.1%** (lcov, SHM backend) |
| Code style | `.clang-format` enforced via CI format gate (clang-format-18) |
| Config tunables | 95+ (JSON, dot-path access, schema-validated) |
| Error handling | `Result<T,E>` monadic type — no exceptions |
| Sanitizers | ASan, TSan, UBSan (all CI-enforced) |
| `[[nodiscard]]` | 26 headers annotated, compiler-enforced |
| OpenCV | 4.10.0 from source (core + imgproc + dnn) |
| MAVSDK | 2.12.12 |
| Target hardware | **NVIDIA Jetson Orin** (Nano/NX/AGX, aarch64, JetPack 6.x) |
| IPC framework | **POSIX SHM + Zenoh 1.7.2** — All 6 phases complete ([ADR-001](docs/adr/ADR-001-ipc-framework-selection.md), PRs #52–#57) |
| E2E testing | **42/42** Zenoh smoke-test checks passing (`tests/test_zenoh_e2e.sh`) |
| Process supervision | **systemd** service units (7 services + target) with `BindsTo` dependencies |
| Thread watchdog | **ThreadHeartbeat + ThreadWatchdog** — per-thread stuck detection via atomics |
| Process watchdog | **ProcessManager** — fork+exec, dependency graph, exponential backoff |
| Observability | JSON logging + IPC latency histograms + cross-process correlation IDs |

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

### Phase 9 — Process & Thread Watchdog (Epic #88)

**Key Deliverables:**

**Three-layer watchdog architecture** ([ADR-004](docs/adr/ADR-004-process-thread-watchdog-architecture.md)):

| Layer | Component | What It Does | PR(s) |
|-------|-----------|-------------|-------|
| **Layer 1** | `ThreadHeartbeat` + `ThreadWatchdog` | Per-thread atomic heartbeats; watchdog scans for stuck threads (configurable timeout) | #94 |
| **Layer 2** | `ProcessManager` (in P7) | fork+exec child processes, PID management, `ShmThreadHealth` publisher | #96, #100, #101, #102 |
| **Layer 2** | `RestartPolicy` + `ProcessGraph` | Exponential backoff, max retries, cooldown; dependency-aware restart cascades | #101, #102 |
| **Layer 3** | systemd service units | 7 `.service` files + `drone-stack.target`; `BindsTo` dependencies; `sd_notify` watchdog | #107 |

**Key files:**
- `common/util/include/util/thread_heartbeat.h` — `ThreadHeartbeat`, `ThreadHeartbeatRegistry`, `ThreadSnapshot`
- `common/util/include/util/thread_watchdog.h` — scans all heartbeats, reports stuck threads
- `common/util/include/util/thread_health_publisher.h` — publishes `ShmThreadHealth` to P7
- `monitor/process_manager.h` — fork+exec supervisor with PID tracking
- `util/restart_policy.h` — per-process restart policy (backoff, max retries)
- `util/process_graph.h` — dependency graph (e.g., perception depends on video_capture)
- `common/util/include/util/sd_notify.h` — thin `sd_notify()` wrapper (no-op when not under systemd)
- `deploy/systemd/drone-*.service` — 7 service units + `drone-stack.target`

**Design decisions:**
1. Supervisor lives in P7 — co-located with health metrics
2. Backend-independent — heartbeats use atomics (no Zenoh), PID polling works in SHM-only mode
3. Lightweight — one `atomic_store(relaxed)` per loop (~1 ns on ARM)
4. systemd uses `BindsTo=` (not `Requires=`) so dependents auto-restart when dependencies restart

**Issues:** #88 (epic), #89, #90, #91, #92, #83  
**PRs:** #94, #96, #100, #101, #102, #107  
**Tech debt:** #97 (snapshot vector→array), #98 (P7 div-by-zero + strncpy) → PR #108  
**Tests:** 464 → 701  

---

### Phase 10 — Foundation Hardening & systemd (Epic #64)

> Tiers 1–3 of Epic #64 were completed as a batch before the watchdog epic.
> Phase numbering here reflects chronological order of completion.

**Tier 1 — Build with Confidence:**
- Sanitizers (ASan/TSan/UBSan) + 7-leg CI matrix (PR #71)
- clang-tidy + clang-format + .editorconfig (PR #72)
- Code coverage (gcov/lcov, 75.1% line coverage) (PR #73)

**Tier 2 — Error Handling & Safety:**
- `Result<T,E>` monadic error type (PR #75)
- Config schema validation at startup — 7 process schemas (PR #76)
- `[[nodiscard]]` audit — 26 headers (PR #77)

**Tier 3 — Observability & Debugging:**
- IPC latency histograms (PR #78)
- Structured JSON logging (PR #79)
- Cross-process correlation IDs (PR #80)

**Tier 4 — Deployment Readiness (partial):**
- systemd service units (PR #107) ✅
- Dockerfile (#81) — deferred to closer to deployment
- Cross-compilation (#82) — deferred, depends on #81

**Additional:**
- Deploy scripts + modular test runners (PR #104)
- Local CI script `run_ci_local.sh` added to workflow

**Issues:** #64 (epic), #65–#70, #78–#80, #83, #103  
**Tests:** 400 → 701

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
| ~~[#31](https://github.com/nmohamaya/companion_software_stack/issues/31)~~ | ~~systemd service files + process supervisor~~ | ~~P1~~ | ✅ Done — superseded by #83 (PR #107) + Epic #88 (watchdog) |

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
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | P0 | 🟡 **Partial** — FaultManager ([#61](https://github.com/nmohamaya/companion_software_stack/issues/61), PR #63) handles 8 fault conditions with graduated response. Remaining: geofencing, motor failure, SLAM divergence detection |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | P2 | `IGimbal` backend for SIYI A8 mini (UART) or PWM servo; stabilisation loop |
| ~~[#45](https://github.com/nmohamaya/companion_software_stack/issues/45)~~ | ~~**[Epic] Zenoh IPC Migration**~~ | ~~P1~~ | ✅ **Complete** — All 6 phases done (PRs #52–#57), Epic closed |
| ~~[#46](https://github.com/nmohamaya/companion_software_stack/issues/46)~~ | ~~Zenoh Phase A — Foundation~~ | ~~P0~~ | ✅ Done (PR #52) — CMake, ZenohMessageBus, security options, 33 tests, CI dual-build |
| ~~[#47](https://github.com/nmohamaya/companion_software_stack/issues/47)~~ | ~~Zenoh Phase B — Low-bandwidth channels~~ | ~~P1~~ | ✅ Done (PR #53) — 10 control/status channels migrated, all 7 processes on factory |
| ~~[#48](https://github.com/nmohamaya/companion_software_stack/issues/48)~~ | ~~Zenoh Phase C — High-bandwidth video~~ | ~~P1~~ | ✅ Done (PR #54) — Zero-copy SHM video publishing via PosixShmProvider |
| ~~[#49](https://github.com/nmohamaya/companion_software_stack/issues/49)~~ | ~~Zenoh Phase D — Service channels + cleanup~~ | ~~P1~~ | ✅ Done (PR #55) — Zenoh queryable services, legacy `ShmServiceChannel` removed |
| ~~[#50](https://github.com/nmohamaya/companion_software_stack/issues/50)~~ | ~~Zenoh Phase E — Network transport~~ | ~~P1~~ | ✅ Done (PR #56) — Drone↔GCS network transport, wire format, GCS client tool |
| ~~[#51](https://github.com/nmohamaya/companion_software_stack/issues/51)~~ | ~~Zenoh Phase F — Liveliness tokens~~ | ~~P1~~ | ✅ Done (PR #57) — Process health via liveliness tokens, P7 death detection |

**Exit Criteria:** Repeated outdoor missions on Jetson Orin with full telemetry logging; graceful degradation on sensor failures.

**Zenoh IPC Migration:** ✅ **Complete** — [Epic #45](https://github.com/nmohamaya/companion_software_stack/issues/45) closed. All 6 phases delivered (PRs #52–#57). E2E smoke test validates all 7 processes running on Zenoh (PR #58).

---

## Issue Tracking

### Epics

| # | Title | State |
|---|-------|-------|
| [#25](https://github.com/nmohamaya/companion_software_stack/issues/25) | [Epic] Real Drone Deployment — From Simulation to Flight | Open |
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | [Epic] Zenoh IPC Migration — From POSIX SHM to Zero-Copy Network-Transparent IPC | **Closed** ✅ |
| [#64](https://github.com/nmohamaya/companion_software_stack/issues/64) | [Epic] Foundation Hardening — CI, Error Handling, Code Quality | **Closed** ✅ |
| [#88](https://github.com/nmohamaya/companion_software_stack/issues/88) | [Epic] Process & Thread Watchdog — Crash Recovery & Stuck-Thread Detection | **Closed** ✅ |

### Foundation Hardening (Epic #64) — ✅ COMPLETE

| # | Title | Tier | State |
|---|-------|------|-------|
| [#65](https://github.com/nmohamaya/companion_software_stack/issues/65) | clang-tidy + clang-format | Tier 1 | **Closed** (PR #72) |
| [#66](https://github.com/nmohamaya/companion_software_stack/issues/66) | Code coverage reporting | Tier 1 | **Closed** (PR #73) |
| [#67](https://github.com/nmohamaya/companion_software_stack/issues/67) | Sanitizer support (ASan/TSan/UBSan) | Tier 1 | **Closed** (PR #71) |
| [#68](https://github.com/nmohamaya/companion_software_stack/issues/68) | Result<T,E> error type | Tier 2 | **Closed** (PR #75) |
| [#69](https://github.com/nmohamaya/companion_software_stack/issues/69) | Config schema validation | Tier 2 | **Closed** (PR #76) |
| [#70](https://github.com/nmohamaya/companion_software_stack/issues/70) | [[nodiscard]] audit | Tier 2 | **Closed** (PR #77) |
| [#78](https://github.com/nmohamaya/companion_software_stack/issues/78) | IPC latency histograms | Tier 3 | **Closed** |
| [#79](https://github.com/nmohamaya/companion_software_stack/issues/79) | Structured JSON logging | Tier 3 | **Closed** |
| [#80](https://github.com/nmohamaya/companion_software_stack/issues/80) | Cross-process correlation IDs | Tier 3 | **Closed** |
| [#83](https://github.com/nmohamaya/companion_software_stack/issues/83) | systemd service units | Tier 4 | **Closed** (PR #107) |
| [#81](https://github.com/nmohamaya/companion_software_stack/issues/81) | Dockerfile for reproducible builds | Tier 4 | Open (deferred) |
| [#82](https://github.com/nmohamaya/companion_software_stack/issues/82) | Cross-compilation toolchain | Tier 4 | Open (deferred) |

### Process & Thread Watchdog (Epic #88) — ✅ COMPLETE

| # | Title | Phase | State |
|---|-------|-------|-------|
| [#89](https://github.com/nmohamaya/companion_software_stack/issues/89) | ThreadHeartbeat + ThreadWatchdog | Phase 1 | **Closed** (PR #94) |
| [#90](https://github.com/nmohamaya/companion_software_stack/issues/90) | ShmThreadHealth integration | Phase 2 | **Closed** (PR #96) |
| [#91](https://github.com/nmohamaya/companion_software_stack/issues/91) | Process supervisor (fork+exec) | Phase 3 | **Closed** (PR #100) |
| [#92](https://github.com/nmohamaya/companion_software_stack/issues/92) | Restart policies + dependency graph | Phase 4 | **Closed** (PRs #101, #102) |
| [#97](https://github.com/nmohamaya/companion_software_stack/issues/97) | Tech debt: snapshot() vector→array | — | **Closed** (PR #108) |
| [#98](https://github.com/nmohamaya/companion_software_stack/issues/98) | Tech debt: P7 div-by-zero + strncpy | — | **Closed** (PR #108) |
| [#103](https://github.com/nmohamaya/companion_software_stack/issues/103) | Deploy scripts + test runners | — | **Closed** (PR #104) |

### Phase 9 — First Safe Flight

| # | Title | State |
|---|-------|-------|
| ~~[#26](https://github.com/nmohamaya/companion_software_stack/issues/26)~~ | ~~Hardware config + launch script~~ | **Closed** (PR #43) |
| [#27](https://github.com/nmohamaya/companion_software_stack/issues/27) | Battery-critical auto-RTL + temperature failsafe | Open |
| [#28](https://github.com/nmohamaya/companion_software_stack/issues/28) | FC heartbeat timeout + link-loss contingency | Open |
| [#29](https://github.com/nmohamaya/companion_software_stack/issues/29) | Geofencing (polygon + altitude ceiling) | Open |
| [#30](https://github.com/nmohamaya/companion_software_stack/issues/30) | Pre-flight check script | Open |
| ~~[#31](https://github.com/nmohamaya/companion_software_stack/issues/31)~~ | ~~systemd service files + process supervisor~~ | **Closed** — superseded by #83 + Epic #88 |

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
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | **Partial** ([#61](https://github.com/nmohamaya/companion_software_stack/issues/61) PR #63) |
| [#61](https://github.com/nmohamaya/companion_software_stack/issues/61) | FaultManager — graceful degradation | **Closed** (PR #63) |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | Open |

### Zenoh IPC Migration (Phase 12) — ✅ COMPLETE

| # | Title | State |
|---|-------|-------|
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | [Epic] Zenoh IPC Migration | **Closed** ✅ |
| [#46](https://github.com/nmohamaya/companion_software_stack/issues/46) | Phase A — Foundation (CMake, ZenohMessageBus, CI) | **Closed** (PR #52) |
| [#47](https://github.com/nmohamaya/companion_software_stack/issues/47) | Phase B — Low-bandwidth channel migration | **Closed** (PR #53) |
| [#48](https://github.com/nmohamaya/companion_software_stack/issues/48) | Phase C — High-bandwidth video migration (zero-copy) | **Closed** (PR #54) |
| [#49](https://github.com/nmohamaya/companion_software_stack/issues/49) | Phase D — Service channel migration + SHM removal | **Closed** (PR #55) |
| [#50](https://github.com/nmohamaya/companion_software_stack/issues/50) | Phase E — Network transport (drone↔GCS) | **Closed** (PR #56) |
| [#51](https://github.com/nmohamaya/companion_software_stack/issues/51) | Phase F — Liveliness tokens (process health) | **Closed** (PR #57) |

---

## Metrics History

| Metric | Phase 1 | Phase 3 | Phase 6 | Phase 7 | Phase 8 | Phase 9 | Zenoh A | Zenoh B | Zenoh C | Zenoh D | Zenoh E | Zenoh F | E2E | FaultMgr | Hardening | **Watchdog + systemd (Current)** |
|--------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|-----|--------|-------|-------|
| Unit tests | 58 | 121 | 196 | 262 | 262 | 262 | 295 | 308 | 329 | 348 | 359 | 370 | 377 | 400 | 464 | **701** |
| Test suites | 6 | 10 | 14 | 18 | 18 | 18 | 19 | 19 | 19 | 19 | 20 | 21 | 22 | 23 | 26 | **31+** |
| Bug fixes | 6 | 6 | 13 | 13 | 15 | 15 | 17 | 17 | 17 | 17 | 17 | 17 | 19 | 19 | 21 | **21** |
| Config tunables | 45+ | 45+ | 70+ | 75+ | 75+ | 80+ | 80+ | 80+ | 85+ | 85+ | 90+ | 90+ | 90+ | 95+ | 95+ | **95+** |
| HAL backends | 0 | 5 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | **8** |
| IPC backends | SHM | SHM | SHM | SHM | SHM | SHM | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | **SHM + Zenoh** |
| Perception backends | 0 | 0 | 1 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | **3** |
| Compiler warnings | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **0** |
| Processes on factory | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 2/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | **7/7** |
| Processes w/ real Gazebo data | 0/7 | 0/7 | 4/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | **5/7** |
| Zenoh channels migrated | — | — | — | — | — | — | 0/12 | 10/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | **12/12** |
| Liveliness tokens | — | — | — | — | — | — | — | — | — | — | — | 7 | 7 | 7 | 7 | **7** |
| Network transport | — | — | — | — | — | — | — | — | — | — | Yes | Yes | Yes | Yes | Yes | **Yes** |
| E2E checks | — | — | — | — | — | — | — | — | — | — | — | — | 42/42 | 42/42 | 42/42 | **42/42** |
| CI matrix legs | 1 | 1 | 1 | 1 | 1 | 1 | 2 | 2 | 2 | 2 | 2 | 2 | 2 | 2 | 9 | **9** |
| Fault conditions | — | — | — | — | — | — | — | — | — | — | — | — | — | 8 | 8 | **8** |
| Sanitizers | — | — | — | — | — | — | — | — | — | — | — | — | — | — | ASan+TSan+UBSan | **ASan+TSan+UBSan** |
| `[[nodiscard]]` headers | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 26 | **26** |
| Config schemas | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 7 | **7** |
| Error handling | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | Result<T,E> | **Result<T,E>** |
| Line coverage | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 75.1% | **75.1%** |
| Code style | — | — | — | — | — | — | — | — | — | — | — | — | — | — | enforced | **enforced** |
| Thread watchdog | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | **ThreadHeartbeat + ThreadWatchdog** |
| Process supervision | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | **systemd + ProcessManager** |

### Process Activity During Simulation

| # | Process | Backend | Real Gazebo Data | Activity |
|---|---------|---------|------------------|----------|
| 1 | Video Capture | Gazebo camera | Yes — rendered frames at 30 Hz | High |
| 2 | Perception | YOLOv8-nano (OpenCV DNN) | Yes — real object detection | High |
| 3 | SLAM/VIO/Nav | Gazebo odometry + IMU | Yes — ground-truth pose | High |
| 4 | Mission Planner | Pure logic + FaultManager | Consumes real pose + system health | High — orchestrates flight + fault response |
| 5 | Comms | MAVLink (MAVSDK) | Yes — real PX4 link | High — controls PX4 |
| 6 | Payload Manager | Simulated gimbal | No | Low |
| 7 | System Monitor | Linux /proc | Host metrics only | Low (1 Hz) |

---

*Last updated after Process & Thread Watchdog (Epic #88) + systemd (Issue #83) — see [tests/TESTS.md](../tests/TESTS.md) for current test counts. 9-job CI pipeline, three-layer watchdog, systemd service units with BindsTo dependencies.*
