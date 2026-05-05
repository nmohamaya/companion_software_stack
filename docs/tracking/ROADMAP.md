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
    - [Phase 11 — Core Autonomy \& Safety (Epic #110)](#phase-11--core-autonomy--safety-epic-110)
  - [Planned Phases (Real Drone Deployment)](#planned-phases-real-drone-deployment)
    - [Phase 9 — First Safe Flight (Safety + Hardware Config)](#phase-9--first-safe-flight-safety--hardware-config)
    - [Phase 10 — Real Cameras \& Perception](#phase-10--real-cameras--perception)
    - [Phase 11 — Autonomous Navigation (Real VIO/SLAM)](#phase-11--autonomous-navigation-real-vioslam)
    - [Phase 12 — Production Hardening](#phase-12--production-hardening)
  - [Issue Tracking](#issue-tracking)
    - [Epics](#epics)
    - [Foundation Hardening (Epic #64) — ✅ COMPLETE](#foundation-hardening-epic-64---complete)
    - [Process \& Thread Watchdog (Epic #88) — ✅ COMPLETE](#process--thread-watchdog-epic-88---complete)
    - [Phase 9 — First Safe Flight](#phase-9--first-safe-flight)
    - [Phase 10 — Real Cameras \& Perception](#phase-10--real-cameras--perception-1)
    - [Phase 11 — Autonomous Navigation](#phase-11--autonomous-navigation)
    - [Phase 12 — Production Hardening](#phase-12--production-hardening-1)
    - [Zenoh IPC Migration (Phase 12) — ✅ COMPLETE](#zenoh-ipc-migration-phase-12---complete)
    - [Core Autonomy \& Safety (Epic #110) — ✅ COMPLETE](#core-autonomy--safety-epic-110---complete)
  - [Metrics History](#metrics-history)
    - [Process Activity During Simulation](#process-activity-during-simulation)

---

## Current State at a Glance

| Metric | Value |
|--------|-------|
| Unit tests | See [tests/TESTS.md](../../tests/TESTS.md) for current counts |
| Compiler warnings | **0** (`-Werror -Wall -Wextra`, `[[nodiscard]]` enforced) |
| HAL interfaces | 6 (ICamera, IFCLink, IGCSLink, IGimbal, IIMUSource, IRadar) |
| HAL backends | 10 (6 simulated + GazeboCam + GazeboIMU + GazeboRadar + MavlinkFCLink) |
| Perception backends | 3 (simulated, color_contour, YOLOv8-nano via OpenCV DNN) |
| Simulation | Full closed-loop Gazebo Harmonic + PX4 SITL (D* Lite planner + 3D avoidance + HD-map) |
| Autonomous flight | ARM → Takeoff → Navigate 7 WPs (D* Lite + 3D avoidance) → RTL → Land → Disarm |
| CI | GitHub Actions — 5-job pipeline: format gate + 3-leg build matrix (sanitizers) + coverage ([docs/CI_SETUP.md](../guides/CI_SETUP.md)) |
| Line coverage | **75.1%** (lcov) |
| Code style | `.clang-format` enforced via CI format gate (clang-format-18) |
| Config tunables | 95+ (JSON, dot-path access, schema-validated) |
| Error handling | `Result<T,E>` monadic type — no exceptions |
| Sanitizers | ASan, TSan, UBSan (all CI-enforced) |
| `[[nodiscard]]` | 26 headers annotated, compiler-enforced |
| OpenCV | 4.10.0 from source (core + imgproc + dnn) |
| MAVSDK | 2.12.12 |
| Target hardware | **NVIDIA Jetson Orin** (Nano/NX/AGX, aarch64, JetPack 6.x) |
| IPC framework | **Zenoh 1.7.2** (sole backend, SHM removed) — All 6 migration phases complete + legacy SHM removed ([ADR-001](../adr/ADR-001-ipc-framework-selection.md), PRs #52–#57, Issue #126) |
| E2E testing | **42/42** Zenoh smoke-test checks passing (`tests/test_zenoh_e2e.sh`) |
| Process supervision | **systemd** service units (7 services + target) with `BindsTo` dependencies |
| Thread watchdog | **ThreadHeartbeat + ThreadWatchdog** — per-thread stuck detection via atomics |
| Process watchdog | **ProcessManager** — fork+exec, dependency graph, exponential backoff |
| Observability | JSON logging + IPC latency histograms + cross-process correlation IDs |
| Planning | **D* Lite incremental planner** (8-connected 2D; 26-connected 3D planned — #250, requires #252) + A* 3D grid planner + 3D obstacle avoidance + potential field fallback |
| Safety | **Geofence** (polygon + altitude) + 3-tier battery RTL + FC link-loss contingency |
| Perception fusion | **UKF** (RGB camera), ITracker + O(n³) Hungarian, **ByteTrack** two-stage association |
| VIO infrastructure | Feature extraction + stereo matching + IMU pre-integration + **covariance-derived quality** (`trace(P_position)` from IMU pre-integrator, configurable thresholds) |
| Integration testing | **20 Tier 1 + 5 Tier 2 scenarios** on Zenoh; sideband fault injector CLI |
| Test scenarios | 25 parameterized JSON configs with fault sequences + pass criteria |
| Bug fixes | **50** total (see [BUG_FIXES.md](BUG_FIXES.md)) |
| Cosys-AirSim Tier 3 perception | **Phase 1 baseline locked** — sim-perfect ground-truth depth + segmentation + Echo radar (sensor type 7) end-to-end. Scenario 33 PASS, all 26 checks green, zero cube collisions ([PR #704](https://github.com/nmohamaya/companion_software_stack/pull/704)) |

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

**Three-layer watchdog architecture** ([ADR-004](../adr/ADR-004-process-thread-watchdog-architecture.md)):

| Layer | Component | What It Does | PR(s) |
|-------|-----------|-------------|-------|
| **Layer 1** | `ThreadHeartbeat` + `ThreadWatchdog` | Per-thread atomic heartbeats; watchdog scans for stuck threads (configurable timeout) | #94 |
| **Layer 2** | `ProcessManager` (in P7) | fork+exec child processes, PID management; supervises process lifecycle in `--supervised` mode | #101, #102 |
| **Layer 2** | `RestartPolicy` + `ProcessGraph` | Exponential backoff, max retries, cooldown; dependency-aware restart cascades | #101, #102 |
| **Layer 3** | systemd service units | 7 `.service` files + `drone-stack.target`; `BindsTo` dependencies; `sd_notify` watchdog | #107 |

**Key files:**
- `common/util/include/util/thread_heartbeat.h` — `ThreadHeartbeat`, `ThreadHeartbeatRegistry`, `ThreadSnapshot`
- `common/util/include/util/thread_watchdog.h` — scans all heartbeats, reports stuck threads
- `common/util/include/util/thread_health_publisher.h` — publishes `ShmThreadHealth` to P7
- `process7_system_monitor/include/monitor/process_manager.h` — fork+exec supervisor with PID tracking
- `common/util/include/util/restart_policy.h` — per-process restart policy (backoff, max retries)
- `common/util/include/util/process_graph.h` — dependency graph (e.g., perception depends on video_capture)
- `common/util/include/util/sd_notify.h` — thin `sd_notify()` wrapper (no-op when not under systemd)
- `deploy/systemd/drone-*.service` — 7 service units + `drone-stack.target`

**Design decisions:**
1. Supervisor lives in P7 — co-located with health metrics
2. Backend-independent — heartbeats use atomics (no Zenoh), PID polling works in SHM-only mode
3. Lightweight — one `atomic_store(relaxed)` per loop (~1 ns on ARM)
4. systemd uses `BindsTo=` (not `Requires=`) so dependents are stopped when dependencies stop or disappear; combined with `Restart=on-failure`, systemd restarts each unit independently

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

### Phase 11 — Core Autonomy & Safety (Epic #110)

> Multi-phase epic: transport-agnostic IPC, perception overhaul, VIO foundation, planning & safety, integration testing.

**Phase 0 — Transport-Agnostic IMessageBus (#111, PR #117):**
- Type-erased `MessageBus` wrapper class hiding `std::variant`
- `bus.advertise()` / `bus.subscribe()` API — all 7 processes migrated
- Adding a new backend (iceoryx, DDS) requires one file change — zero process code changes

**Phase 1A — Remove Simulated LiDAR/Radar (#112, PR #117):**
- Removed `lidar_thread()`, `radar_thread()`, `LiDARCluster`, `RadarDetection`, `PointCloud`
- FusionEngine cleaned to camera-only input path

**Phase 1B — ITracker + Hungarian (#113, PR #117):**
- `ITracker` interface + factory pattern (`"sort"` → `MultiObjectTracker`)
- O(n³) Kuhn-Munkres Hungarian algorithm replacing greedy IOU matching

**Phase 1C — UKF Fusion (#114, PR #117):**
- Per-object UKF fusion engine (RGB camera → 3D position + velocity)
- `IFusionEngine` interface + factory
- *(Thermal camera code path removed in Issue #211 — replaced by radar sensor roadmap)*

**Phase 2A — VIO Foundation (#115, PR #118):**
- ORB/FAST feature extraction + stereo matching (disparity-based depth)
- IMU pre-integration (rotation, velocity, position delta accumulation)
- `IVIOBackend` interface + factory
- Error handling & diagnostics rolled out to all 7 processes

**Phase 3 — Planning & Safety (#116, PR #119):**
- A* 3D grid path planner with 26-connected search, obstacle inflation, Euclidean heuristic
- 3D obstacle avoider (XYZ repulsive field, velocity-based prediction)
- Geofence (ray-casting point-in-polygon + altitude bounds + warning margin)
- FaultManager: 3-tier battery (WARN/RTL/LAND), FC link-loss (LOITER→RTL), geofence breach
- GCS mission upload mid-flight

**Integration Testing (#120, PR #121):**
- Fault injection CLI tool (`tools/fault_injector/`) — battery, FC link, GCS commands, thermal, mission upload
- 8 parameterized test scenarios (JSON configs with fault sequences + pass criteria)
- Scenario runner (`tests/run_scenario.sh`) — two-tier model (Tier 1: SHM-only, Tier 2: Gazebo SITL)
- Architecture documentation with Mermaid diagrams (`docs/SIMULATION_ARCHITECTURE.md`)

**Integration Scenario Fixes (#122, PR #123):**
- 8 root-cause bug fixes to make all 7 Tier 1 scenarios pass (Fix #17–#24 in BUG_FIXES.md)
- Sideband `/fault_overrides` channel — fault injector no longer races with producer processes
- `FAULT_BATTERY_RTL` enum — distinct flag for RTL-level battery escalation
- VIO backend timestamp fix — `steady_clock::now()` instead of frame-counter epoch
- FC link-loss timestamp freeze — stale-heartbeat detection works correctly
- `potential_field_3d` obstacle avoider factory alias
- Transport-aware scenario runner (`--ipc shm|zenoh`) — **8/8 on Gazebo SITL + Zenoh (89/89 checks)** *(updated: scenarios 02/05 fixed post-merge, PR #130)*
- Scenario logs consolidated to `drone_logs/scenarios/`

**Cross-Epic Deliveries (Epic #25):**
- #27 Battery-critical auto-RTL ✅
- #28 FC heartbeat timeout + link-loss contingency ✅
- #29 Geofencing (polygon + altitude ceiling) ✅

**Issues:** #110 (epic), #111–#116, #120, #122  
**PRs:** #117, #118, #119, #121, #123  
**Tests:** 701 → 844 (SHM: 735, SHM+Zenoh: 844)

---

### Phase 12 — Documentation Completeness (Issue #149)

> Filled four major documentation gaps after Phase 11.

**Deliverables:**
- `docs/hal_design.md` — full HAL interface reference + backend availability matrix + factory function API
- `docs/adr/ADR-006-hal-hardware-abstraction-strategy.md` — Strategy + factory pattern ADR
- `docs/config_reference.md` — all 95+ `config/default.json` parameters with type, default, range, and consuming process
- `docs/adr/ADR-007-error-handling.md` — `Result<T,E>` adoption over exceptions ADR
- `docs/error_handling_design.md` — `ErrorCode` enum, `Error` class, `Result<T,E>` / `VoidResult` API, domain alias pattern
- `## Observability` sections added to all 7 process design docs (structured logging fields, correlation ID flow, latency tracking channels)

**Issue:** #149 ✅ Complete

---

## Planned Phases (Real Drone Deployment)

> **Epic:** [#25 — Real Drone Deployment — From Simulation to Flight](https://github.com/nmohamaya/companion_software_stack/issues/25)

### Phase 9 — First Safe Flight (Safety + Hardware Config)

> **Goal:** Fly a waypoint mission on a real flight controller with proper safety guardrails.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| ~~[#26](https://github.com/nmohamaya/companion_software_stack/issues/26)~~ | ~~Hardware config + launch script~~ | ~~P0~~ | ✅ Done (PR #43) — `config/hardware.json` + `deploy/launch_hardware.sh` |
| ~~[#27](https://github.com/nmohamaya/companion_software_stack/issues/27)~~ | ~~Battery-critical auto-RTL + temp failsafe~~ | ~~P0~~ | ✅ Done (Epic #110 Phase 3, PR #119) — 3-tier battery escalation (WARN/RTL/LAND) |
| ~~[#28](https://github.com/nmohamaya/companion_software_stack/issues/28)~~ | ~~FC heartbeat timeout + link-loss contingency~~ | ~~P0~~ | ✅ Done (Epic #110 Phase 3, PR #119) — LOITER→RTL contingency with configurable timeout |
| ~~[#29](https://github.com/nmohamaya/companion_software_stack/issues/29)~~ | ~~Geofencing (polygon + altitude ceiling)~~ | ~~P1~~ | ✅ Done (Epic #110 Phase 3, PR #119) — Ray-casting polygon + altitude bounds + warning margin |
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
| [#252](https://github.com/nmohamaya/companion_software_stack/issues/252) | 3D depth pipeline (stereo/depth sensor) | P1 | True 3D obstacle geometry via stereo disparity or depth camera. Prerequisite for #250 (3D planner) — without measured depth, vertical planning decisions rely on geometric guesses. Required when the drone must fly over or under obstacles. |
| ~~[#253](https://github.com/nmohamaya/companion_software_stack/issues/253)~~ | ~~YOLOv8 in Gazebo scenarios + detector cleanup~~ | ~~P2~~ | ✅ Done (Epic #263, PR #264) — SimulatedDetector extracted, YOLOv8 Gazebo scenario |

**Target Hardware:** NVIDIA Jetson Orin (Nano 8 GB / NX 16 GB / AGX 32–64 GB), JetPack 6.x, CUDA 12.x

**Exit Criteria:** Real camera producing detected objects on Jetson Orin; ground station receives telemetry.

---

### Phase 11 — Autonomous Navigation (Real VIO/SLAM)

> **Goal:** GPS-denied navigation using onboard vision.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| [#37](https://github.com/nmohamaya/companion_software_stack/issues/37) | Real VIO backend (ORB-SLAM3 / VINS-Fusion) | P0 | Integrate ORB-SLAM3 or VINS-Fusion as `IVIOBackend` backend; stereo or mono+IMU |
| [#38](https://github.com/nmohamaya/companion_software_stack/issues/38) | Stereo camera calibration pipeline | P1 | Checkerboard calibration tool; output intrinsics + extrinsics to config JSON |
| [#39](https://github.com/nmohamaya/companion_software_stack/issues/39) | VIO/GPS fusion via PX4 external vision | P1 | Publish VIO pose to PX4 via MAVLink `VISION_POSITION_ESTIMATE`; EKF2 fusion |
| ~~[#254](https://github.com/nmohamaya/companion_software_stack/issues/254)~~ | ~~Covariance-derived VIO quality~~ | ~~P1~~ | ✅ Done (Epic #263, PR #276) — `trace(P_position)` from IMU pre-integrator, configurable thresholds, feature/match fallback |

**Exit Criteria:** Indoor GPS-denied flight using VIO pose estimation.

---

### Phase 12 — Production Hardening

> **Goal:** Reliable enough for repeated real-world missions.

| Issue | Task | Priority | Description |
|-------|------|----------|-------------|
| ~~[#40](https://github.com/nmohamaya/companion_software_stack/issues/40)~~ | ~~Flight data recorder + replay~~ | ~~P1~~ | ✅ Done (Epic #263, PR #272) — Ring-buffer flight recorder with IPC replay tool |
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | P0 | 🟡 **Partial** — FaultManager ([#61](https://github.com/nmohamaya/companion_software_stack/issues/61), PR #63) handles 8 fault conditions with graduated response. Remaining: geofencing, motor failure, SLAM divergence detection |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | P2 | `IGimbal` backend for SIYI A8 mini (UART) or PWM servo; stabilisation loop |
| — | Predictive thermal trend monitoring | P1 | Replace threshold-only thermal zone computation with sliding-window linear regression (dT/dt) to predict time-to-thermal-runaway. Issue early warnings before critical temperature is reached, allowing the drone to RTL while still safe. Thresholds must be calibrated per-platform based on host SBC thermal characteristics (e.g., Jetson Orin throttle at 97°C, heatsink/fan curves). See `config/hardware.json` for deployment thresholds. |
| [#250](https://github.com/nmohamaya/companion_software_stack/issues/250) | D* Lite 3D grid (26-connected) | P2 | Current planner is 8-connected 2D (goal Z snapped to start Z). Upgrade to 26-connected 3D search for non-flat terrain on real hardware. Add config toggle for 2D/3D mode. |
| [#251](https://github.com/nmohamaya/companion_software_stack/issues/251) | Production deployment: Jetson Orin Nano | P1 | Native build pipeline, dependency install, systemd deployment, thermal/memory/serial caveats, TLS security, checklist for first hardware build |
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

### Phase 13 — Perception Pipeline v2 (Rewrite)

> **Meta-Epic:** [#514 — Perception Pipeline v2 (Rewrite)](https://github.com/nmohamaya/companion_software_stack/issues/514)
>
> **Goal:** Replace the current single-path YOLO+UKF+single-layer-grid perception with a 3-path (SAM/detector + monocular depth + radar-Doppler), class-conditional IMM tracker, and dual-layer grid. Fixes known bugs (#506 haunted cells, YOLO blindness to non-COCO objects) and adds primary drone-detection capability via radar-Doppler.
>
> **Design docs (gitignored):** `docs/design/perception_architecture.md` (spec), `docs/design/perception_rewrite_log.md` (living build log), [ADR-012](../adr/ADR-012-detector-licensing-yolo-vs-rtdetr.md) (detector licensing), [ADR-013](../adr/ADR-013-stereo-radar-redundancy-vs-fusion.md) (stereo + radar — redundancy vs. unified UKF fusion).
>
> **Universal acceptance criteria on every sub-issue:**
> 1. Update `docs/design/perception_rewrite_log.md` with a beginner-friendly component section + code links.
> 2. Update `LICENSE` and/or ADR-012 if the work introduces new license obligations (new third-party deps, model weights, datasets).
> 3. Stereo + radar paths must remain architecturally independent into the occupancy grid per ADR-013; do not fold PATH A clusters into the UKF.

#### Foundations (Phase 13a — no-training, pure-algo)

| Epic | Title | Scope |
|------|-------|-------|
| E1 | **HAL Interface Layer** | `IInferenceBackend`, `IVolumetricMap`, `ISemanticProjector`, `IEventCamera` slot reservation. CPU/ORT reference backend. |
| E2 | **Dual-Layer Grid + Free-Space Ray-Casting** | Replace single-layer grid with STATIC (log-odds) + DYNAMIC (track-referenced) layers + ray-cast demotion. Class tags. Compat adapter to existing D*Lite. |
| E3 | **Class-Conditional IMM Tracker** | Replace single-model UKF with IMM over CV/CTRV/CTRA/constant-jerk/random-walk. Class→model lookup. Prediction-cone rasteriser. |
| E4 | **Per-Class Config Schema** | Config keys for per-class safety margins, motion-model assignments, prediction horizons. |
| E13 | **Hardware-Adaptive Runtime & Dev Patterns** | Per-path enable/disable flags, `HardwareCapabilityProbe`, auto model-variant selection, dev-pattern docs. Lands alongside E1 so downstream epics ship with flags from day one. |

#### New perception paths (Phase 13b — v1 uses pretrained weights only)

| Epic | Title | Scope |
|------|-------|-------|
| E5 | **PATH A — SAM + Detector** | SAM backend (class-agnostic masks) + YOLOv8-seg wrapper + mask-detection assignment + mask×depth projection + COCO/VisDrone altitude switching. |
| E6 | **PATH C — Radar-Doppler Pipeline** | CFAR + clustering + rule-based micro-Doppler classifier (drone/bird/human) + Doppler attachment + radar-camera decision-level fusion. |
| E7 | **M-Detector Static/Dynamic Dispatcher** | Point-level static-vs-dynamic classification before grid update. |

#### Evaluation + guardrails (Phase 13c)

| Epic | Title | Scope |
|------|-------|-------|
| E8 | **Perception Benchmark Harness** | TP/FP/FN framework, latency profiler, CI-gated regression detection. |
| E9 | **RT-DETR Parallel Backend** | Per ADR-012. `OpenCvRtDetrDetector` alongside YOLO, scenario validation. Not the default until commercial gate. |
| E10 | **Collision-Confirmed Permanent Cells** | Closes #506 — IMU-jerk trigger freezes cells at collision location. |
| E12 | **AirSim Evaluation Scenarios** | Cluttered scenes, fast movers, adversarial lighting, drone-vs-drone, multi-obstacle. Full-pipeline stress tests. |

#### Parallel track (Phase 13d — independent of perception)

| Epic | Title | Scope |
|------|-------|-------|
| E11 ([#497](https://github.com/nmohamaya/companion_software_stack/issues/497)) | **Custom SWVIO — Sliding-Window Optimization VIO** | MIT-licensed custom stereo-inertial VIO with loop closure. Eigen-only. Multi-sensor integration (radar fusion, dynamic object masking, ML depth priors) designed-in. 5 phases, ~4240 lines. Supersedes #527 (OpenVINS proposal, closed). |

**Open design decisions for #497** (don't block start, can defer):

- **Stereo vs mono camera** — Phase 1-2 of #497 are camera-agnostic. Decision point is before Phase 3 (feature tracking). Affects hardware BOM and `ICamera` shape.
- **Hardware plan (α/β/γ — NVIDIA-tuned vs agnostic vs middle-ground)** — #497 is Eigen-only → hardware-agnostic. Any GPU acceleration of feature tracking is additive and deferred until real-hardware deployment.

**Each Phase 13a/b/c epic also carries ONE targeted integration scenario** as a sub-issue (e.g., "Scenario 31 — dual-layer grid ghost-cell regression") — these land alongside the component. E12's scenarios stress the fully-integrated pipeline.

**Exit Criteria (Phase 13):** Full pipeline passes AirSim evaluation scenarios with improved metrics over baseline; Gazebo scenarios pass or explicitly have twinned "new-pipeline" variants; `docs/design/perception_rewrite_log.md` covers every component; LICENSE updated for any new deps.

---

## Issue Tracking

### Epics

| # | Title | State |
|---|-------|-------|
| [#25](https://github.com/nmohamaya/companion_software_stack/issues/25) | [Epic] Real Drone Deployment — From Simulation to Flight | Open |
| [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) | [Epic] Zenoh IPC Migration — From POSIX SHM to Zero-Copy Network-Transparent IPC | **Closed** ✅ |
| [#64](https://github.com/nmohamaya/companion_software_stack/issues/64) | [Epic] Foundation Hardening — CI, Error Handling, Code Quality | **Closed** ✅ |
| [#88](https://github.com/nmohamaya/companion_software_stack/issues/88) | [Epic] Process & Thread Watchdog — Crash Recovery & Stuck-Thread Detection | **Closed** ✅ |
| [#110](https://github.com/nmohamaya/companion_software_stack/issues/110) | [Epic] Core Autonomy & Safety — IPC, Perception, VIO, Planning | **Closed** ✅ |
| ~~[#126](https://github.com/nmohamaya/companion_software_stack/issues/126)~~ | ~~[Epic] Zenoh-Only IPC — Remove Legacy SHM, Keep Middleware-Swappable~~ | **Closed** ✅ (PR #151) |
| ~~[#263](https://github.com/nmohamaya/companion_software_stack/issues/263)~~ | ~~[Epic] Autonomous Intelligence & Sim Fidelity~~ | **Closed** ✅ (integration branch, 10 PRs) |
| [#419](https://github.com/nmohamaya/companion_software_stack/issues/419) | [Epic] Depth Fusion Infrastructure — Covariance, Priors, Radar Scale | Wave 1 ✅ (PR #427) |

### Standalone Issues (Post-Epic)

| # | Title | State |
|---|-------|-------|
| ~~[#345](https://github.com/nmohamaya/companion_software_stack/issues/345)~~ | ~~Bbox ground-feature filters, depth confidence gating, radar orphan tuning~~ | **Closed** ✅ (PR #346) |
| ~~[#279](https://github.com/nmohamaya/companion_software_stack/issues/279)~~ | ~~Comprehensive documentation update~~ | **Closed** ✅ |

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
| [#91](https://github.com/nmohamaya/companion_software_stack/issues/91) | Process supervisor (fork+exec) | Phase 3 | **Closed** (ADR in PR #100; implementation in PRs #101, #102) |
| [#92](https://github.com/nmohamaya/companion_software_stack/issues/92) | Restart policies + dependency graph | Phase 4 | **Closed** (PRs #101, #102) |
| [#97](https://github.com/nmohamaya/companion_software_stack/issues/97) | Tech debt: snapshot() vector→array | — | **Closed** (PR #108) |
| [#98](https://github.com/nmohamaya/companion_software_stack/issues/98) | Tech debt: P7 div-by-zero + strncpy | — | **Closed** (PR #108) |
| [#103](https://github.com/nmohamaya/companion_software_stack/issues/103) | Deploy scripts + test runners | — | **Closed** (PR #104) |

### Phase 9 — First Safe Flight

| # | Title | State |
|---|-------|-------|
| ~~[#26](https://github.com/nmohamaya/companion_software_stack/issues/26)~~ | ~~Hardware config + launch script~~ | **Closed** (PR #43) |
| ~~[#27](https://github.com/nmohamaya/companion_software_stack/issues/27)~~ | ~~Battery-critical auto-RTL + temperature failsafe~~ | **Closed** (Epic #110, PR #119) |
| ~~[#28](https://github.com/nmohamaya/companion_software_stack/issues/28)~~ | ~~FC heartbeat timeout + link-loss contingency~~ | **Closed** (Epic #110, PR #119) |
| ~~[#29](https://github.com/nmohamaya/companion_software_stack/issues/29)~~ | ~~Geofencing (polygon + altitude ceiling)~~ | **Closed** (Epic #110, PR #119) |
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
| [#252](https://github.com/nmohamaya/companion_software_stack/issues/252) | 3D depth pipeline (stereo/depth sensor) | Open |

### Phase 11 — Autonomous Navigation

| # | Title | State |
|---|-------|-------|
| [#37](https://github.com/nmohamaya/companion_software_stack/issues/37) | Real VIO backend (ORB-SLAM3 / VINS-Fusion) | Open |
| [#38](https://github.com/nmohamaya/companion_software_stack/issues/38) | Stereo camera calibration pipeline | Open |
| [#39](https://github.com/nmohamaya/companion_software_stack/issues/39) | VIO/GPS fusion via PX4 external vision | Open |
| ~~[#254](https://github.com/nmohamaya/companion_software_stack/issues/254)~~ | ~~Covariance-derived VIO quality~~ | **Closed** (Epic #263, PR #276) |

### Phase 12 — Production Hardening

| # | Title | State |
|---|-------|-------|
| [#40](https://github.com/nmohamaya/companion_software_stack/issues/40) | Flight data recorder + replay | Open |
| [#41](https://github.com/nmohamaya/companion_software_stack/issues/41) | Contingency fault tree | **Mostly complete** — FaultManager (#61 PR #63) + geofence/battery/FC-link (Epic #110 PR #119) + collision recovery (Epic #263 PR #268) + covariance-based VIO divergence (PR #276). Remaining: motor failure |
| [#61](https://github.com/nmohamaya/companion_software_stack/issues/61) | FaultManager — graceful degradation | **Closed** (PR #63) |
| [#42](https://github.com/nmohamaya/companion_software_stack/issues/42) | Gimbal driver (SIYI / PWM) | Open |
| [#250](https://github.com/nmohamaya/companion_software_stack/issues/250) | D* Lite 3D grid (26-connected) | Open |
| [#273](https://github.com/nmohamaya/companion_software_stack/issues/273) | Full S-matrix gating for all radar association | Open |
| [#274](https://github.com/nmohamaya/companion_software_stack/issues/274) | Safety-critical C++ audit — scan codebase against full rule set | Open |

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

### Core Autonomy & Safety (Epic #110) — ✅ COMPLETE

| # | Title | Phase | State |
|---|-------|-------|-------|
| [#111](https://github.com/nmohamaya/companion_software_stack/issues/111) | Transport-Agnostic IMessageBus | Phase 0 | **Closed** (PR #117) |
| [#112](https://github.com/nmohamaya/companion_software_stack/issues/112) | Remove LiDAR/Radar | Phase 1A | **Closed** (PR #117) |
| [#113](https://github.com/nmohamaya/companion_software_stack/issues/113) | ITracker + Hungarian | Phase 1B | **Closed** (PR #117) |
| [#114](https://github.com/nmohamaya/companion_software_stack/issues/114) | Thermal Camera + UKF Fusion | Phase 1C | **Closed** (PR #117) |
| [#115](https://github.com/nmohamaya/companion_software_stack/issues/115) | Stereo VIO Foundation | Phase 2A | **Closed** (PR #118) |
| [#116](https://github.com/nmohamaya/companion_software_stack/issues/116) | Planning & Safety (A*, Geofence, FaultMgr) | Phase 3 | **Closed** (PR #119) |
| [#120](https://github.com/nmohamaya/companion_software_stack/issues/120) | Integration Testing — Scenario Harness | Testing | **Closed** (PR #121) |
| [#122](https://github.com/nmohamaya/companion_software_stack/issues/122) | Integration scenario failures — 8 root causes | Bug fix | **Closed** (PR #123) |
| [#125](https://github.com/nmohamaya/companion_software_stack/issues/125) | Scenario 02 Obstacle Avoidance: SITL Fixes (TTL grid, goal snap, thermal, geofence, BFS escape, HD-map) | Bug fix / Feature | **Closed** (Fixes #30–35) |
| [#126](https://github.com/nmohamaya/companion_software_stack/issues/126) | HD-Map Two-Layer Occupancy Grid (static + camera TTL) | Feature | **Closed** (Fix #35) |
| [#127](https://github.com/nmohamaya/companion_software_stack/issues/127) | Proximity-Based Collision Detection in NAVIGATE Loop | Feature | **Closed** (Fix #36) |
| [#129](https://github.com/nmohamaya/companion_software_stack/issues/129) | PX4 exit tears down companion stack and GUI (launch_gazebo.sh) | Bug | **Open** (Bug #29 — FIXME in launch_gazebo.sh) |
| *(this session)* | RTL disarm detection: `nav_was_armed_` gap at fault/GCS/complete RTL entry points | Bug fix | **Closed** (Fix #39 — main.cpp) |
| *(this session)* | OBSTACLE COLLISION guard missing from 6/8 scenario pass criteria | Config fix | **Closed** (Fix #40 — all 8 scenarios) |
| *(this session)* | Scenario 07 thermal false-fail: `temp_crit_c` override below host CPU temp | Bug fix | **Closed** (Fix #38 — scenario 07 config) |
| *(this session)* | Scenario 05 geofence: WP4 clips magenta cylinder + floor flood + RTL lock | Bug fix | **Closed** (Fix #37 — scenario 05 config + main.cpp) |
| [#131](https://github.com/nmohamaya/companion_software_stack/issues/131) | sd_notify/WatchdogSec missing from processes 1–6; StartLimitBurst=5 missing all 7 service files | Reliability fix | **Closed** (PR #132, commit `7ace87c`) |
| *(fault_injector Zenoh fix)* | ~~fault_injector GCS/mission commands invisible to Zenoh subscribers~~ ✅ | Bug fix | **Closed** (Improvement #37, PR #136) |
| *(fault_injector shm_unlink fix)* | ~~fault_injector shm_unlink on exit breaks subsequent runs~~ ✅ | Bug fix | **Closed** (Improvement #37, PR #136) |
| ~~[#154](https://github.com/nmohamaya/companion_software_stack/issues/154)~~ | ~~P4 Mission Planner: Extract 4 classes from main.cpp~~ ✅ | Refactor | **Closed** (Improvement #41, PR #157) |
| ~~[#155](https://github.com/nmohamaya/companion_software_stack/issues/155)~~ | ~~Stale SHM reference cleanup in API.md and ROADMAP.md~~ ✅ | Docs | **Closed** (Improvement #42, PR #156) |
| ~~[#158](https://github.com/nmohamaya/companion_software_stack/issues/158)~~ | ~~D* Lite Incremental Path Planner~~ ✅ | Feature / Refactor | **Closed** (Improvement #43) |
| ~~[#163](https://github.com/nmohamaya/companion_software_stack/issues/163)~~ | ~~ByteTrack Multi-Object Tracker~~ ✅ | Feature | **Closed** (Improvement #44, PR #165) |
| [#167](https://github.com/nmohamaya/companion_software_stack/issues/167) | Scenario configs for ByteTrack + new perception tracking scenario | Bug fix / Testing | **Open** |
| ~~[#210](https://github.com/nmohamaya/companion_software_stack/issues/210)~~ | ~~UKF Radar Fusion — radar measurement model + association~~ ✅ | Feature | **Closed** (PR #218) |
| ~~[#211](https://github.com/nmohamaya/companion_software_stack/issues/211)~~ | ~~Remove Thermal Camera — replaced by radar sensor~~ ✅ | Refactor | **Closed** (PR #214) |
| ~~[#212](https://github.com/nmohamaya/companion_software_stack/issues/212)~~ | ~~Gazebo Radar Backend — gpu_lidar HAL + radar HAL thread~~ ✅ | Feature | **Closed** (PRs #218, #219, #221) |
| [#217](https://github.com/nmohamaya/companion_software_stack/issues/217) | NVIDIA EGL broader scope — hardware deployment | Reliability | **Open** (deferred to hardware phase) |
| ~~[#220](https://github.com/nmohamaya/companion_software_stack/issues/220)~~ | ~~P3 slam_vio_nav unclamped loop rates~~ ✅ | Bug | **Closed** (Epic #263, PR #266) |
| ~~[#222](https://github.com/nmohamaya/companion_software_stack/issues/222)~~ | ~~Perception-driven obstacle avoidance scenario (no HD-map)~~ ✅ | Testing | **Closed** (Improvement #54) |
| ~~[#224](https://github.com/nmohamaya/companion_software_stack/issues/224)~~ | ~~Fix Perception Fusion Pipeline — SPSC Overflow + Radar Fusion Bottleneck~~ ✅ | Bug Fix / Perf | **Closed** (Improvement #55, Fix #42) |
| ~~[#225](https://github.com/nmohamaya/companion_software_stack/issues/225)~~ | ~~Radar ground-plane filter + avoider dead zone fix~~ ✅ | Feature / Bug Fix | **Closed** (Improvement #56, Fix #41) |
| ~~[#229](https://github.com/nmohamaya/companion_software_stack/issues/229)~~ | ~~Radar fusion fix: FOV, ground filter, altitude gate, path-aware avoider~~ ✅ | Bug Fix / Feature | **Closed** (Improvement #59, Fix #45–47) |
| ~~[#234](https://github.com/nmohamaya/companion_software_stack/issues/234)~~ | ~~D* Lite queue performance + Z-band + km reinit~~ ✅ | Performance | **Closed** (Improvement #60-61, Bug #48-49) |
| ~~[#236](https://github.com/nmohamaya/companion_software_stack/issues/236)~~ | ~~Waypoint overshoot detection~~ ✅ | Feature | **Closed** (Improvement #62) |
| ~~[#237](https://github.com/nmohamaya/companion_software_stack/issues/237)~~ | ~~Epic: Accurate Obstacle Detection & Grid Population~~ ✅ | Feature / Bug Fix | **Closed** (Improvement #63, PR #241, Bug #237a-b) |
| ~~[#242](https://github.com/nmohamaya/companion_software_stack/issues/242)~~ | ~~Persistent Timestamped Scenario Logging~~ ✅ | Infrastructure | **Closed** (Improvement #64, PR #241) |

---

### Autonomous Intelligence & Sim Fidelity (Epic #263) — ✅ COMPLETE

| # | Title | Sub-Epic | State |
|---|-------|----------|-------|
| ~~[#220](https://github.com/nmohamaya/companion_software_stack/issues/220)~~ | ~~P3 rate clamping~~ | A: Foundation | **Closed** (PR #266) |
| ~~[#258](https://github.com/nmohamaya/companion_software_stack/issues/258)~~ | ~~D* Lite corner-cutting guard~~ | A: Foundation | **Closed** (PR #267) |
| ~~[#226](https://github.com/nmohamaya/companion_software_stack/issues/226)~~ | ~~Post-collision recovery~~ | A: Safety | **Closed** (PR #268) |
| ~~[#231](https://github.com/nmohamaya/companion_software_stack/issues/231)~~ | ~~Radar-only track initiation~~ | B: Perception | **Closed** (PR #271) |
| ~~[#253](https://github.com/nmohamaya/companion_software_stack/issues/253)~~ | ~~YOLOv8 Gazebo + detector cleanup~~ | B: Perception | **Closed** (PR #264) |
| ~~[#256](https://github.com/nmohamaya/companion_software_stack/issues/256)~~ | ~~Dynamic obstacle prediction via UKF~~ | D: Advanced | **Closed** (PR #270) |
| ~~[#257](https://github.com/nmohamaya/companion_software_stack/issues/257)~~ | ~~Gimbal auto-tracking~~ | D: Advanced | **Closed** (PR #265) |
| ~~[#40](https://github.com/nmohamaya/companion_software_stack/issues/40)~~ | ~~Flight data recorder + replay~~ | D: Advanced | **Closed** (PR #272) |
| ~~[#191](https://github.com/nmohamaya/companion_software_stack/issues/191)~~ | ~~Gazebo Full VIO backend~~ | C: VIO | **Closed** (PR #275) |
| ~~[#254](https://github.com/nmohamaya/companion_software_stack/issues/254)~~ | ~~Covariance-derived VIO quality~~ | C: VIO | **Closed** (PR #276) |
| ~~[#255](https://github.com/nmohamaya/companion_software_stack/issues/255)~~ | ~~Remove legacy IVisualFrontend~~ | C: VIO | **Closed** (PR #277) |

---

### Cosys-AirSim Tier 3 Phase 1 Perception (PR #704) — pending merge

| # | Title | State |
|---|-------|-------|
| [#698](https://github.com/nmohamaya/companion_software_stack/issues/698) | PATH A voxels cement walls without radar agreement; DA V2 max-clamp ghosts feed the grid | **Closes on PR #704 merge** (sidestepped by ground-truth perception baseline; real-algo follow-ups tracked in Epics #514/#520) |
| [#702](https://github.com/nmohamaya/companion_software_stack/issues/702) | Cosys-AirSim radar (lidar-emulated) misses 5/7 spawned obstacles | **Closes on PR #704 merge** (replaced by `CosysEchoBackend`) |
| [#705](https://github.com/nmohamaya/companion_software_stack/issues/705) | Adopt Cosys Echo as physical radar simulator; deprecate lidar-emulated radar | **Closes on PR #704 merge** |
| [#703](https://github.com/nmohamaya/companion_software_stack/issues/703) | Planner: graceful handling of waypoints inside obstacles | **Open** (deferred — filed during this work) |

---

### Depth Fusion Infrastructure (Epic #419) — Wave 1 ✅ COMPLETE

| # | Title | Wave | State |
|---|-------|------|-------|
| ~~[#420](https://github.com/nmohamaya/companion_software_stack/issues/420)~~ | ~~Covariance-weighted depth fusion~~ | Wave 1 | **Closed** ✅ (PR #427) |
| ~~[#421](https://github.com/nmohamaya/companion_software_stack/issues/421)~~ | ~~IMU pitch/roll correction (full quaternion)~~ | Wave 1 | **Closed** ✅ (PR #427) |
| ~~[#422](https://github.com/nmohamaya/companion_software_stack/issues/422)~~ | ~~Radar-learned object heights / scale recovery~~ | Wave 1 | **Closed** ✅ (PR #427) |
| ~~[#423](https://github.com/nmohamaya/companion_software_stack/issues/423)~~ | ~~Multi-class height priors~~ | Wave 1 | **Closed** ✅ (PR #427) |
| ~~[#424](https://github.com/nmohamaya/companion_software_stack/issues/424)~~ | ~~Sim world diversification + depth accuracy scenario~~ | Wave 1 | **Closed** ✅ (PR #427) |

---

## Multi-Developer GitHub Setup

When onboarding new developers or enabling multi-agent collaboration, several files that are currently **local-only** (gitignored) need to be shared. This section documents what needs to be uploaded and configured.

### Files Currently Gitignored That Need Sharing

| File/Directory | Currently | Action Needed | Why |
|----------------|-----------|---------------|-----|
| `CLAUDE.md` | `.gitignore` | **Remove from .gitignore, commit** | Agent role definitions, build commands, architecture reference — all agents need this |
| `tasks/` | `.gitignore` | **Remove from .gitignore, commit** | `tasks/active-work.md` and `tasks/agent-changelog.md` are shared coordination state |
| `tasks/sessions/*.log` | Not tracked | **Keep gitignored** (via `tasks/sessions/.gitignore`) | Session logs are ephemeral, per-machine |
| `work_instructions.md` | `.gitignore` | **Evaluate** — merge useful content into `DEVELOPMENT_WORKFLOW.md` or remove | Personal notes vs shared guidance |
| `docs/SECURITY.md` | `.gitignore` | **Evaluate** — security docs should be shared if they don't contain secrets | Security policy benefits all contributors |
| `.claude/agents/` | **Tracked** | Already committed | Agent role definitions |
| `.claude/shared-context/` | **Tracked** | Already committed | Domain knowledge for all agents |
| `scripts/` | **Tracked** | Already committed | Orchestration scripts |

### GitHub Repository Settings

For multi-developer collaboration, configure these GitHub settings:

1. **Branch protection on `main`:**
   - Require PR reviews (1+ approvals)
   - Require status checks (CI must pass)
   - Require branches to be up to date before merging
   - No force pushes

2. **CODEOWNERS:** Already committed — auto-assigns reviewers by file path

3. **Issue templates:** Already committed (8 templates in `.github/ISSUE_TEMPLATE/`)

4. **PR template:** Already committed (`.github/pull_request_template.md`)

5. **Labels:** Create domain labels for agent routing:
   - `domain:perception`, `domain:nav`, `domain:integration`, `domain:infra`, `domain:comms`
   - `type:bug`, `type:feature`, `type:refactor`, `type:safety-audit`, `type:security-audit`
   - `agent:assigned`, `agent:completed`, `agent:blocked`

6. **GitHub Projects board:** Create a project board with columns:
   - Backlog | Triaged | In Progress | In Review | Done

### .gitignore Changes Required

Remove these lines from `.gitignore` to enable sharing:

```diff
- tasks/
- CLAUDE.md
```

Add these lines to keep ephemeral/sensitive files local:

```diff
+ # Agent session logs (ephemeral, per-machine)
+ tasks/sessions/*.log
+
+ # Local plan files (conversation-specific)
+ .claude/plans/
+ .claude/worktrees/
```

### Developer Onboarding Checklist

- [ ] Clone the repo
- [ ] Install dependencies (see `README.md`)
- [ ] Install pre-commit hook: `ln -sf ../../deploy/pre-commit .git/hooks/pre-commit`
- [ ] Verify build: `bash deploy/build.sh`
- [ ] Verify tests: `./tests/run_tests.sh`
- [ ] Review `CLAUDE.md` for architecture and agent roles
- [ ] Review `.claude/agents/` for your assigned role
- [ ] Read `.claude/shared-context/domain-knowledge.md` for known pitfalls
- [ ] Check `tasks/active-work.md` for current assignments

> **Issue:** [#358 — Enable multi-developer GitHub collaboration](https://github.com/nmohamaya/companion_software_stack/issues/358) — tracks the .gitignore changes and GitHub settings needed.

---

## Metrics History

| Metric | Phase 1 | Phase 3 | Phase 6 | Phase 7 | Phase 8 | Phase 9 | Zenoh A | Zenoh B | Zenoh C | Zenoh D | Zenoh E | Zenoh F | E2E | FaultMgr | Hardening | Watchdog | Epic #110 | Epic #263 | PR #346 | Epic #284 | **Epic #419 (Current)** |
|--------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|-----|--------|-------|-------|-------|-------|-------|-------|-------|
| Unit tests | 58 | 121 | 196 | 262 | 262 | 262 | 295 | 308 | 329 | 348 | 359 | 370 | 377 | 400 | 464 | 701 | 1108 | 1238 | 1259 | 1461 | **1479** |
| Test suites | 6 | 10 | 14 | 18 | 18 | 18 | 19 | 19 | 19 | 19 | 20 | 21 | 22 | 23 | 26 | 31+ | 42 | 47 | 47 | 65 | **66** |
| Bug fixes | 6 | 6 | 13 | 13 | 15 | 15 | 17 | 17 | 17 | 17 | 17 | 17 | 19 | 19 | 21 | 21 | 34 | 48 | 48 | 48 | **48** |
| Config tunables | 45+ | 45+ | 70+ | 75+ | 75+ | 80+ | 80+ | 80+ | 85+ | 85+ | 90+ | 90+ | 90+ | 95+ | 95+ | 95+ | 110+ | 120+ | 120+ | 125+ | **135+** |
| HAL backends | 0 | 5 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 8 | 9 | 9 | 9 | 9+plugin | **9+plugin** |
| IPC backends | SHM | SHM | SHM | SHM | SHM | SHM | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | SHM + Zenoh | Zenoh (sole) | Zenoh (sole) | Zenoh (sole) | Zenoh (sole) | **Zenoh (sole)** |
| Perception backends | 0 | 0 | 1 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3 | 3+plugin | **3+plugin** |
| Compiler warnings | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **0** |
| Processes on factory | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 0/7 | 2/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | **7/7** |
| Processes w/ real Gazebo data | 0/7 | 0/7 | 4/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | 5/7 | **5/7** |
| Zenoh channels migrated | — | — | — | — | — | — | 0/12 | 10/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | 12/12 | **12/12** |
| Liveliness tokens | — | — | — | — | — | — | — | — | — | — | — | 7 | 7 | 7 | 7 | 7 | 7 | 7 | 7 | 7 | **7** |
| Network transport | — | — | — | — | — | — | — | — | — | — | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes | **Yes** |
| E2E checks | — | — | — | — | — | — | — | — | — | — | — | — | 42/42 | 42/42 | 42/42 | 42/42 | 42/42 | 42/42 | 42/42 | 42/42 | **42/42** |
| CI matrix legs | 1 | 1 | 1 | 1 | 1 | 1 | 2 | 2 | 2 | 2 | 2 | 2 | 2 | 2 | 9 | 9 | 9 | 9 | 9 | 9 | **9** |
| Fault conditions | — | — | — | — | — | — | — | — | — | — | — | — | — | 8 | 8 | 8 | 10 | 12 | 12 | 12 | **12** |
| Sanitizers | — | — | — | — | — | — | — | — | — | — | — | — | — | — | ASan+TSan+UBSan | ASan+TSan+UBSan | ASan+TSan+UBSan | ASan+TSan+UBSan | ASan+TSan+UBSan | ASan+TSan+UBSan | **ASan+TSan+UBSan** |
| `[[nodiscard]]` headers | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 26 | 26 | 26 | 26 | 26 | 26 | **26** |
| Config schemas | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 7 | 7 | 7 | 7 | 7 | 7 | **7** |
| Error handling | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | exceptions | Result<T,E> | Result<T,E> | Result<T,E> | Result<T,E> | Result<T,E> | Result<T,E> | **Result<T,E>** |
| Line coverage | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 75.1% | 75.1% | 75.1% | 75.1% | 75.1% | 75.1% | **75.1%** |
| Code style | — | — | — | — | — | — | — | — | — | — | — | — | — | — | enforced | enforced | enforced | enforced | enforced | enforced | **enforced** |
| Thread watchdog | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | ThreadHeartbeat | ThreadHeartbeat + ThreadWatchdog | ThreadHeartbeat + ThreadWatchdog | ThreadHeartbeat + ThreadWatchdog | ThreadHeartbeat + ThreadWatchdog | **ThreadHeartbeat + ThreadWatchdog** |
| Process supervision | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | systemd + ProcessManager | systemd + ProcessManager | systemd + ProcessManager | systemd + ProcessManager | systemd + ProcessManager | **systemd + ProcessManager** |
| Planning | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | D* Lite + A* 3D + potential field | D* Lite + A* 3D + potential field | D* Lite + A* 3D + potential field | D* Lite + A* 3D + potential field | **D* Lite + A* 3D + potential field** |
| Depth estimation | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | hard tiers | hard tiers | hard tiers | **covariance + class priors + radar-learned** |
| Safety subsystems | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | Geofence + battery RTL + FC contingency | + collision recovery | + collision recovery | + collision recovery | **+ collision recovery** |
| Perception fusion | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | UKF radar-primary | + radar-only init + dynamic prediction + gimbal tracking | + radar-only init + dynamic prediction + gimbal tracking | + radar-only init + dynamic prediction + gimbal tracking | **+ covariance depth + class priors + radar-learned heights** |
| Integration scenarios | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 15 (14 T1 + 1 T2) | 25 (20 T1 + 5 T2), 20/20 T1 pass | 25 (20 T1 + 5 T2) | 25 (20 T1 + 5 T2) | **25 (20 T1 + 5 T2) + scenario 27** |
| VIO backends | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | — | 2 (Simulated + Gazebo) | 3 (+ GazeboFull) + covariance quality | 3 | 3 | **3** |

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

*Last updated after Epic #419 Wave 1 (PR #427) — see [tests/TESTS.md](../../tests/TESTS.md) for current test counts. 1479 tests, 66 C++ test files, 250+ scenario checks across 25 scenarios (20 Tier 1 + 5 Tier 2), Zenoh sole IPC backend, 9 CI jobs. Depth fusion infrastructure: covariance-weighted confidence, multi-class height priors, radar-learned object heights, full quaternion camera→world transform, diversified sim world.*
