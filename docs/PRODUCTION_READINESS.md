# Production Readiness Tracker

This document tracks **prototype shortcuts, simulated components, and technical
debt** that must be resolved before deploying to production hardware. Items are
grouped by category and tagged with priority.

**Priority legend:**
- **P0 — Blocker**: Must fix before first flight test
- **P1 — Critical**: Must fix before field deployment
- **P2 — Important**: Should fix before shipping to customers
- **P3 — Nice-to-have**: Improve when time permits

**Status legend:**
- 🔴 Not started
- 🟡 In progress
- 🟢 Done

---

## 1. Hardware Abstraction — Simulated → Real Backends

All hardware access uses the HAL interface pattern (`ICamera`, `IFCLink`, etc.).
Production requires implementing the real backend behind each interface.

| # | Component | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|-----------|--------------------|--------------------|----------|--------|-------|
| 1.1 | Camera backend | `SimulatedCamera` — synthetic gradient | V4L2 / libargus (Jetson CSI) | P0 | 🔴 | `ICamera` interface exists; implement `V4L2Camera` |
| 1.2 | Stereo sync | Independent captures, no HW sync | GPIO-triggered stereo / CSI frame sync | P1 | 🔴 | Required for accurate stereo depth |
| 1.3 | Frame timestamps | `steady_clock::now()` (host-side) | Kernel timestamps via `v4l2_buffer.timestamp` | P1 | 🔴 | Host timestamps have jitter; critical for VIO |
| 1.4 | Object detector | `SimulatedDetector` / `OpenCvYoloDetector` (CPU, ~7–13 FPS) | TensorRT YOLOv8-Nano (INT8, ~2 ms/frame on Orin) | P1 | 🔴 | `IDetector` interface exists |
| 1.5 | FC link | `SimulatedFCLink` — synthetic telemetry | `MavlinkFCLink` via serial UART (MAVLink 2) | P0 | 🟡 | MAVSDK backend exists for Gazebo SITL; serial UART config needed |
| 1.6 | GCS link | `SimulatedGCSLink` — fake RTL after 120 s | UDP MAVLink GCS protocol | P1 | 🔴 | |
| 1.7 | Gimbal | `SimulatedGimbal` — rate-limited slew model | UART / PWM / SBUS gimbal protocol | P1 | 🔴 | `IGimbal` interface exists |
| 1.8 | IMU | `SimulatedIMU` — noisy synthetic data | SPI/I2C driver (BMI088, ICM-42688-P, etc.) | P0 | 🔴 | `IIMUSource` interface exists; Gazebo backend works |
| 1.9 | Visual frontend | `SimulatedVisualFrontend` — circular trajectory | ORB-SLAM3 / VINS-Fusion integration | P1 | 🔴 | `IVisualFrontend` interface exists |
| 1.10 | Path planner | `PotentialFieldPlanner` — attractive force + EMA | RRT* / D* Lite | P2 | 🔴 | Current planner is functional but basic |
| 1.11 | Obstacle avoider | `PotentialFieldAvoider` — repulsive force | VFH+ / 3D-VFH | P2 | 🔴 | |
| 1.12 | LiDAR | Simulated random clusters in P2 | Point cloud driver (Livox, Ouster, etc.) | P2 | 🔴 | Need HAL `ILiDAR` interface |
| 1.13 | Radar | Simulated random detections in P2 | mmWave radar driver (TI AWR, etc.) | P2 | 🔴 | Need HAL `IRadar` interface |

---

## 2. IPC / Networking

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 2.1 | Zenoh security | `ALLOW_INSECURE_ZENOH=ON` — no TLS, no auth | `ZENOH_CONFIG_PATH` with TLS + mutual auth | P0 | 🔴 | CMake gate already exists; need to create zenoh.json5 with certs |
| 2.2 | zenohc SHM in CI | Pre-built debs lack `shared-memory` feature; 7 tests skip | Build zenohc from source with SHM, or wait for upstream fix | P2 | 🔴 | See CI-003 in CI_ISSUES.md |
| 2.3 | Wire format versioning | `kWireVersion = 1`, no negotiation | Version negotiation handshake on GCS connect | P3 | 🔴 | Current format is sufficient; add negotiation if multi-vendor |
| 2.4 | Network QoS | No QoS configuration on Zenoh publishers | Configure reliability, congestion control, priority per topic | P2 | 🔴 | Video = best-effort; commands = reliable |
| 2.5 | GCS client | Basic Python subscriber (`tools/gcs_client/`) | Full GCS application with command sending, mission upload, map UI | P1 | 🔴 | Current tool is for debugging only |
| 2.6 | Network encryption | TCP plaintext between drone ↔ GCS | TLS 1.3 on Zenoh transport (quic or tls protocol) | P0 | 🔴 | Tied to 2.1 |

---

## 3. Algorithms

| # | Component | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|-----------|--------------------|--------------------|----------|--------|-------|
| 3.1 | Kalman tracker | Linear KF, 8D constant-velocity | EKF/UKF with constant-turn-rate model, or IMM | P2 | � | UKFFusionEngine implemented (PR #117); KalmanBoxTracker still linear KF |
| 3.2 | Track association | Greedy nearest-neighbor | Hungarian (Munkres) O(n³) optimal assignment | P2 | 🟢 | O(n³) Kuhn-Munkres in `HungarianSolver` (PR #117) |
| 3.3 | Appearance features | None — position-only matching | DeepSORT / ByteTrack Re-ID vectors | P2 | 🔴 | Reduces ID switches by ~45% |
| 3.4 | Sensor fusion | Weighted average merge | EKF/UKF fusion with per-sensor measurement models | P2 | � | `UKFFusionEngine` with per-object UKF (PR #117); thermal end-to-end wiring pending |
| 3.5 | ISP pipeline | Raw RGB24 passthrough | Bayer demosaic → white balance → gamma → NV12 | P1 | 🔴 | Use NVIDIA ISP on Jetson |

---

## 4. Build & Deployment

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 4.1 | Process supervision | systemd units (production) / ProcessManager (`--supervised` mode) | systemd units with watchdog, auto-restart, dependency ordering | P0 | 🟢 | PR #107 (systemd), Epic #88 (watchdog) |
| 4.2 | OTA updates | None — manual scp + rebuild | Mender / SWUpdate / custom updater | P2 | 🔴 | |
| 4.3 | Log management | spdlog to files, no rotation | Log rotation (logrotate or spdlog rotating sink), remote log shipping | P1 | 🔴 | Disk will fill on long missions |
| 4.4 | Config validation | JSON Schema validation at startup; reject invalid configs | `ConfigSchema` builder-pattern validation at startup (7 schemas) | P1 | 🟢 | PR #76 — Issue #69 |
| 4.5 | Crash reporting | Process exits with return 1 | Core dump collection, crash telemetry upload | P2 | 🔴 | |
| 4.6 | Cross-compilation | Native x86 build only | Cross-compile for aarch64 (Jetson) | P0 | 🔴 | Need CMake toolchain file |
| 4.7 | Release builds | Debug/RelWithDebInfo | `-DCMAKE_BUILD_TYPE=Release` with `-O2 -DNDEBUG` + stripped binaries | P1 | 🔴 | |
| 4.8 | Hardware config | `config/hardware.json` has placeholder values | Board-specific config per drone variant (Jetson Nano/Orin/Xavier) | P1 | 🔴 | |

---

## 5. Safety & Reliability

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 5.1 | Failsafe logic | Basic RTL on battery low / link loss | Comprehensive failsafe: GPS loss, IMU failure, motor fault, geofence breach | P0 | 🔴 | |
| 5.2 | Geofencing | None | Configurable geofence polygons + altitude limits | P0 | 🔴 | |
| 5.3 | Health watchdog | ThreadHeartbeat + ThreadWatchdog + systemd WatchdogSec | Hardware watchdog timer (WDT) + software heartbeat monitoring | P1 | 🟢 | PRs #94, #96, #107; per-thread atomic heartbeats, 3-layer architecture |
| 5.4 | Error recovery | Most errors → `spdlog::error()` + exit | Graceful degradation: retry, fallback, safe-state transition | P1 | � | `Result<T,E>` (PR #75), `FaultManager` (PR #63) provide structured error handling; full recovery chains TBD |
| 5.5 | Process heartbeats | Zenoh liveliness tokens (Phase F, #51) | Zenoh liveliness tokens (Phase F, #51) | P2 | 🟢 | PR #57 — 7 tokens active, P7 monitors |
| 5.6 | Redundancy | Single IMU, single GPS | Dual IMU + dual GPS with voting / consistency checks | P2 | 🔴 | |

---

## 6. Compliance & Documentation

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 6.1 | Flight logging | Custom spdlog files | MAVLink-compatible `.ulg` / `.tlog` format for post-flight analysis | P1 | 🔴 | Required for flight review tools |
| 6.2 | Parameter system | JSON config, cold-reboot only | Runtime parameter tuning (MAVLink param protocol) | P2 | 🔴 | |
| 6.3 | Regulatory docs | None | Risk assessment, CONOPS, if targeting Part 107 waiver / BVLOS | P2 | 🔴 | Depends on operational scope |

---

## 7. Sanitizer & Runtime Analysis Coverage

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 7.1 | ASan coverage | All 717 tests pass under ASan | All tests pass under ASan | P1 | 🟢 | Verified locally 2026-03-06; CI leg `shm,asan` + `zenoh,asan` |
| 7.2 | UBSan coverage | All 717 tests pass under UBSan | All tests pass under UBSan | P1 | 🟢 | Verified locally 2026-03-06; CI leg `shm,ubsan` + `zenoh,ubsan` |
| 7.3 | TSan coverage — own code | 560/560 own-code tests pass under TSan | All own-code tests pass under TSan | P1 | 🟢 | SPSC, seqlock, watchdog, process manager all clean |
| 7.4 | TSan coverage — third-party libs | 157 tests excluded (Zenoh/MAVSDK/OpenCV/Liveliness) — pre-built libs not TSan-instrumented | Run all tests under TSan with instrumented libs or targeted suppressions | P1 | 🔴 | See details below |
| 7.5 | Valgrind / Helgrind soak tests | Not implemented | Multi-hour soak runs under Valgrind + Helgrind for leak/race detection | P2 | 🔴 | Catches different bug classes than sanitizers |
| 7.6 | Fault injection testing | Not implemented | Kill/restart processes mid-flight, saturate IPC queues, simulate sensor dropouts | P1 | 🔴 | |

### 7.4 — TSan Third-Party Library Plan

TSan instruments memory accesses at **compile time**. Pre-built shared libraries (zenohc, MAVSDK, OpenCV, gz-transport) are not instrumented, causing:
- **False positives**: TSan sees gaps in the happens-before graph when data crosses the instrumented/uninstrumented boundary
- **False negatives**: Real races inside the library are invisible

**Production resolution path (in priority order):**

1. **Create `tsan_suppressions.txt`** — Document each suppression with rationale. Run TSan on **all** tests with suppressions instead of blanket `-E` exclusions. This catches races in *our* code at library boundaries.
2. **Build MAVSDK from source with TSan** — C++ CMake project, straightforward. Most critical: handles FC comms. (~1 day)
3. **Build gz-transport from source with TSan** — C++ CMake project. Fix or suppress known init races. (~1 day)
4. **Zenoh (Rust)** — Requires `RUSTFLAGS="-Zsanitizer=thread"` + nightly toolchain. Previously attempted (CI-003 in CI_ISSUES.md) — produced opaque-type size mismatches. Revisit when zenohc upstream provides TSan-instrumented builds.
5. **OpenCV** — Massive build (~30 min). DNN thread pool internals are upstream's responsibility. Suppress and rely on our single-threaded detector call pattern.

---

## Resolved Items

*Move items here as they are completed.*

| # | Item | Resolution | Date |
|---|------|------------|------|
| 4.1 | Process supervision | systemd 7-unit architecture (PR #107) + ProcessManager fork+exec supervisor (implementation in PRs #101, #102; ADR in PR #100) + restart policies with dependency graph. `BindsTo=` stop-propagation semantics, `sd_notify(READY=1)`, `WatchdogSec=10s` on P7. | 2026-03-06 |
| 4.4 | Config validation | `ConfigSchema` builder-pattern validation with 7 process schemas (PR #76, Issue #69) | 2026-03-03 |
| 5.3 | Health watchdog | Three-layer watchdog: (1) `ThreadHeartbeat` atomic per-thread heartbeats + `ThreadWatchdog` scanner (PR #94), (2) `ShmThreadHealth` publisher + `ProcessManager` crash recovery (PRs #96, #101, #102), (3) systemd `WatchdogSec` OS-level supervision (PR #107). See [tests/TESTS.md](../tests/TESTS.md) for test counts. | 2026-03-06 |
| 5.5 | Process heartbeats | Zenoh liveliness tokens — 7 tokens active, P7 monitors deaths (PR #57, Phase F) | 2026-03-01 |

---

## How to Use This Document

1. **Before starting a new feature**, scan this list — the feature may overlap
   with a production item that should be addressed together.
2. **When completing a prototype shortcut**, move the row to "Resolved Items"
   with the commit/PR reference and date.
3. **During sprint planning**, use the priority column to decide which items
   to pull in alongside feature work.
4. **Before flight testing**, all P0 items must be 🟢.
5. **Before field deployment**, all P0 + P1 items must be 🟢.
