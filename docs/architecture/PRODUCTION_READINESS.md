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
| 1.9 | VIO backend | `SimulatedVIOBackend` — target-following + full VIO pipeline | ORB-SLAM3 / VINS-Fusion integration | P1 | 🟡 | `IVIOBackend` interface with 3 backends; covariance-based quality |
| 1.10 | Path planner | `DStarLitePlanner` — 3D grid, incremental D* Lite search, obstacle inflation, two-layer occupancy (HD-map static + camera TTL 3 s), BFS start-escape | RRT* | P2 | 🟢 | D* Lite + HD-map verified in Gazebo SITL scenario 02 (7/7 WP, 0 collisions); incremental replanning handles dynamic environments |
| 1.11 | Obstacle avoider | `ObstacleAvoider3D` — XYZ repulsive field, velocity prediction (`potential_field_3d`) | VFH+ / 3D-VFH | P2 | 🟡 | 3D variant verified in stress scenario (PR #123) + scenario 02 with HD-map (Fix #35). PotentialFieldAvoider (2D) removed in Issue #207 |
| 1.12 | LiDAR | Removed (Phase 1A, PR #117) | Point cloud driver (Livox, Ouster, etc.) | P2 | 🔴 | Need HAL `ILiDAR` interface; all simulated code and data type fields removed |
| 1.13 | Radar | `SimulatedRadar` + `GazeboRadarBackend` (gpu_lidar → radar detections) | mmWave radar driver (TI AWR1843, etc.) | P2 | 🟡 | `IRadar` interface exists (Issue #209); SimulatedRadar + GazeboRadarBackend + UKF fusion implemented (Issues #210, #212). Real hardware driver TBD |
| 1.14 | Gimbal auto-tracking | `compute_auto_track()` — world→body yaw-only transform, pitch/yaw from bearing | Full 3-axis world→body transform (roll/pitch/yaw) | P2 | 🟡 | `AutoTrackConfig` / `AutoTrackResult` implemented; yaw-only rotation assumes near-level flight |

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
| 3.4 | Sensor fusion | Weighted average merge | EKF/UKF fusion with per-sensor measurement models | P2 | � | `UKFFusionEngine` with per-object UKF (PR #117); radar measurement model added (Issues #210, #212) — camera + radar fusion operational |
| 3.5 | ISP pipeline | Raw RGB24 passthrough | Bayer demosaic → white balance → gamma → NV12 | P1 | 🔴 | Use NVIDIA ISP on Jetson |
| 3.6 | Radar-primary perception | Camera bearing + radar range UKF fusion (`UKFFusionEngine`); radar-only orphan tracks; size estimation from bbox + range | Production-tuned association gates, multi-radar support | P2 | 🟡 | Epic #237; camera=bearing, radar=range, covariance-driven trust. Verified in Gazebo SITL |
| 3.7 | Covariance-based VIO health | `trace(P_position)` from IMU pre-integrator thresholded into NOMINAL/DEGRADED/LOST | Adaptive thresholds per flight phase; multi-sensor fusion quality | P2 | 🟢 | Defaults: good ≤ 0.1, degraded ≤ 1.0. Drives `FAULT_VIO_DEGRADED` / `FAULT_VIO_LOST` in FaultManager |
| 3.8 | D* Lite z-band collapse under grid congestion | `color_contour` detection noise causes occupied cell spikes (5→696 in seconds) near obstacles. When flight-altitude z-plane is fully blocked, D* drops to z=0 (ground level) and routes through the obstacle's physical location. Observed: drone commanded through green object at ground level in scenario 18 Run 3 (2026-04-11). `z_band_cells=0` allows all z-levels. | Resolves automatically with a production perception pipeline (YOLOv8/real radar) — stable detections eliminate the occupied cell oscillation that triggers the z-drop. Do NOT add z-band constraints to work around this. | P1 | 🔴 | Blocked by 1.4 (detector backend) and 1.13 (real radar). See domain-knowledge.md. Non-deterministic: 2/3 runs pass, 1/3 collides depending on detection timing near green object |

---

## 4. Build & Deployment

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 4.1 | Process supervision | systemd units (production) / ProcessManager (`--supervised` mode) | systemd units with watchdog, auto-restart, dependency ordering | P0 | 🟢 | PR #107 (systemd), Epic #88 (watchdog) |
| 4.2 | OTA updates | None — manual scp + rebuild | Mender / SWUpdate / custom updater | P2 | 🔴 | |
| 4.3 | Log management | spdlog rotating file sink + console/JSON sink; scenario logs in `drone_logs/scenarios/` | Log rotation (logrotate or spdlog rotating sink), remote log shipping | P1 | 🟡 | Rotating sink done; remote shipping TBD |
| 4.4 | Config validation | JSON Schema validation at startup; reject invalid configs | `ConfigSchema` builder-pattern validation at startup (7 schemas) | P1 | 🟢 | PR #76 — Issue #69 |
| 4.5 | Crash reporting | Process exits with return 1 | Core dump collection, crash telemetry upload | P2 | 🔴 | |
| 4.6 | Cross-compilation | Native x86 build only | Cross-compile for aarch64 (Jetson) | P0 | 🔴 | Need CMake toolchain file |
| 4.7 | Release builds | Debug/RelWithDebInfo | `-DCMAKE_BUILD_TYPE=Release` with `-O2 -DNDEBUG` + stripped binaries | P1 | 🔴 | |
| 4.8 | Hardware config | `config/hardware.json` has placeholder values | Board-specific config per drone variant (Jetson Nano/Orin/Xavier) | P1 | 🔴 | |

---

## 5. Safety & Reliability

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 5.1 | Failsafe logic | FaultManager: 3-tier battery (WARN/RTL/LAND with `FAULT_BATTERY_RTL`), FC link-loss (LOITER→RTL with timestamp freeze), geofence breach → RTL | Comprehensive failsafe: GPS loss, IMU failure, motor fault, sensor dropout | P0 | 🟡 | Battery + FC link + geofence done (PRs #119, #123); GPS/IMU/motor TBD |
| 5.2 | Geofencing | Ray-casting point-in-polygon + altitude ceiling + warning margin | Configurable geofence polygons + altitude limits | P0 | 🟢 | PR #119 (Phase 3); verified via scenario 05 (PR #123) |
| 5.3 | Health watchdog | ThreadHeartbeat + ThreadWatchdog + systemd WatchdogSec | Hardware watchdog timer (WDT) + software heartbeat monitoring | P1 | 🟢 | PRs #94, #96, #107; per-thread atomic heartbeats, 3-layer architecture |
| 5.4 | Error recovery | Most errors → `spdlog::error()` + exit | Graceful degradation: retry, fallback, safe-state transition | P1 | 🟡 | `Result<T,E>` (PR #75), `FaultManager` (PR #63, #119, #123) provide structured error handling; full recovery chains TBD |
| 5.5 | Process heartbeats | Zenoh liveliness tokens (Phase F, #51) | Zenoh liveliness tokens (Phase F, #51) | P2 | 🟢 | PR #57 — 7 tokens active, P7 monitors |
| 5.6 | Redundancy | Single IMU, single GPS | Dual IMU + dual GPS with voting / consistency checks | P2 | 🔴 | |
| 5.7 | Collision recovery | `COLLISION_RECOVERY` FSM state — hover, climb, skip waypoint, resume navigation | Configurable recovery strategy per mission type; multi-obstacle re-route | P2 | 🟢 | MissionState 9; waypoint skip on collision detection |

---

## 6. Compliance & Documentation

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 6.1 | Flight logging | `FlightRecorder` — ring-buffer IPC capture to `.flog` binary files + `flight_replay` tool for offline replay with wire version validation | MAVLink-compatible `.ulg` / `.tlog` format for flight review tools; `.flog` covers internal IPC replay | P1 | 🟡 | Issue #40; ring buffer (default 64 MB), RAII file management, `kWireVersion` check on replay. `.ulg` export TBD |
| 6.2 | Parameter system | JSON config, cold-reboot only | Runtime parameter tuning (MAVLink param protocol) | P2 | 🔴 | |
| 6.3 | Regulatory docs | None | Risk assessment, CONOPS, if targeting Part 107 waiver / BVLOS | P2 | 🔴 | Depends on operational scope |

---

## 7. Sanitizer & Runtime Analysis Coverage

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 7.1 | ASan coverage | All 880 tests pass under ASan | All tests pass under ASan | P1 | 🟢 | Verified locally 2026-03-06; CI leg `shm,asan` + `zenoh,asan` |
| 7.2 | UBSan coverage | All 880 tests pass under UBSan | All tests pass under UBSan | P1 | 🟢 | Verified locally 2026-03-06; CI leg `shm,ubsan` + `zenoh,ubsan` |
| 7.3 | TSan coverage — own code | 560/560 own-code tests pass under TSan | All own-code tests pass under TSan | P1 | 🟢 | SPSC, seqlock, watchdog, process manager all clean |
| 7.4 | TSan coverage — third-party libs | 157 tests excluded (Zenoh/MAVSDK/OpenCV/Liveliness) — pre-built libs not TSan-instrumented | Run all tests under TSan with instrumented libs or targeted suppressions | P1 | 🔴 | See details below |
| 7.5 | Valgrind / Helgrind soak tests | Not implemented | Multi-hour soak runs under Valgrind + Helgrind for leak/race detection | P2 | 🔴 | Catches different bug classes than sanitizers |
| 7.6 | Fault injection testing | `fault_injector` CLI with sideband `/fault_overrides` channel; **8/8 Tier 1 scenarios on Gazebo SITL + Zenoh (89/89 checks)** | Kill/restart processes mid-flight, saturate IPC queues, simulate sensor dropouts | P1 | 🟡 | Fault injection done (PR #123); process-kill + IPC saturation TBD |

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
| 5.3 | Health watchdog | Three-layer watchdog: (1) `ThreadHeartbeat` atomic per-thread heartbeats + `ThreadWatchdog` scanner (PR #94), (2) `ThreadHealthPublisher` publishing `ThreadHealth` messages + `ProcessManager` crash recovery (PRs #96, #101, #102), (3) systemd `WatchdogSec` OS-level supervision (PR #107). See [tests/TESTS.md](../tests/TESTS.md) for test counts. | 2026-03-06 |
| 5.2 | Geofencing | Ray-casting point-in-polygon + altitude ceiling + warning margin (PR #119, Phase 3). Verified end-to-end via geofence breach scenario (PR #123). | 2026-03-08 |
| 5.6 | HD-map obstacle avoidance | Two-layer A* occupancy grid (permanent HD-map static layer + 3 s TTL camera confirmation layer). Verified in Gazebo SITL scenario 02: 7/7 waypoints reached, 0 collisions (Fix #35). | 2026-03-09 |
| 5.7 | Proximity collision detection | NAVIGATE loop checks drone ENU position against all HD-map obstacles each tick (`radius_m + 0.5 m` XY, `height_m + 0.5 m` Z). Throttled 2 s cooldown. Supplements disarm-based crash check (Fix #36). | 2026-03-09 |
| 5.5 | Process heartbeats | Zenoh liveliness tokens — 7 tokens active, P7 monitors deaths (PR #57, Phase F) | 2026-03-01 |

---

## 8. Hardware Platform Portability

The codebase currently targets **Linux/aarch64 (NVIDIA Jetson Orin)**. If the compute
platform changes (e.g., Qualcomm QRB5165, NXP i.MX8, Intel x86, or a custom SBC),
the following platform-specific assumptions need attention.

| # | Area | Current Assumption | Files Affected | Impact | Notes |
|---|------|-------------------|----------------|--------|-------|
| 8.1 | `M_PI` constant | Uses `M_PI` from `<cmath>` (POSIX extension, not ISO C++) | `ukf_fusion_engine.h`, `test_gazebo_radar.cpp`, several perception files | **Low** — works on all Linux variants | Replace with `std::numbers::pi_v<float>` (C++20) or a local `constexpr` if targeting MSVC or strict ISO mode |
| 8.2 | `/proc` filesystem | Reads `/proc/stat`, `/proc/meminfo`, `/proc/self/exe`, `/proc/self/fd` for CPU, memory, binary path, FD enumeration | `sys_info.h`, `process_manager.h`, `process7_system_monitor/src/main.cpp` | **High** — breaks on non-Linux | Qualcomm/NXP Linux BSPs have `/proc`; QNX/VxWorks do not. Need `ISystemInfo` abstraction if leaving Linux |
| 8.3 | Thermal sysfs paths | Reads `/sys/devices/virtual/thermal/thermal_zone0/temp` (Jetson-specific) with fallback to `/sys/class/thermal/thermal_zone0/temp` | `sys_info.h` | **High** — path varies per SoC | Qualcomm uses different thermal zone numbering; NXP has `/sys/class/thermal/thermal_zone1/temp`. Make path configurable |
| 8.4 | Thread affinity | `pthread_setaffinity_np()`, `CPU_ZERO`/`CPU_SET` (GNU extensions, not POSIX) | `realtime.h` | **Medium** — not portable | macOS/QNX use different APIs. Qualcomm/NXP Linux BSPs support these but core topology differs (big.LITTLE) |
| 8.5 | `pthread_setname_np()` | GNU extension for naming threads | `realtime.h` | **Low** — gracefully degradable | Wrap in `#ifdef __GLIBC__` or use `prctl(PR_SET_NAME)` |
| 8.6 | Process management | `fork()`/`execvp()`/`waitpid()`/`kill()` POSIX model | `process_manager.h` | **Low on Linux variants** | All Linux BSPs (Qualcomm, NXP, Intel) support POSIX. Only breaks if moving to RTOS |
| 8.7 | Wire format endianness | Raw `reinterpret_cast` on IPC structs; magic `0x4E4F5244` assumes little-endian | `wire_format.h`, `ipc_types.h` | **Medium** — breaks on big-endian | All current targets (ARM64, x86) are LE. Only matters if GCS runs on a BE system or if targeting PowerPC |
| 8.8 | `__attribute__((packed))` | GCC-specific struct packing on `WireHeader` | `wire_format.h` | **Low** — Clang supports it too | Use `#pragma pack(push, 1)` for MSVC compatibility if needed |
| 8.9 | Cache-line alignment | `alignas(64)` on SPSC ring indices, IPC structs | `spsc_ring.h`, `ipc_types.h` | **Low** — correct but may be suboptimal | 64 bytes is correct for Cortex-A (Qualcomm/NXP) and x86. Not a correctness issue |
| 8.10 | systemd integration | `sd_notify()`, `sd_watchdog_enabled()` guarded by `#ifdef HAVE_SYSTEMD` | `sd_notify.h`, all process `main.cpp` files | **None** — already optional | Gracefully no-ops when disabled; Qualcomm/NXP Linux typically use systemd |
| 8.11 | GPU acceleration | No CUDA/TensorRT/libargus code yet (listed as future in 1.4, 3.5) | N/A | **High when added** — vendor lock-in | Future TensorRT detector locks to NVIDIA. Qualcomm equivalent: SNPE/QNN. NXP: eIQ/TFLite. Plan for `IInferenceEngine` HAL |
| 8.12 | Camera API | `ICamera` abstraction exists; V4L2 backend planned | `icamera.h`, `hal_factory.h` | **Low** — V4L2 works on all Linux | Qualcomm may prefer `libcamera` or proprietary ISP pipeline. NXP supports V4L2 natively |
| 8.13 | Cross-compilation | Currently native x86/aarch64 build only | `CMakeLists.txt` | **Medium** — need toolchain files | Each SoC vendor provides a cross-toolchain; CMake toolchain file needed per target |
| 8.14 | GPU rendering / EGL | `gpu_lidar` and `depth_camera` sensors require GPU ray-casting via EGL. On dual-GPU systems (integrated + discrete NVIDIA), the EGL loader defaults to Mesa DRI2 instead of the NVIDIA ICD, causing silent sensor failure | `deploy/launch_gazebo.sh`, any Gazebo launch script | **High** — sensors silently produce no data | See detailed notes below. Currently mitigated by forcing NVIDIA EGL env vars in `launch_gazebo.sh`. On Jetson (single GPU), this is a non-issue. On x86 with NVIDIA dGPU, the env vars are required. On Intel-only or AMD systems, `gpu_lidar` requires Mesa EGL to work correctly — do not set NVIDIA vars |

### 8.14 — GPU Rendering / EGL Platform Notes

**Problem discovered:** During Gazebo SITL testing with `gpu_lidar` (used as radar geometric backbone), the sensor silently failed to publish data. Cameras and IMU worked fine. Root cause: the EGL loader on a dual-GPU laptop (Intel iGPU + NVIDIA dGPU) selected Mesa DRI2 instead of the NVIDIA EGL ICD. The ogre2 rendering engine requires a working EGL context for GPU ray-casting; without it, `gpu_lidar` and `depth_camera` sensors produce zero frames with no error message.

**Current mitigation** (in `deploy/launch_gazebo.sh`):

```bash
if command -v nvidia-smi &>/dev/null; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
        export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    fi
fi
```

**Hardware platform matrix:**

| Platform | GPU | EGL Status | Action Required |
| -------- | --- | ---------- | --------------- |
| **Dev laptop** (x86, dual GPU) | Intel iGPU + NVIDIA dGPU | Broken without env vars | Force NVIDIA EGL ICD (current fix) |
| **NVIDIA Jetson Orin** (aarch64) | Tegra GPU (single) | Works natively | None — NVIDIA is the only EGL provider |
| **Intel NUC / x86 server** | Intel iGPU only | Mesa EGL | Do NOT set NVIDIA vars; ensure `mesa-libEGL` is installed |
| **AMD GPU system** | AMD dGPU | Mesa EGL (AMDGPU) | Do NOT set NVIDIA vars; ensure `mesa-libEGL` is installed |
| **Qualcomm QRB5165** (aarch64) | Adreno GPU | Qualcomm EGL | Needs Qualcomm EGL ICD; untested with Gazebo |
| **Headless server** (no GPU) | None | Software rendering | `gpu_lidar` will not work; fall back to `lidar` (CPU) or `SimulatedRadar` HAL |

**When moving to new hardware:**

1. Run `eglinfo` (from `mesa-utils-extra`) to verify which EGL vendor is active
2. Test `gpu_lidar` with a minimal world: `gz sim -r -s --headless-rendering test_world.sdf` and check `gz topic -i -t /radar_lidar/scan` for publisher presence
3. If no publisher appears, check the Gazebo server log for `libEGL warning: pci id for fd` messages
4. Set the appropriate EGL vendor environment variables for the target GPU
5. On Jetson, the NVIDIA JetPack SDK configures EGL correctly out of the box — no intervention needed

**Long-term recommendation:** Add a GPU/EGL health check to the launch script that verifies `gpu_lidar` has at least one publisher within 5 seconds of Gazebo startup. If not, log a diagnostic message pointing to the EGL configuration. This was filed as GitHub Issue #217.

### Platform Migration Priority

If switching from Jetson Orin to another SoC, address in this order:

1. **Thermal sysfs paths** (8.3) — immediate crash/misread on different SoC thermal zone layout
2. **GPU/inference acceleration** (8.11) — TensorRT models won't run; need SNPE/QNN (Qualcomm) or eIQ (NXP)
3. **Cross-compilation toolchain** (8.13) — can't build without it
4. **Thread affinity topology** (8.4) — big.LITTLE core assignment may differ
5. **Camera ISP pipeline** (8.12) — Jetson libargus vs Qualcomm Camera2 vs NXP V4L2
6. **GPU rendering / EGL** (8.14) — `gpu_lidar` silent failure on dual-GPU systems; verify EGL vendor on new hardware

Items 8.1, 8.5–8.10 are low-risk on any Linux-based SoC and can be addressed opportunistically.

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
