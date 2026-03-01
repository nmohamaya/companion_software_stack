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
| 3.1 | Kalman tracker | Linear KF, 8D constant-velocity | EKF/UKF with constant-turn-rate model, or IMM | P2 | 🔴 | |
| 3.2 | Track association | Greedy nearest-neighbor | Hungarian (Munkres) O(n³) optimal assignment | P2 | 🔴 | |
| 3.3 | Appearance features | None — position-only matching | DeepSORT / ByteTrack Re-ID vectors | P2 | 🔴 | Reduces ID switches by ~45% |
| 3.4 | Sensor fusion | Weighted average merge | EKF/UKF fusion with per-sensor measurement models | P2 | 🔴 | |
| 3.5 | ISP pipeline | Raw RGB24 passthrough | Bayer demosaic → white balance → gamma → NV12 | P1 | 🔴 | Use NVIDIA ISP on Jetson |

---

## 4. Build & Deployment

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 4.1 | Process supervision | `launch_all.sh` — bash script, manual restart | systemd units with watchdog, auto-restart, dependency ordering | P0 | 🔴 | |
| 4.2 | OTA updates | None — manual scp + rebuild | Mender / SWUpdate / custom updater | P2 | 🔴 | |
| 4.3 | Log management | spdlog to files, no rotation | Log rotation (logrotate or spdlog rotating sink), remote log shipping | P1 | 🔴 | Disk will fill on long missions |
| 4.4 | Config validation | No schema validation on JSON load | JSON Schema validation at startup; reject invalid configs | P1 | 🔴 | |
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
| 5.3 | Health watchdog | `ShmSystemHealth` published at 1 Hz | Hardware watchdog timer (WDT) + software heartbeat monitoring | P1 | 🔴 | |
| 5.4 | Error recovery | Most errors → `spdlog::error()` + exit | Graceful degradation: retry, fallback, safe-state transition | P1 | 🔴 | |
| 5.5 | Process heartbeats | None between processes | Zenoh liveliness tokens (Phase F, #51) | P2 | 🟡 | Planned for next phase |
| 5.6 | Redundancy | Single IMU, single GPS | Dual IMU + dual GPS with voting / consistency checks | P2 | 🔴 | |

---

## 6. Compliance & Documentation

| # | Item | Current (Prototype) | Production Target | Priority | Status | Notes |
|---|------|--------------------|--------------------|----------|--------|-------|
| 6.1 | Flight logging | Custom spdlog files | MAVLink-compatible `.ulg` / `.tlog` format for post-flight analysis | P1 | 🔴 | Required for flight review tools |
| 6.2 | Parameter system | JSON config, cold-reboot only | Runtime parameter tuning (MAVLink param protocol) | P2 | 🔴 | |
| 6.3 | Regulatory docs | None | Risk assessment, CONOPS, if targeting Part 107 waiver / BVLOS | P2 | 🔴 | Depends on operational scope |

---

## Resolved Items

*Move items here as they are completed.*

| # | Item | Resolution | Date |
|---|------|------------|------|
| — | — | — | — |

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
