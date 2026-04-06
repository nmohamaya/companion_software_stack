# Companion Software Stack — Strategic Evolution Plan

> Generated: 2026-03-10 | Based on deep analysis of all 7 processes, 21 open GitHub issues, and full architecture review.

## Table of Contents

- [Current State Assessment](#current-state-assessment)
- [Pillar 1 — State-of-the-Art Algorithms](#pillar-1--state-of-the-art-algorithms)
- [Pillar 2 — Modularity & Replaceability](#pillar-2--modularity--replaceability)
- [Pillar 3 — Simulation → Hardware Transition](#pillar-3--simulation--hardware-transition)
- [Pillar 4 — What Makes This Stack Stand Out](#pillar-4--what-makes-this-stack-stand-out)
- [Proposed Issue Roadmap — Prioritized](#proposed-issue-roadmap--prioritized)

---

## Current State Assessment

**What we have**: A ~3500 LOC C++17 autonomous drone stack with 7 processes, 21 virtual interface classes, Zenoh IPC (sole backend), 1259 unit tests (see [tests/TESTS.md](../../tests/TESTS.md)), 25 scenarios (20 Tier 1 + 5 Tier 2), 75% coverage, full Gazebo SITL closed-loop flight, and a 3-layer watchdog. The architecture uses strategy pattern everywhere, config-driven backend selection, zero recompilation to swap implementations.

### Simulation vs. Real Hardware Maturity

| Process | Simulation Maturity | Real Hardware Maturity |
|---------|--------------------|-----------------------|
| P1 Video Capture | Full (Gazebo cameras) | 0% — no V4L2, no ISP |
| P2 Perception | High (YOLOv8 CPU, ByteTrack tracker, UKF) | 10% — no TensorRT, no radar |
| P3 SLAM/VIO/Nav | Medium (IMU preintegrator real, rest simulated) | 5% — no real VO, no factor graph |
| P4 Mission Planner | High (D* Lite, 3D avoider, FSM, geofence) | 95% — hardware-agnostic |
| P5 Comms | High (MAVSDK FC link works) | 70% — serial config exists, no real GCS |
| P6 Payload | Low (stub gimbal only) | 0% — no real driver |
| P7 System Monitor | High (Linux /proc, supervisor, watchdog) | 80% — needs Jetson thermal zones |

### Interface Inventory (21 Virtual Base Classes)

| Process | Interface | Implementations | Simulated | Real | Gazebo |
|---------|-----------|-----------------|-----------|------|--------|
| HAL | `ICamera` | 3 | SimulatedCamera | V4L2 (TODO #32) | GazeboCameraBackend |
| HAL | `IIMUSource` | 2 | SimulatedIMU | BMI088 (TODO #37) | GazeboIMUBackend |
| HAL | `IFCLink` | 2 | SimulatedFCLink | — | MavlinkFCLink (MAVSDK) |
| HAL | `IGCSLink` | 1 | SimulatedGCSLink | UDP (TODO #34) | — |
| HAL | `IGimbal` | 1 | SimulatedGimbal | SIYI/PWM (TODO #42) | — |
| P2 | `IDetector` | 3 | SimulatedDetector | ColorContour, YOLOv8 (OpenCV) | — |
| P2 | `ITracker` | 1 | — | KalmanBoxTracker | — |
| P2 | `IFusionEngine` | 2 | — | CameraOnly, UKF | — |
| P3 | `IFeatureExtractor` | 1 | SimulatedFeatureExtractor | ORB/SIFT (TODO) | — |
| P3 | `IStereoMatcher` | 1 | SimulatedStereoMatcher | Real (TODO) | — |
| P3 | `IVIOBackend` | 3 | SimulatedVIOBackend | MSCKF (TODO) | GazeboVIOBackend, GazeboFullVIOBackend |
| P4 | `IPathPlanner` | 1 | — | DStarLitePlanner | — |
| P4 | `IObstacleAvoider` | 1 | — | ObstacleAvoider3D | — |
| P7 | `IProcessMonitor` | 1 | — | LinuxProcessMonitor | — |
| IPC | `IPublisher<T>` | 2 | — | ShmPublisher | ZenohPublisher |
| IPC | `ISubscriber<T>` | 2 | — | ShmSubscriber | ZenohSubscriber |

### Factory Pattern

All backends selected at runtime via JSON config + factory functions:

| Factory | Location | Backends |
|---------|----------|----------|
| `create_camera()` | hal_factory.h | simulated, gazebo, (v4l2 TODO) |
| `create_imu_source()` | hal_factory.h | simulated, gazebo, (bmi088 TODO) |
| `create_fc_link()` | hal_factory.h | simulated, mavlink |
| `create_gcs_link()` | hal_factory.h | simulated, (udp TODO) |
| `create_gimbal()` | hal_factory.h | simulated, (siyi/pwm TODO) |
| `create_detector()` | detector_interface.h | simulated, color_contour, yolov8 |
| `create_message_bus()` | message_bus_factory.h | shm, zenoh |

---

## Pillar 1 — State-of-the-Art Algorithms

### P2: Perception — The Biggest Gap

**Current:** OpenCV DNN YOLO on CPU (~200ms inference on Jetson). Single-frame detection, no temporal fusion for small objects.

**Target state:**

| Capability | Implementation | Why It Stands Out |
|------------|---------------|-------------------|
| **GPU Inference** | TensorRT YOLOv8n/YOLOv11n (FP16) | 5-10ms on Orin NX vs 200ms CPU. Non-negotiable for real-time |
| **Tracking** | ByteTrack done (Issue #163); consider **BoT-SORT** for Re-ID | BoT-SORT adds appearance Re-ID for occlusion recovery beyond IoU matching |
| **Multi-sensor fusion** | **EKF fusion** with camera + ultrasonic range (TFMini) | Monocular depth is unreliable. A $15 rangefinder gives ground-truth depth for the primary threat axis |
| **Temporal model** | **Object velocity estimation** via track history + Kalman prediction | Velocity-obstacle (VO) avoidance reacts to where objects *will be* |
| **Segmentation fallback** | **MobileSAM** or **FastSAM** as `IDetector` backend | Detects novel obstacles (wires, branches) that YOLO misses |
| **Radar** | Integrate mmWave radar (e.g. TI AWR1843) as new sensor input to UKF | Provides velocity + range in all weather; complements camera for robust obstacle detection |

**Interface changes needed:** Zero. `IDetector`, `ITracker`, `IFusionEngine` already exist. Each is a new concrete implementation.

### P3: SLAM/VIO — Critical Path to GPS-Denied Flight

**Current:** Real IMU preintegrator (Forster 2017), but visual frontend and stereo matcher are simulated. No optimization backend.

| Capability | Implementation | Why |
|------------|---------------|-----|
| **Visual odometry** | **ORB-SLAM3** (stereo-inertial) as `IVIOBackend` | Battle-tested, handles loop closure, runs on Orin. GPLv3 |
| **Alternative VO** | **VINS-Fusion** as second `IVIOBackend` | BSD license, stereo + IMU tightly-coupled |
| **Fallback chain** | ORB-SLAM3 → VINS-Fusion → GPS-only → dead-reckon (IMU-only) | Graceful degradation when features are lost |
| **Map persistence** | Serialize ORB-SLAM3 map for revisit | Enables revisit without re-mapping |
| **GPS fusion** | **EKF2 external vision** via PX4 `VISUAL_POSITION_ESTIMATE` | Feed VIO pose to PX4's internal EKF. Seamless indoor↔outdoor |

### P4: Mission Planner — Already Strong, Push Further

| Capability | Implementation | Rationale |
|------------|---------------|-----------|
| **Velocity obstacles** | ORCA/HRVO as `IObstacleAvoider` | Predicts collision with *moving* objects |
| **Dynamic replanning** | D* Lite for known-map incremental updates | Re-plans without recomputing from scratch |
| **Mission scripting** | Lua or JSON DSL with conditionals | "If object detected, orbit. If battery < 30%, skip WPs." |
| **Polygon geofence** | Keep-in/keep-out polygon + altitude zones | Current cylinder geofence is basic |
| **D* Lite timeout guard** | `max_search_time_ms` (default 50ms) in `GridPlannerConfig` | Prevents main loop blocking on pathological grids |

### P1, P5, P6: Filling the Gaps

| Process | Upgrade | Implementation |
|---------|---------|---------------|
| **P1** | V4L2 camera backend + ISP | `V4L2Camera : ICamera` — V4L2 MMAP capture, debayer, sensor configs (IMX219, IMX477) |
| **P1** | H.265 video streaming | GStreamer: `v4l2src → nvvidconv → nvv4l2h265enc → rtph265pay → udpsink` |
| **P5** | UDP GCS link | `UDPGCSLink : IGCSLink` — MAVLink v2 telemetry to QGroundControl |
| **P5** | Companion-to-companion mesh | Zenoh multicast discovery + swarm pose sharing (IPC layer already supports this) |
| **P6** | Real gimbal driver | `SIYIGimbal : IGimbal` (UART) or `PWMGimbal : IGimbal` (PCA9685 I2C) |
| **P7** | Jetson thermal zones | Read `/sys/devices/virtual/thermal/thermal_zone*/type` + temp |

---

## Pillar 2 — Modularity & Replaceability

### Already Modular (Keep As-Is)

- **IPC layer**: `MessageBus` with SHM/Zenoh backends. Adding iceoryx2 or DDS = one file.
- **HAL layer**: All 5 interfaces with factory creation from config.
- **Perception pipeline**: `IDetector` → `ITracker` → `IFusionEngine` independently swappable.
- **Path planning**: `IPathPlanner` / `IObstacleAvoider` with two backends each.

### Needs Modularity Work

| Gap | Current State | Fix |
|-----|--------------|-----|
| **P4 main loop** | ~730 lines monolithic `main()` | Extract into `MissionStateMachine` class with per-state handlers |
| **Safety constants** | `kCollisionMarginM=0.5`, etc. hardcoded | Move to `fault_manager` config section |
| **VIO fallback** | Single `IVIOBackend` instance | `FallbackVIOBackend` wrapping 2-3 backends with priority chain |
| **Detector ensemble** | Single `IDetector` per config | `EnsembleDetector` running 2+ detectors with NMS merge |
| **Config schema** | No validation at startup | JSON Schema per process section → fail fast on typos |

---

## Pillar 3 — Simulation → Hardware Transition

### Test Pyramid

```
                    ┌──────────────────────┐
                    │  Real Flight Tests   │  ← Jetson + Pixhawk + real props
                    │  (Phase 9+)          │     Manual go/no-go
                    ├──────────────────────┤
                    │  HITL Tests          │  ← Jetson + PX4 HITL
                    │  (New)               │     Real code, simulated physics
                    ├──────────────────────┤
                    │  Gazebo SITL         │  ← x86 or Jetson
                    │  (8 scenarios, 80    │     PX4 + Gazebo + full stack
                    │   checks) ✅         │
                    ├──────────────────────┤
                    │  E2E Smoke           │  ← x86 or Jetson
                    │  (42 checks) ✅      │     All 7 processes, no physics
                    ├──────────────────────┤
                    │  Unit Tests          │  ← x86 or Jetson (cross-compiled)
                    │  (1259 tests) ✅     │     Isolated, <2 min
                    └──────────────────────┘
```

### Minimum Viable Hardware

- Jetson Orin NX 16GB (~$500)
- Pixhawk 6C or Holybro X500 v2 frame kit (~$300)
- 2x ArduCam IMX219 stereo pair (~$50)
- TFMini-S LiDAR rangefinder (~$40)
- Slamtec RPLIDAR A1 for 2D scan (~$100, optional)

### Phase A — Desk Testing (No Propellers)

| Step | What | How to Test | Issue |
|------|------|------------|-------|
| A.1 | Cross-compile for aarch64 | `cmake -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-jetson.cmake` → unit tests on Jetson | #82 |
| A.2 | V4L2 camera captures frames | `video_capture --config config/hardware.json` → verify real image data | #32 |
| A.3 | Serial MAVLink to Pixhawk | `comms --config config/hardware.json` → verify heartbeat, battery, GPS | #25 |
| A.4 | TensorRT detector on GPU | `perception --config config/hardware.json` → verify <10ms inference | #33 |
| A.5 | Full stack desk test | `launch_all.sh --config config/hardware.json` → all 7 healthy | #30 |

### Phase B — Tethered Flight

| Step | What | Validation |
|------|------|-----------|
| B.1 | ARM + TAKEOFF + LAND | Stack sends ARM → TAKEOFF(3m) → LAND. Tether prevents flyaway |
| B.2 | Simple hover (30s) | Verify pose estimation, health stays nominal |
| B.3 | Single waypoint (2m) | Verify trajectory tracking, no false obstacle triggers |

### Phase C — Free Flight

| Step | What | Validation |
|------|------|-----------|
| C.1 | Triangle mission (3 WPs, 5m legs, 3m AGL) | Full mission with real cameras |
| C.2 | Obstacle avoidance (place box in path) | Detection + avoidance in reality |
| C.3 | GPS-denied (indoor) | VIO-only navigation (requires ORB-SLAM3) |

### Cross-Compilation Strategy

Preferred: Docker-based. `Dockerfile.jetson` pulls NVIDIA L4T base image and builds natively, avoiding sysroot issues with CUDA, TensorRT, GStreamer.

```cmake
# cmake/aarch64-jetson.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /opt/jetson-sysroot)
```

---

## Pillar 4 — What Makes This Stack Stand Out

### 1. Graceful Degradation Chain (Partially Done, Unique if Completed)

```
Full capability
  │ Camera fails → rangefinder-only avoidance
  │ Rangefinder fails → slow speed, increase margins
  │ VIO fails → GPS-only (outdoor) or hover-and-wait (indoor)
  │ GPS fails → VIO-only or RTL via dead reckoning
  │ FC link lost → loiter → RTL → land (done ✅)
  │ Battery critical → emergency land (done ✅)
  ▼
Minimum safe state: land immediately
```

### 2. Process-Level Isolation with Restart (Done)

Supervisor + process graph + restart cascade. Perception crash → restart in <500ms while mission continues degraded.

### 3. Middleware-Agnostic IPC (Done)

`MessageBus` abstraction cleaner than most ROS2 stacks. Path to DDS or iceoryx2 is trivial. No vendor lock-in.

### 4. Novel Features to Build

| Feature | Why It Matters | Difficulty |
|---------|---------------|------------|
| **Flight Data Recorder** | Record all SHM messages for debugging + ML training | Medium (Issue #40) |
| **Anomaly detection** | EWMA on CPU/temp/latency, alert on *trends* not just thresholds | Easy |
| **Sim-to-real autotuner** | Monte Carlo Gazebo runs to auto-tune avoidance parameters | Hard but novel |
| **Visual mission builder** | Web UI (React + WebSocket) via Zenoh for live state + editing | Large, high-impact |

### 5. Documentation as Differentiator

Add:
- Architecture Decision Records (ADRs)
- Integration guide: "I have a Pixhawk and Jetson, how do I fly"
- Auto-generated API reference from interface classes

---

## Proposed Issue Roadmap — Prioritized

### Phase 9: First Safe Flight (Hardware Contact)

| Issue | Title | Priority | Sim-Testable? |
|-------|-------|----------|--------------|
| #82 | Cross-compilation toolchain (aarch64/Jetson) | P0 | Yes (Docker QEMU) |
| #81 | Dockerfile for reproducible builds | P0 | Yes |
| #32 | V4L2Camera HAL backend | P0 | Partial |
| #30 | Pre-flight check script | P0 | Yes |
| NEW | Serial MAVLink validation test bench | P0 | Partial |
| NEW | Jetson thermal zone enumeration in P7 | P1 | No |

### Phase 10: Real Perception Pipeline

| Issue | Title | Priority | Sim-Testable? |
|-------|-------|----------|--------------|
| #33 | TensorRT YOLOv8 detector backend | P0 | Partial |
| #128 | Optimize perception latency | P0 | Yes |
| ~~NEW~~ | ~~ByteTrack tracker upgrade~~ (done — Issue #163, SORT removed #205) | — | — |
| NEW | Rangefinder depth fusion in UKF | P1 | Yes |
| NEW | Ensemble detector (YOLO + radar + contour) | P2 | Yes |

### Phase 11: GPS-Denied Navigation

| Issue | Title | Priority | Sim-Testable? |
|-------|-------|----------|--------------|
| #37 | Real VIO backend (ORB-SLAM3 / VINS-Fusion) | P0 | Partial |
| #38 | Stereo camera calibration pipeline | P0 | Yes |
| #39 | VIO/GPS fusion via PX4 external vision | P0 | Yes |
| NEW | VIO fallback chain | P1 | Yes |
| NEW | Map persistence / relocalization | P2 | Yes |

### Phase 12: Production Hardening

| Issue | Title | Priority | Sim-Testable? |
|-------|-------|----------|--------------|
| #40 | Flight data recorder + replay | P0 | Yes |
| #35 | H.265 video streaming | P1 | Partial |
| #34 | UDP GCS link (QGroundControl) | P1 | Yes |
| #42 | Gimbal driver (SIYI / PWM) | P2 | Partial |
| #41 | Contingency fault tree | P1 | Yes |
| #126 | Zenoh-Only IPC (remove legacy SHM) | P2 | Yes |
| NEW | Mission scripting DSL | P2 | Yes |
| NEW | Polygon geofence zones | P1 | Yes |

### Ongoing / Cross-Cutting

| Issue | Title | Priority |
|-------|-------|----------|
| #125 | SHM segment ownership cleanup | P2 |
| #124 | fault_injector via MessageBus | P2 |
| #129 | PX4 exit teardown fix | P1 |
| ~~NEW~~ | ~~A\* search timeout guard~~ (removed — D\* Lite has timeout built-in) | — |
| NEW | Mission planner main() refactor into FSM class | P1 |
| NEW | JSON Schema config validation | P2 |
| NEW | Architecture Decision Records | P2 |

---

## Summary

**Priority sequence:**
1. **Get it running on Jetson** (cross-compile + Docker + V4L2) — everything else is blocked on this
2. **TensorRT perception** — transforms detection from demo to real-time
3. **Real VIO** — GPS-denied flight makes a companion computer worth having
4. **FDR + fault tree** — safety layer that makes it trustworthy

**Sim-testable fraction: ~70%.** Most algorithm work can be fully developed and tested in simulation before touching hardware.
