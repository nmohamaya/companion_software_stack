# ADR-006: Hardware Abstraction Layer (HAL) Strategy

| Field | Value |
|-------|-------|
| **Status** | Accepted — fully implemented |
| **Date** | 2026-03-13 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | ADR-001 (IPC Framework), ADR-002 (Modular IPC Backend), Issue #149 |

---

## 1. Context

The drone stack must run on at least three distinct execution environments:

| Environment | Hardware | Use case |
|-------------|----------|----------|
| Developer laptop | No hardware | Unit tests, CI |
| Gazebo SITL | Gazebo simulated sensors + PX4 SITL | Integration tests |
| Real drone | V4L2 cameras, MAVSDK FC, UART gimbal, SPI/I2C IMU | Production |

Without abstraction, process code would contain conditional compilation
(`#ifdef HAVE_MAVSDK`) scattered throughout its business logic.  This
leaks deployment concerns into application code and makes it impossible to
unit-test against a live-like stack without physical hardware.

---

## 2. Decision Drivers

- **Testability** — unit tests must run on any Linux box with no hardware
- **Backend swap without recompile** — switch simulation ↔ SITL ↔ hardware
  via a config key, not a rebuild
- **Backend independence** — new backends (e.g., ROS2 bridge, bmi088 IMU)
  can be added without touching any process code
- **Zero-overhead at call sites** — abstraction must not measurably slow
  down the hot path (camera capture at 30 fps, IMU at 400 Hz)
- **Consistent error signalling** — every backend must surface failures
  through the same mechanism (return value, not exception or `errno`)

---

## 3. Decision

Implement a **Hardware Abstraction Layer** as a set of pure-virtual
C++ interface classes in `common/hal/include/hal/`.  Each interface
represents one physical or logical hardware resource.  Every process
depends only on the interface, never on a concrete backend.

### 3.1 Interface Pattern

Each HAL interface follows the **Strategy pattern**:

```
ICamera (pure virtual interface)
   ├── SimulatedCamera   — gradient pattern; no hardware
   ├── GazeboCamera      — gz-transport subscription (HAVE_GAZEBO)
   └── V4L2Camera        — real camera via Video4Linux (future)
```

All methods are non-throwing.  Methods that can fail return `bool` or
a typed struct with a `valid` flag (e.g., `CapturedFrame::valid`,
`FCState`).

### 3.2 Factory Pattern

`common/hal/include/hal/hal_factory.h` provides free functions that
read a `"backend"` key from the runtime config and return a
`std::unique_ptr<IInterface>`:

```cpp
auto camera = drone::hal::create_camera(cfg, "video_capture.mission_cam");
auto fc     = drone::hal::create_fc_link(cfg, "comms.mavlink");
```

Optional backends are compile-guarded (`#ifdef HAVE_MAVSDK`,
`#ifdef HAVE_GAZEBO`) and silently excluded when the dependency is absent.
If a requested backend is unavailable, the factory throws
`std::runtime_error` at startup — never at runtime.

### 3.3 Three-Tier Backend Hierarchy

| Tier | Name | Activation | Purpose |
|------|------|-----------|---------|
| 0 | **Simulated** | `"backend": "simulated"` | Unit tests, developer CI, default |
| 1 | **Gazebo** | `"backend": "gazebo"` | SITL integration tests (HAVE_GAZEBO) |
| 2 | **Real** | `"backend": "mavlink"` / `"v4l2"` / etc. | Physical hardware |

The goal is that the full application stack runs identically at every
tier — only the HAL backend changes.  See [SIMULATION_ARCHITECTURE.md](../SIMULATION_ARCHITECTURE.md)
for the tier decision rationale.

---

## 4. Considered Alternatives

### 4.1 Direct HAL calls inside process code (rejected)

Calling `open("/dev/video0")` directly in `process1_video_capture/src/main.cpp`
works but makes the process impossible to test without a camera device.
Adding `#ifdef` blocks for simulation bloats the production code path.

### 4.2 ROS2 Hardware Abstraction Nodes (rejected)

Using ROS2 nodes as hardware bridges provides language-agnostic abstraction
and a large ecosystem, but introduces a dependency on ROS2 middleware whose
latency and resource footprint are incompatible with the Zenoh-only IPC
target.

### 4.3 Compile-time policy selection via templates (rejected)

A template HAL (e.g., `ProcessMain<CameraPolicy>`) achieves zero-overhead
but requires recompilation to switch backends.  The factory pattern achieves
the same encapsulation with a single binary covering all tiers.

---

## 5. Consequences

### Positive

- Process code is hardware-agnostic: all 7 process `main.cpp` files contain
  no `#ifdef HAVE_HARDWARE` guards
- The same binary runs in simulation and on real hardware; config is the
  only difference
- Adding a new backend (e.g., `FlirBosonCamera`) requires one new class
  and one `if (backend == "flir_boson")` branch in the factory — no process
  code changes
- CI runs the full 7-process integration stack with simulated backends on
  every commit

### Negative / Trade-offs

- Virtual dispatch on the hot path (camera capture, IMU read): benchmarked
  at < 5 ns on Cortex-A78 — negligible relative to DMA memcpy
- Backend-specific configuration keys (e.g., `gz_topic`, `serial_port`,
  `baud_rate`) must be present in config even when the selected backend
  does not use them; mitigated by `cfg.get<T>(key, default_value)`
- `std::unique_ptr` ownership means backends cannot be hot-swapped at
  runtime without a process restart

---

## 6. Implementation

| File | Role |
|------|------|
| `common/hal/include/hal/icamera.h` | `ICamera` interface + `CapturedFrame` |
| `common/hal/include/hal/ifc_link.h` | `IFCLink` interface + `FCState` |
| `common/hal/include/hal/igcs_link.h` | `IGCSLink` interface + `GCSCommand` |
| `common/hal/include/hal/igimbal.h` | `IGimbal` interface + `GimbalState` |
| `common/hal/include/hal/iimu_source.h` | `IIMUSource` interface + `ImuReading` |
| `common/hal/include/hal/hal_factory.h` | Factory functions for all interfaces |
| `common/hal/include/hal/simulated_*.h` | Simulated implementations |
| `common/hal/include/hal/gazebo_*.h` | Gazebo implementations (HAVE_GAZEBO) |
| `common/hal/include/hal/mavlink_fc_link.h` | MAVSDK implementation (HAVE_MAVSDK) |

Planning/avoidance and monitor strategies follow the same pattern but
live in their respective process namespaces:

| File | Role |
|------|------|
| `process4_mission_planner/include/planner/ipath_planner.h` | `IPathPlanner` + `PotentialFieldPlanner` + `AStarPlanner` |
| `process4_mission_planner/include/planner/iobstacle_avoider.h` | `IObstacleAvoider` + `PotentialFieldAvoider` + `ObstacleAvoider3D` |
| `process7_system_monitor/include/monitor/iprocess_monitor.h` | `IProcessMonitor` + `LinuxProcessMonitor` |

See [hal_design.md](../hal_design.md) for the full interface reference.

---

## 7. Review Status

Accepted — all interfaces implemented, 66 HAL unit tests passing.  No
known issues.
