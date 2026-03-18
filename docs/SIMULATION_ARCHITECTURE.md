# Simulation Architecture

## Overview

The companion software stack supports **two-tier** testing — from lightweight
pure-simulated runs (Tier 1) to full Gazebo SITL flights (Tier 2). Every
hardware-touching component sits behind a **Hardware Abstraction Layer (HAL)**
interface. Swapping a single `"backend"` key in the JSON config switches
between simulated, Gazebo, or real-hardware implementations at link time.

---

## Architecture Diagram — Simulated vs Actual Code

```mermaid
flowchart TB
    subgraph ACTUAL["Actual Application Code (always runs)"]
        direction TB
        P1["P1 — Video Capture<br/><i>Frame acquisition loop</i>"]
        P2["P2 — Perception<br/><i>Detection + Tracking + Fusion</i>"]
        P3["P3 — SLAM / VIO / Nav<br/><i>IMU pre-integration, VIO backend</i>"]
        P4["P4 — Mission Planner<br/><i>FSM, Path Planning, Geofence,<br/>Fault Manager, Obstacle Avoider</i>"]
        P5["P5 — Comms<br/><i>FC link, GCS link, MAVLink</i>"]
        P6["P6 — Payload Manager<br/><i>Gimbal control, Capture triggers</i>"]
        P7["P7 — System Monitor<br/><i>CPU/GPU/Thermal, Watchdog,<br/>Process Supervisor</i>"]
    end

    subgraph IPC["IPC Layer (POSIX SHM / Zenoh)"]
        direction LR
        SHM1["/drone_mission_cam"]
        SHM2["/detected_objects"]
        SHM3["/slam_pose"]
        SHM4["/mission_status"]
        SHM5["/fc_state"]
        SHM6["/gcs_commands"]
        SHM7["/system_health"]
        SHM8["/trajectory_cmd"]
        SHM9["/fc_commands"]
        SHM10["/mission_upload"]
        SHM11["/fault_overrides"]
    end

    subgraph HAL["Hardware Abstraction Layer (Swappable Backends)"]
        direction TB
        subgraph SIMULATED["Simulated Backends (Tier 1)"]
            SC["SimulatedCamera<br/><i>Synthetic gradient RGB</i>"]
            SI["SimulatedIMU<br/><i>Gaussian noise accel+gyro</i>"]
            ST["SimulatedThermalCamera<br/><i>Warm blob generation</i>"]
            SF["SimulatedFCLink<br/><i>Battery drain, stub FC</i>"]
            SG["SimulatedGCSLink<br/><i>UDP stub</i>"]
            SGim["SimulatedGimbal<br/><i>Slew-rate limited</i>"]
            SD["SimulatedDetector<br/><i>Random 2D detections</i>"]
        end
        subgraph GAZEBO["Gazebo Backends (Tier 2)"]
            GC["GazeboCameraBackend<br/><i>gz-transport subscriber</i>"]
            GI["GazeboIMUBackend<br/><i>gz-transport IMU data</i>"]
        end
        subgraph HARDWARE["Real Hardware Backends"]
            ML["MavlinkFCLink<br/><i>MAVSDK MAVLink v2</i>"]
            HW_CAM["V4L2 / CSI Camera<br/><i>(future)</i>"]
        end
    end

    subgraph TOOLS["Test Tooling"]
        FI["fault_injector CLI<br/><i>Writes to /fault_overrides<br/>sideband channel</i>"]
        SR["run_scenario.sh<br/><i>Launch → Inject → Verify<br/>Transport-aware (SHM / Zenoh)</i>"]
        SCEN["Scenario Configs<br/><i>8 JSON scenario files</i>"]
    end

    subgraph EXTERNAL["External Systems (Tier 2 only)"]
        PX4["PX4 SITL<br/><i>Autopilot simulation</i>"]
        GZ["Gazebo Harmonic<br/><i>Physics + Sensors</i>"]
        WORLD["test_world.sdf<br/><i>6 obstacles (boxes, cylinder, wall)</i>"]
    end

    P1 -->|frames| SHM1
    SHM1 -->|subscribe| P2
    P2 -->|detections| SHM2
    SHM2 -->|subscribe| P4
    P3 -->|pose| SHM3
    SHM3 -->|subscribe| P4
    P4 -->|trajectory| SHM8
    SHM8 -->|subscribe| P5
    P5 -->|fc_state| SHM5
    SHM5 -->|subscribe| P4
    P5 -->|gcs_commands| SHM6
    SHM6 -->|subscribe| P4
    P7 -->|health| SHM7
    SHM7 -->|subscribe| P4

    P1 ---|HAL| SC
    P1 ---|HAL| GC
    P3 ---|HAL| SI
    P3 ---|HAL| GI
    P5 ---|HAL| SF
    P5 ---|HAL| ML
    P5 ---|HAL| SG
    P6 ---|HAL| SGim
    P2 ---|HAL| SD

    FI -.->|inject| SHM11
    SHM11 -.->|override| P5
    SHM11 -.->|override| P7
    SR -.->|orchestrate| FI
    SCEN -.->|configure| SR

    GC ---|gz-transport| GZ
    GI ---|gz-transport| GZ
    ML ---|MAVLink UDP| PX4
    PX4 ---|SDF plugin| GZ
    GZ --- WORLD

    style SIMULATED fill:#2d5a2d,color:#fff
    style GAZEBO fill:#2d4a7a,color:#fff
    style HARDWARE fill:#7a4a2d,color:#fff
    style ACTUAL fill:#1a1a2e,color:#fff
    style TOOLS fill:#4a2d6a,color:#fff
    style EXTERNAL fill:#4a4a4a,color:#fff
    style IPC fill:#2a2a4a,color:#fff
```

---

## Two-Tier Testing Model

| | **Tier 1 — Pure Simulated** | **Tier 2 — Gazebo SITL** |
|---|---|---|
| **Hardware** | None required | GPU recommended |
| **External deps** | None | PX4-Autopilot, Gazebo Harmonic |
| **Camera** | `SimulatedCamera` (gradient) | `GazeboCameraBackend` (rendered) |
| **IMU** | `SimulatedIMU` (noise model) | `GazeboIMUBackend` (physics) |
| **FC Link** | `SimulatedFCLink` (stub) | `MavlinkFCLink` (MAVSDK) |
| **Detector** | `SimulatedDetector` (random) | `SimulatedDetector` or YOLO |
| **Physics** | None (open-loop) | Gazebo (closed-loop) |
| **Speed** | Fast (seconds) | Slow (minutes) |
| **Use case** | Unit/regression, CI, fault injection | Waypoint validation, obstacle avoidance |

### When to Use Each Tier

- **Tier 1**: Run on every commit in CI. Fast, deterministic, no external
  dependencies. Validates fault handling, FSM transitions, IPC flow, and
  config parsing. Use the `fault_injector` to simulate faults.

- **Tier 2**: Run before releases or when validating navigation. Requires
  PX4 + Gazebo installed. Validates closed-loop waypoint following, obstacle
  avoidance with real simulated sensor data, and MAVLink integration.

---

## Component Classification

### Always-Actual Code (never simulated)

These run identically in sim and on hardware:

| Component | Process | Description |
|---|---|---|
| Mission FSM | P4 | State machine: IDLE → PREFLIGHT → TAKEOFF → EXECUTING → RTL → LANDING → COMPLETE |
| FaultManager | P4 | Battery: WARN / RTL / CRIT (3-tier with `FAULT_BATTERY_RTL`), FC link loss, geofence breach |
| Geofence | P4 | Point-in-polygon + altitude checks with warning margin |
| A* Path Planner | P4 | 3D occupancy grid, A* search, path smoothing |
| ObstacleAvoider3D | P4 | Velocity-space potential field with prediction |
| Kalman Tracker | P2 | Multi-object tracking with Hungarian assignment |
| Fusion Engine | P2 | Camera-only sensor fusion |
| VIO Backend | P3 | Visual-inertial odometry with sliding window (sim uses `steady_clock` timestamps) |
| IMU Pre-integrator | P3 | IMU measurement integration between keyframes |
| Watchdog | P7 | Thread heartbeat monitoring, process management |
| IPC Layer | All | SHM SeqLock pub/sub or Zenoh transport |

### HAL-Swappable Backends

| Interface | Simulated | Gazebo | Hardware |
|---|---|---|---|
| `ICamera` | `SimulatedCamera` | `GazeboCameraBackend` | V4L2/CSI (future) |
| `IIMUSource` | `SimulatedIMU` | `GazeboIMUBackend` | Serial IMU (future) |
| `IFCLink` | `SimulatedFCLink` | — | `MavlinkFCLink` |
| `IGCSLink` | `SimulatedGCSLink` | — | UDP GCS (future) |
| `IGimbal` | `SimulatedGimbal` | — | Serial gimbal (future) |
| `IDetector` | `SimulatedDetector` | — | `OpenCVYOLODetector` |

---

## Simulation Setup

### Prerequisites

**Tier 1 (pure-simulated) — no extra dependencies:**
```bash
# Standard build dependencies only
sudo apt install build-essential cmake libspdlog-dev libeigen3-dev nlohmann-json3-dev
```

**Tier 2 (Gazebo SITL):**
```bash
# PX4 Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && make px4_sitl_default

# Gazebo Harmonic
sudo apt install gz-harmonic

# MAVSDK
sudo apt install libmavsdk-dev  # or build from source
```

### Build

```bash
# Tier 1 build (default — all simulated)
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Tier 2 build (with Gazebo + MAVSDK)
cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_GAZEBO=ON \
      -DENABLE_MAVSDK=ON ..
make -j$(nproc)
```

### Configuration

Backend selection is **config-driven**. All backends read from a single JSON:

```json
{
    "video_capture": {
        "mission_cam": {
            "backend": "simulated"    // or "gazebo"
        }
    },
    "comms": {
        "mavlink": {
            "backend": "simulated"    // or "mavlink"
        }
    },
    "mission_planner": {
        "path_planner":      { "backend": "potential_field" },  // or "astar"
        "obstacle_avoider":  { "backend": "potential_field" },  // or "potential_field_3d" (3D variant)
        "geofence": {
            "polygon": [
                {"x": -50, "y": -50},
                {"x":  50, "y": -50},
                {"x":  50, "y":  50},
                {"x": -50, "y":  50}
            ],
            "altitude_ceiling_m": 120.0,
            "warning_margin_m": 5.0
        }
    },
    "fault_manager": {
        "battery_warn_percent": 30.0,
        "battery_rtl_percent": 20.0,
        "battery_crit_percent": 10.0,
        "fc_link_lost_timeout_ms": 3000,
        "fc_link_rtl_timeout_ms": 15000
    }
}
```

Config files in `config/`:
| File | Description |
|---|---|
| `default.json` | All simulated backends (Tier 1) |
| `gazebo_sitl.json` | Gazebo cameras + MAVLink FC (Tier 2) |
| `hardware.json` | Real hardware backends |

---

## How to Run Simulations

### Tier 1 — Pure Simulated (No Gazebo)

**Option A: Direct launch with default config**
```bash
./deploy/launch_all.sh --config config/default.json
```

**Option B: Run a specific scenario**
```bash
# List available scenarios
./tests/run_scenario.sh --list

# Run a single scenario (uses SHM transport by default)
./tests/run_scenario.sh config/scenarios/01_nominal_mission.json

# Run with explicit Zenoh transport
./tests/run_scenario.sh config/scenarios/01_nominal_mission.json --ipc zenoh

# Dry-run (show plan without executing)
./tests/run_scenario.sh config/scenarios/03_battery_degradation.json --dry-run

# Run all Tier 1 scenarios (SHM)
./tests/run_scenario.sh --all --tier 1

# Run all Tier 1 scenarios (Zenoh)
./tests/run_scenario.sh --all --tier 1 --ipc zenoh
```

**Option C: Manual fault injection (while stack is running)**
```bash
# In terminal 1: launch stack
./deploy/launch_all.sh --config config/default.json

# In terminal 2: inject faults manually
./build/bin/fault_injector battery 25          # trigger battery WARN
./build/bin/fault_injector battery 15          # trigger battery RTL
./build/bin/fault_injector fc_disconnect       # simulate FC link loss
./build/bin/fault_injector gcs_command rtl     # send RTL via GCS
./build/bin/fault_injector thermal_zone 3      # critical thermal
./build/bin/fault_injector mission_upload config/scenarios/data/upload_waypoints.json
```

### Tier 2 — Gazebo SITL

```bash
# Terminal 1: Launch PX4 + Gazebo + companion stack
PX4_DIR=~/PX4-Autopilot ./deploy/launch_gazebo.sh

# Terminal 2 (optional): Inject faults
./build/bin/fault_injector battery 18

# Or run the integration test
./tests/test_gazebo_integration.sh
```

---

## Fault Injection Tool

The `fault_injector` CLI uses a **sideband override channel**
(`/fault_overrides`) rather than writing directly to production SHM
segments. This avoids race conditions — the producing processes (P5
Comms, P7 System Monitor) continue their normal publish loops and
**merge** overrides into the values they publish.

### How It Works

```mermaid
flowchart LR
    FI[fault_injector] -->|write| FO["/fault_overrides<br/>(ShmFaultOverrides)"]
    FO -->|read| P5[P5 Comms]
    FO -->|read| P7[P7 System Monitor]
    P5 -->|publish| FC["/fc_state<br/>(merged)"]
    P7 -->|publish| SH["/system_health<br/>(merged)"]
    FC --> P4[P4 Mission Planner]
    SH --> P4

    style FO fill:#4a2d6a,color:#fff
```

- **Sentinel values**: Override fields default to `-1` (no override).
  Only non-sentinel fields are applied.
- **Sequence counter**: Incremented on each write so consumers can
  detect fresh overrides vs stale values.
- **FC link loss**: When `fc_connected = 0`, P5 freezes the FC
  heartbeat timestamp so the FaultManager's stale-heartbeat check
  fires correctly.
- **Transport-agnostic**: The `/fault_overrides` segment is always
  POSIX SHM (even when the main transport is Zenoh), so fault
  injection works identically across transports.

### ShmFaultOverrides Struct

```cpp
struct alignas(64) ShmFaultOverrides {
    float    battery_percent;   // <0 = no override
    float    battery_voltage;   // <0 = no override
    int32_t  fc_connected;      // <0 = no override, 0/1 = state
    int32_t  thermal_zone;      // <0 = no override, 0-3 = zone
    float    cpu_temp_override;  // <0 = no override
    uint64_t sequence;          // incremented by injector
};
```

### Available Commands

| Command | Override Field | Description |
|---|---|---|
| `battery <percent>` | `battery_percent` | Override FC battery level |
| `fc_disconnect` | `fc_connected = 0` | Freeze FC heartbeat → link-loss detection |
| `fc_reconnect` | `fc_connected = 1` | Release frozen timestamp |
| `gcs_command <cmd> [p1 p2 p3]` | `/gcs_commands` (direct) | Inject GCS command |
| `thermal_zone <0-3>` | `thermal_zone` | Override thermal zone |
| `mission_upload <json>` | `/mission_upload` + `/gcs_commands` | Upload new waypoints |
| `sequence <json>` | Various | Execute timed fault sequence |

### Timed Sequence Format

```json
{
    "steps": [
        {"delay_s": 10, "action": "battery", "value": 28.0},
        {"delay_s": 5,  "action": "fc_disconnect"},
        {"delay_s": 20, "action": "fc_reconnect"},
        {"delay_s": 2,  "action": "gcs_command", "command": "rtl"}
    ]
}
```

---

## Test Scenarios

Eight pre-defined scenarios in `config/scenarios/`:

| # | Scenario | Tier | Gazebo | What it Tests |
|---|---|---|---|---|
| 01 | Nominal Mission | 1 | No | Basic 4-waypoint flight, landing |
| 02 | Obstacle Avoidance | 2 | Yes | A* planner through obstacle field |
| 03 | Battery Degradation | 1 | No | 3-tier: WARN → RTL → EMERGENCY_LAND |
| 04 | FC Link Loss | 1 | No | LOITER → RTL contingency |
| 05 | Geofence Breach | 1 | No | Polygon violation → RTL |
| 06 | Mission Upload | 1 | No | Mid-flight waypoint upload via GCS |
| 07 | Thermal Throttle | 1 | No | Zone escalation (0→1→2→3→0) |
| 08 | Full Stack Stress | 1 | No | Concurrent faults, high rates |

### Scenario JSON Structure

Each scenario file contains:

```json
{
    "scenario": {
        "name": "...",
        "description": "...",
        "tier": 1,
        "timeout_s": 120,
        "requires_gazebo": false
    },
    "config_overrides": { },
    "fault_sequence": {
        "steps": [ ]
    },
    "pass_criteria": {
        "log_contains": ["..."],
        "log_must_not_contain": ["..."],
        "processes_alive": ["..."],
        "shm_segments_exist": ["..."]
    },
    "manual_controls": {
        "notes": "What parameters can be adjusted"
    }
}
```

### Manual Controls

Every scenario includes a `manual_controls` section that documents
which parameters can be adjusted for manual testing:

- **Waypoints**: Edit `config_overrides.mission_planner.waypoints`
- **Speed**: Modify `cruise_speed_mps` within the documented range
- **Thresholds**: Adjust `fault_manager.*_percent` values
- **Fault timing**: Change `delay_s` in `fault_sequence.steps`
- **Geofence polygon**: Modify vertices in `geofence.polygon`
- **Backend selection**: Switch `path_planner.backend` between
  `"potential_field"` and `"astar"`
- **Obstacle avoider backend**: `"potential_field"` or
  `"potential_field_3d"` (3D variant used by stress test)
- **IPC transport**: `--ipc shm` or `--ipc zenoh` on the
  `run_scenario.sh` command line (default: from base config)

---

## Data Flow Diagram

```mermaid
sequenceDiagram
    participant Runner as run_scenario.sh
    participant Stack as Companion Stack<br/>(7 processes)
    participant FI as fault_injector
    participant FO as /fault_overrides

    Runner->>Runner: Merge base config + scenario overrides
    Runner->>Runner: Resolve effective IPC (SHM or Zenoh)
    Runner->>Stack: Launch with merged config + --ipc flag

    alt SHM transport
        Runner->>Runner: Wait for /dev/shm segments
    else Zenoh transport
        Runner->>Runner: Wait for process log output
    end

    Note over Runner,FI: Fault Injection Phase

    Runner->>FI: Execute fault sequence
    FI->>FO: Write battery_percent=28
    FO-->>Stack: P5 reads override, publishes merged /fc_state
    Stack->>Stack: FaultManager: WARN

    FI->>FO: Write battery_percent=18
    FO-->>Stack: P5 reads override, publishes merged /fc_state
    Stack->>Stack: FaultManager: RTL (FAULT_BATTERY_RTL)

    Note over Runner: Verification Phase

    Runner->>Runner: Check logs for expected patterns
    alt SHM transport
        Runner->>Runner: Check SHM segments exist
    end
    Runner->>Runner: Check processes alive
    Runner->>Runner: Report PASS/FAIL
```

---

## File Layout

```
companion_software_stack/
├── config/
│   ├── default.json                  # Tier 1 config (all simulated)
│   ├── gazebo_sitl.json              # Tier 2 config (Gazebo + MAVLink)
│   └── scenarios/
│       ├── 01_nominal_mission.json
│       ├── 02_obstacle_avoidance.json
│       ├── 03_battery_degradation.json
│       ├── 04_fc_link_loss.json
│       ├── 05_geofence_breach.json
│       ├── 06_mission_upload.json
│       ├── 07_thermal_throttle.json
│       ├── 08_full_stack_stress.json
│       └── data/
│           └── upload_waypoints.json
├── common/hal/include/hal/
│   ├── icamera.h                     # HAL interface
│   ├── simulated_camera.h            # Tier 1 backend
│   ├── gazebo_camera.h               # Tier 2 backend
│   └── hal_factory.h                 # Config-driven factory
├── tools/
│   └── fault_injector/
│       ├── CMakeLists.txt
│       └── main.cpp                  # Fault injection CLI (sideband /fault_overrides)
├── tests/
│   ├── run_scenario.sh               # Scenario runner (transport-aware, --ipc flag)
│   └── test_gazebo_integration.sh    # Gazebo smoke test
├── drone_logs/
│   └── scenarios/                    # Scenario runner output logs
└── docs/
    └── SIMULATION_ARCHITECTURE.md    # This file
```
