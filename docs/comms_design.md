# Process 5 — Comms: Design Document

> **Scope**: Detailed design of the Communications process (`process5_comms`).
> This document covers flight-controller and ground-control-station communication,
> thread architecture, HAL backend selection, and fault injection support.

---

## Table of Contents

1. [Overview](#overview)
2. [Thread Architecture](#thread-architecture)
3. [IPC Channels](#ipc-channels)
4. [Component: IFCLink (Flight Controller)](#component-ifclink-flight-controller)
5. [Component: IGCSLink (Ground Control Station)](#component-igcslink-ground-control-station)
6. [FC Command Protocol](#fc-command-protocol)
7. [Fault Injection](#fault-injection)
8. [Main Loop Detail](#main-loop-detail)
9. [Configuration Reference](#configuration-reference)
10. [Testing](#testing)
11. [Known Limitations](#known-limitations)

---

## Overview

Process 5 bridges the companion computer with the flight controller (PX4/ArduPilot)
and the ground control station (QGroundControl or similar). It runs **5 threads** in total:
4 communication threads plus the main health thread.

Key responsibilities:
- **FC receive:** Polls flight controller for heartbeat/state (battery, GPS, mode, arm status)
- **FC transmit:** Forwards trajectory commands and discrete FC commands (ARM, TAKEOFF, RTL, LAND, SET_MODE)
- **GCS receive:** Polls GCS for operator commands (RTL, LAND, MISSION uploads)
- **GCS transmit:** Sends telemetry (pose, battery, mission state) to GCS

---

## Thread Architecture

| Thread | Rate | Direction | Description |
|--------|------|-----------|-------------|
| `fc_rx_thread` | 10 Hz | FC → Bus | Polls `IFCLink::receive_state()`, publishes `ShmFCState`, applies fault overrides |
| `fc_tx_thread` | 20 Hz | Bus → FC | Subscribes to trajectory + FC commands, deduplicates, sends to FC |
| `gcs_rx_thread` | 2 Hz | GCS → Bus | Polls `IGCSLink::poll_command()`, publishes `ShmGCSCommand` / `ShmMissionUpload` |
| `gcs_tx_thread` | 2 Hz | Bus → GCS | Subscribes to pose + mission status + FC state, sends telemetry to GCS |
| Main thread | 1 Hz | — | Health log + `ThreadHealthPublisher` + systemd watchdog notify |

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Comms (P5)                                 │
│                                                                     │
│  Flight Controller Side               GCS Side                     │
│  ─────────────────────               ────────                      │
│                                                                     │
│  ┌──────────┐  receive    ┌────────┐       ┌──────────┐  poll      │
│  │ IFCLink  │────state───►│fc_rx   │       │gcs_rx    │◄──command──│
│  │ (HAL)    │             │(10 Hz) │       │(2 Hz)    │  IGCSLink  │
│  │          │             └───┬────┘       └───┬──────┘  (HAL)     │
│  │          │                 │                 │                    │
│  │          │           ShmFCState        ShmGCSCommand             │
│  │          │           (publish)        ShmMissionUpload           │
│  │          │                                  (publish)            │
│  │          │                                                       │
│  │          │  send_*     ┌────────┐       ┌──────────┐ send_telem │
│  │          │◄──commands──│fc_tx   │       │gcs_tx    │───────────►│
│  │          │◄──traj──────│(20 Hz) │       │(2 Hz)    │  IGCSLink  │
│  └──────────┘             └───┬────┘       └───┬──────┘            │
│                               │                 │                   │
│                         ShmTrajectoryCmd   ShmPose                  │
│                         ShmFCCommand       ShmMissionStatus         │
│                           (subscribe)      ShmFCState               │
│                                              (subscribe)            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## IPC Channels

### Subscriptions (inputs)

| Channel | Type | Source | Thread |
|---------|------|--------|--------|
| `/trajectory_cmd` | `ShmTrajectoryCmd` | P4 | fc_tx |
| `/fc_commands` | `ShmFCCommand` | P4 | fc_tx |
| `/slam_pose` | `ShmPose` | P3 | gcs_tx |
| `/mission_status` | `ShmMissionStatus` | P4 | gcs_tx |
| `/fc_state` | `ShmFCState` | Self (fc_rx) | gcs_tx |
| `/fault_overrides` | `ShmFaultOverrides` | Test harness | fc_rx |

### Publications (outputs)

| Channel | Type | Consumers | Thread |
|---------|------|-----------|--------|
| `/fc_state` | `ShmFCState` | P4, P7 | fc_rx |
| `/gcs_commands` | `ShmGCSCommand` | P4 | gcs_rx |
| `/mission_upload` | `ShmMissionUpload` | P4 | gcs_rx |
| `/thread_health/comms` | `ShmThreadHealth` | P7 | main |

---

## Component: IFCLink (Flight Controller)

- **Interface:** [`ifc_link.h`](../common/hal/include/hal/ifc_link.h)

### Interface Methods

```cpp
class IFCLink {
    virtual bool    open(const std::string& port, int baud) = 0;
    virtual void    close() = 0;
    virtual bool    is_connected() const = 0;
    virtual bool    send_trajectory(float vx, float vy, float vz, float yaw) = 0;
    virtual bool    send_arm(bool arm) = 0;
    virtual bool    send_mode(uint8_t mode) = 0;
    virtual bool    send_takeoff(float altitude_m) = 0;
    virtual FCState receive_state() = 0;
    virtual std::string name() const = 0;
};
```

### FCState Structure

```cpp
struct FCState {
    uint64_t timestamp_ns;
    float    battery_voltage;      // V (nominal 16.4)
    float    battery_current;      // A
    float    battery_percent;      // 0–100
    float    altitude_rel;         // metres AGL
    float    ground_speed;         // m/s
    uint8_t  satellites;           // GPS satellite count
    uint8_t  flight_mode;          // 0=STAB, 1=GUIDED, 2=AUTO, 3=RTL
    bool     armed;
};
```

### Backends

| Backend | Config value | Class | Description |
|---------|-------------|-------|-------------|
| Simulated (HAL) | `"simulated"` | `SimulatedFCLink` | Mutex-guarded state, battery drain 0.05%/s, instant takeoff |
| MAVSDK | `"mavlink"` | `MavlinkFCLink` | Real MAVLink via MAVSDK library (requires `HAVE_MAVSDK`) |

**SimulatedFCLink** behaviour:
- Battery drains linearly: `100 − elapsed_s × 0.05`
- Ground speed mirrors last `send_trajectory()` magnitude: `√(vx² + vy²)`
- Satellites vary sinusoidally: `12 + sin(t × 0.1) × 3`
- `send_takeoff()` instantly sets `altitude_rel` and switches to AUTO mode

### Legacy: MavlinkSim

[`mavlink_sim.h`](../process5_comms/include/comms/mavlink_sim.h) is a legacy simulated FC that
predates the HAL. It has identical behaviour to `SimulatedFCLink` but is not used in production
(P5 `main.cpp` uses the HAL factory path).

---

## Component: IGCSLink (Ground Control Station)

- **Interface:** [`igcs_link.h`](../common/hal/include/hal/igcs_link.h)

### Interface Methods

```cpp
class IGCSLink {
    virtual bool       open(const std::string& addr, int port) = 0;
    virtual void       close() = 0;
    virtual bool       is_connected() const = 0;
    virtual bool       send_telemetry(float lat, float lon, float alt,
                                      float battery, uint8_t state) = 0;
    virtual GCSCommand poll_command() = 0;
    virtual std::string name() const = 0;
};
```

### GCS Command Types

| Type | Value | Meaning |
|------|-------|---------|
| `NONE` | 0 | No command (poll returned empty) |
| `RTL` | 1 | Return to launch |
| `LAND` | 2 | Land immediately |
| `MISSION` | 3 | Mission upload (waypoints in params) |
| `PARAM_SET` | 4 | Parameter change |

### Backend: SimulatedGCSLink

- Only one backend exists (`"simulated"`)
- Logs telemetry every 50th call
- **Sends RTL command after 300 seconds** of runtime (simulates operator watchdog)
- Mutex-protected for thread safety

### Legacy: GCSLink

[`gcs_link.h`](../process5_comms/include/comms/gcs_link.h) is a legacy GCS simulator.
Differs from `SimulatedGCSLink` in RTL timing (120s vs 300s).

---

## FC Command Protocol

The `fc_tx_thread` processes two IPC streams:

### Trajectory Commands (`ShmTrajectoryCmd`)
- Contains `vx`, `vy`, `vz`, `yaw_rate` velocities
- **Deduplication:** Tracks `last_traj_ts` — only forwards if timestamp differs
- Calls `IFCLink::send_trajectory()` with velocity components

### Discrete FC Commands (`ShmFCCommand`)
- Contains `type` enum and optional `param1` (e.g., takeoff altitude)
- **Deduplication:** Tracks `last_cmd_seq` — only forwards if sequence number differs
- Command dispatch:

| Command Type | IFCLink Call | Notes |
|-------------|-------------|-------|
| `ARM` | `send_arm(true)` | — |
| `DISARM` | `send_arm(false)` | — |
| `TAKEOFF` | `send_takeoff(param1)` | `param1` = altitude in metres |
| `SET_MODE` | `send_mode((uint8_t)param1)` | 0=STAB, 1=GUIDED, 2=AUTO, 3=RTL |
| `RTL` | `send_mode(3)` | Mapped to mode change |
| `LAND` | `send_mode(3)` | Mapped to RTL mode |

---

## Fault Injection

The `fc_rx_thread` reads `ShmFaultOverrides` to support integration testing:

### Override: Battery Level
- If `override_battery` is set, `battery_percent` in `ShmFCState` is overwritten
  with the override value before publishing
- Used to test low-battery fault escalation

### Override: FC Link Loss
- If `override_fc_link_loss` is set, `timestamp_ns` is frozen to a past value
- The Mission Planner's FaultManager detects the stale timestamp as an FC link timeout
- Allows testing of link-loss → RTL escalation without physical disconnection

---

## Main Loop Detail

### Startup Sequence
1. Load config from `config/default.json`
2. Create `MessageBus` via factory (SHM or Zenoh)
3. Set up all publishers and subscribers (6 IPC channels)
4. Create `IFCLink` via `drone::hal::create_fc_link(cfg, "comms.mavlink")`
5. Create `IGCSLink` via `drone::hal::create_gcs_link(cfg, "comms.gcs")`
6. Open FC link with configured port + baud rate
7. Open GCS link with configured address + port
8. Create `ThreadWatchdog` (4 threads: fc_rx, fc_tx, gcs_rx, gcs_tx)
9. Launch all 4 communication threads
10. Enter main loop: health log + watchdog check at 1 Hz

### Shutdown
- `g_running` atomic set to `false` by signal handler (SIGTERM/SIGINT)
- All threads check `g_running` on each loop iteration
- Threads join with `thread.join()`
- FC and GCS links closed via RAII (destructor calls `close()`)

---

## Configuration Reference

### `comms.mavlink.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | FC backend (`"simulated"` or `"mavlink"`) |
| `uri` | string | `"udp://:14540"` | MAVSDK connection URI |
| `timeout_ms` | int | 8000 | Connection timeout (ms) |
| `serial_port` | string | `"/dev/ttyTHS1"` | UART device for serial MAVLink |
| `baud_rate` | int | 921600 | Serial baud rate |

### `comms.gcs.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | GCS backend (only `"simulated"` currently) |
| `udp_port` | int | 14550 | GCS UDP listen port |

---

## Testing

| Test File | Tests | Coverage |
|-----------|-------|----------|
| [`test_comms.cpp`](../tests/test_comms.cpp) | 13 | MavlinkSim + GCSLink: connect/disconnect, state simulation, command types |
| [`test_mavlink_fc_link.cpp`](../tests/test_mavlink_fc_link.cpp) | 14 | MavlinkFCLink HAL: connection, timeout, factory, MAVSDK-conditional |
| **Total** | **27** | |

### Key Test Scenarios

- **Battery drain:** Verify `battery_percent` decreases over time but stays in [0, 100]
- **Arm/disarm round-trip:** `send_arm()` → `receive_state().armed` reflects change
- **Mode change round-trip:** `send_mode()` → `receive_state().flight_mode` reflects change
- **GCS RTL timing:** `poll_command()` returns empty before timeout, RTL after
- **MAVSDK fallback:** Without `HAVE_MAVSDK`, `"mavlink"` backend throws, `"simulated"` works

---

## Known Limitations

1. **No real GCS backend:** Only simulated GCS exists. Real UDP/TCP GCS link is planned.
2. **No MAVLink parsing in simulation:** `MavlinkSim` and `SimulatedFCLink` don't parse
   actual MAVLink packets — they simulate state directly.
3. **Fire-and-forget commands:** `send_arm()`, `send_mode()` have no acknowledgement
   or retry logic. If the FC doesn't respond, the command is lost.
4. **Telemetry deduplication is timestamp-only:** If two different trajectory commands
   share the same timestamp (extremely unlikely), one is dropped.
5. **GCS telemetry waits for subscribers:** `gcs_tx_thread` blocks on subscriber
   readiness before first send, which can delay initial telemetry.
