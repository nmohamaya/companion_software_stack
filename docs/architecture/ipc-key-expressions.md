# IPC Key Expressions — Topic Naming Convention

> **Status:** Phase B complete — 10 low-bandwidth channels migrated.
> Video channels deferred to Phase C.

## Overview

All inter-process communication (IPC) topics follow a hierarchical naming
convention based on **Zenoh key expressions**. Legacy POSIX SHM segment names
(e.g. `/slam_pose`) are automatically mapped to key expressions by
`ZenohMessageBus::to_key_expr()`.

Processes use the `MessageBusFactory` and need not know which backend is active:

```cpp
#include "ipc/message_bus_factory.h"
auto bus = drone::ipc::create_message_bus(cfg.get<std::string>("ipc_backend", "shm"));
auto pub = drone::ipc::bus_advertise<ShmPose>(bus, shm_names::SLAM_POSE);
auto sub = drone::ipc::bus_subscribe<ShmPose>(bus, shm_names::SLAM_POSE);
```

## Naming Scheme

```
drone/{domain}/{topic}
```

| Domain | Process | Description |
|--------|---------|-------------|
| `slam/` | P3 SLAM/VIO/Nav | Visual-inertial odometry, pose estimation |
| `perception/` | P2 Perception | Object detection, tracking, fusion |
| `comms/` | P5 Comms | Flight controller + ground station link |
| `mission/` | P4 Mission Planner | FSM, path planning, commands |
| `payload/` | P6 Payload Manager | Gimbal, camera control |
| `monitor/` | P7 System Monitor | CPU, memory, thermal, watchdog |
| `video/` | P1 Video Capture | Camera frames (Phase C) |

## Complete Channel Table

| # | Key Expression | Legacy SHM Name | Struct Type | Size | Rate | Publisher | Subscriber(s) | Phase |
|---|---------------|-----------------|-------------|------|------|-----------|---------------|-------|
| 1 | `drone/slam/pose` | `/slam_pose` | `ShmPose` | ~120 B | 100 Hz | P3 | P4, P5 | B |
| 2 | `drone/comms/fc_state` | `/fc_state` | `ShmFCState` | ~256 B | 10 Hz | P5 | P4, P7 | B |
| 3 | `drone/comms/fc_command` | `/fc_commands` | `ShmFCCommand` | ~128 B | 10 Hz | P4 | P5 | B |
| 4 | `drone/mission/status` | `/mission_status` | `ShmMissionStatus` | ~512 B | 5 Hz | P4 | P5, P7 | B |
| 5 | `drone/mission/trajectory` | `/trajectory_cmd` | `ShmTrajectoryCmd` | ~256 B | 10 Hz | P4 | P5 | B |
| 6 | `drone/comms/gcs_command` | `/gcs_commands` | `ShmGCSCommand` | ~128 B | 1 Hz | P5 | P4 | B |
| 7 | `drone/perception/detections` | `/detected_objects` | `ShmDetectedObjectList` | ~2 KB | 30 Hz | P2 | P4 | B |
| 8 | `drone/mission/payload_command` | `/payload_commands` | `ShmPayloadCommand` | ~64 B | 1 Hz | P4 | P6 | B |
| 9 | `drone/payload/status` | `/payload_status` | `ShmPayloadStatus` | ~128 B | 1 Hz | P6 | P4, P7 | B |
| 10 | `drone/monitor/health` | `/system_health` | `ShmSystemHealth` | ~512 B | 1 Hz | P7 | P4 | B |
| 11 | `drone/video/frame` | `/drone_mission_cam` | `ShmVideoFrame` | ~6 MB | 30 Hz | P1 | P2 | C |
| 12 | `drone/video/stereo_frame` | `/drone_stereo_cam` | `ShmStereoFrame` | ~600 KB | 30 Hz | P1 | P3 | C |
| 13 | `mission/upload` | `/mission_upload` | `ShmMissionUpload` | ~2 KB | event | P5 | P4 | B |
| 14 | — (always POSIX SHM) | `/fault_overrides` | `ShmFaultOverrides` | 64 B | event | fault_injector | P5, P7 | — |

**Total low-bandwidth:** ~100 KB/s (channels 1–10)
**Total high-bandwidth:** ~190 MB/s (channels 11–12, Phase C with Zenoh SHM zero-copy)

## Topic Mapping Rules

`ZenohMessageBus::to_key_expr()` applies the following rules in order:

1. **Exact match:** If the name is in the compile-time mapping table, return the mapped key expression.
2. **Passthrough:** If the name has no leading `/`, it's already a Zenoh key expression — pass through unchanged.
3. **Fallback:** Strip leading `/`, replace `_` with `/`. Example: `/some_topic` → `some/topic`.

The mapping table is defined in
[zenoh_message_bus.h](../common/ipc/include/ipc/zenoh_message_bus.h).

## How to Add a New Channel

1. **Define the struct** in `common/ipc/include/ipc/shm_types.h`
   (must be `trivially_copyable`).

2. **Add a SHM name constant** in `shm_names` namespace:
   ```cpp
   constexpr const char* MY_CHANNEL = "/my_channel";
   ```

3. **Add the key expression mapping** in `ZenohMessageBus::to_key_expr()`:
   ```cpp
   {"/my_channel", "drone/domain/my_topic"},
   ```

4. **Use the factory** in your process:
   ```cpp
   auto pub = drone::ipc::bus_advertise<MyStruct>(bus, shm_names::MY_CHANNEL);
   auto sub = drone::ipc::bus_subscribe<MyStruct>(bus, shm_names::MY_CHANNEL);
   ```

5. **Add a migration test** in `tests/test_zenoh_ipc.cpp`:
   ```cpp
   TEST(ZenohMigration, MyChannel_RoundTrip) { ... }
   ```

6. **Update this table** with the new channel entry.

## Wildcard Subscriptions

Zenoh key expressions support wildcards:

```cpp
// Subscribe to ALL channels
auto sub = bus.subscribe("drone/**");

// Subscribe to all mission-domain channels
auto sub = bus.subscribe("drone/mission/*");

// Subscribe to all status channels
auto sub = bus.subscribe("drone/*/status");
```

> **Note:** Wildcard subscriptions are not yet exposed through the
> `MessageBusFactory` helpers. Use `ZenohMessageBus` directly.

## Backend Selection

Set `ipc_backend` in `config/default.json`:

```json
{
    "ipc_backend": "shm"      // or "zenoh" (requires -DENABLE_ZENOH=ON)
}
```

Or per-process via command line (if `Config` supports overrides).

When `ipc_backend=shm`:
- Topics use POSIX SHM segment names (e.g. `/slam_pose`)
- SeqLock read/write, zero-copy within same host

When `ipc_backend=zenoh`:
- Topics auto-mapped to key expressions (e.g. `drone/slam/pose`)
- Zenoh handles local SHM zero-copy + network transport
- Peer discovery is automatic (multicast scouting)

## Process → Channel Matrix

| Process | Publishes | Subscribes |
|---------|-----------|------------|
| P1 Video Capture | `video/frame`, `video/stereo_frame` | — |
| P2 Perception | `perception/detections` | `video/frame` |
| P3 SLAM/VIO/Nav | `slam/pose` | `video/stereo_frame` |
| P4 Mission Planner | `mission/status`, `mission/trajectory`, `mission/payload_command`, `comms/fc_command` | `slam/pose`, `perception/detections`, `comms/fc_state`, `comms/gcs_command`, `payload/status`, `monitor/health` |
| P5 Comms | `comms/fc_state`, `comms/gcs_command`, `comms/mission_upload` | `mission/trajectory`, `comms/fc_command`, `slam/pose`, `mission/status` |
| P6 Payload Manager | `payload/status` | `mission/payload_command` |
| P7 System Monitor | `monitor/health` | `comms/fc_state` (optional) |

> **Note:** `/fault_overrides` is always POSIX SHM (never Zenoh). The
> `fault_injector` tool writes to it; P5 (Comms) and P7 (System Monitor)
> read overrides and merge them into `/fc_state` and `/system_health`
> respectively. See [SIMULATION_ARCHITECTURE.md](SIMULATION_ARCHITECTURE.md)
> for details.
