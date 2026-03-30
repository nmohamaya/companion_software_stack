# IPC Key Expressions — Topic Naming Convention

> **Status:** Phase B complete — 18 low-bandwidth channels migrated (including radar and thread health).
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
| 15 | `radar/detections`* | `/radar_detections` | `RadarDetectionList` | ~10 KB | 20 Hz | P2 | P4 | B |
| 16 | `drone/thread/health/video/capture`* | `/drone_thread_health_video_capture` | `ThreadHealth` | ~1 KB | 1 Hz | P1 | P7 | B |
| 17 | `drone/thread/health/perception`* | `/drone_thread_health_perception` | `ThreadHealth` | ~1 KB | 1 Hz | P2 | P7 | B |
| 18 | `drone/thread/health/slam/vio/nav`* | `/drone_thread_health_slam_vio_nav` | `ThreadHealth` | ~1 KB | 1 Hz | P3 | P7 | B |
| 19 | `drone/thread/health/mission/planner`* | `/drone_thread_health_mission_planner` | `ThreadHealth` | ~1 KB | 1 Hz | P4 | P7 | B |
| 20 | `drone/thread/health/comms`* | `/drone_thread_health_comms` | `ThreadHealth` | ~1 KB | 1 Hz | P5 | P7 | B |
| 21 | `drone/thread/health/payload/manager`* | `/drone_thread_health_payload_manager` | `ThreadHealth` | ~1 KB | 1 Hz | P6 | P7 | B |
| 22 | `drone/thread/health/system/monitor`* | `/drone_thread_health_system_monitor` | `ThreadHealth` | ~1 KB | 1 Hz | P7 | P7 | B |

\* Not in the explicit `to_key_expr()` mapping table — these use the fallback rule (strip leading `/`, replace `_` with `/`).

**Total low-bandwidth:** ~107 KB/s (channels 1–10, 15–22)
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

## Liveliness Tokens (Process Crash Detection)

Each process declares a **Zenoh liveliness token** on startup using
`LivelinessToken` from
[zenoh_liveliness.h](../common/ipc/include/ipc/zenoh_liveliness.h).
The token key expression follows the pattern:

```
drone/alive/{process_name}
```

For example, P1 declares `drone/alive/video_capture`, P5 declares
`drone/alive/comms`, etc.

When a process exits (gracefully or via crash), Zenoh automatically
revokes the token, producing a `DELETE` sample on the key expression.
P7 (System Monitor) uses `LivelinessMonitor` to watch `drone/alive/**`
and receives immediate callbacks:

- **PUT** — process came alive (token declared)
- **DELETE** — process died (token revoked / session dropped)

```cpp
// Declaring a token (in each process):
auto token = drone::ipc::LivelinessToken("video_capture");

// Monitoring all tokens (in P7):
auto monitor = drone::ipc::LivelinessMonitor(
    [](const std::string& name) { /* on_alive */ },
    [](const std::string& name) { /* on_death */ }
);
auto alive = monitor.get_alive_processes();  // snapshot
bool ok    = monitor.is_alive("comms");      // point query
```

| Token Key Expression | Process |
|---------------------|---------|
| `drone/alive/video_capture` | P1 |
| `drone/alive/perception` | P2 |
| `drone/alive/slam_vio_nav` | P3 |
| `drone/alive/mission_planner` | P4 |
| `drone/alive/comms` | P5 |
| `drone/alive/payload_manager` | P6 |
| `drone/alive/system_monitor` | P7 |

Constants: `kLivelinessPrefix = "drone/alive/"`,
`kLivelinessWildcard = "drone/alive/**"`.

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
| P1 Video Capture | `video/frame`, `video/stereo_frame`, `thread_health/video_capture` | — |
| P2 Perception | `perception/detections`, `radar/detections`, `thread_health/perception` | `video/frame` |
| P3 SLAM/VIO/Nav | `slam/pose`, `thread_health/slam_vio_nav` | `video/stereo_frame` |
| P4 Mission Planner | `mission/status`, `mission/trajectory`, `mission/payload_command`, `comms/fc_command`, `thread_health/mission_planner` | `slam/pose`, `perception/detections`, `radar/detections`, `comms/fc_state`, `comms/gcs_command`, `payload/status`, `monitor/health` |
| P5 Comms | `comms/fc_state`, `comms/gcs_command`, `comms/mission_upload`, `thread_health/comms` | `mission/trajectory`, `comms/fc_command`, `slam/pose`, `mission/status` |
| P6 Payload Manager | `payload/status`, `thread_health/payload_manager` | `mission/payload_command` |
| P7 System Monitor | `monitor/health`, `thread_health/system_monitor` | `comms/fc_state` (optional), `thread_health/*` (all 7) |

> **Note:** `/fault_overrides` is always POSIX SHM (never Zenoh). The
> `fault_injector` tool writes to it; P5 (Comms) and P7 (System Monitor)
> read overrides and merge them into `/fc_state` and `/system_health`
> respectively. See [SIMULATION_ARCHITECTURE.md](SIMULATION_ARCHITECTURE.md)
> for details.
