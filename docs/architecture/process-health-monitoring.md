# Process Health Monitoring — Zenoh Liveliness Tokens

> **Phase F** of the Zenoh IPC Migration (Issue #51)

## Overview

Phase F implements **automatic process health monitoring** using Zenoh's
**liveliness tokens** — a built-in mechanism where each process declares a
token on the Zenoh session, and other processes receive immediate callbacks
when a peer dies, crashes, or disconnects.

This replaces the need for custom heartbeat polling with:
- **Zero network overhead** — tokens are session metadata, not messages
- **Sub-second crash detection** — transport-level, not timeout-based
- **Zero configuration** — no heartbeat intervals or thresholds to tune

## Architecture

```
 P1 ─── LivelinessToken("video_capture")    ──┐
 P2 ─── LivelinessToken("perception")        │
 P3 ─── LivelinessToken("slam_vio_nav")      │    drone/alive/**
 P4 ─── LivelinessToken("mission_planner")   ├────────────────────► P7 LivelinessMonitor
 P5 ─── LivelinessToken("comms")             │    (PUT = alive,      ├─ on_alive callback
 P6 ─── LivelinessToken("payload_manager")   │     DELETE = dead)    ├─ on_death callback
 P7 ─── LivelinessToken("system_monitor")  ──┘                      └─ get_alive_processes()
```

When a process crashes (kill -9, segfault, etc.), its Zenoh session is torn
down by the OS, which automatically drops the liveliness token. The monitor
receives a DELETE sample on `drone/alive/{process_name}` and fires the death
callback.

## Key Expressions

| Process | Token Key Expression |
|---------|---------------------|
| P1 Video Capture | `drone/alive/video_capture` |
| P2 Perception | `drone/alive/perception` |
| P3 SLAM/VIO/Nav | `drone/alive/slam_vio_nav` |
| P4 Mission Planner | `drone/alive/mission_planner` |
| P5 Comms | `drone/alive/comms` |
| P6 Payload Manager | `drone/alive/payload_manager` |
| P7 System Monitor | `drone/alive/system_monitor` |

## Usage

### Declaring a Token (Any Process)

```cpp
#include "ipc/zenoh_liveliness.h"

int main() {
    auto bus = drone::ipc::create_message_bus(cfg);

    // One line — token is RAII, dropped on scope exit or crash
    drone::ipc::LivelinessToken token("video_capture");

    // ... rest of process ...
}
// Token automatically undeclared here → monitors notified
```

### Monitoring Process Health (System Monitor)

```cpp
#include "ipc/zenoh_liveliness.h"

drone::ipc::LivelinessMonitor monitor(
    [](const std::string& proc) {
        spdlog::info("Process ALIVE: {}", proc);
    },
    [](const std::string& proc) {
        spdlog::error("Process DIED: {}", proc);
    }
);

// Query current state
auto alive = monitor.get_alive_processes();
bool comms_ok = monitor.is_alive("comms");
```

## SystemHealth Integration

The `SystemHealth` struct (defined in `common/ipc/include/ipc/ipc_types.h`)
includes per-process liveness and stack-level status:

```cpp
static constexpr uint8_t kMaxTrackedProcesses = 8;  // 7 processes + 1 FC link

struct ProcessHealthEntry {
    char     name[32]     = {};     // Process name (e.g. "video_capture")
    bool     alive        = false;  // Last known liveness state
    uint64_t last_seen_ns = 0;     // Timestamp of last liveliness event
};

struct SystemHealth {
    uint64_t timestamp_ns;
    float    cpu_usage_percent;
    float    memory_usage_percent;
    float    disk_usage_percent;
    float    max_temp_c;
    float    gpu_temp_c;
    float    cpu_temp_c;
    uint32_t total_healthy;
    uint32_t total_degraded;
    uint32_t total_dead;
    float    power_watts;
    uint8_t  thermal_zone;  // 0=normal, 1=warm, 2=hot, 3=critical

    // Stack-level status (Phase 4 supervisor integration)
    uint8_t  stack_status   = 0;  // StackStatus enum (0=NOMINAL, 1=DEGRADED, 2=CRITICAL)
    uint32_t total_restarts = 0;  // Cumulative restart count across all processes

    // Process health (populated by LivelinessMonitor)
    ProcessHealthEntry processes[kMaxTrackedProcesses] = {};
    uint8_t            num_processes                   = 0;      // Number of tracked processes
    bool               critical_failure                = false;  // True if a critical process died
};
```

### Critical Process Detection

The System Monitor flags `critical_failure = true` when either of these
processes dies:
- **comms** — loss of FC link means no flight control
- **slam_vio_nav** — loss of pose estimation means no navigation

Other process deaths are logged but don't set the critical flag. This
can be extended via configuration as needed.

## ThreadWatchdog Configuration

The `ThreadWatchdog` detects stuck threads within each process. Default values
from `config/default.json` under the `watchdog` section:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `thread_stuck_threshold_ms` | 5000 | Time (ms) without a heartbeat touch before a thread is considered stuck |
| `thread_scan_interval_ms` | 1000 | How often (ms) the watchdog scans all registered threads |

These values can be overridden in `config/default.json`:

```json
"watchdog": {
    "thread_stuck_threshold_ms": 5000,
    "thread_scan_interval_ms": 1000
}
```

## Comparison: Liveliness Tokens vs Custom Heartbeat

| Aspect | Custom Heartbeat | Zenoh Liveliness |
|--------|-----------------|------------------|
| Detection latency | Configurable (e.g. 3s) | ~100 ms |
| Network overhead | Periodic messages | Zero |
| Code complexity | Timer + counter + threshold | 1 line per process |
| False positives | Possible under CPU load | Extremely rare |
| Network-aware | Must implement separately | Built-in |
| Crash detection | After timeout expires | Immediate |

## Non-Zenoh Builds

When `HAVE_ZENOH` is not defined (SHM-only builds):
- `LivelinessToken` is a no-op stub (`is_valid()` returns false)
- `LivelinessMonitor` is a no-op stub (`get_alive_processes()` returns empty)
- No compile errors — all process mains include the header unconditionally

## Files

| File | Purpose |
|------|---------|
| `common/ipc/include/ipc/zenoh_liveliness.h` | Token + Monitor classes (+ stubs) |
| `common/ipc/include/ipc/ipc_types.h` | `ProcessHealthEntry`, `SystemHealth` struct |
| `process{1-7}_*/src/main.cpp` | Token declaration (1 line each) |
| `process7_system_monitor/src/main.cpp` | Monitor integration + health struct population |
| `tests/test_zenoh_liveliness.cpp` | Unit tests |
| `docs/process-health-monitoring.md` | This document |

## Related Issues

- **#28** — FC heartbeat timeout (liveliness replaces custom heartbeat)
- **#30** — Pre-flight check (query `get_alive_processes()` to verify all running)
- **#41** — Contingency fault tree (liveliness is the detection mechanism)
- **#50** — Network transport (liveliness works over network links too)
- **#61** — FaultManager graceful degradation (consumes `SystemHealth` +
  `critical_failure` flag to evaluate faults and issue escalating flight-control
  actions: WARN → LOITER → RTL → EMERGENCY_LAND)

## ⚠️ Limitations

- Liveliness detection requires the processes to share a Zenoh session
  (same router or peer network). In SHM-only mode, tokens are no-ops.
- Token drop detection relies on the Zenoh transport layer detecting
  session loss. Over unreliable networks, this may take longer than the
  ~100 ms typical for local sessions.
- The `critical_failure` flag is currently hardcoded for `comms` and
  `slam_vio_nav`. For production, this should be configurable (see
  PRODUCTION_READINESS.md item 5.1).
