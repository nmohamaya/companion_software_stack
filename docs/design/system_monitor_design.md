# Process 7 — System Monitor: Design Document

> **Scope**: Detailed design of the System Monitor process (`process7_system_monitor`).
> This document covers process supervision, three-layer watchdog, health monitoring,
> thermal gating, and the Zenoh liveliness subsystem.

---

## Table of Contents

1. [Overview](#overview)
2. [Operational Modes](#operational-modes)
3. [Thread Architecture](#thread-architecture)
4. [IPC Channels](#ipc-channels)
5. [Health Collection: LinuxProcessMonitor](#health-collection-linuxprocessmonitor)
6. [System Info — ISysInfo Interface](#system-info--isysinfo-interface-epic-284-issue-290)
7. [ProcessManager (Layer 2)](#processmanager-layer-2)
8. [ProcessGraph](#processgraph)
9. [RestartPolicy](#restartpolicy)
10. [Three-Layer Watchdog](#three-layer-watchdog)
11. [Zenoh Liveliness](#zenoh-liveliness)
12. [systemd Integration](#systemd-integration)
13. [Fault Injection](#fault-injection)
14. [Configuration Reference](#configuration-reference)
15. [Testing](#testing)
16. [Known Limitations](#known-limitations)

---

## Overview

Process 7 is the health supervisor of the drone stack. It has two responsibilities:
1. **Process supervision** — fork+exec child processes, detect crashes, restart with
   exponential backoff, cascade stop/restart dependent processes
2. **Health monitoring** — collect CPU, memory, temperature, disk, and battery metrics,
   compute thermal zones, publish health status to IPC bus

It is the only process that runs as a **supervisor** (fork+exec parent).

---

## Operational Modes

| Mode | Flag | Description |
|------|------|-------------|
| **Supervised** | `--supervised` | P7 fork+execs P1–P6 in topological order, monitors + restarts |
| **Monitoring** | (default) | P7 monitors health via IPC + liveliness, no process management |

In supervised mode, P7 is the parent of all other processes. On graceful shutdown,
P7 explicitly sends `SIGTERM` (then `SIGKILL` after a timeout) to each child via
`ProcessManager::stop_all()`. If P7 is killed unexpectedly, children are re-parented
to init (PID 1) and continue running — the systemd `BindsTo=` dependency ensures
they are stopped at the service level.

---

## Thread Architecture

| Thread | Rate | Role |
|--------|------|------|
| Main loop | 1 Hz | Health collection, supervisor tick, systemd notify |
| ThreadWatchdog | 1 Hz | Scans thread heartbeats for stuck threads (5 second threshold) |

```
┌─────────────────────────────────────────────────────────────────────┐
│                      System Monitor (P7)                           │
│                                                                     │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                     Main Loop (1 Hz)                           │ │
│  │                                                                │ │
│  │  1. LinuxProcessMonitor::collect()                             │ │
│  │     ├─ /proc/stat    → CPU %                                  │ │
│  │     ├─ /proc/meminfo → Memory %                               │ │
│  │     ├─ /sys/thermal  → Temperature °C                         │ │
│  │     ├─ df -m /       → Disk %                                 │ │
│  │     └─ battery input → Battery %                               │ │
│  │                                                                │ │
│  │  2. ProcessManager::reap_children() + tick()                   │ │
│  │     ├─ waitpid(WNOHANG) for zombie collection                 │ │
│  │     ├─ Restart scheduling (exponential backoff)                │ │
│  │     ├─ Thermal gating check                                   │ │
│  │     └─ Cascade restart logic                                   │ │
│  │                                                                │ │
│  │  3. LivelinessMonitor (Zenoh) or Heartbeat poll                │ │
│  │     └─ process alive/dead status                               │ │
│  │                                                                │ │
│  │  4. Publish ShmSystemHealth + ShmThreadHealth                  │ │
│  │  5. sd_notify(WATCHDOG=1)                                     │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                     │
│  ┌──────────────────────┐    ┌──────────────────────┐              │
│  │ ThreadWatchdog (1Hz) │    │ LivelinessMonitor    │              │
│  │ Scans heartbeats     │    │ (Zenoh subscriber)   │              │
│  │ stuck > 5s → callback│    │ drone/alive/**       │              │
│  └──────────────────────┘    └──────────────────────┘              │
│                                                                     │
│  Supervised mode: fork+exec children ──────────────────────────    │
│  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐               │
│  │ P1  │ │ P2  │ │ P3  │ │ P4  │ │ P5  │ │ P6  │               │
│  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## IPC Channels

### Subscriptions (inputs)

| Channel | Type | Source |
|---------|------|--------|
| `/fc_state` | `ShmFCState` | P5 (battery percent for health) |
| `/payload_status` | `ShmPayloadStatus` | P6 |
| `/fault_overrides` | `ShmFaultOverrides` | Test harness |

### Publications (outputs)

| Channel | Type | Consumers |
|---------|------|-----------|
| `/system_health` | `ShmSystemHealth` | P4 (mission_planner) |
| `/drone_thread_health_system_monitor` | `ShmThreadHealth` | — |

---

## Health Collection: LinuxProcessMonitor

- **Interface:** [`iprocess_monitor.h`](../process7_system_monitor/include/monitor/iprocess_monitor.h)
- **Factory:** `create_process_monitor("linux")` → `LinuxProcessMonitor`

### IProcessMonitor Interface

```cpp
class IProcessMonitor {
    virtual ShmSystemHealth collect() = 0;
    virtual void set_battery_percent(float pct) = 0;
    virtual std::string name() const = 0;
};
```

### Thermal Zone Mapping

| Zone | Name | Conditions |
|------|------|------------|
| 0 | Normal | CPU < 90%, Mem < 90%, Temp < 80°C, Battery ≥ 20%, Disk < 98% |
| 2 | Warning | Any: CPU ≥ 90%, Mem ≥ 90%, Temp ≥ 80°C, Battery < 20% |
| 3 | Critical | Any: Temp > 95°C, Disk ≥ 98%, Battery < 10% |

The thermal zone drives restart gating in `ProcessManager` — higher zones suppress restarts
to prevent thermal runaway from crash loops.

---

## System Info — ISysInfo Interface (Epic #284, Issue #290)

> **Interface:** [`isys_info.h`](../../common/util/include/util/isys_info.h)
> **Factory:** [`sys_info_factory.h`](../../common/util/include/util/sys_info_factory.h)

System metrics are accessed through the `ISysInfo` interface, abstracting platform-specific paths (`/proc`, `/sys`) behind a testable contract:

```cpp
class ISysInfo {
public:
    virtual ~ISysInfo() = default;
    virtual CpuTimes    read_cpu_times() const = 0;
    virtual MemInfo     read_meminfo() const = 0;
    virtual float       read_cpu_temp() const = 0;
    virtual DiskInfo    read_disk_usage() const = 0;
    virtual bool        is_process_alive(pid_t pid) const = 0;
    virtual std::string name() const = 0;
};
```

**Implementations:**

| Class | File | Platform |
|-------|------|----------|
| `LinuxSysInfo` | `linux_sys_info.h` | Generic Linux (`/proc/stat`, `/sys/class/thermal/thermal_zone0`) |
| `JetsonSysInfo` | `jetson_sys_info.h` | NVIDIA Jetson (discovers CPU thermal zone by `type`, caches zone index) |
| `MockSysInfo` | `mock_sys_info.h` | Tests (injectable fields, requires `DRONE_ENABLE_MOCK`) |

**Factory:** `create_sys_info("linux" | "jetson" | "mock")` — selected via config key `system_monitor.platform`.

**Caching:** `LinuxSysInfo` reuses cached file handles for `/proc` and `/sys` reads by rewinding and rereading them on each collection cycle. `JetsonSysInfo` follows the same approach and additionally caches the discovered thermal zone index. No configurable TTL-based read cache is currently used.

### Data Structures

#### CpuTimes

Parsed from `/proc/stat` first line. Two-sample delta method for CPU usage:
```
usage% = (delta_active / delta_total) × 100
active = user + nice + system + irq + softirq + steal
total  = active + idle + iowait
```

### MemInfo

Parsed from `/proc/meminfo`:
```
usage% = (1 − available / total) × 100
```

### Temperature

Read from:
1. `/sys/devices/virtual/thermal/thermal_zone0/temp` (Jetson path)
2. `/sys/class/thermal/thermal_zone0/temp` (fallback)
3. 42°C (hardcoded fallback)

Value is millidegrees in sysfs, divided by 1000.

### DiskInfo

Read via `popen("df -m / | tail -1")`, parsed for total_mb, used_mb, available_mb, mount point.

### Process Alive Check

`is_process_alive(pid)` → checks `/proc/{pid}/stat` existence.

---

## ProcessManager (Layer 2)

- **Header:** [`process_manager.h`](../common/util/include/util/process_manager.h)

### Process Lifecycle

```
  ┌─────────┐   launch()    ┌─────────┐   crash/exit    ┌────────────┐
  │ STOPPED │──────────────►│ RUNNING │───────────────►│ RESTARTING │
  └─────────┘               └─────────┘                └─────┬──────┘
       ▲                         │                            │
       │    stop()               │                   backoff expired
       └─────────────────────────┘                            │
                                                              ▼
                                                     ┌────────────┐
                                          maxed out  │  FAILED    │
                                          restarts   └────────────┘
```

### Key Methods

| Method | Description |
|--------|-------------|
| `add_process(name, binary, args, policy)` | Register a managed process |
| `launch_all()` | Fork+exec all registered processes |
| `launch(name)` | Fork+exec a single process |
| `stop(name)` / `stop_all()` | SIGTERM → wait → SIGKILL |
| `reap_children()` | `waitpid(WNOHANG)` zombie collection |
| `tick()` | Restart scheduler (backoff, thermal gate, cascade) |
| `set_process_graph(graph)` | Enable cascade restart edges |
| `set_thermal_zone(zone)` | Gate restarts based on temperature |
| `compute_stack_status()` | Return NOMINAL / DEGRADED / CRITICAL |

### Fork+Exec Safety

The child process (between `fork()` and `exec()`):
1. Resets signal handlers to `SIG_DFL`
2. Closes all FDs above 2 by enumerating `/proc/self/fd`
   (avoids brute-force close of 1024+ FDs)
3. Calls `execvp()` with the binary path and arguments

### Cascade Restarts

When process A crashes and has a `restart_cascade` to B:
1. Process A is detected as crashed in `reap_children()`
2. Process B is stopped (SIGTERM → SIGKILL if needed)
3. Process A restarts first (using its backoff schedule)
4. Once A reaches RUNNING, B is launched
5. If A exhausts restarts → B remains stopped

---

## ProcessGraph

- **Header:** [`process_graph.h`](../common/util/include/util/process_graph.h)

### Dual-Edge Model

The process graph has two independent edge types:

| Edge Type | Meaning | Example |
|-----------|---------|---------|
| `launch_after` | B must launch after A is running | perception → video_capture |
| `restart_cascade` | If A dies, stop+restart B | comms → {mission_planner, payload_manager} |

### Default Edge Table

```
Launch dependencies (→ means "launches after"):
  perception → video_capture
  slam_vio_nav → perception
  mission_planner → comms
  mission_planner → slam_vio_nav
  payload_manager → comms

Cascade edges (→ means "if source dies, restart target"):
  slam_vio_nav → perception
  comms → mission_planner
  comms → payload_manager
```

### Topological Sort

`launch_order()` uses Kahn's algorithm on launch edges to produce a deterministic
boot sequence. With the default edges:

```
1. video_capture  (no launch deps)
2. comms          (no launch deps)
3. perception     (after video_capture)
4. slam_vio_nav   (after perception)
5. mission_planner (after comms, slam_vio_nav)
6. payload_manager (after comms)
```

### Validation

`validate()` checks:
- No cycles in launch edges (topological sort would fail)
- All referenced process names exist in the graph
- Returns error string on failure, empty on success

---

## RestartPolicy

- **Header:** [`restart_policy.h`](../common/util/include/util/restart_policy.h)

### Per-Process Policy

```cpp
struct RestartPolicy {
    int  max_restarts      = 5;
    int  cooldown_window_s = 60;    // Reset counter after this stable run
    int  initial_backoff_ms = 500;
    int  max_backoff_ms    = 30000;
    bool is_critical       = false; // Stack → CRITICAL on give-up
    int  thermal_gate      = 3;     // 0=always block, 4=never block
};
```

### Backoff Formula

```
delay = min(initial_backoff_ms × 2^attempt, max_backoff_ms)
```

Example with defaults: 500ms → 1s → 2s → 4s → 8s → 16s → 30s (capped).

### Thermal Gate Levels

| Gate Value | Meaning |
|------------|---------|
| 0 | Always block restarts (disabled process) |
| 1 | Block at warm or higher |
| 2 | Block at hot or higher |
| 3 | Block only at critical temperature |
| 4 | Never block (always restart regardless of temperature) |

### StackStatus

| Value | Name | Condition |
|-------|------|-----------|
| 0 | NOMINAL | All processes alive |
| 1 | DEGRADED | Non-critical process died or restarting |
| 2 | CRITICAL | Critical process exhausted restarts |

---

## Three-Layer Watchdog

The system implements a three-layer watchdog architecture (ADR-004):

### Layer 1: Thread Watchdog

- **Components:** `ThreadHeartbeat` + `ThreadWatchdog`
- **Mechanism:** Lock-free atomic timestamp touch (~1 ns cost)
- **Detection:** ThreadWatchdog scans at 1 Hz, fires callback if `now − last_touch > 5s`
- **Scope:** Per-thread within each process
- **Grace:** `touch_with_grace(handle, duration)` for known long operations

### Layer 2: Process Watchdog

- **Components:** `ProcessManager` + `LivelinessMonitor`
- **Mechanism:** `waitpid(WNOHANG)` for supervised children, Zenoh liveliness tokens for
  independent processes
- **Detection:** Sub-second for Zenoh, ~1 second for waitpid
- **Recovery:** Exponential backoff restart, cascade stop/restart

### Layer 3: OS Watchdog

- **Components:** systemd `WatchdogSec=10s`
- **Mechanism:** P7 calls `sd_notify(WATCHDOG=1)` every ~1 second
- **Detection:** If P7 itself hangs for 10s, systemd kills and restarts it
- **Recovery:** systemd restarts P7, which re-launches all children

### Information Flow

```
Thread heartbeats (Layer 1)
    │
    ▼
ThreadHealthPublisher → ShmThreadHealth → P7 monitors
    │
Process liveliness (Layer 2)                               systemd (Layer 3)
    │                                                          │
    ├─ waitpid() (supervised mode)                            │
    ├─ Zenoh drone/alive/** (monitoring mode)                 │
    │                                                          │
    ▼                                                          ▼
ProcessManager → restart/cascade                   WatchdogSec → restart P7
```

---

## Zenoh Liveliness

- **Header:** [`zenoh_liveliness.h`](../common/ipc/include/ipc/zenoh_liveliness.h)

### LivelinessToken

Each process declares a Zenoh liveliness token on startup:
```
Key: "drone/alive/{process_name}"
```

The token is RAII — it is automatically dropped when the process exits or crashes.
The Zenoh router detects the drop and notifies all listeners.

### LivelinessMonitor

P7 subscribes to `drone/alive/**` and maintains an `alive_set_`:
- `on_alive(process_name)` callback → insert into set
- `on_death(process_name)` callback → remove from set, trigger restart evaluation
- `is_alive(process_name)` / `get_alive_processes()` — thread-safe queries

When `HAVE_ZENOH=OFF`, stub implementations are compiled (no-op).

---

## systemd Integration

- **Header:** [`sd_notify.h`](../common/util/include/util/sd_notify.h)

| Function | SD_NOTIFY | Purpose |
|----------|-----------|---------|
| `notify_ready()` | `READY=1` | Type=notify service readiness |
| `notify_watchdog()` | `WATCHDOG=1` | Pet the WatchdogSec timer |
| `notify_stopping()` | `STOPPING=1` | Graceful shutdown signal |
| `notify_status(msg)` | `STATUS=msg` | Human-readable in `systemctl status` |

P7 calls `notify_watchdog()` on every main loop iteration (1 Hz). If P7 fails to
pet for 10 seconds, systemd kills and restarts it.

---

## Fault Injection

The `fc_rx` thread in P5 and the health collection in P7 both read `ShmFaultOverrides`:

| Override | Effect in P7 |
|----------|-------------|
| `override_battery` | Overrides `battery_percent` in collected health |
| `override_thermal` | Forces specific thermal zone for restart gating tests |

---

## Configuration Reference

### `system_monitor.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"linux"` | Monitor backend |
| `update_rate_hz` | int | 1 | Main loop frequency |
| `disk_check_interval_s` | int | 10 | Disk usage poll interval (expensive) |

### `system_monitor.thresholds.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `cpu_warn_percent` | float | 90.0 | CPU warning threshold |
| `mem_warn_percent` | float | 90.0 | Memory warning threshold |
| `temp_warn_c` | float | 80.0 | Temperature warning (→ zone 2) |
| `temp_crit_c` | float | 95.0 | Temperature critical (→ zone 3) |
| `battery_warn_percent` | float | 20.0 | Battery warning (→ zone 2) |
| `battery_crit_percent` | float | 10.0 | Battery critical (→ zone 3) |
| `disk_crit_percent` | float | 98.0 | Disk critical (→ zone 3) |

### `watchdog.processes.<name>.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `binary` | string | — | Path to executable |
| `critical` | bool | false | Stack → CRITICAL on give-up |
| `max_restarts` | int | 5 | Max restart attempts |
| `cooldown_s` | int | 60 | Stable-run window to reset counter |
| `backoff_ms` | int | 500 | Initial backoff delay |
| `max_backoff_ms` | int | 30000 | Max backoff cap |
| `thermal_gate` | int | 3 | Restart temperature gate (0–4) |
| `launch_after` | string[] | [] | Launch dependency edges |
| `restart_cascade` | string[] | [] | Cascade restart edges |

---

## Testing

| Test File | Tests | Coverage |
|-----------|-------|----------|
| [`test_system_monitor.cpp`](../tests/test_system_monitor.cpp) | 11 | sys_info parsers, health collection |
| [`test_process_manager.cpp`](../tests/test_process_manager.cpp) | 13 | fork+exec, crash detection, restart, backoff |
| [`test_process_graph.cpp`](../tests/test_process_graph.cpp) | 27 | Dual-edge graph, topological sort, cascade BFS |
| [`test_restart_policy.cpp`](../tests/test_restart_policy.cpp) | 19 | Backoff formula, thermal gate, config parsing |
| **Total** | **70** | |

### Key Test Scenarios

- **Process crash → restart:** Launch `test_crasher` → detect via `reap_children()` → verify RESTARTING state → backoff → re-launch
- **Max restart exhaustion:** Repeatedly crash → verify FAILED after `max_restarts` exceeded
- **Cascade validation:** Crash source process → verify transitive targets also stopped
- **Topological sort:** Default 6-process graph → deterministic boot order
- **Thermal gating:** Set thermal_zone=3, verify restart deferred for `thermal_gate ≤ 3`

---

## Observability

P7 has the broadest cross-process visibility: it publishes health
snapshots that summarise all 7 processes in one structured record.

### Structured Logging

| Field | Description |
|-------|-------------|
| `process` | `"system_monitor"` |
| `cpu_percent` | Aggregate CPU usage across all cores |
| `mem_percent` | RSS / total RAM usage |
| `temp_c` | SoC die temperature (°C) |
| `disk_percent` | Root filesystem usage |
| `battery_percent` | Reported battery level |
| `process_states` | Array of `{name, pid, status, restart_count}` per managed process |

> **Note:** These values appear in the `msg` text field of the JSON log line.
> `--json-logs` does not emit them as separate top-level JSON keys.

### Correlation IDs

P7 does not participate in GCS correlation.

### Latency Tracking

| Channel | Direction |
|---------|----------|
| `/fc_state` | subscriber |
| `/payload_status` | subscriber |

Latency is tracked automatically on each `receive()` call. Call
`subscriber->log_latency_if_due(N)` in the monitor thread to
periodically emit a p50/p90/p99 histogram (µs) to the log.

See [observability.md](observability.md) for the `LatencyTracker` API
and `log_latency_if_due()` usage.

---

## Known Limitations

1. **Disk check is expensive:** `popen("df -m /")` spawns a subprocess — rate-limited to
   every `disk_check_interval_s` seconds (default 10).
2. **Single thermal zone:** Only reads `thermal_zone0`. Multi-zone Jetson boards have
   multiple zones (CPU, GPU, SOC) that could be monitored individually.
3. **No MAVSDK GCS:** GCS link is always simulated. Real GCS monitoring (heartbeat loss
   detection) is not implemented.
4. **No cgroup isolation:** Supervised children share the same cgroup as P7. A child
   memory bomb can kill P7.
5. **Liveliness requires Zenoh:** When running with SHM backend, process death detection
   falls back to `waitpid()` (supervised mode only). Standalone processes cannot
   detect each other's death without Zenoh.
