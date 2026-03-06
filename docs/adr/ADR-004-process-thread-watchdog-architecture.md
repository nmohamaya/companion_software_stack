# ADR-004: Process & Thread Watchdog Architecture

| Field | Value |
|-------|-------|
| **Status** | Accepted (Phases 1–2 implemented) |
| **Date** | 2026-03-04 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | ADR-002 (Modular IPC), Epic #25 (Real Drone Deployment), Issue #28 (heartbeat timeout), Issue #31 (systemd + process supervisor), Issue #41 (contingency fault tree) |

---

## 1. Context

### What Works Today

The stack has **Zenoh liveliness tokens** (Phase F of Epic #45) for
process-level death detection:

| Capability | Mechanism | Latency | Backend |
|-----------|-----------|---------|---------|
| Process crash detection | `LivelinessToken` / `LivelinessMonitor` | ~100 ms | Zenoh only |
| Process liveness query | `LivelinessMonitor::is_alive()` | Instant | Zenoh only |
| PID polling fallback | `is_process_alive(pid_t)` via `/proc` | 1 s (poll interval) | SHM only |
| Health reporting | `ShmSystemHealth` struct | 1 Hz | Both |

### What Does Not Work

1. **No restart/recovery** — when any process dies, `launch_all.sh` calls
   `wait -n` and tears down the entire stack. A single non-critical process
   crash (e.g., payload manager) kills the flight.

2. **No thread-level monitoring** — a thread that deadlocks or enters an
   infinite loop is invisible. The process stays alive (PID exists, Zenoh
   token active), but it stops making progress. This is particularly
   dangerous in safety-critical threads like `fc_tx_thread` (comms) or
   the VIO processing loop (SLAM).

3. **SHM-mode blind spot** — the liveliness system compiles to no-op stubs
   when Zenoh is unavailable. Process 7 reports 0 tracked processes and
   `critical_failure` is never set.

4. **No degraded-mode operation** — the stack has no concept of "running
   with reduced capability." It is either fully up or fully down.

### Decision Drivers

- **Flight safety** — a crashed perception process should not crash the
  flight controller link (comms) or navigation (SLAM).
- **Autonomy** — the drone may be in flight when a process dies; it must
  recover without ground-station intervention.
- **Backend independence** — the solution must work with both SHM and Zenoh
  IPC backends, per ADR-002.
- **Debuggability** — stuck threads and restart events must produce
  structured logs with correlation IDs for post-flight analysis.
- **Incremental adoption** — adding thread heartbeats to existing processes
  must not require restructuring their main loops.

---

## 2. Decision

Implement a **three-layer watchdog architecture** that provides thread-level
health monitoring, process-level restart with policy control, and (future)
systemd integration.

### 2.1 Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│ Layer 3: OS Supervisor (Tier 4 — deferred)                       │
│   systemd service units, sd_notify(WATCHDOG=1), Restart=on-fail  │
│   Manages the Process 7 supervisor itself                        │
└────────────────────────┬─────────────────────────────────────────┘
                         │ manages
┌────────────────────────▼─────────────────────────────────────────┐
│ Layer 2: Process Supervisor (in Process 7 — System Monitor)      │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │ Detection                                                 │   │
│  │  • Zenoh: LivelinessMonitor (existing, sub-second)        │   │
│  │  • SHM:   PID polling + ShmThreadHealth reads             │   │
│  │  • Both:  Thread health timeout (configurable, e.g. 5 s)  │   │
│  └────────────────────────────────────────────────────────────┘   │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │ Recovery                                                  │   │
│  │  • fork + exec individual process binaries                │   │
│  │  • Dependency ordering (launch graph)                     │   │
│  │  • Configurable restart policy per process                │   │
│  │  • Exponential backoff with max-retry limit               │   │
│  │  • Cooldown window: reset retry counter after stable run  │   │
│  └────────────────────────────────────────────────────────────┘   │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │ Reporting                                                 │   │
│  │  • ShmSystemHealth: critical_failure, process states      │   │
│  │  • Per-process restart count + last restart timestamp     │   │
│  │  • Stack-level status: NOMINAL / DEGRADED / CRITICAL      │   │
│  │  • JSON-structured log events for post-flight analysis    │   │
│  └────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
                         │ monitors
┌────────────────────────▼─────────────────────────────────────────┐
│ Layer 1: Thread Heartbeat (inside each process)                  │
│                                                                  │
│  ThreadHeartbeat (per-thread, lock-free)                          │
│  ┌───────────────────────────────────────────────┐               │
│  │ • std::atomic<uint64_t> last_touch_ns         │               │
│  │ • Registered at thread startup                │               │
│  │ • .touch() called each loop iteration         │               │
│  │ • Zero overhead: single atomic store          │               │
│  └───────────────────────────────────────────────┘               │
│                                                                  │
│  ThreadWatchdog (one per process, separate thread)               │
│  ┌───────────────────────────────────────────────┐               │
│  │ • Scans all registered heartbeats periodically │               │
│  │ • stuck_threshold: configurable (default 5 s) │               │
│  │ • On stuck thread detected:                   │               │
│  │   1. Log with thread name + correlation ID    │               │
│  │   2. Publish ShmThreadHealth (visible to P7)  │               │
│  │   3. Critical thread → self-terminate         │               │
│  │   4. Non-critical thread → flag degraded      │               │
│  └───────────────────────────────────────────────┘               │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 Layer 1 — Thread Heartbeat

#### Data Structures (`common/util/include/util/thread_heartbeat.h`)

```cpp
/// Per-thread heartbeat — one atomic timestamp touched every iteration.
/// Lock-free, zero-allocation after construction.
struct ThreadHeartbeat {
    char                     name[32] = {};
    std::atomic<uint64_t>    last_touch_ns{0};
    bool                     is_critical = false;  // stuck → process self-terminates
};

/// Registry of thread heartbeats within a single process.
/// Accessed by the watchdog thread (reader) and worker threads (writers).
class ThreadHeartbeatRegistry {
public:
    static ThreadHeartbeatRegistry& instance();

    /// Register a new heartbeat. Returns a handle for touch().
    /// Thread-safe (called from multiple threads at startup).
    size_t register_thread(const char* name, bool critical);

    /// Touch the heartbeat — called every loop iteration.
    /// Cost: one atomic store (relaxed ordering) + one clock read.
    void touch(size_t handle);

    /// Touch the heartbeat with an extended grace period.
    /// The watchdog treats the beat as live for an additional `grace`
    /// beyond the normal stuck_threshold.  Use during known long-startup
    /// operations (model loads, vocabulary loads, MAVLink handshake).
    void touch_with_grace(size_t handle, std::chrono::milliseconds grace);

    /// Snapshot all heartbeats (used by watchdog and SHM publisher).
    std::vector<ThreadHeartbeat> snapshot() const;

private:
    static constexpr size_t kMaxThreads = 16;
    std::array<ThreadHeartbeat, kMaxThreads> beats_{};
    std::atomic<size_t> count_{0};
};
```

#### Usage in Process Code

```cpp
// In any thread's main loop:
auto handle = ThreadHeartbeatRegistry::instance()
                  .register_thread("yolo_inference", /*critical=*/false);

while (running_) {
    ThreadHeartbeatRegistry::instance().touch(handle);

    // ... existing work ...
}
```

This is deliberately minimal — one line at registration, one line per
iteration. No restructuring of existing loops is required.

#### Convenience RAII Wrapper

```cpp
/// ScopedHeartbeat — registers in constructor, no cleanup needed.
/// Call .touch() in your loop.
class ScopedHeartbeat {
public:
    ScopedHeartbeat(const char* name, bool critical = false);
    void touch();
    /// Touch with an extended grace period (see ThreadHeartbeatRegistry::touch_with_grace).
    void touch_with_grace(std::chrono::milliseconds grace);
private:
    size_t handle_;
};
```

#### ThreadWatchdog (one per process)

```cpp
/// Runs a background thread that scans all heartbeats.
/// If any beat exceeds stuck_threshold, takes action.
class ThreadWatchdog {
public:
    struct Config {
        std::chrono::milliseconds stuck_threshold{5000};
        std::chrono::milliseconds scan_interval{1000};
    };

    explicit ThreadWatchdog(Config cfg = {});
    ~ThreadWatchdog();  // stops scan thread

    /// Called by the watchdog when a thread is stuck.
    /// Default: logs + sets flag. Override for custom behaviour.
    using StuckCallback = std::function<void(const ThreadHeartbeat& beat)>;
    void set_stuck_callback(StuckCallback cb);
};
```

**Design choice — why atomics, not `thread_local`?**

Using `thread_local` timestamps would require the watchdog to iterate over
all threads' TLS — which is not possible in standard C++. Atomics in a
shared registry allow any thread (the watchdog, the SHM publisher) to read
all heartbeats. The cost is one `atomic_store(relaxed)` per touch, which is
a single uncontended cache line write (~1 ns on ARM).

### 2.3 Layer 2 — Process Supervisor

The process supervisor lives in Process 7 (System Monitor), which becomes
the parent process that fork+execs the other six processes.

#### Restart Policy

```cpp
struct RestartPolicy {
    uint32_t max_restarts       = 5;       // Max restarts before giving up
    uint32_t cooldown_window_s  = 60;      // Reset counter after N seconds stable
    uint32_t initial_backoff_ms = 500;     // Doubles each attempt
    uint32_t max_backoff_ms     = 30000;   // Cap at 30 s
    bool     is_critical        = false;   // If true, stack enters CRITICAL on give-up
    uint8_t  thermal_gate       = 3;       // Block restart when thermal_zone >= this value
                                           // 0=always block, 3=block at CRITICAL (default), 4=never block
};
```

The `thermal_gate` field prevents restart attempts when the system is
overheating. When `ShmSystemHealth::thermal_zone >= thermal_gate`, the
supervisor defers the restart timer and logs the suppression. Restarts
resume automatically once the temperature drops below the threshold.
This prevents restart storms from worsening thermal situations (e.g.,
CUDA warmup on a throttled Jetson).

#### Process Descriptor

```cpp
struct ProcessDescriptor {
    char            name[32];           // e.g. "video_capture"
    char            binary_path[256];   // e.g. "build/bin/video_capture"
    char            args[256];          // CLI arguments
    RestartPolicy   policy;

    // Launch dependencies: which processes must be alive before this one starts.
    // Used for topological launch ordering.
    char            launch_after[4][32];   // e.g. {"comms", ""}
    uint8_t         num_launch_deps;

    // Restart cascade: which processes should be stopped and restarted
    // when THIS process dies.  These are downstream consumers that may
    // hold stale state from the dead process.
    char            restart_cascade[4][32]; // e.g. {"mission_planner", ""}
    uint8_t         num_cascade;
};
```

#### Dual-Edge Dependency Graph

The graph uses **two separate edge types** because launch ordering and
restart cascading are not the same relationship:

- **`launch_after`** (solid arrows below) = "this process needs data from
  X, so X must be alive before launch."
- **`restart_cascade`** (dashed arrows below) = "if this process dies,
  also stop and restart Y because Y may hold stale state."

```
  Launch order / launch prerequisites (top-down):   Restart cascade (top-down):

  video_capture                                     comms ─ ─ ─► mission_planner
       │                                            comms ─ ─ ─► payload_manager
  perception                                        slam_vio_nav ─ ─► perception
       │
  slam_vio_nav          comms
        └────┬─────────┘
             ▼
       mission_planner       payload_manager
```

**Default edge table:**

| Process | launch_after | restart_cascade | Rationale |
|---------|-------------|-----------------|-----------|
| video_capture | — | — | Independent producer |
| perception | video_capture | — | Consumes frames; no downstream state holders |
| slam_vio_nav | perception | perception | Perception caches SLAM pose; restart it to clear stale state |
| comms | — | mission_planner, payload_manager | Both hold FC state from comms |
| mission_planner | comms, slam_vio_nav | — | Consumes FC state + pose |
| payload_manager | comms | — | Consumes FC commands |

**Example: comms crashes**
1. Supervisor detects comms death (liveliness / PID).
2. Cascade: stop `mission_planner` and `payload_manager` (they hold stale
   FC state from comms).
3. Restart `comms`.
4. Once comms is alive → restart `mission_planner`, `payload_manager`.
5. `slam_vio_nav`, `perception`, `video_capture` are **not** restarted
   (they don't consume comms data directly and can tolerate the gap).

**Example: perception crashes**
1. Supervisor detects perception death.
2. No cascade targets — `video_capture` feeds perception, not the reverse.
3. Restart only `perception`.
4. `video_capture` continues producing frames throughout.

The supervisor launches processes in `launch_after` topological order and
waits for a liveliness token (or PID check) before launching dependents.
On restart, it stops the cascade targets, restarts the crashed process,
then restarts the cascade targets in launch order.

**Design choice — why fork+exec and not a separate launcher binary?**

Keeping the supervisor inside Process 7 means:
1. It shares the Zenoh session (liveliness monitoring is already there).
2. No additional binary to deploy/manage.
3. Health metrics (CPU, temperature) and process supervision are co-located,
   enabling decisions like "don't restart if thermal zone is CRITICAL."
4. When Layer 3 (systemd) is added, it manages exactly one unit
   (`system_monitor.service`) which supervises the rest.

The trade-off is that Process 7 itself is a SPOF — if it crashes, the
entire stack is unsupervised. Layer 3 (systemd) addresses this by
auto-restarting Process 7 with `Restart=always`.

#### Stack Status Model

```
 NOMINAL ──────► DEGRADED ──────► CRITICAL
   ▲                │                │
   └────────────────┘                │
   (process restarted               (critical process
    successfully)                    unrecoverable)
```

| Status | Meaning | Example |
|--------|---------|---------|
| **NOMINAL** | All processes alive, all critical threads healthy | Normal operation |
| **DEGRADED** | A non-critical process died or a non-critical thread is stuck | Payload manager crashed; perception thread stuck but restarting |
| **CRITICAL** | A critical process exhausted restart attempts, or a critical thread in comms/SLAM is stuck | Comms died 5 times in 60 s; fc_tx_thread deadlocked |

### 2.4 SHM Thread Health Struct

To make thread health visible to the supervisor (and to any subscriber),
each process publishes a `ShmThreadHealth` periodically:

```cpp
static constexpr uint8_t  kMaxTrackedThreads = 16;

struct ThreadHealthEntry {
    char     name[32]   = {};      // Thread name (e.g. "fc_tx_thread")
    bool     healthy    = true;    // false if stuck
    bool     critical   = false;   // if stuck → process should self-terminate
    uint64_t last_ns    = 0;       // Last touch timestamp
};

struct ShmThreadHealth {
    char                process_name[32] = {};
    ThreadHealthEntry   threads[kMaxTrackedThreads] = {};
    uint8_t             num_threads = 0;
    uint64_t            timestamp_ns = 0;
};
```

Each process publishes this on a per-process SHM topic:
`/drone_thread_health_{process_name}` (e.g. `/drone_thread_health_comms`).

The supervisor subscribes to all seven and uses thread health as an
additional signal alongside liveliness tokens and PID polling.

### 2.5 Layer 3 — OS Supervisor (Issue #83)

**Revised (2026-03-06):** The original design had P7 as the sole systemd
unit with P1–P6 as fork+exec children. This was changed to 7 independent
service units to avoid the **orphan re-adoption problem**: if P7 crashes
and systemd restarts it, the old P1–P6 processes are orphaned under PID 1
and a restarted P7 cannot re-attach to them — it would launch duplicates.

**Current architecture — 7 independent units + 1 target:**

- Each process is a separate systemd service: `drone-video-capture.service`,
  `drone-perception.service`, etc.
- Services use `After=` directives matching the dependency graph.
- `Restart=on-failure` + `RestartSec=2s` for all services.
- `drone-stack.target` groups all 7 for `systemctl start/stop drone-stack.target`.
- Process 7 (`drone-system-monitor.service`) uses `Type=notify` and
  `WatchdogSec=10` with `sd_notify(0, "WATCHDOG=1")` in the health loop.
- P7 runs in **monitor-only** mode (no `--supervised`): health telemetry,
  liveliness monitoring, `ShmSystemHealth` publishing. It does NOT
  fork+exec child processes when deployed under systemd.
- Cascade restarts are expressed via `PartOf=` directives.

The `--supervised` fork+exec mode remains available for non-systemd
deployments (development, `launch_all.sh`, containers without systemd).

Layer 3 is purely additive — no changes to Layers 1–2.

### 2.6 Thread & Process Criticality Inventory

The following table documents every thread registered with the heartbeat
system (Phase 2, #90), its criticality designation, and the rationale.
A **critical** thread triggers process self-termination when stuck; a
**non-critical** thread logs a warning, marks its own thread state as
unhealthy, and the system monitor may report the overall stack as
DEGRADED while the process continues operating.

| Process | Thread Name | Critical | Rationale |
|---------|-------------|:--------:|-----------|
| **P1 Video Capture** | `mission_cam` | Yes | Primary camera feed — loss means no visual data for perception or SLAM |
| | `stereo_cam` | Yes | Stereo depth data — required for obstacle avoidance and VIO |
| **P2 Perception** | `inference` | Yes | YOLO object detection — loss removes obstacle/target awareness |
| | `tracker` | Yes | Object tracking continuity — stale tracks cause incorrect planning |
| | `fusion` | Yes | Sensor fusion output — downstream consumers depend on fused detections |
| | `lidar` | No | Supplementary range data — perception degrades gracefully without it |
| | `radar` | No | Supplementary range data — perception degrades gracefully without it |
| **P3 SLAM/VIO/Nav** | `visual_frontend` | Yes | Visual odometry — loss means no pose updates, navigation fails |
| | `imu_reader` | Yes | IMU integration — loss means no attitude/velocity estimation |
| | `pose_publisher` | Yes | Pose output — downstream (P4, P5) lose localisation |
| **P4 Mission Planner** | `planning_loop` | Yes | Waypoint tracking and contingency logic — loss means uncontrolled flight |
| **P5 Comms** | `fc_rx` | Yes | Flight controller telemetry — loss means no state feedback from PX4 |
| | `fc_tx` | Yes | Flight controller commands — loss triggers PX4 link-loss failsafe |
| | `gcs_rx` | No | Ground station commands — mission continues autonomously without GCS |
| | `gcs_tx` | No | Ground station telemetry — loss means no operator visibility but flight is safe |
| **P6 Payload Manager** | `payload_loop` | No | Payload actuation — non-essential for flight safety |
| **P7 System Monitor** | `health_loop` | No | Health reporting — loss disables monitoring but does not affect flight control |

**Summary:** 17 threads total — 11 critical, 6 non-critical.

**Design principles applied:**

1. **Flight-safety boundary** — any thread whose stall could lead to
   loss of vehicle control or collision is marked critical. This includes
   the entire perception→SLAM→planning→comms chain.
2. **Graceful degradation** — supplementary sensors (lidar, radar),
   ground station links, payload, and health monitoring are non-critical.
   Their loss reduces capability but the vehicle can still fly safely.
3. **Conservative default** — when in doubt, mark critical. A false
   self-termination triggers a supervised restart (Phase 3), which is
   safer than silently operating with a stuck critical thread.

---

## 3. Alternatives Considered

### 3.1 External Supervisor (supervisord, s6, runit)

| Aspect | Process-manager approach | Our approach (in-process) |
|--------|------------------------|--------------------------|
| Process restart | ✅ Built-in | ✅ fork+exec + policy |
| Thread monitoring | ❌ Not possible | ✅ Heartbeat registry |
| IPC integration | ❌ Separate system | ✅ Shares Zenoh session |
| Dependency graph | ❌ Limited (startup order only) | ✅ Full restart-dependents |
| Drone-specific decisions | ❌ No access to thermals, battery | ✅ Co-located with health metrics |
| Deployment complexity | More daemons to install | Single binary |

**Rejected because:** External supervisors cannot monitor threads, don't
integrate with Zenoh liveliness, and cannot make flight-context restart
decisions (e.g., "don't restart perception if thermal zone is CRITICAL
and we're landing").

### 3.2 Pure Zenoh Liveliness (No Thread Layer)

Zenoh liveliness tokens detect process death but not thread stalls.
A process with a deadlocked critical thread appears "alive" to the
liveliness system. This is the current blind spot that motivates Layer 1.

**Rejected because:** Thread-level monitoring is essential for flight
safety. A stuck `fc_tx_thread` means no heartbeat to the flight
controller, which triggers PX4's link-loss failsafe — but the companion
stack doesn't know why.

### 3.3 Hardware Watchdog Timer (WDT)

The Jetson Orin has a hardware WDT that reboots the entire system if not
kicked. This is a last-resort safety net, not a recovery mechanism.

**Deferred to Tier 4** as a complement, not a replacement. The software
watchdog can recover from most failures without a full reboot.

---

## 4. Consequences

### Positive

- **Graceful degradation** — non-critical process crashes no longer kill
  the flight. The stack continues in DEGRADED mode with reduced capability.
- **Thread visibility** — deadlocks and stuck threads are detected and
  logged, replacing "it just stopped working" with actionable diagnostics.
- **Backend-independent** — the thread heartbeat and PID-polling layers
  work without Zenoh. Only the liveliness-token detection path requires it.
- **Zero overhead in the hot path** — thread heartbeat is one atomic store
  per loop iteration (~1 ns). The watchdog scans are on a separate,
  low-priority thread.
- **Post-flight analysis** — restart events and stuck-thread detections
  emit structured JSON logs with correlation IDs, feeding into the
  existing observability pipeline.
- **Incremental rollout** — heartbeats can be added to one process at a
  time. The supervisor is opt-in via a `--supervised` flag.

### Negative

- **Single point of failure** — Process 7 is the supervisor. If it
  crashes, no process is restarted. Mitigated by Layer 3 (systemd).
- **fork+exec complexity** — managing child PIDs, signal forwarding,
  zombie reaping, and environment inheritance adds non-trivial code to
  Process 7.
- **SHM topic proliferation** — 7 new `ShmThreadHealth` topics. Mitigated
  by keeping the struct small (trivially copyable, < 1 KB).
- **False positives** — a thread doing a legitimate long-running operation
  (e.g., loading a model on startup) could be flagged as stuck. Mitigated
  by allowing threads to temporarily disable their heartbeat via
  `touch_with_grace(30s)`.

### Risks

- **Signal handling** — fork+exec complicates signal propagation. The
  supervisor must mask signals in children and handle SIGCHLD for
  zombie reaping. Well-understood patterns exist for this.
- **Restart storms** — a systemic issue (e.g., corrupt config file) could
  cause all processes to crash-loop. The cooldown window and max-retry
  limit bound this, and the supervisor logs a CRITICAL event.
- **Clock monotonicity** — `steady_clock` is used for all heartbeat
  timestamps to avoid NTP jump issues. The `ShmThreadHealth::timestamp_ns`
  uses the same clock as the existing `ShmSystemHealth::timestamp_ns`.

---

## 5. Implementation Phases

| Phase | Scope | Files | Depends On | Status |
|-------|-------|-------|-----------|--------|
| **Phase 1** | Thread heartbeat + watchdog library | `thread_heartbeat.h`, `thread_watchdog.h`, `safe_name_copy.h`, tests (25) | — | ✅ PR #94 |
| **Phase 2** | SHM thread health struct + per-process publishing | `shm_types.h`, `thread_health_publisher.h`, P1–P7 integration, tests (15) | Phase 1 | ✅ PR #96 |
| **Phase 3** | Process supervisor in System Monitor | `process7_system_monitor/`, launch script changes | Phase 2 | 🔲 Issue #91 |
| **Phase 4** | Restart policies + dependency graph + degraded mode | Config-driven policies, stack status model | Phase 3 | 🔲 Issue #92 |

Tier 4 items (systemd, hardware WDT) are tracked separately in Issues
#83 and the deployment epic.

#### Implementation Notes (Phases 1–2)

During Phase 1 and 2 implementation, several design adjustments were made:

- **`safe_name_copy<N>()`** — a utility template was added to
  `common/util/include/util/safe_name_copy.h` to replace ad-hoc
  `strncpy` calls. It deduces buffer size from the destination array,
  zeroes the buffer, copies at most `N-1` bytes, and explicitly
  null-terminates. A targeted `#pragma GCC diagnostic` suppresses
  `-Wstringop-truncation` in the one audited location, avoiding
  build failures under `-Werror` in Release builds (see CI-008).
- **CAS-based `register_thread()`** — the original `fetch_add` /
  `fetch_sub` rollback pattern for thread registration had a race
  window under concurrent registration. Replaced with a
  compare-and-swap loop that atomically claims slots.
- **`snapshot()` returns `std::vector`** — for Phase 1 simplicity,
  `snapshot()` returns a vector copy. A future optimisation (Issue #97)
  will change this to `std::array<ThreadHeartbeat, kMaxThreads>`
  to eliminate the heap allocation.

---

## 6. Test Strategy (see also §7 OQ-4 for false-positive suppression)

### Unit Tests

| Component | Tests | Validates |
|-----------|-------|-----------|
| `ThreadHeartbeatRegistry` | Register, touch, snapshot, overflow | Lock-free correctness, max-thread handling |
| `ThreadWatchdog` | Stuck detection, callback firing, grace period | Threshold logic, false-positive suppression |
| `ShmThreadHealth` | Trivially copyable, defaults, serialization | SHM compatibility |
| `RestartPolicy` | Backoff calculation, cooldown reset, max-retry, thermal gate | Policy arithmetic |
| `ProcessGraph` | Launch ordering (topological), cascade targets, cycle detection | Dual-edge graph correctness |

### Integration Tests

| Scenario | Validates |
|----------|-----------|
| Process crash + auto-restart | Supervisor detects death, restarts, child runs |
| Thread stuck + self-terminate + restart | End-to-end: stuck → log → kill → restart |
| Max retries exhausted → CRITICAL | Restart storm bounded, status transitions |
| Dependency restart (kill comms → mission_planner restarted too) | Cascade logic |
| Non-cascade (kill perception → video_capture NOT restarted) | Cascade isolation |
| Thermal gate blocks restart, resumes after cooldown | Thermal-aware suppression |

### CI

All tests run in both SHM and Zenoh matrix legs. Thread heartbeat tests
are backend-independent. Supervisor tests use fork+exec of a small test
binary (`test_crasher`) rather than the full stack.

---

## 7. Open Questions & Implementation Notes

These items were identified during design review and should be addressed
during the relevant implementation phase.

### OQ-1: Dependency Graph Direction (Phase 4, #92) — RESOLVED

**Decision:** Use two separate edge types in `ProcessDescriptor`:

- `launch_after[]` — data dependency, controls launch ordering.
- `restart_cascade[]` — stale-state dependency, controls which processes
  to stop+restart when a peer dies.

See §2.3 "Dual-Edge Dependency Graph" for the concrete design, default
graph, and worked examples (comms crash, perception crash).

### OQ-2: Thermal-Aware Restart Suppression (Phase 4, #92) — RESOLVED

**Decision:** Added `thermal_gate` field to `RestartPolicy` (default: 3 =
CRITICAL). When `ShmSystemHealth::thermal_zone >= thermal_gate`, the
supervisor defers restarts and logs the suppression. Restarts resume
automatically when temperature drops.

See §2.3 "Restart Policy" for the updated struct definition.

### OQ-3: Process 7 as Single Point of Failure (Phase 3, #91) — ACCEPTED

**Revised (2026-03-06):** The original design had P7 as the sole systemd
unit managing P1–P6 via fork+exec. Analysis revealed an **orphan
re-adoption problem**: when systemd restarts a crashed P7, the old P1–P6
processes are orphaned under PID 1. The restarted P7 has no record of
their PIDs and would launch duplicates.

**Resolution:** All 7 processes are independent systemd units. P7 is no
longer a single point of failure for process lifecycle — systemd manages
restart independently for each process. P7's crash only loses health
telemetry temporarily (2-second restart gap).

Mitigations (still apply):

1. **Layer 3 (systemd)** — Each process has its own `Restart=on-failure`
   service unit. `drone-stack.target` groups all 7.
2. **Minimal P7 complexity** — the health loop remains simple: read
   sensors, publish `ShmSystemHealth`, call `sd_notify(WATCHDOG=1)`.
3. **Self-monitoring** — P7 registers its own `health_loop` thread in
   the heartbeat registry and runs its own watchdog. If the health loop
   stalls, the watchdog thread logs and stops calling `sd_notify()`,
   triggering a systemd restart of P7 only.

### OQ-4: `touch_with_grace()` for Startup and Long Operations (Phase 1, #89)

Several threads perform one-time operations that exceed the stuck threshold:

| Process | Thread | Operation | Duration |
|---------|--------|-----------|----------|
| P2 Perception | `inference_loop` | YOLO model loading | 5–30 s |
| P3 SLAM | `vio_loop` | ORB vocabulary loading | 10–60 s |
| P5 Comms | `fc_tx_thread` | MAVLink handshake | 3–10 s |

Without `touch_with_grace()`, these would trigger false stuck-thread alerts.

**Implementation guidance:**
- Call `touch_with_grace(30s)` before the long operation starts.
- This bumps `last_touch_ns` to `now + grace`, so the watchdog sees it
  as "recently touched" for the grace period.
- **Do not** use this as a way to suppress real stalls — the grace period
  must match the expected operation duration, not be set arbitrarily high.
- After the long operation completes, call `touch()` to resume normal
  heartbeat cadence.

---

## 8. References

- [Zenoh Liveliness Tokens](https://zenoh.io/docs/manual/liveliness/) — existing detection mechanism
- [docs/process-health-monitoring.md](../process-health-monitoring.md) — Phase F implementation
- [docs/observability.md](../observability.md) — structured logging and correlation IDs
- Issue #28 — FC heartbeat timeout + link-loss contingency
- Issue #31 — systemd service files + process supervisor
- Issue #41 — Contingency fault tree
- Issue #83 — systemd service units (Tier 4)
- Issue #88 — Epic: Process & Thread Watchdog
- Issue #89 / PR #94 — Phase 1: Thread heartbeat + watchdog library
- Issue #90 / PR #96 — Phase 2: SHM thread health + per-process publishing
- Issue #91 — Phase 3: Process supervisor (planned)
- Issue #92 — Phase 4: Restart policies + dependency graph (planned)
- Issue #97 — Tech debt: snapshot() vector → array optimisation
- [CI_ISSUES.md](../CI_ISSUES.md) — CI-008: strncpy truncation warning
