# Foundation Hardening & Watchdog — Design Document

> **Epics:** [#64](https://github.com/nmohamaya/companion_software_stack/issues/64) (Foundation Hardening), [#88](https://github.com/nmohamaya/companion_software_stack/issues/88) (Process & Thread Watchdog)  
> **ADR:** [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md) (Watchdog Architecture)  
> **Status:** Complete (Tiers 1–3 of Epic #64 + all 4 phases of Epic #88 + systemd Issue #83)  
> **Deferred:** Issue #81 (Dockerfile), Issue #82 (cross-compilation) — Tier 4, closer to deployment

---

## Table of Contents

- [Foundation Hardening \& Watchdog — Design Document](#foundation-hardening--watchdog--design-document)
  - [Table of Contents](#table-of-contents)
  - [1. Overview](#1-overview)
  - [2. Architecture](#2-architecture)
    - [2.1 Three-Layer Watchdog](#21-three-layer-watchdog)
    - [2.2 Foundation Hardening Stack](#22-foundation-hardening-stack)
  - [3. Layer 1 — Thread Heartbeat \& Watchdog](#3-layer-1--thread-heartbeat--watchdog)
    - [3.1 ThreadHeartbeatRegistry](#31-threadheartbeatregistry)
    - [3.2 ScopedHeartbeat](#32-scopedheartbeat)
    - [3.3 ThreadWatchdog](#33-threadwatchdog)
    - [3.4 ThreadHealthPublisher](#34-threadhealthpublisher)
  - [4. Layer 2 — Process Supervisor](#4-layer-2--process-supervisor)
    - [4.1 RestartPolicy](#41-restartpolicy)
    - [4.2 ProcessGraph](#42-processgraph)
    - [4.3 ProcessManager](#43-processmanager)
    - [4.4 StackStatus Model](#44-stackstatus-model)
  - [5. Layer 3 — systemd Integration](#5-layer-3--systemd-integration)
    - [5.1 Service Unit Architecture](#51-service-unit-architecture)
    - [5.2 sd\_notify API](#52-sd_notify-api)
    - [5.3 Dependency Semantics](#53-dependency-semantics)
    - [5.4 Restart Rate Limiting](#54-restart-rate-limiting)
    - [5.5 Startup Sequencing — Why Type=notify Matters](#55-startup-sequencing--why-typenotify-matters)
  - [6. Error Handling — Result\<T,E\>](#6-error-handling--resultte)
  - [7. Utilities](#7-utilities)
    - [7.1 safe\_name\_copy](#71-safe_name_copy)
    - [7.2 CorrelationContext](#72-correlationcontext)
  - [8. Configuration Reference](#8-configuration-reference)
  - [9. Thread Criticality Inventory](#9-thread-criticality-inventory)
  - [10. Deployment Modes](#10-deployment-modes)
  - [11. Related Documents](#11-related-documents)

---

## 1. Overview

The hardening work addressed two categories of problems:

1. **Code quality gaps** (Epic #64) — no sanitizers, no structured error handling, no config validation, inconsistent return-value checking.
2. **No fault recovery** (Epic #88 + #83) — a single crashed process killed the entire stack; stuck threads were invisible; no automatic restart.

The result is a three-layer watchdog architecture layered on top of a hardened codebase:

| Layer | Scope | Mechanism | Detects | Recovers |
|-------|-------|-----------|---------|----------|
| **1 — Thread** | Per-thread within each process | `ThreadHeartbeat` atomic + `ThreadWatchdog` scanner | Stuck/deadlocked threads | Self-terminates process (if critical) |
| **2 — Process** | Cross-process in P7 | `ProcessManager` fork+exec + `RestartPolicy` | Crashed processes | Restart with backoff + cascade |
| **3 — OS** | System-wide | systemd `BindsTo` + `WatchdogSec` | Process death (including P7) | `Restart=on-failure` per unit |

Foundation hardening (Epic #64) provides the underpinning:

| Tier | What | PRs |
|------|------|-----|
| **Tier 1** | ASan/TSan/UBSan sanitizers, clang-format gate, lcov coverage | #71, #72, #73 |
| **Tier 2** | `Result<T,E>` monadic errors, config schemas, `[[nodiscard]]` | #74, #75, #76, #77 |
| **Tier 3** | Latency histograms, JSON structured logging, correlation IDs | see Epic #64 |
| **Tier 4** | systemd units (Issue #83 → PR #107), tech debt cleanup (PRs #97, #98 → #108) | #97, #98, #107, #108 |

---

## 2. Architecture

### 2.1 Three-Layer Watchdog

```
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 3: OS Supervisor — systemd                                     │
│   7 independent .service units + drone-stack.target                   │
│   Type=notify + WatchdogSec=10s on all 7 units                      │
│   BindsTo= dependency semantics, StartLimitBurst=5 in [Unit]        │
│   Restart=on-failure for all units                                   │
└──────────────────────────┬───────────────────────────────────────────┘
                           │ manages each process independently
┌──────────────────────────▼───────────────────────────────────────────┐
│ Layer 2: Process Supervisor — ProcessManager (in P7, --supervised)    │
│                                                                      │
│  Detection:   Zenoh liveliness + PID polling + ShmThreadHealth reads │
│  Recovery:    fork+exec with RestartPolicy (backoff, cooldown, gate) │
│  Graph:       ProcessGraph (launch_after + restart_cascade edges)    │
│  Status:      StackStatus: NOMINAL → DEGRADED → CRITICAL            │
│  Reporting:   Structured spdlog with CorrelationContext              │
└──────────────────────────┬───────────────────────────────────────────┘
                           │ reads ShmThreadHealth from each process
┌──────────────────────────▼───────────────────────────────────────────┐
│ Layer 1: Thread Heartbeat — in every process (P1–P7)                 │
│                                                                      │
│  ThreadHeartbeat:   atomic<uint64_t> last_touch_ns per thread        │
│  ThreadHeartbeatRegistry:   process-global singleton (max 16 slots)  │
│  ScopedHeartbeat:   RAII wrapper (register + touch)                  │
│  ThreadWatchdog:    background scanner, fires StuckCallback          │
│  ThreadHealthPublisher:   snapshot → ShmThreadHealth → IPublisher    │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.2 Foundation Hardening Stack

```
┌─────────────────────────────────────────────────────────────┐
│  Application Code (P1–P7)                                   │
│    uses Result<T,E>, [[nodiscard]], ConfigSchema             │
├─────────────────────────────────────────────────────────────┤
│  Utility Layer (common/util/)                               │
│    result.h, safe_name_copy.h, correlation.h,               │
│    thread_heartbeat.h, thread_watchdog.h,                   │
│    thread_health_publisher.h, restart_policy.h,             │
│    process_graph.h, sd_notify.h                             │
├─────────────────────────────────────────────────────────────┤
│  IPC Layer (common/ipc/)                                    │
│    IPublisher<T>, ISubscriber<T>, ShmThreadHealth struct    │
├─────────────────────────────────────────────────────────────┤
│  CI Pipeline                                                │
│    ASan + TSan + UBSan, clang-format-18, lcov coverage      │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Layer 1 — Thread Heartbeat & Watchdog

**Header:** `common/util/include/util/thread_heartbeat.h`  
**Namespace:** `drone::util`  
**PR:** [#94](https://github.com/nmohamaya/companion_software_stack/pull/94) (Phase 1), [#96](https://github.com/nmohamaya/companion_software_stack/pull/96) (Phase 2)

### 3.1 ThreadHeartbeatRegistry

Process-global singleton that tracks per-thread heartbeat timestamps.

```cpp
class ThreadHeartbeatRegistry {
public:
    static ThreadHeartbeatRegistry& instance();

    // Register a thread. Returns handle for touch(), or kInvalidHandle if full.
    size_t register_thread(const char* name, bool critical = false);

    // Touch — call every loop iteration. Cost: ~1 ns (atomic store + clock read).
    void touch(size_t handle);

    // Touch with grace period for known long operations (model loading, etc).
    void touch_with_grace(size_t handle, std::chrono::milliseconds grace);

    // Snapshot all registered heartbeats (stack-allocated, no heap).
    [[nodiscard]] ThreadSnapshot snapshot() const;

    // Number of registered heartbeats.
    [[nodiscard]] size_t count() const;
};
```

**Constants:**
- `kMaxThreads = 16` — max heartbeats per process (headroom over current 6 in P2)
- `kInvalidHandle` — sentinel returned when the registry is full

**Thread Safety:**
- `register_thread()` — CAS loop (not `fetch_add`) for concurrent-safe slot claiming
- `touch()` — single `atomic_store(relaxed)` to own slot only
- `snapshot()` — reads all atomics; checks `initialized` flag with acquire ordering to skip partially-written slots

**Data Structures:**

```cpp
struct ThreadHeartbeat {
    char                  name[32] = {};
    std::atomic<uint64_t> last_touch_ns{0};
    bool                  is_critical = false;
    std::atomic<bool>     initialized{false};  // acquire/release gate
};

struct ThreadSnapshot {
    std::array<ThreadHeartbeat, kMaxThreads> beats{};
    size_t count = 0;
    // Iterable: begin(), end(), size(), empty(), operator[]
};
```

### 3.2 ScopedHeartbeat

RAII convenience wrapper — registers on construction, `touch()` in loop.

```cpp
class ScopedHeartbeat {
public:
    explicit ScopedHeartbeat(const char* name, bool critical = false);
    void touch();
    void touch_with_grace(std::chrono::milliseconds grace);
    [[nodiscard]] bool is_valid() const;
    [[nodiscard]] size_t handle() const;
};
```

**Usage in process code:**

```cpp
ScopedHeartbeat hb("inference", /*critical=*/true);
hb.touch_with_grace(std::chrono::seconds{30});  // model loading
load_model();
while (running_) {
    hb.touch();
    // ... work ...
}
```

### 3.3 ThreadWatchdog

**Header:** `common/util/include/util/thread_watchdog.h`

Background thread that periodically scans the heartbeat registry for stuck threads.

```cpp
class ThreadWatchdog {
public:
    struct Config {
        std::chrono::milliseconds stuck_threshold{5000};  // 5s default
        std::chrono::milliseconds scan_interval{1000};    // 1s default
    };

    explicit ThreadWatchdog(Config cfg = {});
    ~ThreadWatchdog();  // joins scan thread

    using StuckCallback = std::function<void(const ThreadHeartbeat& beat)>;
    void set_stuck_callback(StuckCallback cb);

    [[nodiscard]] std::vector<std::string> get_stuck_threads() const;
};
```

**Scan Logic:**
1. Snapshot heartbeats from registry
2. For each beat with `last_touch_ns > 0` (started) and not in the future (grace):
   - If `now_ns - last_touch_ns > stuck_threshold_ns` → stuck
3. Log only on healthy→stuck transition (prevents log storms)
4. Fire `StuckCallback` for each stuck thread
5. Update internal stuck list (thread-safe via mutex)

### 3.4 ThreadHealthPublisher

**Header:** `common/util/include/util/thread_health_publisher.h`

Bridges the heartbeat registry + watchdog → `ShmThreadHealth` → IPC publisher.

```cpp
template<typename Publisher>
class ThreadHealthPublisher {
public:
    ThreadHealthPublisher(Publisher& pub, const char* process, const ThreadWatchdog& watchdog);
    void publish_snapshot();  // Call periodically from health loop
};
```

**Data flow:**

```
ThreadHeartbeatRegistry::snapshot()
         │
         ▼
ThreadHealthPublisher::publish_snapshot()
    ├── cross-reference with watchdog.get_stuck_threads()
    ├── fill ShmThreadHealth struct
    └── pub_.publish(health)
              │
              ▼
         IPublisher<ShmThreadHealth>   →   P7 subscribes, reads health
```

---

## 4. Layer 2 — Process Supervisor

**Header:** `process7_system_monitor/include/monitor/process_manager.h`  
**Namespace:** `drone::monitor`  
**PRs:** [#100](https://github.com/nmohamaya/companion_software_stack/pull/100) (Phase 3), [#101](https://github.com/nmohamaya/companion_software_stack/pull/101) + [#102](https://github.com/nmohamaya/companion_software_stack/pull/102) (Phase 4)

### 4.1 RestartPolicy

**Header:** `common/util/include/util/restart_policy.h`  
**Namespace:** `drone::util`

Per-process restart configuration with thermal gating.

```cpp
struct RestartPolicy {
    uint32_t max_restarts       = 5;
    uint32_t cooldown_window_s  = 60;     // Reset counter after stable run
    uint32_t initial_backoff_ms = 500;    // Doubles each attempt
    uint32_t max_backoff_ms     = 30000;  // Cap at 30s
    bool     is_critical        = false;  // Stack → CRITICAL on exhaustion
    uint8_t  thermal_gate       = 3;      // Block at thermal_zone >= this

    [[nodiscard]] uint32_t backoff_ms(uint32_t attempt) const;
    [[nodiscard]] bool is_thermal_blocked(uint8_t thermal_zone) const;
};
```

**Backoff formula:** `delay = min(initial_backoff_ms × 2^attempt, max_backoff_ms)`

**Thermal gate values:**

| Value | Meaning |
|-------|---------|
| 0 | Always block (restarts disabled) |
| 1 | Block at WARM or hotter |
| 2 | Block at HOT or hotter |
| 3 | Block at CRITICAL only (default) |
| 4 | Never block |

**ProcessConfig** — loads per-process policy from JSON:

```cpp
struct ProcessConfig {
    std::string              name;
    std::string              binary;
    RestartPolicy            policy;
    std::vector<std::string> launch_after;
    std::vector<std::string> restart_cascade;

    static ProcessConfig from_json(const std::string& name, const nlohmann::json& j);
};
```

### 4.2 ProcessGraph

**Header:** `common/util/include/util/process_graph.h`  
**Namespace:** `drone::util`

Dual-edge dependency graph with two separate edge types:

| Edge type | Meaning | Example |
|-----------|---------|---------|
| `launch_after` | Data dependency — X must be alive before launching Y | perception → video_capture |
| `restart_cascade` | Stale-state — if X dies, stop+restart Y | comms → mission_planner |

```cpp
class ProcessGraph {
public:
    void add_process(const std::string& name);
    void add_launch_dep(const std::string& child, const std::string& parent);
    void add_cascade(const std::string& source, const std::string& target);

    [[nodiscard]] std::vector<std::string> launch_order() const;      // Kahn's topological sort
    [[nodiscard]] std::vector<std::string> cascade_targets(const std::string& process) const;  // BFS
    [[nodiscard]] bool validate() const;                               // Cycle + reference checks
    void populate_defaults();                                          // Standard 6-process graph
};
```

**Default dependency graph:**

```
  Launch order (solid):              Restart cascade (dashed):

  video_capture                      comms ─ ─ ─► mission_planner
       │                             comms ─ ─ ─► payload_manager
  perception                         slam_vio_nav ─ ─► perception
       │
  slam_vio_nav        comms
        └────┬───────┘
             ▼
       mission_planner    payload_manager
```

**Cascade example (comms crash):**
1. Supervisor detects comms death
2. Cascade stops `mission_planner` + `payload_manager` (hold stale FC state)
3. Restart `comms`
4. Once comms is alive → restart `mission_planner`, `payload_manager`
5. `slam_vio_nav`, `perception`, `video_capture` NOT restarted

### 4.3 ProcessManager

**Namespace:** `drone::monitor`

Fork+exec supervisor with PID tracking, automatic restart, and cascade support.

```cpp
class ProcessManager {
public:
    explicit ProcessManager(RestartPolicy default_policy = {});

    // Registration
    void add_process(const char* name, const char* binary, const char* args = "");
    void add_process(const char* name, const char* binary, const char* args,
                     const RestartPolicy& policy);

    // Lifecycle
    void launch_all();
    bool launch(const char* name);
    bool stop(const char* name, std::chrono::milliseconds timeout = 2000ms);
    void stop_all(std::chrono::milliseconds timeout = 2000ms);

    // Periodic scan — call at ~1 Hz
    void tick();            // Reap zombies, detect deaths, trigger restarts
    void reap_children();   // waitpid(WNOHANG) polling

    // Configuration
    void set_death_callback(DeathCallback cb);
    void set_process_graph(const ProcessGraph* graph);
    void set_thermal_zone(uint8_t zone);

    // Queries
    [[nodiscard]] StackStatus compute_stack_status() const;
    [[nodiscard]] uint32_t total_restarts() const;
    [[nodiscard]] std::vector<ManagedProcess> get_all() const;
    [[nodiscard]] const RestartPolicy& get_policy(const char* name) const;
};
```

**Process lifecycle states:**

```
STOPPED ──► RUNNING ──► (death detected) ──► RESTARTING ──► RUNNING
                                                  │
                                          (max retries exceeded)
                                                  │
                                                  ▼
                                               FAILED
```

**`tick()` logic:**
1. `reap_children()` — `waitpid(WNOHANG)` for zombie collection
2. For each RESTARTING process:
   - Check cascade gate (`blocked_by` source must be RUNNING)
   - Check thermal gate (`policy.is_thermal_blocked(thermal_zone)`)
   - Check backoff timer
   - Launch when all gates pass
3. For each RUNNING process with `restart_count > 0`:
   - Reset counter if stable for `cooldown_window_s`

### 4.4 StackStatus Model

```
NOMINAL ─────► DEGRADED ─────► CRITICAL
   ▲                │
   └────────────────┘
   (process recovered)
```

| Status | Meaning | Trigger |
|--------|---------|---------|
| **NOMINAL** | All processes alive, all critical threads healthy | All processes RUNNING |
| **DEGRADED** | Non-critical process down or restarting | Any process RESTARTING or non-critical FAILED |
| **CRITICAL** | Critical process exhausted retries | `is_critical` process enters FAILED |

---

## 5. Layer 3 — systemd Integration

**PRs:** [#107](https://github.com/nmohamaya/companion_software_stack/pull/107), [#132](https://github.com/nmohamaya/companion_software_stack/pull/132), [#133](https://github.com/nmohamaya/companion_software_stack/pull/133)  
**Issues:** [#83](https://github.com/nmohamaya/companion_software_stack/issues/83), [#131](https://github.com/nmohamaya/companion_software_stack/issues/131)

### 5.1 Service Unit Architecture

Seven independent systemd units (Option B) — chosen over a single P7 unit (Option A) to avoid the **orphan re-adoption problem**: if systemd restarts a crashed P7, old P1–P6 children are orphaned under PID 1 and invisible to the new P7 instance.

```
drone-stack.target
    │
    ├── drone-video-capture.service      (P1, Type=notify, WatchdogSec=10s)
    ├── drone-perception.service         (P2, Type=notify, WatchdogSec=10s, BindsTo=P1)
    ├── drone-slam-vio-nav.service       (P3, Type=notify, WatchdogSec=10s, BindsTo=P2)
    ├── drone-comms.service              (P5, Type=notify, WatchdogSec=10s)
    ├── drone-mission-planner.service    (P4, Type=notify, WatchdogSec=10s, BindsTo=P3+P5)
    ├── drone-payload-manager.service    (P6, Type=notify, WatchdogSec=10s, BindsTo=P5)
    └── drone-system-monitor.service     (P7, Type=notify, WatchdogSec=10s)
```

All units have `StartLimitBurst=5` and `StartLimitIntervalSec=30s` in their `[Unit]` section (see [5.4](#54-restart-rate-limiting)).

**Security hardening per unit:**
- `NoNewPrivileges=yes`
- `ProtectSystem=strict` (read-only root)
- `PrivateTmp=yes`
- `ReadWritePaths=` restricted to config, log, SHM paths
- CPU/memory limits via `CPUQuota` and `MemoryMax`

### 5.2 sd_notify API

**Header:** `common/util/include/util/sd_notify.h`  
**Namespace:** `drone::systemd`

Thin wrapper around `sd_notify()`. All functions are no-ops when built without `-DENABLE_SYSTEMD=ON`.

```cpp
namespace drone::systemd {
    void     notify_ready();                    // READY=1 (Type=notify)
    void     notify_watchdog();                 // WATCHDOG=1 (pet the timer)
    void     notify_stopping();                 // STOPPING=1 (graceful shutdown)
    void     notify_status(const char* status); // STATUS=... (visible in systemctl status)
    bool     watchdog_enabled();                // Is WatchdogSec active?
    uint64_t watchdog_usec();                   // Watchdog interval (0 if inactive)
}
```

**All 7 processes** call all three lifecycle notifications. The canonical pattern (from `main.cpp` of each process):

```cpp
// --- Initialization complete ---
spdlog::info("[Process N] READY");
drone::systemd::notify_ready();      // systemd releases dependents

// --- Main loop ---
while (g_running) {
    drone::systemd::notify_watchdog();  // must fire within WatchdogSec=10s
    // ... do work ...
    std::this_thread::sleep_for(1s);
}

// --- Graceful shutdown ---
spdlog::info("Shutting down...");
drone::systemd::notify_stopping();   // tells systemd this exit is intentional
```

**What each call does:**

| Call | systemd effect | Without it |
|------|---------------|------------|
| `notify_ready()` | Releases dependent units; marks service `active` | Dependent units start immediately on process launch, before init is complete |
| `notify_watchdog()` | Resets systemd's 10 s watchdog timer | After 10 s silence systemd kills and restarts the process |
| `notify_stopping()` | Marks shutdown as intentional | Graceful `SIGTERM` exit may be logged as unexpected and trigger restart |

### 5.3 Dependency Semantics

**`BindsTo=` vs `Requires=`:**

| Directive | On dependency stop/disappear | On dependency restart |
|-----------|-----------------------------|-----------------------|
| `Requires=` | Dependent stopped | Dependent stays running |
| `BindsTo=` | Dependent **also stopped** | Dependent stays running |

`BindsTo=` was chosen over `Requires=` because it provides stronger stop propagation — when a dependency crashes, systemd stops all bound dependents too. Combined with `Restart=on-failure` on each unit, systemd then restarts both the crashed dependency and its stopped dependents independently. This clears stale state in dependents, matching the `ProcessGraph` cascade semantics.

### 5.4 Restart Rate Limiting

All 7 units have the following in their `[Unit]` section:

```ini
[Unit]
...
StartLimitBurst=5
StartLimitIntervalSec=30s
```

This caps restart attempts to **5 crashes within any 30-second window**. After the limit is hit, systemd marks the unit `failed` and stops retrying.

**Why this must be in `[Unit]`, not `[Service]`:** `StartLimitBurst` is a unit-level restart gate, not a service process directive. Placed under `[Service]` it is silently ignored — the restart storm protection never applies. This was a bug found during PR #132 review and corrected in PR #133.

**Effect:**

| Scenario | Without limit | With limit |
|----------|--------------|------------|
| Process crashes on bad config at startup | Infinite restart loop, CPU spins | Fails after 5 attempts in 30 s → operator alerted |
| Transient crash (OOM spike) | Restarts fine | Restarts fine (single event, well within 5/30s) |
| Cascading failure storm | All 7 units cycling indefinitely | Each unit fails independently → clear failure state |

### 5.5 Startup Sequencing — Why Type=notify Matters

Processes 1–6 were originally `Type=simple` (PR #107, Issue #83). This was upgraded to `Type=notify` for all 7 processes in PR #132 (Issue #131).

**With `Type=simple`** — systemd considers a unit started the moment the binary is launched:

```
systemd launches P1
→ immediately starts P2 (BindsTo=P1 only waits for P1 to launch, not initialize)
→ P2 tries to subscribe to SHM/Zenoh topics P1 hasn't published yet
→ P2 misses first frames, or reads stale/zero data
```

**With `Type=notify`** — systemd waits for an explicit `READY=1` signal:

```
systemd launches P1
  P1 loads config, initialises camera backend, starts threads
  P1 calls notify_ready()
systemd releases P2
  P2 starts with P1 already publishing frames
systemd releases P3 (only after P2 signals ready)
  ...
```

The `WatchdogSec=10s` setting is only meaningful with `Type=notify` — systemd ignores watchdog pings on `Type=simple` units.

---

## 6. Error Handling — Result\<T,E\>

**Header:** `common/util/include/util/result.h`  
**Namespace:** `drone::util`  
**PR:** [#75](https://github.com/nmohamaya/companion_software_stack/pull/75)

Lightweight monadic error type replacing exceptions in the hot path.

```cpp
// Error type with code + message
enum class ErrorCode : uint8_t {
    Unknown, FileNotFound, ParseError, InvalidValue, MissingKey,
    TypeMismatch, OutOfRange, Timeout, NotConnected, AlreadyExists
};

class Error {
public:
    Error(ErrorCode code, std::string message);
    [[nodiscard]] ErrorCode code() const;
    [[nodiscard]] const std::string& message() const;
};

// Result<T, E> — success or error
template<typename T, typename E = Error>
class [[nodiscard]] Result {
public:
    static Result ok(T value);
    static Result err(E error);

    [[nodiscard]] bool is_ok() const;
    [[nodiscard]] bool is_err() const;
    [[nodiscard]] const T& value() const;
    [[nodiscard]] const E& error() const;
    [[nodiscard]] T value_or(T fallback) const;

    // Monadic operations
    template<typename F> auto map(F&& func) const;       // T → U
    template<typename F> auto and_then(F&& func) const;   // T → Result<U,E>
    template<typename F> auto map_error(F&& func) const;  // E → E2
};

// Void specialisation
template<typename E> class Result<void, E>;
using VoidResult = Result<void, Error>;
```

**Usage:**

```cpp
auto r = parse_port("8080");
int port = r.map([](int p) { return p + 1; })
            .value_or(9090);
```

---

## 7. Utilities

### 7.1 safe_name_copy

**Header:** `common/util/include/util/safe_name_copy.h`

Replaces raw `strncpy` calls for fixed-size SHM buffers. Deduces buffer size, zeroes, copies, null-terminates. Suppresses `-Wstringop-truncation` in one audited location.

```cpp
template<std::size_t N>
inline void safe_name_copy(char (&dst)[N], const char* src);
```

### 7.2 CorrelationContext

Generates unique 64-bit correlation IDs for structured log events, enabling post-flight analysis to trace restart → cascade → recovery chains across processes.

---

## 8. Configuration Reference

Watchdog configuration in `config/default.json`:

```json
{
  "watchdog": {
    "stuck_threshold_ms": 5000,
    "scan_interval_ms": 1000,
    "processes": {
      "video_capture": {
        "binary": "build/bin/video_capture",
        "critical": true,
        "max_restarts": 5,
        "cooldown_s": 60,
        "backoff_ms": 500,
        "max_backoff_ms": 30000,
        "thermal_gate": 3,
        "launch_after": [],
        "restart_cascade": []
      },
      "perception": {
        "binary": "build/bin/perception",
        "critical": true,
        "launch_after": ["video_capture"],
        "restart_cascade": []
      },
      "comms": {
        "binary": "build/bin/comms",
        "critical": true,
        "launch_after": [],
        "restart_cascade": ["mission_planner", "payload_manager"]
      }
    }
  }
}
```

---

## 9. Thread Criticality Inventory

| Process | Thread | Critical | Rationale |
|---------|--------|:--------:|-----------|
| **P1** | `mission_cam` | Yes | Primary camera — no visual data without it |
| **P1** | `stereo_cam` | Yes | Stereo depth — required for VIO + obstacles |
| **P2** | `inference` | Yes | YOLO detection — removes obstacle awareness |
| **P2** | `tracker` | Yes | Object tracking — stale tracks cause bad planning |
| **P2** | `fusion` | Yes | Sensor fusion output — downstream depends on it |
| **P3** | `visual_frontend` | Yes | Visual odometry — no pose without it |
| **P3** | `imu_reader` | Yes | IMU — no attitude/velocity estimation |
| **P3** | `pose_publisher` | Yes | Pose output — P4, P5 lose localisation |
| **P4** | `planning_loop` | Yes | Waypoint + contingency — uncontrolled flight |
| **P5** | `fc_rx` | Yes | FC telemetry — no state feedback from PX4 |
| **P5** | `fc_tx` | Yes | FC commands — triggers PX4 link-loss failsafe |
| **P5** | `gcs_rx` | No | GCS commands — autonomous flight continues |
| **P5** | `gcs_tx` | No | GCS telemetry — flight safe without it |
| **P6** | `payload_loop` | No | Payload — non-essential for safety |
| **P7** | `health_loop` | No | Monitoring — loss disables telemetry only |

**Summary:** 17 threads — 11 critical, 6 non-critical.

---

## 10. Deployment Modes

| Mode | How P7 Runs | Who Manages P1–P6 | Use Case |
|------|-------------|-------------------|----------|
| **systemd** (production) | `drone-system-monitor.service` (monitor-only) | systemd `Restart=on-failure` | Flight hardware |
| **`--supervised`** | P7 fork+execs P1–P6 | ProcessManager in P7 | Docker / no-systemd |
| **`launch_all.sh`** | Separate shell process | Shell script `wait -n` | Development |

---

## 11. Related Documents

| Document | What It Covers |
|----------|---------------|
| [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md) | Decision record: why this architecture, alternatives rejected |
| [API.md](API.md) | Interface API reference (includes watchdog APIs in §4) |
| [CPP_PATTERNS_GUIDE.md](CPP_PATTERNS_GUIDE.md) | C++ patterns used (CAS loops, fork+exec, RAII heartbeats) |
| [process-health-monitoring.md](process-health-monitoring.md) | Zenoh liveliness tokens (Layer 2 detection path) |
| [observability.md](observability.md) | Structured logging, correlation IDs, JSON format |
| [DEPLOY_USAGE.md](DEPLOY_USAGE.md) | systemd install/uninstall, service management commands |
| [tests/TESTS.md](../tests/TESTS.md) | Test index (source of truth for test counts) |
| [ROADMAP.md](ROADMAP.md) | Project roadmap with historical phase progression |
| [PROGRESS.md](PROGRESS.md) | Detailed per-improvement changelog |
