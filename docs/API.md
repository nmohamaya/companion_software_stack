# Interface API Reference

> **Issue:** #4 — API-Driven Development  
> **Status:** All 7 processes wired through abstract interfaces  
> **IPC Migration:** Zenoh Phase A+B done — [ADR-001](adr/ADR-001-ipc-framework-selection.md), Epic [#45](https://github.com/nmohamaya/companion_software_stack/issues/45), PRs [#52](https://github.com/nmohamaya/companion_software_stack/pull/52) + [#53](https://github.com/nmohamaya/companion_software_stack/pull/53)

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│              MessageBus (Zenoh — sole backend)                    │
│    advertise<T>(topic)  →  IPublisher<T>                         │
│    subscribe<T>(topic)  →  ISubscriber<T>                        │
├──────────────────────────────────────────────────────────────────┤
│  ZenohPublisher<T>    /   ZenohSubscriber<T>                     │
│  Zenoh SHM (zero-copy) + network (UDP/TCP to GCS)                │
│  Liveliness tokens for process health                            │
└──────────────────────────────────────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                   Process Internals                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────────┐ │
│  │IVisualFrontend│ │ IPathPlanner │ │ IObstacleAvoider         │ │
│  │ (P3: SLAM)   │ │ (P4: Mission)│ │ (P4: Mission)            │ │
│  └──────────────┘ └──────────────┘ └──────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ IProcessMonitor  (P7: System Monitor)                    │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

All interfaces follow the **Strategy pattern**: abstract base class + concrete implementation + factory function. Swap implementations at construction time via config string.

The IPC backend is Zenoh (the sole backend since Issue #126 removed the legacy POSIX SHM backend). Process code depends only on `IPublisher<T>` / `ISubscriber<T>` — transport is invisible. See [ADR-001](adr/ADR-001-ipc-framework-selection.md) for the decision record.

---

## 1. IPC Layer — Pub-Sub Messaging

### `IPublisher<T>` — `drone::ipc`

**Header:** `common/ipc/include/ipc/ipublisher.h`

Abstract typed publisher. Decouples processes from transport.

| Method | Signature | Description |
|--------|-----------|-------------|
| `publish` | `void publish(const T& msg)` | Publish a message (non-blocking, latest-value) |
| `topic_name` | `const std::string& topic_name() const` | Topic/channel name |
| `is_ready` | `bool is_ready() const` | True if publisher can accept data |

**Constraint:** `T` must be trivially copyable.

### `ISubscriber<T>` — `drone::ipc`

**Header:** `common/ipc/include/ipc/isubscriber.h`

Abstract typed subscriber.

| Method | Signature | Description |
|--------|-----------|-------------|
| `receive` | `bool receive(T& out, uint64_t* ts = nullptr) const` | Read latest message; returns true on consistent read |
| `is_connected` | `bool is_connected() const` | True if connected to data source |
| `topic_name` | `const std::string& topic_name() const` | Topic/channel name |

### `ZenohSession` — Singleton Session Manager

**Header:** `common/ipc/include/ipc/zenoh_session.h`  
**Note:** Always available (Zenoh is the sole backend since Issue #126).

Process-wide Zenoh session singleton. All `ZenohPublisher` / `ZenohSubscriber` instances share this session. Opened lazily on first access in peer mode (no daemon required).

| Method | Signature | Description |
|--------|-----------|-------------|
| `instance` | `static ZenohSession& instance()` | Access the singleton (heap-allocated, intentionally leaked — see BUG_FIXES.md #7) |
| `configure` | `void configure(const std::string& config_json = "")` | Set custom Zenoh config JSON before first use |
| `session` | `zenoh::Session& session()` | Get the underlying session (lazy open) |
| `is_open` | `bool is_open() const` | True if session has been opened |

**Why a wrapper?** Manages session lifetime across all pub/sub instances. The intentionally-leaked singleton avoids a zenohc Rust 1.85 atexit() panic (rust-lang/rust#138696).

### `ZenohPublisher<T>` — Concrete

**Header:** `common/ipc/include/ipc/zenoh_publisher.h`  
**Note:** Always available (Zenoh is the sole backend since Issue #126).

Zenoh-backed publisher implementing `IPublisher<T>`. Serializes trivially-copyable `T` to `zenoh::Bytes` (via `std::vector<uint8_t>`) and calls `Publisher::put(Bytes&&)`.

```cpp
ZenohPublisher<drone::ipc::Pose> pub("drone/slam/pose");
pub.publish(pose);  // Serialized as raw bytes → Zenoh network
```

**Why a wrapper?** Simplifies the raw zenoh-cpp API (explicit `Bytes` construction, rvalue semantics) down to a single `publish(const T&)` call. Enforces `static_assert(is_trivially_copyable_v<T>)` for type safety.

### `ZenohSubscriber<T>` — Concrete

**Header:** `common/ipc/include/ipc/zenoh_subscriber.h`  
**Note:** Always available (Zenoh is the sole backend since Issue #126).

Zenoh-backed subscriber implementing `ISubscriber<T>`. Maintains a latest-value cache updated by the Zenoh callback thread. Uses `std::mutex` + `std::atomic` for thread-safe access between the Zenoh callback thread and the process main loop.

```cpp
ZenohSubscriber<Pose> sub("drone/slam/pose");
Pose pose;
if (sub.receive(pose)) { /* use pose */ }
```

**Why a wrapper?** The raw zenoh-cpp `Subscriber<void>` requires two callbacks (`on_sample` + `on_drop`), manual `Bytes::as_vector()` deserialization, and size validation. The wrapper handles all of this and exposes a clean `receive(T&)` polling API.

### `ZenohMessageBus` — Factory ([#46](https://github.com/nmohamaya/companion_software_stack/issues/46))

**Header:** `common/ipc/include/ipc/zenoh_message_bus.h`  
**Note:** Always available (Zenoh is the sole backend since Issue #126).

The sole message bus implementation. Provides `advertise<T>()` / `subscribe<T>()` backed by Eclipse Zenoh. Maps channel names to Zenoh key expressions.

| Method | Signature | Description |
|--------|-----------|-------------|
| `advertise` | `unique_ptr<IPublisher<T>> advertise<T>(topic)` | Create a `ZenohPublisher<T>` |
| `subscribe` | `unique_ptr<ISubscriber<T>> subscribe<T>(topic)` | Create a `ZenohSubscriber<T>` |
| `subscribe_lazy` | `unique_ptr<ZenohSubscriber<T>> subscribe_lazy<T>(topic)` | Equivalent to `subscribe()` — Zenoh connections are always async |
| `to_key_expr` | `static std::string to_key_expr(const std::string& channel_name)` | Map channel name → Zenoh key expression (guards empty input) |

**Why a wrapper?** Maps the 12 channel names to Zenoh hierarchical key expressions. Provides a consistent `advertise<T>()` / `subscribe<T>()` factory interface so process code is agnostic to the transport.

**Key-expression mapping** (channel names → Zenoh topics):

| Channel Name | Zenoh Key Expression |
|----------|---------------------|
| `/drone_mission_cam` | `drone/video/frame` |
| `/drone_stereo_cam` | `drone/video/stereo_frame` |
| `/detected_objects` | `drone/perception/detections` |
| `/slam_pose` | `drone/slam/pose` |
| `/mission_status` | `drone/mission/status` |
| `/trajectory_cmd` | `drone/mission/trajectory` |
| `/payload_commands` | `drone/mission/payload_command` |
| `/fc_commands` | `drone/comms/fc_command` |
| `/fc_state` | `drone/comms/fc_state` |
| `/gcs_commands` | `drone/comms/gcs_command` |
| `/payload_status` | `drone/payload/status` |
| `/system_health` | `drone/monitor/health` |

### `MessageBusFactory` — Config-Driven Backend Selection

**Header:** `common/ipc/include/ipc/message_bus_factory.h`

Returns the Zenoh message bus. `ZenohMessageBus` is the sole backend (legacy SHM removed in Issue #126). Unknown backends log an error and fall back to Zenoh; the config schema additionally rejects non-`"zenoh"` values at startup.

| Function/Method | Signature | Description |
|-----------------|-----------|-------------|
| `create_message_bus` | `MessageBus create_message_bus(const std::string& backend = "zenoh")` | Create and return a `MessageBus` |
| `bus.advertise<T>` | `unique_ptr<IPublisher<T>> advertise<T>(topic)` | Create a publisher on the bus |
| `bus.subscribe<T>` | `unique_ptr<ISubscriber<T>> subscribe<T>(topic, max_retries, retry_ms)` | Create a subscriber with optional retries |
| `bus.subscribe_optional<T>` | `unique_ptr<ISubscriber<T>> subscribe_optional<T>(topic)` | Single-attempt subscribe; `is_connected()` is true once declared successfully (does not depend on publisher presence). |

**Why a wrapper?** Reads `"ipc_backend"` from config (defaults to `"zenoh"`). Process code calls `bus.advertise<T>()` / `bus.subscribe<T>()` — completely agnostic to the transport.

```cpp
auto bus = create_message_bus(cfg);  // Returns MessageBus wrapping ZenohMessageBus
auto pub = bus.advertise<Pose>("drone/slam/pose");
auto sub = bus.subscribe<Pose>("drone/slam/pose");

pub->publish(pose);
Pose out;
sub->receive(out);
```

**Zenoh features:**
- Zero-copy SHM via loan-based API (no `memcpy` for 6 MB video frames)
- Network-transparent — same pub/sub reachable from GCS over TCP/UDP
- Wildcard subscriptions: `drone/**` captures all channels
- Liveliness tokens for process death detection
- Configurable QoS (reliable/best-effort, history depth)

---

## 2. IPC Layer — Request-Response Services

### `IServiceClient<Req, Resp>` — `drone::ipc`

**Header:** `common/ipc/include/ipc/iservice_channel.h`

| Method | Signature | Description |
|--------|-----------|-------------|
| `send_request` | `uint64_t send_request(const Req& request)` | Send request; returns correlation ID |
| `poll_response` | `optional<ServiceResponse<Resp>> poll_response(uint64_t id)` | Non-blocking poll for matching response |
| `await_response` | `optional<ServiceResponse<Resp>> await_response(uint64_t id, milliseconds timeout)` | Blocking wait with timeout; returns `TIMEOUT` status on expiry |

### `IServiceServer<Req, Resp>` — `drone::ipc`

| Method | Signature | Description |
|--------|-----------|-------------|
| `poll_request` | `optional<ServiceEnvelope<Req>> poll_request()` | Non-blocking poll for incoming request |
| `send_response` | `void send_response(uint64_t id, ServiceStatus status, const Resp& resp)` | Send response for a given correlation ID |

### Supporting Types

```cpp
enum class ServiceStatus : uint8_t { OK = 0, REJECTED = 1, TIMEOUT = 2, ERROR = 3 };

template <typename T>
struct ServiceEnvelope {
    uint64_t correlation_id;
    uint64_t timestamp_ns;
    bool valid;
    T payload;
};

template <typename T>
struct ServiceResponse {
    uint64_t correlation_id;
    uint64_t timestamp_ns;
    ServiceStatus status;
    bool valid;
    T payload;
};
```

---

## 3. Process Strategy Interfaces

### `IVisualFrontend` — `drone::slam`

**Header:** `process3_slam_vio_nav/include/slam/ivisual_frontend.h`  
**Used by:** Process 3 (SLAM/VIO/Nav)

| Method | Signature | Description |
|--------|-----------|-------------|
| `process_frame` | `drone::slam::Pose process_frame(const drone::ipc::StereoFrame& frame)` | Process stereo frame, return 6-DOF pose |
| `name` | `std::string name() const` | Implementation name for logging |

| Implementation | Description |
|----------------|-------------|
| `SimulatedVisualFrontend` | Circular trajectory (r=5m, h=2m, 30Hz) + Gaussian noise (σ=0.01) |

**Factory:** `create_visual_frontend(backend)` — backends: `"simulated"`

```cpp
auto fe = create_visual_frontend("simulated");
Pose p = fe->process_frame(stereo_frame);
```

---

### `IPathPlanner` — `drone::planner`

**Header:** `process4_mission_planner/include/planner/ipath_planner.h`  
**Used by:** Process 4 (Mission Planner)

| Method | Signature | Description |
|--------|-----------|-------------|
| `plan` | `drone::ipc::TrajectoryCmd plan(const drone::ipc::Pose& pose, const Waypoint& target)` | Compute velocity command toward target |
| `name` | `std::string name() const` | Implementation name |

| Implementation | Description |
|----------------|-------------|
| `PotentialFieldPlanner` | Attractive force: normalized direction × min(speed, distance) |

**Factory:** `create_path_planner(backend)` — backends: `"potential_field"`

---

### `IObstacleAvoider` — `drone::planner`

**Header:** `process4_mission_planner/include/planner/iobstacle_avoider.h`  
**Used by:** Process 4 (Mission Planner)

| Method | Signature | Description |
|--------|-----------|-------------|
| `avoid` | `drone::ipc::TrajectoryCmd avoid(const drone::ipc::TrajectoryCmd& planned, const drone::ipc::Pose& pose, const drone::ipc::DetectedObjectList& objects)` | Modify trajectory to avoid obstacles |
| `name` | `std::string name() const` | Implementation name |

| Implementation | Parameters | Description |
|----------------|-----------|-------------|
| `PotentialFieldAvoider` | `influence_radius` (default 5.0m), `repulsive_gain` (default 2.0) | Inverse-square repulsive forces within influence radius |

**Factory:** `create_obstacle_avoider(backend, influence_radius, repulsive_gain)` — backends: `"potential_field"`

---

### `FaultManager` — `drone::planner`

**Header:** `process4_mission_planner/include/planner/fault_manager.h`  
**Used by:** Process 4 (Mission Planner)  
**Issue:** [#61](https://github.com/nmohamaya/companion_software_stack/issues/61)

Config-driven graceful degradation engine. Evaluates system health each loop tick and returns the highest-priority response action. Escalation-only policy — once raised, actions never downgrade within a flight.

| Type | Description |
|------|-------------|
| `FaultAction` | Enum: `NONE(0)` < `WARN(1)` < `LOITER(2)` < `RTL(3)` < `EMERGENCY_LAND(4)` |
| `FaultType` | Bitmask enum: 10 fault conditions (critical process, pose stale, battery low/critical/RTL, thermal warn/critical, perception dead, FC link lost, geofence breach) |
| `FaultState` | Return type: `recommended_action`, `active_faults` bitmask, `reason` string |
| `FaultConfig` | Thresholds: `pose_stale_timeout_ns`, `battery_warn_percent`, `battery_crit_percent`, `fc_link_lost_timeout_ns`, `loiter_escalation_timeout_ns` |

| Method | Signature | Description |
|--------|-----------|-------------|
| `evaluate` | `FaultState evaluate(const ShmSystemHealth&, const ShmFCState&, uint64_t pose_ts, uint64_t now_ns, uint32_t pose_quality = 2)` | Evaluate all fault conditions, return graduated action |
| `reset` | `void reset()` | Clear high-water mark (after landing / new mission) |
| `high_water_mark` | `FaultAction high_water_mark() const` | Highest action ever returned |
| `config` | `const FaultConfig& config() const` | Current config thresholds |

**Constructors:**
```cpp
FaultManager(const Config& cfg);        // Config-driven (reads fault_manager.* keys)
FaultManager(const FaultConfig& cfg);   // Direct config (unit tests)
```

**Config keys** (JSON `fault_manager` section):
| Key | Default | Description |
|-----|---------|-------------|
| `pose_stale_timeout_ms` | 500 | Max pose age before LOITER |
| `battery_warn_percent` | 20.0 | Battery % → RTL |
| `battery_crit_percent` | 10.0 | Battery % → EMERGENCY_LAND |
| `fc_link_lost_timeout_ms` | 3000 | FC disconnect duration → LOITER |
| `loiter_escalation_timeout_s` | 30 | Loiter duration before auto-RTL |

**`Pose.quality` field** (defined in `common/ipc/include/ipc/ipc_types.h`):

The `Pose` struct includes a `uint32_t quality` field that propagates VIO health from P3 to P4:

| Value | Meaning | Source |
|-------|---------|--------|
| 0 | Lost | VIOHealth::LOST (simulated backend) |
| 1 | Degraded | VIOHealth::INITIALIZING or DEGRADED (simulated backend) |
| 2 | Good | VIOHealth::NOMINAL (simulated backend) |
| 3 | Excellent | Ground truth (Gazebo backend) |

FaultManager uses this field (via the `pose_quality` parameter to `evaluate()`) to trigger VIO-related fault responses. See [slam_vio_nav_design.md](slam_vio_nav_design.md) for the VIO health integration details.

---

### `IProcessMonitor` — `drone::monitor`

**Header:** `process7_system_monitor/include/monitor/iprocess_monitor.h`  
**Used by:** Process 7 (System Monitor)

| Method | Signature | Description |
|--------|-----------|-------------|
| `collect` | `ShmSystemHealth collect()` | Collect CPU, memory, temperature, disk, battery metrics |
| `name` | `std::string name() const` | Implementation name |

| Implementation | Description |
|----------------|-------------|
| `LinuxProcessMonitor` | Reads `/proc`, `/sys` via `sys_info.h` helpers. Periodic disk checks to reduce `popen` overhead. |

**Constructor parameters (all optional):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cpu_warn` | 90.0 | CPU % → WARNING |
| `mem_warn` | 90.0 | Memory % → WARNING |
| `temp_warn` | 80.0 | Temperature °C → WARNING |
| `temp_crit` | 95.0 | Temperature °C → CRITICAL |
| `disk_crit` | 98.0 | Disk % → CRITICAL |
| `batt_warn` | 20.0 | Battery % → WARNING |
| `batt_crit` | 10.0 | Battery % → CRITICAL |
| `disk_interval` | 10 | Check disk every N calls |

**Factory:** `create_process_monitor(backend, ...)` — backends: `"linux"`

---

## 4. Watchdog & Hardening APIs

> **Design doc:** [hardening-design.md](hardening-design.md) — full architecture, deployment modes, thread criticality inventory  
> **ADR:** [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md) — decision record and alternatives

### `ThreadHeartbeatRegistry` — `drone::util`

**Header:** `common/util/include/util/thread_heartbeat.h`

Process-global singleton for per-thread heartbeat tracking. Each thread registers a slot and touches it every loop iteration (~1 ns cost via atomic store + `std::chrono::steady_clock::now()`).

| Method | Signature | Description |
|--------|-----------|-------------|
| `instance` | `static ThreadHeartbeatRegistry& instance()` | Singleton access |
| `register_thread` | `size_t register_thread(const char* name, bool critical = false)` | Claim next slot via CAS loop; returns handle or `kInvalidHandle` |
| `touch` | `void touch(size_t handle)` | Update `last_touch_ns` atomically |
| `touch_with_grace` | `void touch_with_grace(size_t handle, milliseconds grace)` | Touch with future timestamp (covers long ops like model loading) |
| `snapshot` | `[[nodiscard]] ThreadSnapshot snapshot() const` | Stack-allocated copy of all registered heartbeats |
| `count` | `[[nodiscard]] size_t count() const` | Number of registered beats |

**Constants:** `kMaxThreads = 16`, `kInvalidHandle` (sentinel).

### `ScopedHeartbeat` — `drone::util`

**Header:** `common/util/include/util/thread_heartbeat.h`

RAII wrapper — registers on construction, provides `touch()` in the loop body.

```cpp
ScopedHeartbeat hb("inference", /*critical=*/true);
hb.touch_with_grace(std::chrono::seconds{30});  // model loading
load_model();
while (running_) {
    hb.touch();
    // ... work ...
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `touch` | `void touch()` | Forward to registry |
| `touch_with_grace` | `void touch_with_grace(milliseconds grace)` | Forward with grace |
| `is_valid` | `[[nodiscard]] bool is_valid() const` | True if registration succeeded |
| `handle` | `[[nodiscard]] size_t handle() const` | The registered slot handle |

### `ThreadWatchdog` — `drone::util`

**Header:** `common/util/include/util/thread_watchdog.h`

Background scanner thread that detects stuck threads via heartbeat age comparison.

| Method | Signature | Description |
|--------|-----------|-------------|
| constructor | `ThreadWatchdog(Config cfg = {})` | Start scan thread with config |
| `set_stuck_callback` | `void set_stuck_callback(StuckCallback cb)` | Called on healthy→stuck transition |
| `get_stuck_threads` | `[[nodiscard]] vector<string> get_stuck_threads() const` | Names of currently stuck threads |

**Config:**

| Field | Default | Description |
|-------|---------|-------------|
| `stuck_threshold` | 5000 ms | Duration without touch before "stuck" |
| `scan_interval` | 1000 ms | Polling period |

### `ThreadHealthPublisher<Publisher>` — `drone::util`

**Header:** `common/util/include/util/thread_health_publisher.h`

Template that bridges heartbeat registry + watchdog → `ShmThreadHealth` → `IPublisher`.

| Method | Signature | Description |
|--------|-----------|-------------|
| constructor | `ThreadHealthPublisher(Publisher& pub, const char* process, const ThreadWatchdog& wd)` | Bind to publisher and watchdog |
| `publish_snapshot` | `void publish_snapshot()` | Take snapshot, cross-reference stuck list, publish health |

### `RestartPolicy` — `drone::util`

**Header:** `common/util/include/util/restart_policy.h`

Per-process restart rules with exponential backoff and thermal gating.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `max_restarts` | `uint32_t` | 5 | Max restarts before FAILED |
| `cooldown_window_s` | `uint32_t` | 60 | Seconds of stability to reset counter |
| `initial_backoff_ms` | `uint32_t` | 500 | First retry delay |
| `max_backoff_ms` | `uint32_t` | 30000 | Backoff cap (30 s) |
| `is_critical` | `bool` | false | Stack → CRITICAL on exhaustion |
| `thermal_gate` | `uint8_t` | 3 | Block restarts at this thermal zone |

| Method | Signature | Description |
|--------|-----------|-------------|
| `backoff_ms` | `[[nodiscard]] uint32_t backoff_ms(uint32_t attempt) const` | `min(initial × 2^attempt, max)` |
| `is_thermal_blocked` | `[[nodiscard]] bool is_thermal_blocked(uint8_t zone) const` | True if zone ≥ gate |

**`StackStatus`** enum: `NOMINAL`, `DEGRADED`, `CRITICAL`

### `ProcessConfig` — `drone::util`

**Header:** `common/util/include/util/restart_policy.h`

Loads per-process config from JSON (name, binary, policy, launch_after, restart_cascade).

| Method | Signature | Description |
|--------|-----------|-------------|
| `from_json` | `static ProcessConfig from_json(const string& name, const json& j)` | Parse JSON object into config |

### `ProcessGraph` — `drone::util`

**Header:** `common/util/include/util/process_graph.h`

Dual-edge dependency graph: `launch_after` (data dependency) + `restart_cascade` (stale-state dependency).

| Method | Signature | Description |
|--------|-----------|-------------|
| `add_process` | `void add_process(const string& name)` | Register a process node |
| `add_launch_dep` | `void add_launch_dep(const string& child, const string& parent)` | Child waits for parent before launch |
| `add_cascade` | `void add_cascade(const string& source, const string& target)` | If source dies, stop+restart target |
| `launch_order` | `[[nodiscard]] vector<string> launch_order() const` | Topological sort (Kahn's algorithm) |
| `cascade_targets` | `[[nodiscard]] vector<string> cascade_targets(const string& proc) const` | BFS transitive cascade set |
| `validate` | `[[nodiscard]] bool validate() const` | Cycle + dangling reference check |
| `populate_defaults` | `void populate_defaults()` | Standard 6-process graph |

### `ProcessManager` — `drone::monitor`

**Header:** `process7_system_monitor/include/monitor/process_manager.h`

Fork+exec process supervisor with automatic restart, cascade stops, and thermal gating.

| Method | Signature | Description |
|--------|-----------|-------------|
| `add_process` | `void add_process(name, binary, args, policy)` | Register a managed child |
| `launch_all` | `void launch_all()` | Fork+exec all processes in graph order |
| `launch` | `bool launch(const char* name)` | Launch a single process |
| `stop` | `bool stop(const char* name, milliseconds timeout)` | SIGTERM → wait → SIGKILL |
| `stop_all` | `void stop_all(milliseconds timeout)` | Stop all in reverse order |
| `tick` | `void tick()` | Reap zombies, detect deaths, trigger restarts (call at ~1 Hz) |
| `set_process_graph` | `void set_process_graph(const ProcessGraph* graph)` | Wire cascade dependencies |
| `set_thermal_zone` | `void set_thermal_zone(uint8_t zone)` | Update thermal gate input |
| `compute_stack_status` | `[[nodiscard]] StackStatus compute_stack_status() const` | NOMINAL / DEGRADED / CRITICAL |
| `total_restarts` | `[[nodiscard]] uint32_t total_restarts() const` | Sum across all processes |

### `sd_notify` wrappers — `drone::systemd`

**Header:** `common/util/include/util/sd_notify.h`

Thin inline wrappers around `sd_notify()`. All are no-ops when built without `-DENABLE_SYSTEMD=ON`.

| Function | Description |
|----------|-------------|
| `notify_ready()` | Sends `READY=1` (Type=notify handshake) |
| `notify_watchdog()` | Sends `WATCHDOG=1` (pet the WatchdogSec timer) |
| `notify_stopping()` | Sends `STOPPING=1` (graceful shutdown) |
| `notify_status(msg)` | Sends `STATUS=msg` (visible in `systemctl status`) |
| `watchdog_enabled()` | Returns true if `WatchdogSec` is active |
| `watchdog_usec()` | Returns watchdog interval in µs (0 if inactive) |

### `Result<T,E>` — `drone::util`

**Header:** `common/util/include/util/result.h`

Monadic error type replacing exceptions on the hot path. See [CPP_PATTERNS_GUIDE.md §3.13](CPP_PATTERNS_GUIDE.md#313-resultte--monadic-error-handling) for usage patterns.

| Method | Signature | Description |
|--------|-----------|-------------|
| `ok` | `static Result ok(T value)` | Success constructor |
| `err` | `static Result err(E error)` | Error constructor |
| `is_ok` / `is_err` | `bool` | Status check |
| `value` / `error` | `const T&` / `const E&` | Accessors (UB if wrong state) |
| `value_or` | `T value_or(T fallback) const` | Safe fallback |
| `map` | `auto map(F&& f) const` | Transform T → U on success |
| `and_then` | `auto and_then(F&& f) const` | Chain T → Result\<U,E\> |
| `map_error` | `auto map_error(F&& f) const` | Transform E → E2 on error |

**Error codes:** `Unknown`, `FileNotFound`, `ParseError`, `InvalidValue`, `MissingKey`, `TypeMismatch`, `OutOfRange`, `Timeout`, `NotConnected`, `AlreadyExists`

### `safe_name_copy<N>` — `drone::util`

**Header:** `common/util/include/util/safe_name_copy.h`

Template replacing raw `strncpy` for fixed-size name buffers. Deduces buffer size, zeroes, copies, null-terminates.

```cpp
char buf[32];
safe_name_copy(buf, "inference_thread");
```

---

## 5. How Processes Use These Interfaces

Every process creates a message bus via the config-driven factory and obtains typed publishers/subscribers:

```cpp
// Example: Process 1 (Video Capture)
auto bus = create_message_bus(cfg);  // Returns MessageBus wrapping ZenohMessageBus
auto mission_pub = bus.advertise<VideoFrame>("drone/video/frame");
auto stereo_pub  = bus.advertise<StereoFrame>("drone/video/stereo_frame");

// Thread function takes IPublisher<T>& — testable with mocks
void capture_thread(IPublisher<VideoFrame>& pub) {
    VideoFrame frame = capture();
    pub.publish(frame);
}
```

### IPC Backend Selection

```json
// config/default.json
{
    "ipc_backend": "zenoh"   // Zenoh is the sole backend (SHM removed in Issue #126)
}
```

```cpp
// Shared factory helper (in a common header)
auto create_message_bus(const drone::Config& cfg) {
    auto backend = cfg.get<std::string>("ipc_backend", "zenoh");
    if (backend != "zenoh") {
        spdlog::error("Unknown ipc_backend '{}', falling back to zenoh", backend);
    }
    return std::make_unique<drone::ipc::ZenohMessageBus>();
}
```

Strategy interfaces are created via factories, typically from config:

```cpp
auto planner = create_path_planner(cfg.get("planner.backend", "potential_field"));
auto avoider = create_obstacle_avoider(
    cfg.get("avoider.backend", "potential_field"),
    cfg.get("avoider.influence_radius", 5.0f),
    cfg.get("avoider.repulsive_gain",   2.0f));
```

---

## 6. Adding a New Implementation

1. Create a class inheriting the abstract interface (e.g., `class OrbSlam3Frontend : public IVisualFrontend`)
2. Implement all pure virtual methods
3. Add a new branch in the factory function
4. Set the backend name in config

No process code changes needed — only the factory + new implementation file.

---

## 7. Test Coverage

For the full test index (test files, suite names, counts, and how to run them), see **[tests/TESTS.md](../tests/TESTS.md)** — the single source of truth for test documentation.

---

## 8. Zenoh IPC Migration

> **Epic:** [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) — Zenoh IPC Migration  
> **Decision:** [ADR-001](adr/ADR-001-ipc-framework-selection.md)  
> **Target Hardware:** NVIDIA Jetson Orin (aarch64, JetPack 6.x)

The IPC layer was migrated from POSIX SHM (SeqLock) to **Eclipse Zenoh** in 6 phases (all complete — Epic #45 closed, legacy SHM fully removed in Issue #126):

| Phase | Issue | Title | Status | Key Changes |
|-------|-------|-------|--------|-------------|
| **A** | [#46](https://github.com/nmohamaya/companion_software_stack/issues/46) | Foundation | **Done** (PR #52) | CMake `find_package(zenohc)`, `HAVE_ZENOH` guard, `ZenohMessageBus`, `MessageBusFactory`, CI dual-build, 30 tests |
| **B** | [#47](https://github.com/nmohamaya/companion_software_stack/issues/47) | Low-bandwidth migration | **Done** (PR #53) | All 7 processes → `MessageBusFactory`, `bus_subscribe_optional<T>()`, 13 new tests |
| **C** | [#48](https://github.com/nmohamaya/companion_software_stack/issues/48) | High-bandwidth migration | **Done** (PR #54) | Video frames → Zenoh SHM provider (zero-copy), `PosixShmProvider` |
| **D** | [#49](https://github.com/nmohamaya/companion_software_stack/issues/49) | Service channels | **Done** (PR #55) | Zenoh queryable services, legacy `ShmServiceChannel` removed |
| **E** | [#50](https://github.com/nmohamaya/companion_software_stack/issues/50) | Network transport | **Done** (PR #56) | Drone↔GCS network transport, wire format, GCS client tool |
| **F** | [#51](https://github.com/nmohamaya/companion_software_stack/issues/51) | Liveliness tokens | **Done** (PR #57) | Process health via liveliness tokens, P7 death detection |

### Why Zenoh?

- **Zero-copy SHM** — eliminates 6 MB `memcpy` per video frame (~180 MB/s saved)
- **Network transparency** — same API works locally (SHM) and to GCS (UDP/TCP)
- **Liveliness tokens** — automatic process death detection (no custom heartbeat)
- **Wildcard subscriptions** — `drone/**` captures all channels for flight data recording (#40)
- **Pre-built aarch64 packages** — no Rust toolchain needed on Jetson Orin

### Zenoh Types — Status

| Type | Header | Implements | Status | Description |
|------|--------|------------|--------|-------------|
| `ZenohSession` | `zenoh_session.h` | — | **Implemented** | Singleton Zenoh session per process (intentionally leaked — see BUG_FIXES.md #7) |
| `ZenohPublisher<T>` | `zenoh_publisher.h` | `IPublisher<T>` | **Implemented** | Serializes `T` to `Bytes` via `vector<uint8_t>`, calls `Publisher::put(Bytes&&)` |
| `ZenohSubscriber<T>` | `zenoh_subscriber.h` | `ISubscriber<T>` | **Implemented** | Callback-based (`Subscriber<void>`), latest-value cache with mutex + atomics |
| `ZenohMessageBus` | `zenoh_message_bus.h` | — | **Implemented** | Factory with 12-channel Zenoh topic mapping |
| `MessageBusFactory` | `message_bus_factory.h` | — | **Implemented** | Config-driven `variant`-based backend selection |
| `ZenohServiceClient<Req,Resp>` | `zenoh_service_client.h` | `IServiceClient<Req,Resp>` | Planned ([#49](https://github.com/nmohamaya/companion_software_stack/issues/49)) | Zenoh query-based service client |
| `ZenohServiceServer<Req,Resp>` | `zenoh_service_server.h` | `IServiceServer<Req,Resp>` | Planned ([#49](https://github.com/nmohamaya/companion_software_stack/issues/49)) | Zenoh queryable-based service server |
| `LivelinessToken` | `zenoh_liveliness.h` | — | Planned ([#51](https://github.com/nmohamaya/companion_software_stack/issues/51)) | Per-process health token |
| `LivelinessMonitor` | `zenoh_liveliness.h` | — | Planned ([#51](https://github.com/nmohamaya/companion_software_stack/issues/51)) | Watches `drone/alive/**` for process death |

### Process Code Impact

Process code changes are **minimal** — only the bus creation line changes:

```cpp
// BEFORE:
drone::ipc::ShmMessageBus bus;
auto sub = bus.subscribe<drone::ipc::ShmPose>("/drone_slam_pose");

// AFTER:
auto bus = create_message_bus(cfg);  // ZenohMessageBus if ipc_backend=zenoh
auto sub = bus.subscribe<drone::ipc::Pose>("drone/slam/pose");
```

All `IPublisher<T>` and `ISubscriber<T>` usage remains identical.
