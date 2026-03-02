# Interface API Reference

> **Issue:** #4 — API-Driven Development  
> **Status:** All 7 processes wired through abstract interfaces; 308 tests pass (including 44 Zenoh IPC tests)  
> **IPC Migration:** Zenoh Phase A+B done — [ADR-001](adr/ADR-001-ipc-framework-selection.md), Epic [#45](https://github.com/nmohamaya/companion_software_stack/issues/45), PRs [#52](https://github.com/nmohamaya/companion_software_stack/pull/52) + [#53](https://github.com/nmohamaya/companion_software_stack/pull/53)

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│              MessageBus (config-driven backend)                   │
│    advertise<T>(topic)  →  IPublisher<T>                         │
│    subscribe<T>(topic)  →  ISubscriber<T>                        │
├───────────────────────────┬──────────────────────────────────────┤
│  Backend: "shm" (default)  │  Backend: "zenoh" (Phase A done #46) │
│  ShmPublisher<T>          │  ZenohPublisher<T>                   │
│  ShmSubscriber<T>         │  ZenohSubscriber<T>                  │
│  SeqLock POSIX SHM        │  Zenoh SHM (zero-copy) + network     │
│  Local-only               │  SHM local + UDP/TCP to GCS          │
└───────────────────────────┴──────────────────────────────────────┘
          │                          │
┌─────────▼──────────────────────────▼────────────────────────────┐
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

The IPC backend is selected via `"ipc_backend"` in config (`"shm"` or `"zenoh"`). Process code depends only on `IPublisher<T>` / `ISubscriber<T>` — transport is invisible. See [ADR-001](adr/ADR-001-ipc-framework-selection.md) for the decision record.

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

### `ShmPublisher<T>` — Concrete

**Header:** `common/ipc/include/ipc/shm_publisher.h`

Wraps `ShmWriter<T>` (SeqLock-based POSIX shared memory).

```cpp
ShmPublisher<ShmVideoFrame> pub("/drone_mission_cam");
pub.publish(frame);
```

### `ShmSubscriber<T>` — Concrete

**Header:** `common/ipc/include/ipc/shm_subscriber.h`

Wraps `ShmReader<T>` with retry-based connection.

```cpp
// Eager connection (with retry)
ShmSubscriber<ShmVideoFrame> sub("/drone_mission_cam", /*retries=*/50, /*ms=*/200);

// Lazy connection
ShmSubscriber<ShmVideoFrame> sub;
sub.connect("/drone_mission_cam");
```

### `ShmMessageBus` — Factory (Current Backend)

**Header:** `common/ipc/include/ipc/shm_message_bus.h`

Lightweight factory — the primary entry point for creating pub/sub pairs.

| Method | Signature | Description |
|--------|-----------|-------------|
| `advertise` | `unique_ptr<IPublisher<T>> advertise<T>(topic)` | Create a publisher |
| `subscribe` | `unique_ptr<ISubscriber<T>> subscribe<T>(topic, retries, ms)` | Create a subscriber with retry |
| `subscribe_lazy` | `unique_ptr<ShmSubscriber<T>> subscribe_lazy<T>()` | Create unconnected subscriber |

```cpp
ShmMessageBus bus;
auto pub = bus.advertise<ShmVideoFrame>("/drone_mission_cam");
auto sub = bus.subscribe<ShmVideoFrame>("/drone_mission_cam");

pub->publish(frame);
ShmVideoFrame out;
sub->receive(out);
```

### `ZenohSession` — Singleton Session Manager

**Header:** `common/ipc/include/ipc/zenoh_session.h`  
**Compile guard:** `#ifdef HAVE_ZENOH`

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
**Compile guard:** `#ifdef HAVE_ZENOH`

Zenoh-backed publisher implementing `IPublisher<T>`. Serializes trivially-copyable `T` to `zenoh::Bytes` (via `std::vector<uint8_t>`) and calls `Publisher::put(Bytes&&)`.

```cpp
ZenohPublisher<ShmPose> pub("drone/slam/pose");
pub.publish(pose);  // Serialized as raw bytes → Zenoh network
```

**Why a wrapper?** Simplifies the raw zenoh-cpp API (explicit `Bytes` construction, rvalue semantics) down to a single `publish(const T&)` call. Enforces `static_assert(is_trivially_copyable_v<T>)` for type safety.

### `ZenohSubscriber<T>` — Concrete

**Header:** `common/ipc/include/ipc/zenoh_subscriber.h`  
**Compile guard:** `#ifdef HAVE_ZENOH`

Zenoh-backed subscriber implementing `ISubscriber<T>`. Maintains a latest-value cache updated by the Zenoh callback thread. Uses `std::mutex` + `std::atomic` for thread-safe access between the Zenoh callback thread and the process main loop.

```cpp
ZenohSubscriber<ShmPose> sub("drone/slam/pose");
ShmPose pose;
if (sub.receive(pose)) { /* use pose */ }
```

**Why a wrapper?** The raw zenoh-cpp `Subscriber<void>` requires two callbacks (`on_sample` + `on_drop`), manual `Bytes::as_vector()` deserialization, and size validation. The wrapper handles all of this and exposes the same `receive(T&)` API as the SHM backend.

### `ZenohMessageBus` — Factory ([#46](https://github.com/nmohamaya/companion_software_stack/issues/46))

**Header:** `common/ipc/include/ipc/zenoh_message_bus.h`  
**Compile guard:** `#ifdef HAVE_ZENOH`

Drop-in replacement for `ShmMessageBus`. Same `advertise<T>()` / `subscribe<T>()` API, backed by Eclipse Zenoh. Automatically maps SHM segment names to Zenoh key expressions.

| Method | Signature | Description |
|--------|-----------|-------------|
| `advertise` | `unique_ptr<IPublisher<T>> advertise<T>(topic)` | Create a `ZenohPublisher<T>` |
| `subscribe` | `unique_ptr<ISubscriber<T>> subscribe<T>(topic)` | Create a `ZenohSubscriber<T>` |
| `subscribe_lazy` | `unique_ptr<ZenohSubscriber<T>> subscribe_lazy<T>(topic)` | Equivalent to `subscribe()` — Zenoh connections are always async |
| `to_key_expr` | `static std::string to_key_expr(const std::string& shm_name)` | Map SHM name → Zenoh key expression (guards empty input) |

**Why a wrapper?** Maps the existing 12 SHM segment names to Zenoh hierarchical key expressions, so process code can use either naming convention. Provides the same factory interface as `ShmMessageBus` for seamless backend swapping.

**Key-expression mapping** (SHM segment names → Zenoh topics):

| SHM Name | Zenoh Key Expression |
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

Returns the appropriate message bus backend based on config. Uses `std::variant<unique_ptr<ShmMessageBus>, unique_ptr<ZenohMessageBus>>` for type-safe backend storage.

| Function | Signature | Description |
|----------|-----------|-------------|
| `create_message_bus` | `MessageBusVariant create_message_bus(const std::string& backend = "shm")` | Create bus by backend name |
| `bus_advertise<T>` | `unique_ptr<IPublisher<T>> bus_advertise<T>(MessageBusVariant&, topic)` | Advertise via variant |
| `bus_subscribe<T>` | `unique_ptr<ISubscriber<T>> bus_subscribe<T>(MessageBusVariant&, topic, ...)` | Subscribe via variant |
| `bus_subscribe_optional<T>` | `unique_ptr<ISubscriber<T>> bus_subscribe_optional<T>(MessageBusVariant&, topic)` | Subscribe with no retries (single attempt). For SHM: `is_connected()` returns false if segment doesn't exist. For Zenoh: `is_connected()` always true (async discovery). |

**Why a wrapper?** Enables runtime backend selection from config (`"ipc_backend": "shm"` or `"zenoh"`). Process code uses `bus_advertise<T>()` / `bus_subscribe<T>()` — completely agnostic to the transport.

```cpp
// Usage is identical to ShmMessageBus — processes don't know the backend:
auto bus = create_message_bus(cfg);  // Returns ZenohMessageBus if ipc_backend="zenoh"
auto pub = bus->advertise<ShmPose>("drone/slam/pose");
auto sub = bus->subscribe<ShmPose>("drone/slam/pose");

pub->publish(pose);
ShmPose out;
sub->receive(out);
```

**Zenoh-specific features (not available with SHM backend):**
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

### `ShmServiceClient` / `ShmServiceServer` — Concrete

**Header:** `common/ipc/include/ipc/shm_service_channel.h`

Uses two SHM segments (request + response). Both sides pre-create missing segments via `ensure_shm_exists()` so construction order is irrelevant.

```cpp
// Server
ShmServiceServer<ReqType, RespType> server("/svc_req", "/svc_resp");
if (auto req = server.poll_request()) {
    RespType resp = process(req->payload);
    server.send_response(req->correlation_id, ServiceStatus::OK, resp);
}

// Client
ShmServiceClient<ReqType, RespType> client("/svc_req", "/svc_resp");
auto id = client.send_request(my_request);
auto resp = client.await_response(id, 500ms);
if (resp && resp->status == ServiceStatus::OK) { /* handle */ }
```

---

## 3. Process Strategy Interfaces

### `IVisualFrontend` — `drone::slam`

**Header:** `process3_slam_vio_nav/include/slam/ivisual_frontend.h`  
**Used by:** Process 3 (SLAM/VIO/Nav)

| Method | Signature | Description |
|--------|-----------|-------------|
| `process_frame` | `Pose process_frame(const ShmStereoFrame& frame)` | Process stereo frame, return 6-DOF pose |
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
| `plan` | `ShmTrajectoryCmd plan(const ShmPose& pose, const Waypoint& target)` | Compute velocity command toward target |
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
| `avoid` | `ShmTrajectoryCmd avoid(const ShmTrajectoryCmd& planned, const ShmPose& pose, const ShmDetectedObjectList& objects)` | Modify trajectory to avoid obstacles |
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
| `FaultType` | Bitmask enum: 8 fault conditions (critical process, pose stale, battery low/critical, thermal warn/critical, perception dead, FC link lost) |
| `FaultState` | Return type: `recommended_action`, `active_faults` bitmask, `reason` string |
| `FaultConfig` | Thresholds: `pose_stale_timeout_ns`, `battery_warn_percent`, `battery_crit_percent`, `fc_link_lost_timeout_ns`, `loiter_escalation_timeout_ns` |

| Method | Signature | Description |
|--------|-----------|-------------|
| `evaluate` | `FaultState evaluate(const ShmSystemHealth&, const ShmFCState&, uint64_t pose_ts, uint64_t now_ns)` | Evaluate all fault conditions, return graduated action |
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

## 4. How Processes Use These Interfaces

Every process creates a message bus via the config-driven factory and obtains typed publishers/subscribers:

```cpp
// Example: Process 1 (Video Capture)
auto bus = create_message_bus(cfg);  // ShmMessageBus or ZenohMessageBus
auto mission_pub = bus->advertise<ShmVideoFrame>("drone/video/frame");
auto stereo_pub  = bus->advertise<ShmStereoFrame>("drone/video/stereo_frame");

// Thread function takes IPublisher<T>& — testable with mocks
void capture_thread(IPublisher<ShmVideoFrame>& pub) {
    ShmVideoFrame frame = capture();
    pub.publish(frame);
}
```

### IPC Backend Selection

```json
// config/default.json
{
    "ipc_backend": "shm"     // Current default — POSIX SHM (SeqLock)
    // "ipc_backend": "zenoh" // Phase A implemented — zero-copy SHM + network transport
}
```

```cpp
// Shared factory helper (in a common header)
auto create_message_bus(const drone::Config& cfg) {
    auto backend = cfg.get<std::string>("ipc_backend", "shm");
#ifdef HAVE_ZENOH
    if (backend == "zenoh") return std::make_unique<drone::ipc::ZenohMessageBus>();
#endif
    return std::make_unique<drone::ipc::ShmMessageBus>();
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

## 5. Adding a New Implementation

1. Create a class inheriting the abstract interface (e.g., `class OrbSlam3Frontend : public IVisualFrontend`)
2. Implement all pure virtual methods
3. Add a new branch in the factory function
4. Set the backend name in config

No process code changes needed — only the factory + new implementation file.

---

## 6. Test Coverage

| Test File | Tests | What's Covered |
|-----------|-------|----------------|
| `test_message_bus.cpp` | 23 | IPublisher, ISubscriber, ShmMessageBus, ShmServiceChannel |
| `test_process_interfaces.cpp` | 19 | IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor |
| `test_shm_ipc.cpp` | 25 | SeqLock ShmWriter/ShmReader, SPSC ring |
| `test_zenoh_ipc.cpp` | 44 | MessageBusFactory (5), ZenohTopicMapping (15), ZenohSession/Publisher/Subscriber (3), ZenohPubSub round-trips (5), ZenohMessageBus (4), ZenohMigration (13: 10 per-channel round-trips, multi-channel, high-rate pose, factory subscribe_optional) |
| `test_mission_fsm.cpp` | 7 | MissionFSM state transitions, waypoint loading, state names |
| `test_fault_manager.cpp` | 23 | FaultManager: nominal, 8 fault conditions, escalation-only, loiter timeout, reset, simultaneous faults, edge cases ([#61](https://github.com/nmohamaya/companion_software_stack/issues/61)) |
| `test_zenoh_service.cpp` | *(planned)* | ZenohServiceClient, ZenohServiceServer ([#49](https://github.com/nmohamaya/companion_software_stack/issues/49)) |
| `test_zenoh_liveliness.cpp` | *(planned)* | LivelinessToken, LivelinessMonitor ([#51](https://github.com/nmohamaya/companion_software_stack/issues/51)) |
| `bench_zenoh_video.cpp` | *(planned)* | Video frame zero-copy benchmarks ([#48](https://github.com/nmohamaya/companion_software_stack/issues/48)) |

Total: **400 tests** (23 suites).

---

## 7. Planned: Zenoh IPC Migration

> **Epic:** [#45](https://github.com/nmohamaya/companion_software_stack/issues/45) — Zenoh IPC Migration  
> **Decision:** [ADR-001](adr/ADR-001-ipc-framework-selection.md)  
> **Target Hardware:** NVIDIA Jetson Orin (aarch64, JetPack 6.x)

The IPC layer will be migrated from POSIX SHM (SeqLock) to **Eclipse Zenoh** in 6 phases:

| Phase | Issue | Title | Status | Key Changes |
|-------|-------|-------|--------|-------------|
| **A** | [#46](https://github.com/nmohamaya/companion_software_stack/issues/46) | Foundation | **Done** (PR #52) | CMake `find_package(zenohc)`, `HAVE_ZENOH` guard, `ZenohMessageBus`, `MessageBusFactory`, CI dual-build, 30 tests |
| **B** | [#47](https://github.com/nmohamaya/companion_software_stack/issues/47) | Low-bandwidth migration | **In Review** (PR #53) | All 7 processes → `MessageBusFactory`, `bus_subscribe_optional<T>()`, 13 new tests, [ipc-key-expressions.md](ipc-key-expressions.md) |
| **C** | [#48](https://github.com/nmohamaya/companion_software_stack/issues/48) | High-bandwidth migration | Planned | `ShmVideoFrame`/`ShmStereoFrame` → Zenoh SHM provider (zero-copy) |
| **D** | [#49](https://github.com/nmohamaya/companion_software_stack/issues/49) | Service channels | Planned | `ShmServiceChannel` → Zenoh queryable; delete legacy SHM files |
| **E** | [#50](https://github.com/nmohamaya/companion_software_stack/issues/50) | Network transport | Planned | Same pub/sub reachable from GCS over UDP/TCP — enables #34, #35 |
| **F** | [#51](https://github.com/nmohamaya/companion_software_stack/issues/51) | Liveliness tokens | Planned | Process death detection — enables #28, #41 |

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
| `ZenohMessageBus` | `zenoh_message_bus.h` | — | **Implemented** | Factory with 12-segment SHM→Zenoh topic mapping |
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
auto sub = bus->subscribe<drone::ipc::ShmPose>("drone/slam/pose");
```

All `IPublisher<T>` and `ISubscriber<T>` usage remains identical.
