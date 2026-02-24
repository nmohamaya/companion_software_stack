# Interface API Reference

> **Issue:** #4 — API-Driven Development  
> **Branch:** `feature/issue-4-api-interfaces`  
> **Status:** All 7 processes wired through abstract interfaces; 163 tests pass

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│                      ShmMessageBus                           │
│    advertise<T>(topic)  →  IPublisher<T>                     │
│    subscribe<T>(topic)  →  ISubscriber<T>                    │
└─────────┬───────────────────────────┬────────────────────────┘
          │  SHM-backed (SeqLock)     │
     ShmPublisher<T>           ShmSubscriber<T>
          │                          │
┌─────────▼──────────────────────────▼────────────────────────┐
│                   Process Internals                          │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐ │
│  │IVisualFrontend│ │ IPathPlanner │ │ IObstacleAvoider     │ │
│  │ (P3: SLAM)   │ │ (P4: Mission)│ │ (P4: Mission)        │ │
│  └──────────────┘ └──────────────┘ └──────────────────────┘ │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ IProcessMonitor  (P7: System Monitor)                │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

All interfaces follow the **Strategy pattern**: abstract base class + concrete implementation + factory function. Swap implementations at construction time via config string.

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

### `ShmMessageBus` — Factory

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

Every process creates a `ShmMessageBus` and obtains typed publishers/subscribers:

```cpp
// Example: Process 1 (Video Capture)
ShmMessageBus bus;
auto mission_pub = bus.advertise<ShmVideoFrame>(shm_names::VIDEO_MISSION_CAM);
auto stereo_pub  = bus.advertise<ShmStereoFrame>(shm_names::VIDEO_STEREO_CAM);

// Thread function takes IPublisher<T>& — testable with mocks
void capture_thread(IPublisher<ShmVideoFrame>& pub) {
    ShmVideoFrame frame = capture();
    pub.publish(frame);
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

Total: **42 new tests** (163 total with existing 121).
