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
│  │ IVIOBackend  │ │ IPathPlanner │ │ IObstacleAvoider         │ │
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

**Why a wrapper?** Maps channel names to Zenoh hierarchical key expressions. Provides a consistent `advertise<T>()` / `subscribe<T>()` factory interface so process code is agnostic to the transport.

**Key-expression mapping** (channel names → Zenoh topics):

| Channel Name | Zenoh Key Expression |
|----------|---------------------|
| `/drone_mission_cam` | `drone/video/frame` |
| `/drone_stereo_cam` | `drone/video/stereo_frame` |
| `/detected_objects` | `drone/perception/detections` |
| `/semantic_voxels` | `drone/perception/voxels` |
| `/slam_pose` | `drone/slam/pose` |
| `/mission_status` | `drone/mission/status` |
| `/trajectory_cmd` | `drone/mission/trajectory` |
| `/payload_commands` | `drone/mission/payload_command` |
| `/fc_commands` | `drone/comms/fc_command` |
| `/fc_state` | `drone/comms/fc_state` |
| `/gcs_commands` | `drone/comms/gcs_command` |
| `/mission_upload` | `drone/mission/upload` |
| `/payload_status` | `drone/payload/status` |
| `/system_health` | `drone/monitor/health` |
| `/fault_overrides` | `fault/overrides` * |
| `/radar_detections` | `radar/detections` * |
| `/drone_thread_health_video_capture` | `drone/thread/health/video/capture` * |
| `/drone_thread_health_perception` | `drone/thread/health/perception` * |
| `/drone_thread_health_slam_vio_nav` | `drone/thread/health/slam/vio/nav` * |
| `/drone_thread_health_mission_planner` | `drone/thread/health/mission/planner` * |
| `/drone_thread_health_comms` | `drone/thread/health/comms` * |
| `/drone_thread_health_payload_manager` | `drone/thread/health/payload/manager` * |
| `/drone_thread_health_system_monitor` | `drone/thread/health/system/monitor` * |

\* Not in the explicit mapping table — resolved via the fallback rule (strip leading `/`, replace `_` with `/`).

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

### `ISerializer<T>` — `drone::ipc` — Epic #284, Issue #294

> **File:** `common/ipc/include/ipc/iserializer.h`

Abstract serialization interface decoupling wire format from transport.

| Method | Returns | Description |
|--------|---------|-------------|
| `serialize(msg, buf, buf_size)` | `size_t` | Serialize into pre-allocated buffer (SHM path) |
| `serialize(msg)` | `vector<uint8_t>` | Serialize into new vector (bytes path) |
| `deserialize(data, size, out)` | `bool` | Deserialize from raw bytes |
| `serialized_size(msg)` | `size_t` | Query serialized size |
| `name()` | `string_view` | Human-readable name (e.g. "raw") |

**Implementation:** `RawSerializer<T>` (`raw_serializer.h`) — byte-identical memcpy with `static_assert(is_trivially_copyable_v<T>)`.

### `TopicResolver` — `drone::ipc` — Epic #284, Issue #289

> **File:** `common/ipc/include/ipc/topic_resolver.h`

Multi-vehicle topic namespacing. Prepends `/<vehicle_id>` to topic names.

| Method | Returns | Description |
|--------|---------|-------------|
| `TopicResolver(vehicle_id)` | — | Constructor; empty = no prefix (backward compat) |
| `resolve(base_topic)` | `string` | Namespace a topic: `"/slam_pose"` → `"/drone42/slam_pose"` |
| `vehicle_id()` | `const string&` | Configured vehicle ID |
| `has_prefix()` | `bool` | True if vehicle_id is non-empty |

Validation: `[a-zA-Z0-9_-]` only. Throws `invalid_argument` at construction.

---

## 1.5 Intra-Process Infrastructure — Epic #284

### `EventBus<Event>` — `drone::util` — Issue #293

> **File:** `common/util/include/util/event_bus.h`

Lightweight typed intra-process pub/sub with RAII subscriptions.

| Method | Returns | Description |
|--------|---------|-------------|
| `subscribe(handler)` | `Subscription<Event>` | Subscribe; returns RAII token |
| `publish(event)` | `void` | Notify all subscribers (snapshot-copy, re-entrant safe) |
| `subscriber_count()` | `size_t` | Active subscription count |

Thread-safe: subscribe/publish/unsubscribe from any thread. Lifetime: bus must outlive all subscriptions.

### `PluginLoader` — `drone::util` — Epic #284, Issue #295

> **File:** `common/util/include/util/plugin_loader.h`
> **Gate:** `#ifdef HAVE_PLUGINS`

Runtime .so loading via dlopen with RAII lifetime management.

| Class | Purpose |
|-------|---------|
| `PluginLoader` | Static factory: `load<Interface>(so_path, factory_symbol)` → `Result<PluginHandle<I>>` |
| `PluginHandle<I>` | RAII: owns dl handle + instance. Destroys instance before dlclose. |
| `PluginRegistry` | Process-lifetime singleton storing dl handles when caller needs `unique_ptr<I>` |

Plugin .so contract: `extern "C" __attribute__((visibility("default"))) Interface* create_instance();`

### Config Key Registry — `drone::cfg_key` — Issue #287

> **File:** `common/util/include/util/config_keys.h`

~130 `inline constexpr const char*` constants organized per-process namespace. Eliminates string typos in `cfg.get<>()` calls. Namespaces: `zenoh::`, `perception::`, `slam::`, `mission_planner::`, `comms::`, `payload_manager::`, `system_monitor::`, `watchdog::`, `recorder::`, `hal::`, `fault_manager::`.

### ConfigValidator — `drone::util` — Issue #298

> **File:** `common/util/include/util/config_validator.h`

Fluent schema builder for config validation at startup.

| Method | Description |
|--------|-------------|
| `required<T>(key)` | Key must exist with correct type |
| `optional<T>(key)` | Key may exist; if present, must match type |
| `.range(lo, hi)` | Value must be in [lo, hi] |
| `.one_of({...})` | Value must be in set |
| `.satisfies(predicate, desc)` | Custom validation |
| `required_section(key)` | JSON object must exist |
| `custom(rule)` | Arbitrary validation lambda |

Pre-built schemas: `common_schema()`, `video_capture_schema()`, ..., `system_monitor_schema()`.

---

## 1.6 IPC Message Types (`common/ipc/include/ipc/ipc_types.h`)

Key wire-format structs used across pub/sub topics. All must be trivially copyable.

### `RadarDetection` — Issue [#209](https://github.com/nmohamaya/companion_software_stack/issues/209)

Single radar return from an `IRadar` backend.

```cpp
struct RadarDetection {
    uint64_t timestamp_ns;        // monotonic clock nanoseconds
    float    range_m;             // metres (must be ≥ 0)
    float    azimuth_rad;         // radians
    float    elevation_rad;       // radians
    float    radial_velocity_mps; // radial velocity, m/s
    float    rcs_dbsm;           // radar cross-section, dBsm
    float    snr_db;             // signal-to-noise ratio, dB
    float    confidence;         // 0.0–1.0
    uint32_t track_id;           // stable track identifier (0 = untracked)
};
```

### `RadarDetectionList` — Issue [#209](https://github.com/nmohamaya/companion_software_stack/issues/209)

Fixed-capacity list of radar returns published on `/radar_detections`.

```cpp
constexpr uint32_t MAX_RADAR_DETECTIONS = 128;

struct RadarDetectionList {
    uint64_t       timestamp_ns;
    uint32_t       num_detections;  // number of valid entries (≤ MAX_RADAR_DETECTIONS)
    RadarDetection detections[MAX_RADAR_DETECTIONS];
};
```

**Topic:** `/radar_detections` → Zenoh key `radar/detections`
**Publisher:** P2 (perception) or a dedicated radar process
**Subscribers:** P4 (mission planner) for obstacle avoidance fusion

### `DetectedObject` — Issues [#210](https://github.com/nmohamaya/companion_software_stack/issues/210), [#237](https://github.com/nmohamaya/companion_software_stack/issues/237)

IPC output struct for a single fused object, published as part of `DetectedObjectList` on `/detected_objects`.

The `has_radar` flag was added in Issue #210. Size estimation fields (`estimated_radius_m`, `estimated_height_m`) and `radar_update_count` were added in Epic #237 for per-object grid inflation and radar-confirmed static promotion.

| Field | Type | Description |
|-------|------|-------------|
| `position` | `Vector3f` | World-frame ENU position (m) |
| `velocity` | `Vector3f` | World-frame ENU velocity (m/s) |
| `class_id` | `uint8_t` | `ObjectClass` enum value |
| `heading` | `float` | Heading (rad); always 0.0 currently |
| `has_camera` | `bool` | Camera measurement present this cycle |
| `has_radar` | `bool` | Radar measurement matched/applied this cycle, or radar-init used (Issue #210, #237) |
| `estimated_radius_m` | `float` | Back-projected obstacle radius from camera bbox + radar range; 0 = unknown (Issue #237) |
| `estimated_height_m` | `float` | Back-projected obstacle height from camera bbox + radar range; 0 = unknown (Issue #237) |
| `radar_update_count` | `uint32_t` | Number of radar `update_radar()` calls on this track — used for grid promotion threshold (Issue #237) |
| `depth_confidence` | `float` | Depth estimation quality [0.0=guess, 1.0=radar]. Validated in [0,1] range. |

**Validation:** `validate()` checks all floats are `isfinite()`, confidence in [0,1], `estimated_radius_m` / `estimated_height_m` >= 0, and `depth_confidence` in [0,1].

### `VideoFrame`

Raw RGB24 video frame from a mission camera (P1 to P2).

```cpp
struct VideoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t stride;
    uint8_t  pixel_data[1920 * 1080 * 3];  // RGB24 max (~6 MB)
};
```

**Topic:** `/drone_mission_cam` to Zenoh key `drone/video/frame`
**Publisher:** P1 (video capture)
**Subscribers:** P2 (perception)
**Validation:** `validate()` checks non-zero dimensions and that `width * height * channels <= sizeof(pixel_data)`.

### `StereoFrame`

Grayscale stereo pair for VIO/SLAM (P1 to P3).

```cpp
struct StereoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint8_t  left_data[640 * 480];   // GRAY8
    uint8_t  right_data[640 * 480];  // GRAY8
};
```

**Topic:** `/drone_stereo_cam` to Zenoh key `drone/video/stereo_frame`
**Publisher:** P1 (video capture)
**Subscribers:** P3 (SLAM/VIO/Nav)
**Validation:** `validate()` checks non-zero dimensions and that `width * height` fits each buffer.

### `ObjectClass` (enum)

Classification label for detected objects.

| Value | Name |
|-------|------|
| 0 | `UNKNOWN` |
| 1 | `PERSON` |
| 2 | `VEHICLE_CAR` |
| 3 | `VEHICLE_TRUCK` |
| 4 | `DRONE` |
| 5 | `ANIMAL` |
| 6 | `BUILDING` |
| 7 | `TREE` |

### `DetectedObjectList`

Fixed-capacity list of detected objects published on `/detected_objects`.

```cpp
constexpr int MAX_DETECTED_OBJECTS = 64;

struct DetectedObjectList {
    uint64_t       timestamp_ns;
    uint32_t       frame_sequence;
    uint32_t       num_objects;       // valid entries (<=  MAX_DETECTED_OBJECTS)
    DetectedObject objects[MAX_DETECTED_OBJECTS];
};
```

**Topic:** `/detected_objects` to Zenoh key `drone/perception/detections`
**Publisher:** P2 (perception)
**Subscribers:** P4 (mission planner)

### `SemanticVoxel` / `SemanticVoxelBatch`

Output of the PATH A pipeline (Epic #520): mask-aware back-projection of SAM masks through
depth into world-frame voxels. Each voxel is already confidence-scored and class-labelled
by `MaskClassAssigner`, so the planner can insert into the occupancy grid without the
`promotion_hits` filter required by the detector-driven path.

```cpp
struct SemanticVoxel {
    float       position_x, position_y, position_z;  // world frame (m)
    float       occupancy;                           // [0, 1]
    float       confidence;                          // [0, 1]
    ObjectClass semantic_label;                      // from MaskClassAssigner
    uint8_t     _pad0[3];
    uint64_t    timestamp_ns;                        // source-frame capture time
};

constexpr int MAX_VOXELS_PER_BATCH = 1024;

struct SemanticVoxelBatch {
    uint64_t      timestamp_ns;     // batch emission time
    uint64_t      frame_sequence;   // source video-frame sequence
    uint32_t      num_voxels;
    uint32_t      _pad0;
    SemanticVoxel voxels[MAX_VOXELS_PER_BATCH];
};
```

**Topic:** `/semantic_voxels` to Zenoh key `drone/perception/voxels`
**Publisher:** P2 (perception, PATH A — added in PR wiring E5.INT / Issue #608)
**Subscribers:** P4 (mission planner — occupancy-grid writer, added in PR 3)
**Wire size:** ~32 KB per batch (1024 voxels × 32 B + header)
**Trivially copyable / standard layout:** yes — `static_assert`ed in `ipc_types.h`

Coexists with `/detected_objects` for backwards compatibility: scenarios that keep the
detector-only path unchanged see no new traffic on this channel.

### `MissionState` (enum)

FSM state for the mission planner (published as part of `MissionStatus`).

| Value | Name | Description |
|-------|------|-------------|
| 0 | `IDLE` | No mission loaded |
| 1 | `PREFLIGHT` | Pre-flight checks |
| 2 | `TAKEOFF` | Taking off |
| 3 | `NAVIGATE` | Following waypoints |
| 4 | `LOITER` | Holding position |
| 5 | `RTL` | Return to launch |
| 6 | `LAND` | Landing |
| 7 | `EMERGENCY` | Emergency state |
| 8 | `SURVEY` | Post-takeoff obstacle survey before navigation |
| 9 | `COLLISION_RECOVERY` | Post-collision recovery: hover, climb, replan |

### `FaultAction` (enum)

Graduated response severity (stored in `MissionStatus::fault_action`).

| Value | Name | Description |
|-------|------|-------------|
| 0 | `NONE` | Nominal -- no action |
| 1 | `WARN` | Alert GCS, continue mission |
| 2 | `LOITER` | Hold position, wait for recovery |
| 3 | `RTL` | Return to launch |
| 4 | `EMERGENCY_LAND` | Land immediately at current position |

### `FaultType` (bitmask enum)

Bitmask of active fault conditions (stored in `MissionStatus::active_faults`). 12 fault conditions total.

| Bit | Name | Description |
|-----|------|-------------|
| `1 << 0` | `FAULT_CRITICAL_PROCESS` | Comms or SLAM died |
| `1 << 1` | `FAULT_POSE_STALE` | No pose update within timeout |
| `1 << 2` | `FAULT_BATTERY_LOW` | Battery below warning threshold |
| `1 << 3` | `FAULT_BATTERY_CRITICAL` | Battery below critical threshold |
| `1 << 4` | `FAULT_THERMAL_WARNING` | Thermal zone 2 (hot) |
| `1 << 5` | `FAULT_THERMAL_CRITICAL` | Thermal zone 3 (critical) |
| `1 << 6` | `FAULT_PERCEPTION_DEAD` | Perception process died |
| `1 << 7` | `FAULT_FC_LINK_LOST` | FC not connected for > timeout |
| `1 << 8` | `FAULT_GEOFENCE_BREACH` | Outside geofence boundary |
| `1 << 9` | `FAULT_BATTERY_RTL` | Battery below RTL threshold |
| `1 << 10` | `FAULT_VIO_DEGRADED` | VIO quality degraded -- triggers LOITER |
| `1 << 11` | `FAULT_VIO_LOST` | VIO tracking lost -- triggers RTL |

Bitwise operators (`|`, `&`, `|=`, `==`, `!=`) and `to_uint()` / `fault_flags_string()` helpers are provided for bitmask usage.

### `MissionStatus`

Mission planner status (P4 to P5, P7).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `correlation_id` | `uint64_t` | Cross-process trace ID (0 = none) |
| `state` | `MissionState` | Current FSM state |
| `current_waypoint` | `uint32_t` | Index of current waypoint |
| `total_waypoints` | `uint32_t` | Total waypoints in mission |
| `progress_percent` | `float` | Mission completion percentage |
| `target_x/y/z` | `float` | Current target position (world frame, m) |
| `battery_percent` | `float` | Battery level |
| `mission_active` | `bool` | True if mission is running |
| `active_faults` | `uint32_t` | Bitmask of `FaultType` (0 = nominal) |
| `fault_action` | `uint8_t` | Current `FaultAction` severity (0 = NONE) |

**Topic:** `/mission_status` to Zenoh key `drone/mission/status`
**Publisher:** P4 (mission planner)
**Subscribers:** P5 (comms), P7 (system monitor)

### `TrajectoryCmd`

Velocity command from planner to flight controller (P4 to P5).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `correlation_id` | `uint64_t` | Cross-process trace ID (0 = none) |
| `target_x/y/z` | `float` | Target position, world frame (m) |
| `target_yaw` | `float` | Target heading (radians) |
| `velocity_x/y/z` | `float` | Velocity command (m/s) |
| `yaw_rate` | `float` | Yaw rate command (rad/s) |
| `coordinate_frame` | `uint8_t` | MAVLink frame enum |
| `valid` | `bool` | False = stop command |

**Topic:** `/trajectory_cmd` to Zenoh key `drone/mission/trajectory`

### `FCCommandType` (enum)

| Value | Name |
|-------|------|
| 0 | `NONE` |
| 1 | `ARM` |
| 2 | `DISARM` |
| 3 | `TAKEOFF` |
| 4 | `SET_MODE` |
| 5 | `RTL` |
| 6 | `LAND` |

### `FCCommand`

Flight controller command (P4 to P5).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `correlation_id` | `uint64_t` | Cross-process trace ID (0 = none) |
| `command` | `FCCommandType` | Command type |
| `param1` | `float` | TAKEOFF: altitude_m; SET_MODE: mode_id |
| `sequence_id` | `uint64_t` | Monotonic, for dedup |
| `valid` | `bool` | Message validity flag |

**Topic:** `/fc_commands` to Zenoh key `drone/comms/fc_command`

### `FCState`

Flight controller telemetry (P5 to P4, P7).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `gps_lat/lon/alt` | `float` | GPS position |
| `rel_alt` | `float` | Relative altitude (m) |
| `roll/pitch/yaw` | `float` | Attitude (radians) |
| `vx/vy/vz` | `float` | Velocity (m/s) |
| `battery_voltage` | `float` | Battery voltage (0-60V) |
| `battery_remaining` | `float` | Battery percentage (0-100%) |
| `flight_mode` | `uint8_t` | FC flight mode |
| `armed` | `bool` | True if armed |
| `connected` | `bool` | True if FC link active |
| `gps_fix_type` | `uint8_t` | GPS fix type |
| `satellites_visible` | `uint8_t` | GPS satellite count |

**Topic:** `/fc_state` to Zenoh key `drone/comms/fc_state`

### `GCSCommandType` (enum)

| Value | Name |
|-------|------|
| 0 | `NONE` |
| 1 | `ARM` |
| 2 | `DISARM` |
| 3 | `TAKEOFF` |
| 4 | `LAND` |
| 5 | `RTL` |
| 6 | `MISSION_START` |
| 7 | `MISSION_PAUSE` |
| 8 | `MISSION_ABORT` |
| 9 | `MISSION_UPLOAD` |

### `GCSCommand`

Ground control station command (P5 to P4).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `correlation_id` | `uint64_t` | Cross-process trace ID (0 = none) |
| `command` | `GCSCommandType` | Command type |
| `param1/2/3` | `float` | Command-specific parameters |
| `sequence_id` | `uint64_t` | Monotonic, for dedup |
| `valid` | `bool` | Message validity flag |

**Topic:** `/gcs_commands` to Zenoh key `drone/comms/gcs_command`

### `IpcWaypoint`

Single waypoint for mid-flight mission upload.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `x/y/z` | `float` | 0.0 | World-frame position (m) |
| `yaw` | `float` | 0.0 | Heading (rad) |
| `radius` | `float` | 2.0 | Acceptance radius (m) |
| `speed` | `float` | 2.0 | Cruise speed (m/s) |
| `trigger_payload` | `bool` | false | Capture at this waypoint |

### `MissionUpload`

Mid-flight waypoint upload (P5 to P4). Sent when GCS issues `MISSION_UPLOAD`.

```cpp
constexpr uint8_t kMaxUploadWaypoints = 32;

struct MissionUpload {
    uint64_t    timestamp_ns;
    uint64_t    correlation_id;
    uint8_t     num_waypoints;      // valid entries (<= kMaxUploadWaypoints)
    IpcWaypoint waypoints[kMaxUploadWaypoints];
    bool        valid;
};
```

**Topic:** `/mission_upload` to Zenoh key `drone/mission/upload`
**Publisher:** P5 (comms)
**Subscribers:** P4 (mission planner)

### `PayloadAction` (enum)

| Value | Name |
|-------|------|
| 0 | `NONE` |
| 1 | `GIMBAL_POINT` |
| 2 | `CAMERA_CAPTURE` |
| 3 | `CAMERA_START_VIDEO` |
| 4 | `CAMERA_STOP_VIDEO` |

### `PayloadCommand`

Gimbal/camera command (P4 to P6).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `correlation_id` | `uint64_t` | Cross-process trace ID (0 = none) |
| `action` | `PayloadAction` | Payload action type |
| `gimbal_pitch/yaw` | `float` | Gimbal angles (degrees) |
| `sequence_id` | `uint64_t` | Monotonic, for dedup |
| `valid` | `bool` | Message validity flag |

**Topic:** `/payload_commands` to Zenoh key `drone/mission/payload_command`

### `PayloadStatus`

Payload telemetry (P6 to P4, P7).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `gimbal_pitch/yaw` | `float` | Current gimbal angles (degrees) |
| `images_captured` | `uint32_t` | Total images captured |
| `recording_video` | `bool` | True if recording |
| `gimbal_stabilized` | `bool` | True if stabilized |
| `num_plugins_active` | `uint8_t` | Active payload plugins |

**Topic:** `/payload_status` to Zenoh key `drone/payload/status`

### `Pose`

6-DOF pose from VIO/SLAM (P3 to P4, P5, P6). Aligned to 64 bytes.

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `translation[3]` | `double` | x, y, z in world frame (m) |
| `quaternion[4]` | `double` | w, x, y, z rotation |
| `velocity[3]` | `double` | vx, vy, vz (m/s) |
| `covariance[36]` | `double` | 6x6 pose covariance matrix |
| `quality` | `uint32_t` | 0=lost, 1=degraded, 2=good, 3=excellent (ground truth) |

**Topic:** `/slam_pose` to Zenoh key `drone/slam/pose`

### `ProcessHealthEntry`

Per-process liveness state, embedded in `SystemHealth`.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `name[32]` | `char[]` | `""` | Process name (e.g. "video_capture") |
| `alive` | `bool` | false | Last known liveness state |
| `last_seen_ns` | `uint64_t` | 0 | Timestamp of last liveliness event |

Constant: `kMaxTrackedProcesses = 8` (7 processes + 1 FC link).

### `SystemHealth`

System-wide health report (P7 to all).

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | Monotonic clock nanoseconds |
| `cpu_usage_percent` | `float` | CPU utilization |
| `memory_usage_percent` | `float` | Memory utilization |
| `disk_usage_percent` | `float` | Disk utilization |
| `max_temp_c` | `float` | Maximum temperature (C) |
| `gpu_temp_c` | `float` | GPU temperature (C) |
| `cpu_temp_c` | `float` | CPU temperature (C) |
| `total_healthy/degraded/dead` | `uint32_t` | Process status counts |
| `power_watts` | `float` | Power consumption |
| `thermal_zone` | `uint8_t` | 0=normal, 1=warm, 2=hot, 3=critical |
| `stack_status` | `uint8_t` | StackStatus enum (0=NOMINAL, 1=DEGRADED, 2=CRITICAL) |
| `total_restarts` | `uint32_t` | Cumulative restart count across all processes |
| `processes[]` | `ProcessHealthEntry[8]` | Per-process liveness (populated by LivelinessMonitor) |
| `num_processes` | `uint8_t` | Number of tracked processes |
| `critical_failure` | `bool` | True if a critical process died |

**Topic:** `/system_health` to Zenoh key `drone/monitor/health`

### `ThreadHealthEntry`

Per-thread liveness state, embedded in `ThreadHealth`.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `name[32]` | `char[]` | `""` | Thread name (e.g. "mission_cam") |
| `healthy` | `bool` | true | False when watchdog detects stuck |
| `critical` | `bool` | false | True for mission-critical threads |
| `last_ns` | `uint64_t` | 0 | Last heartbeat timestamp (ns) |

Constant: `kMaxTrackedThreads = 16`.

### `ThreadHealth`

Per-process thread health snapshot (each process to P7).

```cpp
struct ThreadHealth {
    char              process_name[32];
    ThreadHealthEntry threads[kMaxTrackedThreads];  // up to 16 threads
    uint8_t           num_threads;
    uint64_t          timestamp_ns;
};
```

**Topics:** 7 per-process channels:
- `/drone_thread_health_video_capture`
- `/drone_thread_health_perception`
- `/drone_thread_health_slam_vio_nav`
- `/drone_thread_health_mission_planner`
- `/drone_thread_health_comms`
- `/drone_thread_health_payload_manager`
- `/drone_thread_health_system_monitor`

**Publisher:** Each process (P1-P7)
**Subscriber:** P7 (system monitor)

### `FaultOverrides`

Fault injection overrides written by the `fault_injector` tool and read by owning processes. Each field uses a sentinel value (< 0) for "no override". Aligned to 64 bytes.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `battery_percent` | `float` | -1.0 | Override FC battery % (< 0 = no override) |
| `battery_voltage` | `float` | -1.0 | Override FC battery voltage (< 0 = no override) |
| `fc_connected` | `int32_t` | -1 | Override FC connection (< 0 = no override, 0 = disconnected, 1 = connected) |
| `thermal_zone` | `int32_t` | -1 | Override thermal zone (< 0 = no override, 0-3 = zone) |
| `cpu_temp_override` | `float` | -1.0 | Override CPU temperature (< 0 = no override) |
| `vio_quality` | `int32_t` | -1 | Override VIO quality level (< 0 = no override, 0-3 = quality) |
| `sequence` | `uint64_t` | 0 | Incremented by injector so consumers detect new writes |

**Topic:** `/fault_overrides` to Zenoh key `fault/overrides`
**Publisher:** `fault_injector` tool
**Subscribers:** P5 (comms), P7 (system monitor), P3 (SLAM/VIO)

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

### `IVIOBackend` — `drone::slam`

**Header:** `process3_slam_vio_nav/include/slam/ivio_backend.h`  
**Used by:** Process 3 (SLAM/VIO/Nav)

| Method | Signature | Description |
|--------|-----------|-------------|
| `process_frame` | `VIOResult<VIOOutput> process_frame(const StereoFrame&, const std::vector<ImuSample>&)` | Process stereo frame + IMU, return pose + diagnostics |
| `health` | `VIOHealth health() const` | Current pipeline health |
| `name` | `std::string name() const` | Implementation name for logging |
| `set_trajectory_target` | `void set_trajectory_target(float x, float y, float z, float yaw)` | Set target for simulated navigation (no-op on real/Gazebo) |

| Implementation | Description |
|----------------|-------------|
| `SimulatedVIOBackend` | Target-following dynamics + feature extraction + stereo matching + IMU pre-integration |
| `GazeboVIOBackend` | Ground-truth odometry via gz-transport (HAVE_GAZEBO) |
| `GazeboFullVIOBackend` | Full VIO pipeline on Gazebo frames + ground-truth pose (HAVE_GAZEBO) |

**Factory:** `create_vio_backend(backend)` — backends: `"simulated"`, `"gazebo"`, `"gazebo_full_vio"`

```cpp
auto vio = create_vio_backend("simulated");
auto result = vio->process_frame(stereo_frame, imu_samples);
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
| `DStarLitePlanner` | 3D incremental D* Lite search with obstacle-aware routing, EMA smoothing |

**Factory:** `create_path_planner(backend)` — backends: `"dstar_lite"`

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
| `ObstacleAvoider3D` | `influence_radius` (default 5.0m), `repulsive_gain` (default 2.0) | Full 3D repulsive field with velocity prediction, NaN guards, per-axis clamping |

**Factory:** `create_obstacle_avoider(backend, influence_radius, repulsive_gain)` — backends: `"3d"`, `"obstacle_avoider_3d"`, `"potential_field_3d"`

---

### `FaultManager` — `drone::planner`

**Header:** `process4_mission_planner/include/planner/fault_manager.h`  
**Used by:** Process 4 (Mission Planner)  
**Issue:** [#61](https://github.com/nmohamaya/companion_software_stack/issues/61)

Config-driven graceful degradation engine. Evaluates system health each loop tick and returns the highest-priority response action. Escalation-only policy — once raised, actions never downgrade within a flight.

| Type | Description |
|------|-------------|
| `FaultAction` | Enum: `NONE(0)` < `WARN(1)` < `LOITER(2)` < `RTL(3)` < `EMERGENCY_LAND(4)` |
| `FaultType` | Bitmask enum: 12 fault conditions (critical process, pose stale, battery low/critical/RTL, thermal warn/critical, perception dead, FC link lost, geofence breach, VIO degraded, VIO lost) |
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

### `IRadar` — `drone::hal` — Issue [#209](https://github.com/nmohamaya/companion_software_stack/issues/209)

**Header:** `common/hal/include/hal/iradar.h`
**Used by:** P2 (perception) or a dedicated radar process

Radar sensor interface. Returns a `RadarDetectionList` each call to `read()`. Designed for short-range obstacle detection and velocity measurement.

| Method | Signature | Description |
|--------|-----------|-------------|
| `init` | `bool init()` | Initialise hardware / start acquisition. Returns false on error. |
| `read` | `RadarDetectionList read()` | Non-blocking read of latest detections. Before successful `init()`, returns an empty list (`num_detections == 0`). |
| `is_active` | `bool is_active() const` | True after successful `init()`. |
| `name` | `std::string name() const` | Human-readable backend identifier (e.g., `"SimulatedRadar"`). |

| Key | Class | Notes |
| --- | ----- | ----- |
| `"simulated"` | `SimulatedRadar` | Configurable FoV, range, target count, and Gaussian noise model. |

**Config section:** `perception.radar`

**Key config keys:**

| Key | Default | Description |
| --- | ------- | ----------- |
| `enabled` | `false` | Enable radar sensor |
| `backend` | `"simulated"` | Backend selection |
| `fov_azimuth_rad` | 1.047 | Horizontal field-of-view in radians (~60° half-angle) |
| `fov_elevation_rad` | 0.262 | Vertical field-of-view in radians (~15° half-angle) |
| `max_range_m` | 100.0 | Maximum detection range in metres |
| `num_targets` | 3 | Simulated target count (`SimulatedRadar`) |
| `noise.range_std_m` | 0.3 | Range noise standard deviation in metres |
| `noise.azimuth_std_rad` | 0.026 | Azimuth noise standard deviation in radians |
| `noise.elevation_std_rad` | 0.026 | Elevation noise standard deviation in radians |
| `noise.velocity_std_mps` | 0.1 | Velocity noise standard deviation in m/s |
| `false_alarm_rate` | 0.02 | Probability of false alarm per scan |

**Factory:** `create_radar(cfg, section)` — returns `std::unique_ptr<IRadar>`

---

### `IFusionEngine` — `drone::perception` — Issue [#210](https://github.com/nmohamaya/companion_software_stack/issues/210)

**Header:** `process2_perception/include/perception/ifusion_engine.h`
**Used by:** Process 2 (Perception), fusion thread

Abstract interface for the per-frame fusion step that maps 2D tracked objects to 3D camera-frame positions. The `set_radar_detections()` method was added in Issue #210. The `set_drone_altitude()` and `set_drone_pose()` methods were added in Epic #237 for radar ground-plane filtering and world-frame dormant re-identification.

| Method | Signature | Description |
|--------|-----------|-------------|
| `fuse` | `[[nodiscard]] FusedObjectList fuse(const TrackedObjectList& tracked)` | Fuse tracked 2D objects into 3D estimates. Returns `FusedObjectList`. |
| `set_radar_detections` | `virtual void set_radar_detections(const RadarDetectionList&)` | Store latest radar scan for the next `fuse()` call. Default no-op (base class). `UKFFusionEngine` overrides this. |
| `set_drone_altitude` | `virtual void set_drone_altitude(float altitude_m)` | Provide drone AGL altitude for radar ground-plane filtering (Issue #237). Default no-op. |
| `set_drone_pose` | `virtual void set_drone_pose(float north, float east, float up, float yaw)` | Provide full drone pose for body→world transform in dormant re-identification (Issue #237). Default no-op. |
| `name` | `virtual std::string name() const = 0` | Implementation name for logging |

| Implementation | Description |
|----------------|-------------|
| `CameraOnlyFusionEngine` | Stateless monocular depth estimation (pinhole apparent-size formula). No radar support. |
| `UKFFusionEngine` | Per-track Unscented Kalman Filter. Supports camera update and gated Mahalanobis radar association + update. |

**Factory:** `create_fusion_engine(backend, cfg)` — backends: `"camera_only"`, `"ukf"`

---

### `RadarNoiseConfig` — `drone::perception` — Issues [#210](https://github.com/nmohamaya/companion_software_stack/issues/210), [#237](https://github.com/nmohamaya/companion_software_stack/issues/237)

**Header:** `process2_perception/include/perception/ukf_fusion_engine.h`

Configurable noise and radar-primary parameters for `UKFFusionEngine`. Noise fields control the radar measurement model; radar-primary fields (Epic #237) control independent radar track creation and output gating.

**Config keys** (under `perception.fusion.radar`):

| Key | Default | Description |
|-----|---------|-------------|
| `range_std_m` | `0.3` | Range noise standard deviation (m) |
| `azimuth_std_rad` | `0.026` | Azimuth noise standard deviation (rad) |
| `elevation_std_rad` | `0.026` | Elevation noise standard deviation (rad) |
| `velocity_std_mps` | `0.1` | Radial velocity noise standard deviation (m/s) |
| `gate_threshold` | `9.21` | χ²(4) Mahalanobis gate at 95% confidence |
| `min_object_altitude_m` | `0.3` | Reject radar returns below this AGL (ground filter) |
| `ground_filter_enabled` | `true` | Enable/disable ground-plane filter |
| `altitude_gate_m` | `2.0` | Reject radar-track pairs with body-Z diff > this |
| `radar_orphan_proximity_m` | `3.0` | Min distance to existing track for orphan creation |
| `radar_adopt_gate_m` | `5.0` | Max range error for camera adoption of radar-only track |
| `radar_only_default_radius_m` | `1.5` | Conservative inflation radius for radar-only tracks (no bbox) |
| `radar_max_orphan_range_m` | `40.0` | Max range for orphan track creation |
| `orphan_min_hits` | `1` | Min radar observations before radar-only track is output to grid |

---

### `FusedObject` — `drone::perception` — Issue [#210](https://github.com/nmohamaya/companion_software_stack/issues/210)

**Header:** `process2_perception/include/perception/types.h`

Per-track fusion output (camera body frame before world-ENU rotation). Size estimation and radar fields added in Epic #237.

| Field | Type | Description |
|-------|------|-------------|
| `track_id` | `uint32_t` | Matches `TrackedObject::track_id`; radar-only tracks have high bit set (>= `0x80000000`) |
| `position_3d` | `Vector3f` | 3D position in camera body frame (or world frame if `in_world_frame` is true) |
| `velocity_3d` | `Vector3f` | 3D velocity estimate (m/s) in body frame |
| `position_covariance` | `Matrix3f` | 3×3 position covariance (UKF only; fixed `5·I₃` for camera_only) |
| `has_camera` | `bool` | Camera measurement applied this cycle |
| `has_radar` | `bool` | Radar detection matched/applied this cycle, or radar-init used |
| `in_world_frame` | `bool` | Position is already in world frame (from dormant re-ID pool); skip body→world transform |
| `estimated_radius_m` | `float` | Back-projected obstacle radius; 0 = unknown |
| `estimated_height_m` | `float` | Back-projected obstacle height; 0 = unknown |
| `radar_update_count` | `uint32_t` | Number of `update_radar()` calls — drives grid promotion threshold |

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

## 4.5 Flight Data Recorder — `drone::recorder`

**Header:** `common/recorder/include/recorder/flight_recorder.h`  
**Issue:** [#40](https://github.com/nmohamaya/companion_software_stack/issues/40)

Ring-buffer based IPC traffic recorder for post-flight analysis and replay. Records messages to an in-memory ring buffer (configurable capacity), then flushes to a timestamped `.flog` binary file.

### Binary Format (`.flog`)

Each log file starts with a `FlightLogFileHeader` (24 bytes), followed by a sequence of records:

```
[FlightLogFileHeader (24 B)]
[RecordHeader (34 B)] [topic_name (variable)] [payload (variable)]
[RecordHeader (34 B)] [topic_name (variable)] [payload (variable)]
...
```

**`FlightLogFileHeader`** (written once at file start):

| Field | Type | Description |
|-------|------|-------------|
| `magic` | `uint32_t` | `0x474F4C46` ("FLOG" in little-endian) |
| `version` | `uint32_t` | File format version (currently 1) |
| `start_time_ns` | `uint64_t` | Monotonic timestamp of first record |
| `reserved` | `uint64_t` | Reserved for future use |

**`RecordHeader`** (per record, 34 bytes):

| Field | Type | Description |
|-------|------|-------------|
| `wire_header` | `WireHeader` (32 B) | Standard IPC wire header (magic, version, msg_type, payload_size, sequence, timestamp_ns) |
| `topic_name_len` | `uint16_t` | Length of topic name string following this header |

After each `RecordHeader`, `topic_name_len` bytes of topic name string, then `wire_header.payload_size` bytes of payload.

### `RecordRingBuffer`

Fixed-capacity ring buffer using `std::deque` for O(1) front eviction. Discards oldest entries when `max_bytes` is reached.

### `FlightRecorder`

| Method | Signature | Description |
|--------|-----------|-------------|
| `record` | `void record<T>(topic, msg_type, msg, seq)` | Record a trivially-copyable message (no-op if not recording) |
| `start` / `stop` | `void start()` / `void stop()` | Atomic recording flag (lock-free) |
| `is_recording` | `bool is_recording() const` | Check recording state |
| `flush` | `std::string flush()` | Flush ring buffer to timestamped `.flog` file; returns path or `""` on failure |
| `entry_count` | `std::size_t entry_count() const` | Buffered entry count |
| `buffered_bytes` | `std::size_t buffered_bytes() const` | Buffered byte count |

**Config keys** (under `recorder`):

| Key | Default | Description |
|-----|---------|-------------|
| `max_size_mb` | 64 | Ring buffer capacity in MB (clamped to [1, 4096]) |
| `output_dir` | `/tmp/flight_logs` | Directory for `.flog` output files |

**Wire version validation:** The `flight_replay` tool validates `kWireVersion` on each record during replay, skipping records with mismatched versions to handle format evolution.

### Read/Write Functions

| Function | Signature | Description |
|----------|-----------|-------------|
| `write_log_file` | `bool write_log_file(path, entries, start_time_ns)` | Serialize entries to `.flog` binary file |
| `read_log_file` | `ReadLogResult read_log_file(path)` | Deserialize `.flog` file; detects truncation and corruption |

---

## 4.6 Gimbal Auto-Tracking — `drone::payload`

**Header:** `process6_payload_manager/include/payload/auto_tracker.h`  
**Used by:** Process 6 (Payload Manager)

Transforms world-frame object positions into body-frame bearing using the SLAM pose, then computes gimbal pitch/yaw angles to point at the highest-confidence tracked object.

### `AutoTrackConfig`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `enabled` | `bool` | `false` | Master enable for auto-tracking |
| `min_confidence` | `float` | `0.5` | Ignore objects below this confidence |

### `AutoTrackResult`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `has_target` | `bool` | `false` | True if a valid target was found |
| `pitch_deg` | `float` | `0.0` | Gimbal pitch command (degrees) |
| `yaw_deg` | `float` | `0.0` | Gimbal yaw command (degrees) |
| `target_confidence` | `float` | `0.0` | Confidence of the selected target |
| `target_track_id` | `uint32_t` | `0` | Track ID of the selected target |

### Functions

| Function | Signature | Description |
|----------|-----------|-------------|
| `compute_auto_track` | `AutoTrackResult compute_auto_track(objects, pose, config)` | Select highest-confidence object, transform world to body frame via pose yaw, compute pitch/yaw |
| `compute_bearing` | `pair<float,float> compute_bearing(dx, dy, dz)` | Pitch/yaw angles from body-frame relative position |
| `yaw_from_quaternion` | `float yaw_from_quaternion(const double quat[4])` | Extract yaw (rad) from quaternion (w, x, y, z) |

**Coordinate transform:** World→body rotation uses yaw-only (`R_z(-yaw)`) from the SLAM pose quaternion. Roll/pitch are assumed near-zero for gimbal bearing computation. Body frame: x=forward, y=right, z=up.

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
auto planner = create_path_planner(cfg.get("planner.backend", "dstar_lite"));
auto avoider = create_obstacle_avoider(
    cfg.get("avoider.backend", "potential_field_3d"),
    cfg.get("avoider.influence_radius", 5.0f),
    cfg.get("avoider.repulsive_gain",   2.0f));
```

---

## 6. Adding a New Implementation

1. Create a class inheriting the abstract interface (e.g., `class OrbSlam3Backend : public IVIOBackend`)
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
| `ZenohMessageBus` | `zenoh_message_bus.h` | — | **Implemented** | Factory with Zenoh topic mapping (13 explicit + fallback rule) |
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
