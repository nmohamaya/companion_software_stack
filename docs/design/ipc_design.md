# IPC Layer: Design Document

> **Scope**: Detailed design of the inter-process communication layer in `common/ipc/`.
> This document covers the Zenoh transport, wire format, liveliness
> monitoring, service channels, and the transport-agnostic MessageBus abstraction.

> **Note (March 2026):** The POSIX SHM backend was removed in Issue #126 (PR #151).
> **Zenoh is the sole IPC backend.** Historical SHM sections are retained below
> (marked as such) for architectural reference. See
> [ADR-001](adr/ADR-001-ipc-framework-selection.md) for the migration rationale.

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Message Types](#message-types)
4. [SHM Transport — Historical](#shm-transport--historical)
5. [SPSC Ring Buffer](#spsc-ring-buffer)
6. [Zenoh Transport](#zenoh-transport)
7. [Wire Format](#wire-format)
8. [Topic Mapping](#topic-mapping)
9. [MessageBus Abstraction](#messagebus-abstraction)
10. [Service Channels](#service-channels)
11. [Liveliness Monitoring](#liveliness-monitoring)
12. [Configuration Reference](#configuration-reference)
13. [Testing](#testing)
14. [Known Limitations](#known-limitations)

---

## Overview

The IPC layer provides type-safe, zero-copy communication between the 7 processes
using **Zenoh** as the sole transport backend.

Zenoh provides network-capable, zero-copy SHM + TCP/UDP pub-sub. Messages use
**latest-value semantics** — only the most recent message is retained. Older
messages are silently overwritten.

Process code never touches Zenoh directly. All IPC goes through the
`MessageBus` abstraction, which is created via a factory. The `MessageBus`
interface is backend-agnostic by design ([ADR-002](adr/ADR-002-modular-ipc-backend.md)),
so adding a new backend (e.g., iceoryx, DDS) requires one file change — zero
process code changes.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      Process Code                            │
│                                                              │
│   IPublisher<T>::publish()          ISubscriber<T>::receive() │
│        │                                    ▲                │
│        ▼                                    │                │
│  ┌────────────────────── MessageBus ──────────────────────┐  │
│  │  ZenohMessageBus (sole backend)                        │  │
│  └───────────────────────┬────────────────────────────────┘  │
│                          │                                   │
│                ┌─────────▼──────────┐                        │
│                │  Zenoh Backend      │                        │
│                │  ZenohPublisher<T>  │                        │
│                │  ZenohSubscriber<T> │                        │
│                │  (Session)          │                        │
│                └────────────────────┘                         │
│                          │                                   │
│                   Zenoh SHM + TCP                             │
│                (zenoh::Session)                               │
└──────────────────────────────────────────────────────────────┘
```

---

## Message Types

All IPC messages are defined in [`shm_types.h`](../common/ipc/include/ipc/shm_types.h).
Every type is **trivially copyable** (no heap allocations, no virtual methods).

### Video Channels (P1 → P2, P3)

| Type | Size | Content |
|------|------|---------|
| `ShmVideoFrame` | ~6.2 MB | 1920×1080×3 RGB24 mission camera |
| `ShmStereoFrame` | ~600 KB | 640×480 grayscale × 2 (left + right) |
| `ShmThermalFrame` | ~320 KB | 640×512 grayscale LWIR thermal |

### Perception Output (P2 → P4)

| Type | Content |
|------|---------|
| `ShmDetectedObject` | Track ID, class (7 types), confidence, 3D position + velocity, heading, bbox, sensor flags |
| `ShmDetectedObjectList` | Array of up to 64 `ShmDetectedObject`, timestamp, frame sequence |

Object classes: `PERSON`, `VEHICLE_CAR`, `TRUCK`, `DRONE`, `ANIMAL`, `BUILDING`, `TREE`.

### Navigation (P3 → P4, P5)

| Type | Content |
|------|---------|
| `ShmPose` | 3D translation, quaternion rotation, velocity, 6×6 covariance, quality (0=lost, 1=degraded, 2=good) |

### Mission & Commands

| Type | Direction | Content |
|------|-----------|---------|
| `ShmMissionStatus` | P4 → P5, P7 | FSM state, waypoint progress, fault action + active_faults bitmask |
| `ShmTrajectoryCmd` | P4 → P5 | Target pose + velocity, coordinate frame |
| `ShmFCCommand` | P4 → P5 | ARM/DISARM/TAKEOFF/SET_MODE/RTL/LAND + monotonic sequence_id |
| `ShmGCSCommand` | P5 → P4 | ARM/DISARM/TAKEOFF/LAND/RTL/MISSION_START/PAUSE/ABORT/UPLOAD |
| `ShmMissionUpload` | P5 → P4 | Up to 32 waypoints (x/y/z, yaw, radius, speed, payload trigger) |
| `ShmPayloadCommand` | P4 → P6 | Gimbal pointing + camera action |

### State & Health

| Type | Direction | Content |
|------|-----------|---------|
| `ShmFCState` | P5 → P4, P7 | GPS, attitude, velocity, battery, mode, armed, satellites |
| `ShmPayloadStatus` | P6 → P4, P7 | Gimbal angles, capture count, recording flag |
| `ShmSystemHealth` | P7 → P4 | CPU/mem/temp/disk/battery, per-process health, stack status |
| `ShmThreadHealth` | All → P7 | Per-thread heartbeat status (name, healthy, critical, last_ns) |

### Fault Injection

| Type | Direction | Content |
|------|-----------|---------|
| `ShmFaultOverrides` | Test harness → P5, P7 | Battery/FC state/thermal overrides, sequence counter |

---

## SHM Transport — Historical

> **Removed in Issue #126 (PR #151).** This section is retained for architectural
> reference only. The POSIX SHM SeqLock backend is no longer part of the codebase.

- **Former headers:** `shm_writer.h`, `shm_reader.h` (removed)

### Memory Layout

Each POSIX SHM segment (`/dev/shm/drone_*`) contains:

```
┌──────────────────────────────────┐
│ seq: atomic<uint64_t>  (8 bytes) │  SeqLock counter
│ timestamp_ns: uint64_t (8 bytes) │  Writer timestamp
│ data: T                (N bytes) │  Message payload
└──────────────────────────────────┘
```

### Write Protocol

```cpp
void ShmWriter<T>::write(const T& data) {
    block->seq.store(seq + 1, release);    // Odd = writing (torn)
    block->timestamp_ns = now();
    memcpy(&block->data, &data, sizeof(T));
    block->seq.store(seq + 2, release);    // Even = idle (consistent)
}
```

### Read Protocol

```cpp
bool ShmReader<T>::read(T& out, uint64_t* ts) {
    for (int attempt = 0; attempt < 4; ++attempt) {
        uint64_t s1 = block->seq.load(acquire);
        if (s1 & 1) continue;               // Odd = writer active
        out = block->data;                   // Copy data
        *ts = block->timestamp_ns;
        uint64_t s2 = block->seq.load(acquire);
        if (s1 == s2) return true;           // Consistent read
    }
    return false;                            // Torn read after 4 retries
}
```

### Ownership Modes

| Method | Segment Lifecycle |
|--------|-------------------|
| `create(name)` | Writer creates + owns. `shm_unlink` on destructor. |
| `create_non_owning(name)` | Writer creates but does NOT unlink. |
| `attach(name)` | Writer opens existing. Does not own. |
| `open(name)` (reader) | Reader opens existing. Does not own. |

---

## SPSC Ring Buffer

- **Header:** [`spsc_ring.h`](../common/ipc/include/ipc/spsc_ring.h)

Lock-free single-producer / single-consumer ring for **intra-process** thread
communication (not used for inter-process IPC).

```cpp
template<typename T, size_t N>  // N must be power of 2
class SPSCRing {
    bool try_push(const T& item);      // O(1), false if full
    std::optional<T> try_pop();        // O(1), nullopt if empty
    uint64_t available() const;
};
```

### Design

- Atomic 64-bit write/read indices, cache-line aligned to avoid false sharing
- `acquire`/`release` memory ordering (no seq_cst)
- Full when `write − read ≥ N`
- Empty when `read ≥ write`
- Wrapping: indices grow monotonically, masked at `N − 1` for array access

Used for high-throughput intra-process producer/consumer queues where the single-writer, single-reader constraint can be guaranteed.

---

## Zenoh Transport

### ZenohSession (Singleton)

```cpp
class ZenohSession {
    static ZenohSession& instance();     // Singleton (intentionally leaked)
    void configure(const std::string& json);
    void configure_network(ZenohNetworkConfig cfg);
    void configure_shm(size_t pool_bytes);
    zenoh::Session& session();           // Lazy-open on first call
    zenoh::PosixShmProvider* shm_provider();
};
```

The session is **intentionally leaked** (never destroyed) to avoid Rust FFI panics
during static destruction.

### ZenohPublisher\<T\>

Two publish paths based on message size:

| Path | Condition | Method |
|------|-----------|--------|
| **Bytes** | `sizeof(T) < 64 KB` | Serialize to `zenoh::Bytes`, publish |
| **SHM** | `sizeof(T) ≥ 64 KB` | Allocate from SHM pool, `memcpy` data, publish descriptor |

The SHM path is used for video frames (~6.2 MB) to avoid copying through the
network stack. The subscriber receives a zero-copy reference.

### ZenohSubscriber\<T\>

- Receives via Zenoh callback (runs on Zenoh internal thread)
- Stores latest value in a mutex + atomic cache
- `receive()` returns a copy of the cached value
- `is_connected()` returns true after first message received

### ZenohNetworkConfig

```cpp
struct ZenohNetworkConfig {
    std::string               mode;               // "peer", "client", "router"
    std::vector<std::string>  listen_endpoints;   // Drone: ["tcp/0.0.0.0:7447"]
    std::vector<std::string>  connect_endpoints;  // GCS: ["tcp/DRONE_IP:7447"]
    bool                      multicast_scouting;
    bool                      gossip_scouting;
};

// Presets:
ZenohNetworkConfig::make_drone(port, addr, proto);  // Peer + listener
ZenohNetworkConfig::make_gcs(drone_ip, port);       // Client + connect
ZenohNetworkConfig::make_local();                    // Peer, no network
```

---

## Wire Format

- **Header:** [`wire_format.h`](../common/ipc/include/ipc/wire_format.h)

Used for network transport. Every message is prefixed with a 32-byte header:

```cpp
struct __attribute__((packed)) WireHeader {
    uint32_t magic          = 0x4E4F5244;  // "DRON" little-endian
    uint8_t  version        = 3;
    uint8_t  flags          = 0;
    WireMessageType msg_type;
    uint32_t payload_size;
    uint64_t timestamp_ns;
    uint32_t sequence;
    uint64_t correlation_id;               // v2: cross-process trace ID
};
```

### Message Types

| Type | Value | IPC Channel |
|------|-------|-------------|
| `VIDEO_FRAME` | 1 | `/drone_mission_cam` |
| `STEREO_FRAME` | 2 | `/drone_stereo_cam` |
| `DETECTIONS` | 10 | `/detected_objects` |
| `SLAM_POSE` | 20 | `/slam_pose` |
| `MISSION_STATUS` | 30 | `/mission_status` |
| `TRAJECTORY_CMD` | 31 | `/trajectory_cmd` |
| `PAYLOAD_COMMAND` | 32 | `/payload_commands` |
| `FC_COMMAND` | 33 | `/fc_commands` |
| `FC_STATE` | 40 | `/fc_state` |
| `GCS_COMMAND` | 41 | `/gcs_commands` |
| `PAYLOAD_STATUS` | 50 | `/payload_status` |
| `SYSTEM_HEALTH` | 60 | `/system_health` |

### Helper Functions

```cpp
vector<uint8_t> wire_serialize(const T& msg, WireMessageType, seq, ts=0, corr_id=0);
bool wire_validate(const uint8_t* data, size_t len);
WireHeader wire_read_header(const uint8_t* data);
bool wire_deserialize<T>(const uint8_t* data, size_t len, T& out);
WireMessageType key_to_wire_type(const string& zenoh_key);
```

---

## Topic Mapping

SHM segment names map to Zenoh key expressions:

| SHM Segment | Zenoh Key |
|-------------|-----------|
| `/drone_mission_cam` | `drone/video/frame` |
| `/drone_stereo_cam` | `drone/video/stereo_frame` |
| `/drone_thermal_cam` | `drone/video/thermal_frame` |
| `/detected_objects` | `drone/perception/detections` |
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

---

## MessageBus Abstraction

- **Header:** [`message_bus.h`](../common/ipc/include/ipc/message_bus.h)

The `MessageBus` class wraps the Zenoh backend behind a type-erased interface:

```cpp
class MessageBus {
    // Internally: unique_ptr<ZenohMessageBus>

    template<typename T>
    unique_ptr<IPublisher<T>> advertise(const string& topic);

    template<typename T>
    unique_ptr<ISubscriber<T>> subscribe(const string& topic,
                                          int max_retries = 50,
                                          int retry_ms = 200);

    template<typename T>
    unique_ptr<ISubscriber<T>> subscribe_optional(const string& topic);

    string backend_name() const;       // "zenoh"
    bool has_service_channels() const; // true (Zenoh supports request-reply)
};
```

### Factory

```cpp
// Manual:
MessageBus create_message_bus(const string& backend = "zenoh",
                               size_t shm_pool_mb = 0,
                               const string& zenoh_json = "");

// Config-driven (reads ipc_backend, zenoh.* keys):
MessageBus create_message_bus(const Config& cfg);
```

The factory is the **only** place backend selection happens. Process code never
references Zenoh directly — it only uses the `IPublisher`/`ISubscriber` interfaces.

---

## Service Channels

Request-reply channels for synchronous operations (built on Zenoh queryables).

### Envelope Types

```cpp
template<typename T>
struct ServiceEnvelope {
    uint64_t correlation_id;
    uint64_t timestamp_ns;
    bool     valid;
    T        payload;
};

enum class ServiceStatus { OK = 0, REJECTED = 1, TIMEOUT = 2, ERROR = 3 };

template<typename T>
struct ServiceResponse {
    uint64_t      correlation_id;
    uint64_t      timestamp_ns;
    ServiceStatus status;
    bool          valid;
    T             payload;
};
```

### Client / Server Interfaces

```cpp
template<typename Req, typename Resp>
class IServiceClient {
    virtual uint64_t send_request(const Req& req) = 0;
    virtual optional<ServiceResponse<Resp>> poll_response(uint64_t corr_id) = 0;
    virtual optional<ServiceResponse<Resp>> await_response(uint64_t corr_id,
                                                            int timeout_ms) = 0;
};

template<typename Req, typename Resp>
class IServiceServer {
    virtual optional<ServiceEnvelope<Req>> poll_request() = 0;
    virtual void send_response(uint64_t corr_id, ServiceStatus, const Resp& resp) = 0;
};
```

Service channels are created via `MessageBus::create_client()` / `create_server()`.

---

## Liveliness Monitoring

- **Header:** [`zenoh_liveliness.h`](../common/ipc/include/ipc/zenoh_liveliness.h)

### LivelinessToken (RAII)

Each process declares a Zenoh liveliness token on startup:
```
Key: "drone/alive/{process_name}"
```

Auto-dropped on exit or crash — the Zenoh router detects the drop and fires callbacks.

### LivelinessMonitor

```cpp
class LivelinessMonitor {
    LivelinessMonitor(OnAliveCallback, OnDeathCallback);
    vector<string> get_alive_processes() const;
    bool is_alive(const string& process_name) const;
};
```

Subscribes to `drone/alive/**` wildcard. Used by P7 for process health detection.

---

## Configuration Reference

### `config/default.json`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `ipc_backend` | string | `"zenoh"` | Transport backend (Zenoh is the sole backend) |
| `zenoh.shm_pool_size_mb` | int | 32 | SHM pool for zero-copy (video frames) |
| `zenoh.network.mode` | string | `"peer"` | Zenoh session mode |
| `zenoh.network.listen` | string[] | `["tcp/0.0.0.0:7447"]` | Listen endpoints |
| `zenoh.network.connect` | string[] | `[]` | Connect endpoints (for GCS client) |
| `zenoh.network.multicast_scouting` | bool | true | Enable multicast discovery |
| `zenoh.network.gossip_scouting` | bool | true | Enable gossip discovery |

### Key Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| SHM publish threshold | 64 KB | Messages above this use Zenoh SHM path |
| Default SHM pool | 32 MB | Zenoh SHM pool for zero-copy frames |
| Max detected objects | 64 | Objects per detection list |
| Max tracked processes | 8 | 7 processes + FC in health |
| Max tracked threads | 16 | Threads per process in health |
| Max upload waypoints | 32 | Mid-flight mission upload limit |
| Zenoh listen port | 7447 | Default drone TCP/UDP listener |

---

## Testing

| Test File | Tests | Coverage |
|-----------|-------|----------|
| [`test_shm_ipc.cpp`](../tests/test_shm_ipc.cpp) | ~15 | SeqLock write/read, concurrent stress (10K), real types |
| [`test_message_bus.cpp`](../tests/test_message_bus.cpp) | 17 | MessageBus factory, publisher/subscriber, service stubs |
| [`test_spsc_ring.cpp`](../tests/test_spsc_ring.cpp) | 7 | Push/pop, wrap-around, concurrent 100K messages |
| [`test_zenoh_ipc.cpp`](../tests/test_zenoh_ipc.cpp) | 70 | Topic mapping, round-trip, SHM zero-copy, 200 Hz stress |
| [`test_zenoh_liveliness.cpp`](../tests/test_zenoh_liveliness.cpp) | 28 | Token lifecycle, monitor alive/death, multi-token |
| **Total** | **~137** | |

### Key Test Scenarios

- **SeqLock consistency:** 10K concurrent writes/reads with no torn reads
- **Topic mapping round-trip:** All 12+ channels validated SHM ↔ Zenoh key mapping
- **Zero-copy video:** 6.2 MB `ShmVideoFrame` published via Zenoh SHM path,
  verified `shm_publish_count > 0`
- **High-rate throughput:** 400 `ShmPose` messages at 200 Hz, all received
- **Service channels:** Request-reply round-trip with correlation ID tracking
- **Liveliness:** Token declare → monitor detects alive → token drop → monitor detects death

---

## Known Limitations

1. **Latest-value only:** Zenoh overwrites old messages. No queue or
   guaranteed delivery. Streams with burst losses (perception at high FPS) can
   miss detections.
2. **No message ordering guarantees:** With Zenoh over network, messages on
   different topics may arrive out of order relative to each other.
3. **Single Zenoh session per process:** The singleton design prevents a process
   from having multiple sessions (e.g., isolated Zenoh domains).
4. **No flow control:** Publishers are not throttled by slow subscribers. A slow
   P2 consumer receiving 30 fps 6.2 MB frames generates ~180 MB/s of copies.
5. **Zenoh SHM requires `shared-memory` Cargo feature:** The pre-built CI debs
   lack this feature, so `PosixShmProvider` returns `nullptr` on CI. SHM-path
   tests use `GTEST_SKIP()`. Locally-built zenohc works correctly.
