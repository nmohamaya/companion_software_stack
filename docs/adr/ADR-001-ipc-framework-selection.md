# ADR-001: IPC Framework Selection — iceoryx2 vs Zenoh

| Field | Value |
|-------|-------|
| **Status** | Proposed |
| **Date** | 2026-02-28 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Target Hardware** | NVIDIA Jetson Orin (aarch64, 8–64 GB RAM, CUDA 12.x, JetPack 6.x) |

---

## 1. Context

The companion software stack uses a hand-rolled IPC layer built on POSIX shared memory (`shm_open` / `mmap`) with:

- **SeqLock-based `ShmWriter`/`ShmReader`** — version-counter protocol for torn-read detection (4 retries)
- **`ShmMessageBus`** — factory producing `IPublisher<T>` / `ISubscriber<T>` backed by `ShmPublisher` / `ShmSubscriber`
- **`ShmServiceChannel`** — request/response pattern via paired SHM blocks with correlation IDs
- **`PoseDoubleBuffer`** — atomic index swap for intra-process pose exchange
- **`SPSCRing<T, N>`** — lock-free single-producer/single-consumer ring buffer
- **12 named SHM segments** — centrally defined in `shm_types.h::shm_names`

### Current Pain Points

1. **Manual lifecycle management** — every launch script requires `clean_shm()` helpers, signal trapping, and `/dev/shm` cleanup. Process crashes leave orphaned segments.
2. **No process death detection** — if a publisher dies, subscribers read stale data indefinitely with no notification.
3. **Torn reads under load** — SeqLock with 4 retries can still fail at high update rates (100 Hz VIO + 30 Hz video).
4. **`memcpy` overhead** — `ShmVideoFrame` is ~6 MB; every `write()` copies the full frame into SHM under the SeqLock.
5. **No network transport** — the IPC layer is local-only. Future GCS telemetry (#34) and multi-vehicle (#35 video streaming) require separate networking code.
6. **No backpressure** — publishers always overwrite; slow consumers silently lose data.

### Future Roadmap Requirements

The IPC framework must support these planned features:

| Phase | Issue | IPC Requirement |
|-------|-------|-----------------|
| 9 | #28 Heartbeat timeout | Process liveness detection |
| 10 | #32 V4L2Camera | Zero-copy video frame sharing (~6 MB @ 30 Hz) |
| 10 | #34 UDPGCSLink | Network-transparent pub/sub to ground station |
| 10 | #35 Video streaming | High-bandwidth data path (H.265 frames) |
| 10 | #36 Jetson cross-compile | aarch64 build on NVIDIA Jetson Orin |
| 11 | #37 Real VIO | High-rate pose updates (100–200 Hz), low latency |
| 12 | #40 Flight data recorder | Subscribe to all channels for logging without impacting producers |
| 12 | #41 Contingency fault tree | Process health monitoring, crash detection |

### Target Hardware: NVIDIA Jetson Orin

| Spec | Value |
|------|-------|
| SoC | NVIDIA Jetson Orin (Nano/NX/AGX variants) |
| Architecture | aarch64 (ARMv8.2-A) |
| RAM | 8 GB (Nano) / 16 GB (NX) / 32–64 GB (AGX) |
| GPU | Ampere, 1024–2048 CUDA cores |
| JetPack | 6.x (Ubuntu 22.04 base, CUDA 12.x) |
| Storage | NVMe SSD (typical) |
| I/O | USB 3.2, UART, SPI, I2C, CSI camera |

The framework must build natively on aarch64 Linux and run efficiently within the Jetson's memory and thermal constraints.

---

## 2. Decision Drivers

Ranked by priority for this project:

1. **Zero-copy shared memory** — eliminate `memcpy` for 6 MB video frames at 30 Hz
2. **Process crash resilience** — automatic cleanup of resources on publisher/subscriber death
3. **aarch64 / Jetson Orin support** — native build, no emulation or cross-compile hacks
4. **C++ API quality** — idiomatic C++17, type-safe, integrates with existing `IPublisher`/`ISubscriber` interfaces
5. **Network transparency** — same API for local SHM and remote (drone↔GCS) communication
6. **Minimal dependencies** — small footprint for embedded, few transitive deps
7. **Maturity / community** — production deployments, active maintenance, documentation
8. **Migration cost** — effort to replace `ShmMessageBus` and `ShmServiceChannel`

---

## 3. Options Considered

### Option A: iceoryx2

**Overview:** Second generation of Eclipse iceoryx, rewritten in Rust with C/C++ bindings. True zero-copy inter-process communication using shared memory with a loan-based API. No central daemon (unlike iceoryx v1's RouDi).

| Aspect | Assessment |
|--------|------------|
| **Language** | Rust core, `iceoryx2-cxx` C++ bindings |
| **Zero-copy model** | Publisher "loans" a memory chunk from a shared pool → writes in-place → publishes. Subscriber receives a reference. No `memcpy` at any point. |
| **Pub/Sub** | Native, typed, configurable history depth |
| **Request/Response** | Native service pattern (client/server) |
| **Process death** | Heartbeat-based liveness; resources auto-reclaimed on crash |
| **Network transport** | **Not supported.** Local-only (shared memory). Network would require a separate bridge/gateway. |
| **aarch64 / Jetson** | Rust compiles to aarch64 natively. C++ bindings use `cbindgen`. Tested on ARM Linux. |
| **Daemon** | None required (unlike iceoryx v1). Peer-to-peer discovery via SHM. |
| **Memory model** | Pre-allocated shared memory pools. Publisher and subscriber agree on chunk size at creation. |
| **Build dependency** | Requires Rust toolchain (`rustup`) + `cargo` in addition to CMake. C++ bindings built via `cmake` wrapping `cargo`. |
| **Maturity** | Younger than v1. Production use in automotive (Eclipse Foundation). Active development by ZettaScale (2024–present). |
| **License** | Apache 2.0 / MIT dual |
| **Footprint** | Small — no heap allocation in the data path, suitable for embedded |
| **QoS** | Best-effort or reliable delivery via configurable policies |

**Strengths for this project:**
- True zero-copy eliminates the 6 MB `memcpy` for video frames — critical on Jetson Orin where memory bandwidth is shared with GPU
- No daemon simplifies deployment (no extra process in `launch_hardware.sh`)
- Process crash detection directly addresses #28 (heartbeat timeout) and #41 (fault tree)
- Small footprint suits Jetson Orin Nano (8 GB)

**Weaknesses for this project:**
- **No network transport** — #34 (GCS telemetry) and #35 (video streaming) would need a completely separate networking layer (e.g., raw UDP, gRPC, or another framework)
- Rust toolchain required on Jetson build environment — manageable but adds `~300 MB` to the toolchain
- C++ bindings are functional but less idiomatic than native C++ APIs — some boilerplate for type registration
- No built-in serialization — SHM types must remain trivially copyable (same constraint as current system)

### Option B: Zenoh

**Overview:** Eclipse Zenoh is a pub/sub/query protocol designed for robotics and edge computing. Rust core with first-class C and C++ bindings. Supports shared memory for local zero-copy and seamlessly extends to network transport (UDP/TCP/QUIC) with zero code change.

| Aspect | Assessment |
|--------|------------|
| **Language** | Rust core, `zenoh-cpp` C++17 bindings (header-only wrapper around C API) |
| **Zero-copy model** | SHM plugin: publisher allocates from a shared memory pool, writes in-place, publishes. Subscriber receives SHM reference for local peers. For remote peers, Zenoh serializes/deserializes automatically. |
| **Pub/Sub** | Native, key-expression based topics (e.g., `drone/slam/pose`), wildcard subscriptions |
| **Request/Response** | Native "queryable" pattern — client sends query, server responds. Supports get/put semantics. |
| **Process death** | Liveliness tokens — a subscriber can watch for publisher death via `liveliness().declare_token()`. Automatic SHM cleanup on process exit. |
| **Network transport** | **Built-in.** Same API works over: SHM (local), UDP multicast, TCP, TLS, QUIC, WebSocket. Drone↔GCS communication uses the exact same pub/sub code. |
| **aarch64 / Jetson** | First-class ARM support. Pre-built Debian packages for aarch64. Used extensively in ROS 2 Iron/Jazzy on Jetson platforms. |
| **Daemon** | Optional `zenohd` router for complex topologies. For local-only SHM, no daemon needed ("peer" mode). |
| **Memory model** | SHM provider with configurable pool sizes. Non-SHM path uses heap allocation with efficient internal buffers. |
| **Build dependency** | Rust toolchain for building from source. **However**, pre-built `libzenohc` binaries available for aarch64 — can use without Rust if consuming pre-built packages. `zenoh-cpp` is header-only C++17. |
| **Maturity** | Eclipse Foundation project. Backed by ZettaScale (same team that built Cyclone DDS and iceoryx). Used in production by: Autoware (autonomous driving), ROS 2 rmw_zenoh (default in Jazzy), multiple drone platforms. |
| **License** | Apache 2.0 / EPL 2.0 dual |
| **Footprint** | Moderate — `libzenohc.so` is ~5 MB. Larger than iceoryx2 but smaller than DDS. Runtime memory: ~2–4 MB baseline for router process; zero daemon mode uses ~500 KB. |
| **QoS** | Reliability (best-effort / reliable), congestion control, priority, bandwidth limiting |
| **Serialization** | Payload is opaque bytes. Can use raw `memcpy`/trivially-copyable (like now), or integrate protobuf/CDR/MessagePack for network transport. |

**Strengths for this project:**
- **Location transparency** — the single biggest advantage. `pub.put(pose_data)` works identically over SHM (local, ~1 μs) and over UDP to a ground station (network). This directly solves #34 (GCS telemetry) and #35 (video streaming) without a separate networking layer.
- Zero-copy SHM for local IPC — same loan-based model as iceoryx2, eliminates `memcpy` for video frames
- **Liveliness tokens** — built-in process health monitoring. When a publisher dies, subscribers get a callback. Directly addresses #28 and #41.
- Wildcard subscriptions (`drone/*/status`) — the flight data recorder (#40) can subscribe to `drone/**` and capture all channels without knowing them at compile time
- **Pre-built aarch64 packages** — no Rust toolchain needed on the Jetson itself; consume `libzenohc` as a pre-built `.deb` or `.so`
- **ROS 2 alignment** — `rmw_zenoh` is the default RMW in ROS 2 Jazzy. If the project ever integrates with ROS 2 nodes, Zenoh is already the transport.
- Active community — ZettaScale team (same people behind Cyclone DDS and iceoryx) provides commercial support and rapid issue response

**Weaknesses for this project:**
- Slightly higher baseline memory than iceoryx2 (~5 MB library vs ~1 MB) — not a concern on Jetson Orin (8+ GB)
- C++ bindings are a header-only wrapper around the C API — more ergonomic than iceoryx2's `cbindgen` approach but still not "native" C++ feel
- SHM zero-copy requires the SHM plugin to be enabled and configured — small setup overhead
- Serialization for network transport is the user's responsibility — need to define a wire format for GCS messages (trivially-copyable works for SHM, but network needs serialization)

---

## 4. Detailed Comparison

### 4.1 Performance (Local SHM Path)

| Metric | iceoryx2 | Zenoh (SHM mode) | Current (SeqLock) |
|--------|----------|-------------------|-------------------|
| Latency (small msg) | ~100–300 ns | ~500 ns–1 μs | ~200 ns (no contention) |
| Latency (6 MB video) | ~200 ns (zero-copy pointer) | ~500 ns (zero-copy pointer) | **~2 ms** (`memcpy` 6 MB) |
| Throughput (small msg) | ~15 M msg/s | ~8 M msg/s | ~10 M msg/s |
| CPU overhead | Minimal | Minimal + routing logic | Minimal |
| Memory copies | **0** | **0** (SHM path) | **1** per publish |

Both iceoryx2 and Zenoh eliminate the `memcpy` bottleneck for video frames. iceoryx2 has slightly lower latency for small messages, but the difference (~500 ns) is irrelevant at our 10–100 Hz control rates.

### 4.2 Network Capability

| Capability | iceoryx2 | Zenoh |
|------------|----------|-------|
| Local SHM | ✅ | ✅ |
| UDP unicast | ❌ | ✅ |
| UDP multicast | ❌ | ✅ |
| TCP | ❌ | ✅ |
| TLS / QUIC | ❌ | ✅ |
| WebSocket | ❌ | ✅ |
| Drone ↔ GCS | Needs separate implementation | Same API as local pub/sub |
| Multi-vehicle mesh | Not supported | ✅ (peer-to-peer or routed) |

**This is the decisive differentiator.** Our roadmap includes:
- **#34 UDPGCSLink** — telemetry to ground station
- **#35 Video streaming** — H.265 frames to GCS
- Future multi-drone coordination

With Zenoh, these are configuration changes (add a network endpoint), not code changes. With iceoryx2, we'd need to build and maintain a separate networking layer.

### 4.3 Jetson Orin Compatibility

| Factor | iceoryx2 | Zenoh |
|--------|----------|-------|
| aarch64 native build | ✅ (Rust cross-compile) | ✅ (Rust cross-compile or pre-built `.deb`) |
| Pre-built aarch64 binaries | Limited | ✅ (`libzenohc` packages for aarch64) |
| Rust toolchain on Jetson | Required for from-source build | Optional (pre-built C library available) |
| JetPack 6.x tested | Community reports | ✅ (ROS 2 Jazzy/Zenoh on Jetson is a documented setup) |
| CUDA interop | N/A (CPU-only IPC) | N/A (CPU-only IPC) |
| Thermal impact | Negligible | Negligible |
| Memory overhead | ~1 MB | ~5 MB |

Both work on Jetson Orin. Zenoh has an edge with pre-built packages and ROS 2/Jetson community adoption.

### 4.4 Migration Effort

Our existing `IPublisher<T>` / `ISubscriber<T>` / `IServiceClient` / `IServiceServer` interfaces make migration tractable. The `ShmMessageBus` factory pattern means process code depends on interfaces, not implementations.

| Migration Task | iceoryx2 | Zenoh |
|----------------|----------|-------|
| New `MessageBus` backend | `Iceoryx2MessageBus` | `ZenohMessageBus` |
| Publisher impl | `Iceoryx2Publisher<T>` wrapping `iox2::Publisher` | `ZenohPublisher<T>` wrapping `zenoh::Publisher` |
| Subscriber impl | `Iceoryx2Subscriber<T>` wrapping `iox2::Subscriber` | `ZenohSubscriber<T>` wrapping `zenoh::Subscriber` |
| Service channel | `Iceoryx2ServiceChannel` wrapping `iox2::Server/Client` | `ZenohServiceChannel` wrapping `zenoh::Queryable/Query` |
| Topic naming | Map `/drone_*` SHM names to iceoryx2 service names | Map `/drone_*` to Zenoh key expressions: `drone/slam/pose` |
| Type constraint | Still trivially-copyable (SHM requirement) | Trivially-copyable for SHM; serializable for network |
| Test migration | ~60 IPC tests to update | ~60 IPC tests to update |
| Process `main.cpp` changes | Bus type swap only (4 processes use `ShmMessageBus`) | Bus type swap only |
| CMake integration | `find_package(iceoryx2)` + Rust build step | `find_package(zenohc)` + link `libzenohc` |
| Estimated effort | ~3–4 days | ~3–4 days |

Effort is comparable. Zenoh has a slight edge because:
- Pre-built `libzenohc` avoids Rust build integration in CMake
- Network transport for #34/#35 comes "for free" — no additional implementation

### 4.5 Roadmap Alignment

| Roadmap Item | iceoryx2 | Zenoh | Winner |
|-------------|----------|-------|--------|
| #28 Heartbeat timeout | ✅ Built-in | ✅ Liveliness tokens | Tie |
| #32 V4L2 zero-copy video | ✅ SHM loan | ✅ SHM loan | Tie |
| #34 GCS telemetry | ❌ Separate impl needed | ✅ Same API over UDP/TCP | **Zenoh** |
| #35 Video streaming | ❌ Separate impl needed | ✅ Same API over network | **Zenoh** |
| #36 Jetson cross-compile | ✅ Works | ✅ Pre-built packages | **Zenoh** |
| #40 Flight data recorder | ✅ Subscribe to all topics | ✅ Wildcard `drone/**` | **Zenoh** |
| #41 Fault tree | ✅ Heartbeat | ✅ Liveliness + network health | **Zenoh** |
| Multi-drone (future) | ❌ Not supported | ✅ Peer-to-peer mesh | **Zenoh** |

---

## 5. Decision

**Selected: Zenoh**

### Rationale

1. **Network transparency is non-negotiable for our roadmap.** Issues #34 (GCS telemetry), #35 (video streaming), and future multi-drone coordination all require the IPC layer to work across the network. Zenoh provides this with zero code change — the same `pub.put()` call works over SHM locally and over UDP/TCP to a ground station. With iceoryx2, we would need to design, implement, and maintain a separate networking layer, effectively building two communication systems.

2. **Zero-copy SHM performance is equivalent.** Both frameworks offer loan-based zero-copy shared memory. The ~500 ns latency difference is irrelevant at our 10–100 Hz control rates. The critical improvement — eliminating the 6 MB `memcpy` for video frames — is achieved by both.

3. **Jetson Orin deployment is smoother.** Pre-built `libzenohc` aarch64 packages mean we don't need the Rust toolchain on the Jetson itself. The ROS 2 Jazzy ecosystem already validates Zenoh on Jetson extensively.

4. **Wildcard subscriptions unlock the flight data recorder.** Issue #40 requires subscribing to all IPC channels for logging. Zenoh's key-expression wildcards (`drone/**`) handle this elegantly without compile-time knowledge of all topics.

5. **Process liveness detection via liveliness tokens** maps directly to our #28 (heartbeat timeout) and #41 (contingency fault tree) requirements, with network-aware health monitoring as a bonus.

6. **Same team, same pedigree.** ZettaScale (the company behind Zenoh) also created iceoryx and Cyclone DDS. Zenoh is their "next generation" answer that unifies local zero-copy IPC with network transport — it is the convergence of iceoryx's SHM model and Cyclone's networking.

### Trade-offs Accepted

- **~5 MB library footprint** vs iceoryx2's ~1 MB. Acceptable on Jetson Orin (8+ GB RAM).
- **C++ bindings are a wrapper over C API**, not native C++. Mitigated by our `IPublisher`/`ISubscriber` interface layer — process code never touches Zenoh directly.
- **Serialization for network path** is our responsibility. We'll use trivially-copyable `memcpy` for SHM (same as now) and add lightweight serialization (e.g., protobuf or packed structs) only for the network path to GCS.

---

## 6. Implementation Plan

### Phase A — Foundation (New Issue)
- Add `zenohc` as optional CMake dependency (`find_package(zenohc QUIET)`)
- Compile guard: `HAVE_ZENOH`
- Create `ZenohMessageBus`, `ZenohPublisher<T>`, `ZenohSubscriber<T>` implementing existing `IPublisher` / `ISubscriber` interfaces
- Config-driven backend selection: `"ipc_backend": "zenoh"` (default remains `"shm"` for backward compat)
- Unit tests for the new backend
- CI: build with and without Zenoh

### Phase B — Low-Bandwidth Channel Migration
- Migrate `ShmPose`, `ShmMissionStatus`, `ShmFCState`, `ShmFCCommand`, `ShmGCSCommand` to Zenoh pub/sub
- Validate correctness against existing SeqLock tests
- Key-expression scheme: `drone/{process}/{topic}` (e.g., `drone/slam/pose`, `drone/comms/fc_state`)

### Phase C — High-Bandwidth Channel Migration
- Migrate `ShmVideoFrame` and `ShmStereoFrame` using Zenoh SHM provider
- Validate zero-copy with performance benchmark (latency, CPU, memory bandwidth)
- Target: < 1 μs publish latency for 6 MB frames on Jetson Orin

### Phase D — Service Channel Migration
- Replace `ShmServiceChannel` with Zenoh queryable pattern
- Migrate `IServiceClient` / `IServiceServer` implementations
- Remove old SeqLock SHM primitives (`ShmWriter`, `ShmReader`)

### Phase E — Network Transport (aligns with #34, #35)
- Enable Zenoh network transport to ground station
- Same publishers, now reachable over UDP/TCP from GCS
- Add serialization layer for network-bound messages

---

## 7. Consequences

### Positive
- Eliminates all manual SHM lifecycle management (`clean_shm()`, signal traps, `/dev/shm` cleanup)
- True zero-copy for video frames — saves ~180 MB/s of memory bandwidth (6 MB × 30 Hz) on Jetson Orin
- Process crash detection and automatic resource cleanup
- GCS telemetry and video streaming use the same IPC API — no separate networking code
- Wildcard subscriptions for flight data recording
- ROS 2 ecosystem compatibility via `rmw_zenoh`

### Negative
- New external dependency (~5 MB shared library)
- Team must learn Zenoh concepts (sessions, key expressions, SHM providers)
- Network serialization format must be chosen and maintained
- Pre-built packages tie us to ZettaScale's release cadence for aarch64

### Neutral
- `IPublisher`/`ISubscriber` interface layer shields process code from the transport change
- Existing SHM types (`ShmVideoFrame`, `ShmPose`, etc.) remain unchanged — they're still trivially-copyable structs
- Test count will increase (new backend tests) but existing behavioral tests remain valid

---

## 8. References

- [Zenoh documentation](https://zenoh.io/docs/)
- [zenoh-cpp C++ bindings](https://github.com/eclipse-zenoh/zenoh-cpp)
- [iceoryx2 documentation](https://iceoryx.io/v2/)
- [iceoryx2-cxx C++ bindings](https://github.com/eclipse-iceoryx/iceoryx2)
- [Zenoh SHM zero-copy](https://zenoh.io/blog/2024-01-shared-memory/)
- [rmw_zenoh — ROS 2 Zenoh middleware](https://github.com/ros2/rmw_zenoh)
- [ZettaScale (company behind Zenoh, iceoryx, Cyclone DDS)](https://zettascale.tech/)
- Companion stack IPC interfaces: `common/ipc/include/ipc/ipublisher.h`, `isubscriber.h`, `iservice_channel.h`
- Companion stack SHM types: `common/ipc/include/ipc/shm_types.h`
