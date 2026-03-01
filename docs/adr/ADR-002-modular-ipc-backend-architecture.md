# ADR-002: Modular IPC Backend Architecture

| Field | Value |
|-------|-------|
| **Status** | Accepted |
| **Date** | 2026-03-01 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | ADR-001 (IPC Framework Selection), Epic #45, Issue #49 |

---

## 1. Context

Following the Zenoh IPC migration (Epic #45, Phases A–D), the stack now has
two IPC backends:

| Backend | Pub/Sub | Service Channels | Network Transport |
|---------|---------|-------------------|-------------------|
| **POSIX SHM** (`ShmMessageBus`) | ✅ SeqLock-based | ❌ Removed (Phase D) | ❌ Local only |
| **Zenoh** (`ZenohMessageBus`) | ✅ Zero-copy SHM + bytes | ✅ Queryable pattern | ✅ UDP/TCP/QUIC/TLS |

The architecture must remain **modular** so that:

1. The stack builds and runs with **or without** Zenoh (fallback to SHM pub/sub).
2. A new middleware (DDS, gRPC, iceoryx2, custom) can replace Zenoh without
   rewriting process code.
3. Process code is **backend-agnostic** — it only depends on abstract interfaces.

### Decision Drivers

- **Portability** — not all target platforms may have Zenoh packages available.
- **Testability** — SHM backend has zero external dependencies; ideal for unit tests.
- **Future-proofing** — the robotics middleware landscape evolves rapidly.
- **Migration cost** — must be minimal when swapping backends.

---

## 2. Decision

### 2.1 Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Layer 3: Factory (selects backend at build/run time)   │
│  message_bus_factory.h → MessageBusVariant + helpers    │
├─────────────────────────────────────────────────────────┤
│  Layer 2: Concrete Backends (each implements Layer 1)   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  SHM Backend  │  │ Zenoh Backend │  │ Future (DDS) │  │
│  │  ShmPublisher │  │ ZenohPublisher│  │ DdsPublisher │  │
│  │  ShmSubscriber│  │ ZenohSubscr.  │  │ DdsSubscr.   │  │
│  │  (no services)│  │ ZenohSvcCli.  │  │ DdsSvcCli.   │  │
│  │              │  │ ZenohSvcSrv.  │  │ DdsSvcSrv.   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Abstract Interfaces (backend-agnostic)        │
│  IPublisher<T>  ISubscriber<T>                          │
│  IServiceClient<Req,Resp>  IServiceServer<Req,Resp>     │
│  ServiceEnvelope<T>  ServiceResponse<T>  ServiceStatus  │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Key Rules

1. **Process code MUST only depend on Layer 1 interfaces** (`IPublisher`,
   `ISubscriber`, `IServiceClient`, `IServiceServer`) and the factory helpers
   (`bus_advertise`, `bus_subscribe`, `bus_create_client`, `bus_create_server`).

2. **No process code may include backend-specific headers** (e.g.,
   `zenoh_publisher.h`, `shm_writer.h`).

3. **Backend selection** is controlled by:
   - **Build time**: CMake option `ENABLE_ZENOH` defines `HAVE_ZENOH`.
   - **Run time**: `create_message_bus("zenoh")` or `create_message_bus("shm")`,
     typically driven by `config/default.json`.

4. **SHM backend is always available** as the zero-dependency fallback.
   Service channel helpers return `nullptr` on the SHM backend (with a warning).

5. **Only `shm_service_channel.h` is removed** (Phase D). The SHM pub/sub
   files (`shm_writer.h`, `shm_reader.h`, `shm_publisher.h`,
   `shm_subscriber.h`, `shm_message_bus.h`) are retained as the fallback
   backend.

6. **Internal thread-level constructs** (`SPSCRing`, `PoseDoubleBuffer`)
   are NOT IPC — they are process-internal synchronisation primitives and
   remain in place regardless of backend selection.

### 2.3 MessageBusVariant

The factory uses `std::variant<...>` rather than virtual dispatch because
`ShmMessageBus` and `ZenohMessageBus` are template factories — they cannot
share a common vtable. The variant pattern provides:

- Compile-time type safety (no casts).
- Zero runtime overhead (visit is branch-predicted).
- Easy extension (add another alternative to the variant).

```cpp
using MessageBusVariant = std::variant<
    std::unique_ptr<ShmMessageBus>
#ifdef HAVE_ZENOH
    , std::unique_ptr<ZenohMessageBus>
#endif
#ifdef HAVE_DDS
    , std::unique_ptr<DdsMessageBus>
#endif
>;
```

---

## 3. How to Add a New Middleware Backend

Follow these steps to replace Zenoh or add a new backend (e.g., DDS, gRPC,
iceoryx2):

### Step 1: Implement the Interfaces

Create these files in `common/ipc/include/ipc/`:

| File | Class | Implements |
|------|-------|-----------|
| `dds_publisher.h` | `DdsPublisher<T>` | `IPublisher<T>` |
| `dds_subscriber.h` | `DdsSubscriber<T>` | `ISubscriber<T>` |
| `dds_service_client.h` | `DdsServiceClient<Req,Resp>` | `IServiceClient<Req,Resp>` |
| `dds_service_server.h` | `DdsServiceServer<Req,Resp>` | `IServiceServer<Req,Resp>` |
| `dds_message_bus.h` | `DdsMessageBus` | Factory: `advertise()`, `subscribe()`, `create_client()`, `create_server()` |

Each implementation must:

- Accept trivially-copyable `T` (via `static_assert<std::is_trivially_copyable_v<T>>`).
- Serialise/deserialise using `memcpy` + raw bytes (same pattern as Zenoh).
- Provide `publish()`, `receive()`, `send_request()`, `poll_response()`,
  `poll_request()`, `send_response()` matching the interface signatures.

**Reference implementation:** Use `zenoh_publisher.h` / `zenoh_service_client.h`
as templates — they show the exact pattern.

### Step 2: Add CMake Option

In the root `CMakeLists.txt`:

```cmake
option(ENABLE_DDS "Build with DDS IPC backend" OFF)

if(ENABLE_DDS)
    find_package(CycloneDDS REQUIRED)
    add_compile_definitions(HAVE_DDS=1)
endif()
```

### Step 3: Register in the Factory

In `message_bus_factory.h`:

```cpp
#ifdef HAVE_DDS
#include "ipc/dds_message_bus.h"
#endif

using MessageBusVariant = std::variant<
    std::unique_ptr<ShmMessageBus>
#ifdef HAVE_ZENOH
    , std::unique_ptr<ZenohMessageBus>
#endif
#ifdef HAVE_DDS
    , std::unique_ptr<DdsMessageBus>
#endif
>;
```

Add a case in `create_message_bus()`:

```cpp
#ifdef HAVE_DDS
if (backend == "dds") {
    return std::make_unique<DdsMessageBus>();
}
#endif
```

The existing `bus_advertise()`, `bus_subscribe()`, `bus_create_client()`, and
`bus_create_server()` helpers use `std::visit` — they will automatically
dispatch to the new backend's methods via structural typing (duck typing
on the template factory's `advertise<T>()`, `subscribe<T>()`, etc.).

### Step 4: Add Tests

Create a test category in `tests/test_zenoh_ipc.cpp` (or a new file
`tests/test_dds_ipc.cpp`) with the same structure:

1. Backend-specific unit tests (under `#ifdef HAVE_DDS`)
2. Factory integration tests (always compiled — verify SHM fallback)

### Step 5: Update CI

In `.github/workflows/ci.yml`, add a matrix entry:

```yaml
strategy:
  matrix:
    backend: [shm, zenoh, dds]
```

### Step 6: Update Config

In `config/default.json`, add the backend option:

```json
{
  "ipc_backend": "dds",
  "dds": {
    "domain_id": 42
  }
}
```

### Step 7: Update Documentation

- Update this ADR's backend comparison table.
- Add backend-specific notes to `DEVELOPMENT_WORKFLOW.md`.
- If the new backend has a CI issue, document it in `CI_ISSUES.md`.

---

## 4. Consequences

### Positive

- **Zero lock-in** — any middleware can be swapped by implementing 5 files +
  1 factory registration. Process code does not change.
- **Graceful degradation** — without Zenoh (or any other middleware), the
  stack still works with SHM pub/sub. Service channels are unavailable but
  the system doesn't crash (factory returns `nullptr`).
- **Test isolation** — SHM tests run without external dependencies; Zenoh
  tests are compile-guarded and only run when `ENABLE_ZENOH=ON`.
- **Incremental adoption** — new backends can be added alongside existing
  ones; processes can be migrated one at a time.

### Negative

- **Variant boilerplate** — adding a new variant alternative requires
  touching `MessageBusVariant` and the factory. This is minimal (~5 lines).
- **Feature parity not enforced** — the SHM backend doesn't support service
  channels, so callers must handle `nullptr` returns. This is documented
  and tested.
- **Trivially-copyable constraint** — all message types must be
  `std::is_trivially_copyable_v`, which prevents using `std::string`,
  `std::vector`, etc. in messages. This is intentional (performance) and
  applies to all backends equally.

### Risks

- **Backend-specific features** — some features (e.g., Zenoh's wildcard
  subscriptions, liveliness tokens) don't have equivalents on all backends.
  These should be accessed through backend-specific extension points, not
  through the generic interfaces.
- **Serialisation divergence** — if a future backend requires protobuf/CDR
  serialisation for network transport, the raw-bytes approach may need a
  serialisation abstraction layer. This is expected for Phase E (network
  transport) and will be addressed in a separate ADR.

---

## 5. Current Backend Inventory

| File | Backend | Role | Status |
|------|---------|------|--------|
| `ipublisher.h` | — | Abstract interface | ✅ Stable |
| `isubscriber.h` | — | Abstract interface | ✅ Stable |
| `iservice_channel.h` | — | Abstract interface | ✅ Stable |
| `shm_types.h` | — | Data structures | ✅ Stable (shared) |
| `shm_writer.h` | SHM | Low-level writer | ✅ Kept (fallback) |
| `shm_reader.h` | SHM | Low-level reader | ✅ Kept (fallback) |
| `shm_publisher.h` | SHM | `IPublisher<T>` impl | ✅ Kept (fallback) |
| `shm_subscriber.h` | SHM | `ISubscriber<T>` impl | ✅ Kept (fallback) |
| `shm_message_bus.h` | SHM | Factory | ✅ Kept (fallback) |
| `shm_service_channel.h` | SHM | Service channels | ❌ **Removed** (Phase D) |
| `zenoh_session.h` | Zenoh | Singleton session | ✅ Active |
| `zenoh_publisher.h` | Zenoh | `IPublisher<T>` impl | ✅ Active |
| `zenoh_subscriber.h` | Zenoh | `ISubscriber<T>` impl | ✅ Active |
| `zenoh_service_client.h` | Zenoh | `IServiceClient` impl | ✅ New (Phase D) |
| `zenoh_service_server.h` | Zenoh | `IServiceServer` impl | ✅ New (Phase D) |
| `zenoh_message_bus.h` | Zenoh | Factory | ✅ Active |
| `message_bus_factory.h` | — | Backend selector | ✅ Active |
| `spsc_ring.h` | — | Thread-level queue | ✅ Stable (not IPC) |

---

## 6. Test Configuration

Tests use **one source tree, two build configurations**. The CI matrix runs
both; `#ifdef HAVE_ZENOH` compile guards control which tests are included
in each build.

### CI Matrix

```yaml
# .github/workflows/ci.yml
strategy:
  matrix:
    backend: [shm, zenoh]
```

- **`shm`** job: builds with `ENABLE_ZENOH=OFF` (no Zenoh dependency).
- **`zenoh`** job: builds with `-DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON`.

### What Runs in Each Configuration

#### `ENABLE_ZENOH=OFF` (SHM-only build)

| Test binary | Tests | Purpose |
|---|---|---|
| `test_message_bus` | SHM pub/sub (14) + service interface types (4) | Validate SHM fallback + interface contracts |
| `test_zenoh_ipc` | Factory fallback tests (7) | Verify SHM bus selected when Zenoh unavailable, `bus_create_client()` / `bus_create_server()` return `nullptr` |
| All other binaries | Unchanged | Process / algorithm tests (no IPC dependency) |

#### `ENABLE_ZENOH=ON` (Zenoh build)

| Test binary | Tests | Purpose |
|---|---|---|
| `test_message_bus` | Same 18 tests | SHM fallback still tested alongside Zenoh |
| `test_zenoh_ipc` | Topic mapping (15) + Factory (7) + Pub/sub round-trips (10+) + SHM provider (7) + **Service channels (12)** | Full Zenoh backend validation |
| All other binaries | Unchanged | Process / algorithm tests |

### How Compile Guards Work

All backend-specific tests are wrapped in `#ifdef HAVE_ZENOH`:

```cpp
// tests/test_zenoh_ipc.cpp

// Always compiled — validates factory fallback to SHM
TEST(MessageBusFactory, DefaultCreatesShmBus) { ... }
TEST(MessageBusFactory, ShmBusServiceClientReturnsNull) { ... }

#ifdef HAVE_ZENOH
// Only compiled when ENABLE_ZENOH=ON
TEST(ZenohServiceChannel, ClientSendServerReceive) { ... }
TEST(ZenohServiceChannel, FullRoundTrip) { ... }
// ... 10 more service channel tests
#endif
```

### Adding Tests for a New Backend

When adding a new backend (e.g., DDS), follow the same pattern:

1. Add backend-specific tests inside `#ifdef HAVE_DDS` in a new file
   `tests/test_dds_ipc.cpp` (or in `test_zenoh_ipc.cpp` if small).
2. Add always-compiled factory tests that verify `bus_create_client()`
   returns `nullptr` on SHM and the correct type on DDS.
3. Add a matrix entry in CI: `backend: [shm, zenoh, dds]`.
