# ADR-003: C++17 Language Standard

| Field | Value |
|-------|-------|
| **Status** | Accepted |
| **Date** | 2026-03-02 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Related** | ADR-001 (IPC Framework Selection), ADR-002 (Modular IPC Backend Architecture) |

---

## 1. Context

The companion software stack targets the **NVIDIA Jetson Orin Nano** as the
primary deployment platform.  The language standard chosen must:

1. Be **fully supported** by the compiler toolchain shipped with the target OS.
2. Be **compatible** with all third-party dependencies (MAVSDK, gz-transport13,
   Zenoh C++ bindings, Google Test, spdlog).
3. Provide the **modern C++ features** the codebase relies on.

### Target Platform Constraints

| Spec | Value |
|------|-------|
| **Hardware** | NVIDIA Jetson Orin Nano (aarch64) |
| **OS** | JetPack 6.x (Ubuntu 22.04 base) |
| **System compiler** | GCC 11.4 |
| **Alternate compiler** | GCC 12 (available via `apt`, not default) |

### C++ Standard Support in GCC

| Standard | GCC 11 (system default) | GCC 12 | GCC 13 |
|----------|------------------------|--------|--------|
| C++17 | ✅ Full | ✅ Full | ✅ Full |
| C++20 | ⚠️ Partial (missing modules, coroutines bugs) | ✅ Mostly complete | ✅ Full |
| C++23 | ❌ Minimal | ⚠️ Partial | ⚠️ Partial |

---

## 2. Decision

**Use C++17** (`CMAKE_CXX_STANDARD 17`, `CMAKE_CXX_STANDARD_REQUIRED ON`).

Set in the root `CMakeLists.txt`:

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

## 3. Decision Drivers

### 3.1 Toolchain Availability on Target Hardware

The Jetson Orin Nano runs JetPack 6.x (Ubuntu 22.04), which ships **GCC 11**
as the system default.  C++17 is the newest standard with **full, stable
support** in GCC 11.  C++20 has known gaps in GCC 11 (incomplete `<format>`,
coroutine bugs, no module support), and C++23 is barely available.

Requiring GCC 12+ would mean either:
- Manual toolchain installation on every Jetson — fragile and unsupported by
  NVIDIA's JetPack ecosystem.
- Cross-compilation from a newer host — adds CI/deployment complexity.

Neither is justified when C++17 meets all functional requirements.

### 3.2 Dependency Compatibility

All third-party libraries build cleanly against C++17:

| Dependency | Minimum Standard | Notes |
|------------|-----------------|-------|
| **Zenoh-cpp** (0.11.x) | C++17 | Uses `std::variant`, `std::optional` |
| **MAVSDK** (2.x) | C++17 | Officially targets C++17 |
| **gz-transport13** | C++17 | Gazebo Harmonic ecosystem |
| **Google Test** (1.14) | C++14 | Compatible with C++17 |
| **spdlog** (1.x) | C++11 | Compatible with C++17 |
| **nlohmann/json** | C++11 | Compatible with C++17 |

Bumping to C++20 would risk **ABI mismatches** if any dependency is
pre-compiled against C++17, or require rebuilding all dependencies from source
on the Jetson — an expensive proposition on an embedded board.

### 3.3 C++17 Features Used in the Codebase

The codebase makes extensive use of C++17 features (see `CPP_PATTERNS_GUIDE.md`
for detailed examples):

| Feature | Where Used |
|---------|-----------|
| `std::variant` + `std::visit` | `MessageBusVariant` (IPC backend dispatch) |
| `std::optional` | `SPSCRing::try_pop()`, `ZenohSession` lazy init, service responses |
| `if constexpr` | `ZenohPublisher` SHM/bytes routing, service channel type dispatch |
| `static_assert` with message | `ShmWriter`, `ShmReader`, `wire_serialize` (trivially-copyable checks) |
| `std::string_view` | `LivelinessToken` key parsing, Zenoh key expressions |
| `[[maybe_unused]]`, `[[nodiscard]]` | Signal handler, factory functions |
| Inline variables | `SignalHandler::s_running_` static inline member |

---

## 4. Alternatives Considered

### 4.1 C++20

**Pros:**
- `std::jthread` + `std::stop_token` — cooperative thread cancellation (replaces
  `std::thread` + `std::atomic<bool>` shutdown flag pattern used throughout).
- `concepts` / `requires` — stronger compile-time interface contracts (replaces
  `static_assert` + type traits).
- `std::format` — type-safe formatting (but spdlog already provides this via
  `fmt`).
- `std::counting_semaphore`, `std::latch`, `std::barrier` — richer
  synchronization primitives.
-  Coroutines — potential for async I/O without threads.
- Three-way comparison (`<=>`) — simplifies ordering operators.

**Cons:**
- **Not fully supported on system GCC 11** (Jetson default).  Modules are
  missing, coroutines have known bugs, `<format>` is incomplete.
- **ABI risk** with pre-built dependencies.
- **No critical feature gap** — every C++20 feature listed above has a
  working C++17 equivalent already in the codebase.

**Decision:** Rejected.  The benefits are ergonomic, not functional.  The
toolchain risk on the Jetson outweighs the convenience.

### 4.2 C++23

**Pros:**
- `std::expected` — cleaner error handling than `std::optional` + error codes.
- `std::print` / `std::println` — modern I/O (but spdlog handles all logging).
- `std::mdspan` — multidimensional array views (useful for image processing).
- Deducing `this` — cleaner CRTP-like patterns.
- `std::generator` — lazy ranges with coroutines.

**Cons:**
- **Requires GCC 13+** — not available on JetPack 6.x without manual
  toolchain installation.
- **Library support is still incomplete** even in GCC 14.
- **No dependency has adopted C++23** as a minimum requirement.

**Decision:** Rejected.  Too bleeding-edge for an embedded deployment target.

### 4.3 C++14

**Pros:**
- Maximum compiler compatibility.

**Cons:**
- Missing `std::variant`, `std::optional`, `if constexpr`, `std::string_view`,
  and inline variables — all of which are heavily used in the codebase.
- Would require significant refactoring or Boost polyfills.

**Decision:** Rejected.  C++14 lacks too many features the codebase depends on.

---

## 5. Consequences

### Positive

- **Zero toolchain friction** — builds out-of-the-box on JetPack 6.x with the
  system GCC.
- **Full dependency compatibility** — no ABI mismatches, no rebuilds.
- **Rich feature set** — `std::variant`, `std::optional`, `if constexpr`, and
  `static_assert` enable the modular IPC architecture (ADR-002) without
  external dependencies.
- **Future upgrade path** — when JetPack moves to Ubuntu 24.04 (GCC 13+),
  upgrading to C++20 is straightforward.  No architectural changes needed.

### Negative

- **Manual thread cancellation** — `std::thread` + `std::atomic<bool>` instead
  of `std::jthread` + `stop_token`.  More boilerplate, but well-understood.
- **`static_assert` instead of `concepts`** — error messages are less precise
  (shows assertion failure, not "type X does not satisfy concept Y").
- **No coroutines** — async patterns must use threads + condition variables.

### Neutral

- **`spdlog` / `fmt` cover `std::format`** — no loss from missing C++20
  `<format>`.
- **Lock-free designs** — `SeqLock`, `SPSCRing`, `PoseDoubleBuffer` don't
  benefit from C++20 synchronization primitives (`latch`, `barrier`, etc.)
  since they're single-producer/single-consumer.

---

## 6. Review Triggers

Re-evaluate this decision when:

1. **JetPack ships GCC 13+** as the default system compiler (likely JetPack 7.x
   on Ubuntu 24.04).
2. **A dependency requires C++20** as its minimum standard.
3. **Coroutines become necessary** for an async networking layer.
4. **`std::expected`** would eliminate a meaningful class of bugs in the
   service channel error paths.

---

## 7. References

- [GCC C++ Standards Support](https://gcc.gnu.org/projects/cxx-status.html)
- [NVIDIA JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive)
- `CPP_PATTERNS_GUIDE.md` — detailed examples of C++17 features used in codebase
- Root `CMakeLists.txt` lines 21–22 — standard enforcement
