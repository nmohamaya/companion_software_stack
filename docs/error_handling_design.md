# Error Handling Design

This document describes the `Result<T,E>` error-handling pattern used
throughout the drone stack.  The architecture decision is in
[ADR-007](adr/ADR-007-error-handling.md).

---

## 1. Overview

All operations that can fail return `drone::util::Result<T, E>` instead
of throwing exceptions or returning raw `bool`.  The type is:

- **`[[nodiscard]]`** — ignoring the return value is a compiler error
  (`-Werror` is enabled in CI)
- **Zero heap allocation** — internally a `std::variant<T, E>`
- **Monadically chainable** — `.map()`, `.and_then()`, `.map_error()`
  allow linear call chains without nested `if` blocks

**Header:** `common/util/include/util/result.h`

---

## 2. Standard Error Type

### 2.1 `ErrorCode` Enum

```cpp
enum class ErrorCode : uint8_t {
    Unknown       = 0,  // Unclassified error
    FileNotFound  = 1,  // File path does not exist
    ParseError    = 2,  // Syntax error in a config / data file
    InvalidValue  = 3,  // Value out of domain (e.g. negative radius)
    MissingKey    = 4,  // Required config key absent
    TypeMismatch  = 5,  // Config value has wrong type
    OutOfRange    = 6,  // Numeric value outside allowed bounds
    Timeout       = 7,  // Operation didn't complete in time
    NotConnected  = 8,  // Hardware link is not open
    AlreadyExists = 9,  // Resource already allocated / registered
};
```

### 2.2 `Error` Class

```cpp
class Error {
public:
    explicit Error(ErrorCode code, std::string message = "");

    ErrorCode          code()    const;
    const std::string& message() const;

    bool operator==(const Error& other) const;
};
```

`Error` is cheap to copy (one `uint8_t` + `std::string`).

---

## 3. `Result<T, E>` API

### 3.1 Construction

```cpp
// Success
auto r = drone::util::Result<int>::ok(42);

// Failure (standard Error)
auto r = drone::util::Result<int>::err(
    drone::util::Error{drone::util::ErrorCode::Timeout, "read timed out"});

// Failure (custom error type)
auto r = drone::util::Result<int, VIOError>::err(VIOError::FeatureTrackingFailed);
```

### 3.2 Inspection

```cpp
if (r.is_ok()) {
    std::cout << r.value();      // safe: only call after is_ok()
} else {
    std::cerr << r.error().message();
}

// Safe fallback
int v = r.value_or(0);
```

### 3.3 Monadic Combinators

| Method | Input → Output | Use case |
|--------|---------------|----------|
| `.map(F)` | `T → U` | Transform a success value without changing the error type |
| `.and_then(F)` | `T → Result<U,E>` | Chain a fallible operation; short-circuits on error |
| `.map_error(F)` | `E → F` | Convert between error types (e.g., domain error → standard `Error`) |

**Example — chain of fallible operations:**

```cpp
auto result = cfg.load("config/default.json")
    .and_then([&](auto) { return camera->open(w, h, fps); })
    .and_then([&](auto) { return ipc_bus->init(); });

if (result.is_err()) {
    LOG_ERROR("startup failed: {}", result.error().message());
    return 1;
}
```

---

## 4. `Result<void, E>` Specialisation

For operations that succeed without a value:

```cpp
// typedef convenience
using VoidResult = drone::util::Result<void, drone::util::Error>;

// Construction
VoidResult r = VoidResult::ok();
VoidResult r = VoidResult::err({ErrorCode::NotConnected, "link down"});

// Chain into void-returning operations
VoidResult res = connect().and_then([&]() { return authenticate(); });
```

---

## 5. Domain-Specific Error Aliases

For modules with fine-grained error taxonomies, define a domain enum and
alias:

```cpp
// In process3_slam_vio_nav/include/slam/vio_types.h
enum class VIOError {
    FeatureTrackingFailed = 0,
    InsufficientFeatures  = 1,
    IMUPreintegrationFailed = 2,
    OptimisationDiverged  = 3,
};

template <typename T>
using VIOResult = drone::util::Result<T, VIOError>;
```

Usage inside P3:

```cpp
VIOResult<Pose> track_features(const Frame& frame) {
    if (features.size() < kMinFeatures)
        return VIOResult<Pose>::err(VIOError::InsufficientFeatures);
    ...
    return VIOResult<Pose>::ok(pose);
}
```

When propagating a domain error out of the module boundary, convert with
`.map_error()`:

```cpp
auto pose = vio_.track(frame)
    .map_error([](VIOError e) {
        return Error{ErrorCode::Unknown, "VIO: " + to_string(e)};
    });
```

---

## 6. Usage Rules

### 6.1 Do

- Return `Result<T,E>` from every public API that can fail.
- Use `VoidResult` for void-returning fallible operations.
- Check `is_ok()` before calling `value()`.
- Prefer `.and_then()` chains over nested `if` blocks.
- Use `.value_or(default)` for non-critical fallbacks.
- Log errors **once** at the point of detection; propagate without
  re-logging at higher layers.

### 6.2 Don't

- Don't `throw` in flight-path code; exceptions are only acceptable in
  `main()` startup before IPC channels open.
- Don't call `.value()` without first verifying `.is_ok()` — it is
  undefined behaviour when `is_err()` is true.
- Don't ignore a `Result` return value — `[[nodiscard]]` makes this a
  compiler warning, promoted to error in CI.
- Don't define domain error enums for simple operations that map cleanly
  to `ErrorCode`; only add a domain type when you genuinely need more
  error cases.

---

## 7. Error Propagation in the Processing Pipeline

```
cfg.load()           → Result<void,Error>
  └─.and_then         → camera->open()    → Result<void,Error>
      └─.and_then      → ipc_bus->init()  → Result<void,Error>
            ↓ err at any step short-circuits everything below
            ↓ ok continues to next step
```

Inside P3 vision pipeline:

```
feature_extractor.extract(frame)   → VIOResult<Features>
  └─.and_then  tracker.track(prev, cur) → VIOResult<Tracks>
      └─.and_then  imu_preintegrator.integrate() → VIOResult<IMUDelta>
          └─.and_then  optimizer.solve() → VIOResult<Pose>
```

---

## 8. Where `Result<T,E>` Is Used

| Module | Error type | Notes |
|--------|-----------|-------|
| `drone::Config` | `Result<T, Error>` | Used by `cfg.load()`, `cfg.get<T>()` |
| All HAL interfaces (open, capture, etc.) | `bool` / struct with `valid` flag | HAL methods use `bool` for simplicity at call sites; failures are logged internally |
| `drone::ipc::ShmWriter` / `ZenohPublisher` | `VoidResult` | IPC publish / subscribe init |
| P3 VIO pipeline | `VIOResult<T>` | Full monadic chain from feature extraction to pose |
| `drone::util::ThreadWatchdog` | `VoidResult` | Thread registration / deregistration |

> **Note on HAL methods:** `ICamera::open()` returns `bool` rather than
> `Result<void, Error>`.  This predates the `Result` adoption (PR #75) and
> is a known inconsistency.  Future backend additions should adopt
> `Result`-returning signatures.

---

## 9. Testing Error Paths

Every `Result`-returning function should have at least one test that
exercises the error branch:

```cpp
TEST(ConfigTest, MissingKeyReturnsError) {
    drone::Config cfg;
    auto result = cfg.get<int>("nonexistent.key");
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code(), drone::util::ErrorCode::MissingKey);
}
```

Use `ASSERT_TRUE(r.is_ok()) << r.error().message()` to get meaningful
failure messages when tests unexpectedly hit the error path.

---

## 10. Related Documents

- [ADR-007 — Error Handling Strategy](adr/ADR-007-error-handling.md)
- [CPP_PATTERNS_GUIDE.md](CPP_PATTERNS_GUIDE.md) — `Result<T,E>` patterns section
- `common/util/include/util/result.h` — implementation
- `process3_slam_vio_nav/include/slam/vio_types.h` — `VIOResult` alias
