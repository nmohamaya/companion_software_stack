# ADR-007: Error Handling Strategy — `Result<T,E>` Over Exceptions

| Field | Value |
|-------|-------|
| **Status** | Accepted — fully implemented |
| **Date** | 2026-03-13 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | ADR-003 (C++17), Issue #149 |

---

## 1. Context

An autonomous drone is a safety-critical embedded system with two
requirements in tension:

1. **Robustness** — every possible failure must be handled and surfaced
   to the fault manager, never silently swallowed.
2. **Determinism** — flight-control threads run on soft-real-time
   schedules; unexpected heap allocations or stack-unwinding pauses are
   not acceptable.

The C++ exception mechanism provides condition-based error propagation
but at a cost:

- Exceptions require RTTI and stack-unwinding tables which roughly double
  the binary size on gcc/clang for AArch64.
- An unhandled exception terminates the entire process; one faulty
  thread kills all seven flight-critical threads.
- The compiler cannot enforce that callers check for failures:
  `throw`/`catch` sites are invisible at the call site.
- Exception-based control flow is difficult to reason about under
  real-time scheduling pressure.

Before the current approach was adopted the codebase mixed raw `bool`
returns, errno checks, and occasional `std::optional<T>` — with no
uniform way to attach an error message to a failure.

---

## 2. Decision Drivers

- No exceptions in flight-path code
- No RTTI (`-fno-rtti` compatible)
- Zero heap allocation for error values
- Compiler-enforced handling (`[[nodiscard]]`)
- Rich error codes (not just `bool`) for structured logging
- Monadically chainable without callback pyramid
- Domain-specific error types for tightly-scoped modules

---

## 3. Decision

Adopt `drone::util::Result<T, E>` (header-only, in
`common/util/include/util/result.h`) as the single error-handling
primitive.  No exceptions are used in flight-path code.

### 3.1 `Result<T, E>` Basics

```cpp
template <typename T, typename E = Error>
class [[nodiscard]] Result {
public:
    static Result ok(T value);
    static Result err(E error);

    bool is_ok()  const;
    bool is_err() const;

    const T& value() const;   // UB if is_err() — only call after checking
    const E& error() const;   // UB if is_ok()

    T value_or(T default_value) const;

    // Monadic combinators
    template <typename F> auto map(F&& f) const;        // T → U
    template <typename F> auto and_then(F&& f) const;   // T → Result<U,E>
    template <typename F> auto map_error(F&& f) const;  // E → F
};
```

`Result<void, E>` specialisation handles operations that succeed without
a value.  The `VoidResult` alias (`Result<void, Error>`) is used for
void-returning operations.

### 3.2 Standard Error Type

```cpp
enum class ErrorCode : uint8_t {
    Unknown       = 0,
    FileNotFound  = 1,
    ParseError    = 2,
    InvalidValue  = 3,
    MissingKey    = 4,
    TypeMismatch  = 5,
    OutOfRange    = 6,
    Timeout       = 7,
    NotConnected  = 8,
    AlreadyExists = 9,
};

class Error {
public:
    explicit Error(ErrorCode code, std::string message);
    ErrorCode   code()    const;
    const std::string& message() const;
};
```

### 3.3 Domain-Specific Aliases

Tightly-scoped modules define a domain error type and alias:

```cpp
// In process3_slam_vio_nav/include/slam/vio_types.h
enum class VIOError { ... };
template <typename T>
using VIOResult = drone::util::Result<T, VIOError>;
```

This gives module-internal code rich error semantics without coupling the
rest of the codebase to a single `Error` superset.

---

## 4. Considered Alternatives

### 4.1 C++ exceptions (rejected)

See §1 (Context) for the detailed objections.  The primary rejection
reasons are RTTI overhead, non-`[[nodiscard]]` enforcement, and
stack-unwinding behaviour under real-time scheduling.

### 4.2 `std::expected<T, E>` (C++23) (deferred)

`std::expected` is semantically equivalent and part of the standard.
Adoption is deferred until compiler support is stable on the GCC 12 /
Clang 15 toolchain used in CI.  Migration from `Result<T,E>` will be
straightforward when ready (same call-site API).

### 4.3 `std::optional<T>` (insufficient)

`std::optional<T>` distinguishes "value" from "no value" but carries no
error information.  It is still used in this codebase for case where
absence is semantically meaningful and not an error (e.g., optional
config keys), but is never used as an error-propagation mechanism.

### 4.4 Raw `bool` + out-parameter (rejected)

`bool open(int& error_code)` is unenforceable: callers can ignore the
bool, and the out-parameter requires an lvalue at every call site.

---

## 5. Consequences

### Positive

- `[[nodiscard]]` makes ignoring an error a compiler warning (with
  `-Wall`, promoted to error with `-Werror`)
- Monadic combinators (`.map()`, `.and_then()`) allow linear, readable
  error-propagation chains without deeply nested `if` checks
- All error paths are visible at the call site, making code review
  and static analysis straightforward
- Zero heap allocation: `Result<T, E>` holds a `std::variant<T, E>`
- Domain aliases allow module-local error taxonomies without polluting
  the global `ErrorCode` enum

### Negative / Trade-offs

- New developers must learn the `Result` idiom; the `value()` precondition
  (only safe to call after `is_ok()`) is a footgun if ignored
- Domains that legitimately have many error codes need to maintain their
  domain enum alongside the standard `ErrorCode`
- `and_then` chains can produce deep template instantiations that slow
  compilation in large translation units

---

## 6. Usage Rules

1. **All public APIs that can fail must return `Result<T,E>`.** Void
   operations return `VoidResult`.
2. **Never call `.value()` without first checking `.is_ok()`.** Prefer
   `.value_or()` or monadic chains.
3. **Never `throw` in flight-path code.** Exceptions are permitted in
   `main()` startup before IPC channels open.
4. **Log then propagate.** If a recoverable error is detected inside a
   subsystem, log it and return `err(...)`.  Do not log the same error
   at multiple levels.
5. **Domain aliases for modules with rich errors.** If a module has
   more error cases than `ErrorCode` covers, define a module-local
   enum and `using MyResult = drone::util::Result<T, MyError>`.

See [error_handling_design.md](../error_handling_design.md) for the full
API reference and code examples.

---

## 7. Review Status

Accepted — implemented across all 7 processes and common utilities.
No known issues.
