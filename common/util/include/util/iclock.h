// common/util/include/util/iclock.h
// IClock interface — abstracts std::chrono::steady_clock for testability.
// Part of Epic #284 (Platform Modularity), Sub-Epic A, Gap G2.
//
// Production code uses drone::util::get_clock() to get timestamps.
// Tests use drone::util::set_clock() to inject MockClock.
//
// Implementations:
//   - SteadyClock  (default, production)  — defined below
//   - MockClock    (tests, time control)  — see mock_clock.h
//   - SimClock     (future Gazebo co-sim) — TBD
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace drone::util {

/// Abstract clock interface for the entire drone stack.
///
/// All time-dependent code should use get_clock().now_ns() instead of
/// std::chrono::steady_clock::now() directly.  This enables:
///   - Deterministic unit tests via MockClock
///   - Time acceleration for simulation
///   - Consistent epoch across all components
class IClock {
public:
    virtual ~IClock() = default;

    /// Current time as nanoseconds since an unspecified epoch.
    /// Equivalent to steady_clock::now().time_since_epoch() in nanoseconds.
    [[nodiscard]] virtual uint64_t now_ns() const = 0;

    /// Sleep for the given number of milliseconds.
    /// MockClock can override to advance simulated time instead of blocking.
    virtual void sleep_for_ms(uint32_t ms) const = 0;

    /// Convenience: current time as steady_clock::time_point.
    /// Reconstructs from now_ns() so MockClock time is reflected.
    [[nodiscard]] std::chrono::steady_clock::time_point now() const {
        return std::chrono::steady_clock::time_point{std::chrono::nanoseconds{now_ns()}};
    }

    /// Convenience: current time as floating-point seconds since epoch.
    [[nodiscard]] double now_seconds() const {
        return static_cast<double>(now_ns()) / 1'000'000'000.0;
    }

protected:
    IClock()                         = default;
    IClock(const IClock&)            = default;
    IClock& operator=(const IClock&) = default;
    IClock(IClock&&)                 = default;
    IClock& operator=(IClock&&)      = default;
};

// ── SteadyClock — production default ───────────────────────────────

/// Production clock backed by std::chrono::steady_clock.
/// This is the default clock returned by drone::util::get_clock().
class SteadyClock final : public IClock {
public:
    [[nodiscard]] uint64_t now_ns() const override {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                         .count());
    }

    void sleep_for_ms(uint32_t ms) const override {
        std::this_thread::sleep_for(std::chrono::milliseconds{ms});
    }
};

// ── Global clock accessor ──────────────────────────────────────────
//
// Thread safety: get_clock() is hot-path (called on every timestamp
// read), so it uses an atomic load (acquire).  set_clock() is cold-path
// (startup / tests) and holds a mutex to protect ownership.
//
// SAFETY CONTRACT: set_clock() must only be called during single-threaded
// startup or in test fixture setup/teardown (before/after worker threads
// are running).  Calling it while other threads are actively reading the
// clock is a data race on the pointed-to object's lifetime.
// See issue #384 for a planned RCU-style fix.

namespace detail {

/// Process-wide default SteadyClock instance.
inline IClock& default_clock() {
    static SteadyClock instance;
    return instance;
}

/// Mutex protecting ownership transfer (cold-path only).
inline std::mutex& clock_mutex() {
    static std::mutex mtx;
    return mtx;
}

/// Atomic raw pointer cache for hot-path reads (null = use default).
inline std::atomic<IClock*>& clock_ptr() {
    static std::atomic<IClock*> ptr{nullptr};
    return ptr;
}

}  // namespace detail

/// Get the current global clock.  Never returns null.
/// Hot-path cost: one atomic load (acquire) + one branch.
/// Named get_clock() to avoid conflict with POSIX clock() from <time.h>.
[[nodiscard]] inline const IClock& get_clock() {
    auto* p = detail::clock_ptr().load(std::memory_order_acquire);
    if (p) return *p;
    return detail::default_clock();
}

/// Set the global clock.  Call once at startup or in test setup.
/// Thread-safe: serialized by mutex, atomic store visible to all readers.
///
/// @param clk  Pointer to clock instance.  Caller retains ownership
///             and must ensure the clock outlives all users.
///             Pass nullptr to restore the default SteadyClock.
inline void set_clock(IClock* clk) {
    std::lock_guard<std::mutex> lock(detail::clock_mutex());
    detail::clock_ptr().store(clk, std::memory_order_release);
}

}  // namespace drone::util
