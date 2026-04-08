// common/util/include/util/iclock.h
// IClock interface — abstracts std::chrono::steady_clock for testability.
// Part of Epic #284 (Platform Modularity), Sub-Epic A, Gap G2.
//
// Production code uses drone::util::get_clock() to get timestamps.
// Tests use drone::util::set_clock() to inject MockClock.
//
// Implementations:
//   - SteadyClock  (default, production)  — see steady_clock.h
//   - MockClock    (tests, time control)  — see mock_clock.h
//   - SimClock     (future Gazebo co-sim) — TBD
#pragma once

#include <chrono>
#include <memory>
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

namespace detail {

/// Returns a reference to the global clock pointer.
/// Default-initializes to SteadyClock on first call.
/// NOT thread-safe for set_clock() — call set_clock() once at startup
/// before spawning worker threads.
inline IClock*& global_clock_ptr() {
    static SteadyClock default_clock;
    static IClock*     ptr = &default_clock;
    return ptr;
}

}  // namespace detail

/// Get the current global clock.
/// Thread-safe for reads (after initialization).
/// Returns SteadyClock by default.
/// Named get_clock() to avoid conflict with POSIX clock() from <time.h>.
[[nodiscard]] inline const IClock& get_clock() {
    return *detail::global_clock_ptr();
}

/// Set the global clock.  Call once at startup or in test setup.
/// NOT thread-safe — must be called before worker threads start,
/// or inside a test fixture setUp() when no other threads are running.
///
/// @param clk  Pointer to clock instance.  Caller retains ownership
///             and must ensure the clock outlives all users.
///             Pass nullptr to restore the default SteadyClock.
inline void set_clock(IClock* clk) {
    if (clk == nullptr) {
        // Restore default
        static SteadyClock default_clock;
        detail::global_clock_ptr() = &default_clock;
    } else {
        detail::global_clock_ptr() = clk;
    }
}

}  // namespace drone::util
