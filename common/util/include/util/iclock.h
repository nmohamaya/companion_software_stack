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
#include <vector>

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
// RCU-style retirement (Issue #384): When set_clock() replaces the active
// clock, the OLD clock is moved into a retirement vector rather than
// destroyed.  This eliminates use-after-free: any thread that loaded the
// old pointer via get_clock() can safely finish its operation even if
// another thread concurrently swaps the clock.  The cost is a few leaked
// clock objects (2-3 per process lifetime max), which is negligible.

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

/// Owning pointer to the user-installed clock (null = use default).
/// Must hold clock_mutex() to read or write.
inline std::unique_ptr<IClock>& clock_owner() {
    static std::unique_ptr<IClock> owner;
    return owner;
}

/// Retired clocks — kept alive to prevent use-after-free.
/// Only accessed under clock_mutex().
inline std::vector<std::unique_ptr<IClock>>& retired_clocks() {
    static std::vector<std::unique_ptr<IClock>> vec;
    return vec;
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

/// Set the global clock (owning overload).  Takes ownership of the clock.
/// Thread-safe: serialized by mutex, atomic store visible to all readers.
/// The previous clock is retired (kept alive) to prevent use-after-free.
inline void set_clock(std::unique_ptr<IClock> clk) {
    std::lock_guard<std::mutex> lock(detail::clock_mutex());
    // Retire the old clock — do NOT destroy it.
    if (detail::clock_owner()) {
        detail::retired_clocks().push_back(std::move(detail::clock_owner()));
    }
    detail::clock_owner() = std::move(clk);
    detail::clock_ptr().store(detail::clock_owner().get(), std::memory_order_release);
}

/// Set the global clock (non-owning overload for stack-allocated clocks).
/// The caller retains ownership and must ensure the clock outlives all users.
/// Pass nullptr to restore the default SteadyClock.
/// The previous owned clock (if any) is retired, not destroyed.
inline void set_clock(IClock* clk) {
    std::lock_guard<std::mutex> lock(detail::clock_mutex());
    // Retire the old owned clock — do NOT destroy it.
    if (detail::clock_owner()) {
        detail::retired_clocks().push_back(std::move(detail::clock_owner()));
    }
    // No ownership transfer — clock_owner remains null.
    detail::clock_ptr().store(clk, std::memory_order_release);
}

}  // namespace drone::util
