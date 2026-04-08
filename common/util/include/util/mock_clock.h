// common/util/include/util/mock_clock.h
// MockClock — manually advanceable clock for deterministic testing.
// Part of Epic #284 (Platform Modularity), Sub-Epic A, Gap G2.
//
// Usage in tests:
//   drone::util::MockClock mock;
//   drone::util::set_clock(&mock);
//
//   auto t0 = mock.now_ns();
//   mock.advance_ms(100);
//   auto t1 = mock.now_ns();   // t1 == t0 + 100'000'000
//
//   // Cleanup
//   drone::util::set_clock(nullptr);  // restore SteadyClock
#pragma once

#include "util/iclock.h"

#include <atomic>
#include <cstdint>

namespace drone::util {

/// Test clock with manually controlled time.
///
/// Time starts at a configurable initial value (default: 1'000'000'000 ns = 1 second).
/// Non-zero default avoids edge cases where code checks "if (timestamp == 0)"
/// to mean "uninitialized".
///
/// advance_ms() / advance_ns() move time forward.  Time never goes backward.
/// sleep_for_ms() advances time by the requested amount (no real blocking).
///
/// Thread safety: now_ns() uses atomic load (acquire).  advance_*() uses
/// atomic fetch_add (acq_rel).  Safe for concurrent readers + single writer.
class MockClock final : public IClock {
public:
    /// Construct with initial time in nanoseconds.
    /// Default 1 second to avoid zero-timestamp edge cases.
    explicit MockClock(uint64_t initial_ns = 1'000'000'000ULL) : time_ns_(initial_ns) {}

    [[nodiscard]] uint64_t now_ns() const override {
        return time_ns_.load(std::memory_order_acquire);
    }

    /// sleep_for_ms on MockClock advances simulated time (no real blocking).
    void sleep_for_ms(uint32_t ms) const override {
        // const_cast is safe here: the mutable semantic of "advancing time"
        // is intentional for MockClock — sleep should advance simulated time.
        const_cast<MockClock*>(this)->advance_ms(ms);
    }

    /// Advance time by the given number of nanoseconds.
    void advance_ns(uint64_t ns) { time_ns_.fetch_add(ns, std::memory_order_acq_rel); }

    /// Advance time by the given number of milliseconds.
    void advance_ms(uint32_t ms) { advance_ns(static_cast<uint64_t>(ms) * 1'000'000ULL); }

    /// Advance time by the given number of seconds.
    void advance_s(uint32_t s) { advance_ns(static_cast<uint64_t>(s) * 1'000'000'000ULL); }

    /// Set time to an absolute nanosecond value.
    /// Must be >= current time (time never goes backward).
    void set_ns(uint64_t ns) { time_ns_.store(ns, std::memory_order_release); }

    /// Reset to the given time.  For use in test setUp().
    void reset(uint64_t initial_ns = 1'000'000'000ULL) {
        time_ns_.store(initial_ns, std::memory_order_release);
    }

private:
    std::atomic<uint64_t> time_ns_;
};

/// RAII guard that installs a MockClock and restores the default on destruction.
/// Simplifies test fixtures:
///
///   TEST(Foo, Bar) {
///       drone::util::ScopedMockClock guard;
///       guard.mock().advance_ms(500);
///       // ... test code using drone::util::get_clock() ...
///   }  // default SteadyClock restored
class ScopedMockClock {
public:
    explicit ScopedMockClock(uint64_t initial_ns = 1'000'000'000ULL) : mock_(initial_ns) {
        set_clock(&mock_);
    }

    ~ScopedMockClock() { set_clock(nullptr); }

    ScopedMockClock(const ScopedMockClock&)            = delete;
    ScopedMockClock& operator=(const ScopedMockClock&) = delete;
    ScopedMockClock(ScopedMockClock&&)                 = delete;
    ScopedMockClock& operator=(ScopedMockClock&&)      = delete;

    /// Access the underlying MockClock for advance/set operations.
    [[nodiscard]] MockClock&       mock() { return mock_; }
    [[nodiscard]] const MockClock& mock() const { return mock_; }

private:
    MockClock mock_;
};

}  // namespace drone::util
