// tests/test_iclock.cpp
// Unit tests for IClock interface, SteadyClock, MockClock, and ScopedMockClock.
// Issue #286 — Epic #284 (Platform Modularity), Sub-Epic A, Gap G2.
#include "util/iclock.h"
#include "util/mock_clock.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::util;

// ════════════════════════════════════════════════════════════════════
//  SteadyClock tests
// ════════════════════════════════════════════════════════════════════

TEST(SteadyClockTest, NowNsReturnsSteadyClockTime) {
    SteadyClock clk;
    const auto  before =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    const auto now = clk.now_ns();
    const auto after =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    EXPECT_GE(now, before);
    EXPECT_LE(now, after);
}

TEST(SteadyClockTest, NowReturnsTimePoint) {
    SteadyClock clk;
    const auto  before = std::chrono::steady_clock::now();
    const auto  now    = clk.now();
    const auto  after  = std::chrono::steady_clock::now();
    EXPECT_GE(now, before);
    EXPECT_LE(now, after);
}

TEST(SteadyClockTest, NowSecondsReturnsPositiveValue) {
    SteadyClock clk;
    const auto  seconds = clk.now_seconds();
    EXPECT_GT(seconds, 0.0);
}

TEST(SteadyClockTest, MonotonicallyIncreasing) {
    SteadyClock clk;
    const auto  t0 = clk.now_ns();
    const auto  t1 = clk.now_ns();
    EXPECT_GE(t1, t0);
}

// ════════════════════════════════════════════════════════════════════
//  MockClock tests
// ════════════════════════════════════════════════════════════════════

TEST(MockClockTest, DefaultInitialTime) {
    MockClock mock;
    EXPECT_EQ(mock.now_ns(), 1'000'000'000ULL);  // 1 second
}

TEST(MockClockTest, CustomInitialTime) {
    MockClock mock(42'000'000ULL);
    EXPECT_EQ(mock.now_ns(), 42'000'000ULL);
}

TEST(MockClockTest, AdvanceNs) {
    MockClock mock(0);
    mock.advance_ns(12345);
    EXPECT_EQ(mock.now_ns(), 12345ULL);
}

TEST(MockClockTest, AdvanceMs) {
    MockClock mock(0);
    mock.advance_ms(100);
    EXPECT_EQ(mock.now_ns(), 100'000'000ULL);
}

TEST(MockClockTest, AdvanceS) {
    MockClock mock(0);
    mock.advance_s(3);
    EXPECT_EQ(mock.now_ns(), 3'000'000'000ULL);
}

TEST(MockClockTest, AdvanceCumulative) {
    MockClock mock(1'000'000'000ULL);
    mock.advance_ms(100);
    mock.advance_ms(200);
    EXPECT_EQ(mock.now_ns(), 1'300'000'000ULL);  // 1s + 100ms + 200ms
}

TEST(MockClockTest, SetNs) {
    MockClock mock(0);
    mock.set_ns(5'000'000'000ULL);
    EXPECT_EQ(mock.now_ns(), 5'000'000'000ULL);
}

TEST(MockClockTest, Reset) {
    MockClock mock(0);
    mock.advance_s(10);
    mock.reset();
    EXPECT_EQ(mock.now_ns(), 1'000'000'000ULL);  // Default reset value
}

TEST(MockClockTest, ResetWithCustomValue) {
    MockClock mock(0);
    mock.advance_s(10);
    mock.reset(42ULL);
    EXPECT_EQ(mock.now_ns(), 42ULL);
}

TEST(MockClockTest, SleepForMsAdvancesTime) {
    MockClock mock(0);
    mock.sleep_for_ms(50);
    EXPECT_EQ(mock.now_ns(), 50'000'000ULL);
}

TEST(MockClockTest, NowReturnsCorrectTimePoint) {
    MockClock mock(2'000'000'000ULL);  // 2 seconds
    auto      tp = mock.now();
    auto      ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    EXPECT_EQ(static_cast<uint64_t>(ns.count()), 2'000'000'000ULL);
}

TEST(MockClockTest, NowSecondsReturnsCorrectValue) {
    MockClock mock(3'500'000'000ULL);  // 3.5 seconds
    EXPECT_DOUBLE_EQ(mock.now_seconds(), 3.5);
}

TEST(MockClockTest, ThreadSafeReads) {
    MockClock         mock(0);
    std::atomic<bool> start{false};
    std::atomic<bool> done{false};

    // Reader thread
    std::thread reader([&] {
        while (!start.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        for (int i = 0; i < 1000; ++i) {
            auto t = mock.now_ns();
            (void)t;  // Just ensure no crash/TSAN race
        }
        done.store(true, std::memory_order_release);
    });

    start.store(true, std::memory_order_release);
    for (int i = 0; i < 100; ++i) {
        mock.advance_ns(1000);
    }

    reader.join();
    EXPECT_TRUE(done.load(std::memory_order_acquire));
}

// ════════════════════════════════════════════════════════════════════
//  Global clock accessor tests
// ════════════════════════════════════════════════════════════════════

TEST(GlobalClockTest, DefaultIsSteadyClock) {
    // Default clock should return a time close to steady_clock::now()
    const auto expected =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    const auto actual = get_clock().now_ns();
    // Allow 10ms tolerance for timing jitter
    EXPECT_NEAR(static_cast<double>(actual), static_cast<double>(expected), 10'000'000.0);
}

TEST(GlobalClockTest, SetClockToMock) {
    MockClock mock(42'000'000ULL);
    set_clock(&mock);

    EXPECT_EQ(get_clock().now_ns(), 42'000'000ULL);

    mock.advance_ms(10);
    EXPECT_EQ(get_clock().now_ns(), 52'000'000ULL);

    // Restore default
    reset_clock();

    // Now it should be steady_clock again — verify with before/after window
    const auto before_restore =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    const auto restored = get_clock().now_ns();
    const auto after_restore =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    EXPECT_GE(restored, before_restore);
    EXPECT_LE(restored, after_restore);
}

TEST(GlobalClockTest, SetClockNullRestoresDefault) {
    MockClock mock(1ULL);
    set_clock(&mock);
    EXPECT_EQ(get_clock().now_ns(), 1ULL);

    reset_clock();
    // Default SteadyClock — verify with before/after window
    const auto before =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    const auto actual = get_clock().now_ns();
    const auto after =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    EXPECT_GE(actual, before);
    EXPECT_LE(actual, after);
}

// ════════════════════════════════════════════════════════════════════
//  ScopedMockClock tests
// ════════════════════════════════════════════════════════════════════

TEST(ScopedMockClockTest, InstallsAndRestores) {
    // Helper to read steady_clock as uint64_t nanoseconds.
    auto steady_ns = []() {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                         .count());
    };

    // Before scope: should be SteadyClock — verify with window
    const auto before1 = steady_ns();
    const auto val1    = get_clock().now_ns();
    const auto after1  = steady_ns();
    EXPECT_GE(val1, before1);
    EXPECT_LE(val1, after1);

    {
        ScopedMockClock guard;
        EXPECT_EQ(get_clock().now_ns(), 1'000'000'000ULL);  // MockClock default

        guard.mock().advance_ms(500);
        EXPECT_EQ(get_clock().now_ns(), 1'500'000'000ULL);
    }

    // After scope: restored to SteadyClock — verify with window
    const auto before2 = steady_ns();
    const auto val2    = get_clock().now_ns();
    const auto after2  = steady_ns();
    EXPECT_GE(val2, before2);
    EXPECT_LE(val2, after2);
}

TEST(ScopedMockClockTest, CustomInitialTime) {
    {
        ScopedMockClock guard(5'000'000'000ULL);  // 5 seconds
        EXPECT_EQ(get_clock().now_ns(), 5'000'000'000ULL);
    }
}

TEST(ScopedMockClockTest, SleepAdvancesTime) {
    {
        ScopedMockClock guard(0);
        get_clock().sleep_for_ms(100);
        EXPECT_EQ(get_clock().now_ns(), 100'000'000ULL);
    }
}

// ════════════════════════════════════════════════════════════════════
//  IClock polymorphism tests
// ════════════════════════════════════════════════════════════════════

TEST(IClockPolymorphismTest, VirtualDispatch) {
    MockClock   mock(42ULL);
    SteadyClock steady;

    const IClock& as_mock   = mock;
    const IClock& as_steady = steady;

    EXPECT_EQ(as_mock.now_ns(), 42ULL);

    // Verify SteadyClock via polymorphic ref with before/after window
    const auto before_steady =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    const auto steady_val = as_steady.now_ns();
    const auto after_steady =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    EXPECT_GE(steady_val, before_steady);
    EXPECT_LE(steady_val, after_steady);
}

TEST(IClockPolymorphismTest, NowConvenienceMethod) {
    MockClock     mock(2'000'000'000ULL);
    const IClock& ref = mock;

    auto tp = ref.now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    EXPECT_EQ(static_cast<uint64_t>(ns.count()), 2'000'000'000ULL);
}

TEST(IClockPolymorphismTest, NowSecondsConvenienceMethod) {
    MockClock     mock(1'500'000'000ULL);
    const IClock& ref = mock;
    EXPECT_DOUBLE_EQ(ref.now_seconds(), 1.5);
}

// ════════════════════════════════════════════════════════════════════
//  RCU-style retirement tests (Issue #384)
// ════════════════════════════════════════════════════════════════════

TEST(ClockRCUTest, SetClockOwningOverload) {
    // Test the unique_ptr overload of set_clock.
    auto  mock = std::make_unique<MockClock>(42'000'000ULL);
    auto* ptr  = mock.get();
    set_clock(std::move(mock));

    EXPECT_EQ(get_clock().now_ns(), 42'000'000ULL);

    // Advance via the raw pointer we kept.
    ptr->advance_ms(10);
    EXPECT_EQ(get_clock().now_ns(), 52'000'000ULL);

    // Restore default.
    reset_clock();
}

TEST(ClockRCUTest, OldClockSurvivesAfterReplacement) {
    // The old clock must remain callable after set_clock() replaces it,
    // because the retirement vector keeps owned clocks alive.
    auto  mock1 = std::make_unique<MockClock>(100ULL);
    auto* ptr1  = mock1.get();
    set_clock(std::move(mock1));

    EXPECT_EQ(get_clock().now_ns(), 100ULL);

    // Replace with a second clock — ptr1 should still be alive (retired).
    auto mock2 = std::make_unique<MockClock>(200ULL);
    set_clock(std::move(mock2));

    // The old clock (ptr1) must still be callable — not destroyed.
    EXPECT_EQ(ptr1->now_ns(), 100ULL);
    ptr1->advance_ns(50);
    EXPECT_EQ(ptr1->now_ns(), 150ULL);

    // Restore default.
    reset_clock();
}

TEST(ClockRCUTest, ConcurrentClockAccessDuringSwap) {
    // Stress test: multiple threads call get_clock() while another thread
    // swaps clocks.  Must not crash or trigger TSAN/ASAN.
    constexpr int kIterations = 5000;
    constexpr int kReaders    = 4;

    std::atomic<bool> start{false};
    std::atomic<bool> stop{false};

    // Reader threads: continuously call get_clock().now_ns().
    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    for (int i = 0; i < kReaders; ++i) {
        readers.emplace_back([&] {
            while (!start.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            while (!stop.load(std::memory_order_acquire)) {
                auto t = get_clock().now_ns();
                (void)t;  // Just ensure no crash/TSAN race
            }
        });
    }

    start.store(true, std::memory_order_release);

    // Writer thread: swap clocks repeatedly using the OWNING overload
    // to exercise the RCU retirement path (retired_clocks vector).
    for (int i = 0; i < kIterations; ++i) {
        set_clock(std::make_unique<MockClock>(static_cast<uint64_t>(i + 1) * 1'000ULL));
    }

    stop.store(true, std::memory_order_release);
    for (auto& t : readers) {
        t.join();
    }

    // Restore default.
    reset_clock();
    // If we get here without crash/TSAN flag, the test passes.
}
