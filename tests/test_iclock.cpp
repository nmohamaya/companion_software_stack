// tests/test_iclock.cpp
// Unit tests for IClock interface, SteadyClock, MockClock, and ScopedMockClock.
// Issue #286 — Epic #284 (Platform Modularity), Sub-Epic A, Gap G2.
#include "util/iclock.h"
#include "util/mock_clock.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

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
    set_clock(nullptr);

    // Now it should be steady_clock again (large value)
    EXPECT_GT(get_clock().now_ns(), 1'000'000'000'000ULL);  // Should be way past 1 second
}

TEST(GlobalClockTest, SetClockNullRestoresDefault) {
    MockClock mock(1ULL);
    set_clock(&mock);
    EXPECT_EQ(get_clock().now_ns(), 1ULL);

    set_clock(nullptr);
    // Default SteadyClock — should be a large number (current wall time)
    EXPECT_GT(get_clock().now_ns(), 1'000'000'000ULL);
}

// ════════════════════════════════════════════════════════════════════
//  ScopedMockClock tests
// ════════════════════════════════════════════════════════════════════

TEST(ScopedMockClockTest, InstallsAndRestores) {
    // Before scope: should be SteadyClock (large value)
    EXPECT_GT(get_clock().now_ns(), 1'000'000'000'000ULL);

    {
        ScopedMockClock guard;
        EXPECT_EQ(get_clock().now_ns(), 1'000'000'000ULL);  // MockClock default

        guard.mock().advance_ms(500);
        EXPECT_EQ(get_clock().now_ns(), 1'500'000'000ULL);
    }

    // After scope: restored to SteadyClock
    EXPECT_GT(get_clock().now_ns(), 1'000'000'000'000ULL);
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
    EXPECT_GT(as_steady.now_ns(), 1'000'000'000ULL);
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
