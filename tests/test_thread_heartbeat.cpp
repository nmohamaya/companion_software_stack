// tests/test_thread_heartbeat.cpp
// Unit tests for ThreadHeartbeatRegistry, ScopedHeartbeat, and ThreadWatchdog.
// ADR-004 Layer 1 — Phase 1 (#89).
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::util;
using namespace std::chrono_literals;

// ─── Fixture: reset singleton between tests ─────────────────────────

class ThreadHeartbeatTest : public ::testing::Test {
protected:
    void SetUp() override { ThreadHeartbeatRegistry::instance().reset_for_testing(); }

    void TearDown() override { ThreadHeartbeatRegistry::instance().reset_for_testing(); }
};

// ════════════════════════════════════════════════════════════════════
//  ThreadHeartbeat struct tests
// ════════════════════════════════════════════════════════════════════

TEST_F(ThreadHeartbeatTest, DefaultValues) {
    ThreadHeartbeat hb;
    EXPECT_EQ(hb.name[0], '\0');
    EXPECT_EQ(hb.last_touch_ns.load(), 0u);
    EXPECT_FALSE(hb.is_critical);
}

TEST_F(ThreadHeartbeatTest, CopyPreservesFields) {
    ThreadHeartbeat original;
    std::strncpy(original.name, "test_thread", sizeof(original.name) - 1);
    original.last_touch_ns.store(42, std::memory_order_relaxed);
    original.is_critical = true;

    ThreadHeartbeat copy(original);
    EXPECT_STREQ(copy.name, "test_thread");
    EXPECT_EQ(copy.last_touch_ns.load(), 42u);
    EXPECT_TRUE(copy.is_critical);
}

// ════════════════════════════════════════════════════════════════════
//  ThreadHeartbeatRegistry tests
// ════════════════════════════════════════════════════════════════════

TEST_F(ThreadHeartbeatTest, RegisterSingle) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("worker", false);
    EXPECT_NE(handle, kInvalidHandle);
    EXPECT_EQ(ThreadHeartbeatRegistry::instance().count(), 1u);
}

TEST_F(ThreadHeartbeatTest, RegisterMultiple) {
    auto h1 = ThreadHeartbeatRegistry::instance().register_thread("t1", false);
    auto h2 = ThreadHeartbeatRegistry::instance().register_thread("t2", true);
    auto h3 = ThreadHeartbeatRegistry::instance().register_thread("t3", false);

    EXPECT_NE(h1, kInvalidHandle);
    EXPECT_NE(h2, kInvalidHandle);
    EXPECT_NE(h3, kInvalidHandle);
    EXPECT_NE(h1, h2);
    EXPECT_NE(h2, h3);
    EXPECT_EQ(ThreadHeartbeatRegistry::instance().count(), 3u);
}

TEST_F(ThreadHeartbeatTest, RegisterOverflowReturnsSentinel) {
    for (size_t i = 0; i < kMaxThreads; ++i) {
        std::string name = "t" + std::to_string(i);
        auto handle      = ThreadHeartbeatRegistry::instance().register_thread(name.c_str(), false);
        EXPECT_NE(handle, kInvalidHandle) << "Failed at index " << i;
    }
    // Registry is now full
    auto overflow = ThreadHeartbeatRegistry::instance().register_thread("overflow", false);
    EXPECT_EQ(overflow, kInvalidHandle);
    EXPECT_EQ(ThreadHeartbeatRegistry::instance().count(), kMaxThreads);
}

TEST_F(ThreadHeartbeatTest, TouchUpdatesTimestamp) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("worker", false);

    // Before touch: last_touch_ns should be 0
    auto snap1 = ThreadHeartbeatRegistry::instance().snapshot();
    ASSERT_EQ(snap1.size(), 1u);
    EXPECT_EQ(snap1[0].last_touch_ns.load(), 0u);

    // After touch: should be > 0
    ThreadHeartbeatRegistry::instance().touch(handle);
    auto snap2 = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_GT(snap2[0].last_touch_ns.load(), 0u);
}

TEST_F(ThreadHeartbeatTest, TouchMonotonicallyIncreases) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("worker", false);
    ThreadHeartbeatRegistry::instance().touch(handle);
    auto     snap1 = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts1   = snap1[0].last_touch_ns.load();

    std::this_thread::sleep_for(1ms);

    ThreadHeartbeatRegistry::instance().touch(handle);
    auto     snap2 = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts2   = snap2[0].last_touch_ns.load();

    EXPECT_GT(ts2, ts1);
}

TEST_F(ThreadHeartbeatTest, TouchWithGraceBumpsTimestampForward) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("loader", false);
    ThreadHeartbeatRegistry::instance().touch(handle);

    auto     snap1     = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts_before = snap1[0].last_touch_ns.load();

    // Grace of 10 seconds
    ThreadHeartbeatRegistry::instance().touch_with_grace(handle, 10000ms);

    auto     snap2    = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts_after = snap2[0].last_touch_ns.load();

    // Should be at least 9 seconds ahead of the previous timestamp
    const uint64_t nine_seconds_ns = 9'000'000'000ULL;
    EXPECT_GT(ts_after, ts_before + nine_seconds_ns);
}

TEST_F(ThreadHeartbeatTest, SnapshotReturnsAll) {
    ThreadHeartbeatRegistry::instance().register_thread("alpha", false);
    ThreadHeartbeatRegistry::instance().register_thread("beta", true);

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    ASSERT_EQ(snap.size(), 2u);
    EXPECT_STREQ(snap[0].name, "alpha");
    EXPECT_FALSE(snap[0].is_critical);
    EXPECT_STREQ(snap[1].name, "beta");
    EXPECT_TRUE(snap[1].is_critical);
}

TEST_F(ThreadHeartbeatTest, SnapshotIsDeepCopy) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("w", false);
    ThreadHeartbeatRegistry::instance().touch(handle);
    auto     snap        = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t original_ts = snap[0].last_touch_ns.load();

    // Touch again — snapshot should not change
    std::this_thread::sleep_for(1ms);
    ThreadHeartbeatRegistry::instance().touch(handle);
    EXPECT_EQ(snap[0].last_touch_ns.load(), original_ts);
}

TEST_F(ThreadHeartbeatTest, TouchInvalidHandleIsSafe) {
    // Should not crash
    ThreadHeartbeatRegistry::instance().touch(kInvalidHandle);
    ThreadHeartbeatRegistry::instance().touch(kMaxThreads);
    ThreadHeartbeatRegistry::instance().touch(kMaxThreads + 100);
    SUCCEED();  // No crash = pass
}

TEST_F(ThreadHeartbeatTest, NameTruncation) {
    const char* long_name = "this_is_a_very_long_thread_name_exceeding_32_chars";
    auto        handle    = ThreadHeartbeatRegistry::instance().register_thread(long_name, false);
    EXPECT_NE(handle, kInvalidHandle);

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    ASSERT_EQ(snap.size(), 1u);
    // Name should be truncated to 31 chars + null terminator
    EXPECT_EQ(std::strlen(snap[0].name), 31u);
    EXPECT_EQ(snap[0].name[31], '\0');
}

TEST_F(ThreadHeartbeatTest, CriticalFlagPreserved) {
    ThreadHeartbeatRegistry::instance().register_thread("critical_t", true);
    ThreadHeartbeatRegistry::instance().register_thread("normal_t", false);

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_TRUE(snap[0].is_critical);
    EXPECT_FALSE(snap[1].is_critical);
}

// ════════════════════════════════════════════════════════════════════
//  ScopedHeartbeat tests
// ════════════════════════════════════════════════════════════════════

TEST_F(ThreadHeartbeatTest, ScopedHeartbeatRegisters) {
    ScopedHeartbeat hb("scoped_thread");
    EXPECT_TRUE(hb.is_valid());
    EXPECT_EQ(ThreadHeartbeatRegistry::instance().count(), 1u);

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_STREQ(snap[0].name, "scoped_thread");
    EXPECT_FALSE(snap[0].is_critical);
}

TEST_F(ThreadHeartbeatTest, ScopedHeartbeatCritical) {
    ScopedHeartbeat hb("crit", true);
    auto            snap = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_TRUE(snap[0].is_critical);
}

TEST_F(ThreadHeartbeatTest, ScopedHeartbeatTouch) {
    ScopedHeartbeat hb("worker");
    hb.touch();

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_GT(snap[0].last_touch_ns.load(), 0u);
}

TEST_F(ThreadHeartbeatTest, ScopedHeartbeatTouchWithGrace) {
    ScopedHeartbeat hb("loader");
    hb.touch();

    auto     snap1 = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts1   = snap1[0].last_touch_ns.load();

    hb.touch_with_grace(5000ms);

    auto     snap2 = ThreadHeartbeatRegistry::instance().snapshot();
    uint64_t ts2   = snap2[0].last_touch_ns.load();

    // Should jump forward by ~5s
    EXPECT_GT(ts2, ts1 + 4'000'000'000ULL);
}

// ════════════════════════════════════════════════════════════════════
//  ThreadWatchdog tests
// ════════════════════════════════════════════════════════════════════

TEST_F(ThreadHeartbeatTest, WatchdogHealthyThreadNotFlagged) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("healthy", false);
    ThreadHeartbeatRegistry::instance().touch(handle);

    std::atomic<bool> callback_fired{false};

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 2000ms;
    cfg.scan_interval   = 100ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback(
        [&](const ThreadHeartbeat&) { callback_fired.store(true, std::memory_order_relaxed); });

    // Keep touching for 500ms — should never trigger
    auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        ThreadHeartbeatRegistry::instance().touch(handle);
        std::this_thread::sleep_for(50ms);
    }

    EXPECT_FALSE(callback_fired.load());
    EXPECT_TRUE(watchdog.get_stuck_threads().empty());
}

TEST_F(ThreadHeartbeatTest, WatchdogStuckThreadTriggersCallback) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("stuck_thread", true);
    ThreadHeartbeatRegistry::instance().touch(handle);

    std::atomic<bool> callback_fired{false};
    std::string       stuck_name;
    std::mutex        name_mtx;

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 200ms;  // Short threshold for testing
    cfg.scan_interval   = 50ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback([&](const ThreadHeartbeat& beat) {
        callback_fired.store(true, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(name_mtx);
        stuck_name = beat.name;
    });

    // Don't touch — let it become stuck
    std::this_thread::sleep_for(500ms);

    EXPECT_TRUE(callback_fired.load());
    {
        std::lock_guard<std::mutex> lk(name_mtx);
        EXPECT_EQ(stuck_name, "stuck_thread");
    }

    auto stuck = watchdog.get_stuck_threads();
    ASSERT_FALSE(stuck.empty());
    EXPECT_EQ(stuck[0], "stuck_thread");
}

TEST_F(ThreadHeartbeatTest, WatchdogGracePeriodSuppressesCallback) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("model_loader", false);

    // Touch with grace period — 5 seconds into the future
    ThreadHeartbeatRegistry::instance().touch_with_grace(handle, 5000ms);

    std::atomic<bool> callback_fired{false};

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 200ms;
    cfg.scan_interval   = 50ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback(
        [&](const ThreadHeartbeat&) { callback_fired.store(true, std::memory_order_relaxed); });

    // Wait 500ms — would normally trigger, but grace period prevents it
    std::this_thread::sleep_for(500ms);

    EXPECT_FALSE(callback_fired.load());
    EXPECT_TRUE(watchdog.get_stuck_threads().empty());
}

TEST_F(ThreadHeartbeatTest, WatchdogCriticalFlagPropagated) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("safety_thread", true);
    ThreadHeartbeatRegistry::instance().touch(handle);

    std::atomic<bool> critical_was_set{false};

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 200ms;
    cfg.scan_interval   = 50ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback([&](const ThreadHeartbeat& beat) {
        critical_was_set.store(beat.is_critical, std::memory_order_relaxed);
    });

    std::this_thread::sleep_for(500ms);

    EXPECT_TRUE(critical_was_set.load());
}

TEST_F(ThreadHeartbeatTest, WatchdogIgnoresUnstartedThread) {
    // Register but never touch — last_touch_ns stays 0
    ThreadHeartbeatRegistry::instance().register_thread("lazy_thread", false);

    std::atomic<bool> callback_fired{false};

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 200ms;
    cfg.scan_interval   = 50ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback(
        [&](const ThreadHeartbeat&) { callback_fired.store(true, std::memory_order_relaxed); });

    std::this_thread::sleep_for(500ms);

    // Should NOT fire — thread was registered but never started (ts == 0)
    EXPECT_FALSE(callback_fired.load());
}

TEST_F(ThreadHeartbeatTest, WatchdogMultipleThreadsMixed) {
    auto h_healthy = ThreadHeartbeatRegistry::instance().register_thread("healthy", false);
    auto h_stuck   = ThreadHeartbeatRegistry::instance().register_thread("stuck", false);

    ThreadHeartbeatRegistry::instance().touch(h_healthy);
    ThreadHeartbeatRegistry::instance().touch(h_stuck);

    std::vector<std::string> fired_names;
    std::mutex               fired_mtx;

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = 200ms;
    cfg.scan_interval   = 50ms;
    ThreadWatchdog watchdog(cfg);
    watchdog.set_stuck_callback([&](const ThreadHeartbeat& beat) {
        std::lock_guard<std::mutex> lk(fired_mtx);
        // Avoid duplicates
        for (const auto& n : fired_names) {
            if (n == beat.name) return;
        }
        fired_names.push_back(beat.name);
    });

    // Keep healthy thread alive, let stuck thread go stale
    auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        ThreadHeartbeatRegistry::instance().touch(h_healthy);
        std::this_thread::sleep_for(50ms);
    }

    std::lock_guard<std::mutex> lk(fired_mtx);
    EXPECT_EQ(fired_names.size(), 1u);
    EXPECT_EQ(fired_names[0], "stuck");
}

// ════════════════════════════════════════════════════════════════════
//  Concurrent safety tests (TSan-friendly)
// ════════════════════════════════════════════════════════════════════

TEST_F(ThreadHeartbeatTest, ConcurrentRegisterAndTouch) {
    constexpr int            kNumThreads = 8;
    std::vector<std::thread> threads;
    std::atomic<int>         registered{0};

    for (int i = 0; i < kNumThreads; ++i) {
        threads.emplace_back([i, &registered] {
            std::string name   = "thread_" + std::to_string(i);
            auto        handle = ThreadHeartbeatRegistry::instance().register_thread(name.c_str(),
                                                                                     i % 2 == 0);
            if (handle != kInvalidHandle) {
                registered.fetch_add(1, std::memory_order_relaxed);
                for (int j = 0; j < 100; ++j) {
                    ThreadHeartbeatRegistry::instance().touch(handle);
                }
            }
        });
    }

    for (auto& t : threads) t.join();

    EXPECT_EQ(registered.load(), kNumThreads);
    EXPECT_EQ(ThreadHeartbeatRegistry::instance().count(), static_cast<size_t>(kNumThreads));

    auto snap = ThreadHeartbeatRegistry::instance().snapshot();
    EXPECT_EQ(snap.size(), static_cast<size_t>(kNumThreads));

    // All should have non-zero timestamps
    for (const auto& beat : snap) {
        EXPECT_GT(beat.last_touch_ns.load(), 0u);
        EXPECT_GT(std::strlen(beat.name), 0u);
    }
}

TEST_F(ThreadHeartbeatTest, ConcurrentTouchAndSnapshot) {
    auto handle = ThreadHeartbeatRegistry::instance().register_thread("contended", false);
    ThreadHeartbeatRegistry::instance().touch(handle);

    std::atomic<bool> stop{false};

    // Writer thread — touches rapidly
    std::thread writer([&] {
        while (!stop.load(std::memory_order_relaxed)) {
            ThreadHeartbeatRegistry::instance().touch(handle);
        }
    });

    // Reader thread — takes snapshots rapidly
    std::thread reader([&] {
        for (int i = 0; i < 1000; ++i) {
            auto snap = ThreadHeartbeatRegistry::instance().snapshot();
            ASSERT_EQ(snap.size(), 1u);
            EXPECT_GT(snap[0].last_touch_ns.load(), 0u);
        }
    });

    reader.join();
    stop.store(true, std::memory_order_relaxed);
    writer.join();
}
