// tests/test_thread_health_publisher.cpp
// Tests for ThreadHealth struct + ThreadHealthPublisher bridge logic.
// Phase 2 of the Process & Thread Watchdog (Epic #88, Issue #90).

#include "ipc/ipc_types.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <cstring>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::util;
using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Mock publisher — captures last published ThreadHealth
// ═══════════════════════════════════════════════════════════
class MockPublisher {
public:
    void publish(const ThreadHealth& health) {
        last_ = health;
        ++count_;
    }
    bool is_ready() const { return true; }

    const ThreadHealth& last() const { return last_; }
    int                 count() const { return count_; }

private:
    ThreadHealth last_{};
    int          count_ = 0;
};

// ═══════════════════════════════════════════════════════════
// Test fixture — resets the heartbeat registry between tests
// ═══════════════════════════════════════════════════════════
class ThreadHealthPublisherTest : public ::testing::Test {
protected:
    void SetUp() override { ThreadHeartbeatRegistry::instance().reset_for_testing(); }
    void TearDown() override { ThreadHeartbeatRegistry::instance().reset_for_testing(); }
};

// ───────────────────────────────────────────────────────────
// ThreadHealth struct correctness
// ───────────────────────────────────────────────────────────

TEST(ShmThreadHealthStruct, TrivialCopyable) {
    static_assert(std::is_trivially_copyable_v<ThreadHealth>,
                  "ThreadHealth must be trivially copyable for SHM transport");
}

TEST(ShmThreadHealthStruct, ThreadHealthEntryTrivialCopyable) {
    static_assert(std::is_trivially_copyable_v<ThreadHealthEntry>,
                  "ThreadHealthEntry must be trivially copyable");
}

TEST(ShmThreadHealthStruct, DefaultValues) {
    ThreadHealth health{};
    EXPECT_EQ(health.num_threads, 0);
    EXPECT_EQ(health.timestamp_ns, 0u);
    EXPECT_EQ(health.process_name[0], '\0');

    for (uint8_t i = 0; i < kMaxTrackedThreads; ++i) {
        EXPECT_EQ(health.threads[i].name[0], '\0');
        EXPECT_TRUE(health.threads[i].healthy);
        EXPECT_FALSE(health.threads[i].critical);
        EXPECT_EQ(health.threads[i].last_ns, 0u);
    }
}

TEST(ShmThreadHealthStruct, MaxTrackedThreadsIs16) {
    EXPECT_EQ(kMaxTrackedThreads, 16);
}

TEST(ShmThreadHealthStruct, ProcessNameFits31Chars) {
    ThreadHealth health{};
    const char*  long_name = "a_very_long_process_name_12345";  // 30 chars
    std::strncpy(health.process_name, long_name, sizeof(health.process_name) - 1);
    EXPECT_STREQ(health.process_name, long_name);
}

// ───────────────────────────────────────────────────────────
// ThreadHealthPublisher — basic publishing
// ───────────────────────────────────────────────────────────

TEST_F(ThreadHealthPublisherTest, ZeroThreadsPublishesEmptySnapshot) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "test_proc", watchdog);

    publisher.publish_snapshot();

    ASSERT_EQ(pub.count(), 1);
    const auto& h = pub.last();
    EXPECT_STREQ(h.process_name, "test_proc");
    EXPECT_EQ(h.num_threads, 0);
    EXPECT_GT(h.timestamp_ns, 0u);
}

TEST_F(ThreadHealthPublisherTest, SingleThreadPopulatesCorrectly) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "video_capture", watchdog);

    auto hb = ScopedHeartbeat("mission_cam", true);
    ThreadHeartbeatRegistry::instance().touch(hb.handle());

    publisher.publish_snapshot();

    ASSERT_EQ(pub.count(), 1);
    const auto& h = pub.last();
    EXPECT_STREQ(h.process_name, "video_capture");
    EXPECT_EQ(h.num_threads, 1);
    EXPECT_STREQ(h.threads[0].name, "mission_cam");
    EXPECT_TRUE(h.threads[0].critical);
    EXPECT_TRUE(h.threads[0].healthy);
    EXPECT_GT(h.threads[0].last_ns, 0u);
}

TEST_F(ThreadHealthPublisherTest, MultipleThreadsPopulateCorrectly) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "perception", watchdog);

    auto hb1 = ScopedHeartbeat("inference", true);
    auto hb2 = ScopedHeartbeat("tracker", true);
    auto hb3 = ScopedHeartbeat("diagnostics", false);

    ThreadHeartbeatRegistry::instance().touch(hb1.handle());
    ThreadHeartbeatRegistry::instance().touch(hb2.handle());
    ThreadHeartbeatRegistry::instance().touch(hb3.handle());

    publisher.publish_snapshot();

    const auto& h = pub.last();
    EXPECT_EQ(h.num_threads, 3);
    EXPECT_STREQ(h.threads[0].name, "inference");
    EXPECT_TRUE(h.threads[0].critical);
    EXPECT_STREQ(h.threads[1].name, "tracker");
    EXPECT_TRUE(h.threads[1].critical);
    EXPECT_STREQ(h.threads[2].name, "diagnostics");
    EXPECT_FALSE(h.threads[2].critical);
}

TEST_F(ThreadHealthPublisherTest, StuckThreadMarkedUnhealthy) {
    MockPublisher pub;

    // Use tight watchdog: 50ms stuck threshold, 10ms scan interval
    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = std::chrono::milliseconds(50);
    cfg.scan_interval   = std::chrono::milliseconds(10);
    ThreadWatchdog        watchdog(cfg);
    ThreadHealthPublisher publisher(pub, "test_stuck", watchdog);

    auto hb = ScopedHeartbeat("worker", true);
    ThreadHeartbeatRegistry::instance().touch(hb.handle());

    // Let watchdog detect the thread is stuck
    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    publisher.publish_snapshot();

    const auto& h = pub.last();
    ASSERT_EQ(h.num_threads, 1);
    EXPECT_STREQ(h.threads[0].name, "worker");
    EXPECT_FALSE(h.threads[0].healthy);  // marked stuck
    EXPECT_TRUE(h.threads[0].critical);
}

TEST_F(ThreadHealthPublisherTest, HealthyThreadMarkedHealthy) {
    MockPublisher pub;

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = std::chrono::milliseconds(200);
    cfg.scan_interval   = std::chrono::milliseconds(10);
    ThreadWatchdog        watchdog(cfg);
    ThreadHealthPublisher publisher(pub, "test_healthy", watchdog);

    auto hb = ScopedHeartbeat("active_worker", true);
    // Touch recent enough to not be stuck
    ThreadHeartbeatRegistry::instance().touch(hb.handle());

    // Brief sleep, but well within threshold
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    publisher.publish_snapshot();

    const auto& h = pub.last();
    ASSERT_EQ(h.num_threads, 1);
    EXPECT_TRUE(h.threads[0].healthy);
}

TEST_F(ThreadHealthPublisherTest, MaxThreadsSaturates) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "max_test", watchdog);

    // Register kMaxThreads (16) threads — that's the registry max too
    std::vector<ScopedHeartbeat> heartbeats;
    for (int i = 0; i < kMaxTrackedThreads; ++i) {
        std::string name = "t" + std::to_string(i);
        heartbeats.emplace_back(name.c_str(), i % 2 == 0);
        ThreadHeartbeatRegistry::instance().touch(heartbeats.back().handle());
    }

    publisher.publish_snapshot();

    const auto& h = pub.last();
    EXPECT_EQ(h.num_threads, kMaxTrackedThreads);
    for (uint8_t i = 0; i < h.num_threads; ++i) {
        EXPECT_GT(h.threads[i].last_ns, 0u);
    }
}

TEST_F(ThreadHealthPublisherTest, TimestampIsMonotonic) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "mono_test", watchdog);

    publisher.publish_snapshot();
    uint64_t ts1 = pub.last().timestamp_ns;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    publisher.publish_snapshot();
    uint64_t ts2 = pub.last().timestamp_ns;

    EXPECT_GT(ts2, ts1);
}

TEST_F(ThreadHealthPublisherTest, MultiplePublishCallsWork) {
    MockPublisher         pub;
    ThreadWatchdog        watchdog;
    ThreadHealthPublisher publisher(pub, "multi_test", watchdog);

    auto hb = ScopedHeartbeat("worker", false);
    ThreadHeartbeatRegistry::instance().touch(hb.handle());

    for (int i = 0; i < 5; ++i) {
        publisher.publish_snapshot();
    }
    EXPECT_EQ(pub.count(), 5);
}

// ───────────────────────────────────────────────────────────
// Integration: mock 2-thread process → publish → verify
// ───────────────────────────────────────────────────────────

TEST_F(ThreadHealthPublisherTest, TwoThreadIntegration) {
    MockPublisher pub;

    ThreadWatchdog::Config cfg;
    cfg.stuck_threshold = std::chrono::milliseconds(100);
    cfg.scan_interval   = std::chrono::milliseconds(10);
    ThreadWatchdog        watchdog(cfg);
    ThreadHealthPublisher publisher(pub, "integration", watchdog);

    // Simulate two threads: one active, one will become stuck
    auto hb_active = ScopedHeartbeat("active", true);
    auto hb_stuck  = ScopedHeartbeat("stuck", true);

    ThreadHeartbeatRegistry::instance().touch(hb_active.handle());
    ThreadHeartbeatRegistry::instance().touch(hb_stuck.handle());

    // Keep touching active, let stuck go stale
    for (int i = 0; i < 15; ++i) {
        ThreadHeartbeatRegistry::instance().touch(hb_active.handle());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    publisher.publish_snapshot();

    const auto& h = pub.last();
    EXPECT_STREQ(h.process_name, "integration");
    EXPECT_EQ(h.num_threads, 2);

    // Find each thread in the published data
    const ThreadHealthEntry* active_entry = nullptr;
    const ThreadHealthEntry* stuck_entry  = nullptr;
    for (uint8_t i = 0; i < h.num_threads; ++i) {
        if (std::strcmp(h.threads[i].name, "active") == 0) active_entry = &h.threads[i];
        if (std::strcmp(h.threads[i].name, "stuck") == 0) stuck_entry = &h.threads[i];
    }

    ASSERT_NE(active_entry, nullptr);
    ASSERT_NE(stuck_entry, nullptr);

    EXPECT_TRUE(active_entry->healthy);
    EXPECT_TRUE(active_entry->critical);
    EXPECT_GT(active_entry->last_ns, 0u);

    EXPECT_FALSE(stuck_entry->healthy);  // watchdog should flag this
    EXPECT_TRUE(stuck_entry->critical);
    EXPECT_GT(stuck_entry->last_ns, 0u);
}

TEST_F(ThreadHealthPublisherTest, ProcessNameTruncatesGracefully) {
    MockPublisher  pub;
    ThreadWatchdog watchdog;

    // Name longer than 31 chars — should be truncated, not overflow
    const char*           long_name = "this_is_a_very_long_process_name_that_exceeds_buffer";
    ThreadHealthPublisher publisher(pub, long_name, watchdog);

    publisher.publish_snapshot();

    const auto& h = pub.last();
    // Must be null-terminated within 32 bytes
    EXPECT_EQ(h.process_name[31], '\0');
    EXPECT_EQ(std::strlen(h.process_name), 31u);
}
