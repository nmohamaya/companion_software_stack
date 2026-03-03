// tests/test_latency_tracker.cpp
// Unit tests for drone::util::LatencyTracker — IPC latency histogram utility.
#include "util/latency_tracker.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

using drone::util::LatencySummary;
using drone::util::LatencyTracker;

// ═══════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, DefaultConstruction) {
    LatencyTracker tracker;
    EXPECT_EQ(tracker.capacity(), 1024u);
    EXPECT_EQ(tracker.total_count(), 0u);
}

TEST(LatencyTrackerTest, CustomCapacityRoundedToPowerOfTwo) {
    LatencyTracker tracker(500);
    EXPECT_EQ(tracker.capacity(), 512u);  // next power of two

    LatencyTracker tracker2(1024);
    EXPECT_EQ(tracker2.capacity(), 1024u);  // already power of two

    LatencyTracker tracker3(1);
    EXPECT_EQ(tracker3.capacity(), 1u);
}

// ═══════════════════════════════════════════════════════════
// Record + Count
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, RecordIncrementsCount) {
    LatencyTracker tracker(64);
    for (int i = 0; i < 10; ++i) {
        tracker.record(1000 * i);
    }
    EXPECT_EQ(tracker.total_count(), 10u);
}

TEST(LatencyTrackerTest, RecordWrapsRingBuffer) {
    LatencyTracker tracker(4);  // capacity = 4
    for (int i = 0; i < 20; ++i) {
        tracker.record(i * 100);
    }
    EXPECT_EQ(tracker.total_count(), 20u);
    EXPECT_EQ(tracker.capacity(), 4u);
}

// ═══════════════════════════════════════════════════════════
// Summary — empty tracker
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, SummaryEmptyTracker) {
    LatencyTracker tracker;
    auto           s = tracker.summary();
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.min_ns, 0u);
    EXPECT_EQ(s.max_ns, 0u);
    EXPECT_DOUBLE_EQ(s.mean_ns, 0.0);
    EXPECT_EQ(s.p50_ns, 0u);
    EXPECT_EQ(s.p99_ns, 0u);
}

// ═══════════════════════════════════════════════════════════
// Summary — known data
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, SummarySingleSample) {
    LatencyTracker tracker(16);
    tracker.record(5000);
    auto s = tracker.summary();
    EXPECT_EQ(s.count, 1u);
    EXPECT_EQ(s.min_ns, 5000u);
    EXPECT_EQ(s.max_ns, 5000u);
    EXPECT_DOUBLE_EQ(s.mean_ns, 5000.0);
    EXPECT_EQ(s.p50_ns, 5000u);
    EXPECT_EQ(s.p99_ns, 5000u);
}

TEST(LatencyTrackerTest, SummaryUniformData) {
    LatencyTracker tracker(128);
    // Record 1..100 (each value = index * 1000 ns)
    for (uint64_t i = 1; i <= 100; ++i) {
        tracker.record(i * 1000);
    }
    auto s = tracker.summary();
    EXPECT_EQ(s.count, 100u);
    EXPECT_EQ(s.min_ns, 1000u);
    EXPECT_EQ(s.max_ns, 100'000u);

    // Mean of 1..100 = 50.5, × 1000 = 50500
    EXPECT_NEAR(s.mean_ns, 50'500.0, 1.0);

    // p50 ≈ 50000–51000 (interpolated)
    EXPECT_GE(s.p50_ns, 49'000u);
    EXPECT_LE(s.p50_ns, 52'000u);

    // p99 ≈ 99000–100000
    EXPECT_GE(s.p99_ns, 98'000u);
    EXPECT_LE(s.p99_ns, 100'000u);
}

TEST(LatencyTrackerTest, SummaryWithOutlier) {
    LatencyTracker tracker(256);
    // 99 samples at 1000 ns, 1 sample at 1,000,000 ns
    for (int i = 0; i < 99; ++i) {
        tracker.record(1000);
    }
    tracker.record(1'000'000);  // outlier

    auto s = tracker.summary();
    EXPECT_EQ(s.count, 100u);
    EXPECT_EQ(s.min_ns, 1000u);
    EXPECT_EQ(s.max_ns, 1'000'000u);
    EXPECT_EQ(s.p50_ns, 1000u);     // median is still 1000
    EXPECT_EQ(s.p90_ns, 1000u);     // 90th percentile still 1000
    EXPECT_GE(s.p99_ns, 1000u);     // 99th might interpolate to outlier
}

// ═══════════════════════════════════════════════════════════
// Percentile ordering
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, PercentilesAreMonotonic) {
    LatencyTracker tracker(256);
    for (uint64_t i = 0; i < 200; ++i) {
        tracker.record(i * 500);
    }
    auto s = tracker.summary();
    EXPECT_LE(s.min_ns, s.p50_ns);
    EXPECT_LE(s.p50_ns, s.p90_ns);
    EXPECT_LE(s.p90_ns, s.p95_ns);
    EXPECT_LE(s.p95_ns, s.p99_ns);
    EXPECT_LE(s.p99_ns, s.max_ns);
}

// ═══════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, ResetClearsState) {
    LatencyTracker tracker(64);
    for (int i = 0; i < 30; ++i) {
        tracker.record(i * 1000);
    }
    EXPECT_EQ(tracker.total_count(), 30u);

    tracker.reset();
    EXPECT_EQ(tracker.total_count(), 0u);
    auto s = tracker.summary();
    EXPECT_EQ(s.count, 0u);
}

TEST(LatencyTrackerTest, ResetAllowsReuse) {
    LatencyTracker tracker(32);
    tracker.record(100);
    tracker.record(200);
    tracker.reset();

    tracker.record(5000);
    tracker.record(10000);
    auto s = tracker.summary();
    EXPECT_EQ(s.count, 2u);
    EXPECT_EQ(s.min_ns, 5000u);
    EXPECT_EQ(s.max_ns, 10000u);
}

// ═══════════════════════════════════════════════════════════
// Conversion utilities
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, ConversionToUs) {
    EXPECT_DOUBLE_EQ(LatencySummary::to_us(1000), 1.0);
    EXPECT_DOUBLE_EQ(LatencySummary::to_us(1'500'000), 1500.0);
    EXPECT_DOUBLE_EQ(LatencySummary::to_us(0), 0.0);
}

TEST(LatencyTrackerTest, ConversionToMs) {
    EXPECT_DOUBLE_EQ(LatencySummary::to_ms(1'000'000), 1.0);
    EXPECT_DOUBLE_EQ(LatencySummary::to_ms(500'000), 0.5);
    EXPECT_DOUBLE_EQ(LatencySummary::to_ms(0), 0.0);
}

// ═══════════════════════════════════════════════════════════
// now_ns() utility
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, NowNsReturnsIncreasingValues) {
    auto t1 = LatencyTracker::now_ns();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    auto t2 = LatencyTracker::now_ns();
    EXPECT_GT(t2, t1);
    // Should be at least ~100µs = 100,000 ns apart
    EXPECT_GE(t2 - t1, 50'000u);  // allow some scheduling slack
}

// ═══════════════════════════════════════════════════════════
// log_summary_if_due
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, LogSummaryNotDueWithTooFewSamples) {
    LatencyTracker tracker(64);
    tracker.record(1000);
    tracker.record(2000);
    // min_samples defaults to 10 — should not log or reset
    EXPECT_FALSE(tracker.log_summary_if_due("test_topic"));
    EXPECT_EQ(tracker.total_count(), 2u);  // not reset
}

TEST(LatencyTrackerTest, LogSummaryDueWithEnoughSamples) {
    LatencyTracker tracker(64);
    for (int i = 0; i < 15; ++i) {
        tracker.record(1000 * i);
    }
    // Should log and reset
    EXPECT_TRUE(tracker.log_summary_if_due("test_topic"));
    EXPECT_EQ(tracker.total_count(), 0u);  // was reset
}

TEST(LatencyTrackerTest, LogSummaryCustomMinSamples) {
    LatencyTracker tracker(64);
    tracker.record(1000);
    tracker.record(2000);
    tracker.record(3000);
    // Custom: need only 3 samples
    EXPECT_TRUE(tracker.log_summary_if_due("test_topic", 3));
    EXPECT_EQ(tracker.total_count(), 0u);
}

// ═══════════════════════════════════════════════════════════
// Ring buffer wrapping correctness
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, WrappedBufferReportsMostRecentData) {
    LatencyTracker tracker(4);  // capacity = 4

    // Write 4 low values
    for (int i = 0; i < 4; ++i) {
        tracker.record(100);
    }

    // Overwrite with 4 high values
    for (int i = 0; i < 4; ++i) {
        tracker.record(9999);
    }

    // Summary should reflect only the 4 most recent (capacity-limited snapshot)
    auto s = tracker.summary();
    // The ring contains [9999, 9999, 9999, 9999] after wrapping
    EXPECT_EQ(s.min_ns, 9999u);
    EXPECT_EQ(s.max_ns, 9999u);
}

// ═══════════════════════════════════════════════════════════
// Stress: many samples
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, StressManyRecords) {
    LatencyTracker tracker(4096);
    for (uint64_t i = 0; i < 100'000; ++i) {
        tracker.record(i % 10'000);
    }
    EXPECT_EQ(tracker.total_count(), 100'000u);
    auto s = tracker.summary();
    EXPECT_GT(s.count, 0u);
    EXPECT_LT(s.min_ns, s.max_ns);
    EXPECT_LE(s.p50_ns, s.p90_ns);
}

// ═══════════════════════════════════════════════════════════
// Integration: measure real time
// ═══════════════════════════════════════════════════════════

TEST(LatencyTrackerTest, IntegrationRealLatency) {
    LatencyTracker tracker(64);
    for (int i = 0; i < 20; ++i) {
        auto start = LatencyTracker::now_ns();
        // Simulate some work (~10µs)
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        auto end = LatencyTracker::now_ns();
        tracker.record(end - start);
    }
    auto s = tracker.summary();
    EXPECT_EQ(s.count, 20u);
    // Each sample should be at least ~5µs (allowing scheduling variance)
    EXPECT_GE(s.p50_ns, 5'000u);
    // And less than 10ms (sanity — sleep(10µs) shouldn't take that long)
    EXPECT_LT(s.p99_ns, 10'000'000u);
}
