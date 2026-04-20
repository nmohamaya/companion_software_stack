// tests/test_latency_profiler.cpp
//
// Unit tests for the LatencyProfiler (Issue #571, Epic #523).

#include "util/correlation.h"
#include "util/latency_profiler.h"
#include "util/mock_clock.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace du = drone::util;


// ────────────────────────────────────────────────────────────────────────────
// Basic record / summary / reset
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, EmptyProfilerHasNoStagesNoTraces) {
    du::LatencyProfiler p;
    EXPECT_EQ(p.stage_count(), 0U);
    EXPECT_EQ(p.trace_count(), 0U);
    EXPECT_TRUE(p.summaries().empty());
    EXPECT_TRUE(p.traces().empty());
}

TEST(LatencyProfiler, RecordPopulatesPerStageTracker) {
    du::LatencyProfiler p;
    p.record("detector", /*corr=*/1, /*start_ns=*/100, /*duration_ns=*/500);
    p.record("detector", /*corr=*/2, /*start_ns=*/200, /*duration_ns=*/700);
    p.record("tracker", /*corr=*/1, /*start_ns=*/600, /*duration_ns=*/300);

    const auto s = p.summaries();
    ASSERT_EQ(s.size(), 2U);
    EXPECT_EQ(s.at("detector").count, 2U);
    EXPECT_EQ(s.at("detector").min_ns, 500U);
    EXPECT_EQ(s.at("detector").max_ns, 700U);
    EXPECT_EQ(s.at("tracker").count, 1U);
    EXPECT_EQ(s.at("tracker").min_ns, 300U);
    EXPECT_EQ(s.at("tracker").max_ns, 300U);
}

TEST(LatencyProfiler, ResetClearsEverything) {
    du::LatencyProfiler p;
    p.record("detector", 1, 100, 500);
    p.record("tracker", 1, 600, 300);
    ASSERT_EQ(p.stage_count(), 2U);
    p.reset();
    EXPECT_EQ(p.stage_count(), 0U);
    EXPECT_EQ(p.trace_count(), 0U);
    EXPECT_TRUE(p.summaries().empty());
}

// ────────────────────────────────────────────────────────────────────────────
// Percentile correctness — inject known timings, verify sorted percentiles
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, PercentilesMatchInjectedTimings) {
    // Inject durations 1..100 ns. p50 should be ~50, p90 ~90, p99 ~99.
    du::LatencyProfiler p;
    for (uint64_t i = 1; i <= 100; ++i) {
        p.record("stage", /*corr=*/0, /*start_ns=*/0, i);
    }
    const auto s = p.summaries().at("stage");
    EXPECT_EQ(s.count, 100U);
    EXPECT_EQ(s.min_ns, 1U);
    EXPECT_EQ(s.max_ns, 100U);
    // LatencyTracker uses linear interpolation: rank(p) = p/100 * (N-1) with
    // sorted[idx] * (1-frac) + sorted[idx+1] * frac, truncated to uint64_t.
    // For N=100 sorted 1..100:
    //   p50: rank=49.5  → 50 * 0.5 + 51 * 0.5 = 50.5 → 50
    //   p90: rank=89.1  → 90 * 0.9 + 91 * 0.1 = 90.1 → 90
    //   p95: rank=94.05 → 95 * 0.95 + 96 * 0.05 = 95.05 → 95
    //   p99: rank=98.01 → 99 * 0.99 + 100 * 0.01 = 99.01 → 99
    // All exact — assert equality so an off-by-one rank-formula regression is caught.
    EXPECT_EQ(s.p50_ns, 50U);
    EXPECT_EQ(s.p90_ns, 90U);
    EXPECT_EQ(s.p95_ns, 95U);
    EXPECT_EQ(s.p99_ns, 99U);
}

// ────────────────────────────────────────────────────────────────────────────
// Correlation-ID trace attachment
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, TracesCarryCorrelationIdsInOrder) {
    du::LatencyProfiler p(/*per_stage_capacity=*/128, /*trace_ring_capacity=*/8);
    p.record("detector", 10, 100, 50);
    p.record("detector", 11, 200, 60);
    p.record("tracker", 10, 150, 40);
    p.record("grid", 10, 200, 25);

    const auto t = p.traces();
    ASSERT_EQ(t.size(), 4U);
    EXPECT_EQ(t[0].correlation_id, 10U);
    EXPECT_EQ(t[0].stage, "detector");
    EXPECT_EQ(t[1].correlation_id, 11U);
    EXPECT_EQ(t[2].correlation_id, 10U);
    EXPECT_EQ(t[2].stage, "tracker");
    EXPECT_EQ(t[3].stage, "grid");
}

TEST(LatencyProfiler, TraceRingWrapsOnOverflow) {
    // Capacity 4; write 10 records. Should retain the 4 most recent.
    du::LatencyProfiler p(/*per_stage_capacity=*/128, /*trace_ring_capacity=*/4);
    for (uint64_t i = 0; i < 10; ++i) {
        p.record("s", /*corr=*/100 + i, /*start_ns=*/i * 10, /*duration_ns=*/i);
    }
    const auto t = p.traces();
    ASSERT_EQ(t.size(), 4U);
    // Oldest-first ordering: records 6,7,8,9.
    EXPECT_EQ(t[0].correlation_id, 106U);
    EXPECT_EQ(t[3].correlation_id, 109U);
    // Per-stage tracker keeps count of all 10.
    EXPECT_EQ(p.summaries().at("s").count, 10U);
}

TEST(LatencyProfiler, TraceRingCapacityZeroDisablesTracing) {
    du::LatencyProfiler p(/*per_stage_capacity=*/128, /*trace_ring_capacity=*/0);
    p.record("s", 1, 0, 100);
    p.record("s", 2, 0, 200);
    EXPECT_TRUE(p.traces().empty());
    // Per-stage aggregation still works.
    EXPECT_EQ(p.summaries().at("s").count, 2U);
}

// ────────────────────────────────────────────────────────────────────────────
// ScopedLatency RAII timer
// ────────────────────────────────────────────────────────────────────────────

TEST(ScopedLatency, RecordsDurationOnDestruction) {
    du::ScopedMockClock clock_guard;
    du::LatencyProfiler p;
    {
        du::ScopedLatency s(p, "detector");
        clock_guard.mock().advance_ns(42'000);  // 42 µs
    }
    const auto sum = p.summaries().at("detector");
    EXPECT_EQ(sum.count, 1U);
    EXPECT_EQ(sum.min_ns, 42'000U);
}

TEST(ScopedLatency, CapturesCorrelationIdAtConstruction) {
    du::ScopedMockClock clock_guard;
    du::LatencyProfiler p;
    du::CorrelationContext::set(0xAABBCCDD);
    // Always clear thread-local state on test exit — ASSERT_* above would
    // skip a naked clear() at the end of the test body.
    struct CorrelationCleanup {
        ~CorrelationCleanup() { du::CorrelationContext::clear(); }
    } cleanup;
    const uint64_t expected_start = du::get_clock().now_ns();
    {
        du::ScopedLatency s(p, "detector");
        // Change the correlation ID mid-scope. The trace should still record
        // the value captured at construction.
        du::CorrelationContext::set(0x11223344);
        // Exercise the public getters while the scope is live.
        EXPECT_EQ(s.correlation_id(), 0xAABBCCDDULL);
        EXPECT_EQ(s.start_ns(), expected_start);
        clock_guard.mock().advance_ns(10'000);
    }
    const auto t = p.traces();
    ASSERT_EQ(t.size(), 1U);
    EXPECT_EQ(t[0].correlation_id, 0xAABBCCDDULL);
}

TEST(ScopedLatency, ZeroDurationIfClockNonMonotonic) {
    // Defensive: if a wayward clock rolls backward, ScopedLatency should
    // record zero rather than a wrapped gigantic uint64_t.
    du::ScopedMockClock clock_guard;
    clock_guard.mock().set_ns(1'000'000);
    du::LatencyProfiler p;
    {
        du::ScopedLatency s(p, "stage");
        clock_guard.mock().set_ns(500'000);  // time goes backward
    }
    const auto summary = p.summaries().at("stage");
    EXPECT_EQ(summary.count, 1U);   // record must land — not silently dropped
    EXPECT_EQ(summary.min_ns, 0U);  // clamped to zero, not wrapped to 2^64
}

// ────────────────────────────────────────────────────────────────────────────
// Thread safety — multiple writers hammering the profiler concurrently
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, ThreadSafeConcurrentWrites) {
    du::LatencyProfiler      p(/*per_stage_capacity=*/4096, /*trace_ring_capacity=*/4096);
    constexpr int            kThreads          = 4;
    constexpr int            kRecordsPerThread = 500;
    std::atomic<bool>        go{false};
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t]() {
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            for (int i = 0; i < kRecordsPerThread; ++i) {
                const std::string stage = (i % 2 == 0) ? "detector" : "tracker";
                p.record(stage, /*corr=*/static_cast<uint64_t>(t * 10'000 + i),
                         /*start_ns=*/0, /*duration_ns=*/static_cast<uint64_t>(t * 1000 + i));
            }
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& th : threads) th.join();

    const auto s   = p.summaries();
    const auto det = s.at("detector").count;
    const auto trk = s.at("tracker").count;
    EXPECT_EQ(det + trk, static_cast<uint64_t>(kThreads * kRecordsPerThread));
    // Trace ring capacity (4096) is > kThreads*kRecordsPerThread (2000), so
    // every record must have landed in the trace ring too.
    EXPECT_EQ(p.trace_count(), static_cast<std::size_t>(kThreads * kRecordsPerThread));
}

TEST(LatencyProfiler, ConcurrentReadersDoNotRaceWriters) {
    // Writers hammer record() while a reader spins on summaries()+traces()+to_json().
    // Proves the mutex covers the read path — a TSan-enabled build would catch
    // any unguarded access to the shared state.
    du::LatencyProfiler      p(/*per_stage_capacity=*/1024, /*trace_ring_capacity=*/2048);
    constexpr int            kWriters          = 3;
    constexpr int            kRecordsPerWriter = 300;
    std::atomic<bool>        go{false};
    std::atomic<bool>        stop{false};
    std::atomic<int>         reader_iterations{0};
    std::vector<std::thread> writers;
    for (int t = 0; t < kWriters; ++t) {
        writers.emplace_back([&, t]() {
            while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
            for (int i = 0; i < kRecordsPerWriter; ++i) {
                p.record("stage", static_cast<uint64_t>(t * 10'000 + i), 0,
                         static_cast<uint64_t>(i + 1));
            }
        });
    }
    std::thread reader([&]() {
        while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
        while (!stop.load(std::memory_order_acquire)) {
            const auto s = p.summaries();  // NOLINT(readability-redundant-declaration)
            const auto t = p.traces();
            const auto j = p.to_json();
            (void)s;
            (void)t;
            (void)j;
            reader_iterations.fetch_add(1, std::memory_order_release);
        }
    });

    go.store(true, std::memory_order_release);
    for (auto& w : writers) w.join();
    stop.store(true, std::memory_order_release);
    reader.join();

    EXPECT_EQ(p.summaries().at("stage").count, static_cast<uint64_t>(kWriters * kRecordsPerWriter));
    EXPECT_GT(reader_iterations.load(), 0);
}

// ────────────────────────────────────────────────────────────────────────────
// JSON snapshot — round-trips through nlohmann/json for validity checking
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, ToJsonParsesAsValidJson) {
    du::LatencyProfiler p(/*per_stage_capacity=*/128, /*trace_ring_capacity=*/4);
    p.record("detector", 1, 100, 500);
    p.record("tracker", 1, 600, 250);
    const auto js = nlohmann::json::parse(p.to_json());

    ASSERT_TRUE(js.contains("stages"));
    ASSERT_TRUE(js.contains("traces"));
    EXPECT_TRUE(js["stages"].contains("detector"));
    EXPECT_TRUE(js["stages"].contains("tracker"));
    EXPECT_EQ(js["stages"]["detector"]["count"].get<uint64_t>(), 1U);
    EXPECT_EQ(js["traces"].size(), 2U);
    EXPECT_EQ(js["traces"][0]["stage"].get<std::string>(), "detector");
    EXPECT_EQ(js["traces"][0]["correlation_id"].get<uint64_t>(), 1U);
}

TEST(LatencyProfiler, ToJsonEmptyProfilerIsValid) {
    du::LatencyProfiler p;
    const auto          js = nlohmann::json::parse(p.to_json());
    ASSERT_TRUE(js.contains("stages"));
    ASSERT_TRUE(js.contains("traces"));
    EXPECT_TRUE(js["stages"].empty());
    EXPECT_TRUE(js["traces"].empty());
}

TEST(LatencyProfiler, ToJsonEscapesStageNames) {
    // Cover the full escape set: quote, backslash, tab, newline, and a control
    // char below 0x20 (which must be emitted as \uXXXX per RFC 8259).
    du::LatencyProfiler p;
    const std::string   tricky = std::string("q\"\\t\nx\x01y");
    p.record(tricky, 1, 0, 100);
    const auto js = nlohmann::json::parse(p.to_json());
    // Parser round-trips the escaped characters back to the original bytes.
    EXPECT_EQ(js["traces"][0]["stage"].get<std::string>(), tricky);
}

TEST(LatencyProfiler, ToJsonStagesAreLexicographicallyOrdered) {
    du::LatencyProfiler p;
    p.record("zoo", 1, 0, 10);
    p.record("aaa", 2, 0, 20);
    p.record("mid", 3, 0, 30);
    const std::string js  = p.to_json();
    const auto        aaa = js.find("\"aaa\"");
    const auto        mid = js.find("\"mid\"");
    const auto        zoo = js.find("\"zoo\"");
    ASSERT_NE(aaa, std::string::npos);
    ASSERT_NE(mid, std::string::npos);
    ASSERT_NE(zoo, std::string::npos);
    EXPECT_LT(aaa, mid);
    EXPECT_LT(mid, zoo);
}

// ────────────────────────────────────────────────────────────────────────────
// Overhead budget — ScopedLatency around a no-op must be cheap.
// Issue AC: < 2% of pipeline tick time. We assert an absolute per-record
// ceiling that is ~20× below a 30 Hz pipeline tick budget.
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfiler, OverheadUnderBudget) {
    du::LatencyProfiler p(/*per_stage_capacity=*/4096, /*trace_ring_capacity=*/4096);
    constexpr int       n = 10'000;

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) {
        du::ScopedLatency s(p, "detector");
        // empty scope — we're measuring the guard itself
    }
    const auto   t1       = std::chrono::steady_clock::now();
    const auto   total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    const double per_ns   = static_cast<double>(total_ns) / n;

    // 2 % of a 33 ms tick is 660 µs; per record should be orders below that.
    // Measured cost on the dev laptop: ~300-800 ns/record (2× now_ns() + mutex
    // + heterogeneous map find + string_view assign into a short stage name).
    // A 2000 ns ceiling leaves headroom for slower targets (Orin Nano, noisy CI
    // runners) while still catching a ~3-4× regression from a stale mutex or
    // per-call heap allocation. If this fires spuriously on a loaded runner,
    // raise to 3000 — but do not move past 5000 without investigation.
    EXPECT_LT(per_ns, 2'000.0) << "ScopedLatency cost " << per_ns
                               << " ns/record (target < 2000 ns)";
    EXPECT_EQ(p.summaries().at("detector").count, static_cast<uint64_t>(n));
}
