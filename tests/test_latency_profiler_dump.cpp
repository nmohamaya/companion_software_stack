// tests/test_latency_profiler_dump.cpp
//
// Smoke test for the profiler-wiring pattern used in process2_perception/src/main.cpp
// and process4_mission_planner/src/main.cpp (Issue #571 wiring PR).
//
// Does not link the real process binaries — that would require full Zenoh IPC
// plumbing and is covered by scenario integration tests. Instead, exercises
// the minimum contract: can we construct a LatencyProfiler, record a mix of
// stages from multiple threads, dump to a JSON file, and parse it back?
//
// If a future refactor breaks the "profiler -> JSON on disk" path, this test
// will catch it without depending on a full scenario run.

#include "util/latency_profiler.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace du = drone::util;
namespace fs = std::filesystem;

namespace {

fs::path make_tmp_dir(const std::string& leaf) {
    const fs::path dir = fs::temp_directory_path() / ("drone_test_profiler_dump_" + leaf);
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

// Simulate what a process's main() does: create a profiler, hand pointers to
// worker threads that record from each stage, then dump on "shutdown."
void simulate_perception_run(du::LatencyProfiler& p, int frames_per_thread) {
    auto thread_fn = [&p, frames_per_thread](const char* stage) {
        for (int i = 0; i < frames_per_thread; ++i) {
            du::ScopedLatency guard(p, stage);
            // Trivial busy-wait substitute for "work" — we only care that the
            // guard's dtor fires and records.
            volatile int x = 0;
            for (int k = 0; k < 100; ++k) x = x + k;
            (void)x;
        }
    };

    std::thread t_detect(thread_fn, "Detect");
    std::thread t_track(thread_fn, "Track");
    std::thread t_fuse(thread_fn, "Fuse");

    t_detect.join();
    t_track.join();
    t_fuse.join();
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// End-to-end: simulated worker threads → profiler → JSON on disk → parse
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfilerDump, SimulatedPerceptionRunProducesValidJsonOnDisk) {
    constexpr int  kFramesPerStage = 100;
    const fs::path tmp_dir         = make_tmp_dir("perception");

    du::LatencyProfiler profiler;
    simulate_perception_run(profiler, kFramesPerStage);

    // Dump JSON — the same pattern process2_perception/src/main.cpp uses.
    const fs::path out_path = tmp_dir / "latency_perception.json";
    {
        std::ofstream out(out_path);
        ASSERT_TRUE(out) << "Could not open " << out_path;
        out << profiler.to_json();
    }

    // Parse it back and check structure.
    std::ifstream in(out_path);
    ASSERT_TRUE(in) << "Could not read back " << out_path;
    const nlohmann::json js = nlohmann::json::parse(in);

    ASSERT_TRUE(js.contains("stages"));
    ASSERT_TRUE(js.contains("traces"));

    // All three stages should have accumulated kFramesPerStage records.
    for (const char* stage : {"Detect", "Track", "Fuse"}) {
        ASSERT_TRUE(js["stages"].contains(stage)) << "Missing stage " << stage;
        EXPECT_EQ(js["stages"][stage]["count"].get<uint64_t>(),
                  static_cast<uint64_t>(kFramesPerStage))
            << "Stage " << stage << " count mismatch";
        // Percentiles should all be non-zero since we did real (if trivial) work.
        EXPECT_GT(js["stages"][stage]["p50_ns"].get<uint64_t>(), 0U) << stage;
        EXPECT_GT(js["stages"][stage]["p99_ns"].get<uint64_t>(), 0U) << stage;
    }

    // Trace ring default capacity is 4096; our 3×100 = 300 records fit.
    EXPECT_EQ(js["traces"].size(), 3U * static_cast<std::size_t>(kFramesPerStage));

    fs::remove_all(tmp_dir);
}

TEST(LatencyProfilerDump, DisabledProfilerDoesNotCreateFile) {
    // Mirror the wiring pattern: when `benchmark.profiler.enabled` is false, we
    // never construct a profiler; the "if (benchmark_profiler)" guard in main()
    // skips the dump step entirely. This test asserts that choosing not to
    // emplace the optional is, in fact, the way to get zero output.
    const fs::path                     tmp_dir = make_tmp_dir("disabled");
    std::optional<du::LatencyProfiler> profiler;  // intentionally empty

    if (profiler) {
        // Dead path — the optional is empty. If this ever runs, the wiring
        // logic inverted somewhere.
        const fs::path path = tmp_dir / "latency.json";
        std::ofstream  out(path);
        out << profiler->to_json();
    }

    // Directory should be empty.
    std::size_t count = 0;
    for (const auto& _ : fs::directory_iterator(tmp_dir)) {
        (void)_;
        ++count;
    }
    EXPECT_EQ(count, 0U);

    fs::remove_all(tmp_dir);
}

// ────────────────────────────────────────────────────────────────────────────
// DR-022 invariant: on a simulated "flight-critical" concurrent workload, the
// per-stage count must equal the number of scopes that entered — a lost record
// would indicate a mutex / ordering bug in the wiring pattern.
// ────────────────────────────────────────────────────────────────────────────

TEST(LatencyProfilerDump, AllRecordsLandUnderConcurrentWorkers) {
    constexpr int       kThreads         = 6;  // P2 has 6 threads
    constexpr int       kFramesPerThread = 500;
    du::LatencyProfiler profiler;

    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&profiler, t]() {
            // Spread work across 3 stage names so the map actually has multiple entries.
            const char* const stages[] = {"Detect", "Track", "Fuse"};
            for (int i = 0; i < kFramesPerThread; ++i) {
                du::ScopedLatency g(profiler, stages[(t + i) % 3]);
            }
        });
    }
    for (auto& th : threads) th.join();

    const auto summaries = profiler.summaries();
    uint64_t   total     = 0;
    for (const auto& [stage, s] : summaries) {
        total += s.count;
    }
    EXPECT_EQ(total, static_cast<uint64_t>(kThreads * kFramesPerThread));
}
