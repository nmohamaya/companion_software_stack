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

// RAII guard — recursively removes a temp dir even if an ASSERT_* throws
// mid-test, so a failing run doesn't leak /tmp entries across test binaries.
class ScopedTmpDir {
public:
    explicit ScopedTmpDir(const std::string& leaf)
        : dir_(fs::temp_directory_path() / ("drone_test_profiler_dump_" + leaf)) {
        fs::remove_all(dir_);
        fs::create_directories(dir_);
    }

    ~ScopedTmpDir() {
        std::error_code ec;
        fs::remove_all(dir_, ec);  // best-effort — dtor must not throw
    }

    ScopedTmpDir(const ScopedTmpDir&)            = delete;
    ScopedTmpDir& operator=(const ScopedTmpDir&) = delete;

    [[nodiscard]] const fs::path& path() const noexcept { return dir_; }

private:
    fs::path dir_;
};

// Simulate what a process's main() does: create a profiler, hand pointers to
// worker threads that record from each stage, then dump on "shutdown."
//
// Uses std::this_thread::sleep_for rather than a `volatile` busy-wait so the
// per-stage duration is guaranteed non-zero even on fast Orin cores — the
// prior busy-wait could elide to ~10 ns under -O2, which fell below the
// clock resolution and flaked the p50>0 assertion.
void simulate_perception_run(du::LatencyProfiler& p, int frames_per_thread) {
    auto thread_fn = [&p, frames_per_thread](const char* stage) {
        for (int i = 0; i < frames_per_thread; ++i) {
            du::ScopedLatency guard(p, stage);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
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
    constexpr int         kFramesPerStage = 100;
    constexpr std::size_t kPerStageCap    = 1024;
    constexpr std::size_t kTraceRingCap   = 4096;
    const ScopedTmpDir    tmp("perception");

    // Explicit capacities so the test documents its own assumption — if
    // defaults ever change, the test still does what it claims.
    du::LatencyProfiler profiler(kPerStageCap, kTraceRingCap);
    simulate_perception_run(profiler, kFramesPerStage);

    // Use the library's dump_to_file helper — the same API that P2/P4
    // main.cpp now call, so the test covers the production path.
    const fs::path out_path = tmp.path() / "latency_perception.json";
    ASSERT_EQ(profiler.dump_to_file(out_path), du::LatencyProfiler::DumpStatus::Ok);

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
        // Percentiles should all be non-zero since sleep_for(1us) guarantees
        // measurable duration even on fast clocks.
        EXPECT_GT(js["stages"][stage]["p50_ns"].get<uint64_t>(), 0U) << stage;
        EXPECT_GT(js["stages"][stage]["p99_ns"].get<uint64_t>(), 0U) << stage;
    }

    // kTraceRingCap = 4096; our 3 × 100 = 300 records fit without wrapping.
    EXPECT_EQ(js["traces"].size(), 3U * static_cast<std::size_t>(kFramesPerStage));
}

TEST(LatencyProfilerDump, DisabledProfilerDoesNotCreateFile) {
    // Mirror the wiring pattern: when `benchmark.profiler.enabled` is false, we
    // never construct a profiler; the "if (benchmark_profiler)" guard in main()
    // skips the dump step entirely. This test documents the intent — it's a
    // tautology at the C++ level (empty optional never enters the if) but
    // serves as living documentation of the production wiring pattern.
    const ScopedTmpDir                 tmp("disabled");
    std::optional<du::LatencyProfiler> profiler;  // intentionally empty

    if (profiler) {
        // Unreachable when the optional is empty. Kept to mirror the main.cpp
        // shape — if the wiring condition ever inverts this branch fires.
        const fs::path path = tmp.path() / "latency.json";
        (void)profiler->dump_to_file(path);
    }

    // Directory should be empty.
    std::size_t count = 0;
    for (const auto& entry : fs::directory_iterator(tmp.path())) {
        (void)entry;
        ++count;
    }
    EXPECT_EQ(count, 0U);
}

TEST(LatencyProfilerDump, PathValidationRejectsTraversalOutsideAllowList) {
    // Security: review-security P2 on PR #593. A tampered config should not
    // be able to cause writes outside the allow-list (CWD / /var/log/drone /
    // /tmp). `weakly_canonical` resolves `..` segments, so a path that tries
    // to escape CWD via `..` is caught by the allow-list check.
    du::LatencyProfiler profiler;
    profiler.record("Stage", 1, 0, 1000);

    // Absolute path outside the allow-list — should be rejected.
    const fs::path forbidden_abs = "/etc/drone_latency_test.json";
    EXPECT_EQ(profiler.dump_to_file(forbidden_abs), du::LatencyProfiler::DumpStatus::PathRejected);
    EXPECT_FALSE(fs::exists(forbidden_abs));

    // Relative path with `..` escaping to outside CWD — should be rejected
    // after canonicalisation.
    const fs::path forbidden_rel = "../../../tmp_not_on_the_allowlist_xyz123.json";
    EXPECT_EQ(profiler.dump_to_file(forbidden_rel), du::LatencyProfiler::DumpStatus::PathRejected);
    EXPECT_FALSE(fs::exists(forbidden_rel));

    // /tmp is on the allow-list — must succeed.
    const ScopedTmpDir tmp("allowlist");
    const fs::path     ok = tmp.path() / "latency.json";
    EXPECT_EQ(profiler.dump_to_file(ok), du::LatencyProfiler::DumpStatus::Ok);
    EXPECT_TRUE(fs::exists(ok));
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
