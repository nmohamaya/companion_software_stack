// common/util/include/util/latency_profiler.h
//
// Per-stage latency profiler for the perception benchmark harness (Issue #571,
// Epic #523). Extends the single-tracker LatencyTracker with:
//   - Per-stage percentile aggregation (stages keyed by name)
//   - End-to-end trace ring (correlation_id → stage durations)
//   - Thread-safe recording across pipeline threads
//   - JSON dump for the benchmark-harness baseline file
//
// Usage (scoped RAII timer):
//
//   drone::util::LatencyProfiler profiler;
//
//   void PerceptionThread::tick() {
//       drone::util::ScopedLatency guard(profiler, "detector");
//       run_detector();           // guard records duration on scope exit
//   }
//
//   // Periodic / on-demand dump:
//   auto summaries = profiler.summaries();   // map<stage, LatencySummary>
//   std::string json = profiler.to_json();   // serializable snapshot
//
// Threading: `record` / `summaries` / `traces` / `to_json` are all thread-safe
// via an internal mutex. Contention is minimal at realistic pipeline rates
// (per-tick recording at ~30 Hz across a handful of threads). For hot inner
// loops (>10 kHz), prefer the single-threaded LatencyTracker directly and
// merge off-line.
//
// Overhead target: < 2 % of pipeline tick time on an Orin Nano (issue AC).

#pragma once

#include "util/correlation.h"
#include "util/iclock.h"
#include "util/latency_tracker.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace drone::util {

/// A single end-to-end trace record: one (stage, correlation_id, duration) tuple.
///
/// Traces are kept in a bounded ring so a long-running process does not grow
/// memory without bound. Consumers that want per-frame traces should call
/// `LatencyProfiler::traces()` periodically.
struct LatencyTrace {
    std::string stage{};
    uint64_t    correlation_id{0};
    uint64_t    start_ns{0};
    uint64_t    duration_ns{0};
};

/// Combines per-stage percentile aggregation with a bounded correlation-tagged
/// trace ring. Designed for the perception benchmark harness.
class LatencyProfiler {
public:
    /// Default ring size of each per-stage LatencyTracker (samples).
    static constexpr std::size_t kDefaultPerStageCapacity = 1024;

    /// Default ring size of the end-to-end trace buffer (records).
    /// Pass 0 to the constructor to disable trace recording while still
    /// aggregating per-stage percentiles.
    static constexpr std::size_t kDefaultTraceRingCapacity = 4096;

    /// @param per_stage_capacity  Ring size of each per-stage tracker (samples).
    /// @param trace_ring_capacity Ring size of the trace buffer (records). Pass 0
    ///                            to disable trace recording — per-stage
    ///                            aggregation still functions.
    explicit LatencyProfiler(std::size_t per_stage_capacity  = kDefaultPerStageCapacity,
                             std::size_t trace_ring_capacity = kDefaultTraceRingCapacity)
        : per_stage_capacity_(per_stage_capacity)
        , trace_ring_(trace_ring_capacity)
        , trace_capacity_(trace_ring_capacity) {}

    // Non-copyable, non-movable — owns a mutex.
    LatencyProfiler(const LatencyProfiler&)                = delete;
    LatencyProfiler& operator=(const LatencyProfiler&)     = delete;
    LatencyProfiler(LatencyProfiler&&) noexcept            = delete;
    LatencyProfiler& operator=(LatencyProfiler&&) noexcept = delete;
    ~LatencyProfiler()                                     = default;

    /// Record a single (stage, correlation_id, duration) observation.
    /// Called directly or (more commonly) via ScopedLatency's destructor.
    /// Thread-safe.
    void record(std::string_view stage, uint64_t correlation_id, uint64_t start_ns,
                uint64_t duration_ns) {
        const std::lock_guard<std::mutex> lock(mtx_);
        // Heterogeneous lookup (std::less<>) lets us search the map with a
        // string_view without allocating a temporary std::string on every hit.
        // On miss, we still allocate once to seed the key — amortised O(1)
        // after warmup.
        auto it = stage_trackers_.find(stage);
        if (it == stage_trackers_.end()) {
            it = stage_trackers_.try_emplace(std::string(stage), per_stage_capacity_).first;
        }
        it->second.record(duration_ns);

        if (trace_capacity_ == 0) {
            return;
        }
        LatencyTrace& slot = trace_ring_[trace_write_pos_ % trace_capacity_];
        slot.stage.assign(stage.data(), stage.size());
        slot.correlation_id = correlation_id;
        slot.start_ns       = start_ns;
        slot.duration_ns    = duration_ns;
        ++trace_write_pos_;
    }

    /// Per-stage summary snapshot.
    /// Thread-safe; computes sorted percentiles on a snapshot.
    [[nodiscard]] std::map<std::string, LatencySummary> summaries() const {
        const std::lock_guard<std::mutex>     lock(mtx_);
        std::map<std::string, LatencySummary> out;
        for (const auto& [stage, tracker] : stage_trackers_) {
            out.emplace(stage, tracker.summary());
        }
        return out;
    }

    /// Snapshot of the trace ring, oldest-first. Thread-safe.
    [[nodiscard]] std::vector<LatencyTrace> traces() const {
        const std::lock_guard<std::mutex> lock(mtx_);
        return collect_traces_locked();
    }

    /// Stable JSON snapshot: per-stage summaries + bounded trace ring.
    /// Intended for the benchmark-harness baseline file. Ordering is stable:
    ///   - `stages` — std::map sorts stage keys lexicographically.
    ///   - `traces` — oldest-first (matches `traces()`).
    /// Snapshots both under a single lock acquisition so the stages block and
    /// the traces array are guaranteed consistent (no trace can reference a
    /// stage whose aggregate count hasn't yet included it).
    [[nodiscard]] std::string to_json() const {
        std::map<std::string, LatencySummary> summaries_snap;
        std::vector<LatencyTrace>             traces_snap;
        {
            const std::lock_guard<std::mutex> lock(mtx_);
            for (const auto& [stage, tracker] : stage_trackers_) {
                summaries_snap.emplace(stage, tracker.summary());
            }
            traces_snap = collect_traces_locked();
        }

        std::ostringstream os;
        os << "{\n";
        os << "  \"stages\": {";
        bool first = true;
        for (const auto& [stage, s] : summaries_snap) {
            os << (first ? "\n" : ",\n");
            first = false;
            os << "    \"" << escape_json(stage) << "\": {";
            os << "\"count\": " << s.count;
            os << ", \"window_size\": " << s.window_size;
            os << ", \"min_ns\": " << s.min_ns;
            os << ", \"max_ns\": " << s.max_ns;
            os << ", \"mean_ns\": " << static_cast<uint64_t>(s.mean_ns);
            os << ", \"p50_ns\": " << s.p50_ns;
            os << ", \"p90_ns\": " << s.p90_ns;
            os << ", \"p95_ns\": " << s.p95_ns;
            os << ", \"p99_ns\": " << s.p99_ns;
            os << "}";
        }
        os << (summaries_snap.empty() ? "" : "\n  ");
        os << "},\n";
        os << "  \"traces\": [";
        first = true;
        for (const auto& t : traces_snap) {
            os << (first ? "\n" : ",\n");
            first = false;
            os << "    {";
            os << "\"stage\": \"" << escape_json(t.stage) << "\"";
            os << ", \"correlation_id\": " << t.correlation_id;
            os << ", \"start_ns\": " << t.start_ns;
            os << ", \"duration_ns\": " << t.duration_ns;
            os << "}";
        }
        os << (traces_snap.empty() ? "" : "\n  ");
        os << "]\n";
        os << "}\n";
        return os.str();
    }

    /// Result of `dump_to_file`.
    enum class DumpStatus : uint8_t {
        Ok,                     // Wrote JSON to disk.
        PathRejected,           // Path canonicalised outside the allow-list (security).
        DirectoryCreateFailed,  // filesystem::create_directories returned an error.
        OpenFailed,             // ofstream failed to open.
    };

    /// Write `to_json()` output to `path`. Performs:
    ///   1. Path validation — `path` is canonicalised and must resolve to either
    ///      (a) inside the current working directory (dev/local runs), or
    ///      (b) under one of a compile-time production allow-list
    ///          (`/var/log/drone`, `/tmp`). Paths with `..` traversal or
    ///          outside the allow-list are rejected — no I/O performed.
    ///   2. Create the parent directory (best-effort, ignores "already exists").
    ///   3. Open the file and stream `to_json()` output.
    ///
    /// Thread-safe. Does not acquire the profiler mutex during file I/O —
    /// only during the internal `to_json()` snapshot (which takes the mutex
    /// for as long as the two-phase snapshot takes, then releases).
    ///
    /// Consolidates the dump logic that both P2 and P4 main.cpp used to
    /// duplicate. See DR-022 for the flight-critical-thread justification and
    /// DR-024 for the path-validation allow-list rationale.
    [[nodiscard]] DumpStatus dump_to_file(const std::filesystem::path& path) const {
        if (!is_safe_output_path(path)) {
            return DumpStatus::PathRejected;
        }
        std::error_code ec;
        const auto      parent = path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
            if (ec) {
                return DumpStatus::DirectoryCreateFailed;
            }
        }
        std::ofstream out(path);
        if (!out) {
            return DumpStatus::OpenFailed;
        }
        out << to_json();
        return DumpStatus::Ok;
    }

    /// Human-readable error string for a `DumpStatus`. Used by the P2/P4 main
    /// logging paths to produce consistent WARN/ERROR messages.
    [[nodiscard]] static const char* describe(DumpStatus s) noexcept {
        switch (s) {
            case DumpStatus::Ok: return "ok";
            case DumpStatus::PathRejected:
                return "path rejected: canonicalises outside the allow-list "
                       "(CWD / /var/log/drone / /tmp)";
            case DumpStatus::DirectoryCreateFailed: return "create_directories failed";
            case DumpStatus::OpenFailed: return "ofstream open failed";
        }
        return "unknown";
    }

    /// Clear all per-stage state and the trace ring. Thread-safe.
    void reset() {
        const std::lock_guard<std::mutex> lock(mtx_);
        stage_trackers_.clear();
        trace_write_pos_ = 0;
        for (auto& t : trace_ring_) {
            t = LatencyTrace{};
        }
    }

    /// Introspection helpers.
    [[nodiscard]] std::size_t stage_count() const {
        const std::lock_guard<std::mutex> lock(mtx_);
        return stage_trackers_.size();
    }

    [[nodiscard]] std::size_t trace_count() const {
        const std::lock_guard<std::mutex> lock(mtx_);
        return trace_write_pos_ < trace_capacity_ ? trace_write_pos_ : trace_capacity_;
    }

private:
    // Return true iff `p` resolves under the current working directory OR
    // under a compile-time production allow-list (/var/log/drone, /tmp).
    // Rejects `..`-traversal by canonicalising first — a relative path like
    // "drone_logs/../../etc/cron.d" canonicalises to an absolute path outside
    // the CWD, which then fails the allow-list check.
    //
    // This is a security boundary: `benchmark.profiler.output_dir` comes from
    // config, and a tampered config must not be able to cause file writes to
    // arbitrary locations. See DR-024 and review-security P2 on PR #593.
    [[nodiscard]] static bool is_safe_output_path(const std::filesystem::path& p) {
        std::error_code ec;
        // weakly_canonical handles non-existent paths (we may be creating the
        // file) while still resolving `..` segments, unlike canonical which
        // requires the path to exist.
        const auto resolved = std::filesystem::weakly_canonical(std::filesystem::absolute(p, ec),
                                                                ec);
        if (ec) {
            return false;
        }

        const auto path_is_under = [](const std::filesystem::path& candidate,
                                      const std::filesystem::path& root) {
            const std::string cs = candidate.string();
            const std::string rs = root.string();
            if (cs.size() < rs.size()) return false;
            if (cs.compare(0, rs.size(), rs) != 0) return false;
            // Must be exactly the root or followed by a separator —
            // prevents "/etc/foo" matching "/etc/f" as a prefix.
            return cs.size() == rs.size() || cs[rs.size()] == '/';
        };

        const auto cwd = std::filesystem::current_path(ec);
        if (!ec && path_is_under(resolved, cwd)) {
            return true;
        }

        static const std::array<std::filesystem::path, 2> kSafeRoots = {
            std::filesystem::path{"/var/log/drone"},
            std::filesystem::path{"/tmp"},
        };
        for (const auto& root : kSafeRoots) {
            if (path_is_under(resolved, root)) {
                return true;
            }
        }
        return false;
    }

    // Walk the trace ring oldest-first. Caller must hold `mtx_`.
    [[nodiscard]] std::vector<LatencyTrace> collect_traces_locked() const {
        if (trace_capacity_ == 0) {
            return {};
        }
        const std::size_t         n = trace_write_pos_ < trace_capacity_ ? trace_write_pos_
                                                                         : trace_capacity_;
        std::vector<LatencyTrace> out;
        out.reserve(n);
        const std::size_t start =
            trace_write_pos_ < trace_capacity_ ? 0 : trace_write_pos_ % trace_capacity_;
        for (std::size_t i = 0; i < n; ++i) {
            out.push_back(trace_ring_[(start + i) % trace_capacity_]);
        }
        return out;
    }

    // Escape a string for JSON embedding per RFC 8259 §7: quote, backslash,
    // the short-form whitespace escapes, and \uXXXX for any remaining control
    // character in 0x00..0x1F. Forward slash is intentionally NOT escaped
    // (RFC 8259 permits but does not require it and leaving it readable
    // keeps the baseline diffs skim-friendly).
    static std::string escape_json(std::string_view s) {
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            const auto uc = static_cast<unsigned char>(c);
            switch (c) {
                case '"': out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\n': out.append("\\n"); break;
                case '\r': out.append("\\r"); break;
                case '\t': out.append("\\t"); break;
                case '\b': out.append("\\b"); break;
                case '\f': out.append("\\f"); break;
                default:
                    if (uc < 0x20U) {
                        std::ostringstream hex;
                        hex << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<unsigned>(uc);
                        out.append(hex.str());
                    } else {
                        out.push_back(c);
                    }
                    break;
            }
        }
        return out;
    }

    // Heterogeneous comparator (`std::less<>`) so lookups can take `string_view`
    // without allocating a temporary `std::string` on every record() call.
    using StageMap = std::map<std::string, LatencyTracker, std::less<>>;

    mutable std::mutex        mtx_;
    std::size_t               per_stage_capacity_;
    StageMap                  stage_trackers_;
    std::vector<LatencyTrace> trace_ring_;
    std::size_t               trace_capacity_;
    std::size_t               trace_write_pos_ = 0;
};

/// RAII timer that records a stage latency to a LatencyProfiler on destruction.
///
/// The correlation ID is captured at construction time from the thread-local
/// CorrelationContext — so the ID attached to the record is the one that was
/// active when the work began, even if it changes before scope exit.
///
/// Lifetime contract: a ScopedLatency must not outlive the LatencyProfiler it
/// references. The class is non-movable/non-copyable to keep the scope tied to
/// a single stack frame, which makes the contract trivially true for the
/// intended usage.
class ScopedLatency {
public:
    ScopedLatency(LatencyProfiler& profiler, std::string_view stage)
        : profiler_(profiler)
        , stage_(stage)
        , correlation_id_(CorrelationContext::get())
        , start_ns_(now_ns()) {}

    ~ScopedLatency() {
        const uint64_t end_ns = now_ns();
        // Defensive: a wayward clock rolling backward must record zero rather
        // than wrapping to ~2^64 ns (unsigned subtraction underflow).
        const uint64_t duration_ns = end_ns > start_ns_ ? end_ns - start_ns_ : 0;
        profiler_.record(stage_, correlation_id_, start_ns_, duration_ns);
    }

    // Non-copyable, non-movable — the scope defines the timing window.
    ScopedLatency(const ScopedLatency&)                = delete;
    ScopedLatency& operator=(const ScopedLatency&)     = delete;
    ScopedLatency(ScopedLatency&&) noexcept            = delete;
    ScopedLatency& operator=(ScopedLatency&&) noexcept = delete;

    /// Start time captured at construction (wall clock, ns).
    [[nodiscard]] uint64_t start_ns() const noexcept { return start_ns_; }

    /// Correlation ID captured at construction (0 if none was set).
    [[nodiscard]] uint64_t correlation_id() const noexcept { return correlation_id_; }

private:
    static uint64_t now_ns() { return get_clock().now_ns(); }

    LatencyProfiler& profiler_;
    // NOTE: stage_ is a non-owning view. Callers must pass a stage name whose
    // storage outlives this ScopedLatency — string literals (the dominant
    // usage pattern `ScopedLatency g(p, "detector")`) satisfy this trivially.
    // Temporary strings built in the constructor argument list are also safe
    // because their lifetime extends to the end of the full-expression, and
    // ScopedLatency is used exclusively as a stack local at that point.
    std::string_view stage_;
    uint64_t         correlation_id_;
    uint64_t         start_ns_;
};

}  // namespace drone::util
