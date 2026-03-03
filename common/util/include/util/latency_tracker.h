// common/util/include/util/latency_tracker.h
// Lock-free, fixed-size latency histogram for IPC performance monitoring.
//
// Usage:
//   LatencyTracker tracker(1024);          // 1024-sample ring buffer
//   tracker.record(latency_ns);            // O(1), no allocation
//   auto stats = tracker.summary();        // p50/p90/p99/max/mean
//   tracker.reset();                       // clear for next window
//
// Thread safety: record() is safe to call from one writer thread.
// summary() / reset() should be called from a single reporting thread.
// Multiple concurrent writers require external synchronisation.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::util {

/// Summary statistics from a LatencyTracker reporting window.
struct LatencySummary {
    uint64_t count    = 0;       ///< Number of samples recorded.
    uint64_t min_ns   = 0;       ///< Minimum latency (ns).
    uint64_t max_ns   = 0;       ///< Maximum latency (ns).
    double   mean_ns  = 0.0;     ///< Arithmetic mean (ns).
    uint64_t p50_ns   = 0;       ///< 50th percentile (ns).
    uint64_t p90_ns   = 0;       ///< 90th percentile (ns).
    uint64_t p95_ns   = 0;       ///< 95th percentile (ns).
    uint64_t p99_ns   = 0;       ///< 99th percentile (ns).

    /// Convert a nanosecond value to microseconds for readability.
    static double to_us(uint64_t ns) { return static_cast<double>(ns) / 1000.0; }

    /// Convert a nanosecond value to milliseconds for readability.
    static double to_ms(uint64_t ns) { return static_cast<double>(ns) / 1'000'000.0; }
};

/// Fixed-size ring buffer that collects latency samples and computes
/// percentile statistics on demand.
///
/// Designed for the IPC hot path:
///   - record() is O(1) with zero heap allocation.
///   - summary() sorts a snapshot — call it only from the periodic
///     reporting path, not on every message.
///
/// Not thread-safe for concurrent writers. Each subscriber should
/// own its own LatencyTracker instance.
class LatencyTracker {
public:
    /// @param capacity  Ring buffer size (rounded up to power-of-two internally).
    ///                  Larger = more accurate percentiles, more memory.
    explicit LatencyTracker(size_t capacity = 1024)
        : mask_(next_power_of_two(capacity) - 1), samples_(mask_ + 1, 0) {}

    /// Record a single latency sample.  O(1), no allocation.
    void record(uint64_t latency_ns) {
        samples_[write_pos_ & mask_] = latency_ns;
        ++write_pos_;
        ++total_count_;
    }

    /// Compute summary statistics over the current window.
    /// Sorts a copy of the internal buffer — O(n log n).
    /// Safe to call while record() is writing (snapshot-based).
    [[nodiscard]] LatencySummary summary() const {
        LatencySummary s;
        size_t         n = std::min(total_count_, samples_.size());
        if (n == 0) return s;

        // Take a snapshot of the ring buffer
        std::vector<uint64_t> sorted(samples_.begin(), samples_.begin() + n);
        std::sort(sorted.begin(), sorted.end());

        s.count  = total_count_;
        s.min_ns = sorted.front();
        s.max_ns = sorted.back();

        uint64_t sum = 0;
        for (auto v : sorted) sum += v;
        s.mean_ns = static_cast<double>(sum) / static_cast<double>(n);

        s.p50_ns = percentile_sorted(sorted, 50.0);
        s.p90_ns = percentile_sorted(sorted, 90.0);
        s.p95_ns = percentile_sorted(sorted, 95.0);
        s.p99_ns = percentile_sorted(sorted, 99.0);

        return s;
    }

    /// Reset all samples and counters for the next reporting window.
    void reset() {
        std::fill(samples_.begin(), samples_.end(), 0);
        write_pos_   = 0;
        total_count_ = 0;
    }

    /// Total number of samples recorded (may exceed ring capacity).
    [[nodiscard]] size_t total_count() const { return total_count_; }

    /// Ring buffer capacity.
    [[nodiscard]] size_t capacity() const { return mask_ + 1; }

    // ── Convenience: now_ns() for computing latency ──────────

    /// Return the current steady_clock time in nanoseconds.
    static uint64_t now_ns() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    // ── Periodic logging helper ──────────────────────────────

    /// Log a summary line if enough samples have been collected.
    /// Returns true if a summary was logged (and the tracker was reset).
    bool log_summary_if_due(const std::string& topic_name, size_t min_samples = 10) {
        if (total_count_ < min_samples) return false;

        auto s = summary();
        spdlog::info("[Latency] {} — n={}, p50={:.1f}µs, p90={:.1f}µs, "
                     "p99={:.1f}µs, max={:.1f}µs, mean={:.1f}µs",
                     topic_name, s.count, LatencySummary::to_us(s.p50_ns),
                     LatencySummary::to_us(s.p90_ns), LatencySummary::to_us(s.p99_ns),
                     LatencySummary::to_us(s.max_ns), LatencySummary::to_us(s.mean_ns));
        reset();
        return true;
    }

private:
    size_t                mask_;          ///< Bitmask for power-of-2 ring indexing.
    std::vector<uint64_t> samples_;      ///< Ring buffer of latency samples (ns).
    size_t                write_pos_   = 0;  ///< Next write position (wraps via mask_).
    size_t                total_count_ = 0;  ///< Total samples recorded (may exceed capacity).

    /// Round up to next power of two.
    static size_t next_power_of_two(size_t n) {
        size_t v = 1;
        while (v < n) v <<= 1;
        return v;
    }

    /// Compute the p-th percentile from a sorted vector.
    static uint64_t percentile_sorted(const std::vector<uint64_t>& sorted, double p) {
        if (sorted.empty()) return 0;
        double rank = (p / 100.0) * static_cast<double>(sorted.size() - 1);
        size_t idx  = static_cast<size_t>(rank);
        if (idx >= sorted.size() - 1) return sorted.back();
        // Linear interpolation between adjacent ranks
        double frac = rank - static_cast<double>(idx);
        return static_cast<uint64_t>(static_cast<double>(sorted[idx]) * (1.0 - frac) +
                                     static_cast<double>(sorted[idx + 1]) * frac);
    }
};

}  // namespace drone::util
