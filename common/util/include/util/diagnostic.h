// common/util/include/util/diagnostic.h
// Per-frame diagnostic collector — accumulates metrics, warnings, and errors
// from pipeline stages.  Published alongside output for monitoring and
// post-mortem analysis.
//
// Design rationale:
//   When a simulation run breaks, we need to answer "what went wrong, where,
//   and when?" quickly.  FrameDiagnostics gives every pipeline stage a place
//   to store structured numeric metrics *and* free-text explanations that
//   are automatically logged, timestamped, and available to downstream
//   consumers (system_monitor, test harness, post-run analysis scripts).
//
// Usage:
//   FrameDiagnostics diag(frame_id);
//   diag.add_timing("FeatureExtractor", 4.2);
//   diag.add_metric("FeatureExtractor", "num_features", 127);
//   diag.add_warning("StereoMatcher", "Low disparity variance (σ=0.8)");
//   diag.add_error("ImuPreintegrator", "Gap detected: 12ms missing");
//   diag.log_summary();   // → structured spdlog output
//   bool ok = !diag.has_errors();
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::util {

// ── Severity levels ────────────────────────────────────────
enum class DiagSeverity : uint8_t {
    INFO  = 0,  // informational metric
    WARN  = 1,  // degraded but recoverable
    ERROR = 2,  // failure in this stage
    FATAL = 3,  // unrecoverable — pipeline should abort
};

inline const char* diag_severity_str(DiagSeverity s) {
    switch (s) {
        case DiagSeverity::INFO: return "INFO";
        case DiagSeverity::WARN: return "WARN";
        case DiagSeverity::ERROR: return "ERROR";
        case DiagSeverity::FATAL: return "FATAL";
        default: return "???";
    }
}

// ── Single diagnostic entry ────────────────────────────────
struct DiagnosticEntry {
    std::string  component;  // e.g. "FeatureExtractor", "IMUPreintegrator"
    DiagSeverity severity;
    std::string  message;
    double       value;  // optional numeric metric (NaN = not set)

    DiagnosticEntry(std::string comp, DiagSeverity sev, std::string msg,
                    double val = std::numeric_limits<double>::quiet_NaN())
        : component(std::move(comp)), severity(sev), message(std::move(msg)), value(val) {}
};

// ── Frame-level diagnostic collector ───────────────────────
// One instance per pipeline iteration.  Not thread-safe —
// call from the owning pipeline thread only.
class FrameDiagnostics {
public:
    explicit FrameDiagnostics(uint64_t frame_id) : frame_id_(frame_id) {}

    // ── Accessors ────────────────────────────────────────────
    [[nodiscard]] uint64_t                            frame_id() const { return frame_id_; }
    [[nodiscard]] const std::vector<DiagnosticEntry>& entries() const { return entries_; }
    [[nodiscard]] int                                 error_count() const { return error_count_; }
    [[nodiscard]] int  warning_count() const { return warning_count_; }
    [[nodiscard]] bool has_errors() const { return error_count_ > 0; }
    [[nodiscard]] bool has_warnings() const { return warning_count_ > 0; }
    [[nodiscard]] bool has_fatal() const { return fatal_count_ > 0; }

    // ── Recording methods ────────────────────────────────────

    /// Record a timing measurement (ms) for a pipeline stage.
    void add_timing(const std::string& component, double ms) {
        entries_.emplace_back(component, DiagSeverity::INFO, "timing_ms", ms);
    }

    /// Record a named numeric metric.
    void add_metric(const std::string& component, const std::string& name, double value) {
        entries_.emplace_back(component, DiagSeverity::INFO, name, value);
    }

    /// Record a warning (degraded but recoverable).
    void add_warning(const std::string& component, const std::string& message) {
        entries_.emplace_back(component, DiagSeverity::WARN, message);
        ++warning_count_;
    }

    /// Record an error (stage failed).
    void add_error(const std::string& component, const std::string& message) {
        entries_.emplace_back(component, DiagSeverity::ERROR, message);
        ++error_count_;
    }

    /// Record a fatal error (pipeline should abort).
    void add_fatal(const std::string& component, const std::string& message) {
        entries_.emplace_back(component, DiagSeverity::FATAL, message);
        ++fatal_count_;
        ++error_count_;
    }

    /// Most severe level recorded.
    [[nodiscard]] DiagSeverity worst_severity() const {
        if (fatal_count_ > 0) return DiagSeverity::FATAL;
        if (error_count_ > 0) return DiagSeverity::ERROR;
        if (warning_count_ > 0) return DiagSeverity::WARN;
        return DiagSeverity::INFO;
    }

    /// Log a structured summary via spdlog.
    /// Called once per frame at the end of the pipeline.
    void log_summary(const std::string& pipeline_name = "VIO") const {
        auto level = spdlog::level::info;
        if (has_fatal())
            level = spdlog::level::critical;
        else if (has_errors())
            level = spdlog::level::err;
        else if (has_warnings())
            level = spdlog::level::warn;

        spdlog::log(level, "[{}] Frame {} diagnostics: {} entries, {} errors, {} warnings",
                    pipeline_name, frame_id_, entries_.size(), error_count_, warning_count_);

        // Log each entry at appropriate level
        for (const auto& e : entries_) {
            auto entry_level = spdlog::level::info;
            switch (e.severity) {
                case DiagSeverity::WARN: entry_level = spdlog::level::warn; break;
                case DiagSeverity::ERROR: entry_level = spdlog::level::err; break;
                case DiagSeverity::FATAL: entry_level = spdlog::level::critical; break;
                default: break;  // DiagSeverity::INFO stays at info level
            }

            spdlog::log(entry_level, "[{}] Frame {} [{}] {}: {}", pipeline_name, frame_id_,
                        diag_severity_str(e.severity), e.component, e.message);
        }
    }

    /// Merge entries from another FrameDiagnostics (e.g. sub-pipeline).
    void merge(const FrameDiagnostics& other) {
        entries_.insert(entries_.end(), other.entries_.begin(), other.entries_.end());
        error_count_ += other.error_count_;
        warning_count_ += other.warning_count_;
        fatal_count_ += other.fatal_count_;
    }

    /// Reset for reuse with a new frame ID.
    void reset(uint64_t new_frame_id) {
        frame_id_ = new_frame_id;
        entries_.clear();
        error_count_   = 0;
        warning_count_ = 0;
        fatal_count_   = 0;
    }

private:
    uint64_t                     frame_id_;
    std::vector<DiagnosticEntry> entries_;
    int                          error_count_   = 0;
    int                          warning_count_ = 0;
    int                          fatal_count_   = 0;
};

// ── RAII scoped timer that records into FrameDiagnostics ────
// Usage:
//   {
//       ScopedDiagTimer timer(diag, "FeatureExtractor");
//       // ... do work ...
//   }  // auto-records elapsed ms as timing metric
class ScopedDiagTimer {
public:
    ScopedDiagTimer(FrameDiagnostics& diag, std::string component)
        : diag_(diag), component_(std::move(component)), start_(std::chrono::steady_clock::now()) {}

    ~ScopedDiagTimer() {
        auto   end = std::chrono::steady_clock::now();
        double ms  = std::chrono::duration<double, std::milli>(end - start_).count();
        diag_.add_timing(component_, ms);
    }

    ScopedDiagTimer(const ScopedDiagTimer&)            = delete;
    ScopedDiagTimer& operator=(const ScopedDiagTimer&) = delete;

private:
    FrameDiagnostics&                     diag_;
    std::string                           component_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace drone::util
