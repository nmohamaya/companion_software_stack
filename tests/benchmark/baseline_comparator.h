// tests/benchmark/baseline_comparator.h
//
// Compares a current perception run against a stored baseline, detecting
// regressions that exceed configurable thresholds (Issue #572, Epic #523).

#pragma once

#include "benchmark/baseline_capture.h"

#include <cstdint>
#include <string>
#include <vector>

namespace drone::benchmark {

struct ComparisonThresholds {
    double recall_drop_pct    = 0.05;
    double precision_drop_pct = 0.05;
    double ap_drop_pct        = 0.05;
    double mota_drop_pct      = 0.05;
    double latency_rise_pct   = 0.20;
};

struct MetricDelta {
    std::string name{};
    double      baseline{0.0};
    double      current{0.0};
    double      delta_pct{0.0};
    double      threshold_pct{0.0};
    bool        passed{true};
    bool        skipped{false};
    std::string direction{};
};

struct ScenarioComparison {
    std::string              scenario_name{};
    bool                     passed{true};
    bool                     found_in_current{false};
    std::vector<MetricDelta> metrics{};
};

struct ComparisonResult {
    bool                            passed{true};
    uint32_t                        scenarios_checked{0};
    uint32_t                        metrics_checked{0};
    uint32_t                        metrics_failed{0};
    std::vector<ScenarioComparison> scenarios{};
};

// Compare a current run against a baseline. Only scenarios present in the
// baseline are checked — extra scenarios in current are ignored. Zero-valued
// baseline metrics are skipped (not yet captured).
[[nodiscard]] ComparisonResult compare_baselines(const BaselineCapture&      baseline,
                                                 const BaselineCapture&      current,
                                                 const ComparisonThresholds& thresholds = {});

// Render the comparison result as a Markdown table suitable for CI output.
[[nodiscard]] std::string format_comparison(const ComparisonResult& result);

}  // namespace drone::benchmark
