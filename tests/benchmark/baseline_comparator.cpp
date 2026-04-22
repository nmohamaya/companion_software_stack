// tests/benchmark/baseline_comparator.cpp

#include "benchmark/baseline_comparator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <nlohmann/json.hpp>

namespace drone::benchmark {

namespace {

constexpr std::size_t kMaxLatencyJsonBytes = 1024 * 1024;

MetricDelta make_delta(const std::string& name, double baseline_val, double current_val,
                       double threshold_pct, const char* direction) {
    MetricDelta d;
    d.name          = name;
    d.baseline      = baseline_val;
    d.current       = current_val;
    d.threshold_pct = threshold_pct * 100.0;
    d.direction     = direction;

    if (baseline_val == 0.0) {
        d.skipped   = true;
        d.passed    = true;
        d.delta_pct = 0.0;
        return d;
    }

    d.delta_pct = (current_val - baseline_val) / baseline_val * 100.0;
    return d;
}

MetricDelta check_higher_is_better(const std::string& name, double baseline_val, double current_val,
                                   double threshold_pct) {
    auto d = make_delta(name, baseline_val, current_val, threshold_pct, "higher_is_better");
    if (!d.skipped) {
        d.passed = current_val >= baseline_val * (1.0 - threshold_pct);
    }
    return d;
}

MetricDelta check_lower_is_better(const std::string& name, double baseline_val, double current_val,
                                  double threshold_pct) {
    auto d = make_delta(name, baseline_val, current_val, threshold_pct, "lower_is_better");
    if (!d.skipped) {
        d.passed = current_val <= baseline_val * (1.0 + threshold_pct);
    }
    return d;
}

bool is_valid_stage_name(const std::string& name) {
    return std::all_of(name.begin(), name.end(),
                       [](char c) { return std::isalnum(c) != 0 || c == '_'; });
}

void compare_latency(const std::string& baseline_json, const std::string& current_json,
                     double threshold_pct, ScenarioComparison& sc) {
    if (baseline_json.empty() || current_json.empty()) {
        return;
    }
    if (baseline_json.size() > kMaxLatencyJsonBytes || current_json.size() > kMaxLatencyJsonBytes) {
        return;
    }

    auto bl  = nlohmann::json::parse(baseline_json, nullptr, false);
    auto cur = nlohmann::json::parse(current_json, nullptr, false);
    if (bl.is_discarded() || cur.is_discarded()) {
        return;
    }

    if (!bl.contains("stages") || !cur.contains("stages")) {
        return;
    }

    for (auto it = bl["stages"].begin(); it != bl["stages"].end(); ++it) {
        const auto& stage_name = it.key();
        if (!is_valid_stage_name(stage_name)) {
            continue;
        }
        if (!cur["stages"].contains(stage_name)) {
            continue;
        }
        const auto& bl_stage  = it.value();
        const auto& cur_stage = cur["stages"][stage_name];

        if (bl_stage.contains("p95_ns") && cur_stage.contains("p95_ns")) {
            double bl_p95  = bl_stage["p95_ns"].get<double>();
            double cur_p95 = cur_stage["p95_ns"].get<double>();
            sc.metrics.push_back(check_lower_is_better("latency." + stage_name + ".p95_ns", bl_p95,
                                                       cur_p95, threshold_pct));
        }
    }
}

}  // namespace

ComparisonResult compare_baselines(const BaselineCapture& baseline, const BaselineCapture& current,
                                   const ComparisonThresholds& thresholds) {
    ComparisonResult result;
    result.passed = true;

    for (const auto& name : baseline.scenario_names()) {
        ScenarioComparison sc;
        sc.scenario_name = name;

        const auto* bl  = baseline.scenario(name);
        const auto* cur = current.scenario(name);

        if (cur == nullptr) {
            sc.found_in_current = false;
            sc.passed           = false;
            result.passed       = false;
            result.scenarios.push_back(std::move(sc));
            ++result.scenarios_checked;
            continue;
        }

        sc.found_in_current = true;

        // Detection metrics (higher is better).
        sc.metrics.push_back(check_higher_is_better("micro_recall", bl->micro_recall,
                                                    cur->micro_recall, thresholds.recall_drop_pct));
        sc.metrics.push_back(check_higher_is_better("micro_precision", bl->micro_precision,
                                                    cur->micro_precision,
                                                    thresholds.precision_drop_pct));
        sc.metrics.push_back(
            check_higher_is_better("mean_ap", bl->mean_ap, cur->mean_ap, thresholds.ap_drop_pct));

        // Tracking metrics (higher is better).
        sc.metrics.push_back(
            check_higher_is_better("mota", bl->mota, cur->mota, thresholds.mota_drop_pct));
        sc.metrics.push_back(
            check_higher_is_better("motp", bl->motp, cur->motp, thresholds.motp_drop_pct));

        // Latency (lower is better).
        compare_latency(bl->latency_json, cur->latency_json, thresholds.latency_rise_pct, sc);

        // Roll up scenario pass/fail.
        for (const auto& m : sc.metrics) {
            if (!m.skipped) {
                ++result.metrics_checked;
            }
            if (!m.passed) {
                sc.passed = false;
                ++result.metrics_failed;
            }
        }
        if (!sc.passed) {
            result.passed = false;
        }

        result.scenarios.push_back(std::move(sc));
        ++result.scenarios_checked;
    }

    return result;
}

std::string format_comparison(const ComparisonResult& result) {
    std::ostringstream os;
    os.str().reserve(512 * result.scenarios.size() + 256);

    os << "## Baseline Comparison\n\n";
    os << "| Scenario | Metric | Baseline | Current | Delta% | Threshold | Status |\n";
    os << "|----------|--------|----------|---------|--------|-----------|--------|\n";

    for (const auto& sc : result.scenarios) {
        if (!sc.found_in_current) {
            os << "| " << sc.scenario_name << " | — | — | — | — | — | **MISSING** |\n";
            continue;
        }
        for (const auto& m : sc.metrics) {
            if (m.skipped) {
                continue;
            }
            os << "| " << sc.scenario_name << " | " << m.name << " | " << std::fixed
               << std::setprecision(4) << m.baseline << " | " << m.current << " | " << std::showpos
               << std::setprecision(1) << m.delta_pct << "%" << std::noshowpos << " | "
               << std::setprecision(1) << m.threshold_pct << "% | "
               << (m.passed ? "PASS" : "**FAIL**") << " |\n";
        }
    }

    os << "\n**Result:** " << result.scenarios_checked << " scenario(s), " << result.metrics_checked
       << " metric(s) checked, " << result.metrics_failed << " failed — "
       << (result.passed ? "**PASS**" : "**FAIL**") << "\n";

    return os.str();
}

}  // namespace drone::benchmark
