// tests/test_baseline_comparator.cpp
//
// Unit tests for BaselineComparator (Issue #572, Epic #523).
// Verifies regression detection, threshold logic, and output formatting.

#include "benchmark/baseline_capture.h"
#include "benchmark/baseline_comparator.h"
#include "benchmark/perception_metrics.h"

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

using namespace drone::benchmark;

namespace {

// Populate a scenario with known detection metrics via raw struct fields
// (bypasses finalise — we set metrics directly for deterministic testing).
void set_detection_metrics(BaselineCapture& capture, const std::string& name, double recall,
                           double precision, double mean_ap, uint32_t tp, uint32_t fp,
                           uint32_t fn) {
    auto& s           = capture.add_scenario(name);
    s.micro_recall    = recall;
    s.micro_precision = precision;
    s.mean_ap         = mean_ap;
    s.total_tp        = tp;
    s.total_fp        = fp;
    s.total_fn        = fn;
    s.frame_count     = tp + fn;
}

void set_tracking_metrics(BaselineCapture& capture, const std::string& name, double mota,
                          double motp, uint32_t id_switches) {
    auto* s = const_cast<ScenarioBaseline*>(capture.scenario(name));
    if (s == nullptr) {
        return;
    }
    s->mota        = mota;
    s->motp        = motp;
    s->id_switches = id_switches;
}

}  // namespace

// -- Improvement passes -------------------------------------------------------

TEST(BaselineComparatorTest, ImprovementPasses) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.90, 0.90, 0.85, 90, 10, 10);

    auto result = compare_baselines(baseline, current);
    EXPECT_TRUE(result.passed);
    EXPECT_EQ(result.metrics_failed, 0U);
}

// -- Regression beyond threshold fails ----------------------------------------

TEST(BaselineComparatorTest, RegressionFails) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    // 10% recall drop (0.80 → 0.70) exceeds 5% threshold.
    set_detection_metrics(current, "s1", 0.70, 0.85, 0.75, 70, 15, 30);

    auto result = compare_baselines(baseline, current);
    EXPECT_FALSE(result.passed);
    EXPECT_GE(result.metrics_failed, 1U);

    // Find the recall metric.
    bool found_recall_fail = false;
    for (const auto& sc : result.scenarios) {
        for (const auto& m : sc.metrics) {
            if (m.name == "micro_recall" && !m.passed) {
                found_recall_fail = true;
            }
        }
    }
    EXPECT_TRUE(found_recall_fail);
}

// -- Within threshold passes --------------------------------------------------

TEST(BaselineComparatorTest, WithinThresholdPasses) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    // 3% recall drop (0.80 → 0.776) is within 5% threshold.
    set_detection_metrics(current, "s1", 0.776, 0.83, 0.73, 78, 17, 22);

    auto result = compare_baselines(baseline, current);
    EXPECT_TRUE(result.passed);
}

// -- Missing scenario in current run fails ------------------------------------

TEST(BaselineComparatorTest, MissingScenarioFails) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "expected_scenario", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    set_detection_metrics(current, "different_scenario", 0.90, 0.90, 0.85, 90, 10, 10);

    auto result = compare_baselines(baseline, current);
    EXPECT_FALSE(result.passed);
    ASSERT_EQ(result.scenarios.size(), 1U);
    EXPECT_FALSE(result.scenarios[0].found_in_current);
}

// -- Zero-valued baseline metrics are skipped ---------------------------------

TEST(BaselineComparatorTest, ZeroBaselineSkipped) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.0, 0.0, 0.0, 0, 0, 0);

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.5, 0.5, 0.5, 50, 50, 50);

    auto result = compare_baselines(baseline, current);
    EXPECT_TRUE(result.passed);

    // All detection metrics should be skipped.
    for (const auto& sc : result.scenarios) {
        for (const auto& m : sc.metrics) {
            if (m.direction == "higher_is_better") {
                EXPECT_TRUE(m.skipped) << "Metric " << m.name << " should be skipped";
            }
        }
    }
}

// -- Latency regression fails -------------------------------------------------

TEST(BaselineComparatorTest, LatencyRegressionFails) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);
    auto* bl_s         = const_cast<ScenarioBaseline*>(baseline.scenario("s1"));
    bl_s->latency_json = R"({"stages":{"detector":{"p95_ns":5000000}}})";

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.80, 0.85, 0.75, 80, 15, 20);
    auto* cur_s = const_cast<ScenarioBaseline*>(current.scenario("s1"));
    // 30% latency increase exceeds 20% threshold.
    cur_s->latency_json = R"({"stages":{"detector":{"p95_ns":6500000}}})";

    auto result = compare_baselines(baseline, current);
    EXPECT_FALSE(result.passed);

    bool found_latency_fail = false;
    for (const auto& sc : result.scenarios) {
        for (const auto& m : sc.metrics) {
            if (m.name.find("latency") != std::string::npos && !m.passed) {
                found_latency_fail = true;
            }
        }
    }
    EXPECT_TRUE(found_latency_fail);
}

// -- Format produces Markdown table -------------------------------------------

TEST(BaselineComparatorTest, FormatTableOutput) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.82, 0.87, 0.77, 82, 13, 18);

    auto result = compare_baselines(baseline, current);
    auto table  = format_comparison(result);

    EXPECT_NE(table.find("## Baseline Comparison"), std::string::npos);
    EXPECT_NE(table.find("| Scenario"), std::string::npos);
    EXPECT_NE(table.find("micro_recall"), std::string::npos);
    EXPECT_NE(table.find("PASS"), std::string::npos);
    EXPECT_NE(table.find("**PASS**"), std::string::npos);
}

// -- Extra scenario in current is ignored -------------------------------------

TEST(BaselineComparatorTest, ExtraScenarioIgnored) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.82, 0.87, 0.77, 82, 13, 18);
    set_detection_metrics(current, "s2_extra", 0.50, 0.50, 0.50, 50, 50, 50);

    auto result = compare_baselines(baseline, current);
    EXPECT_TRUE(result.passed);
    EXPECT_EQ(result.scenarios_checked, 1U);
}

// -- Custom thresholds respected ----------------------------------------------

TEST(BaselineComparatorTest, CustomThresholds) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    // 10% recall drop — fails with default 5%, passes with 15%.
    set_detection_metrics(current, "s1", 0.72, 0.85, 0.75, 72, 15, 28);

    ComparisonThresholds strict;
    strict.recall_drop_pct = 0.05;
    EXPECT_FALSE(compare_baselines(baseline, current, strict).passed);

    ComparisonThresholds relaxed;
    relaxed.recall_drop_pct = 0.15;
    EXPECT_TRUE(compare_baselines(baseline, current, relaxed).passed);
}

// -- Tracking regression detected ---------------------------------------------

TEST(BaselineComparatorTest, TrackingRegressionFails) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "s1", 0.80, 0.85, 0.75, 80, 15, 20);
    set_tracking_metrics(baseline, "s1", 0.85, 0.78, 2);

    BaselineCapture current;
    set_detection_metrics(current, "s1", 0.80, 0.85, 0.75, 80, 15, 20);
    // MOTA drops from 0.85 to 0.70 — 17.6% drop, exceeds 5% threshold.
    set_tracking_metrics(current, "s1", 0.70, 0.78, 2);

    auto result = compare_baselines(baseline, current);
    EXPECT_FALSE(result.passed);
}

// -- Multiple scenarios — partial failure -------------------------------------

TEST(BaselineComparatorTest, MultipleScenarioPartialFailure) {
    BaselineCapture baseline;
    set_detection_metrics(baseline, "good", 0.80, 0.85, 0.75, 80, 15, 20);
    set_detection_metrics(baseline, "bad", 0.80, 0.85, 0.75, 80, 15, 20);

    BaselineCapture current;
    set_detection_metrics(current, "good", 0.82, 0.87, 0.77, 82, 13, 18);
    // "bad" scenario has severe recall regression.
    set_detection_metrics(current, "bad", 0.50, 0.85, 0.75, 50, 15, 50);

    auto result = compare_baselines(baseline, current);
    EXPECT_FALSE(result.passed);
    EXPECT_EQ(result.scenarios_checked, 2U);

    bool good_passed = false;
    bool bad_failed  = false;
    for (const auto& sc : result.scenarios) {
        if (sc.scenario_name == "good" && sc.passed) good_passed = true;
        if (sc.scenario_name == "bad" && !sc.passed) bad_failed = true;
    }
    EXPECT_TRUE(good_passed);
    EXPECT_TRUE(bad_failed);
}
