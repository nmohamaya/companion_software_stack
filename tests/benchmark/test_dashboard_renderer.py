#!/usr/bin/env python3
"""Unit tests for dashboard_renderer.py (Issue #574, Epic #523)."""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from dashboard_renderer import (
    ComparisonResult,
    MetricDelta,
    ScenarioResult,
    _top_changes,
    compare_scenarios,
    load_baseline,
    render_full_report,
    render_pr_comment,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
BASELINE_PATH = os.path.join(REPO_ROOT, "benchmarks", "baseline.json")


def _make_scenario(name, recall=0.8, precision=0.85, mean_ap=0.75,
                   mota=0.8, motp=0.75, tp=80, fp=15, fn=20,
                   latency=None, per_class=None):
    s = {
        "frame_count": tp + fn,
        "iou_threshold": 0.5,
        "num_classes": 2,
        "detection": {
            "total_tp": tp, "total_fp": fp, "total_fn": fn,
            "micro_recall": recall, "micro_precision": precision,
            "mean_ap": mean_ap,
        },
        "per_class": per_class or [],
        "tracking": {"mota": mota, "motp": motp, "id_switches": 0,
                      "fragmentations": 0},
    }
    if latency is not None:
        s["latency"] = latency
    return s


def _wrap(scenarios):
    return {"version": 1, "scenarios": scenarios}


class TestLoadBaseline(unittest.TestCase):

    def test_load_valid_baseline(self):
        if not os.path.exists(BASELINE_PATH):
            self.skipTest("benchmarks/baseline.json not found")
        data = load_baseline(BASELINE_PATH)
        self.assertIsNotNone(data)
        self.assertIn("scenarios", data)
        self.assertIn("version", data)

    def test_load_missing_file(self):
        result = load_baseline("/nonexistent/path.json")
        self.assertIsNone(result)

    def test_load_invalid_json(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                         delete=False) as f:
            f.write("{not valid json}")
            f.flush()
            result = load_baseline(f.name)
        os.unlink(f.name)
        self.assertIsNone(result)


class TestCompareScenarios(unittest.TestCase):

    def test_improvement_passes(self):
        baseline = _wrap({"s1": _make_scenario("s1", recall=0.80)})
        current = _wrap({"s1": _make_scenario("s1", recall=0.90)})
        result = compare_scenarios(baseline, current)
        self.assertTrue(result.passed)
        self.assertEqual(result.metrics_failed, 0)
        self.assertGreater(result.metrics_checked, 0)

    def test_regression_fails(self):
        baseline = _wrap({"s1": _make_scenario("s1", recall=0.80)})
        current = _wrap({"s1": _make_scenario("s1", recall=0.70)})
        result = compare_scenarios(baseline, current)
        self.assertFalse(result.passed)
        self.assertGreaterEqual(result.metrics_failed, 1)

    def test_zero_baseline_skipped(self):
        baseline = _wrap({"s1": _make_scenario("s1", recall=0.0,
                                                precision=0.0, mean_ap=0.0,
                                                mota=0.0, motp=0.0)})
        current = _wrap({"s1": _make_scenario("s1", recall=0.5)})
        result = compare_scenarios(baseline, current)
        self.assertTrue(result.passed)

    def test_missing_scenario_fails(self):
        baseline = _wrap({"expected": _make_scenario("expected")})
        current = _wrap({"other": _make_scenario("other")})
        result = compare_scenarios(baseline, current)
        self.assertFalse(result.passed)
        self.assertFalse(result.scenarios[0].found_in_current)

    def test_latency_regression(self):
        lat_bl = {"stages": {"detector": {"p95_ns": 5000000}}}
        lat_cur = {"stages": {"detector": {"p95_ns": 6500000}}}
        baseline = _wrap({"s1": _make_scenario("s1", latency=lat_bl)})
        current = _wrap({"s1": _make_scenario("s1", latency=lat_cur)})
        result = compare_scenarios(baseline, current)
        latency_failed = any(
            m.name.startswith("latency.") and not m.passed
            for sc in result.scenarios for m in sc.metrics
        )
        self.assertTrue(latency_failed)


class TestRender(unittest.TestCase):

    def test_pr_comment_contains_sections(self):
        baseline = _wrap({"s1": _make_scenario("s1", recall=0.80)})
        current = _wrap({"s1": _make_scenario("s1", recall=0.82)})
        result = compare_scenarios(baseline, current)
        output = render_pr_comment(result)
        self.assertIn("## Perception Benchmark Report", output)
        self.assertIn("### Summary", output)
        self.assertIn("### Top Changes", output)
        self.assertIn("Advisory", output)
        line_count = len(output.strip().split("\n"))
        self.assertLessEqual(line_count, 60)

    def test_pr_comment_missing_scenario(self):
        baseline = _wrap({"absent": _make_scenario("absent")})
        current = _wrap({"other": _make_scenario("other")})
        result = compare_scenarios(baseline, current)
        output = render_pr_comment(result)
        self.assertIn("**MISSING**", output)
        self.assertIn("FAIL", output)

    def test_full_report_has_detail(self):
        per_class = [{"class_id": 0, "class_name": "drone",
                      "precision": 0.9, "recall": 0.85, "f1": 0.87,
                      "ap": 0.82, "detection_count": 50, "tp": 42,
                      "fp": 5, "fn": 8}]
        lat = {"stages": {"detector": {"p50_ns": 3e6, "p95_ns": 5e6,
                                        "p99_ns": 7e6}}}
        baseline = _wrap({"s1": _make_scenario("s1", latency=lat,
                                                per_class=per_class)})
        current = _wrap({"s1": _make_scenario("s1", latency=lat,
                                               per_class=per_class)})
        result = compare_scenarios(baseline, current)
        output = render_full_report(result, baseline, current)
        self.assertIn("### Detailed Breakdown", output)
        self.assertIn("Per-class breakdown", output)
        self.assertIn("Latency stages", output)
        self.assertIn("### Thresholds", output)
        self.assertIn("drone", output)

    def test_full_report_skipped_scenarios(self):
        baseline = _wrap({"s1": _make_scenario("s1", recall=0.0,
                                                precision=0.0, mean_ap=0.0,
                                                mota=0.0, motp=0.0)})
        current = _wrap({"s1": _make_scenario("s1", recall=0.5)})
        result = compare_scenarios(baseline, current)
        output = render_full_report(result, baseline, current)
        self.assertIn("skipped", output)


class TestTopChanges(unittest.TestCase):

    def test_top_changes_sorted(self):
        result = ComparisonResult(scenarios=[
            ScenarioResult(name="s1", metrics=[
                MetricDelta(name="micro_recall", baseline=0.80, current=0.90,
                            delta_pct=12.5, direction="higher_is_better"),
                MetricDelta(name="micro_precision", baseline=0.85,
                            current=0.87, delta_pct=2.4,
                            direction="higher_is_better"),
                MetricDelta(name="mean_ap", baseline=0.75, current=0.70,
                            delta_pct=-6.7, direction="higher_is_better"),
            ])
        ])
        regressions, improvements = _top_changes(result, n=3)
        self.assertEqual(len(regressions), 1)
        self.assertEqual(regressions[0][1].name, "mean_ap")
        self.assertEqual(len(improvements), 2)
        self.assertEqual(improvements[0][1].name, "micro_recall")

    def test_top_changes_skipped_excluded(self):
        result = ComparisonResult(scenarios=[
            ScenarioResult(name="s1", metrics=[
                MetricDelta(name="micro_recall", baseline=0.0, current=0.5,
                            delta_pct=0.0, skipped=True,
                            direction="higher_is_better"),
            ])
        ])
        regressions, improvements = _top_changes(result)
        self.assertEqual(len(regressions), 0)
        self.assertEqual(len(improvements), 0)


if __name__ == "__main__":
    unittest.main()
