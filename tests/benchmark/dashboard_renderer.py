#!/usr/bin/env python3
"""Perception benchmark dashboard renderer.

Reads BaselineCapture JSON files (baseline + current run) and produces a
human-readable Markdown report for PR comments or standalone viewing.

Usage:
    python3 tests/benchmark/dashboard_renderer.py \\
        --baseline benchmarks/baseline.json \\
        --current /tmp/current-run-baseline.json \\
        --mode pr-comment

    python3 tests/benchmark/dashboard_renderer.py \\
        --baseline benchmarks/baseline.json \\
        --current /tmp/current-run-baseline.json \\
        --mode full
"""

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

DEFAULT_THRESHOLDS = {
    "recall": 0.05,
    "precision": 0.05,
    "ap": 0.05,
    "mota": 0.05,
    "motp": 0.05,
    "latency": 0.20,
}

HIGHER_IS_BETTER = {"micro_recall", "micro_precision", "mean_ap", "mota", "motp"}


@dataclass
class MetricDelta:
    name: str
    baseline: float
    current: float
    delta_pct: float = 0.0
    threshold_pct: float = 0.0
    passed: bool = True
    skipped: bool = False
    direction: str = "higher_is_better"


@dataclass
class ScenarioResult:
    name: str
    passed: bool = True
    found_in_current: bool = True
    metrics: List[MetricDelta] = field(default_factory=list)


@dataclass
class ComparisonResult:
    passed: bool = True
    scenarios: List[ScenarioResult] = field(default_factory=list)
    scenarios_checked: int = 0
    metrics_checked: int = 0
    metrics_failed: int = 0


def load_baseline(path: str) -> Optional[dict]:
    """Load and validate a BaselineCapture JSON file."""
    p = Path(path)
    if not p.exists():
        print(f"Error: file not found: {path}", file=sys.stderr)
        return None
    try:
        data = json.loads(p.read_text())
    except json.JSONDecodeError as e:
        print(f"Error: invalid JSON in {path}: {e}", file=sys.stderr)
        return None
    if "scenarios" not in data:
        print(f"Error: missing 'scenarios' key in {path}", file=sys.stderr)
        return None
    return data


def _threshold_for(metric_name: str, thresholds: dict) -> float:
    if metric_name == "micro_recall":
        return thresholds.get("recall", 0.05)
    if metric_name == "micro_precision":
        return thresholds.get("precision", 0.05)
    if metric_name == "mean_ap":
        return thresholds.get("ap", 0.05)
    if metric_name == "mota":
        return thresholds.get("mota", 0.05)
    if metric_name == "motp":
        return thresholds.get("motp", 0.05)
    if "latency" in metric_name:
        return thresholds.get("latency", 0.20)
    return 0.05


def _check_metric(name: str, baseline_val: float, current_val: float,
                  threshold: float) -> MetricDelta:
    higher = name in HIGHER_IS_BETTER
    direction = "higher_is_better" if higher else "lower_is_better"

    if baseline_val == 0.0:
        return MetricDelta(name=name, baseline=baseline_val, current=current_val,
                           skipped=True, direction=direction,
                           threshold_pct=threshold * 100.0)

    delta_pct = (current_val - baseline_val) / baseline_val * 100.0
    if higher:
        passed = current_val >= baseline_val * (1.0 - threshold)
    else:
        passed = current_val <= baseline_val * (1.0 + threshold)

    return MetricDelta(name=name, baseline=baseline_val, current=current_val,
                       delta_pct=delta_pct, threshold_pct=threshold * 100.0,
                       passed=passed, direction=direction)


def compare_scenarios(baseline: dict, current: dict,
                      thresholds: Optional[dict] = None) -> ComparisonResult:
    """Compare baseline vs current run, producing per-scenario results."""
    if thresholds is None:
        thresholds = DEFAULT_THRESHOLDS

    result = ComparisonResult()
    bl_scenarios = baseline.get("scenarios", {})
    cur_scenarios = current.get("scenarios", {})

    for name, bl_s in bl_scenarios.items():
        sc = ScenarioResult(name=name)
        result.scenarios_checked += 1

        if name not in cur_scenarios:
            sc.found_in_current = False
            sc.passed = False
            result.passed = False
            result.scenarios.append(sc)
            continue

        cur_s = cur_scenarios[name]
        sc.found_in_current = True

        bl_det = bl_s.get("detection", {})
        cur_det = cur_s.get("detection", {})
        for metric in ["micro_recall", "micro_precision", "mean_ap"]:
            t = _threshold_for(metric, thresholds)
            m = _check_metric(metric, bl_det.get(metric, 0.0),
                              cur_det.get(metric, 0.0), t)
            sc.metrics.append(m)

        bl_trk = bl_s.get("tracking", {})
        cur_trk = cur_s.get("tracking", {})
        for metric in ["mota", "motp"]:
            t = _threshold_for(metric, thresholds)
            m = _check_metric(metric, bl_trk.get(metric, 0.0),
                              cur_trk.get(metric, 0.0), t)
            sc.metrics.append(m)

        _compare_latency(bl_s, cur_s, thresholds, sc)

        for m in sc.metrics:
            if not m.skipped:
                result.metrics_checked += 1
            if not m.passed:
                sc.passed = False
                result.metrics_failed += 1

        if not sc.passed:
            result.passed = False

        result.scenarios.append(sc)

    return result


def _compare_latency(bl_s: dict, cur_s: dict, thresholds: dict,
                     sc: ScenarioResult) -> None:
    bl_lat = bl_s.get("latency")
    cur_lat = cur_s.get("latency")
    if not bl_lat or not cur_lat:
        return

    if isinstance(bl_lat, str):
        try:
            bl_lat = json.loads(bl_lat)
        except (json.JSONDecodeError, TypeError):
            return
    if isinstance(cur_lat, str):
        try:
            cur_lat = json.loads(cur_lat)
        except (json.JSONDecodeError, TypeError):
            return

    bl_stages = bl_lat.get("stages", {})
    cur_stages = cur_lat.get("stages", {})
    threshold = thresholds.get("latency", 0.20)

    for stage_name, bl_stage in bl_stages.items():
        if stage_name not in cur_stages:
            continue
        cur_stage = cur_stages[stage_name]
        bl_p95 = bl_stage.get("p95_ns", 0.0)
        cur_p95 = cur_stage.get("p95_ns", 0.0)
        metric_name = f"latency.{stage_name}.p95_ns"
        sc.metrics.append(_check_metric(metric_name, bl_p95, cur_p95, threshold))


def _top_changes(result: ComparisonResult, n: int = 3):
    """Extract top N regressions and improvements by absolute delta."""
    regressions = []
    improvements = []
    for sc in result.scenarios:
        if not sc.found_in_current:
            continue
        for m in sc.metrics:
            if m.skipped:
                continue
            entry = (sc.name, m)
            if m.direction == "higher_is_better":
                if m.delta_pct < 0:
                    regressions.append(entry)
                elif m.delta_pct > 0:
                    improvements.append(entry)
            else:
                if m.delta_pct > 0:
                    regressions.append(entry)
                elif m.delta_pct < 0:
                    improvements.append(entry)

    regressions.sort(key=lambda x: abs(x[1].delta_pct), reverse=True)
    improvements.sort(key=lambda x: abs(x[1].delta_pct), reverse=True)
    return regressions[:n], improvements[:n]


def _fmt_val(val: float, name: str) -> str:
    if "latency" in name:
        ms = val / 1_000_000
        return f"{ms:.1f}ms"
    return f"{val:.2f}"


def _fmt_delta(delta_pct: float) -> str:
    return f"{delta_pct:+.1f}%"


def _scenario_summary_row(sc: ScenarioResult) -> str:
    """Render one row of the summary table."""
    if not sc.found_in_current:
        return f"| {sc.name} | — | — | — | — | — | **MISSING** |"

    by_name: Dict[str, MetricDelta] = {m.name: m for m in sc.metrics}

    def cell(metric_name: str) -> str:
        m = by_name.get(metric_name)
        if m is None or m.skipped:
            return "—"
        return f"{_fmt_val(m.current, metric_name)} ({_fmt_delta(m.delta_pct)})"

    lat_key = next((k for k in by_name if k.startswith("latency.")), None)
    lat_cell = cell(lat_key) if lat_key else "—"

    status = "\u2705" if sc.passed else "\u274c"
    return (f"| {sc.name} | {cell('micro_recall')} | {cell('micro_precision')} "
            f"| {cell('mean_ap')} | {cell('mota')} | {lat_cell} | {status} |")


def render_pr_comment(result: ComparisonResult,
                      thresholds: Optional[dict] = None) -> str:
    """Render a condensed ~50-line Markdown report for PR comments."""
    if thresholds is None:
        thresholds = DEFAULT_THRESHOLDS

    status = "PASS" if result.passed else "FAIL"
    lines = [
        "## Perception Benchmark Report",
        "",
        f"**Result: {status}** ({result.scenarios_checked} scenario(s), "
        f"{result.metrics_checked} metric(s) checked, "
        f"{result.metrics_failed} failed)",
        "",
        "### Summary",
        "",
        "| Scenario | Recall | Precision | mAP | MOTA | Latency p95 | Status |",
        "|----------|--------|-----------|-----|------|-------------|--------|",
    ]

    for sc in result.scenarios:
        lines.append(_scenario_summary_row(sc))

    regressions, improvements = _top_changes(result)

    lines.extend(["", "### Top Changes", ""])

    lines.append(f"\U0001f534 **Regressions ({len(regressions)})**")
    if not regressions:
        lines.append("None")
    else:
        for i, (sname, m) in enumerate(regressions, 1):
            lines.append(
                f"{i}. {sname} / {m.name}: "
                f"{_fmt_val(m.baseline, m.name)} \u2192 "
                f"{_fmt_val(m.current, m.name)} ({_fmt_delta(m.delta_pct)})"
            )

    lines.append("")
    lines.append(f"\U0001f7e2 **Improvements ({len(improvements)})**")
    if not improvements:
        lines.append("None")
    else:
        for i, (sname, m) in enumerate(improvements, 1):
            lines.append(
                f"{i}. {sname} / {m.name}: "
                f"{_fmt_val(m.baseline, m.name)} \u2192 "
                f"{_fmt_val(m.current, m.name)} ({_fmt_delta(m.delta_pct)})"
            )

    acc_t = int(thresholds.get("recall", 0.05) * 100)
    lat_t = int(thresholds.get("latency", 0.20) * 100)
    lines.extend([
        "",
        f"> Advisory \u2014 does not block merge. "
        f"Thresholds: {acc_t}% accuracy, {lat_t}% latency.",
    ])

    return "\n".join(lines) + "\n"


def render_full_report(result: ComparisonResult, baseline: dict,
                       current: dict,
                       thresholds: Optional[dict] = None) -> str:
    """Render a detailed report with per-class and latency breakdowns."""
    if thresholds is None:
        thresholds = DEFAULT_THRESHOLDS

    parts = [render_pr_comment(result, thresholds)]

    parts.append("\n---\n\n### Detailed Breakdown\n")

    for sc in result.scenarios:
        parts.append(f"\n#### {sc.name}\n")

        if not sc.found_in_current:
            parts.append("**Status:** MISSING in current run\n")
            continue

        parts.append("| Metric | Baseline | Current | Delta | Threshold | Status |")
        parts.append("|--------|----------|---------|-------|-----------|--------|")
        for m in sc.metrics:
            if m.skipped:
                parts.append(
                    f"| {m.name} | {_fmt_val(m.baseline, m.name)} | "
                    f"{_fmt_val(m.current, m.name)} | — | — | skipped |"
                )
                continue
            status = "PASS" if m.passed else "**FAIL**"
            parts.append(
                f"| {m.name} | {_fmt_val(m.baseline, m.name)} | "
                f"{_fmt_val(m.current, m.name)} | {_fmt_delta(m.delta_pct)} "
                f"| {m.threshold_pct:.0f}% | {status} |"
            )

        cur_s = current.get("scenarios", {}).get(sc.name, {})
        per_class = cur_s.get("per_class", [])
        if per_class:
            parts.append(f"\n**Per-class breakdown ({sc.name}):**\n")
            parts.append("| Class | Precision | Recall | F1 | AP | Count |")
            parts.append("|-------|-----------|--------|----|----|-------|")
            for pc in per_class:
                parts.append(
                    f"| {pc.get('class_name', pc.get('class_id', '?'))} "
                    f"| {pc.get('precision', 0):.2f} "
                    f"| {pc.get('recall', 0):.2f} "
                    f"| {pc.get('f1', 0):.2f} "
                    f"| {pc.get('ap', 0):.2f} "
                    f"| {pc.get('detection_count', 0)} |"
                )

        cur_lat = cur_s.get("latency")
        if cur_lat:
            if isinstance(cur_lat, str):
                try:
                    cur_lat = json.loads(cur_lat)
                except (json.JSONDecodeError, TypeError):
                    cur_lat = None
            if cur_lat and "stages" in cur_lat:
                parts.append(f"\n**Latency stages ({sc.name}):**\n")
                parts.append("| Stage | p50 | p95 | p99 |")
                parts.append("|-------|-----|-----|-----|")
                for stage, vals in cur_lat["stages"].items():
                    p50 = vals.get("p50_ns", 0) / 1_000_000
                    p95 = vals.get("p95_ns", 0) / 1_000_000
                    p99 = vals.get("p99_ns", 0) / 1_000_000
                    parts.append(f"| {stage} | {p50:.1f}ms | {p95:.1f}ms | {p99:.1f}ms |")

    parts.append(f"\n### Thresholds\n")
    parts.append("| Metric | Threshold |")
    parts.append("|--------|-----------|")
    for k, v in thresholds.items():
        parts.append(f"| {k} | {v * 100:.0f}% |")

    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description="Perception benchmark dashboard renderer")
    parser.add_argument("--baseline", required=True,
                        help="Path to baseline JSON")
    parser.add_argument("--current", required=True,
                        help="Path to current run JSON")
    parser.add_argument("--mode", choices=["pr-comment", "full"],
                        default="pr-comment",
                        help="Output mode (default: pr-comment)")
    parser.add_argument("--thresholds", default=None,
                        help="JSON string of threshold overrides")
    args = parser.parse_args()

    baseline = load_baseline(args.baseline)
    if baseline is None:
        return 2
    current = load_baseline(args.current)
    if current is None:
        return 2

    thresholds = dict(DEFAULT_THRESHOLDS)
    if args.thresholds:
        try:
            overrides = json.loads(args.thresholds)
            thresholds.update(overrides)
        except json.JSONDecodeError as e:
            print(f"Error: invalid thresholds JSON: {e}", file=sys.stderr)
            return 2

    result = compare_scenarios(baseline, current, thresholds)

    if args.mode == "pr-comment":
        print(render_pr_comment(result, thresholds), end="")
    else:
        print(render_full_report(result, baseline, current, thresholds), end="")

    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
