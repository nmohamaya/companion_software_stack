"""Health report command — test count + coverage vs baseline.

Replaces health-report-diff.sh (83 lines).
"""

from __future__ import annotations

from pathlib import Path

from orchestrator.build import BuildSystem
from orchestrator.config import resolve_project_dir
from orchestrator.console import Console


def run(io: Console | None = None) -> int:
    """Print health report: test count, coverage, build status."""
    if io is None:
        io = Console()

    project_dir = resolve_project_dir()
    build = BuildSystem(project_dir)

    io.header("Health Report")

    # Test count
    current_tests = build.test_count() if build.has_build_dir() else 0
    baseline_tests = build.expected_test_count()

    if current_tests and baseline_tests:
        if current_tests >= baseline_tests:
            io.pass_(f"Test count: {current_tests} (baseline: {baseline_tests})")
        else:
            io.fail(
                f"Test count: {current_tests} (baseline: {baseline_tests}) "
                f"BELOW BASELINE"
            )
    elif current_tests:
        io.warn(f"Test count: {current_tests} (no baseline found)")
    elif baseline_tests:
        io.fail(f"No build found (baseline: {baseline_tests})")
    else:
        io.fail("No data available")

    # Coverage
    coverage_info = project_dir / "build" / "coverage.info"
    if coverage_info.exists():
        pct = build.coverage_percent()
        if pct is not None:
            if pct >= 70.0:
                io.pass_(f"Coverage: {pct}%")
            else:
                io.warn(f"Coverage: {pct}% (below 70%)")
        else:
            io.warn("coverage.info exists but could not parse")
    else:
        io.warn("No coverage.info found (run: deploy/build.sh --coverage)")

    # Build status
    if build.has_build_dir():
        if build.has_binaries():
            io.pass_("Build configured with executables")
        else:
            io.warn("Build dir exists, no executables found")
    else:
        io.fail("No build directory")

    return 0
