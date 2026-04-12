# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Build system subprocess wrapper — cmake, ctest, clang-format, lcov.

Wraps build tool CLI calls used by:
  - validate-session.sh (build, test count, test execution, format check)
  - run-session.sh (health baseline test count)
  - health-report-diff.sh (test count, coverage)
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


class BuildError(Exception):
    """Raised when a build command fails."""

    def __init__(self, tool: str, returncode: int, output: str) -> None:
        self.tool = tool
        self.returncode = returncode
        self.output = output
        super().__init__(f"{tool} failed (exit {returncode})")


class BuildSystem:
    """Thin wrapper around build tool subprocess calls."""

    def __init__(self, project_dir: Path) -> None:
        self.project_dir = project_dir
        self.build_dir = project_dir / "build"

    def _run(
        self,
        *args: str,
        check: bool = False,
        timeout: int = 300,
    ) -> subprocess.CompletedProcess[str]:
        """Run a build command."""
        return subprocess.run(
            args,
            capture_output=True,
            text=True,
            cwd=str(self.project_dir),
            timeout=timeout,
        )

    # ── Build ──────────────────────────────────────────────────────────────

    def build(self, jobs: int | None = None) -> tuple[bool, str, int]:
        """Run cmake --build. Returns (success, output, warning_count)."""
        args = ["cmake", "--build", str(self.build_dir)]
        if jobs:
            args.extend([f"-j{jobs}"])
        else:
            args.extend(["-j"])  # use all cores

        result = self._run(*args, timeout=600)
        output = result.stdout + result.stderr

        warning_count = len(re.findall(r"warning:", output, re.IGNORECASE))

        return (result.returncode == 0, output, warning_count)

    def has_build_dir(self) -> bool:
        """Check if build directory exists."""
        return self.build_dir.exists()

    def has_binaries(self) -> bool:
        """Check if build/bin/ directory exists."""
        return (self.build_dir / "bin").exists()

    # ── Tests ──────────────────────────────────────────────────────────────

    def test_count(self) -> int:
        """Get the total number of registered tests (ctest -N)."""
        result = self._run(
            "ctest", "-N", "--test-dir", str(self.build_dir),
        )
        for line in result.stdout.splitlines():
            if "Total Tests:" in line:
                parts = line.strip().split()
                try:
                    return int(parts[-1])
                except (ValueError, IndexError):
                    pass
        return 0

    def run_tests(self, jobs: int | None = None) -> tuple[str, int]:
        """Run all tests via ctest. Returns (output, exit_code)."""
        args = [
            "ctest", "--test-dir", str(self.build_dir),
            "--output-on-failure",
        ]
        if jobs:
            args.extend([f"-j{jobs}"])
        else:
            args.append("-j")

        result = self._run(*args, timeout=600)
        output = result.stdout + result.stderr
        return (output, result.returncode)

    def parse_test_results(self, output: str) -> dict[str, str]:
        """Parse ctest output for key metrics."""
        results: dict[str, str] = {}

        # Look for "X% tests passed, Y tests failed"
        match = re.search(r"(\d+)% tests passed", output)
        if match:
            results["pass_rate"] = match.group(0)

        match = re.search(r"(\d+) tests? failed", output)
        if match:
            results["failures"] = match.group(0)

        return results

    # ── Format ─────────────────────────────────────────────────────────────

    def check_format(self, files: list[str]) -> int:
        """Check clang-format compliance for given files. Returns issue count."""
        issues = 0
        for filepath in files:
            full_path = self.project_dir / filepath
            if not full_path.exists():
                continue
            result = self._run(
                "clang-format-18", "--dry-run", "--Werror", str(full_path),
            )
            if result.returncode != 0:
                issues += 1
        return issues

    def has_clang_format(self) -> bool:
        """Check if clang-format-18 is available."""
        result = self._run("clang-format-18", "--version")
        return result.returncode == 0

    # ── Coverage ───────────────────────────────────────────────────────────

    def coverage_percent(self) -> float | None:
        """Get line coverage percentage from lcov. Returns None if unavailable."""
        info_file = self.build_dir / "coverage.info"
        if not info_file.exists():
            return None

        result = self._run("lcov", "--summary", str(info_file))
        output = result.stdout + result.stderr

        match = re.search(r"lines[.\s]*:\s*([\d.]+)%", output)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                pass
        return None

    # ── Expected test count ────────────────────────────────────────────────

    def expected_test_count(self) -> int | None:
        """Extract expected test count from tests/TESTS.md.

        The Total row format is:
          | **Total** | **56 C++ + 5 shell** | **1259 + 42 + 250+** | |
        We need the ctest-registered count (1259), not the file count (56).
        Strategy: find the Total row, extract all numbers >= 100, return the
        largest — that's the unit test baseline.
        """
        tests_md = self.project_dir / "tests" / "TESTS.md"
        if not tests_md.exists():
            return None

        content = tests_md.read_text()

        # Find lines containing "Total" (case-insensitive)
        for line in content.splitlines():
            if re.search(r"(?i)\btotal\b", line):
                # Extract all numbers >= 100 on this line
                numbers = [int(n) for n in re.findall(r"\d+", line) if int(n) >= 100]
                if numbers:
                    return max(numbers)

        return None
