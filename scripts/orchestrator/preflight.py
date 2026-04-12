# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Preflight checks — run before launching an agent.

Replaces the preflight checks in start-agent.sh (lines 140-186) and
similar checks scattered across other scripts.

6 checks, each returning (status, message):
  1. Agent definition file exists
  2. Git repository valid
  3. Current branch name
  4. Uncommitted changes
  5. Build directory exists
  6. clang-format-18 available
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from orchestrator.console import IOProtocol
from orchestrator.git import Git


class CheckStatus(Enum):
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"
    INFO = "info"


@dataclass
class CheckResult:
    status: CheckStatus
    message: str


def check_agent_file(project_dir: Path, role: str) -> CheckResult:
    """Check that the agent definition file exists."""
    agent_file = project_dir / ".claude" / "agents" / f"{role}.md"
    if agent_file.exists():
        return CheckResult(CheckStatus.PASS, f"Agent file: {role}.md")
    return CheckResult(CheckStatus.FAIL, f"Agent file not found: {agent_file}")


def check_git_repo(git: Git) -> CheckResult:
    """Check that the directory is a git repository."""
    if git.is_repo():
        return CheckResult(CheckStatus.PASS, "Git repository")
    return CheckResult(CheckStatus.FAIL, "Not a git repository")


def check_branch(git: Git) -> CheckResult:
    """Report the current branch name."""
    branch = git.current_branch()
    return CheckResult(CheckStatus.INFO, f"Branch: {branch}")


def check_uncommitted(git: Git) -> CheckResult:
    """Check for uncommitted changes."""
    if git.has_uncommitted_changes():
        count = git.uncommitted_count()
        return CheckResult(CheckStatus.WARN, f"{count} uncommitted file(s)")
    return CheckResult(CheckStatus.PASS, "Working tree clean")


def check_build_dir(project_dir: Path) -> CheckResult:
    """Check if build/bin/ directory exists."""
    if (project_dir / "build" / "bin").exists():
        return CheckResult(CheckStatus.PASS, "Build directory exists")
    return CheckResult(CheckStatus.WARN, "No build/bin/ — build may be needed")


def check_clang_format() -> CheckResult:
    """Check if clang-format-18 is available."""
    if shutil.which("clang-format-18"):
        return CheckResult(CheckStatus.PASS, "clang-format-18 available")
    return CheckResult(CheckStatus.WARN, "clang-format-18 not found in PATH")


def run_preflight(
    project_dir: Path,
    role: str,
    git: Git,
    io: IOProtocol,
) -> list[CheckResult]:
    """Run all preflight checks and display results.

    Returns the list of CheckResults. Raises SystemExit if any check FAILs.
    """
    checks = [
        check_agent_file(project_dir, role),
        check_git_repo(git),
        check_branch(git),
        check_uncommitted(git),
        check_build_dir(project_dir),
        check_clang_format(),
    ]

    has_failure = False
    for result in checks:
        if result.status == CheckStatus.PASS:
            io.pass_(result.message)
        elif result.status == CheckStatus.WARN:
            io.warn(result.message)
        elif result.status == CheckStatus.FAIL:
            io.fail(result.message)
            has_failure = True
        else:
            io.info(result.message)

    if has_failure:
        raise SystemExit(1)

    return checks
