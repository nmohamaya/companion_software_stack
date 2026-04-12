# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Full session orchestrator — 6-phase wrapper.

Replaces run-session.sh (253 lines).
Phases: preflight → baseline → agent → validate → metrics → changelog.
"""

from __future__ import annotations

import time
from datetime import datetime, timezone
from pathlib import Path

from orchestrator.config import get_role, resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git
from orchestrator.github import GitHub
from orchestrator.build import BuildSystem
from orchestrator.claude import Claude


def run(
    role: str,
    task: str,
    *,
    io: Console | None = None,
    git: Git | None = None,
    github: GitHub | None = None,
    claude: Claude | None = None,
    issue: int | None = None,
) -> int:
    """Run a full orchestrated session."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())
    if github is None:
        github = GitHub()
    if claude is None:
        claude = Claude()

    project_dir = resolve_project_dir()
    build = BuildSystem(project_dir)
    role_config = get_role(role)

    start_time = time.monotonic()
    now = datetime.now(timezone.utc)
    session_ts = now.strftime("%Y-%m-%d-%H%M")

    # Session log
    session_dir = project_dir / "tasks" / "sessions"
    session_dir.mkdir(parents=True, exist_ok=True)
    session_log = session_dir / f"{session_ts}-{role}.log"

    io.header(f"Session: {role} @ {session_ts}")

    # ── 1. Pre-flight ──────────────────────────────────────────────────
    io.print("[1/6] Pre-flight checks")
    branch = git.current_branch()
    io.info(f"Branch: {branch}")

    if git.has_uncommitted_changes():
        count = git.uncommitted_count()
        io.warn(f"{count} uncommitted file(s)")
    else:
        io.pass_("Working tree clean")

    try:
        behind = git.behind_remote()
        if behind > 0:
            io.warn(f"Branch is {behind} commit(s) behind remote")
    except Exception:
        pass

    if build.has_build_dir():
        io.pass_("Build directory exists")
    else:
        io.warn("No build/bin/ — build may be needed")

    start_commit = git.head_sha()
    io.print("")

    # ── 2. Health baseline ─────────────────────────────────────────────
    io.print("[2/6] Health baseline")
    baseline_tests = build.test_count() if build.has_build_dir() else 0
    io.print(f"  Test count (baseline): {baseline_tests}")

    if build.has_build_dir() and build.has_clang_format():
        try:
            fmt_result = build.check_format()
            if fmt_result.returncode == 0:
                io.pass_("Changed files pass format check")
            else:
                io.warn("Format issues detected")
        except Exception:
            io.info("Could not run format check")
    io.print("")

    # ── 3. Launch agent ────────────────────────────────────────────────
    io.print("[3/6] Launching agent")
    io.print(f"  Role: {role}")
    task_display = task[:80] + ("..." if len(task) > 80 else "")
    io.print(f"  Task: {task_display}")
    io.print("")

    agent_exit = 0
    try:
        claude.launch_print(
            model=role_config.model,
            agent=role,
            prompt=task,
        )
    except Exception:
        agent_exit = 1
    io.print("")

    # ── 4. Post-session validation ─────────────────────────────────────
    io.print("[4/6] Post-session validation")
    from orchestrator.commands.validate_session import run as validate_run
    validate_exit = validate_run(
        io=io, git=git, github=github, build=build, branch=branch
    )
    validate_result = "FAIL" if validate_exit else "PASS"
    io.print("")

    # ── 5. Post-session metrics ────────────────────────────────────────
    io.print("[5/6] Post-session report")

    elapsed = int(time.monotonic() - start_time)
    elapsed_min, elapsed_sec = divmod(elapsed, 60)

    post_tests = build.test_count() if build.has_build_dir() else 0
    post_uncommitted = git.uncommitted_count()

    try:
        new_commits = git.commit_count_since(start_commit)
    except Exception:
        new_commits = 0

    io.print(f"  Tests:     {baseline_tests} -> {post_tests}")
    io.print(f"  Uncommitted files: {post_uncommitted}")
    io.print(f"  New commits:       {new_commits}")
    io.print(f"  Elapsed:           {elapsed_min}m {elapsed_sec}s")
    io.print(f"  Agent exit code:   {agent_exit}")
    io.print(f"  Validation:        {validate_result}")
    io.print("")

    # PR reference
    pr_ref = "no PR"
    try:
        prs = github.pr_list_for_branch(branch)
        if prs:
            pr_ref = f"PR #{prs[0].number}"
    except Exception:
        pass

    # ── 6. Append to changelog ─────────────────────────────────────────
    io.print("[6/6] Updating agent changelog")

    changelog = project_dir / "tasks" / "agent-changelog.md"
    if not changelog.exists():
        changelog.write_text("# Agent Changelog\n\n")

    session_date = now.strftime("%Y-%m-%d")
    issue_ref = f" | #{issue}" if issue else ""

    entry = (
        f"\n### {session_date} | {role} | {role_config.tier.value} | "
        f"{pr_ref}{issue_ref}\n"
        f"- **Task:** {task[:120]}\n"
        f"- **Branch:** {branch}\n"
        f"- **Tests:** {baseline_tests} -> {post_tests}\n"
        f"- **Commits:** {new_commits}\n"
        f"- **Duration:** {elapsed_min}m {elapsed_sec}s\n"
        f"- **Validation:** {validate_result}\n"
        f"- **Exit:** {agent_exit}\n"
        f"- **Log:** tasks/sessions/{session_log.name}\n"
    )
    with open(changelog, "a") as f:
        f.write(entry)

    io.pass_("Appended to tasks/agent-changelog.md")
    io.print(f"  Session log: {session_log}")
    io.print("")

    return agent_exit
