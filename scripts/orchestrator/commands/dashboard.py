"""Agent dashboard command — per-agent and team-wide metrics.

Replaces agent-dashboard.sh (256 lines).
"""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

from orchestrator.config import resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git
from orchestrator.github import GitHub


def run(
    io: Console | None = None,
    git: Git | None = None,
    github: GitHub | None = None,
    *,
    agent: str = "",
    team: bool = False,
    since: str = "",
) -> int:
    """Print agent dashboard."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())
    if github is None:
        github = GitHub()

    # Default to team mode
    if not agent and not team:
        team = True

    if agent:
        _agent_report(io, git, github, agent, since)

    if team:
        _team_report(io, git, github, since)

    return 0


def _agent_report(
    io: Console,
    git: Git,
    github: GitHub,
    role: str,
    since: str,
) -> None:
    """Per-agent report from changelog + git."""
    project_dir = resolve_project_dir()
    changelog = project_dir / "tasks" / "agent-changelog.md"

    io.header(f"Agent Report: {role}")

    # Sessions from changelog
    sessions = 0
    if changelog.exists():
        text = changelog.read_text()
        sessions = text.lower().count(role.lower())
    io.print(f"  Sessions referenced:  {sessions} entries in changelog")

    # Commits
    try:
        commits = git.log_grep(role, since=since if since else None)
        io.print(f"  Commits:              {len(commits)}")
    except Exception:
        io.print("  Commits:              N/A")

    # PRs
    try:
        prs = github.pr_list(state="all", limit=100)
        role_prs = [p for p in prs if role in p.title.lower()]
        io.print(f"  PRs:                  {len(role_prs)}")
    except Exception:
        io.print("  PRs:                  N/A")

    # Parse changelog for details
    if changelog.exists():
        text = changelog.read_text()
        role_lines = [l for l in text.splitlines() if role.lower() in l.lower()]

        pass_count = sum(1 for l in role_lines if "PASS" in l)
        warn_count = sum(1 for l in role_lines if "WARN" in l)
        fail_count = sum(1 for l in role_lines if "FAIL" in l)

        io.print(
            f"  Validation:           PASS={pass_count} WARN={warn_count} "
            f"FAIL={fail_count}"
        )

    io.print("")


def _team_report(
    io: Console,
    git: Git,
    github: GitHub,
    since: str,
) -> None:
    """Aggregate team dashboard."""
    project_dir = resolve_project_dir()
    changelog = project_dir / "tasks" / "agent-changelog.md"

    io.header("Team Dashboard")

    # Pipeline throughput
    io.print("--- Pipeline Throughput ---")
    try:
        closed = github.issue_list(state="closed", limit=500)
        io.print(f"  Issues closed:      {len(closed)}")
    except Exception:
        io.print("  Issues closed:      N/A")

    try:
        open_issues = github.issue_list(state="open", limit=500)
        io.print(f"  Issues open:        {len(open_issues)}")
    except Exception:
        io.print("  Issues open:        N/A")

    try:
        open_prs = github.pr_list(state="open", limit=100)
        io.print(f"  PRs open:           {len(open_prs)}")
    except Exception:
        io.print("  PRs open:           N/A")
    io.print("")

    # Activity by role from changelog
    if changelog.exists():
        text = changelog.read_text()
        io.print("--- Activity by Role ---")
        for role_name in (
            "perception", "nav", "integration",
            "infra-core", "infra-platform",
        ):
            count = text.lower().count(role_name)
            if count > 0:
                io.print(f"  {role_name:<20} {count} entries")
        io.print("")

    # Git commit summary
    io.print("--- Git Commit Summary ---")
    try:
        log_lines = git.log_oneline(since=since if since else None)
        total = len(log_lines)
        io.print(f"  Total commits:  {total}")
    except Exception:
        io.print("  Total commits:  N/A")
    io.print("")
