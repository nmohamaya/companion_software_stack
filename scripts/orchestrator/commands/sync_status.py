"""Auto-update project-status.md from live data.

New command (no bash equivalent).
Regenerates factual sections from git/gh/ctest while preserving the
human-authored ## Notes section.

Auto-derived sections:
  - Test count (ctest -N)
  - Open issues and PRs (gh)
  - Recently merged PRs (gh)
  - Active worktrees/branches (git)
  - Build status

Preserved section:
  - ## Notes — strategic priorities, architecture decisions, blockers
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path

from orchestrator.build import BuildSystem
from orchestrator.console import Console
from orchestrator.git import Git
from orchestrator.github import GitHub


def run(
    io: Console | None = None,
    git: Git | None = None,
    github: GitHub | None = None,
    build: BuildSystem | None = None,
    *,
    project_dir: Path | None = None,
) -> int:
    """Regenerate project-status.md preserving ## Notes."""
    if io is None:
        io = Console()
    if git is None:
        git = Git()
    if github is None:
        github = GitHub()

    if project_dir is None:
        from orchestrator.config import resolve_project_dir
        project_dir = resolve_project_dir()

    if build is None:
        build = BuildSystem(project_dir)

    status_path = (
        project_dir / ".claude" / "shared-context" / "project-status.md"
    )

    # Extract existing ## Notes section
    notes = ""
    if status_path.exists():
        text = status_path.read_text()
        match = re.search(r"(## Notes\b.*)", text, re.DOTALL)
        if match:
            notes = match.group(1)

    # Build new content
    sections: list[str] = []
    sections.append(f"# Project Status")
    sections.append(
        f"*Auto-generated on {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}*\n"
    )

    # Test count
    test_count = build.test_count() if build.has_build_dir() else 0
    baseline = build.expected_test_count()
    sections.append("## Test Count")
    if test_count:
        status = "OK" if test_count >= baseline else "BELOW BASELINE"
        sections.append(
            f"- Current: **{test_count}** (baseline: {baseline}) — {status}\n"
        )
    else:
        sections.append(f"- No build available (baseline: {baseline})\n")

    # Open issues and PRs
    sections.append("## Open Issues & PRs")
    try:
        issues = github.issue_list(state="open", limit=50)
        sections.append(f"- Open issues: **{len(issues)}**")
    except Exception:
        sections.append("- Open issues: *could not fetch*")

    try:
        prs = github.pr_list(state="open", limit=50)
        sections.append(f"- Open PRs: **{len(prs)}**")
    except Exception:
        sections.append("- Open PRs: *could not fetch*")
    sections.append("")

    # Recently merged PRs
    sections.append("## Recently Merged PRs")
    try:
        merged = github.pr_list(state="merged", limit=10)
        if merged:
            for pr in merged:
                sections.append(f"- #{pr.number}: {pr.title}")
        else:
            sections.append("- None")
    except Exception:
        sections.append("- *could not fetch*")
    sections.append("")

    # Active worktrees
    sections.append("## Active Worktrees")
    try:
        wts = git.worktree_list()
        if wts:
            for wt in wts:
                sections.append(f"- {wt}")
        else:
            sections.append("- None")
    except Exception:
        sections.append("- *could not fetch*")
    sections.append("")

    # Build status
    sections.append("## Build Status")
    if build.has_build_dir():
        if build.has_binaries():
            sections.append("- Build: **configured** with executables")
        else:
            sections.append("- Build: configured, no executables")
    else:
        sections.append("- Build: **not configured**")
    sections.append("")

    # Append preserved notes
    if notes:
        sections.append(notes)
    else:
        sections.append(
            "## Notes\n"
            "*Add strategic priorities, architecture decisions, "
            "and blockers here. This section is preserved across "
            "auto-updates.*\n"
        )

    # Write
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_text("\n".join(sections))

    io.pass_(f"Updated {status_path}")
    return 0
