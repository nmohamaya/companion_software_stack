# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Agent commit statistics command.

Replaces agent-stats.sh (131 lines).
Analyzes git history for agent commits by model, type, and role.
"""

from __future__ import annotations

import re
from collections import Counter

from orchestrator.config import resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git


def run(
    io: Console | None = None,
    git: Git | None = None,
    *,
    since: str = "30 days ago",
    all_time: bool = False,
) -> int:
    """Print agent commit statistics."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())

    period = "all time" if all_time else since
    io.header("Agent Commit Statistics")
    io.print(f"  Period: {period}")
    io.print("")

    # Get commit log with trailers
    since_arg = None if all_time else since
    log_lines = git.log_oneline(since=since_arg) if since_arg else git.log_oneline()
    total_commits = len(log_lines)

    # Parse agent commits from full log with trailers
    model_counts: Counter[str] = Counter()
    type_counts: Counter[str] = Counter()
    role_counts: Counter[str] = Counter()
    agent_count = 0

    # Get detailed log with trailers
    try:
        from orchestrator.git import GitError
        result = git._run(
            "log",
            *(["--since", since] if since_arg else []),
            "--format=%H %s%n%(trailers:key=Co-Authored-By,valueonly)",
        )
        lines = (result.stdout or "").splitlines()
    except Exception:
        lines = []

    current_subject = ""
    current_hash = ""

    for line in lines:
        if not line.strip():
            continue

        # Commit line: starts with 40-char hash
        if re.match(r"^[0-9a-f]{40} ", line):
            parts = line.split(" ", 1)
            current_hash = parts[0]
            current_subject = parts[1] if len(parts) > 1 else ""
            continue

        # Co-Authored-By line
        if "Claude" in line:
            agent_count += 1

            # Model
            if "Opus" in line:
                model_counts["Opus"] += 1
            elif "Sonnet" in line:
                model_counts["Sonnet"] += 1
            elif "Haiku" in line:
                model_counts["Haiku"] += 1
            else:
                model_counts["Unknown"] += 1

            # Commit type
            match = re.match(
                r"^(feat|fix|refactor|test|docs|chore|perf)[:(]", current_subject
            )
            type_counts[match.group(1) if match else "other"] += 1

            # Role from branch (simplified — uses subject heuristics)
            role = _guess_role(current_subject)
            role_counts[role] += 1

    # Print results
    io.print("--- By Model ---")
    for model, count in model_counts.most_common():
        io.print(f"  {model:<12} {count}")
    io.print("")

    io.print("--- By Type ---")
    for typ, count in type_counts.most_common():
        io.print(f"  {typ:<12} {count}")
    io.print("")

    io.print("--- By Role ---")
    for role, count in role_counts.most_common():
        io.print(f"  {role:<16} {count}")
    io.print("")

    io.print("--- Summary ---")
    pct = (agent_count * 100 // total_commits) if total_commits > 0 else 0
    io.print(
        f"  Agent commits:  {agent_count} / {total_commits} total ({pct}%)"
    )

    return 0


def _guess_role(subject: str) -> str:
    """Guess agent role from commit subject."""
    s = subject.lower()
    if any(w in s for w in ("perception", "camera", "detector", "tracker")):
        return "perception"
    if any(w in s for w in ("nav", "mission", "slam", "planner")):
        return "nav"
    if any(w in s for w in ("comms", "monitor", "integration")):
        return "integration"
    if any(w in s for w in ("infra", "common", "modularity", "config")):
        return "infra-core"
    if any(w in s for w in ("deploy", "ci", "platform", "systemd")):
        return "infra-platform"
    if any(w in s for w in ("doc",)):
        return "docs"
    return "unknown"
