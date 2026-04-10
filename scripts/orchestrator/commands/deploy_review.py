"""Deploy review agents for a PR — asyncio parallel launch.

Replaces deploy-review.sh (357 lines).
Fetches PR diff, routes reviewers/testers, launches them in parallel
via asyncio, collects output, posts consolidated findings as PR comment.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from pathlib import Path

from orchestrator.claude import AsyncClaude
from orchestrator.config import ROLES, resolve_project_dir
from orchestrator.console import Console
from orchestrator.github import GitHub
from orchestrator.routing import route_review_agents


def run(
    pr_number: int,
    *,
    io: Console | None = None,
    github: GitHub | None = None,
    force_all: bool = False,
    dry_run: bool = False,
) -> int:
    """Launch review agents for a PR."""
    if io is None:
        io = Console()
    if github is None:
        github = GitHub()

    project_dir = resolve_project_dir()

    # 1. Fetch PR diff
    io.print(f"Fetching PR #{pr_number} diff...")
    try:
        diff = github.pr_diff(pr_number)
    except Exception as e:
        io.error(f"Failed to fetch PR #{pr_number}: {e}")
        return 1

    diff_lines = diff.count("\n")
    io.print(f"  Diff size: {diff_lines} lines")
    io.print("")

    # 2. Route reviewers
    routing = route_review_agents(diff, force_all=force_all)

    io.print("Reviewers to launch:")
    for r in routing.reviewers:
        io.print(f"  {r}")
    io.print("")

    io.print("Test agents to launch:")
    for t in routing.testers:
        io.print(f"  {t}")
    io.print("")

    io.print("Pass 2 agents to launch:")
    for p2 in routing.pass2_reviewers:
        io.print(f"  {p2}")
    io.print("")

    if dry_run:
        io.warn(
            f"DRY RUN — would launch {len(routing.reviewers)} reviewers + "
            f"{len(routing.testers)} test agents (pass 1) + "
            f"{len(routing.pass2_reviewers)} quality agents (pass 2) "
            f"for PR #{pr_number}"
        )
        io.print("")
        io.print("Routing reasons:")
        for agent, reason in routing.reasons.items():
            io.print(f"  {agent}: {reason}")
        return 0

    # 3. Get PR branch for test agents
    try:
        pr = github.pr_view(pr_number)
        pr_branch = pr.head_branch if pr else ""
    except Exception:
        io.error("Cannot determine PR branch")
        return 1

    # Build prompts
    diff_files = "\n".join(
        line.replace("diff --git a/", "").split(" b/")[0]
        for line in diff.splitlines()
        if line.startswith("diff --git")
    )
    diff_truncated = "\n".join(diff.splitlines()[:500])

    review_prompt = (
        f"Review PR #{pr_number}.\n\n"
        f"Changed files:\n{diff_files}\n\n"
        f"Diff (first 500 lines):\n{diff_truncated}\n\n"
        f"Provide a detailed safety review focused on your domain. "
        f"Post findings as a structured list with file:line, severity "
        f"(P1-P4), and fix suggestion."
    )

    test_unit_prompt = (
        f"Verify PR #{pr_number} (branch: {pr_branch}).\n\n"
        f"Changed files:\n{diff_files}\n\n"
        f"1. Check out branch: git checkout {pr_branch}\n"
        f"2. Build: bash deploy/build.sh\n"
        f"3. Run all tests: ./tests/run_tests.sh\n"
        f"4. Verify test count matches baseline\n"
        f"5. Run module-specific tests for changed files\n"
        f"6. Check formatting\n\n"
        f"Report: build result, test results, count vs baseline, "
        f"failures, formatting issues."
    )

    test_scenario_prompt = (
        f"Verify PR #{pr_number} (branch: {pr_branch}) with scenarios.\n\n"
        f"Changed files:\n{diff_files}\n\n"
        f"1. Check out branch: git checkout {pr_branch}\n"
        f"2. Build: bash deploy/build.sh\n"
        f"3. Run scenario tests: tests/run_scenario.sh\n"
        f"4. Verify full stack coverage in configs\n\n"
        f"Report: scenario results, regressions, config issues."
    )

    # 4. Launch Pass 1 agents (safety & correctness)
    pass1_agents = routing.reviewers + routing.testers
    session_dir = project_dir / "tasks" / "sessions"
    session_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%d-%H%M")

    io.print(f"Pass 1: Launching {len(pass1_agents)} agents in parallel...")

    pass1_configs = []
    for agent in pass1_agents:
        role_config = ROLES[agent]
        if agent == "test-unit":
            prompt = test_unit_prompt
        elif agent == "test-scenario":
            prompt = test_scenario_prompt
        else:
            prompt = review_prompt

        log_file = session_dir / f"{timestamp}-review-pr{pr_number}-{agent}.log"
        pass1_configs.append({
            "agent": agent,
            "model": role_config.model,
            "prompt": prompt,
            "log_file": log_file,
        })
        io.print(f"  Launching {agent} (log: {log_file.name})")

    pass1_results = asyncio.run(_launch_agents(pass1_configs))

    io.print("")
    failed = 0
    for agent, success in pass1_results.items():
        if success:
            io.pass_(agent)
        else:
            io.warn(f"{agent} exited with non-zero status")
            failed += 1

    # 5. Collect Pass 1 findings for Pass 2 context
    pass1_findings_parts: list[str] = []
    for cfg in pass1_configs:
        log_file = cfg["log_file"]
        if log_file.exists() and log_file.stat().st_size > 0:
            content = log_file.read_text()
            lines = content.splitlines()[-100:]
            pass1_findings_parts.append(
                f"--- {cfg['agent']} ---\n" + "\n".join(lines)
            )
    pass1_findings = "\n\n".join(pass1_findings_parts)

    # 6. Launch Pass 2 agents (quality & contracts) with Pass 1 context
    pass2_configs: list[dict] = []
    if routing.pass2_reviewers:
        io.print("")
        io.print(
            f"Pass 2: Launching {len(routing.pass2_reviewers)} agents "
            f"in parallel (with Pass 1 findings as context)..."
        )

        pass2_prompt = (
            f"Review PR #{pr_number} (Pass 2 — quality & contracts).\n\n"
            f"Changed files:\n{diff_files}\n\n"
            f"Diff (first 500 lines):\n{diff_truncated}\n\n"
            f"## Pass 1 Findings (from safety & correctness reviewers)\n\n"
            f"{pass1_findings}\n\n"
            f"Your job: provide a quality/contract review focused on your "
            f"domain. Build on Pass 1 findings — don't duplicate them. "
            f"Post findings as a structured list with file:line, severity "
            f"(P1-P4), and fix suggestion."
        )

        for agent in routing.pass2_reviewers:
            role_config = ROLES[agent]
            log_file = (
                session_dir / f"{timestamp}-review-pr{pr_number}-{agent}.log"
            )
            pass2_configs.append({
                "agent": agent,
                "model": role_config.model,
                "prompt": pass2_prompt,
                "log_file": log_file,
            })
            io.print(f"  Launching {agent} (log: {log_file.name})")

        pass2_results = asyncio.run(_launch_agents(pass2_configs))

        io.print("")
        for agent, success in pass2_results.items():
            if success:
                io.pass_(agent)
            else:
                io.warn(f"{agent} exited with non-zero status")
                failed += 1

    # 7. Consolidate findings from both passes
    all_configs = pass1_configs + pass2_configs
    io.print("")
    io.print("Consolidating findings...")

    date_str = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    sections = [
        f"## Automated Two-Pass Review — PR #{pr_number}\n",
        f"**Pass 1 (safety & correctness):** {' '.join(pass1_agents)}",
        f"**Pass 2 (quality & contracts):** "
        f"{' '.join(routing.pass2_reviewers)}",
        f"**Date:** {date_str}\n",
        "---\n",
        "# Pass 1: Safety & Correctness Findings\n",
    ]

    for cfg in pass1_configs:
        agent = cfg["agent"]
        log_file = cfg["log_file"]
        sections.append(f"### {agent}\n")
        if log_file.exists() and log_file.stat().st_size > 0:
            content = log_file.read_text()
            lines = content.splitlines()[-200:]
            sections.append("\n".join(lines))
        else:
            sections.append("_No output captured._")
        sections.append("")

    if pass2_configs:
        sections.append("---\n")
        sections.append("# Pass 2: Quality & Contract Findings\n")

        for cfg in pass2_configs:
            agent = cfg["agent"]
            log_file = cfg["log_file"]
            sections.append(f"### {agent}\n")
            if log_file.exists() and log_file.stat().st_size > 0:
                content = log_file.read_text()
                lines = content.splitlines()[-200:]
                sections.append("\n".join(lines))
            else:
                sections.append("_No output captured._")
            sections.append("")

    sections.append("---")
    sections.append(
        f"_Generated by orchestrator deploy-review — "
        f"{len(pass1_agents)} pass 1 + "
        f"{len(routing.pass2_reviewers)} pass 2 agents_"
    )

    consolidated = "\n".join(sections)

    # 8. Post as PR comment
    io.print(f"Posting consolidated review to PR #{pr_number}...")
    try:
        github.pr_comment(pr_number, consolidated)
        io.pass_(f"Review posted to PR #{pr_number}")
    except Exception:
        io.fail("Could not post review comment")
        io.print("Logs:")
        for cfg in all_configs:
            io.print(f"  {cfg['log_file']}")
        return 1

    io.print("")
    io.print("Review complete")
    io.print(f"  PR:          #{pr_number}")
    io.print(f"  Pass 1:      {len(pass1_agents)} agents")
    io.print(f"  Pass 2:      {len(routing.pass2_reviewers)} agents")
    io.print(f"  Failed:      {failed}")

    return 0


async def _launch_agents(
    configs: list[dict],
) -> dict[str, bool]:
    """Launch all agents in parallel and return results."""
    aclaude = AsyncClaude()
    results: dict[str, bool] = {}

    async def run_one(cfg: dict) -> None:
        try:
            await aclaude.launch_agent(
                model=cfg["model"],
                agent=cfg["agent"],
                prompt=cfg["prompt"],
                log_file=cfg["log_file"],
                timeout=1800,  # 30 min
            )
            results[cfg["agent"]] = True
        except Exception:
            results[cfg["agent"]] = False

    await asyncio.gather(*[run_one(cfg) for cfg in configs])
    return results
