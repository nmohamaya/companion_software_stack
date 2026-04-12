# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Orchestrate command — tech-lead driven issue/epic workflow.

New command (Phase 10, no bash equivalent).

Usage:
  python -m orchestrator orchestrate <issue-number>
  python -m orchestrator orchestrate <epic-number> --epic

For single issues, this delegates to deploy-issue (which now includes
tech-lead orchestration by default). The orchestrate command remains
the entry point for epic (multi-issue) orchestration with phased plans.
"""

from __future__ import annotations

from pathlib import Path

from orchestrator.config import resolve_project_dir
from orchestrator.console import Console
from orchestrator.claude import Claude
from orchestrator.github import GitHub
from orchestrator.tech_lead import TechLead


def run(
    number: int,
    *,
    io: Console | None = None,
    github: GitHub | None = None,
    claude: Claude | None = None,
    epic: bool = False,
    dry_run: bool = False,
) -> int:
    """Run tech-lead orchestrated workflow."""
    if io is None:
        io = Console()
    if github is None:
        github = GitHub()
    if claude is None:
        claude = Claude()

    if epic:
        project_dir = resolve_project_dir()
        tech_lead = TechLead(claude=claude, github=github, io=io)
        return _orchestrate_epic(
            tech_lead, number, io=io, github=github, claude=claude,
            project_dir=project_dir, dry_run=dry_run,
        )

    # Single issue: delegate to deploy-issue (includes tech-lead by default)
    from orchestrator.commands.deploy_issue import run as deploy_run

    return deploy_run(
        number,
        io=io,
        github=github,
        claude=claude,
        dry_run=dry_run,
    )


def _orchestrate_epic(
    tech_lead: TechLead,
    epic_number: int,
    *,
    io: Console,
    github: GitHub,
    claude: Claude,
    project_dir: Path,
    dry_run: bool,
) -> int:
    """Orchestrate an epic (multi-issue) with tech-lead phased plan."""
    io.header(f"Tech-Lead Orchestration — Epic #{epic_number}")

    # 1. Tech-lead produces phased plan
    io.print("[1/3] Tech-lead epic analysis")
    plan = tech_lead.plan_epic(epic_number, project_dir=project_dir)

    io.print("")
    io.print(f"  Epic:    #{plan.epic_number} — {plan.epic_title}")
    io.print(f"  Issues:  {plan.total_issues}")
    io.print(f"  Waves:   {plan.estimated_waves}")
    io.print("")

    for i, phase in enumerate(plan.phases, 1):
        io.print(f"  Phase {i}: {phase.name}")
        io.print(f"    Issues: {phase.issues}")
        io.print(f"    {phase.description}")
        if phase.depends_on:
            io.print(f"    Depends on: {', '.join(phase.depends_on)}")
        io.print("")

    if dry_run:
        io.info("Dry-run mode — showing plan only.")
        return 0

    # 2. Human approves plan
    io.print("[2/3] Human approval of epic plan")
    io.options([
        ("a", "accept — proceed with phased plan"),
        ("r", "reject — cancel orchestration"),
    ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("r", "reject"):
        io.print("  Orchestration cancelled.")
        return 0

    # 3. Execute phases sequentially
    io.print("")
    io.print("[3/3] Executing phases")

    for i, phase in enumerate(plan.phases, 1):
        io.header(f"Phase {i}/{len(plan.phases)}: {phase.name}")

        for issue_num in phase.issues:
            io.print(f"  Orchestrating issue #{issue_num}...")
            result = _orchestrate_issue(
                tech_lead, issue_num,
                io=io, github=github, claude=claude,
                project_dir=project_dir, dry_run=False,
            )
            if result != 0:
                io.warn(
                    f"Issue #{issue_num} did not complete successfully. "
                    f"Continue with next issue?"
                )
                if not io.confirm("  Continue?", default=True):
                    return result

        io.pass_(f"Phase {i} complete")

        # Pause between phases for human review
        if i < len(plan.phases):
            io.print("")
            if not io.confirm(f"  Proceed to Phase {i + 1}?", default=True):
                io.print("  Paused. Resume manually for remaining phases.")
                return 0

    io.pass_("Epic orchestration complete")
    return 0
