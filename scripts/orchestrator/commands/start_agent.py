"""Start agent command — preflight, context, launch.

Replaces start-agent.sh (308 lines).
Validates role, runs preflight checks, gathers cross-agent context,
and launches the Claude CLI with the appropriate model and agent.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from orchestrator.config import ALL_ROLES, ROLES, get_role, resolve_project_dir
from orchestrator.console import Console
from orchestrator.context import format_context, gather_context
from orchestrator.claude import Claude
from orchestrator.git import Git
from orchestrator.preflight import run_preflight


def run(
    role: str,
    task: str = "",
    *,
    io: Console | None = None,
    git: Git | None = None,
    claude: Claude | None = None,
    interactive: bool = False,
    dry_run: bool = False,
    skip_preflight: bool = False,
) -> int:
    """Launch an agent session."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())
    if claude is None:
        claude = Claude()

    project_dir = resolve_project_dir()

    # Validate role
    try:
        role_config = get_role(role)
    except KeyError:
        io.error(f"Unknown role '{role}'")
        io.print(f"Available roles: {', '.join(ALL_ROLES)}")
        return 1

    model = role_config.model
    tier = role_config.tier.value

    if not task and not dry_run:
        io.info("No task provided — launching interactive session")

    # Preflight
    if not skip_preflight:
        run_preflight(
            project_dir=project_dir,
            role=role,
            git=git,
            io=io,
        )

    # Tool restriction notes
    if role_config.read_only:
        io.info(f"Review agents are read-only (enforced by agent file)")
    if role == "ops-github":
        io.info("ops-github: Bash restricted to gh CLI (enforced by agent file)")

    # Cross-agent context
    ctx = gather_context(project_dir)
    ctx_text = format_context(ctx)

    # Build prompt
    prompt = task
    if prompt and ctx_text:
        prompt = f"{prompt}{ctx_text}"
    elif ctx_text:
        prompt = f"Review the cross-agent context below for this session.{ctx_text}"

    # Set environment variable
    os.environ["CLAUDE_AGENT_ROLE"] = role

    # Summary
    task_display = task[:80] + "..." if len(task) > 80 else task
    mode = "dry-run (read-only)" if dry_run else "normal"

    io.print("")
    io.header("Agent configuration")
    io.print(f"  Role:       {role}")
    io.print(f"  Model:      {tier} ({model})")
    io.print(f"  Agent file: {project_dir / '.claude' / 'agents' / f'{role}.md'}")
    io.print(f"  Task:       {task_display or '<interactive>'}")
    io.print(f"  Mode:       {mode}")
    io.print("")

    if dry_run:
        io.info("Dry-run mode — not launching agent.")
        return 0

    # Launch
    if interactive and prompt:
        claude.launch_interactive_with_prompt(
            model=model, agent=role, prompt=prompt
        )
    elif prompt:
        claude.launch_print(model=model, agent=role, prompt=prompt)
    else:
        claude.launch_interactive(model=model, agent=role)

    return 0
