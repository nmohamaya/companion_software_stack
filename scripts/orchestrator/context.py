"""Cross-agent context gathering — shared state injected into agent prompts.

Replaces the duplicated context gathering in:
  - start-agent.sh (lines 199-257)
  - deploy-issue.sh (lines 405-461)

Reads 4 files and formats them for prompt injection:
  - tasks/active-work.md — current agent assignments
  - tasks/agent-changelog.md — recently completed work
  - .claude/shared-context/project-status.md — project state
  - .claude/shared-context/domain-knowledge.md — non-obvious pitfalls
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class CrossAgentContext:
    """Gathered context from shared state files."""

    active_work: str = ""
    recent_work: str = ""
    project_status: str = ""
    domain_knowledge: str = ""


def gather_context(project_dir: Path) -> CrossAgentContext:
    """Gather cross-agent context from shared state files.

    Reads each file if it exists, extracts relevant lines.
    Returns a CrossAgentContext with populated fields.
    """
    ctx = CrossAgentContext()

    # Active work by other agents
    active_work_file = project_dir / "tasks" / "active-work.md"
    if active_work_file.exists():
        lines = active_work_file.read_text().splitlines()
        relevant = [
            l for l in lines
            if any(k in l for k in ("## Issue", "Branch:", "Status:"))
        ]
        ctx.active_work = "\n".join(relevant[-30:])

    # Recent completed work
    changelog_file = project_dir / "tasks" / "agent-changelog.md"
    if changelog_file.exists():
        lines = changelog_file.read_text().splitlines()
        relevant = [
            l for l in lines
            if any(k in l for k in ("###", "Issue:", "Status:"))
        ]
        ctx.recent_work = "\n".join(relevant[-15:])

    # Project status
    status_file = project_dir / ".claude" / "shared-context" / "project-status.md"
    if status_file.exists():
        ctx.project_status = status_file.read_text().strip()

    # Domain knowledge
    dk_file = project_dir / ".claude" / "shared-context" / "domain-knowledge.md"
    if dk_file.exists():
        ctx.domain_knowledge = dk_file.read_text().strip()

    return ctx


def format_context(ctx: CrossAgentContext) -> str:
    """Format cross-agent context for injection into an agent prompt.

    Returns a string with labeled sections, or empty string if no context.
    """
    parts: list[str] = []

    if ctx.active_work:
        parts.append(
            "\nCROSS-AGENT CONTEXT — Active work by other agents "
            "(avoid touching these files/branches):\n"
            + ctx.active_work
        )

    if ctx.recent_work:
        parts.append(
            "\nCROSS-AGENT CONTEXT — Recently completed work "
            "(for awareness, avoid duplicate effort):\n"
            + ctx.recent_work
        )

    if ctx.project_status:
        parts.append(
            "\nCROSS-AGENT CONTEXT — Project status "
            "(current state, priorities, blocking bugs):\n"
            + ctx.project_status
        )

    if ctx.domain_knowledge:
        parts.append(
            "\nCROSS-AGENT CONTEXT — Domain knowledge "
            "(non-obvious pitfalls shared by all agents):\n"
            + ctx.domain_knowledge
        )

    return "\n".join(parts)
