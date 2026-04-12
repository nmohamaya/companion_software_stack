"""Tech-lead strategy layer — routing, quality assessment, recommendations.

The tech-lead agent acts as a strategy orchestrator:
  - Reads issues/epics and decides routing, priority, sequencing
  - Reviews AGENT_REPORT.md and recommends accept/changes at checkpoints
  - Coordinates cross-agent work (dependency ordering, conflict detection)
  - Delegates mechanical GitHub tasks to ops-github (Haiku)

The tech-lead never has final say — the human approves at every checkpoint.
Tech-lead provides recommendations with reasoning that the human can
accept, modify, or override.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from orchestrator.config import get_role, ROLES
from orchestrator.claude import Claude
from orchestrator.console import Console
from orchestrator.context import format_context, gather_context
from orchestrator.github import GitHub
from orchestrator.routing import triage_issue_labels


@dataclass
class RoutingDecision:
    """Tech-lead's routing recommendation for an issue."""

    issue_number: int
    issue_title: str
    recommended_role: str
    recommended_priority: str  # "high", "medium", "low"
    reasoning: str
    sequencing_notes: str = ""
    dependencies: list[int] = field(default_factory=list)
    labels_to_add: list[str] = field(default_factory=list)
    labels_to_remove: list[str] = field(default_factory=list)


@dataclass
class ReviewAssessment:
    """Tech-lead's assessment of agent work at a checkpoint."""

    recommendation: str  # "accept", "changes", "reject"
    reasoning: str
    specific_concerns: list[str] = field(default_factory=list)
    change_requests: list[str] = field(default_factory=list)


@dataclass
class EpicPlan:
    """Tech-lead's plan for an epic (multi-issue work)."""

    epic_number: int
    epic_title: str
    phases: list[EpicPhase] = field(default_factory=list)
    total_issues: int = 0
    estimated_waves: int = 0


@dataclass
class EpicPhase:
    """A phase within an epic plan."""

    name: str
    issues: list[int] = field(default_factory=list)
    description: str = ""
    depends_on: list[str] = field(default_factory=list)


class TechLead:
    """Strategy orchestrator that provides recommendations via Claude.

    Uses the tech-lead agent (Opus) to analyze issues, review work,
    and coordinate multi-agent sessions. All recommendations are
    advisory — the human has final approval at every checkpoint.
    """

    def __init__(
        self,
        claude: Claude | None = None,
        github: GitHub | None = None,
        io: Console | None = None,
    ) -> None:
        self._claude = claude or Claude()
        self._github = github or GitHub()
        self._io = io or Console()
        self._role_config = get_role("tech-lead")

    def analyze_issue(
        self,
        issue_number: int,
        *,
        project_dir: Path | None = None,
    ) -> RoutingDecision:
        """Analyze an issue and recommend routing/priority.

        The tech-lead reads the issue, considers current project state
        (active work, recent changes, domain knowledge), and produces
        a routing recommendation.
        """
        # Fetch issue
        issue = self._github.fetch_issue(issue_number)

        # Automated triage first (fast, deterministic)
        auto_triage = triage_issue_labels(issue.labels)

        # Gather project context
        ctx_text = ""
        if project_dir:
            ctx = gather_context(project_dir)
            ctx_text = format_context(ctx)

        # Build analysis prompt
        prompt = (
            f"You are the tech-lead orchestrator. Analyze this issue and "
            f"provide a routing recommendation.\n\n"
            f"## Issue #{issue_number}: {issue.title}\n\n"
            f"{issue.body}\n\n"
            f"**Labels:** {', '.join(issue.labels)}\n\n"
            f"**Automated triage suggests:** role={auto_triage.role}, "
            f"is_bug={auto_triage.is_bug}, is_refactor={auto_triage.is_refactor}\n\n"
            f"{ctx_text}\n\n"
            f"Respond with EXACTLY this format (no markdown fences):\n"
            f"ROLE: <agent-role>\n"
            f"PRIORITY: <high|medium|low>\n"
            f"REASONING: <one paragraph>\n"
            f"SEQUENCING: <any ordering notes, or 'none'>\n"
            f"DEPENDENCIES: <comma-separated issue numbers, or 'none'>\n"
            f"LABELS_ADD: <comma-separated labels to add, or 'none'>\n"
            f"LABELS_REMOVE: <comma-separated labels to remove, or 'none'>\n"
        )

        self._io.print("  Tech-lead analyzing issue...")
        output = self._claude.launch_print(
            model=self._role_config.model,
            agent="tech-lead",
            prompt=prompt,
            capture=True,
        )

        # Parse response
        decision = RoutingDecision(
            issue_number=issue_number,
            issue_title=issue.title,
            recommended_role=auto_triage.role or "tech-lead",
            recommended_priority="medium",
            reasoning="",
        )

        if output:
            decision = _parse_routing_decision(output, decision)

        return decision

    def review_checkpoint(
        self,
        issue_number: int,
        checkpoint: str,
        *,
        report_path: Path | None = None,
        diff_stat: str = "",
        review_findings: str = "",
    ) -> ReviewAssessment:
        """Review agent work at a checkpoint and recommend action.

        The tech-lead reads the AGENT_REPORT.md, diff, and any review
        findings to produce a recommendation for the human.
        """
        report_content = ""
        if report_path and report_path.exists():
            report_content = report_path.read_text()

        prompt = (
            f"You are the tech-lead reviewing work at {checkpoint} "
            f"for issue #{issue_number}.\n\n"
        )

        if report_content:
            prompt += f"## Agent Report\n\n{report_content}\n\n"
        if diff_stat:
            prompt += f"## Diff Summary\n\n{diff_stat}\n\n"
        if review_findings:
            prompt += f"## Review Findings\n\n{review_findings}\n\n"

        prompt += (
            f"Based on this information, recommend one action:\n"
            f"- accept: work looks good, proceed\n"
            f"- changes: specific modifications needed\n"
            f"- reject: fundamental issues, abort\n\n"
            f"Respond with EXACTLY this format:\n"
            f"RECOMMENDATION: <accept|changes|reject>\n"
            f"REASONING: <one paragraph>\n"
            f"CONCERNS: <comma-separated specific concerns, or 'none'>\n"
            f"CHANGE_REQUESTS: <numbered list of changes, or 'none'>\n"
        )

        self._io.print(f"  Tech-lead reviewing {checkpoint}...")
        output = self._claude.launch_print(
            model=self._role_config.model,
            agent="tech-lead",
            prompt=prompt,
            capture=True,
        )

        assessment = ReviewAssessment(
            recommendation="accept",
            reasoning="No tech-lead output available.",
        )

        if isinstance(output, str) and output.strip():
            assessment = _parse_review_assessment(output, assessment)
            # If parsing failed to extract reasoning, show what we got
            if assessment.reasoning == "No tech-lead output available.":
                assessment.reasoning = (
                    f"Tech-lead responded but output could not be parsed. "
                    f"First 200 chars: {output[:200]}"
                )
        else:
            self._io.print(
                f"  WARN: Tech-lead returned no output for {checkpoint}. "
                f"Check stderr for diagnostics."
            )

        return assessment

    def plan_epic(
        self,
        epic_number: int,
        *,
        project_dir: Path | None = None,
    ) -> EpicPlan:
        """Analyze an epic and produce a phased implementation plan.

        Used by `orchestrate` command for multi-issue work.
        """
        issue = self._github.fetch_issue(epic_number)

        ctx_text = ""
        if project_dir:
            ctx = gather_context(project_dir)
            ctx_text = format_context(ctx)

        prompt = (
            f"You are the tech-lead orchestrator. Analyze this epic "
            f"and produce a phased implementation plan.\n\n"
            f"## Epic #{epic_number}: {issue.title}\n\n"
            f"{issue.body}\n\n"
            f"{ctx_text}\n\n"
            f"Identify all sub-issues, group them into phases with "
            f"dependencies, and suggest a wave-based execution order.\n\n"
            f"Respond with EXACTLY this format:\n"
            f"TOTAL_ISSUES: <number>\n"
            f"ESTIMATED_WAVES: <number>\n"
            f"PHASE: <name>\n"
            f"  ISSUES: <comma-separated issue numbers>\n"
            f"  DESCRIPTION: <one line>\n"
            f"  DEPENDS_ON: <phase names, or 'none'>\n"
            f"(repeat PHASE blocks as needed)\n"
        )

        self._io.print("  Tech-lead planning epic...")
        output = self._claude.launch_print(
            model=self._role_config.model,
            agent="tech-lead",
            prompt=prompt,
            capture=True,
        )

        plan = EpicPlan(
            epic_number=epic_number,
            epic_title=issue.title,
        )

        if output:
            plan = _parse_epic_plan(output, plan)

        return plan

    def delegate_to_ops(
        self,
        task: str,
    ) -> str:
        """Delegate a mechanical GitHub task to ops-github (Haiku).

        Used for: label application, milestone tracking, status comments.
        Returns the ops-github output.
        """
        ops_config = get_role("ops-github")

        self._io.print("  Delegating to ops-github...")
        output = self._claude.launch_print(
            model=ops_config.model,
            agent="ops-github",
            prompt=task,
            capture=True,
        )
        return output or ""


# ── Response parsers ───────────────────────────────────────────────────────


def _parse_routing_decision(
    output: str, default: RoutingDecision
) -> RoutingDecision:
    """Parse tech-lead routing response into RoutingDecision."""
    lines = output.strip().splitlines()
    result = default

    for line in lines:
        line = line.strip()
        if line.startswith("ROLE:"):
            role = line.split(":", 1)[1].strip()
            if role in ROLES:
                result.recommended_role = role
        elif line.startswith("PRIORITY:"):
            p = line.split(":", 1)[1].strip().lower()
            if p in ("high", "medium", "low"):
                result.recommended_priority = p
        elif line.startswith("REASONING:"):
            result.reasoning = line.split(":", 1)[1].strip()
        elif line.startswith("SEQUENCING:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                result.sequencing_notes = val
        elif line.startswith("DEPENDENCIES:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                try:
                    result.dependencies = [
                        int(x.strip().lstrip("#"))
                        for x in val.split(",")
                        if x.strip()
                    ]
                except ValueError:
                    pass
        elif line.startswith("LABELS_ADD:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                result.labels_to_add = [
                    x.strip() for x in val.split(",") if x.strip()
                ]
        elif line.startswith("LABELS_REMOVE:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                result.labels_to_remove = [
                    x.strip() for x in val.split(",") if x.strip()
                ]

    return result


def _parse_review_assessment(
    output: str, default: ReviewAssessment
) -> ReviewAssessment:
    """Parse tech-lead review response into ReviewAssessment."""
    lines = output.strip().splitlines()
    result = default

    for line in lines:
        line = line.strip()
        if line.startswith("RECOMMENDATION:"):
            rec = line.split(":", 1)[1].strip().lower()
            if rec in ("accept", "changes", "reject"):
                result.recommendation = rec
        elif line.startswith("REASONING:"):
            result.reasoning = line.split(":", 1)[1].strip()
        elif line.startswith("CONCERNS:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                result.specific_concerns = [
                    x.strip() for x in val.split(",") if x.strip()
                ]
        elif line.startswith("CHANGE_REQUESTS:"):
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                result.change_requests = [
                    x.strip() for x in val.split(",") if x.strip()
                ]

    return result


def _parse_epic_plan(output: str, default: EpicPlan) -> EpicPlan:
    """Parse tech-lead epic plan response into EpicPlan."""
    lines = output.strip().splitlines()
    result = default
    current_phase: EpicPhase | None = None

    for line in lines:
        line = line.strip()
        if line.startswith("TOTAL_ISSUES:"):
            try:
                result.total_issues = int(line.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif line.startswith("ESTIMATED_WAVES:"):
            try:
                result.estimated_waves = int(line.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif line.startswith("PHASE:"):
            if current_phase:
                result.phases.append(current_phase)
            current_phase = EpicPhase(name=line.split(":", 1)[1].strip())
        elif line.startswith("ISSUES:") and current_phase:
            val = line.split(":", 1)[1].strip()
            try:
                current_phase.issues = [
                    int(x.strip().lstrip("#"))
                    for x in val.split(",")
                    if x.strip()
                ]
            except ValueError:
                pass
        elif line.startswith("DESCRIPTION:") and current_phase:
            current_phase.description = line.split(":", 1)[1].strip()
        elif line.startswith("DEPENDS_ON:") and current_phase:
            val = line.split(":", 1)[1].strip()
            if val.lower() != "none":
                current_phase.depends_on = [
                    x.strip() for x in val.split(",") if x.strip()
                ]

    if current_phase:
        result.phases.append(current_phase)

    return result
