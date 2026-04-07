"""Pipeline checkpoint handlers — CP1 through CP5.

Each checkpoint is an interactive pause where the human reviews progress
and decides the next action. Every checkpoint function:
  - Takes PipelineStateData + IOProtocol (Console or TestConsole)
  - Displays relevant information
  - Prompts for a decision
  - Returns the action string that drives the FSM transition

Optional ntfy.sh push notifications are sent at each checkpoint so the
user can monitor progress from their phone without watching the terminal.

This keeps all user-facing interaction in one place and makes checkpoints
fully testable via TestConsole with scripted inputs.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from orchestrator.pipeline.state import PipelineStateData

if TYPE_CHECKING:
    from orchestrator.console import IOProtocol
    from orchestrator.pipeline.notifications import Notifier
    from orchestrator.tech_lead import TechLead


def cp1_review(
    state: PipelineStateData,
    io: IOProtocol,
    *,
    report_path: Path | None = None,
    diff_stat: str = "",
    tech_lead: TechLead | None = None,
    notifier: Notifier | None = None,
) -> str:
    """Checkpoint 1: Changes Review.

    Shows AGENT_REPORT.md (or diff stat), optional tech-lead recommendation,
    and asks: accept / changes / reject
    """
    io.header("CHECKPOINT 1/5 — Changes Review")

    # Send push notification
    if notifier:
        summary = diff_stat[:200] if diff_stat else "Agent work complete — review needed"
        notifier.send_checkpoint(
            1, summary,
            issue=state.issue_number,
            recommendation="review changes",
        )

    if report_path and report_path.exists():
        io.print(report_path.read_text())
    else:
        io.warn("No AGENT_REPORT.md — showing diff instead")
        if diff_stat:
            io.print(diff_stat)

    io.print("")
    if diff_stat:
        io.print("--- Diff Summary ---")
        io.print(diff_stat)
        io.print("")

    # Tech-lead advisory review (if available)
    if tech_lead:
        assessment = tech_lead.review_checkpoint(
            state.issue_number,
            "CP1",
            report_path=report_path,
            diff_stat=diff_stat,
        )
        io.print("--- Tech-Lead Recommendation ---")
        io.print(f"  Recommendation: {assessment.recommendation}")
        io.print(f"  Reasoning:      {assessment.reasoning}")
        if assessment.specific_concerns:
            io.print(f"  Concerns:       {', '.join(assessment.specific_concerns)}")
        if assessment.change_requests:
            io.print(f"  Changes:        {', '.join(assessment.change_requests)}")
        io.print("")

    io.options([
        ("a", "accept — proceed to validation"),
        ("c", "changes — open interactive session to request modifications"),
        ("r", "reject — abort pipeline (worktree preserved)"),
    ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("a", "accept"):
        return "accept"
    if choice in ("c", "changes"):
        return "changes"
    if choice in ("r", "reject"):
        if io.confirm("  Abort pipeline? (worktree preserved)"):
            return "reject"
        return "changes"  # stay at CP1 if they don't confirm
    io.error("Invalid choice.")
    return "changes"  # default: stay at CP1


def cp2_commit(
    state: PipelineStateData,
    io: IOProtocol,
    *,
    validation_output: str = "",
    validation_passed: bool = True,
    notifier: Notifier | None = None,
) -> str:
    """Checkpoint 2: Commit Approval.

    Shows validation results and asks:
      commit / back / abort
    """
    io.header("CHECKPOINT 2/5 — Commit Approval")

    # Send push notification
    if notifier:
        status = "passed" if validation_passed else "FAILED"
        notifier.send_checkpoint(
            2, f"Validation {status} — commit approval needed",
            issue=state.issue_number,
            recommendation="commit" if validation_passed else "review failures",
        )

    io.print("--- Validation Results ---")
    if validation_output:
        io.print(validation_output)
    io.print("")

    if not validation_passed:
        io.warn("Validation had failures. You may still commit (your judgment).")
    else:
        io.pass_("Validation passed.")
    io.print("")

    io.options([
        ("c", "commit — stage all changes and commit"),
        ("b", "back — return to CP1 (changes review)"),
        ("a", "abort — exit pipeline"),
    ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("c", "commit"):
        return "commit"
    if choice in ("b", "back"):
        return "back"
    if choice in ("a", "abort"):
        return "abort"
    io.error("Invalid choice.")
    return "commit"  # default: proceed


def cp3_pr_preview(
    state: PipelineStateData,
    io: IOProtocol,
    *,
    notifier: Notifier | None = None,
) -> str:
    """Checkpoint 3: PR Preview.

    Shows PR title/body and asks:
      create / edit / update / skip / back / abort
    Existing PR shows update/skip; new PR shows create/edit.
    """
    io.header("CHECKPOINT 3/5 — PR Preview")

    # Send push notification
    if notifier:
        action = "update" if state.pr_number else "create"
        notifier.send_checkpoint(
            3, f"PR ready to {action}: {state.pr_title}",
            issue=state.issue_number,
            recommendation=action,
        )

    io.print(f"Title: {state.pr_title}")
    io.print("")
    io.print("Body:")
    io.print(state.pr_body)
    io.print("")

    if state.pr_number:
        io.info(f"PR #{state.pr_number} already exists.")
        io.options([
            ("u", "update — edit PR title/body"),
            ("s", "skip — proceed to review agents"),
            ("b", "back — return to CP2"),
            ("a", "abort — exit pipeline"),
        ])
    else:
        io.options([
            ("c", "create — create the PR on GitHub"),
            ("e", "edit — modify title before creating"),
            ("b", "back — return to CP2"),
            ("a", "abort — exit pipeline"),
        ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("c", "create") and not state.pr_number:
        return "create"
    if choice in ("u", "update") and state.pr_number:
        return "update"
    if choice in ("s", "skip"):
        return "skip"
    if choice in ("e", "edit"):
        new_title = io.prompt(f"  New title (Enter to keep '{state.pr_title}'): ")
        if new_title:
            state.pr_title = new_title
        return "edit"
    if choice in ("b", "back"):
        return "back"
    if choice in ("a", "abort"):
        return "abort"
    io.error("Invalid choice.")
    return "edit"  # default: re-show


def cp4_findings(
    state: PipelineStateData,
    io: IOProtocol,
    *,
    review_findings: str = "",
    tech_lead: TechLead | None = None,
    notifier: Notifier | None = None,
) -> str:
    """Checkpoint 4: Review & Test Findings.

    Shows review findings, optional tech-lead assessment,
    and asks: accept / fix / back / reject
    """
    io.header("CHECKPOINT 4/5 — Review & Test Findings")

    # Send push notification — deliberately omit review finding content to
    # avoid leaking security-relevant code review comments to the external
    # ntfy server. Send only a generic summary.
    if notifier:
        if not review_findings:
            finding_summary = "No findings"
        else:
            # Count severity levels without leaking content
            p1_count = review_findings.lower().count("[p1]")
            p2_count = review_findings.lower().count("[p2]")
            p3_count = review_findings.lower().count("[p3]")
            finding_summary = (
                f"{p1_count} P1, {p2_count} P2, {p3_count} P3 finding(s)"
            )
        notifier.send_checkpoint(
            4, f"Review complete — {finding_summary}",
            issue=state.issue_number,
            recommendation="review findings",
        )

    if review_findings:
        io.print(review_findings)
    else:
        io.warn(
            f"No review findings found. "
            f"Check PR #{state.pr_number} comments manually."
        )
    io.print("")

    # Tech-lead advisory assessment (if available)
    if tech_lead:
        assessment = tech_lead.review_checkpoint(
            state.issue_number,
            "CP4",
            review_findings=review_findings,
        )
        io.print("--- Tech-Lead Assessment ---")
        io.print(f"  Recommendation: {assessment.recommendation}")
        io.print(f"  Reasoning:      {assessment.reasoning}")
        if assessment.specific_concerns:
            io.print(f"  Concerns:       {', '.join(assessment.specific_concerns)}")
        if assessment.change_requests:
            io.print(f"  Changes:        {', '.join(assessment.change_requests)}")
        io.print("")

    io.options([
        ("a", "accept — no fixes needed, proceed to final summary"),
        ("f", "fix — feed findings to feature agent, then re-run reviews"),
        ("b", "back — return to PR preview (CP3)"),
        ("r", "reject — abort pipeline"),
    ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("a", "accept"):
        return "accept"
    if choice in ("f", "fix"):
        return "fix"
    if choice in ("b", "back"):
        return "back"
    if choice in ("r", "reject"):
        return "reject"
    io.error("Invalid choice.")
    return "accept"  # default: proceed


def cp5_final(
    state: PipelineStateData,
    io: IOProtocol,
    *,
    commit_log: str = "",
    validation_passed: bool = True,
    notifier: Notifier | None = None,
) -> str:
    """Checkpoint 5: Final Summary.

    Shows commit history and asks:
      done / re_review / back / abort
    """
    io.header("CHECKPOINT 5/5 — Final Summary")

    # Send push notification
    if notifier:
        status = "passed" if validation_passed else "has warnings"
        pr_info = f" PR: {state.pr_url}" if state.pr_url else ""
        notifier.send_checkpoint(
            5, f"Final review — validation {status}.{pr_info}",
            issue=state.issue_number,
            recommendation="done" if validation_passed else "review",
        )

    io.print("--- Commit History ---")
    if commit_log:
        io.print(commit_log)
    io.print("")

    if not validation_passed:
        io.warn("Last validation had warnings/failures.")
    else:
        io.pass_("Last validation passed.")

    if state.pr_url:
        io.print(f"  PR: {state.pr_url}")
    io.print("")

    io.options([
        ("d", "done — finalize and clean up"),
        ("r", "re-review — run review + test agents again"),
        ("b", "back — return to review findings (CP4)"),
        ("a", "abort — exit pipeline"),
    ])

    choice = io.prompt("  Your choice: ").lower()
    if choice in ("d", "done"):
        return "done"
    if choice in ("r", "re-review", "re_review"):
        return "re_review"
    if choice in ("b", "back"):
        return "back"
    if choice in ("a", "abort"):
        return "abort"
    io.error("Invalid choice.")
    return "done"  # default: finalize
