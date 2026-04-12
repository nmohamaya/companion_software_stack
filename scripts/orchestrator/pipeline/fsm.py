# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Pipeline FSM — 12-state guided workflow for deploy-issue --pipeline.

Replaces the 550-line while/case state machine in deploy-issue.sh
(lines 722-1275) with a clean enum-based FSM and transition table.

States:
  AGENT_WORK → CP1_REVIEW → VALIDATE → CP2_COMMIT → PR_CREATE →
  CP3_PR_PREVIEW → REVIEW → CP4_FINDINGS → CP5_FINAL → CLEANUP
  (with FIX_AND_REVALIDATE loop and ABORT from any checkpoint)
"""

from __future__ import annotations

from enum import Enum, auto


class PipelineState(Enum):
    """The 12 states of the pipeline workflow."""

    # Phase 1: Agent works autonomously
    AGENT_WORK = auto()

    # Checkpoint 1: User reviews changes (accept / changes / reject)
    CP1_REVIEW = auto()

    # Run validate-session.sh
    VALIDATE = auto()

    # Checkpoint 2: Commit approval (commit / back / abort)
    CP2_COMMIT = auto()

    # Push and prepare PR
    PR_CREATE = auto()

    # Checkpoint 3: PR preview (create / edit / update / skip / back / abort)
    CP3_PR_PREVIEW = auto()

    # Deploy review + test agents
    REVIEW = auto()

    # Checkpoint 4: Review findings (accept / fix / back / reject)
    CP4_FINDINGS = auto()

    # Fix review findings → re-run review agents
    FIX_AND_REVALIDATE = auto()

    # Checkpoint 5: Final summary (done / re-review / back / abort)
    CP5_FINAL = auto()

    # Finalize: cleanup, update shared state
    CLEANUP = auto()

    # Exit with resume instructions
    ABORT = auto()


# ── Transition table ───────────────────────────────────────────────────────
# Maps (current_state, action) → next_state.
# None means terminal (pipeline exits).

TRANSITIONS: dict[tuple[PipelineState, str], PipelineState | None] = {
    # AGENT_WORK
    (PipelineState.AGENT_WORK, "complete"): PipelineState.CP1_REVIEW,
    # CP1_REVIEW
    (PipelineState.CP1_REVIEW, "accept"): PipelineState.VALIDATE,
    (PipelineState.CP1_REVIEW, "changes"): PipelineState.CP1_REVIEW,
    (PipelineState.CP1_REVIEW, "reject"): PipelineState.ABORT,
    # VALIDATE
    (PipelineState.VALIDATE, "complete"): PipelineState.CP2_COMMIT,
    # CP2_COMMIT
    (PipelineState.CP2_COMMIT, "commit"): PipelineState.PR_CREATE,
    (PipelineState.CP2_COMMIT, "back"): PipelineState.CP1_REVIEW,
    (PipelineState.CP2_COMMIT, "abort"): PipelineState.ABORT,
    # PR_CREATE
    (PipelineState.PR_CREATE, "complete"): PipelineState.CP3_PR_PREVIEW,
    (PipelineState.PR_CREATE, "push_failed"): PipelineState.CP2_COMMIT,
    # CP3_PR_PREVIEW
    (PipelineState.CP3_PR_PREVIEW, "create"): PipelineState.REVIEW,
    (PipelineState.CP3_PR_PREVIEW, "update"): PipelineState.REVIEW,
    (PipelineState.CP3_PR_PREVIEW, "skip"): PipelineState.REVIEW,
    (PipelineState.CP3_PR_PREVIEW, "edit"): PipelineState.CP3_PR_PREVIEW,
    (PipelineState.CP3_PR_PREVIEW, "back"): PipelineState.CP2_COMMIT,
    (PipelineState.CP3_PR_PREVIEW, "abort"): PipelineState.ABORT,
    # REVIEW
    (PipelineState.REVIEW, "complete"): PipelineState.CP4_FINDINGS,
    (PipelineState.REVIEW, "no_pr"): PipelineState.CP5_FINAL,
    # CP4_FINDINGS
    (PipelineState.CP4_FINDINGS, "accept"): PipelineState.CP5_FINAL,
    (PipelineState.CP4_FINDINGS, "fix"): PipelineState.FIX_AND_REVALIDATE,
    (PipelineState.CP4_FINDINGS, "back"): PipelineState.CP3_PR_PREVIEW,
    (PipelineState.CP4_FINDINGS, "reject"): PipelineState.ABORT,
    # FIX_AND_REVALIDATE
    (PipelineState.FIX_AND_REVALIDATE, "complete"): PipelineState.REVIEW,
    (PipelineState.FIX_AND_REVALIDATE, "push_failed"): PipelineState.CP5_FINAL,
    # CP5_FINAL
    (PipelineState.CP5_FINAL, "done"): PipelineState.CLEANUP,
    (PipelineState.CP5_FINAL, "re_review"): PipelineState.REVIEW,
    (PipelineState.CP5_FINAL, "back"): PipelineState.CP4_FINDINGS,
    (PipelineState.CP5_FINAL, "abort"): PipelineState.ABORT,
    # CLEANUP (terminal)
    (PipelineState.CLEANUP, "complete"): None,
    # ABORT (terminal)
    (PipelineState.ABORT, "complete"): None,
}


class InvalidTransitionError(Exception):
    """Raised when an action is not valid for the current state."""

    def __init__(self, state: PipelineState, action: str) -> None:
        self.state = state
        self.action = action
        valid = [a for (s, a) in TRANSITIONS if s == state]
        super().__init__(
            f"Invalid action '{action}' for state {state.name}. "
            f"Valid actions: {valid}"
        )


def transition(state: PipelineState, action: str) -> PipelineState | None:
    """Compute the next state given current state and action.

    Returns the next PipelineState, or None if the pipeline should exit.
    Raises InvalidTransitionError if the action is not valid.
    """
    key = (state, action)
    if key not in TRANSITIONS:
        raise InvalidTransitionError(state, action)
    return TRANSITIONS[key]


def valid_actions(state: PipelineState) -> list[str]:
    """Get the list of valid actions for a given state."""
    return [a for (s, a) in TRANSITIONS if s == state]


def is_checkpoint(state: PipelineState) -> bool:
    """Check if a state is an interactive checkpoint."""
    return state.name.startswith("CP")


def is_terminal(state: PipelineState) -> bool:
    """Check if a state is terminal (CLEANUP or ABORT)."""
    return state in (PipelineState.CLEANUP, PipelineState.ABORT)
