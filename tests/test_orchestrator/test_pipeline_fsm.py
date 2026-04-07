"""Tests for orchestrator.pipeline.fsm — pipeline state machine."""

from __future__ import annotations

import pytest

from orchestrator.pipeline.fsm import (
    InvalidTransitionError,
    PipelineState,
    TRANSITIONS,
    is_checkpoint,
    is_terminal,
    transition,
    valid_actions,
)


class TestPipelineStateEnum:
    """Test PipelineState enum basics."""

    def test_has_12_states(self):
        assert len(PipelineState) == 12

    def test_starts_with_agent_work(self):
        states = list(PipelineState)
        assert states[0] == PipelineState.AGENT_WORK

    def test_ends_with_abort(self):
        states = list(PipelineState)
        assert states[-1] == PipelineState.ABORT

    def test_all_states_named(self):
        expected = {
            "AGENT_WORK", "CP1_REVIEW", "VALIDATE", "CP2_COMMIT",
            "PR_CREATE", "CP3_PR_PREVIEW", "REVIEW", "CP4_FINDINGS",
            "FIX_AND_REVALIDATE", "CP5_FINAL", "CLEANUP", "ABORT",
        }
        assert {s.name for s in PipelineState} == expected


class TestTransitionTable:
    """Test the TRANSITIONS dict coverage and correctness."""

    def test_transition_count(self):
        assert len(TRANSITIONS) == 30

    def test_all_non_terminal_states_have_transitions(self):
        non_terminal = {s for s in PipelineState if not is_terminal(s)}
        states_with_transitions = {s for (s, _) in TRANSITIONS}
        # Terminal states also have "complete" transitions (to None)
        assert non_terminal.issubset(states_with_transitions)

    def test_terminal_states_transition_to_none(self):
        assert TRANSITIONS[(PipelineState.CLEANUP, "complete")] is None
        assert TRANSITIONS[(PipelineState.ABORT, "complete")] is None


class TestTransitionFunction:
    """Test the transition() function."""

    def test_agent_work_complete(self):
        assert transition(PipelineState.AGENT_WORK, "complete") == PipelineState.CP1_REVIEW

    def test_cp1_accept(self):
        assert transition(PipelineState.CP1_REVIEW, "accept") == PipelineState.VALIDATE

    def test_cp1_changes_loops(self):
        assert transition(PipelineState.CP1_REVIEW, "changes") == PipelineState.CP1_REVIEW

    def test_cp1_reject_aborts(self):
        assert transition(PipelineState.CP1_REVIEW, "reject") == PipelineState.ABORT

    def test_validate_complete(self):
        assert transition(PipelineState.VALIDATE, "complete") == PipelineState.CP2_COMMIT

    def test_cp2_commit(self):
        assert transition(PipelineState.CP2_COMMIT, "commit") == PipelineState.PR_CREATE

    def test_cp2_back(self):
        assert transition(PipelineState.CP2_COMMIT, "back") == PipelineState.CP1_REVIEW

    def test_cp2_abort(self):
        assert transition(PipelineState.CP2_COMMIT, "abort") == PipelineState.ABORT

    def test_pr_create_complete(self):
        assert transition(PipelineState.PR_CREATE, "complete") == PipelineState.CP3_PR_PREVIEW

    def test_pr_create_push_failed(self):
        assert transition(PipelineState.PR_CREATE, "push_failed") == PipelineState.CP2_COMMIT

    def test_cp3_create(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "create") == PipelineState.REVIEW

    def test_cp3_update(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "update") == PipelineState.REVIEW

    def test_cp3_skip(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "skip") == PipelineState.REVIEW

    def test_cp3_edit_loops(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "edit") == PipelineState.CP3_PR_PREVIEW

    def test_cp3_back(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "back") == PipelineState.CP2_COMMIT

    def test_cp3_abort(self):
        assert transition(PipelineState.CP3_PR_PREVIEW, "abort") == PipelineState.ABORT

    def test_review_complete(self):
        assert transition(PipelineState.REVIEW, "complete") == PipelineState.CP4_FINDINGS

    def test_review_no_pr(self):
        assert transition(PipelineState.REVIEW, "no_pr") == PipelineState.CP5_FINAL

    def test_cp4_accept(self):
        assert transition(PipelineState.CP4_FINDINGS, "accept") == PipelineState.CP5_FINAL

    def test_cp4_fix(self):
        assert transition(PipelineState.CP4_FINDINGS, "fix") == PipelineState.FIX_AND_REVALIDATE

    def test_cp4_back(self):
        assert transition(PipelineState.CP4_FINDINGS, "back") == PipelineState.CP3_PR_PREVIEW

    def test_cp4_reject(self):
        assert transition(PipelineState.CP4_FINDINGS, "reject") == PipelineState.ABORT

    def test_fix_complete(self):
        assert transition(PipelineState.FIX_AND_REVALIDATE, "complete") == PipelineState.REVIEW

    def test_fix_push_failed(self):
        assert transition(PipelineState.FIX_AND_REVALIDATE, "push_failed") == PipelineState.CP5_FINAL

    def test_cp5_done(self):
        assert transition(PipelineState.CP5_FINAL, "done") == PipelineState.CLEANUP

    def test_cp5_re_review(self):
        assert transition(PipelineState.CP5_FINAL, "re_review") == PipelineState.REVIEW

    def test_cp5_back(self):
        assert transition(PipelineState.CP5_FINAL, "back") == PipelineState.CP4_FINDINGS

    def test_cp5_abort(self):
        assert transition(PipelineState.CP5_FINAL, "abort") == PipelineState.ABORT

    def test_cleanup_terminal(self):
        assert transition(PipelineState.CLEANUP, "complete") is None

    def test_abort_terminal(self):
        assert transition(PipelineState.ABORT, "complete") is None

    def test_invalid_action_raises(self):
        with pytest.raises(InvalidTransitionError) as exc_info:
            transition(PipelineState.AGENT_WORK, "invalid")
        assert "invalid" in str(exc_info.value)
        assert "AGENT_WORK" in str(exc_info.value)

    def test_invalid_transition_lists_valid_actions(self):
        with pytest.raises(InvalidTransitionError) as exc_info:
            transition(PipelineState.CP1_REVIEW, "commit")
        assert "accept" in str(exc_info.value)
        assert "changes" in str(exc_info.value)
        assert "reject" in str(exc_info.value)


class TestValidActions:
    """Test valid_actions() helper."""

    def test_agent_work_actions(self):
        assert valid_actions(PipelineState.AGENT_WORK) == ["complete"]

    def test_cp1_actions(self):
        actions = valid_actions(PipelineState.CP1_REVIEW)
        assert set(actions) == {"accept", "changes", "reject"}

    def test_cp3_has_six_actions(self):
        actions = valid_actions(PipelineState.CP3_PR_PREVIEW)
        assert len(actions) == 6
        assert set(actions) == {"create", "update", "skip", "edit", "back", "abort"}

    def test_cleanup_actions(self):
        assert valid_actions(PipelineState.CLEANUP) == ["complete"]


class TestIsCheckpoint:
    """Test is_checkpoint() helper."""

    def test_cp_states_are_checkpoints(self):
        assert is_checkpoint(PipelineState.CP1_REVIEW)
        assert is_checkpoint(PipelineState.CP2_COMMIT)
        assert is_checkpoint(PipelineState.CP3_PR_PREVIEW)
        assert is_checkpoint(PipelineState.CP4_FINDINGS)
        assert is_checkpoint(PipelineState.CP5_FINAL)

    def test_non_cp_states_are_not_checkpoints(self):
        assert not is_checkpoint(PipelineState.AGENT_WORK)
        assert not is_checkpoint(PipelineState.VALIDATE)
        assert not is_checkpoint(PipelineState.PR_CREATE)
        assert not is_checkpoint(PipelineState.REVIEW)
        assert not is_checkpoint(PipelineState.FIX_AND_REVALIDATE)
        assert not is_checkpoint(PipelineState.CLEANUP)
        assert not is_checkpoint(PipelineState.ABORT)


class TestIsTerminal:
    """Test is_terminal() helper."""

    def test_cleanup_is_terminal(self):
        assert is_terminal(PipelineState.CLEANUP)

    def test_abort_is_terminal(self):
        assert is_terminal(PipelineState.ABORT)

    def test_non_terminal_states(self):
        non_terminal = [
            PipelineState.AGENT_WORK, PipelineState.CP1_REVIEW,
            PipelineState.VALIDATE, PipelineState.CP2_COMMIT,
            PipelineState.PR_CREATE, PipelineState.CP3_PR_PREVIEW,
            PipelineState.REVIEW, PipelineState.CP4_FINDINGS,
            PipelineState.FIX_AND_REVALIDATE, PipelineState.CP5_FINAL,
        ]
        for state in non_terminal:
            assert not is_terminal(state)


class TestHappyPath:
    """Walk the full happy path through the FSM."""

    def test_full_happy_path(self):
        """AGENT_WORK → CP1 → VALIDATE → CP2 → PR_CREATE → CP3 →
        REVIEW → CP4 → CP5 → CLEANUP → None"""
        state = PipelineState.AGENT_WORK
        steps = [
            "complete",  # → CP1_REVIEW
            "accept",    # → VALIDATE
            "complete",  # → CP2_COMMIT
            "commit",    # → PR_CREATE
            "complete",  # → CP3_PR_PREVIEW
            "create",    # → REVIEW
            "complete",  # → CP4_FINDINGS
            "accept",    # → CP5_FINAL
            "done",      # → CLEANUP
            "complete",  # → None (exit)
        ]
        for action in steps:
            state = transition(state, action)
        assert state is None

    def test_fix_loop_path(self):
        """CP4 → FIX_AND_REVALIDATE → REVIEW → CP4 → CP5"""
        state = PipelineState.CP4_FINDINGS
        state = transition(state, "fix")
        assert state == PipelineState.FIX_AND_REVALIDATE
        state = transition(state, "complete")
        assert state == PipelineState.REVIEW
        state = transition(state, "complete")
        assert state == PipelineState.CP4_FINDINGS
        state = transition(state, "accept")
        assert state == PipelineState.CP5_FINAL

    def test_back_navigation_cp5_to_cp4(self):
        state = transition(PipelineState.CP5_FINAL, "back")
        assert state == PipelineState.CP4_FINDINGS

    def test_abort_from_any_checkpoint(self):
        """Every checkpoint should have an abort/reject path."""
        abort_actions = {
            PipelineState.CP1_REVIEW: "reject",
            PipelineState.CP2_COMMIT: "abort",
            PipelineState.CP3_PR_PREVIEW: "abort",
            PipelineState.CP4_FINDINGS: "reject",
            PipelineState.CP5_FINAL: "abort",
        }
        for state, action in abort_actions.items():
            result = transition(state, action)
            assert result == PipelineState.ABORT, f"{state.name} + {action} should abort"
