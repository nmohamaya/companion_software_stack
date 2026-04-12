"""Tests for orchestrator.pipeline.state — pipeline state persistence."""

from __future__ import annotations

import json

import pytest

from orchestrator.pipeline.fsm import PipelineState
from orchestrator.pipeline.state import PipelineStateData


class TestPipelineStateDataDefaults:
    """Test default values."""

    def test_default_state(self):
        s = PipelineStateData()
        assert s.current_state == "AGENT_WORK"
        assert s.get_state() == PipelineState.AGENT_WORK

    def test_default_issue(self):
        s = PipelineStateData()
        assert s.issue_number == 0
        assert s.issue_title == ""
        assert s.issue_labels == []

    def test_default_pr(self):
        s = PipelineStateData()
        assert s.pr_number == 0
        assert s.pr_url == ""

    def test_default_lists_are_independent(self):
        s1 = PipelineStateData()
        s2 = PipelineStateData()
        s1.issue_labels.append("bug")
        assert s2.issue_labels == []

    def test_default_checkpoint_history_empty(self):
        s = PipelineStateData()
        assert s.checkpoint_history == []


class TestStateAccessors:
    """Test get_state / set_state / record_checkpoint."""

    def test_set_state(self):
        s = PipelineStateData()
        s.set_state(PipelineState.CP1_REVIEW)
        assert s.current_state == "CP1_REVIEW"

    def test_get_state(self):
        s = PipelineStateData(current_state="VALIDATE")
        assert s.get_state() == PipelineState.VALIDATE

    def test_record_checkpoint(self):
        s = PipelineStateData()
        s.set_state(PipelineState.CP1_REVIEW)
        s.record_checkpoint("accept")
        assert s.checkpoint_history == ["CP1_REVIEW:accept"]

    def test_record_multiple_checkpoints(self):
        s = PipelineStateData()
        s.set_state(PipelineState.CP1_REVIEW)
        s.record_checkpoint("accept")
        s.set_state(PipelineState.CP2_COMMIT)
        s.record_checkpoint("commit")
        assert len(s.checkpoint_history) == 2
        assert s.checkpoint_history[1] == "CP2_COMMIT:commit"


class TestJsonPersistence:
    """Test JSON save/load round-trip."""

    def test_save_creates_file(self, tmp_path):
        s = PipelineStateData(issue_number=42, issue_title="Add radar")
        path = tmp_path / ".pipeline-state"
        s.save(path)
        assert path.exists()

    def test_save_is_valid_json(self, tmp_path):
        s = PipelineStateData(issue_number=42)
        path = tmp_path / ".pipeline-state"
        s.save(path)
        data = json.loads(path.read_text())
        assert data["issue_number"] == 42

    def test_round_trip(self, tmp_path):
        original = PipelineStateData(
            current_state="CP3_PR_PREVIEW",
            issue_number=42,
            issue_title="Add radar fusion",
            issue_labels=["perception", "enhancement"],
            agent_role="feature-perception",
            model_tier="opus",
            branch_name="feature/issue-42-add-radar-fusion",
            worktree_path="/tmp/wt-42",
            is_bug=False,
            pr_number=55,
            pr_url="https://github.com/owner/repo/pull/55",
            pr_title="feat(#42): Add radar fusion",
            pipeline_version="2.0.0",
            checkpoint_history=["CP1_REVIEW:accept", "CP2_COMMIT:commit"],
            review_findings=["P2: missing error check in radar.cpp"],
            fix_iterations=1,
            started_at="2026-04-07T10:00:00Z",
            last_checkpoint_at="2026-04-07T10:30:00Z",
        )
        path = tmp_path / ".pipeline-state"
        original.save(path)
        loaded = PipelineStateData.load(path)

        assert loaded.current_state == original.current_state
        assert loaded.issue_number == original.issue_number
        assert loaded.issue_title == original.issue_title
        assert loaded.issue_labels == original.issue_labels
        assert loaded.agent_role == original.agent_role
        assert loaded.branch_name == original.branch_name
        assert loaded.worktree_path == original.worktree_path
        assert loaded.is_bug == original.is_bug
        assert loaded.pr_number == original.pr_number
        assert loaded.pr_url == original.pr_url
        assert loaded.checkpoint_history == original.checkpoint_history
        assert loaded.review_findings == original.review_findings
        assert loaded.fix_iterations == original.fix_iterations

    def test_load_empty_file(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text("")
        s = PipelineStateData.load(path)
        assert s.current_state == "AGENT_WORK"

    def test_load_ignores_unknown_fields(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        data = {"current_state": "VALIDATE", "unknown_field": "value", "issue_number": 10}
        path.write_text(json.dumps(data))
        s = PipelineStateData.load(path)
        assert s.current_state == "VALIDATE"
        assert s.issue_number == 10


class TestLegacyBashFormat:
    """Test loading from legacy bash KEY=VALUE format."""

    def test_basic_bash_format(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text(
            'PIPELINE_STATE=CP2_COMMIT\n'
            'ISSUE_NUMBER=42\n'
            'ISSUE_TITLE="Add radar fusion"\n'
            'BRANCH_NAME=feature/issue-42-add-radar-fusion\n'
            'AGENT_ROLE=feature-perception\n'
        )
        s = PipelineStateData.load(path)
        assert s.current_state == "CP2_COMMIT"
        assert s.issue_number == 42
        assert s.issue_title == "Add radar fusion"
        assert s.branch_name == "feature/issue-42-add-radar-fusion"
        assert s.agent_role == "feature-perception"

    def test_bash_boolean(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text('IS_BUG=true\nISSUE_NUMBER=10\n')
        s = PipelineStateData.load(path)
        assert s.is_bug is True

    def test_bash_boolean_false(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text('IS_BUG=false\n')
        s = PipelineStateData.load(path)
        assert s.is_bug is False

    def test_bash_single_quotes(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text("ISSUE_TITLE='Fix crash on startup'\n")
        s = PipelineStateData.load(path)
        assert s.issue_title == "Fix crash on startup"

    def test_bash_comments_ignored(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text(
            '# Pipeline state file\n'
            'PIPELINE_STATE=VALIDATE\n'
            '# This is a comment\n'
            'ISSUE_NUMBER=5\n'
        )
        s = PipelineStateData.load(path)
        assert s.current_state == "VALIDATE"
        assert s.issue_number == 5

    def test_bash_blank_lines_ignored(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text(
            'PIPELINE_STATE=REVIEW\n'
            '\n'
            'ISSUE_NUMBER=99\n'
            '\n'
        )
        s = PipelineStateData.load(path)
        assert s.current_state == "REVIEW"
        assert s.issue_number == 99

    def test_bash_unknown_keys_ignored(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text(
            'PIPELINE_STATE=CLEANUP\n'
            'SOME_OLD_KEY=old_value\n'
            'ISSUE_NUMBER=1\n'
        )
        s = PipelineStateData.load(path)
        assert s.current_state == "CLEANUP"
        assert s.issue_number == 1

    def test_bash_worktree_dir(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text('WORKTREE_DIR=/home/user/worktrees/wt-42\n')
        s = PipelineStateData.load(path)
        assert s.worktree_path == "/home/user/worktrees/wt-42"

    def test_bash_pr_fields(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        path.write_text(
            'PR_NUMBER=123\n'
            'PR_URL=https://github.com/owner/repo/pull/123\n'
        )
        s = PipelineStateData.load(path)
        assert s.pr_number == 123
        assert s.pr_url == "https://github.com/owner/repo/pull/123"


class TestMigrationPath:
    """Test that legacy bash state can be loaded and re-saved as JSON."""

    def test_bash_to_json_migration(self, tmp_path):
        path = tmp_path / ".pipeline-state"
        # Write legacy format
        path.write_text(
            'PIPELINE_STATE=CP4_FINDINGS\n'
            'ISSUE_NUMBER=42\n'
            'ISSUE_TITLE="Add radar fusion"\n'
            'BRANCH_NAME=feature/issue-42-add-radar-fusion\n'
            'AGENT_ROLE=feature-perception\n'
            'PR_NUMBER=55\n'
            'IS_BUG=false\n'
        )
        # Load legacy
        state = PipelineStateData.load(path)
        assert state.current_state == "CP4_FINDINGS"

        # Re-save as JSON
        state.save(path)
        text = path.read_text().strip()
        assert text.startswith("{")

        # Reload as JSON
        reloaded = PipelineStateData.load(path)
        assert reloaded.current_state == "CP4_FINDINGS"
        assert reloaded.issue_number == 42
        assert reloaded.agent_role == "feature-perception"
        assert reloaded.pr_number == 55
