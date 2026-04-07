"""Tests for orchestrator.pipeline.tmux — tmux session management."""

from __future__ import annotations

import json
from unittest.mock import MagicMock, patch

import pytest

from orchestrator.pipeline.tmux import (
    TmuxSession,
    TmuxSessionInfo,
    list_pipeline_sessions,
    session_status,
    tmux_available,
)


class TestTmuxAvailable:
    """Test tmux availability check."""

    @patch("orchestrator.pipeline.tmux.shutil.which")
    def test_available(self, mock_which):
        mock_which.return_value = "/usr/bin/tmux"
        assert tmux_available() is True

    @patch("orchestrator.pipeline.tmux.shutil.which")
    def test_not_available(self, mock_which):
        mock_which.return_value = None
        assert tmux_available() is False


class TestTmuxSessionInfo:
    """Test TmuxSessionInfo dataclass."""

    def test_status_line_attached(self):
        info = TmuxSessionInfo(
            name="pipeline-42",
            issue_number=42,
            created="Mon Apr  7 10:00:00 2026",
            attached=True,
            windows=1,
            width=200,
            height=50,
        )
        line = info.status_line
        assert "pipeline-42" in line
        assert "42" in line
        assert "attached" in line

    def test_status_line_detached(self):
        info = TmuxSessionInfo(
            name="pipeline-99",
            issue_number=99,
            created="Mon Apr  7 11:00:00 2026",
            attached=False,
            windows=2,
            width=200,
            height=50,
        )
        line = info.status_line
        assert "detached" in line
        assert "2 window(s)" in line


class TestTmuxSession:
    """Test TmuxSession class."""

    def test_session_name(self):
        s = TmuxSession(issue=42)
        assert s.session_name == "pipeline-42"
        assert s.issue == 42

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=False)
    def test_exists_no_tmux(self, _):
        s = TmuxSession(issue=42)
        assert s.exists() is False

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_exists_true(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=0)
        s = TmuxSession(issue=42)
        assert s.exists() is True
        mock_run.assert_called_once()
        cmd = mock_run.call_args[0][0]
        assert cmd == ["tmux", "has-session", "-t", "pipeline-42"]

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_exists_false(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=1)
        s = TmuxSession(issue=42)
        assert s.exists() is False

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=False)
    def test_launch_no_tmux(self, _):
        s = TmuxSession(issue=42)
        assert s.launch("echo hello") is False

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_launch_success(self, mock_run, _):
        # First call: has-session (not exists) → returncode 1
        # Second call: new-session → returncode 0
        mock_run.side_effect = [
            MagicMock(returncode=1),   # has-session → not found
            MagicMock(returncode=0, stderr=""),   # new-session → success
        ]
        s = TmuxSession(issue=42)
        assert s.launch("python -m orchestrator deploy-issue 42") is True

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_launch_already_exists(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=0)
        s = TmuxSession(issue=42)
        assert s.launch("echo hello") is True

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_kill_success(self, mock_run, _):
        # First call: has-session → exists
        # Second call: kill-session → success
        mock_run.side_effect = [
            MagicMock(returncode=0),  # has-session
            MagicMock(returncode=0),  # kill-session
        ]
        s = TmuxSession(issue=42)
        assert s.kill() is True

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_kill_not_exists(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=1)  # has-session → not found
        s = TmuxSession(issue=42)
        assert s.kill() is True  # already dead = success

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_send_keys(self, mock_run, _):
        mock_run.side_effect = [
            MagicMock(returncode=0),  # has-session
            MagicMock(returncode=0),  # send-keys
        ]
        s = TmuxSession(issue=42)
        assert s.send_keys("accept") is True

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_capture_pane(self, mock_run, _):
        mock_run.side_effect = [
            MagicMock(returncode=0),  # has-session
            MagicMock(returncode=0, stdout="CHECKPOINT 3/5\nPR Preview\n"),
        ]
        s = TmuxSession(issue=42)
        output = s.capture_pane(lines=10)
        assert "CHECKPOINT" in output

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_capture_pane_not_exists(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=1)  # has-session → no
        s = TmuxSession(issue=42)
        assert s.capture_pane() == ""


class TestListPipelineSessions:
    """Test list_pipeline_sessions()."""

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=False)
    def test_no_tmux(self, _):
        assert list_pipeline_sessions() == []

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_no_sessions(self, mock_run, _):
        mock_run.return_value = MagicMock(returncode=1, stdout="")
        assert list_pipeline_sessions() == []

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_filters_non_pipeline_sessions(self, mock_run, _):
        mock_run.return_value = MagicMock(
            returncode=0,
            stdout=(
                "my-dev|Mon Apr  7 10:00:00 2026|0|1|200|50\n"
                "pipeline-42|Mon Apr  7 11:00:00 2026|1|1|200|50\n"
                "other-session|Mon Apr  7 12:00:00 2026|0|1|200|50\n"
            ),
        )
        sessions = list_pipeline_sessions()
        assert len(sessions) == 1
        assert sessions[0].issue_number == 42
        assert sessions[0].attached is True

    @patch("orchestrator.pipeline.tmux.tmux_available", return_value=True)
    @patch("orchestrator.pipeline.tmux.subprocess.run")
    def test_multiple_pipeline_sessions_sorted(self, mock_run, _):
        mock_run.return_value = MagicMock(
            returncode=0,
            stdout=(
                "pipeline-99|Mon Apr  7 10:00:00 2026|0|1|200|50\n"
                "pipeline-42|Mon Apr  7 11:00:00 2026|1|1|200|50\n"
                "pipeline-7|Mon Apr  7 12:00:00 2026|0|2|200|50\n"
            ),
        )
        sessions = list_pipeline_sessions()
        assert len(sessions) == 3
        assert sessions[0].issue_number == 7
        assert sessions[1].issue_number == 42
        assert sessions[2].issue_number == 99


class TestSessionStatus:
    """Test session_status()."""

    def test_no_state_file(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "orchestrator.pipeline.tmux.resolve_project_dir",
            lambda: tmp_path,
        )
        assert session_status(42) is None

    def test_wrong_issue(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "orchestrator.pipeline.tmux.resolve_project_dir",
            lambda: tmp_path,
        )
        state_file = tmp_path / ".pipeline-state"
        state_file.write_text(json.dumps({
            "current_state": "CP2_COMMIT",
            "issue_number": 99,
            "issue_title": "Other issue",
        }))
        assert session_status(42) is None

    def test_matching_issue(self, tmp_path, monkeypatch):
        monkeypatch.setattr(
            "orchestrator.pipeline.tmux.resolve_project_dir",
            lambda: tmp_path,
        )
        state_file = tmp_path / ".pipeline-state"
        state_file.write_text(json.dumps({
            "current_state": "CP3_PR_PREVIEW",
            "issue_number": 42,
            "issue_title": "Add radar",
            "agent_role": "feature-perception",
            "branch_name": "feature/issue-42-add-radar",
            "checkpoint_history": ["CP1_REVIEW:accept", "CP2_COMMIT:commit"],
            "fix_iterations": 0,
        }))
        status = session_status(42)
        assert status is not None
        assert "Issue #42" in status
        assert "CP3_PR_PREVIEW" in status
        assert "2/5 completed" in status
