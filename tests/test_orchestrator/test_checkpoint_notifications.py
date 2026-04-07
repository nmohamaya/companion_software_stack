"""Tests for checkpoint notification integration.

Verifies that checkpoints send ntfy.sh notifications when a Notifier
is provided, and gracefully skip when no notifier is given.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from orchestrator.console import TestConsole
from orchestrator.pipeline.checkpoints import (
    cp1_review,
    cp2_commit,
    cp3_pr_preview,
    cp4_findings,
    cp5_final,
)
from orchestrator.pipeline.notifications import Notifier, NotifyConfig
from orchestrator.pipeline.state import PipelineStateData


@pytest.fixture
def state():
    return PipelineStateData(
        issue_number=42,
        issue_title="Add radar fusion",
        agent_role="feature-perception",
        pr_title="feat(#42): Add radar fusion",
        pr_url="https://github.com/owner/repo/pull/55",
        pr_number=55,
    )


@pytest.fixture
def mock_notifier():
    """A notifier with all send methods mocked."""
    n = Notifier(topic="test-topic")
    n.send_checkpoint = MagicMock(return_value=True)
    n.send_error = MagicMock(return_value=True)
    n.send_complete = MagicMock(return_value=True)
    return n


class TestCp1WithNotifier:
    def test_sends_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["accept"])
        cp1_review(state, io, diff_stat="5 files changed", notifier=mock_notifier)
        mock_notifier.send_checkpoint.assert_called_once()
        call_args = mock_notifier.send_checkpoint.call_args
        assert call_args[0][0] == 1  # checkpoint number
        assert call_args[1]["issue"] == 42

    def test_works_without_notifier(self, state):
        io = TestConsole(inputs=["accept"])
        result = cp1_review(state, io, diff_stat="5 files changed")
        assert result == "accept"


class TestCp2WithNotifier:
    def test_sends_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["commit"])
        cp2_commit(state, io, validation_passed=True, notifier=mock_notifier)
        mock_notifier.send_checkpoint.assert_called_once()
        call_args = mock_notifier.send_checkpoint.call_args
        assert call_args[0][0] == 2
        assert "passed" in call_args[0][1]

    def test_sends_failure_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["commit"])
        cp2_commit(state, io, validation_passed=False, notifier=mock_notifier)
        call_args = mock_notifier.send_checkpoint.call_args
        assert "FAILED" in call_args[0][1]

    def test_works_without_notifier(self, state):
        io = TestConsole(inputs=["commit"])
        result = cp2_commit(state, io, validation_passed=True)
        assert result == "commit"


class TestCp3WithNotifier:
    def test_sends_notification_new_pr(self, state, mock_notifier):
        state.pr_number = 0  # no existing PR
        io = TestConsole(inputs=["create"])
        cp3_pr_preview(state, io, notifier=mock_notifier)
        mock_notifier.send_checkpoint.assert_called_once()
        call_args = mock_notifier.send_checkpoint.call_args
        assert call_args[0][0] == 3
        assert "create" in call_args[0][1]

    def test_sends_notification_existing_pr(self, state, mock_notifier):
        io = TestConsole(inputs=["update"])
        cp3_pr_preview(state, io, notifier=mock_notifier)
        call_args = mock_notifier.send_checkpoint.call_args
        assert "update" in call_args[0][1]


class TestCp4WithNotifier:
    def test_sends_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["accept"])
        cp4_findings(
            state, io,
            review_findings="P2: missing error check",
            notifier=mock_notifier,
        )
        mock_notifier.send_checkpoint.assert_called_once()
        call_args = mock_notifier.send_checkpoint.call_args
        assert call_args[0][0] == 4
        assert "P2:" in call_args[0][1]

    def test_no_findings_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["accept"])
        cp4_findings(state, io, review_findings="", notifier=mock_notifier)
        call_args = mock_notifier.send_checkpoint.call_args
        assert "No findings" in call_args[0][1]


class TestCp5WithNotifier:
    def test_sends_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["done"])
        cp5_final(
            state, io,
            commit_log="abc123 feat: add radar",
            validation_passed=True,
            notifier=mock_notifier,
        )
        mock_notifier.send_checkpoint.assert_called_once()
        call_args = mock_notifier.send_checkpoint.call_args
        assert call_args[0][0] == 5
        assert "passed" in call_args[0][1]

    def test_failed_validation_notification(self, state, mock_notifier):
        io = TestConsole(inputs=["done"])
        cp5_final(state, io, validation_passed=False, notifier=mock_notifier)
        call_args = mock_notifier.send_checkpoint.call_args
        assert "warnings" in call_args[0][1]
