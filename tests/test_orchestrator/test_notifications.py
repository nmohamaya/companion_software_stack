"""Tests for orchestrator.pipeline.notifications — ntfy.sh push notifications."""

from __future__ import annotations

import subprocess
from unittest.mock import MagicMock, patch

import pytest

from orchestrator.pipeline.notifications import (
    NotifyConfig,
    NotifyEvent,
    Notifier,
)


class TestNotifyConfig:
    """Test NotifyConfig construction and defaults."""

    def test_defaults(self):
        cfg = NotifyConfig()
        assert cfg.enabled is False
        assert cfg.provider == "ntfy"
        assert cfg.topic == ""
        assert cfg.server_url == "https://ntfy.sh"
        assert cfg.notify_on == ["checkpoint", "error", "complete"]

    def test_from_env_with_topic(self, monkeypatch):
        monkeypatch.setenv("NTFY_TOPIC", "my-pipeline")
        cfg = NotifyConfig.from_env()
        assert cfg.enabled is True
        assert cfg.topic == "my-pipeline"
        assert cfg.server_url == "https://ntfy.sh"

    def test_from_env_without_topic(self, monkeypatch):
        monkeypatch.delenv("NTFY_TOPIC", raising=False)
        cfg = NotifyConfig.from_env()
        assert cfg.enabled is False
        assert cfg.topic == ""

    def test_from_env_custom_server(self, monkeypatch):
        monkeypatch.setenv("NTFY_TOPIC", "test")
        monkeypatch.setenv("NTFY_SERVER", "https://ntfy.example.com")
        cfg = NotifyConfig.from_env()
        assert cfg.server_url == "https://ntfy.example.com"

    def test_from_env_custom_events(self, monkeypatch):
        monkeypatch.setenv("NTFY_TOPIC", "test")
        monkeypatch.setenv("NTFY_EVENTS", "error,complete")
        cfg = NotifyConfig.from_env()
        assert cfg.notify_on == ["error", "complete"]

    def test_from_env_empty_events_string(self, monkeypatch):
        monkeypatch.setenv("NTFY_TOPIC", "test")
        monkeypatch.setenv("NTFY_EVENTS", "")
        cfg = NotifyConfig.from_env()
        assert cfg.notify_on == []


class TestNotifyEvent:
    """Test NotifyEvent enum."""

    def test_checkpoint(self):
        assert NotifyEvent.CHECKPOINT.name == "CHECKPOINT"

    def test_error(self):
        assert NotifyEvent.ERROR.name == "ERROR"

    def test_complete(self):
        assert NotifyEvent.COMPLETE.name == "COMPLETE"


class TestNotifier:
    """Test Notifier class."""

    def test_disabled_by_default(self):
        # No topic, no env var → disabled
        with patch.dict("os.environ", {}, clear=True):
            n = Notifier()
            assert not n.enabled

    def test_enabled_with_topic(self):
        n = Notifier(topic="test-topic")
        assert n.enabled
        assert n.topic == "test-topic"

    def test_enabled_with_config(self):
        cfg = NotifyConfig(enabled=True, topic="cfg-topic")
        n = Notifier(config=cfg)
        assert n.enabled
        assert n.topic == "cfg-topic"

    def test_config_takes_precedence_over_topic(self):
        cfg = NotifyConfig(enabled=True, topic="cfg-topic")
        n = Notifier(topic="kwarg-topic", config=cfg)
        assert n.topic == "cfg-topic"

    def test_should_notify_when_disabled(self):
        n = Notifier(config=NotifyConfig(enabled=False, topic="t"))
        assert not n.should_notify(NotifyEvent.CHECKPOINT)

    def test_should_notify_checkpoint(self):
        n = Notifier(topic="test")
        assert n.should_notify(NotifyEvent.CHECKPOINT)

    def test_should_not_notify_excluded_event(self):
        cfg = NotifyConfig(enabled=True, topic="test", notify_on=["error"])
        n = Notifier(config=cfg)
        assert not n.should_notify(NotifyEvent.CHECKPOINT)
        assert n.should_notify(NotifyEvent.ERROR)


class TestNotifierSend:
    """Test actual send methods (mocked subprocess)."""

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_checkpoint_success(self, mock_run):
        mock_run.return_value = MagicMock(
            stdout="200", stderr="", returncode=0
        )
        n = Notifier(topic="test-topic")
        result = n.send_checkpoint(1, "12 files changed", issue=367)

        assert result is True
        mock_run.assert_called_once()
        call_args = mock_run.call_args
        cmd = call_args[0][0]
        assert "curl" in cmd[0]
        assert "https://ntfy.sh/test-topic" in cmd

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_checkpoint_includes_issue(self, mock_run):
        mock_run.return_value = MagicMock(stdout="200", stderr="")
        n = Notifier(topic="test")
        n.send_checkpoint(3, "PR ready", issue=42, recommendation="create")

        cmd = mock_run.call_args[0][0]
        # Find the -d argument (message body)
        d_idx = cmd.index("-d")
        message = cmd[d_idx + 1]
        assert "Issue #42" in message
        assert "Agent recommends: create" in message
        assert "tmux attach -t pipeline-42" in message

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_error_high_priority(self, mock_run):
        mock_run.return_value = MagicMock(stdout="200", stderr="")
        n = Notifier(topic="test")
        n.send_error("Build failed", issue=99, state="VALIDATE")

        cmd = mock_run.call_args[0][0]
        assert any("Priority: high" in h for h in cmd)

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_complete(self, mock_run):
        mock_run.return_value = MagicMock(stdout="200", stderr="")
        n = Notifier(topic="test")
        result = n.send_complete(42, pr_url="https://github.com/pull/55")

        assert result is True
        cmd = mock_run.call_args[0][0]
        d_idx = cmd.index("-d")
        message = cmd[d_idx + 1]
        assert "Issue #42" in message
        assert "https://github.com/pull/55" in message

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_fails_gracefully_on_http_error(self, mock_run):
        mock_run.return_value = MagicMock(stdout="500", stderr="server error")
        n = Notifier(topic="test")
        result = n.send_checkpoint(1, "test")
        assert result is False

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_fails_gracefully_on_timeout(self, mock_run):
        mock_run.side_effect = subprocess.TimeoutExpired(cmd="curl", timeout=10)
        n = Notifier(topic="test")
        result = n.send_checkpoint(1, "test")
        assert result is False

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_send_fails_gracefully_on_exception(self, mock_run):
        mock_run.side_effect = OSError("no curl")
        n = Notifier(topic="test")
        result = n.send_checkpoint(1, "test")
        assert result is False

    def test_send_when_disabled_returns_false(self):
        n = Notifier(config=NotifyConfig(enabled=False, topic=""))
        result = n.send_checkpoint(1, "test")
        assert result is False

    @patch("orchestrator.pipeline.notifications.subprocess.run")
    def test_custom_server_url(self, mock_run):
        mock_run.return_value = MagicMock(stdout="200", stderr="")
        n = Notifier(
            topic="test",
            server_url="https://ntfy.example.com",
        )
        n.send_checkpoint(1, "test")
        cmd = mock_run.call_args[0][0]
        assert "https://ntfy.example.com/test" in cmd
