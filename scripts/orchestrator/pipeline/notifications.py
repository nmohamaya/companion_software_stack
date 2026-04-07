"""Push notification support via ntfy.sh for pipeline checkpoints.

Sends mobile-friendly push notifications when the pipeline reaches a
checkpoint, encounters an error, or completes. Users receive a ping on
their phone and can attach to the tmux session via SSH to interact.

Configuration:
  Set NTFY_TOPIC environment variable or pass topic= to Notifier.
  No account needed — just pick a unique, hard-to-guess topic string.

Usage:
    notifier = Notifier(topic="drone-pipeline-nm")
    notifier.send_checkpoint(1, "12 files changed, build passes", issue=367)
    notifier.send_error("Build failed with 3 errors", issue=367)
    notifier.send_complete(367, pr_url="https://github.com/...")
"""

from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass, field
from enum import Enum, auto

logger = logging.getLogger(__name__)


class NotifyEvent(Enum):
    """Events that can trigger a notification."""

    CHECKPOINT = auto()
    ERROR = auto()
    COMPLETE = auto()


@dataclass
class NotifyConfig:
    """Notification configuration — mirrors pipeline.notifications in config."""

    enabled: bool = False
    provider: str = "ntfy"
    topic: str = ""
    server_url: str = "https://ntfy.sh"
    notify_on: list[str] = field(
        default_factory=lambda: ["checkpoint", "error", "complete"]
    )

    @classmethod
    def from_env(cls) -> NotifyConfig:
        """Build config from environment variables.

        NTFY_TOPIC:   topic name (required to enable)
        NTFY_SERVER:  server URL (default: https://ntfy.sh)
        NTFY_EVENTS:  comma-separated event list (default: checkpoint,error,complete)
        """
        topic = os.environ.get("NTFY_TOPIC", "")
        server = os.environ.get("NTFY_SERVER", "https://ntfy.sh")
        events_str = os.environ.get("NTFY_EVENTS", "checkpoint,error,complete")
        events = [e.strip() for e in events_str.split(",") if e.strip()]

        return cls(
            enabled=bool(topic),
            topic=topic,
            server_url=server,
            notify_on=events,
        )


class Notifier:
    """Send push notifications via ntfy.sh.

    Gracefully degrades — notification failures are logged but never
    raise exceptions or block the pipeline.
    """

    def __init__(
        self,
        *,
        topic: str = "",
        server_url: str = "https://ntfy.sh",
        config: NotifyConfig | None = None,
    ) -> None:
        if config is not None:
            self._config = config
        elif topic:
            self._config = NotifyConfig(
                enabled=True,
                topic=topic,
                server_url=server_url,
            )
        else:
            self._config = NotifyConfig.from_env()

    @property
    def enabled(self) -> bool:
        return self._config.enabled and bool(self._config.topic)

    @property
    def topic(self) -> str:
        return self._config.topic

    def should_notify(self, event: NotifyEvent) -> bool:
        """Check if this event type should trigger a notification."""
        if not self.enabled:
            return False
        return event.name.lower() in self._config.notify_on

    def send_checkpoint(
        self,
        checkpoint_number: int,
        summary: str,
        *,
        issue: int = 0,
        recommendation: str = "",
    ) -> bool:
        """Send notification when pipeline reaches a checkpoint.

        Returns True if notification was sent successfully.
        """
        if not self.should_notify(NotifyEvent.CHECKPOINT):
            return False

        title = f"Pipeline Checkpoint {checkpoint_number}/5"
        lines = []
        if issue:
            lines.append(f"Issue #{issue}")
        lines.append(summary)
        if recommendation:
            lines.append(f"Agent recommends: {recommendation}")
        lines.append("")
        if issue:
            lines.append(f"Attach: tmux attach -t pipeline-{issue}")

        return self._send(
            message="\n".join(lines),
            title=title,
            tags="robot,hourglass",
            priority="default",
        )

    def send_error(
        self,
        error_message: str,
        *,
        issue: int = 0,
        state: str = "",
    ) -> bool:
        """Send notification when pipeline encounters an error.

        Returns True if notification was sent successfully.
        """
        if not self.should_notify(NotifyEvent.ERROR):
            return False

        title = "Pipeline Error"
        lines = []
        if issue:
            lines.append(f"Issue #{issue}")
        if state:
            lines.append(f"State: {state}")
        lines.append(error_message)
        if issue:
            lines.append(f"\nAttach: tmux attach -t pipeline-{issue}")

        return self._send(
            message="\n".join(lines),
            title=title,
            tags="warning",
            priority="high",
        )

    def send_complete(
        self,
        issue: int,
        *,
        pr_url: str = "",
        pr_number: int = 0,
    ) -> bool:
        """Send notification when pipeline completes successfully.

        Returns True if notification was sent successfully.
        """
        if not self.should_notify(NotifyEvent.COMPLETE):
            return False

        title = "Pipeline Complete"
        lines = [f"Issue #{issue} — pipeline finished successfully."]
        if pr_url:
            lines.append(f"PR: {pr_url}")
        elif pr_number:
            lines.append(f"PR #{pr_number}")

        return self._send(
            message="\n".join(lines),
            title=title,
            tags="white_check_mark",
            priority="default",
        )

    def _send(
        self,
        *,
        message: str,
        title: str = "",
        tags: str = "",
        priority: str = "default",
    ) -> bool:
        """Send a notification via curl to ntfy.sh.

        Returns True on success, False on failure. Never raises.
        """
        if not self.enabled:
            return False

        url = f"{self._config.server_url.rstrip('/')}/{self._config.topic}"
        cmd = ["curl", "-s", "-o", "/dev/null", "-w", "%{http_code}"]
        cmd.extend(["-d", message])
        if title:
            cmd.extend(["-H", f"Title: {title}"])
        if tags:
            cmd.extend(["-H", f"Tags: {tags}"])
        if priority:
            cmd.extend(["-H", f"Priority: {priority}"])
        cmd.append(url)

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10,
            )
            status_code = result.stdout.strip()
            if status_code == "200":
                logger.debug("Notification sent: %s", title)
                return True
            logger.warning(
                "Notification failed (HTTP %s): %s",
                status_code,
                result.stderr.strip(),
            )
            return False
        except subprocess.TimeoutExpired:
            logger.warning("Notification timed out: %s", title)
            return False
        except Exception as e:
            logger.warning("Notification error: %s", e)
            return False
