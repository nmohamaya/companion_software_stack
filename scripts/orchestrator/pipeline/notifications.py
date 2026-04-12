# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Push notification support via ntfy.sh for pipeline checkpoints.

Sends mobile-friendly push notifications when the pipeline reaches a
checkpoint, encounters an error, or completes. Users receive a ping on
their phone and can attach to the tmux session via SSH to interact.

Configuration:
  Set NTFY_TOPIC environment variable or pass topic= to Notifier.
  No account needed — just pick a unique, hard-to-guess topic string.

  For production use, self-host ntfy and set NTFY_TOKEN for authentication:
    NTFY_SERVER=https://ntfy.internal.example.com
    NTFY_TOPIC=pipeline-notifications
    NTFY_TOKEN=tk_your_auth_token_here

  WARNING: The default ntfy.sh server is public — anyone who knows the
  topic name can subscribe to notifications. Use a hard-to-guess topic
  or self-host with authentication for sensitive pipelines.

Usage:
    notifier = Notifier(topic="drone-pipeline-nm")
    notifier.send_checkpoint(1, "12 files changed, build passes", issue=367)
    notifier.send_error("Build failed with 3 errors", issue=367)
    notifier.send_complete(367, pr_url="https://github.com/...")
"""

from __future__ import annotations

import logging
import os
import re
import subprocess
from dataclasses import dataclass, field
from enum import Enum, auto
from urllib.parse import quote as url_quote

logger = logging.getLogger(__name__)


class NotifyEvent(Enum):
    """Events that can trigger a notification."""

    CHECKPOINT = auto()
    ERROR = auto()
    COMPLETE = auto()


# Topic names must be alphanumeric, hyphens, or underscores only.
_VALID_TOPIC_RE = re.compile(r"^[a-zA-Z0-9_-]+$")


def _validate_topic(topic: str) -> str:
    """Validate and return the topic, or raise ValueError."""
    if topic and not _VALID_TOPIC_RE.match(topic):
        raise ValueError(
            f"Invalid ntfy topic '{topic}': must contain only "
            f"alphanumeric characters, hyphens, or underscores."
        )
    return topic


def _validate_server_url(url: str) -> str:
    """Validate server URL uses HTTPS (unless explicitly opted out)."""
    if url and not url.startswith("https://"):
        if os.environ.get("NTFY_INSECURE") == "1":
            logger.warning(
                "NTFY_SERVER is not HTTPS (%s) — proceeding because "
                "NTFY_INSECURE=1 is set.",
                url,
            )
        else:
            raise ValueError(
                f"NTFY_SERVER must use HTTPS (got '{url}'). "
                f"Set NTFY_INSECURE=1 to allow non-HTTPS."
            )
    return url


@dataclass
class NotifyConfig:
    """Notification configuration — mirrors pipeline.notifications in config."""

    enabled: bool = False
    provider: str = "ntfy"
    topic: str = ""
    server_url: str = "https://ntfy.sh"
    auth_token: str = ""  # NTFY_TOKEN for authenticated ntfy servers
    notify_on: list[str] = field(
        default_factory=lambda: ["checkpoint", "error", "complete"]
    )

    @classmethod
    def from_env(cls) -> NotifyConfig:
        """Build config from environment variables.

        NTFY_TOPIC:    topic name (required to enable)
        NTFY_SERVER:   server URL (default: https://ntfy.sh), must be HTTPS
        NTFY_TOKEN:    auth token for authenticated ntfy servers (optional)
        NTFY_EVENTS:   comma-separated event list (default: checkpoint,error,complete)
        NTFY_INSECURE: set to "1" to allow non-HTTPS server URLs
        """
        topic = os.environ.get("NTFY_TOPIC", "")
        server = os.environ.get("NTFY_SERVER", "https://ntfy.sh")
        token = os.environ.get("NTFY_TOKEN", "")
        events_str = os.environ.get("NTFY_EVENTS", "checkpoint,error,complete")
        events = [e.strip().lower() for e in events_str.split(",") if e.strip()]

        # Validate inputs
        if topic:
            _validate_topic(topic)
            _validate_server_url(server)

        # Warn about public ntfy.sh without authentication
        if (
            topic
            and server.rstrip("/") == "https://ntfy.sh"
            and not token
        ):
            logger.warning(
                "Using public ntfy.sh without authentication — anyone who "
                "knows the topic '%s' can read notifications. Consider "
                "self-hosting ntfy with NTFY_TOKEN for sensitive pipelines.",
                topic,
            )

        return cls(
            enabled=bool(topic),
            topic=topic,
            server_url=server,
            auth_token=token,
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
        auth_token: str = "",
        config: NotifyConfig | None = None,
    ) -> None:
        if config is not None:
            self._config = config
        elif topic:
            _validate_topic(topic)
            self._config = NotifyConfig(
                enabled=True,
                topic=topic,
                server_url=server_url,
                auth_token=auth_token,
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

        Returns True on success, False on failure. Never raises — notification
        failures are logged but must not block the pipeline.
        """
        if not self.enabled:
            return False

        # Build URL with properly encoded topic to prevent path traversal
        base = self._config.server_url.rstrip("/")
        safe_topic = url_quote(self._config.topic, safe="")
        url = f"{base}/{safe_topic}"

        cmd = [
            "curl", "-s",
            "--fail-with-body",  # fail on HTTP errors
            "-o", "/dev/null",
            "-w", "%{http_code}",
        ]
        cmd.extend(["-d", message])
        if title:
            cmd.extend(["-H", f"Title: {title}"])
        if tags:
            cmd.extend(["-H", f"Tags: {tags}"])
        if priority:
            cmd.extend(["-H", f"Priority: {priority}"])
        # Add authentication header if token is configured
        if self._config.auth_token:
            cmd.extend(["-H", f"Authorization: Bearer {self._config.auth_token}"])
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
        except (OSError, subprocess.SubprocessError) as e:
            logger.warning("Notification error: %s", e)
            return False
