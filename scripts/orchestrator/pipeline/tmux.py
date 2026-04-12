# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""tmux session management for pipeline runs.

Provides:
  - TmuxSession: launch, attach, detach, and query pipeline sessions
  - list_pipeline_sessions: find all active pipeline-* sessions
  - session_status: query which checkpoint a pipeline is waiting at

Usage:
    tmux = TmuxSession(issue=367)
    tmux.launch("python -m orchestrator deploy-issue 367 --pipeline")
    # From any terminal / phone SSH:
    tmux.attach()

Session naming: pipeline-<issue-number>
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
from dataclasses import dataclass

logger = logging.getLogger(__name__)


def tmux_available() -> bool:
    """Check if tmux is installed and accessible."""
    return shutil.which("tmux") is not None


@dataclass
class TmuxSessionInfo:
    """Information about a running tmux session."""

    name: str
    issue_number: int
    created: str  # tmux-formatted creation time
    attached: bool
    windows: int
    width: int
    height: int

    @property
    def status_line(self) -> str:
        """Human-readable one-line status."""
        attached_str = "attached" if self.attached else "detached"
        return (
            f"  {self.name:<25} issue #{self.issue_number:<6} "
            f"{attached_str:<10} {self.windows} window(s) "
            f"({self.width}x{self.height}) "
            f"created {self.created}"
        )


class TmuxSession:
    """Manage a tmux session for a pipeline run.

    Each pipeline gets a named session: pipeline-<issue-number>.
    The session persists after detach, allowing reattach from any device.
    """

    SESSION_PREFIX = "pipeline-"

    def __init__(self, issue: int) -> None:
        self.issue = issue
        self.session_name = f"{self.SESSION_PREFIX}{issue}"

    def exists(self) -> bool:
        """Check if the tmux session already exists."""
        if not tmux_available():
            return False
        result = subprocess.run(
            ["tmux", "has-session", "-t", self.session_name],
            capture_output=True,
        )
        return result.returncode == 0

    def launch(self, command: str, *, cwd: str | None = None) -> bool:
        """Create a new tmux session running the given command.

        Returns True if the session was created, False on failure.
        Does NOT attach — the session runs in the background.
        """
        if not tmux_available():
            logger.warning("tmux not installed — running without session")
            return False

        if self.exists():
            logger.info("Session %s already exists", self.session_name)
            return True

        cmd = [
            "tmux", "new-session",
            "-d",  # detached
            "-s", self.session_name,
            "-x", "200",  # width (generous for agent output)
            "-y", "50",   # height
        ]
        if cwd:
            cmd.extend(["-c", cwd])
        cmd.append(command)

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                logger.info("tmux session created: %s", self.session_name)
                return True
            logger.warning(
                "Failed to create tmux session: %s", result.stderr.strip()
            )
            return False
        except Exception as e:
            logger.warning("tmux launch error: %s", e)
            return False

    def exec_attach(self) -> bool:
        """Attach to the tmux session via execvp (replaces current process).

        WARNING: On success, this call does NOT return — os.execvp replaces
        the current process image entirely. Only call this when you are ready
        to hand control to tmux.

        Returns False if the session doesn't exist, tmux is unavailable,
        or execvp fails (e.g., session died between exists() check and exec).
        """
        if not tmux_available():
            return False
        if not self.exists():
            return False

        try:
            # execvp replaces the process — only returns on failure
            os.execvp("tmux", ["tmux", "attach-session", "-t", self.session_name])
        except OSError as e:
            # TOCTOU: session may have died between exists() and execvp,
            # or tmux binary could be missing/broken.
            logger.warning(
                "Failed to exec-attach to %s: %s", self.session_name, e
            )
        return False

    def attach(self) -> bool:
        """Attach to the tmux session via subprocess (caller retains control).

        Unlike exec_attach(), this runs tmux in a subprocess and returns
        when the user detaches. Safer for callers that need to continue
        after the user detaches.

        Returns True if tmux ran successfully, False on failure.
        """
        if not tmux_available():
            return False
        if not self.exists():
            return False

        try:
            result = subprocess.run(
                ["tmux", "attach-session", "-t", self.session_name],
            )
            return result.returncode == 0
        except OSError as e:
            logger.warning(
                "Failed to attach to %s: %s", self.session_name, e
            )
            return False

    def send_keys(self, keys: str) -> bool:
        """Send keys to the tmux session.

        Intended for test automation only — keys are passed directly to
        ``tmux send-keys`` without sanitization. Do not expose to
        untrusted input.

        Returns True on success.
        """
        if not self.exists():
            return False
        result = subprocess.run(
            ["tmux", "send-keys", "-t", self.session_name, keys, "Enter"],
            capture_output=True,
        )
        return result.returncode == 0

    def kill(self) -> bool:
        """Kill the tmux session.

        Returns True on success.
        """
        if not self.exists():
            return True
        result = subprocess.run(
            ["tmux", "kill-session", "-t", self.session_name],
            capture_output=True,
        )
        return result.returncode == 0

    def capture_pane(self, *, lines: int = 50) -> str:
        """Capture the last N lines of visible output from the session.

        Useful for status queries without attaching.
        """
        if not self.exists():
            return ""
        result = subprocess.run(
            [
                "tmux", "capture-pane",
                "-t", self.session_name,
                "-p",  # print to stdout
                "-S", f"-{lines}",  # start N lines back
            ],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            return result.stdout
        return ""


def list_pipeline_sessions() -> list[TmuxSessionInfo]:
    """List all active pipeline-* tmux sessions.

    Returns session info sorted by issue number.
    """
    if not tmux_available():
        return []

    result = subprocess.run(
        [
            "tmux", "list-sessions",
            "-F",
            "#{session_name}|#{session_created_string}|"
            "#{session_attached}|#{session_windows}|"
            "#{session_width}|#{session_height}",
        ],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        return []

    sessions: list[TmuxSessionInfo] = []
    for line in result.stdout.strip().splitlines():
        if not line:
            continue
        parts = line.split("|")
        if len(parts) < 6:
            continue

        name = parts[0]
        if not name.startswith(TmuxSession.SESSION_PREFIX):
            continue

        try:
            issue_number = int(name[len(TmuxSession.SESSION_PREFIX):])
        except ValueError:
            continue

        sessions.append(TmuxSessionInfo(
            name=name,
            issue_number=issue_number,
            created=parts[1],
            attached=parts[2] == "1",
            windows=int(parts[3]) if parts[3].isdigit() else 1,
            width=int(parts[4]) if parts[4].isdigit() else 0,
            height=int(parts[5]) if parts[5].isdigit() else 0,
        ))

    sessions.sort(key=lambda s: s.issue_number)
    return sessions


def session_status(issue: int) -> str | None:
    """Query which checkpoint a pipeline session is waiting at.

    Reads the .pipeline-state file to determine the current FSM state.
    Returns a human-readable status string, or None if not found.
    """
    from orchestrator.config import resolve_project_dir
    from orchestrator.pipeline.state import PipelineStateData

    state_file = resolve_project_dir() / ".pipeline-state"
    if not state_file.exists():
        return None

    try:
        state = PipelineStateData.load(state_file)
        if state.issue_number != issue:
            return None

        current = state.current_state
        checkpoints_done = len(state.checkpoint_history)

        return (
            f"Issue #{issue}: {state.issue_title}\n"
            f"  State:       {current}\n"
            f"  Role:        {state.agent_role}\n"
            f"  Branch:      {state.branch_name}\n"
            f"  Checkpoints: {checkpoints_done}/5 completed\n"
            f"  Fix iters:   {state.fix_iterations}"
        )
    except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
        return None
