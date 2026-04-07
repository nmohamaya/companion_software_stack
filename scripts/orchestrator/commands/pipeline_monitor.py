"""Pipeline monitoring commands — list, attach, status.

Provides the `python -m orchestrator pipeline` subcommand group:
  - pipeline list    — show active pipeline tmux sessions
  - pipeline attach  — attach to a running pipeline session
  - pipeline status  — show checkpoint status for a pipeline
  - pipeline kill    — kill a pipeline tmux session
"""

from __future__ import annotations

from orchestrator.console import Console
from orchestrator.pipeline.tmux import (
    TmuxSession,
    list_pipeline_sessions,
    session_status,
    tmux_available,
)


def run_list(*, io: Console | None = None) -> int:
    """List all active pipeline tmux sessions."""
    if io is None:
        io = Console()

    if not tmux_available():
        io.error("tmux is not installed. Install with: sudo apt install tmux")
        return 1

    sessions = list_pipeline_sessions()
    if not sessions:
        io.print("No active pipeline sessions.")
        io.print("")
        io.print("Launch a pipeline in tmux:")
        io.print(
            "  python -m orchestrator deploy-issue <issue> --pipeline --tmux"
        )
        return 0

    io.header("Active Pipeline Sessions")
    for session in sessions:
        io.print(session.status_line)
    io.print("")
    io.print("Attach to a session:")
    io.print("  python -m orchestrator pipeline attach <issue-number>")
    io.print("  tmux attach -t pipeline-<issue-number>")
    return 0


def run_attach(issue: int, *, io: Console | None = None) -> int:
    """Attach to a running pipeline tmux session."""
    if io is None:
        io = Console()

    if not tmux_available():
        io.error("tmux is not installed.")
        return 1

    session = TmuxSession(issue)
    if not session.exists():
        io.error(f"No pipeline session for issue #{issue}.")
        io.print("")
        io.print("Active sessions:")
        return run_list(io=io)

    io.print(f"Attaching to pipeline-{issue}...")
    session.attach()
    # attach() uses exec — if we reach here, it failed
    io.error("Failed to attach to tmux session.")
    return 1


def run_status(issue: int | None = None, *, io: Console | None = None) -> int:
    """Show status of a pipeline or all pipelines."""
    if io is None:
        io = Console()

    if issue is not None:
        status = session_status(issue)
        if status:
            io.header(f"Pipeline Status — Issue #{issue}")
            io.print(status)

            # Also check tmux session
            session = TmuxSession(issue)
            if session.exists():
                io.print(f"  tmux:        session alive (pipeline-{issue})")
            else:
                io.print("  tmux:        no active session")
            return 0
        else:
            io.error(f"No pipeline state found for issue #{issue}.")
            return 1

    # No specific issue — show all sessions with their states
    sessions = list_pipeline_sessions()
    if not sessions:
        io.print("No active pipeline sessions.")
        return 0

    io.header("Pipeline Status — All Sessions")
    for session_info in sessions:
        status = session_status(session_info.issue_number)
        if status:
            io.print(status)
        else:
            io.print(f"Issue #{session_info.issue_number}: (no state file)")
        io.print("")
    return 0


def run_kill(issue: int, *, io: Console | None = None) -> int:
    """Kill a pipeline tmux session."""
    if io is None:
        io = Console()

    session = TmuxSession(issue)
    if not session.exists():
        io.error(f"No pipeline session for issue #{issue}.")
        return 1

    if io.confirm(f"Kill pipeline session for issue #{issue}?"):
        if session.kill():
            io.pass_(f"Session pipeline-{issue} killed.")
            return 0
        else:
            io.fail("Failed to kill session.")
            return 1
    return 0
