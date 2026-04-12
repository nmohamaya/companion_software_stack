"""Session metrics parser command.

Replaces parse-session-metrics.sh (125 lines).
Parses session log files for duration, tool calls, and error counts.
"""

from __future__ import annotations

import re
from pathlib import Path

from orchestrator.config import resolve_project_dir
from orchestrator.console import Console


def run(
    io: Console | None = None,
    *,
    logfile: str | None = None,
    summary: bool = False,
) -> int:
    """Parse session metrics from log files."""
    if io is None:
        io = Console()

    if summary:
        return _run_summary(io)
    if logfile:
        return _parse_file(io, Path(logfile))

    io.print("Usage: python -m orchestrator metrics [<logfile>] [--summary]")
    return 1


def _run_summary(io: Console) -> int:
    """Parse all session logs."""
    project_dir = resolve_project_dir()
    session_dir = project_dir / "tasks" / "sessions"

    io.header("Session Metrics Summary")

    if not session_dir.is_dir():
        io.print(f"No sessions directory found at {session_dir}")
        return 0

    logs = sorted(session_dir.glob("*.log"))
    if not logs:
        io.print(f"No session logs found in {session_dir}")
        return 0

    for log in logs:
        _parse_file(io, log)
        io.print("---")

    io.print(f"Total sessions: {len(logs)}")
    return 0


def _parse_file(io: Console, path: Path) -> int:
    """Parse a single session log file."""
    if not path.exists():
        io.error(f"File not found: {path}")
        return 1

    text = path.read_text()
    name = path.name

    # Extract role from filename: YYYY-MM-DD-HHMM-<role>.log
    role_match = re.match(r"\d{4}-\d{2}-\d{2}-\d{4}-(.+)\.log$", name)
    role = role_match.group(1) if role_match else "unknown"

    # Extract date
    date_match = re.match(r"(\d{4}-\d{2}-\d{2})", name)
    session_date = date_match.group(1) if date_match else "unknown"

    # Duration from timestamps in first/last lines
    timestamps = re.findall(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}", text)
    duration = "unknown"
    if len(timestamps) >= 2:
        # Simple approximation from first and last timestamp
        duration = f"{timestamps[0]} → {timestamps[-1]}"

    # Counts
    files_changed = len(
        re.findall(r"(?i)Edit|Write|created|modified", text)
    )
    test_runs = len(re.findall(r"(?i)ctest|test.*pass|test.*fail", text))
    errors = len(re.findall(r"(?i)\berror\b|\bfailed\b|\bFAIL\b", text))

    # Tool calls
    tool_read = text.count("Read")
    tool_edit = text.count("Edit")
    tool_write = text.count("Write")
    tool_bash = text.count("Bash")
    tool_grep = text.count("Grep")
    tool_glob = text.count("Glob")

    io.print(f"{'File':<30}  {'Role':<14}  {'Date':<10}")
    io.print(f"{name[:30]:<30}  {role:<14}  {session_date:<10}")
    io.print("")
    io.print(f"  Files changed:  {files_changed}")
    io.print(f"  Test runs:      {test_runs}")
    io.print(f"  Errors:         {errors}")
    io.print(
        f"  Tool calls:     Read={tool_read} Edit={tool_edit} "
        f"Write={tool_write} Bash={tool_bash} "
        f"Grep={tool_grep} Glob={tool_glob}"
    )
    io.print("")

    return 0
