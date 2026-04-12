"""Log session command — captures session transcript.

Replaces log-session.sh (31 lines).
Wraps a command, teeing stdout/stderr to a timestamped log file.
"""

from __future__ import annotations

import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from orchestrator.config import resolve_project_dir


def run(role: str, command: list[str]) -> int:
    """Run a command and log its output to tasks/sessions/.

    Returns the command's exit code.
    """
    project_dir = resolve_project_dir()
    session_dir = project_dir / "tasks" / "sessions"
    session_dir.mkdir(parents=True, exist_ok=True)

    now = datetime.now(timezone.utc)
    logfile = session_dir / f"{now.strftime('%Y-%m-%d-%H%M')}-{role}.log"

    header = (
        f"=== Session: role={role} date={now.isoformat()} ===\n"
        f"=== Command: {' '.join(command)} ===\n"
        f"---\n"
    )
    sys.stdout.write(header)
    sys.stdout.flush()

    with open(logfile, "w") as f:
        f.write(header)

        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        f.write(result.stdout)
        sys.stdout.write(result.stdout)
        sys.stdout.flush()

        end_ts = datetime.now(timezone.utc).isoformat()
        footer = (
            f"---\n"
            f"=== Session ended: {end_ts} exit={result.returncode} ===\n"
        )
        f.write(footer)
        sys.stdout.write(footer)
        sys.stdout.flush()

    print(f"Log file: {logfile}")
    return result.returncode
