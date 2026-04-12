# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Console I/O abstraction for testable interactive output.

Provides IOProtocol interface with two implementations:
  - Console: real terminal with ANSI colors (production)
  - TestConsole: captures output and provides scripted input (testing)

Replaces the color constants and interactive prompts duplicated across
all 12 bash scripts.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Protocol


class IOProtocol(Protocol):
    """Interface for all console I/O — enables mock injection in tests."""

    def print(self, msg: str, *, end: str = "\n") -> None: ...
    def prompt(self, msg: str, default: str = "") -> str: ...
    def confirm(self, msg: str, default: bool = False) -> bool: ...


class Console:
    """Real terminal implementation with ANSI color output."""

    # ANSI color codes — previously defined identically in all 12 scripts
    GREEN = "\033[0;32m"
    YELLOW = "\033[1;33m"
    RED = "\033[0;31m"
    CYAN = "\033[0;36m"
    BOLD = "\033[1m"
    RESET = "\033[0m"

    def print(self, msg: str, *, end: str = "\n") -> None:
        sys.stdout.write(msg + end)
        sys.stdout.flush()

    def prompt(self, msg: str, default: str = "") -> str:
        try:
            value = input(msg).strip()
        except (EOFError, KeyboardInterrupt):
            self.print("")
            return default
        return value if value else default

    def confirm(self, msg: str, default: bool = False) -> bool:
        hint = "[Y/n]" if default else "[y/N]"
        response = self.prompt(f"{msg} {hint} ")
        if not response:
            return default
        return response.lower().startswith("y")

    # ── Convenience methods ────────────────────────────────────────────────

    def pass_(self, msg: str) -> None:
        """Print a green PASS message."""
        self.print(f"  {self.GREEN}PASS{self.RESET}  {msg}")

    def warn(self, msg: str) -> None:
        """Print a yellow WARN message."""
        self.print(f"  {self.YELLOW}WARN{self.RESET}  {msg}")

    def fail(self, msg: str) -> None:
        """Print a red FAIL message."""
        self.print(f"  {self.RED}FAIL{self.RESET}  {msg}")

    def info(self, msg: str) -> None:
        """Print a cyan INFO message."""
        self.print(f"  {self.CYAN}INFO{self.RESET}  {msg}")

    def error(self, msg: str) -> None:
        """Print a red Error message to stderr."""
        sys.stderr.write(f"{self.RED}Error: {msg}{self.RESET}\n")
        sys.stderr.flush()

    def header(self, title: str) -> None:
        """Print a bold boxed header (replaces pipeline_header in bash)."""
        self.print("")
        bar = "═" * 55
        self.print(f"{self.BOLD}{bar}{self.RESET}")
        self.print(f"{self.BOLD}  {title}{self.RESET}")
        self.print(f"{self.BOLD}{bar}{self.RESET}")
        self.print("")

    def options(self, choices: list[tuple[str, str]]) -> None:
        """Print a colored option list. Each item is (key, description)."""
        for key, desc in choices:
            self.print(f"  {self.CYAN}[{key}]{self.RESET} {desc}")
        self.print("")

    def role_table(self, roles: list[tuple[str, str, str]]) -> None:
        """Print a formatted role table. Each item is (role, tier, model)."""
        self.print(f"\n{self.BOLD}{'ROLE':<28} {'TIER':<10} MODEL{self.RESET}")
        self.print(f"{'---':<28} {'---':<10} ---")
        for role, tier, model in roles:
            self.print(f"{role:<28} {tier:<10} {model}")
        self.print("")


class AutoConsole(Console):
    """Console that auto-picks "forward" actions and logs all output to a file.

    Used by ``--auto`` pipeline mode: the full FSM runs with all checkpoints,
    but no interactive prompts block.  After completion the caller (e.g.
    Claude Code acting as tech-lead) reads the log file to review and present
    findings to the user.

    Context-aware: tracks the most recent header() to determine which
    checkpoint is active, then picks the correct "forward" action:
      - CP1 (Changes Review):    "a" (accept)
      - CP2 (Commit Approval):   "c" (commit)
      - CP3 (PR Preview):        "c" (create) or "s" (skip if PR exists)
      - CP4 (Review Findings):   "a" (accept)
      - CP5 (Final Summary):     "d" (done)
      - Pipeline Aborted:        "p" (preserve worktree)
      - Tech-lead routing:       "a" (accept recommendation)
      - Branch strategy:         "m" (main)
      - Anything else:           the prompt's default value
    """

    # Map checkpoint header substring → forward action for generic prompts.
    _CHECKPOINT_ACTIONS: dict[str, str] = {
        "CHECKPOINT 1": "a",   # accept
        "CHECKPOINT 2": "c",   # commit
        "CHECKPOINT 3": "c",   # create PR
        "CHECKPOINT 4": "a",   # accept findings
        "CHECKPOINT 5": "d",   # done
        "Pipeline Aborted": "p",  # preserve
    }

    # Prompt-specific overrides (checked before checkpoint context).
    # Branch strategy defaults to "m" (main). Use --base to override:
    #   --auto --base integration/epic-284
    _PROMPT_ANSWERS: list[tuple[str, str]] = [
        ("Your choice [a]", "a"),       # tech-lead routing default
        ("Your choice [m]", "m"),       # branch strategy default
        ("Abort pipeline?", "n"),       # don't confirm abort
        ("New title", ""),              # keep existing PR title
        ("Integration branch", ""),     # keep default name
        ("Select branch", "1"),         # pick first integration branch
    ]

    def __init__(self, log_path: Path | None = None) -> None:
        self._log_path = log_path
        self._log_lines: list[str] = []
        self._current_context: str = ""  # last header seen

    def header(self, title: str) -> None:
        super().header(title)
        self._log_lines.append(f"=== {title} ===")
        self._current_context = title

    def print(self, msg: str, *, end: str = "\n") -> None:
        super().print(msg, end=end)
        self._log_lines.append(msg)

    def prompt(self, msg: str, default: str = "") -> str:
        # 1. Check prompt-specific overrides first
        for substr, auto_val in self._PROMPT_ANSWERS:
            if substr in msg:
                answer = auto_val if auto_val else default
                self._log_lines.append(f"[AUTO] {msg.strip()} → {answer!r}")
                super().print(f"[AUTO] {msg.strip()} → {answer!r}")
                return answer

        # 2. Use checkpoint context for generic "Your choice:" prompts
        if "Your choice" in msg:
            for ctx_key, action in self._CHECKPOINT_ACTIONS.items():
                if ctx_key in self._current_context:
                    self._log_lines.append(
                        f"[AUTO:{ctx_key}] {msg.strip()} → {action!r}"
                    )
                    super().print(
                        f"[AUTO:{ctx_key}] {msg.strip()} → {action!r}"
                    )
                    return action

        # 3. Fall back to prompt default
        self._log_lines.append(f"[AUTO:default] {msg.strip()} → {default!r}")
        super().print(f"[AUTO:default] {msg.strip()} → {default!r}")
        return default

    def confirm(self, msg: str, default: bool = False) -> bool:
        # Auto-confirm forward actions (yes), deny destructive ones
        result = default
        self._log_lines.append(f"[AUTO-CONFIRM] {msg.strip()} → {result}")
        super().print(f"[AUTO-CONFIRM] {msg.strip()} → {result}")
        return result

    @property
    def log(self) -> str:
        """Full captured output as a single string."""
        return "\n".join(self._log_lines)

    def flush_log(self) -> None:
        """Write captured output to the log file (if configured)."""
        if self._log_path:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_path.write_text(self.log)


class TestConsole:
    """Test double that captures output and provides scripted input.

    Usage in tests:
        io = TestConsole(inputs=["accept", "y", "done"])
        # ... run code that calls io.prompt() / io.confirm()
        assert "PASS" in io.output
    """

    def __init__(self, inputs: list[str] | None = None) -> None:
        self._inputs: list[str] = list(inputs or [])
        self._input_idx: int = 0
        self.lines: list[str] = []

    @property
    def output(self) -> str:
        """All captured output as a single string."""
        return "\n".join(self.lines)

    def print(self, msg: str, *, end: str = "\n") -> None:
        self.lines.append(msg)

    def prompt(self, msg: str, default: str = "") -> str:
        self.lines.append(f"[PROMPT] {msg}")
        if self._input_idx < len(self._inputs):
            value = self._inputs[self._input_idx]
            self._input_idx += 1
            return value
        return default

    def confirm(self, msg: str, default: bool = False) -> bool:
        self.lines.append(f"[CONFIRM] {msg}")
        if self._input_idx < len(self._inputs):
            value = self._inputs[self._input_idx]
            self._input_idx += 1
            return value.lower().startswith("y")
        return default

    # ── Convenience methods (mirror Console) ───────────────────────────────

    def pass_(self, msg: str) -> None:
        self.print(f"  PASS  {msg}")

    def warn(self, msg: str) -> None:
        self.print(f"  WARN  {msg}")

    def fail(self, msg: str) -> None:
        self.print(f"  FAIL  {msg}")

    def info(self, msg: str) -> None:
        self.print(f"  INFO  {msg}")

    def error(self, msg: str) -> None:
        self.print(f"Error: {msg}")

    def header(self, title: str) -> None:
        self.print(f"=== {title} ===")

    def options(self, choices: list[tuple[str, str]]) -> None:
        for key, desc in choices:
            self.print(f"  [{key}] {desc}")

    def role_table(self, roles: list[tuple[str, str, str]]) -> None:
        for role, tier, model in roles:
            self.print(f"{role:<28} {tier:<10} {model}")
