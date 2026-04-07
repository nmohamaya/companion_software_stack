"""Console I/O abstraction for testable interactive output.

Provides IOProtocol interface with two implementations:
  - Console: real terminal with ANSI colors (production)
  - TestConsole: captures output and provides scripted input (testing)

Replaces the color constants and interactive prompts duplicated across
all 12 bash scripts.
"""

from __future__ import annotations

import sys
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
