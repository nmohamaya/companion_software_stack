"""Tests for orchestrator.console — Console and TestConsole I/O."""

from __future__ import annotations

from pathlib import Path

from orchestrator.console import AutoConsole, Console, TestConsole


class TestTestConsole:
    """Test the TestConsole mock implementation."""

    def test_print_captures_output(self):
        io = TestConsole()
        io.print("hello")
        io.print("world")
        assert io.lines == ["hello", "world"]
        assert "hello" in io.output

    def test_prompt_returns_scripted_input(self):
        io = TestConsole(inputs=["alice", "bob"])
        assert io.prompt("Name? ") == "alice"
        assert io.prompt("Name? ") == "bob"

    def test_prompt_returns_default_when_exhausted(self):
        io = TestConsole(inputs=["first"])
        assert io.prompt("Q? ") == "first"
        assert io.prompt("Q? ", default="fallback") == "fallback"

    def test_confirm_yes(self):
        io = TestConsole(inputs=["y"])
        assert io.confirm("Continue?") is True

    def test_confirm_no(self):
        io = TestConsole(inputs=["n"])
        assert io.confirm("Continue?") is False

    def test_confirm_default_when_exhausted(self):
        io = TestConsole(inputs=[])
        assert io.confirm("Continue?", default=True) is True
        assert io.confirm("Continue?", default=False) is False

    def test_pass_warn_fail_info(self):
        io = TestConsole()
        io.pass_("ok")
        io.warn("careful")
        io.fail("broken")
        io.info("note")
        assert "PASS" in io.lines[0]
        assert "WARN" in io.lines[1]
        assert "FAIL" in io.lines[2]
        assert "INFO" in io.lines[3]

    def test_error(self):
        io = TestConsole()
        io.error("bad thing")
        assert "Error: bad thing" in io.output

    def test_header(self):
        io = TestConsole()
        io.header("Test Phase")
        assert "Test Phase" in io.output

    def test_options(self):
        io = TestConsole()
        io.options([("a", "accept"), ("r", "reject")])
        assert "[a]" in io.output
        assert "[r]" in io.output

    def test_role_table(self):
        io = TestConsole()
        io.role_table([("tech-lead", "opus", "claude-opus-4-6")])
        assert "tech-lead" in io.output
        assert "opus" in io.output

    def test_prompt_records_in_output(self):
        io = TestConsole(inputs=["val"])
        io.prompt("Enter: ")
        assert "[PROMPT]" in io.output

    def test_confirm_records_in_output(self):
        io = TestConsole(inputs=["y"])
        io.confirm("Sure?")
        assert "[CONFIRM]" in io.output


class TestAutoConsole:
    """Test the AutoConsole context-aware auto-approve implementation."""

    def test_cp1_auto_accepts(self):
        ac = AutoConsole()
        ac.header("CHECKPOINT 1/5 — Changes Review")
        assert ac.prompt("  Your choice: ") == "a"

    def test_cp2_auto_commits(self):
        """CP2 generic 'Your choice:' must return 'c' (commit), not 'a' (abort)."""
        ac = AutoConsole()
        ac.header("CHECKPOINT 2/5 — Commit Approval")
        assert ac.prompt("  Your choice: ") == "c"

    def test_cp3_auto_creates(self):
        ac = AutoConsole()
        ac.header("CHECKPOINT 3/5 — PR Preview")
        assert ac.prompt("  Your choice: ") == "c"

    def test_cp4_auto_accepts(self):
        ac = AutoConsole()
        ac.header("CHECKPOINT 4/5 — Review & Test Findings")
        assert ac.prompt("  Your choice: ") == "a"

    def test_cp5_auto_done(self):
        ac = AutoConsole()
        ac.header("CHECKPOINT 5/5 — Final Summary")
        assert ac.prompt("  Your choice: ") == "d"

    def test_abort_handler_preserves(self):
        """Abort handler must preserve worktree, not clean up."""
        ac = AutoConsole()
        ac.header("Pipeline Aborted")
        assert ac.prompt("  Your choice [c]: ", default="c") == "p"

    def test_tech_lead_routing_accepts(self):
        ac = AutoConsole()
        assert ac.prompt("  Your choice [a]: ", default="a") == "a"

    def test_branch_strategy_picks_main(self):
        """Branch strategy defaults to main; use --base to override."""
        ac = AutoConsole()
        assert ac.prompt("  Your choice [m]: ", default="m") == "m"

    def test_unknown_prompt_returns_default(self):
        ac = AutoConsole()
        assert ac.prompt("Something else: ", default="fallback") == "fallback"

    def test_confirm_returns_default(self):
        ac = AutoConsole()
        assert ac.confirm("Abort?", default=False) is False
        assert ac.confirm("Continue?", default=True) is True

    def test_output_captured_in_log(self):
        ac = AutoConsole()
        ac.print("line one")
        ac.header("CHECKPOINT 1/5 — Changes Review")
        ac.prompt("  Your choice: ")
        assert "line one" in ac.log
        assert "[AUTO" in ac.log

    def test_flush_log_writes_file(self, tmp_path: Path):
        log_file = tmp_path / "test.log"
        ac = AutoConsole(log_path=log_file)
        ac.print("hello auto")
        ac.header("CHECKPOINT 1/5 — Changes Review")
        ac.prompt("  Your choice: ")
        ac.flush_log()
        assert log_file.exists()
        content = log_file.read_text()
        assert "hello auto" in content
        assert "[AUTO" in content

    def test_has_all_console_methods(self):
        ac = AutoConsole()
        for method in ["print", "prompt", "confirm", "pass_", "warn",
                        "fail", "info", "error", "header", "options"]:
            assert hasattr(ac, method), f"AutoConsole missing method: {method}"

    def test_context_switches_between_checkpoints(self):
        """Verify context updates when headers change."""
        ac = AutoConsole()
        ac.header("CHECKPOINT 1/5 — Changes Review")
        assert ac.prompt("  Your choice: ") == "a"
        ac.header("CHECKPOINT 2/5 — Commit Approval")
        assert ac.prompt("  Your choice: ") == "c"
        ac.header("CHECKPOINT 5/5 — Final Summary")
        assert ac.prompt("  Your choice: ") == "d"


class TestConsoleInstantiation:
    """Test that Console can be instantiated (no crashes)."""

    def test_console_has_color_constants(self):
        c = Console()
        assert c.GREEN.startswith("\033[")
        assert c.RESET == "\033[0m"

    def test_console_has_all_methods(self):
        c = Console()
        for method in ["print", "prompt", "confirm", "pass_", "warn",
                        "fail", "info", "error", "header", "options",
                        "role_table"]:
            assert hasattr(c, method), f"Console missing method: {method}"
