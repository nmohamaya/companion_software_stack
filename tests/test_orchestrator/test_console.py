"""Tests for orchestrator.console — Console and TestConsole I/O."""

from __future__ import annotations

from orchestrator.console import Console, TestConsole


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
