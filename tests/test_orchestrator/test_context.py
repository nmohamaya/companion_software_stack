"""Tests for orchestrator.context — cross-agent context gathering."""

from __future__ import annotations

from orchestrator.context import CrossAgentContext, format_context, gather_context


class TestGatherContext:
    """Test context gathering from filesystem."""

    def test_gathers_active_work(self, project_dir):
        (project_dir / "tasks" / "active-work.md").write_text(
            "# Active Work\n\n"
            "## Issue #42 — Add radar fusion\n"
            "- **Branch:** feature/issue-42-radar\n"
            "- **Status:** in-progress\n"
        )
        ctx = gather_context(project_dir)
        assert "Issue #42" in ctx.active_work
        assert "Branch:" in ctx.active_work

    def test_gathers_recent_work(self, project_dir):
        (project_dir / "tasks" / "agent-changelog.md").write_text(
            "# Agent Changelog\n\n"
            "### 2026-04-07 | feature-nav | opus | PR #100\n"
            "- **Issue:** #42\n"
            "- **Status:** completed\n"
        )
        ctx = gather_context(project_dir)
        assert "###" in ctx.recent_work

    def test_gathers_project_status(self, project_dir):
        status_file = project_dir / ".claude" / "shared-context" / "project-status.md"
        status_file.write_text("## Current State\n1259 tests, 7 processes\n")
        ctx = gather_context(project_dir)
        assert "1259 tests" in ctx.project_status

    def test_gathers_domain_knowledge(self, project_dir):
        dk_file = project_dir / ".claude" / "shared-context" / "domain-knowledge.md"
        dk_file.write_text("## Pitfalls\n- Zenoh max 8 parallel sessions\n")
        ctx = gather_context(project_dir)
        assert "Zenoh" in ctx.domain_knowledge

    def test_missing_files_return_empty(self, tmp_path):
        ctx = gather_context(tmp_path)
        assert ctx.active_work == ""
        assert ctx.recent_work == ""
        assert ctx.project_status == ""
        assert ctx.domain_knowledge == ""


class TestFormatContext:
    """Test context formatting for prompt injection."""

    def test_empty_context_returns_empty(self):
        ctx = CrossAgentContext()
        assert format_context(ctx) == ""

    def test_active_work_formatted(self):
        ctx = CrossAgentContext(active_work="## Issue #42\n- Branch: feature/42")
        formatted = format_context(ctx)
        assert "CROSS-AGENT CONTEXT" in formatted
        assert "Active work" in formatted
        assert "Issue #42" in formatted

    def test_all_sections_present(self):
        ctx = CrossAgentContext(
            active_work="work",
            recent_work="recent",
            project_status="status",
            domain_knowledge="knowledge",
        )
        formatted = format_context(ctx)
        assert "Active work" in formatted
        assert "Recently completed" in formatted
        assert "Project status" in formatted
        assert "Domain knowledge" in formatted

    def test_partial_context(self):
        ctx = CrossAgentContext(project_status="status only")
        formatted = format_context(ctx)
        assert "Project status" in formatted
        assert "Active work" not in formatted
