"""Tests for orchestrator.commands.deploy_issue — unified mode behavior."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from orchestrator.commands.deploy_issue import run, _tech_lead_routing


class TestRunModeDispatch:
    """Test that run() dispatches correctly based on flags."""

    @patch("orchestrator.commands.deploy_issue.resolve_project_dir")
    @patch("orchestrator.commands.deploy_issue.gather_context")
    @patch("orchestrator.commands.deploy_issue.format_context")
    def test_headless_skips_pipeline(self, mock_fmt, mock_ctx, mock_dir, tmp_path):
        """Headless mode should call launch_print, not _run_pipeline."""
        mock_dir.return_value = tmp_path

        io = MagicMock()
        git = MagicMock()
        git.branch_exists.return_value = False
        github = MagicMock()
        github.fetch_issue.return_value = MagicMock(
            title="Test", body="Body", labels=["enhancement", "platform"]
        )
        github.issue_milestone.return_value = ""
        claude = MagicMock()

        mock_ctx.return_value = MagicMock()
        mock_fmt.return_value = ""

        run(
            42,
            io=io, git=git, github=github, claude=claude,
            headless=True, skip_tech_lead=True,
        )

        claude.launch_print.assert_called_once()
        # Should NOT have entered the pipeline
        assert not any(
            "Phase 1/5" in str(c) for c in io.header.call_args_list
        )

    @patch("orchestrator.commands.deploy_issue.resolve_project_dir")
    @patch("orchestrator.commands.deploy_issue.gather_context")
    @patch("orchestrator.commands.deploy_issue.format_context")
    def test_dry_run_returns_zero(self, mock_fmt, mock_ctx, mock_dir, tmp_path):
        """Dry-run should return 0 without creating worktree."""
        mock_dir.return_value = tmp_path

        io = MagicMock()
        git = MagicMock()
        github = MagicMock()
        github.fetch_issue.return_value = MagicMock(
            title="Test", body="Body", labels=["enhancement", "perception"]
        )
        github.issue_milestone.return_value = ""
        claude = MagicMock()

        mock_ctx.return_value = MagicMock()
        mock_fmt.return_value = ""

        result = run(
            42,
            io=io, git=git, github=github, claude=claude,
            dry_run=True, skip_tech_lead=True,
        )

        assert result == 0
        git.worktree_add.assert_not_called()


class TestTechLeadRouting:
    """Test _tech_lead_routing helper."""

    def test_accept_uses_tech_lead_role(self):
        """When user accepts, the tech-lead recommended role is used."""
        io = MagicMock()
        io.prompt.return_value = "a"
        claude = MagicMock()
        github = MagicMock()

        decision = MagicMock()
        decision.recommended_role = "feature-nav"
        decision.recommended_priority = "high"
        decision.reasoning = "Navigation issue"
        decision.sequencing_notes = ""
        decision.dependencies = []

        with patch("orchestrator.commands.deploy_issue.TechLead") as MockTL:
            MockTL.return_value.analyze_issue.return_value = decision
            role = _tech_lead_routing(
                io, claude, github, None, 42, auto_role="feature-perception"
            )

        assert role == "feature-nav"

    def test_override_uses_auto_role(self):
        """When user overrides, the auto-triage role is used."""
        io = MagicMock()
        io.prompt.return_value = "o"
        claude = MagicMock()
        github = MagicMock()

        decision = MagicMock()
        decision.recommended_role = "feature-nav"
        decision.recommended_priority = "high"
        decision.reasoning = "Navigation issue"
        decision.sequencing_notes = ""
        decision.dependencies = []

        with patch("orchestrator.commands.deploy_issue.TechLead") as MockTL:
            MockTL.return_value.analyze_issue.return_value = decision
            role = _tech_lead_routing(
                io, claude, github, None, 42, auto_role="feature-perception"
            )

        assert role == "feature-perception"

    def test_custom_role(self):
        """When user picks custom, they enter a role manually."""
        io = MagicMock()
        io.prompt.side_effect = ["c", "test-unit"]
        claude = MagicMock()
        github = MagicMock()

        decision = MagicMock()
        decision.recommended_role = "feature-nav"
        decision.recommended_priority = "high"
        decision.reasoning = "Navigation issue"
        decision.sequencing_notes = ""
        decision.dependencies = []

        with patch("orchestrator.commands.deploy_issue.TechLead") as MockTL:
            MockTL.return_value.analyze_issue.return_value = decision
            role = _tech_lead_routing(
                io, claude, github, None, 42, auto_role="feature-perception"
            )

        assert role == "test-unit"


class TestPipelineStateNewFields:
    """Test that new PipelineStateData fields work correctly."""

    def test_milestone_field(self):
        from orchestrator.pipeline.state import PipelineStateData

        state = PipelineStateData(milestone="v1.0")
        assert state.milestone == "v1.0"

    def test_epic_number_field(self):
        from orchestrator.pipeline.state import PipelineStateData

        state = PipelineStateData(epic_number=263)
        assert state.epic_number == 263

    def test_json_roundtrip_with_new_fields(self, tmp_path):
        from orchestrator.pipeline.state import PipelineStateData

        state = PipelineStateData(
            issue_number=42,
            issue_labels=["enhancement", "perception"],
            milestone="v1.0",
            epic_number=263,
        )
        path = tmp_path / "state.json"
        state.save(path)

        loaded = PipelineStateData.load(path)
        assert loaded.issue_labels == ["enhancement", "perception"]
        assert loaded.milestone == "v1.0"
        assert loaded.epic_number == 263
