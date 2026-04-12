"""Tests for orchestrator.preflight — preflight checks."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from orchestrator.console import TestConsole
from orchestrator.preflight import (
    CheckResult,
    CheckStatus,
    check_agent_file,
    check_build_dir,
    check_branch,
    check_git_repo,
    check_uncommitted,
)


class TestCheckAgentFile:
    """Test agent file existence check."""

    def test_exists(self, project_dir):
        (project_dir / ".claude" / "agents" / "feature-nav.md").write_text("# Nav")
        result = check_agent_file(project_dir, "feature-nav")
        assert result.status == CheckStatus.PASS

    def test_missing(self, project_dir):
        result = check_agent_file(project_dir, "nonexistent")
        assert result.status == CheckStatus.FAIL


class TestCheckGitRepo:
    """Test git repo check."""

    def test_valid_repo(self):
        git = MagicMock()
        git.is_repo.return_value = True
        result = check_git_repo(git)
        assert result.status == CheckStatus.PASS

    def test_not_a_repo(self):
        git = MagicMock()
        git.is_repo.return_value = False
        result = check_git_repo(git)
        assert result.status == CheckStatus.FAIL


class TestCheckBranch:
    """Test branch name check."""

    def test_reports_branch(self):
        git = MagicMock()
        git.current_branch.return_value = "feature/issue-42"
        result = check_branch(git)
        assert result.status == CheckStatus.INFO
        assert "feature/issue-42" in result.message


class TestCheckUncommitted:
    """Test uncommitted changes check."""

    def test_clean(self):
        git = MagicMock()
        git.has_uncommitted_changes.return_value = False
        result = check_uncommitted(git)
        assert result.status == CheckStatus.PASS

    def test_dirty(self):
        git = MagicMock()
        git.has_uncommitted_changes.return_value = True
        git.uncommitted_count.return_value = 3
        result = check_uncommitted(git)
        assert result.status == CheckStatus.WARN
        assert "3" in result.message


class TestCheckBuildDir:
    """Test build directory check."""

    def test_exists(self, project_dir):
        result = check_build_dir(project_dir)
        assert result.status == CheckStatus.PASS

    def test_missing(self, tmp_path):
        result = check_build_dir(tmp_path)
        assert result.status == CheckStatus.WARN
