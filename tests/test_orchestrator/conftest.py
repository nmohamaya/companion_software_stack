"""Shared fixtures for orchestrator tests."""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure scripts/ is on the import path for orchestrator package
_scripts_dir = str(Path(__file__).resolve().parent.parent.parent / "scripts")
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

import pytest

from orchestrator.console import TestConsole


@pytest.fixture
def test_io() -> TestConsole:
    """A TestConsole with no pre-loaded inputs."""
    return TestConsole()


@pytest.fixture
def project_dir(tmp_path):
    """A temporary project directory with minimal structure for testing."""
    # Create directories that preflight/context checks look for
    (tmp_path / ".claude" / "agents").mkdir(parents=True)
    (tmp_path / ".claude" / "shared-context").mkdir(parents=True)
    (tmp_path / "tasks").mkdir()
    (tmp_path / "build" / "bin").mkdir(parents=True)
    (tmp_path / "tests").mkdir()
    (tmp_path / "config").mkdir()

    # Create minimal files
    (tmp_path / "tests" / "TESTS.md").write_text("Total: **1259**\n")
    (tmp_path / "tasks" / "active-work.md").write_text("# Active Work\n")
    (tmp_path / "tasks" / "agent-changelog.md").write_text("# Agent Changelog\n")

    return tmp_path
