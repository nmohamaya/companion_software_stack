# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.
"""Pipeline state persistence — JSON serialization with legacy bash reader.

PipelineStateData holds all mutable state for a pipeline run:
  - Current FSM state
  - Issue/PR metadata
  - Branch and worktree info
  - Checkpoint decisions and timestamps
  - Review findings

Persistence:
  - save() writes JSON
  - load() reads JSON (or legacy bash KEY=VALUE format for migration)
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from orchestrator.pipeline.fsm import PipelineState


@dataclass
class PipelineStateData:
    """All mutable state for a pipeline run."""

    # FSM state
    current_state: str = PipelineState.AGENT_WORK.name

    # Issue metadata
    issue_number: int = 0
    issue_title: str = ""
    issue_labels: list[str] = field(default_factory=list)

    # Routing
    agent_role: str = ""
    model_tier: str = ""

    # Branch / worktree
    branch_name: str = ""
    worktree_path: str = ""
    is_bug: bool = False

    # Issue context for PR linking
    milestone: str = ""
    epic_number: int = 0

    # PR metadata
    pr_number: int = 0
    pr_url: str = ""
    pr_title: str = ""
    pr_body: str = ""

    # Pipeline progress
    pipeline_version: str = ""
    checkpoint_history: list[str] = field(default_factory=list)
    review_findings: list[str] = field(default_factory=list)
    fix_iterations: int = 0

    # Timestamps (ISO format strings)
    started_at: str = ""
    last_checkpoint_at: str = ""

    def get_state(self) -> PipelineState:
        """Get current state as PipelineState enum."""
        return PipelineState[self.current_state]

    def set_state(self, state: PipelineState) -> None:
        """Set current state from PipelineState enum."""
        self.current_state = state.name

    def record_checkpoint(self, action: str) -> None:
        """Record a checkpoint action in history."""
        self.checkpoint_history.append(
            f"{self.current_state}:{action}"
        )

    def save(self, path: Path) -> None:
        """Save state to JSON file."""
        data = asdict(self)
        path.write_text(json.dumps(data, indent=2) + "\n")

    @classmethod
    def load(cls, path: Path) -> PipelineStateData:
        """Load state from JSON or legacy bash format.

        Auto-detects format:
          - JSON: starts with '{' (standard)
          - Bash: KEY=VALUE lines (legacy migration from deploy-issue.sh)
        """
        text = path.read_text().strip()
        if not text:
            return cls()

        if text.startswith("{"):
            return cls._from_json(text)
        return cls._from_bash(text)

    @classmethod
    def _from_json(cls, text: str) -> PipelineStateData:
        """Parse JSON format."""
        data = json.loads(text)
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})

    @classmethod
    def _from_bash(cls, text: str) -> PipelineStateData:
        """Parse legacy bash KEY=VALUE format.

        Maps bash variable names to dataclass fields:
          PIPELINE_STATE → current_state
          ISSUE_NUMBER → issue_number
          ISSUE_TITLE → issue_title
          BRANCH_NAME → branch_name
          WORKTREE_DIR → worktree_path
          AGENT_ROLE → agent_role
          PR_NUMBER → pr_number
          PR_URL → pr_url
          IS_BUG → is_bug (true/false → bool)
        """
        bash_to_field = {
            "PIPELINE_STATE": "current_state",
            "ISSUE_NUMBER": "issue_number",
            "ISSUE_TITLE": "issue_title",
            "BRANCH_NAME": "branch_name",
            "WORKTREE_DIR": "worktree_path",
            "AGENT_ROLE": "agent_role",
            "PR_NUMBER": "pr_number",
            "PR_URL": "pr_url",
            "IS_BUG": "is_bug",
            "PIPELINE_VERSION": "pipeline_version",
        }

        int_fields = {"issue_number", "pr_number", "fix_iterations"}
        bool_fields = {"is_bug"}

        state = cls()
        for line in text.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue

            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip().strip('"').strip("'")

            field_name = bash_to_field.get(key)
            if field_name is None:
                continue

            if field_name in int_fields:
                setattr(state, field_name, int(value) if value else 0)
            elif field_name in bool_fields:
                setattr(state, field_name, value.lower() == "true")
            else:
                setattr(state, field_name, value)

        return state
