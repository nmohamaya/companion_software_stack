"""Single source of truth for agent roles, models, and pipeline configuration.

Replaces the role-model mappings previously duplicated in:
  - scripts/start-agent.sh (lines 18-48)
  - scripts/deploy-issue.sh (lines 464-472)
  - scripts/deploy-review.sh (lines 213-231)
  - scripts/run-session.sh (lines 214-218)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path


class ModelTier(Enum):
    """Agent model tier — determines cost and capability."""

    OPUS = "opus"
    SONNET = "sonnet"
    HAIKU = "haiku"


@dataclass(frozen=True)
class RoleConfig:
    """Configuration for a single agent role."""

    name: str
    model: str
    tier: ModelTier
    read_only: bool = False  # review agents are read-only


# ── Agent Roster ───────────────────────────────────────────────────────────
# 13 roles across 4 categories: feature (5), review (4), test (2), ops (1), lead (1)

ROLES: dict[str, RoleConfig] = {
    # Orchestration
    "tech-lead": RoleConfig(
        "tech-lead", "claude-opus-4-6", ModelTier.OPUS
    ),
    # Feature agents (read-write)
    "feature-perception": RoleConfig(
        "feature-perception", "claude-opus-4-6", ModelTier.OPUS
    ),
    "feature-nav": RoleConfig(
        "feature-nav", "claude-opus-4-6", ModelTier.OPUS
    ),
    "feature-integration": RoleConfig(
        "feature-integration", "claude-opus-4-6", ModelTier.OPUS
    ),
    "feature-infra-core": RoleConfig(
        "feature-infra-core", "claude-opus-4-6", ModelTier.OPUS
    ),
    "feature-infra-platform": RoleConfig(
        "feature-infra-platform", "claude-opus-4-6", ModelTier.OPUS
    ),
    # Review agents (read-only)
    "review-memory-safety": RoleConfig(
        "review-memory-safety", "claude-opus-4-6", ModelTier.OPUS, read_only=True
    ),
    "review-concurrency": RoleConfig(
        "review-concurrency", "claude-opus-4-6", ModelTier.OPUS, read_only=True
    ),
    "review-fault-recovery": RoleConfig(
        "review-fault-recovery", "claude-opus-4-6", ModelTier.OPUS, read_only=True
    ),
    "review-security": RoleConfig(
        "review-security", "claude-opus-4-6", ModelTier.OPUS, read_only=True
    ),
    # Test agents
    "test-unit": RoleConfig(
        "test-unit", "claude-sonnet-4-6", ModelTier.SONNET
    ),
    "test-scenario": RoleConfig(
        "test-scenario", "claude-sonnet-4-6", ModelTier.SONNET
    ),
    # Ops
    "ops-github": RoleConfig(
        "ops-github", "claude-haiku-4-5-20251001", ModelTier.HAIKU
    ),
}

ALL_ROLES: list[str] = list(ROLES.keys())

PIPELINE_VERSION = "multi-agent-pipeline v2.0"

# ── Role Categories ────────────────────────────────────────────────────────

FEATURE_ROLES = [r for r in ALL_ROLES if r.startswith("feature-")]
REVIEW_ROLES = [r for r in ALL_ROLES if r.startswith("review-")]
TEST_ROLES = [r for r in ALL_ROLES if r.startswith("test-")]

# ── Boundary Patterns ──────────────────────────────────────────────────────
# File patterns each role is allowed to touch (from check-agent-boundaries.sh).
# Patterns ending with / are directory prefixes; patterns with * use fnmatch.

ALWAYS_ALLOWED_PATTERNS: list[str] = [
    "docs/",
    "tasks/",
    "CLAUDE.md",
    "tests/TESTS.md",
    "tests/CMakeLists.txt",
]

ROLE_BOUNDARY_PATTERNS: dict[str, list[str]] = {
    "feature-perception": [
        "process1_video_capture/",
        "process2_perception/",
        "common/hal/include/hal/*camera*",
        "common/hal/include/hal/*detector*",
        "tests/test_*perception*",
        "tests/test_*camera*",
        "tests/test_*detector*",
        "tests/test_*tracker*",
        "tests/test_*fusion*",
    ],
    "feature-nav": [
        "process3_slam_vio_nav/",
        "process4_mission_planner/",
        "common/hal/include/hal/*planner*",
        "common/hal/include/hal/*avoider*",
        "tests/test_*mission*",
        "tests/test_*slam*",
        "tests/test_*nav*",
        "tests/test_*planner*",
    ],
    "feature-integration": [
        "process5_comms/",
        "process6_payload_manager/",
        "process7_system_monitor/",
        "common/ipc/",
        "common/hal/include/hal/*fc_link*",
        "common/hal/include/hal/*gcs*",
        "common/hal/include/hal/*gimbal*",
        "common/hal/include/hal/*imu*",
        "tests/test_*comms*",
        "tests/test_*ipc*",
        "tests/test_*hal*",
        "tests/test_*monitor*",
    ],
    "feature-infra-core": [
        "common/util/",
        "common/recorder/",
        "CMakeLists.txt",
        "*/CMakeLists.txt",
        "config/default.json",
        "tests/test_*util*",
        "tests/test_*config*",
        "tests/test_*replay*",
    ],
    "feature-infra-platform": [
        "deploy/",
        "scripts/",
        ".github/",
        "boards/",
        "config/customers/",
    ],
}

# ── Label → Role Routing ──────────────────────────────────────────────────
# Priority-based: higher number wins. From deploy-issue.sh lines 134-154.

LABEL_ROUTING: list[tuple[list[str], str, int]] = [
    # (labels, domain, priority)
    # Priority 3: specific domains
    (["perception", "domain:perception"], "perception", 3),
    (["nav-planning", "domain:nav"], "nav", 3),
    (["comms", "domain:comms"], "integration", 3),
    (["ipc"], "infra-core", 3),
    # Priority 2: mid-level
    (["integration", "domain:integration"], "integration", 2),
    (
        ["common", "infra", "infrastructure", "modularity", "domain:infra-core"],
        "infra-core",
        2,
    ),
    # Priority 1: broad
    (
        ["platform", "deploy", "ci", "bsp", "domain:infra-platform"],
        "infra-platform",
        1,
    ),
]

# Direct role overrides (bypass domain routing)
LABEL_DIRECT_ROLES: dict[str, str] = {
    "safety-audit": "review-memory-safety",
    "security-audit": "review-security",
    "test-coverage": "test-unit",
    "cross-domain": "tech-lead",
}

# ── Review Agent Routing Patterns ──────────────────────────────────────────
# Regex patterns in diff that trigger additional reviewers.

REVIEW_CONCURRENCY_PATTERNS = [
    r"atomic",
    r"mutex",
    r"thread",
    r"lock_guard",
    r"condition_variable",
    r"memory_order",
]

REVIEW_FAULT_RECOVERY_PATTERNS = [
    r"process4_",
    r"process5_",
    r"process7_",
    r"watchdog",
    r"fault",
    r"recovery",
]

REVIEW_SCENARIO_PATTERNS = [
    r"common/ipc",
    r"common/hal",
    r"config/scenarios",
    r"gazebo",
]


def get_role(name: str) -> RoleConfig:
    """Look up a role by name. Raises KeyError if not found."""
    return ROLES[name]


def resolve_project_dir() -> Path:
    """Resolve the project root directory (parent of scripts/)."""
    return Path(__file__).resolve().parent.parent.parent
