"""Tests for orchestrator.config — role lookup, tier mapping, boundaries."""

from __future__ import annotations

import pytest

from orchestrator.config import (
    ALL_ROLES,
    ALWAYS_ALLOWED_PATTERNS,
    FEATURE_ROLES,
    LABEL_DIRECT_ROLES,
    LABEL_ROUTING,
    ModelTier,
    PIPELINE_VERSION,
    REVIEW_ROLES,
    ROLE_BOUNDARY_PATTERNS,
    ROLES,
    TEST_ROLES,
    get_role,
    resolve_project_dir,
)


class TestRoleConfig:
    """Test the role configuration registry."""

    def test_all_17_roles_defined(self):
        assert len(ROLES) == 17

    def test_all_roles_list_matches_dict(self):
        assert ALL_ROLES == list(ROLES.keys())

    def test_get_role_valid(self):
        cfg = get_role("tech-lead")
        assert cfg.name == "tech-lead"
        assert cfg.model == "claude-opus-4-6"
        assert cfg.tier == ModelTier.OPUS

    def test_get_role_invalid_raises(self):
        with pytest.raises(KeyError):
            get_role("nonexistent-role")

    @pytest.mark.parametrize("role", ALL_ROLES)
    def test_every_role_has_model(self, role: str):
        cfg = get_role(role)
        assert cfg.model.startswith("claude-")

    @pytest.mark.parametrize("role", ALL_ROLES)
    def test_every_role_has_valid_tier(self, role: str):
        cfg = get_role(role)
        assert isinstance(cfg.tier, ModelTier)


class TestTierCategories:
    """Test role categorization by tier."""

    def test_opus_roles(self):
        opus = [r for r, c in ROLES.items() if c.tier == ModelTier.OPUS]
        assert len(opus) == 11  # 1 lead + 5 feature + 5 review (pass 1 + test-quality)

    def test_sonnet_roles(self):
        sonnet = [r for r, c in ROLES.items() if c.tier == ModelTier.SONNET]
        assert len(sonnet) == 5  # test-unit, test-scenario, api-contract, code-quality, performance

    def test_haiku_roles(self):
        haiku = [r for r, c in ROLES.items() if c.tier == ModelTier.HAIKU]
        assert len(haiku) == 1  # ops-github

    def test_feature_roles(self):
        assert len(FEATURE_ROLES) == 5
        assert all(r.startswith("feature-") for r in FEATURE_ROLES)

    def test_review_roles(self):
        assert len(REVIEW_ROLES) == 8  # 4 pass 1 + 4 pass 2
        assert all(r.startswith("review-") for r in REVIEW_ROLES)

    def test_test_roles(self):
        assert len(TEST_ROLES) == 2
        assert all(r.startswith("test-") for r in TEST_ROLES)


class TestReadOnlyFlag:
    """Test that review agents are marked read-only."""

    @pytest.mark.parametrize("role", REVIEW_ROLES)
    def test_review_agents_are_read_only(self, role: str):
        assert get_role(role).read_only is True

    @pytest.mark.parametrize("role", FEATURE_ROLES)
    def test_feature_agents_are_not_read_only(self, role: str):
        assert get_role(role).read_only is False


class TestBoundaryPatterns:
    """Test boundary pattern configuration completeness."""

    def test_all_feature_roles_have_boundaries(self):
        for role in FEATURE_ROLES:
            assert role in ROLE_BOUNDARY_PATTERNS, f"{role} missing boundary patterns"

    def test_always_allowed_includes_docs(self):
        assert "docs/" in ALWAYS_ALLOWED_PATTERNS

    def test_always_allowed_includes_tasks(self):
        assert "tasks/" in ALWAYS_ALLOWED_PATTERNS


class TestLabelRouting:
    """Test label routing configuration."""

    def test_routing_priorities_valid(self):
        for labels, domain, priority in LABEL_ROUTING:
            assert priority in (1, 2, 3), f"Invalid priority {priority} for {domain}"
            assert isinstance(labels, list)
            assert len(labels) > 0

    def test_direct_roles_are_valid(self):
        for label, role in LABEL_DIRECT_ROLES.items():
            assert role in ROLES, f"Direct role '{role}' for label '{label}' not in ROLES"


class TestPipelineVersion:
    """Test pipeline version string."""

    def test_version_format(self):
        assert PIPELINE_VERSION.startswith("multi-agent-pipeline v")


class TestResolveProjectDir:
    """Test project directory resolution."""

    def test_resolves_to_existing_path(self):
        project = resolve_project_dir()
        assert project.exists()
        assert (project / "scripts" / "orchestrator").exists()
