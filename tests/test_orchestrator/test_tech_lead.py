"""Tests for orchestrator.tech_lead — tech-lead strategy layer."""

from __future__ import annotations

import pytest

from orchestrator.tech_lead import (
    EpicPhase,
    EpicPlan,
    ReviewAssessment,
    RoutingDecision,
    _parse_epic_plan,
    _parse_review_assessment,
    _parse_routing_decision,
)


class TestParseRoutingDecision:
    """Test parsing of tech-lead routing responses."""

    def test_parses_full_response(self):
        output = (
            "ROLE: feature-perception\n"
            "PRIORITY: high\n"
            "REASONING: This issue touches the detection pipeline.\n"
            "SEQUENCING: Should be done before issue #100.\n"
            "DEPENDENCIES: #98, #99\n"
            "LABELS_ADD: perception, enhancement\n"
            "LABELS_REMOVE: needs-triage\n"
        )
        default = RoutingDecision(
            issue_number=42,
            issue_title="Test",
            recommended_role="tech-lead",
            recommended_priority="medium",
            reasoning="",
        )
        result = _parse_routing_decision(output, default)
        assert result.recommended_role == "feature-perception"
        assert result.recommended_priority == "high"
        assert "detection pipeline" in result.reasoning
        assert "before issue #100" in result.sequencing_notes
        assert result.dependencies == [98, 99]
        assert result.labels_to_add == ["perception", "enhancement"]
        assert result.labels_to_remove == ["needs-triage"]

    def test_parses_minimal_response(self):
        output = (
            "ROLE: feature-nav\n"
            "PRIORITY: low\n"
            "REASONING: Minor nav fix.\n"
            "SEQUENCING: none\n"
            "DEPENDENCIES: none\n"
            "LABELS_ADD: none\n"
            "LABELS_REMOVE: none\n"
        )
        default = RoutingDecision(
            issue_number=10,
            issue_title="Test",
            recommended_role="tech-lead",
            recommended_priority="medium",
            reasoning="",
        )
        result = _parse_routing_decision(output, default)
        assert result.recommended_role == "feature-nav"
        assert result.recommended_priority == "low"
        assert result.sequencing_notes == ""
        assert result.dependencies == []
        assert result.labels_to_add == []

    def test_invalid_role_keeps_default(self):
        output = "ROLE: nonexistent-role\nPRIORITY: medium\nREASONING: test\n"
        default = RoutingDecision(
            issue_number=1,
            issue_title="Test",
            recommended_role="tech-lead",
            recommended_priority="medium",
            reasoning="",
        )
        result = _parse_routing_decision(output, default)
        assert result.recommended_role == "tech-lead"

    def test_invalid_priority_keeps_default(self):
        output = "ROLE: feature-nav\nPRIORITY: urgent\nREASONING: test\n"
        default = RoutingDecision(
            issue_number=1,
            issue_title="Test",
            recommended_role="tech-lead",
            recommended_priority="medium",
            reasoning="",
        )
        result = _parse_routing_decision(output, default)
        assert result.recommended_priority == "medium"

    def test_hash_prefix_in_dependencies(self):
        output = "DEPENDENCIES: #42, #43, #44\n"
        default = RoutingDecision(
            issue_number=1,
            issue_title="Test",
            recommended_role="tech-lead",
            recommended_priority="medium",
            reasoning="",
        )
        result = _parse_routing_decision(output, default)
        assert result.dependencies == [42, 43, 44]


class TestParseReviewAssessment:
    """Test parsing of tech-lead review responses."""

    def test_parses_accept(self):
        output = (
            "RECOMMENDATION: accept\n"
            "REASONING: Clean implementation, tests pass.\n"
            "CONCERNS: none\n"
            "CHANGE_REQUESTS: none\n"
        )
        default = ReviewAssessment(recommendation="accept", reasoning="")
        result = _parse_review_assessment(output, default)
        assert result.recommendation == "accept"
        assert "Clean implementation" in result.reasoning
        assert result.specific_concerns == []
        assert result.change_requests == []

    def test_parses_changes(self):
        output = (
            "RECOMMENDATION: changes\n"
            "REASONING: Missing error handling in radar path.\n"
            "CONCERNS: no RAII on file handle, missing test for edge case\n"
            "CHANGE_REQUESTS: Add RAII wrapper, Add test for null input\n"
        )
        default = ReviewAssessment(recommendation="accept", reasoning="")
        result = _parse_review_assessment(output, default)
        assert result.recommendation == "changes"
        assert len(result.specific_concerns) == 2
        assert len(result.change_requests) == 2
        assert "RAII" in result.specific_concerns[0]

    def test_parses_reject(self):
        output = (
            "RECOMMENDATION: reject\n"
            "REASONING: Wrong approach — should use factory pattern.\n"
            "CONCERNS: architectural mismatch\n"
            "CHANGE_REQUESTS: Redesign using factory pattern\n"
        )
        default = ReviewAssessment(recommendation="accept", reasoning="")
        result = _parse_review_assessment(output, default)
        assert result.recommendation == "reject"

    def test_invalid_recommendation_keeps_default(self):
        output = "RECOMMENDATION: maybe\nREASONING: unsure\n"
        default = ReviewAssessment(recommendation="accept", reasoning="")
        result = _parse_review_assessment(output, default)
        assert result.recommendation == "accept"


class TestParseEpicPlan:
    """Test parsing of tech-lead epic plan responses."""

    def test_parses_full_plan(self):
        output = (
            "TOTAL_ISSUES: 8\n"
            "ESTIMATED_WAVES: 3\n"
            "PHASE: Foundation\n"
            "  ISSUES: #100, #101\n"
            "  DESCRIPTION: Core infrastructure changes\n"
            "  DEPENDS_ON: none\n"
            "PHASE: Feature\n"
            "  ISSUES: #102, #103, #104\n"
            "  DESCRIPTION: Main feature implementation\n"
            "  DEPENDS_ON: Foundation\n"
            "PHASE: Polish\n"
            "  ISSUES: #105, #106, #107\n"
            "  DESCRIPTION: Testing and documentation\n"
            "  DEPENDS_ON: Feature\n"
        )
        default = EpicPlan(epic_number=99, epic_title="Test Epic")
        result = _parse_epic_plan(output, default)
        assert result.total_issues == 8
        assert result.estimated_waves == 3
        assert len(result.phases) == 3
        assert result.phases[0].name == "Foundation"
        assert result.phases[0].issues == [100, 101]
        assert result.phases[0].depends_on == []
        assert result.phases[1].name == "Feature"
        assert result.phases[1].issues == [102, 103, 104]
        assert result.phases[1].depends_on == ["Foundation"]
        assert result.phases[2].name == "Polish"

    def test_parses_single_phase(self):
        output = (
            "TOTAL_ISSUES: 2\n"
            "ESTIMATED_WAVES: 1\n"
            "PHASE: Quick Fix\n"
            "  ISSUES: #50, #51\n"
            "  DESCRIPTION: Simple fixes\n"
            "  DEPENDS_ON: none\n"
        )
        default = EpicPlan(epic_number=49, epic_title="Test")
        result = _parse_epic_plan(output, default)
        assert len(result.phases) == 1
        assert result.phases[0].issues == [50, 51]

    def test_empty_output_returns_default(self):
        default = EpicPlan(epic_number=1, epic_title="Test")
        result = _parse_epic_plan("", default)
        assert result.phases == []
        assert result.total_issues == 0
