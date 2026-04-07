"""Tests for orchestrator.routing — label triage and review agent routing."""

from __future__ import annotations

import pytest

from orchestrator.routing import (
    ReviewRouting,
    branch_name_for_issue,
    role_for_branch,
    route_review_agents,
    triage_issue_labels,
)


class TestTriageIssueLabels:
    """Test label-to-role routing."""

    def test_perception_label(self):
        result = triage_issue_labels(["perception", "enhancement"])
        assert result.role == "feature-perception"
        assert result.domain_priority == 3

    def test_nav_planning_label(self):
        result = triage_issue_labels(["nav-planning"])
        assert result.role == "feature-nav"

    def test_comms_label(self):
        result = triage_issue_labels(["comms"])
        assert result.role == "feature-integration"

    def test_ipc_label(self):
        result = triage_issue_labels(["ipc"])
        assert result.role == "feature-infra-core"

    def test_common_label_priority_2(self):
        result = triage_issue_labels(["common"])
        assert result.role == "feature-infra-core"
        assert result.domain_priority == 2

    def test_platform_label_priority_1(self):
        result = triage_issue_labels(["platform"])
        assert result.role == "feature-infra-platform"
        assert result.domain_priority == 1

    def test_higher_priority_wins(self):
        # perception (P3) should beat platform (P1)
        result = triage_issue_labels(["platform", "perception"])
        assert result.role == "feature-perception"
        assert result.domain_priority == 3

    def test_direct_override_safety_audit(self):
        result = triage_issue_labels(["safety-audit", "perception"])
        assert result.role == "review-memory-safety"

    def test_direct_override_security_audit(self):
        result = triage_issue_labels(["security-audit"])
        assert result.role == "review-security"

    def test_direct_override_test_coverage(self):
        result = triage_issue_labels(["test-coverage"])
        assert result.role == "test-unit"

    def test_direct_override_cross_domain(self):
        result = triage_issue_labels(["cross-domain"])
        assert result.role == "tech-lead"

    def test_bug_flag(self):
        result = triage_issue_labels(["bug", "perception"])
        assert result.is_bug is True
        assert result.role == "feature-perception"

    def test_refactor_flag(self):
        result = triage_issue_labels(["refactor", "common"])
        assert result.is_refactor is True
        assert result.role == "feature-infra-core"

    def test_refactor_without_domain_returns_none(self):
        result = triage_issue_labels(["refactor"])
        assert result.is_refactor is True
        assert result.role is None  # needs domain label

    def test_no_labels_returns_none(self):
        result = triage_issue_labels([])
        assert result.role is None

    def test_unknown_labels_returns_none(self):
        result = triage_issue_labels(["documentation", "good-first-issue"])
        assert result.role is None

    def test_domain_prefixed_labels(self):
        result = triage_issue_labels(["domain:perception"])
        assert result.role == "feature-perception"

    def test_modularity_label(self):
        result = triage_issue_labels(["modularity"])
        assert result.role == "feature-infra-core"


class TestRoleForBranch:
    """Test branch-name-to-role detection."""

    def test_perception_branch(self):
        assert role_for_branch("feature/issue-42-perception-pipeline") == "feature-perception"

    def test_nav_branch(self):
        assert role_for_branch("feature/issue-10-nav-planner") == "feature-nav"

    def test_mission_branch(self):
        assert role_for_branch("fix/issue-5-mission-fsm-bug") == "feature-nav"

    def test_slam_branch(self):
        assert role_for_branch("feature/slam-vio-update") == "feature-nav"

    def test_comms_branch(self):
        assert role_for_branch("feature/issue-20-comms-link") == "feature-integration"

    def test_infra_branch(self):
        assert role_for_branch("refactor/infra-config-cleanup") == "feature-infra-core"

    def test_deploy_branch(self):
        assert role_for_branch("feature/deploy-systemd-units") == "feature-infra-platform"

    def test_unrecognized_branch(self):
        assert role_for_branch("feature/random-stuff") is None

    def test_main_branch(self):
        assert role_for_branch("main") is None


class TestBranchNameForIssue:
    """Test branch name generation."""

    def test_feature_branch(self):
        name = branch_name_for_issue(42, "Add radar fusion", is_bug=False)
        assert name == "feature/issue-42-add-radar-fusion"

    def test_bug_fix_branch(self):
        name = branch_name_for_issue(10, "Fix null pointer crash", is_bug=True)
        assert name == "fix/issue-10-fix-null-pointer-crash"

    def test_long_title_truncated(self):
        title = "This is a very long issue title that should be truncated to forty chars"
        name = branch_name_for_issue(99, title, is_bug=False)
        # Slug is max 40 chars
        slug = name.split("-", 2)[-1]  # everything after "issue-99-"
        assert len(name.split("/")[1]) <= 60  # reasonable total length

    def test_special_chars_removed(self):
        name = branch_name_for_issue(1, "Fix: crash (on startup) [P1]", is_bug=True)
        assert "(" not in name
        assert ")" not in name
        assert "[" not in name


class TestRouteReviewAgents:
    """Test diff-based review agent routing."""

    def test_always_includes_memory_safety_and_security(self):
        routing = route_review_agents("some diff content")
        assert "review-memory-safety" in routing.reviewers
        assert "review-security" in routing.reviewers

    def test_always_includes_test_unit(self):
        routing = route_review_agents("some diff content")
        assert "test-unit" in routing.testers

    def test_concurrency_triggered_by_atomic(self):
        routing = route_review_agents("+ std::atomic<int> counter;")
        assert "review-concurrency" in routing.reviewers

    def test_concurrency_triggered_by_mutex(self):
        routing = route_review_agents("+ std::mutex m_;")
        assert "review-concurrency" in routing.reviewers

    def test_concurrency_triggered_by_thread(self):
        routing = route_review_agents("+ std::thread worker_;")
        assert "review-concurrency" in routing.reviewers

    def test_fault_recovery_triggered_by_p4(self):
        routing = route_review_agents("diff --git a/process4_mission_planner/src/main.cpp")
        assert "review-fault-recovery" in routing.reviewers

    def test_fault_recovery_triggered_by_watchdog(self):
        routing = route_review_agents("+ ThreadWatchdog watchdog_;")
        assert "review-fault-recovery" in routing.reviewers

    def test_scenario_triggered_by_ipc(self):
        routing = route_review_agents("diff --git a/common/ipc/src/zenoh.cpp")
        assert "test-scenario" in routing.testers

    def test_scenario_triggered_by_gazebo(self):
        routing = route_review_agents("+ gazebo_sitl.json")
        assert "test-scenario" in routing.testers

    def test_no_extra_agents_for_plain_diff(self):
        routing = route_review_agents("+ int x = 5;")
        assert len(routing.reviewers) == 2  # only memory-safety + security
        assert len(routing.testers) == 1  # only test-unit

    def test_force_all(self):
        routing = route_review_agents("+ int x = 5;", force_all=True)
        assert "review-concurrency" in routing.reviewers
        assert "review-fault-recovery" in routing.reviewers
        assert "test-scenario" in routing.testers

    def test_reasons_populated(self):
        routing = route_review_agents("+ std::atomic<bool> flag;")
        assert "review-memory-safety" in routing.reasons
        assert routing.reasons["review-memory-safety"] == "always"
        assert "review-concurrency" in routing.reasons
        assert "atomic" in routing.reasons["review-concurrency"]
