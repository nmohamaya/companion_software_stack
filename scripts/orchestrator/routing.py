"""Label-to-role triage and diff-based review agent routing.

Replaces:
  - deploy-issue.sh label scanning (lines 134-231)
  - deploy-review.sh diff-based routing (lines 73-131)

Two main functions:
  - triage_issue_labels(): maps GitHub issue labels to an agent role
  - route_review_agents(): determines which reviewers to launch based on PR diff
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from orchestrator.config import (
    LABEL_DIRECT_ROLES,
    LABEL_ROUTING,
    REVIEW_CONCURRENCY_PATTERNS,
    REVIEW_FAULT_RECOVERY_PATTERNS,
    REVIEW_SCENARIO_PATTERNS,
    ROLES,
)


@dataclass
class TriageResult:
    """Result of issue label triage."""

    role: str | None = None
    is_bug: bool = False
    is_refactor: bool = False
    is_perf: bool = False
    domain_label: str = ""
    domain_priority: int = 0


def triage_issue_labels(labels: list[str]) -> TriageResult:
    """Route an issue to an agent role based on its GitHub labels.

    Priority system (higher wins):
      - P3: specific domains (perception, nav-planning, comms, ipc)
      - P2: mid-level (integration, common/infra)
      - P1: broad (platform, deploy, ci)

    Direct overrides for audit/test/cross-domain labels bypass priority.
    """
    result = TriageResult()

    for label in labels:
        # Check type labels
        if label == "bug":
            result.is_bug = True
        elif label == "refactor":
            result.is_refactor = True
        elif label == "performance":
            result.is_perf = True

        # Check direct role overrides
        if label in LABEL_DIRECT_ROLES:
            result.role = LABEL_DIRECT_ROLES[label]

        # Check domain routing (priority-based, highest wins)
        for match_labels, domain, priority in LABEL_ROUTING:
            if label in match_labels and priority > result.domain_priority:
                result.domain_label = domain
                result.domain_priority = priority

    # Resolve role from domain if not directly set
    if not result.role:
        if result.domain_label:
            result.role = f"feature-{result.domain_label}"
        elif result.is_refactor or result.is_perf:
            # Refactor/performance needs a domain label
            result.role = None  # caller should handle this

    return result


def role_for_branch(branch: str) -> str | None:
    """Detect agent role from branch name pattern.

    Used by check-agent-boundaries.sh to determine role from branch.
    """
    patterns: list[tuple[str, str]] = [
        (r"perception", "feature-perception"),
        (r"nav|mission|slam", "feature-nav"),
        (r"comms|integration|monitor", "feature-integration"),
        (r"infra|common|modularity", "feature-infra-core"),
        (r"platform|deploy|ci-|cert", "feature-infra-platform"),
    ]
    for pattern, role in patterns:
        if re.search(pattern, branch):
            return role
    return None


def branch_prefix_for_issue(title: str, is_bug: bool) -> str:
    """Generate a branch name prefix from issue title and type."""
    slug = re.sub(r"[^a-z0-9-]", "", title.lower().replace(" ", "-"))[:40]
    slug = slug.rstrip("-")
    prefix = "fix" if is_bug else "feature"
    return prefix


def branch_name_for_issue(number: int, title: str, is_bug: bool) -> str:
    """Generate a full branch name for an issue."""
    slug = re.sub(r"[^a-z0-9-]", "", title.lower().replace(" ", "-"))[:40]
    slug = slug.rstrip("-")
    prefix = "fix" if is_bug else "feature"
    return f"{prefix}/issue-{number}-{slug}"


@dataclass
class ReviewRouting:
    """Result of review agent routing decision.

    Two-pass architecture:
      Pass 1 (safety & correctness): reviewers + testers — run in parallel
      Pass 2 (quality & contracts): pass2_reviewers — run after Pass 1,
        receive Pass 1 findings as context
    """

    reviewers: list[str] = field(default_factory=list)
    testers: list[str] = field(default_factory=list)
    pass2_reviewers: list[str] = field(default_factory=list)
    reasons: dict[str, str] = field(default_factory=dict)


# Pass 2 agents always run (they need Pass 1 findings as input)
PASS2_REVIEW_AGENTS: list[str] = [
    "review-test-quality",
    "review-api-contract",
    "review-code-quality",
    "review-performance",
]


def route_review_agents(diff: str, force_all: bool = False) -> ReviewRouting:
    """Determine which review and test agents to launch based on PR diff.

    Pass 1 — Safety & Correctness (parallel):
      Always: review-memory-safety, review-security, test-unit.
      Conditional:
        - review-concurrency: if diff contains atomic/mutex/thread patterns
        - review-fault-recovery: if diff touches P4/P5/P7 or watchdog/fault code
        - test-scenario: if diff touches IPC/HAL/Gazebo configs

    Pass 2 — Quality & Contracts (parallel, after Pass 1):
      Always: review-test-quality, review-api-contract, review-code-quality,
              review-performance.
      These agents receive Pass 1 findings as context.
    """
    routing = ReviewRouting()

    # ── Pass 1: Always-on reviewers ───────────────────────────────────────
    routing.reviewers = ["review-memory-safety", "review-security"]
    routing.reasons["review-memory-safety"] = "always (pass 1)"
    routing.reasons["review-security"] = "always (pass 1)"

    # Pass 1: Always-on testers
    routing.testers = ["test-unit"]
    routing.reasons["test-unit"] = "always (pass 1)"

    if force_all:
        routing.reviewers.extend(["review-concurrency", "review-fault-recovery"])
        routing.testers.append("test-scenario")
        routing.reasons["review-concurrency"] = "forced (--all, pass 1)"
        routing.reasons["review-fault-recovery"] = "forced (--all, pass 1)"
        routing.reasons["test-scenario"] = "forced (--all, pass 1)"
    else:
        # Conditional: concurrency
        for pattern in REVIEW_CONCURRENCY_PATTERNS:
            if re.search(pattern, diff):
                if "review-concurrency" not in routing.reviewers:
                    routing.reviewers.append("review-concurrency")
                    routing.reasons["review-concurrency"] = (
                        f"diff contains '{pattern}' pattern (pass 1)"
                    )
                break

        # Conditional: fault recovery
        for pattern in REVIEW_FAULT_RECOVERY_PATTERNS:
            if re.search(pattern, diff):
                if "review-fault-recovery" not in routing.reviewers:
                    routing.reviewers.append("review-fault-recovery")
                    routing.reasons["review-fault-recovery"] = (
                        f"diff touches '{pattern}' pattern (pass 1)"
                    )
                break

        # Conditional: scenario tests
        for pattern in REVIEW_SCENARIO_PATTERNS:
            if re.search(pattern, diff):
                if "test-scenario" not in routing.testers:
                    routing.testers.append("test-scenario")
                    routing.reasons["test-scenario"] = (
                        f"diff touches '{pattern}' pattern (pass 1)"
                    )
                break

    # ── Pass 2: Quality & Contracts (always-on) ──────────────────────────
    routing.pass2_reviewers = list(PASS2_REVIEW_AGENTS)
    for agent in PASS2_REVIEW_AGENTS:
        routing.reasons[agent] = "always (pass 2, receives pass 1 findings)"

    return routing
