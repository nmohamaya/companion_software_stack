# Pending In-Depth Reviews

Recent PRs that landed (or are open) without the standard Pass 1 + Pass 2
multi-agent review.  The original deferral rule was: ship the smallest
correct change to unblock scenario 33, verify in flight, then come back
and review.

**Status (2026-04-30 evening):** the #645 scenario-33 fix series got
post-merge orchestrator reviews completed today, and review-fix PRs
(#664–#675) landed on integration.  Three older pre-#645 merged PRs still
need review.  Two open PRs (#643, #644) were closed/superseded today —
forward-port for #644 landed as #674; #643 was closed with deferred items
captured in `docs/tracking/IMPROVEMENTS.md` (PR #675).

## Review roster (per CLAUDE.md ADR-010)

For each PR, run:

**Pass 1 — Safety & Correctness** (parallel):
- `review-memory-safety`
- `review-concurrency` (if any `std::atomic`, `mutex`, or `thread`)
- `review-fault-recovery` (if P4/P5/P7, watchdog, or fault handling)
- `review-security`
- `test-unit` (any new or modified tests; verify they exercise new code paths)
- `test-scenario` (if IPC/HAL/Gazebo configs touched)

**Pass 2 — Quality & Contracts** (parallel, with Pass 1 findings as context):
- `review-test-quality`
- `review-api-contract`
- `review-code-quality`
- `review-performance`

Use `python3 -m orchestrator deploy-review <PR#>` to auto-route.

## PRs still needing review

Audit method (2026-04-30): cross-referenced `gh pr view <N> --json comments`
output for the "Automated Review" / "Automated Two-Pass Review" comment
pattern that the deploy-review pipeline posts.  Absence of that comment
+ no review-fix commits = unreviewed.

| PR | Title | Routing hints | Why deferred |
|---|---|---|---|
| **#631** | `fix(#626): wire use_cuda through DA V2 with canary-inference fallback` | CUDA buffer lifetimes (memory) + fallback path correctness (fault-recovery) + canary-input validation (security) + unit | merged 2026-04-24, never reviewed |
| **#633** | `feat(#626): wire FastSAM CUDA backend (canary-guarded, same pattern as DA V2)` | mirror of #631 routing | merged 2026-04-24, never reviewed |
| **#636** | `fix(#635): PATH A voxels flow through dynamic-TTL bucket before promotion` | **flight-critical (P4 occupancy grid promotion)**.  Pass 1 = memory + concurrency + fault-recovery + security + unit + scenario; Pass 2 = all four.  **Highest priority of the unreviewed set.** | merged 2026-04-27 |

## Reviewed and addressed today (2026-04-30)

All entries below have an "Automated Review …" comment posted by the
orchestrator pipeline AND a fix-mapping table comment from the addressing
review-fix PR.

| Source PR | Review-fix PR(s) addressing findings |
|---|---|
| #646 | #664 (final_clamp atomic, NaN guards) |
| #647 | #664 (trivially-copyable static_assert) |
| #649 | #665 (UKF construction R guard, observability) |
| #650 | #667 (hover_fallback atomic — bundled with batch 4) |
| #651 | #666 (ARM resets trajectory dedup sentinel) |
| #653 / #654 / #655 / #656 | low-impact scenario-config tuning, no code findings |
| #657 | #664 (NaN guards on AABB extents) |
| #658 | #668 (NaN-z guard, atomic counters, ENU docstring) |
| #659 | #668 (bbox NaN/negative, atomic counters) |
| #660 | #669 (voxel_input.min_confidence clamp), #672 (hardcoded path, instance-gate comment) |
| #661 | #667 (P1 SAFETY-CRITICAL landing-pause) + #671 (atomic pause flags) |

## Closed / superseded today (2026-04-30)

| PR | Disposition |
|---|---|
| **#643** | Closed as superseded.  Branched before today's keystone fixes; scenario-33 config diff would have reverted validated values from #660/#658/#659/#642 phase 4.  Three genuinely-new ideas (max_match_distance bump, steady_clock ageing switch, diagnostic counters) captured in `docs/tracking/IMPROVEMENTS.md` (PR #675) for follow-up. |
| **#644** | Superseded by **#674** — same single-commit fix cherry-picked forward onto current integration HEAD (the original branch was stale vs integration after the #645 batch merges).  IMPROVEMENTS.md entry moved to Resolved in #674. |

## Workflow for the three remaining reviews

1. Switch to integration branch, pull.
2. For each merged PR (#631, #633, #636): run `python3 -m orchestrator deploy-review <PR#>`, address any P1/P2 findings as a follow-up fix-up PR (`fix: address #N review findings`) targeting integration.  Don't retroactively review the merged commit.
3. Once all three are clean, archive this file: move into `tasks/agent-changelog.md` with the review-PR references and delete `tasks/pending-reviews.md`.
