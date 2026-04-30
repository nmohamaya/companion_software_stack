# Pending In-Depth Reviews

Recent PRs that landed (or are open) without the standard Pass 1 + Pass 2
multi-agent review.  We deferred the reviews to keep the scenario-33
debugging loop tight — the rule was: ship the smallest correct change,
verify in flight, then come back and review.

**Rule:** review these AFTER scenario 33 passes end-to-end.  Once the
mission completes WP1→WP5 cleanly, work through the list below in PR-number
order.

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

## PRs to review

Audit method (2026-04-30): cross-referenced `gh pr view <N> --json comments`
output for the "Automated Two-Pass Review — PR #N" comment that the
deploy-review pipeline posts.  Absence of that comment + no review-fix
commits = unreviewed.

**Confirmed reviewed** (skip these): #639, #640, #641, #642 (each has a
"Review fixes — landed in commit X" comment), #623/#628/#630/#632
(PR #637 was the batch-fix that addressed those four).

**Skipped** (don't need full review): #634 (merge consolidation, no new
code — just stacked-PR rebase on integration).

### Merged (need post-merge review on integration branch)

| PR | Title | Routing hints | Status |
| --- | --- | --- | --- |
| #631 | `fix(#626): wire use_cuda through DA V2 with canary-inference fallback` | CUDA wiring + canary-fallback mechanism. Pass 1 = memory (CUDA buffer lifetimes) + fault-recovery (fallback path correctness) + security (no command injection on canary inputs) + unit; Pass 2 = all four | merged 2026-04-24 |
| #633 | `feat(#626): wire FastSAM CUDA backend (canary-guarded, same pattern as DA V2)` | mirror of #631 for FastSAM; review same axes. Pass 1 + Pass 2 routing identical to #631 | merged 2026-04-24 |
| #636 | `fix(#635): PATH A voxels flow through dynamic-TTL bucket before promotion` | **flight-critical (P4 occupancy grid promotion logic)** — superseded by #642's instance gate but the dynamic-TTL bucket plumbing remains in production paths. Pass 1 = memory + concurrency + fault-recovery + security + unit + scenario (P4 grid behaviour); Pass 2 = all four. Highest priority of the merged-unreviewed set. | merged 2026-04-27 |
| #646 | `fix(#645): ObstacleAvoider3D post-correction toward-obstacle hard clamp` | flight-critical (P4 planner tick), no concurrency primitives changed; Pass 1 = memory + security + unit + fault-recovery (P4 path), Pass 2 = all four | merged 2026-04-30 |
| #647 | `feat(#645): surface PATH A voxel-tracked obstacles to ObstacleAvoider3D` | new lock-free TripleBuffer between threads, new IPC payload type, P2 fusion publish path; Pass 1 = memory + concurrency + security + unit + scenario, Pass 2 = all four | merged 2026-04-30 |

### Open (review before merge)

| PR | Title | Routing hints | Status |
| --- | --- | --- | --- |
| #643 | `fix(#638): tracker tuning + diagnostics for scenario-33 ID-explosion` | tracker config changes + new diagnostic accessors, no flight-critical structural changes | open, 1 comment, 0 reviews |
| #644 | `fix: ZenohSubscriber latency tracker accumulates phantom samples on quiet topics` | header-only IPC primitive change, adds one `std::atomic<uint64_t>`; concurrency review must verify the relaxed-ordering choice; performance review on hot-path receive() | open, 0 reviews |
| #649 | `fix(#645): UKF radar-only S-matrix recovery (eigenvalue clamp + R-only fallback)` | UKF math change on flight-critical fusion thread. Pass 1 = memory + concurrency + fault-recovery (degraded-input path) + security + unit; Pass 2 = all four. Adds `<Eigen/Eigenvalues>` include — eigensolver runs only on failure path. | merged 2026-04-30 (move to merged section on next sweep) |
| #650 | `fix(#645): correct misleading planner-fallback message + add hover-fallback counter` | log-text + diagnostic-counter only, no behavioural change to the planner. Pass 1 = unit + fault-recovery (verify counter semantics on real failure paths); Pass 2 = test-quality (false-positive guard test) + api-contract (new IGridPlanner method) + code-quality (legacy flag-name retained). | open, 0 reviews |

## Concrete review focus areas

Things I noticed while writing the code that reviewers should poke at:

### #646 (avoider final clamp)

- Is `nearest_dx/dy/dz` always safe to normalise inside the clamp block?  Gated on `min_active_dist > kMinDistGateM`, but verify the bounds.
- The clamp uses the *nearest* obstacle's direction.  In a multi-obstacle scenario the second-nearest could still have positive v_toward — is single-axis clamping sufficient, or should we clamp against multiple?  (Doc was: "single-nearest is enough as a safety floor; deflection is a separate follow-up.")
- `final_clamp_count_` is read by `final_clamp_count()` from a different thread (logging).  No memory ordering guarantee.  Probably fine because diagnostic, but flag it.
- Existing test `BrakeInCloseRegime_DisabledLeavesForwardMotion` was split — review-test-quality should verify the split doesn't reduce coverage.

### #647 (voxel obstacle surface)

- `kVoxelTrackIdPrefix=0xC0000000`.  Camera/radar tracks are sequential ints.  If a fusion track grows past 0x3FFFFFFF (≈1 B IDs) it would collide.  Realistic?  Document or assert.
- `TripleBuffer<VoxelObstacleSnapshot>` payload is 64 entries × ~64 bytes ≈ 4 KB × 3 slots = 12 KB stack.  Acceptable but reviewers should verify against the producer's stack budget.
- `voxel_obstacle_min_observations` and `max_age_ms` use ad-hoc string keys (`"perception.path_a.avoider_surface.min_observations"`).  Should move to `cfg_key.h` per the modularity epic convention.
- The `last_seen_ns` written by `VoxelInstanceTracker::update()` is steady-clock; the consumer compares against `steady_clock::now()`.  Consistent, but reviewers should verify there's no path where `now_ns()` has a different epoch.
- Synthesised `velocity = 0` assumes static.  Voxel tracker doesn't compute velocity.  If a moving cube ever shows up, the avoider's predictive look-ahead won't help.  Document as a known limitation or follow-up.

### #644 (zenoh latency)

- `last_recorded_ts_` uses relaxed memory ordering.  Single reader (the consumer thread), but the comment in the source claims "single-reader, single-writer per subscriber by contract" — verify that contract holds across all callers in the codebase.
- The fix changes the semantic of `LatencyTracker` samples in a way other consumers might not expect.  Are there other call sites that rely on the old "every receive records a sample" behaviour for non-latency purposes (e.g. counting receive calls)?

### #643 (tracker tuning)

- `max_match_distance_m` bumped 3.0 → 8.0.  Justify against expected per-frame motion vs distinct-pillar separation.  Already in PR body but a reviewer should sanity-check.
- New `last_aged_out_count()` and `last_match_failure_count()` accessors are written by `update()`, read by `main.cpp`.  Single thread, no atomics needed, but verify.

### #650 (planner hover-fallback observability)

- The legacy `direct_fallback_` flag and `using_direct_fallback()` method were intentionally kept (the rename has wide blast radius).  Code-quality reviewer should weigh in on whether the deferred rename is worth a follow-up issue.
- New `hover_fallback_count_` is only incremented in the no-cached-path branch — verify that's the right semantic vs. "any plan() call where search failed regardless of cache state".
- `noexcept` on `hover_fallback_count()` accessor — confirm the return path can never throw (it's a plain `uint64_t` member read, should be safe).
- The WARN message now does string concatenation via `std::to_string` on a hot-ish path (every plan() that hits the fallback).  Per-call cost negligible; just flag for perf reviewer to confirm.

### #649 (UKF radar S-matrix recovery)

- Eigenvalue floor `1e-3 · mean(R.diagonal())`: relative to R rather than absolute, so it tracks the radar config.  Verify it's not so tight that the recovered S is essentially R, nor so loose that it admits implausible associations.  Compare to the gate threshold `χ²(4) at 95% = 9.21` to bound worst-case false-acceptance.
- `0.5 * (S + S.transpose().eval())` symmetrize: `.eval()` allocates a 4×4 temporary on each call.  Failure-path-only so likely fine, but flag for a perf reviewer to confirm against the budget on the radar-only branch.
- R-only fallback uses `z_pred` (refreshed via `ukf.predicted_radar_measurement()`).  Originally the function set `z_pred` once at the top; with the fallback flipping `use_full_S = false` mid-block, we re-call to get a fresh value.  Verify there's no path where `z_pred` is read stale.
- New `s_matrix_fallback_count_` is read from the engine externally (run-report); no concurrency annotation.  Single-thread (fusion) by IPC contract — confirm.
- The previous WARN log was downgraded to DEBUG.  This means scenario runs that exhibit the failure won't surface a noisy log line.  The `s_matrix_fallback_count()` accessor is the canonical signal — make sure run-report or scenario gating reads it (currently doesn't).

## Workflow for catch-up reviews

1. After scenario 33 passes end-to-end, switch to integration branch, pull.
2. For each open PR (#643, #644): run `deploy-review <PR#>`, address findings, merge.
3. For each merged PR (#646, #647): create a follow-up branch off integration, address review findings as a fix-up PR (`fix: address #646 review` etc.).  Don't try to retroactively review the merged commit — the scope is small enough that fix-up PRs are cleaner.
4. Once all four are reviewed + clean, close this file by moving the entries to `tasks/agent-changelog.md` with the review-PR references.
