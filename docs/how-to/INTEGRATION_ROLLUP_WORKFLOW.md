# Integration → main Rollup Workflow

When a long-lived integration branch (typically an epic with 10+ PRs over multiple weeks) is ready to merge into `main`, follow this 8-phase checklist. The strategy decision behind this workflow is captured in [ADR-015](../adr/ADR-015-integration-branch-rollup-strategy.md).

Tracked end-to-end in a GitHub issue (use a plain issue tagged `tech-debt` + `domain:integration`).

---

## When to use this workflow

Use the full 8-phase rollup when **two or more of** the following apply:

- ≥10 PRs expected against the integration branch
- Shared interfaces or HAL changes (cross-cutting impact)
- Coordinated multi-agent work (parallel epic delivery)
- Duration >2 weeks

For smaller efforts (a few related PRs, <2 weeks), the "Integration Branch Pattern" earlier in [`DEVELOPMENT_WORKFLOW.md`](DEVELOPMENT_WORKFLOW.md) is sufficient — squash the integration branch directly without the themed-review pass.

---

## Why a special process is needed

Individual PRs are reviewed at land-time, but the **combined diff** going to `main` is much larger than any single PR. Risks the standard PR review misses:

- Cross-cutting interactions between features that landed separately
- Documentation drift across PROGRESS.md / ROADMAP.md / API.md / TESTS.md
- Test-baseline drift (TESTS.md counts go stale across many PRs)
- Latent gaps where a per-site fix should have been a wrapper-level fix (see #720/#722 for an example)
- Performance / CPU-usage regression that's invisible per-PR but compounds

The standard `/review-pr` skill is designed for single-PR review (~hundreds of lines). An integration rollup needs a different approach.

---

## Phase 1 — Pre-review cleanup

Before kicking off agent reviews, get the branch into a clean baseline state:

- [ ] **Resolve all failing tests** — `ctest -j$(nproc)` must be fully green at the integration HEAD. A "fix" means either correcting the code-under-test or correcting the test itself with a documented rationale; **simply leaving a failing test in place is not acceptable**. Tests that are intentionally inactive must be marked `GTEST_SKIP()` (counts as passed) or commented out with an explanation, not left as `FAILED`.
- [ ] **Refresh `docs/tracking/PROGRESS.md`** with improvement entries for every PR landed since the last main merge.
- [ ] **Refresh `docs/tracking/ROADMAP.md`** — mark issues done (strikethrough + checkmark), update metrics table.
- [ ] **Refresh `docs/design/API.md`** if any IPC types or HAL interfaces changed.
- [ ] **Refresh `tests/TESTS.md`** — update test counts + add entries for new test files.
- [ ] **Verify `bash deploy/run_ci_local.sh` clean** (build + format + tests + sanitizers).
- [ ] **Triage all open follow-up issues** filed during the integration work — mark which block merge vs which are post-merge.

---

## Phase 2 — Scenario sweep on integration HEAD

Run all Gazebo scenarios (and Cosys-AirSim scenarios if applicable) on the integration branch's HEAD. Capture results in a rollup-scoped tracking doc (e.g. `docs/tracking/INTEGRATION_MERGE_SCENARIO_SWEEP-YYYY-MM-<rollup-name>.md`):

- [ ] Each scenario: PASS / FAIL + key metrics (PX4 denies count, hover-fallback events, mission-complete state)
- [ ] **2–3 consecutive PASS runs per scenario before declaring it clean** — single-run PASSes are insufficient given Gazebo physics seeding + EKF2 cold-start variance. Same-binary-opposite-result outcomes are run-to-run noise, not code-level signal. Re-runs at the same commit are the only way to separate the two.
- [ ] Document any scenarios known to be flaky on this machine
- [ ] Compare against the last `main` baseline if available

This is the **integration-test gate** — multi-agent code review cannot catch runtime regressions. If a scenario regressed, it's a P1 blocker.

---

## Phase 3 — Themed multi-agent reviews

Split the diff into ~5–7 thematic chunks of related PRs. Run `/review-pr` (10-agent pipeline + Copilot) on each chunk separately to avoid context-window saturation.

Typical themes:

1. **Benchmark / observability infrastructure** (profilers, dashboards, GT emitters)
2. **HAL / interface layer changes** (new HAL types, refactors)
3. **Feature epics** (one per epic if there were multiple)
4. **Scenario-specific stacks** (e.g. scenario 33 stack)
5. **Review-fix waves** (Pass 1/2 follow-ups from individual PRs)
6. **Safety / fault recovery changes**
7. **Cross-cutting docs / tests / infra**

For each pass, deliverables (follow the standard severity policy defined in [`DEVELOPMENT_WORKFLOW.md` § Review Comment Handling](DEVELOPMENT_WORKFLOW.md#review-comment-handling)):

- [ ] All P1 findings either fixed inline or filed as merge-blockers
- [ ] All P2 findings fixed inline (per the standard "within PR" policy); explicit deferrals require a DR-NNN entry in `docs/tracking/DESIGN_RATIONALE.md`
- [ ] All P3 findings fixed inline OR deferred with a DR-NNN entry in `docs/tracking/DESIGN_RATIONALE.md` — **NOT** filed to `IMPROVEMENTS.md`. Review-flagged items always route to `DESIGN_RATIONALE.md` per the "Critical distinction" rule documented in `DEVELOPMENT_WORKFLOW.md` Step 7. `IMPROVEMENTS.md` is reserved for proactive findings the agent noticed itself, not review comments.
- [ ] Copilot findings overlap with agent findings — deduplicate

> **Note:** Rollup reviews tend to surface more P2/P3 findings than per-PR reviews (larger surface, more cross-cutting context). Use judgement — if a P3 finding is genuinely a "would be nice but won't change correctness" comment, a one-line DR-NNN entry is appropriate. If it's actionable in <15 minutes, fix it inline.

> **Phase 4 and Phase 5 run in parallel — but with a deliberate ordering for Copilot independence.** Open the integration→main PR *first* (per Phase 5 below) so Copilot starts reviewing in the background. Then do Phase 4 fix-finding **from your own agent reviews only** — do **not** look at Copilot's comments while resolving Phase 3 findings. Only *after* Phase 4 is complete should you triage Copilot's comments (see Phase 4b below). This preserves the independence of the two review streams: when Copilot and the themed-agent passes converge on the same finding, that's strong evidence; when they diverge, both perspectives are useful.

---

## Phase 4 — Fix findings (your agent reviews only)

Address all P1 findings before merge. Apply the same severity policy as standard PRs — P2/P3 should be fixed inline; any deferral must be recorded as a DR-NNN entry in `docs/tracking/DESIGN_RATIONALE.md` per the "Critical distinction" rule. Land fixes as small follow-up PRs against the integration branch — keeps the merge-to-main PR's diff stable.

**Do not read Copilot's comments yet.** Work only from the themed-agent passes (Phase 3) and your own proactive notice. This is the rollup's chance to compare independent reviewer streams — peeking now collapses that signal.

---

## Phase 4b — Triage Copilot comments (after Phase 4 is otherwise complete)

Once Phase 4's agent-finding fixes have landed, switch to Copilot's review on the open PR. Same severity policy applies — fix inline or DR-NNN. Two outcomes are interesting to record on the rollup tracking issue:

- **Overlap** (Copilot found the same issue your agents found): high confidence — typically a real bug.
- **Divergence** (Copilot raises a concern your agents didn't, or vice versa): a different perspective — usually a code-quality / test-coverage / doc-staleness item that the themed agents underweighted.

Note the overlap/divergence ratio in the Phase 4 checkpoint comment on the rollup tracking issue — it's useful calibration data for tuning the themed-review prompts in future rollups.

---

## Phase 5 — Open the integration→main PR

**Open this PR as soon as Phase 3 completes (i.e. before starting Phase 4 fix-finding).** The PR's diff updates automatically as Phase 4 commits land on the integration branch, so Copilot reviews the evolving rollup in parallel with the human/agent fix-finding. This is a meaningful wall-time savings on a multi-day rollup.

- Title: `feat: merge feature/<name> into main (<duration> of work)`
- Body must include:
  - Summary of themes (link to the tracking issue)
  - Link to the scenario-sweep results doc
  - List of every merged PR with one-line description
  - Link to the changes-since-main doc (e.g. `tasks/INTEGRATION_BRANCH_CHANGES_SINCE_MAIN.md`)
  - Any DR-NNN entries written during the rollup
  - A `## Known limitations` section listing any P1 findings deliberately deferred post-merge (with linked issues) — these should be exceptional and explicitly green-lit by the maintainer at the Phase 2 or Phase 4 checkpoint

---

## Phase 5b — Agent re-review on the post-fix diff (before Phase 6)

Once Phase 4 (agent findings) and Phase 4b (Copilot triage) have both landed their fixes onto the integration branch, **re-run the themed multi-agent reviews from Phase 3 one more time** against the now-updated commit range. The point isn't to re-do every finding from scratch — it's to verify that:

1. Each P1 fix actually addresses the original concern without introducing a new one (sanity check on the patches that landed).
2. Any P2/P3 deferrals are now reflected in `docs/tracking/DESIGN_RATIONALE.md` as DR-NNN entries (the routing rule actually held).
3. No new P1 surfaces appeared in the Phase 4 fix-commits themselves — easy to introduce a regression when refactoring under pressure.

Document the re-review's findings as a brief comment on the rollup tracking issue. If a new P1 surfaces, return to Phase 4 — do **not** proceed to Phase 6 until the re-review is clean. Phase 6 runs Gazebo + Cosys scenarios which take real wall-clock time; you want the re-review's blessing before paying that cost.

---

## Phase 6 — Final pre-merge validation

By this point Phase 4 / 4b / 5b have all landed fix-commits onto the integration branch, and a `main` merge-back may have brought additional changes in (resolve those first if any). Verify the post-everything state holds:

- [ ] Re-run scenario sweep on the PR branch with main merged-in — catches surface regressions from any merge conflicts. Apply the same 2–3-consecutive-passes discipline as Phase 2 (re-runs are the only way to distinguish code regressions from Gazebo/EKF2 run-to-run noise).
- [ ] Run `bash deploy/run_ci_local.sh` **(full, not `--quick`)** locally. Phase 1 typically uses `--quick` for fast iteration during cleanup; this final pass should run the full matrix (FMT + Build + ctest + ASan + TSan + UBSan + Coverage) so that any sanitizer-detected issue introduced by a Phase 4 fix-commit surfaces before the PR is merged, not after. GitHub Actions CI covers the same sanitizer matrix in `ci.yml`, so this is defense-in-depth — but a local run also catches problems before the (slower) cloud CI completes.
- [ ] Verify `ctest -j$(nproc)` fully green on the PR branch — should match the test count in `tests/TESTS.md`.
- [ ] **Verify the GitHub Actions CI workflow on the PR is fully green — every job, not just the build matrix.** `ci.yml` runs `format-check`, the `build (default|asan|tsan|ubsan)` sanitizer matrix, and `coverage`; `ci-perception.yml` runs the dedicated perception-check sanity job; the orchestrator-tests workflow (if applicable to the rollup's diff) must also be green. Each is a separate gate. In particular, a red sanitizer job is a P1 blocker even if `ctest` passed locally — sanitizers catch undefined behavior that release-mode tests silently absorb. A red `format-check` or `coverage` job is equally blocking: the bar is *all-green*, not *most-green*.

---

## Phase 7 — Merge decision

Two options exist; the right one depends on the **size of the rollup**:

- **Squash** the integration branch into one commit on main: keeps main linear, loses individual PR history
- **Merge commit** (preserves all N commits): main retains the development arc, useful for `git log` / `git bisect` on per-PR resolution

| Rollup size | Default | Why |
|-------------|---------|-----|
| **Small** (a few related PRs, <2 weeks) | **Squash** | Short-lived integration branches have lower audit-trail value; squash keeps `git log` tidy on `main`. This is what the "Integration Branch Pattern" in `DEVELOPMENT_WORKFLOW.md` uses for short integrations. |
| **Large** (50+ commits / weeks of work, this rollup process) | **Merge commit** | Every PR already has review evidence; squashing erases that audit trail. Future `git bisect` across hundreds of commits needs per-PR resolution. Worth paying the "non-linear main" cost. |

**Default for this checklist (large rollups): merge commit.** Use squash only if a PR landed buggy and got rescued by follow-up commits that you don't want polluting `git log` — in which case, squash that specific PR's portion via interactive rebase before opening the integration-to-main PR, then keep the rest as merge commit.

---

## Phase 8 — Post-merge cleanup

- [ ] Delete the integration branch on origin
- [ ] Remove all integration-branch worktrees
- [ ] Update `tasks/active-work.md` and similar trackers
- [ ] Close the rollup tracking issue with a summary

---

## Cost estimate

Typical integration rollup: **6–12 hours over 2–3 sessions**. Phase 2 (scenario sweep) is often the longest because of Gazebo's slow iteration. Phase 3 (themed reviews) is the most parallelisable.

---

## Worked example

See [Issue #723 — Integration → main merge for `feature/perception-v2-integration`](https://github.com/nmohamaya/companion_software_stack/issues/723) for a fully-tracked rollup. 87 commits over 3 weeks; this checklist applied; PR #729 was the merge.

---

## Related

- [ADR-015](../adr/ADR-015-integration-branch-rollup-strategy.md) — Strategy decision behind this workflow
- [ADR-005](../adr/ADR-005-parallel-agent-git-worktree.md) — Parallel-agent git worktree model
- [ADR-010](../adr/ADR-010-multi-agent-pipeline-architecture.md) — Multi-agent review pipeline
- [`DEVELOPMENT_WORKFLOW.md`](DEVELOPMENT_WORKFLOW.md) — Standard per-PR workflow
