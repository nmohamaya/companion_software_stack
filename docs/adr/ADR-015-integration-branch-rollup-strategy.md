# ADR-015: Integration-Branch Rollup Strategy

| Field           | Value                                                                                                                                                                                                       |
|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Status**      | Accepted                                                                                                                                                                                                    |
| **Date**        | 2026-05-13                                                                                                                                                                                                  |
| **Author**      | Team                                                                                                                                                                                                        |
| **Deciders**    | Project leads                                                                                                                                                                                               |
| **Supersedes**  | —                                                                                                                                                                                                           |
| **Related**     | PR #724 (introduced the 8-phase rollup checklist), Issue #723 (first rollup tracked end-to-end — `feature/perception-v2-integration`), [ADR-005](ADR-005-parallel-agent-git-worktree.md), [ADR-010](ADR-010-multi-agent-pipeline-architecture.md), [`docs/how-to/INTEGRATION_ROLLUP_WORKFLOW.md`](../how-to/INTEGRATION_ROLLUP_WORKFLOW.md) |

---

## 1. Context

Most PRs land directly on `main`. That works when a change is bounded (≲400 lines, one concern, single agent or developer). It stops working when a coordinated multi-agent push produces dozens of PRs over weeks that depend on each other — typically a large epic (E5 Perception v2, Epic #284 Modularity, Epic #497 SWVIO).

We have three failure modes when long-running work hits `main` without a coordination layer:

1. **Cross-PR interactions are invisible at per-PR review time.** Each PR passes its own review, but the union of changes introduces latent regressions (a HAL signature change in PR #A breaks a consumer added in PR #B that didn't yet exist when #A was reviewed).
2. **Documentation drift outpaces the per-PR doc updates.** PROGRESS, ROADMAP, API.md, TESTS.md each lag by N PRs by the time we notice.
3. **`main` cannot be both demo-ready and the work surface.** If a user clones at any commit on main during a multi-week epic, the tree should be in a runnable state. Coordinating that across N independent PRs is impossible without an intermediate branch.

The first occurrence (Issue #723, merged via PR #729 — `feature/perception-v2-integration`, 87 commits over 3 weeks) was the forcing function. We piloted a long-lived integration branch + 8-phase rollup checklist (introduced by PR #724). It worked: cross-cutting findings were caught at the rollup stage, not in production; main stayed clean throughout.

This ADR records the strategy so future epics use it by default rather than re-deriving it under pressure.

---

## 2. Decision

For multi-issue features with **two or more of**: (a) ≥10 PRs expected, (b) shared interfaces/HAL changes, (c) coordinated multi-agent work, (d) duration >2 weeks — use a **long-lived integration branch** plus the **8-phase rollup checklist** when merging back to `main`.

**Phases** (full text in [`docs/how-to/INTEGRATION_ROLLUP_WORKFLOW.md`](../how-to/INTEGRATION_ROLLUP_WORKFLOW.md)):

1. Pre-review cleanup (docs, tests, CI baseline)
2. Scenario sweep on integration HEAD (2–3 consecutive passes per scenario)
3. Themed multi-agent reviews (split diff into 5–7 chunks)
4. Fix findings from agent reviews only
4b. Triage Copilot comments (after Phase 4)
5. Open integration→main PR
5b. Agent re-review on post-fix diff
6. Final pre-merge validation (full CI matrix, scenario sweep with main merged in)
7. Merge decision (squash for small, merge-commit for large)
8. Post-merge cleanup (worktrees, trackers, tracking issue)

**Branch naming**: `feature/<epic-slug>-integration` or `integration/epic-<N>-wave-<W>`.

**Tracking**: every rollup gets a GitHub issue (no dedicated template — use a plain issue tagged `tech-debt` + `domain:integration`).

---

## 3. Consequences

### Positive

- **Cross-PR interactions surface at the rollup stage**, not in production. Phase 3 themed reviews specifically look for issues invisible at single-PR scope (signature drift, latent missing wrappers — see #720/#722 for an example caught this way).
- **Main stays demo-ready.** Long-running work never touches main until Phase 7.
- **Documentation drift is bounded.** Phase 1 forces a refresh of PROGRESS / ROADMAP / API.md / TESTS.md before the rollup is even reviewable.
- **Independent reviewer streams.** Phase 4 (agent reviews) and Phase 4b (Copilot) are deliberately sequenced so the streams don't contaminate each other — when they converge it's strong evidence, when they diverge both perspectives are useful.
- **Audit trail.** The rollup tracking issue captures every decision; DR-NNN entries capture every deferred review finding.

### Negative

- **Wall-clock cost.** A rollup is 6–12 hours over 2–3 sessions. The scenario sweep alone (Phase 2) is the long pole because Gazebo iteration is slow.
- **Merge conflict risk grows with branch age.** Mitigated by periodic `main → integration` merge-backs during development. Phase 6 catches anything that slipped through.
- **Cognitive load.** The 8-phase checklist is heavyweight. Using it for a 3-PR / 1-week feature would be overkill. Hence the "two of four" trigger criteria.

### Neutral

- **Two merge strategies coexist.** Small rollups squash (keeps main linear); large rollups merge-commit (preserves per-PR audit trail for `git bisect`). Choice documented per-rollup in Phase 7.

---

## 4. Alternatives considered

### Direct-to-main with rebase trains
Land every PR on main as soon as it's reviewable; rebase the rest of the train on each merge.

- Pro: main is the only branch; no coordination layer.
- Con: each PR review sees a non-final dependency stack, so cross-PR findings still escape. Doesn't solve the "main is the work surface and the demo surface" problem.

### Stacked PRs without an integration branch
Use git's `--base` flag to chain PRs; each one auto-retargets as the predecessor merges.

- Pro: low overhead for short stacks (≤3 PRs).
- Con: doesn't scale past ~5 PRs (depth becomes painful on every rebase); doesn't carry the cross-cutting review pass; doesn't keep main clean during the epic.
- **We use stacked PRs *within* rollup phases** (e.g. Phase A→B1→B2 of the doc audit), but not as a replacement for the integration branch.

### Feature flags + direct merge
Land everything behind a runtime flag; flip the flag when done.

- Pro: main always builds; no merge conflicts at the end.
- Con: adds permanent flag-management overhead; doesn't solve the cross-PR review gap; flags accumulate as cruft.
- We use flags for runtime-toggleable behaviour (e.g. `aabb_aware_distance`) but not as the primary delivery vehicle for an epic.

---

## 5. Status

**Accepted** 2026-05-13. The 8-phase checklist was first applied to Issue #723 (rollup tracking issue) and merged via PR #729 — `feature/perception-v2-integration`, 87 commits, 3 weeks, perception PATH A end-to-end. Outcome: every cross-cutting finding from the rollup reviews was actionable and caught before main merge; no regressions surfaced on main in the week following the rollup.

The checklist lives in [`docs/how-to/INTEGRATION_ROLLUP_WORKFLOW.md`](../how-to/INTEGRATION_ROLLUP_WORKFLOW.md) and is the canonical reference. `DEVELOPMENT_WORKFLOW.md` retains a brief pointer to that doc.

---

## 6. References

- PR #724 — Introduced the rollup checklist
- Issue #723 — First rollup tracked end-to-end
- PR #729 — The integration→main merge for #723
- [ADR-005](ADR-005-parallel-agent-git-worktree.md) — Parallel agent git-worktree model
- [ADR-010](ADR-010-multi-agent-pipeline-architecture.md) — Multi-agent review pipeline (Phase 3 / 5b reviews depend on this)
- [`docs/how-to/INTEGRATION_ROLLUP_WORKFLOW.md`](../how-to/INTEGRATION_ROLLUP_WORKFLOW.md) — The phase-by-phase how-to
