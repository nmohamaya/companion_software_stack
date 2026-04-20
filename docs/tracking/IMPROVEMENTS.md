# Improvements Backlog

Running list of improvements noticed in passing while doing other work. Not urgent enough to derail the current task, but worth fixing when we look for lighter work or take a breather between deep sessions.

**How to use:**
- New findings go at the top of the current date's section.
- When a finding is addressed, move it to the bottom **Resolved** section with the PR/commit reference.
- Priority is a quick read, not a promise:
  - **P1** — blocks or will block something real (CI, build, deploy)
  - **P2** — obvious paper cut, worth fixing in the next quiet window
  - **P3** — minor, nice to have, fix if you're already touching the area

**Categories:** `build`, `ci`, `docs`, `dev-tooling`, `test-infra`, `workflow`, `architecture`, `scripts`.

---

## Open

### 2026-04-20

#### 3. `compute_ap` — O(11 × log nP) via max-precision envelope

- **Priority:** P3
- **Category:** test-infra (benchmark harness performance)
- **Source:** deferred from `review-performance` agent on PR #590
- **Current state:** `compute_ap` in `tests/benchmark/perception_metrics.cpp` runs an O(11 × nP) scan over the precision-recall curve for each of the 11 VOC recall checkpoints. Measured ≈5 ms at N=1000, well under the 100 ms AC.
- **Proposed fix:** Standard VOC trick — sweep the precision curve right-to-left once to build a max-precision envelope (`max_precision[i] = max(precision_curve[i..end])`), then binary-search for each recall checkpoint. Reduces the interpolation step from O(11 × nP) to O(11 × log nP). Net: one extra O(nP) pass.
- **When worth doing:** benchmark harness scales to N ≥ 10 000 detections per scenario, or profiling shows `compute_ap` on a hot path.

#### 4. `std::stable_sort` in `compute_ap` / `match_frame` for cross-run determinism

- **Priority:** P3
- **Category:** test-infra (benchmark harness reproducibility)
- **Source:** deferred from `review-performance` agent on PR #590
- **Current state:** both `std::sort` call sites (confidence-desc sorting in `compute_ap` and `match_frame`) use unstable sort. Tie-broken predictions can land in implementation-defined order, producing non-deterministic TP/FP assignments (and therefore AP) across platforms when two predictions share the exact same confidence.
- **Proposed fix:** swap to `std::stable_sort`. ~10–20% slower at N=100 000 but still well within budget.
- **When worth doing:** CI gates AP across Linux × macOS runners, or an investigation chases a flaky AP delta to tie-breaking.

#### 1. Ninja/Unix-Makefiles generator mismatch on `airsim-build` blocks rebuild
- **Priority:** P2
- **Category:** build
- **Noticed while:** building PR #590 metrics framework
- **Symptom:** On a machine where `airsim_external-prefix/` was previously configured with one generator, subsequent `cmake`/`ninja` invocations fail with:
  > CMake Error: Error: generator : Ninja — Does not match the generator used previously: Unix Makefiles
- **Current workaround:** `rm -rf build/airsim-build/CMakeCache.txt build/airsim-build/CMakeFiles build/airsim_external-prefix/src/airsim_external-stamp/airsim_external-configure` then rebuild.
- **Fix options:**
  1. Pin the generator explicitly in `deploy/build.sh` (e.g. `cmake .. -G Ninja`) so every invocation matches.
  2. Pass `-G ${CMAKE_GENERATOR}` through to the `ExternalProject_Add` call for `airsim_external` in `cmake/FindAirSim.cmake` so the sub-project always mirrors the parent.
  3. Document the workaround near `FindAirSim.cmake` and in `docs/guides/DEV_MACHINE_SETUP.md`.
- **Recommendation:** option 2 — it's the root cause; options 1 and 3 are palliatives.

#### 2. `.gitignore` hides `perception_v2_detailed_design.md` from PR diffs
- **Priority:** P2
- **Category:** docs / workflow
- **Noticed while:** opening PR #590; the Universal AC from meta-epic #514 says "Update `docs/design/perception_v2_detailed_design.md`" on every sub-issue, but the file is gitignored (it's still a draft from 2026-04-18).
- **Symptom:** reviewers can't see whether the design doc was actually updated; the acceptance-criterion check becomes honor-system.
- **Fix options:**
  1. **(preferred)** Commit the doc — it's the canonical record for the rewrite now, not a draft. Drop the `.gitignore` entry; subsequent PRs update it in-tree and the updates show in diffs.
  2. Keep it local-only, require each PR author to paste a diff snippet into the PR body.
  3. Move the "update the design doc" line out of Universal AC into per-issue AC where applicable.
- **Recommendation:** option 1 when we're comfortable the doc is public-facing.

---

## Resolved

*(none yet — as improvements land, move entries here with a link to the PR/commit that closed them.)*
