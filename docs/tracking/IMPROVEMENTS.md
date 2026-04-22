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

### 2026-04-22

#### (new) Revisit: extract PATH A (SAM + mask projection) into its own process

- **Priority:** P3
- **Category:** architecture
- **Noticed while:** Issue #608 PR 2 design discussion — weighing "new process P8" vs. "two more threads in P2" for the SAM + MaskDepthProjector pipeline.
- **Current decision:** Stay in P2 as threads (per-path factory from Epic #516 supports this cleanly). Data locality is the dominant factor — MaskDepthProjector needs frame, bboxes, SAM masks, depth, and camera pose, all of which already exist in P2.
- **Revisit trigger:** When the first real SAM backend lands (non-`SimulatedSAMBackend`). At that point, evaluate:
  - Does SAM's ONNX / GPU footprint cause OOMs, hangs, or CUDA-context fights that take P2's core detection/tracking down with it?
  - Is the GPU memory ceiling pushing us toward per-process cgroups + CUDA budgeting?
  - Would a restart policy specific to the ML-heavy SAM path (exponential backoff, disable-on-repeated-failure) be meaningfully different from the current shared P2 policy?
- **If yes to any:** extract PATH A to a new process (P8). The per-path factory means the extraction is primarily moving the two thread constructors and wiring new IPC hops; protocol-level breakage is limited to making `/drone_mission_cam` have a second subscriber and adding a new `/semantic_voxels` publisher location — both low-risk given existing patterns.
- **If no:** leave as threads, document the decision's residual risk in a DR entry.

#### 15. MaskClassAssigner copies full InferenceDetection (including mask pixel buffer) per mask per frame

- **Priority:** P2
- **Category:** architecture
- **Noticed while:** PR #604 review (E5.4 MaskDepthProjector). Performance agent flagged mask pixel buffer copy.
- **Current state:** `MaskAssignment` stores `hal::InferenceDetection mask_detection` by value. `assign()` copies each SAM mask's full struct — including the `std::vector<uint8_t> mask` pixel buffer — into `MaskAssignment`. At 30 fps with N SAM masks, this is N heap allocations per frame. Currently cheap (8×8=64 bytes per mask), but production SAM masks can be 28×28+ (784+ bytes).
- **Proposed fix:** Store `const hal::InferenceDetection*` or `size_t mask_idx` in `MaskAssignment` instead of a value copy. `assign()` takes `sam_masks` by const ref and the result is consumed immediately in `project()`, so pointer/index is safe. Eliminates all mask pixel copies on the assignment path.
- **When worth doing:** When integrating real SAM backend (non-simulated) — profile first to confirm it matters.

#### 16. Three heap allocations per frame through PATH A pipeline (scratch buffers)

- **Priority:** P3
- **Category:** architecture
- **Noticed while:** PR #604 review (E5.4 MaskDepthProjector). Performance agent flagged repeated allocations.
- **Current state:** Each call to `MaskClassAssigner::assign()` allocates `det_order` and `mask_matched` vectors; `MaskDepthProjector::project()` allocates `classified` vector. Total: 3 heap allocs per frame on this path. All are bounded small vectors (10-50 elements).
- **Proposed fix:** Add `mutable` scratch buffers to both classes, `resize()` and reuse each call. Reduces steady-state allocations to zero.
- **When worth doing:** When PATH A is hot-path in production — profile first to confirm the 3 allocs are measurable vs CpuSemanticProjector's per-pixel work.

#### 17. DRY: `make_depth_map()` test helper duplicated across 3 test files

- **Priority:** P3
- **Category:** test-infra
- **Noticed while:** PR #604 review (E5.4 MaskDepthProjector). Code quality agent flagged duplication.
- **Current state:** `make_depth_map()` is defined identically in `test_semantic_projector.cpp`, `test_mask_class_assigner.cpp`, and `test_mask_depth_projector.cpp`. Similar duplication for `make_sam_mask`/`make_det` helpers with divergent naming (make_mask vs make_sam_mask, make_det vs make_detector_output).
- **Proposed fix:** Extract into `tests/test_perception_helpers.h` (inline statics). Align naming across all test files.
- **When worth doing:** Next time any of these 3 test files are modified — or as a standalone cleanup PR.

### 2026-04-21

#### 12. HAL factory functions use `throw` instead of `Result<T,E>`

- **Priority:** P2
- **Category:** architecture
- **Noticed while:** PR #599 review (Epic #519 per-class config). P4 planner/avoider factories converted in this PR; HAL factories remain.
- **Current state:** All 7 factory functions in `common/hal/include/hal/hal_factory.h` (create_camera, create_fc_link, create_gcs_link, create_gimbal, create_imu_source, create_radar, create_depth_estimator) throw `std::runtime_error` on unknown backend strings. This is startup-only code but violates the project's `Result<T,E>` error-handling pattern.
- **Proposed fix:** Convert all HAL factories to return `Result<std::unique_ptr<Interface>>`. Update callers in all 7 process main.cpp files and tests.
- **When worth doing:** Standalone refactor PR — touches many files across the whole stack.

#### 13. No IPC-boundary validation of `class_id` before `avoid()`

- **Priority:** P3
- **Category:** architecture (defense-in-depth)
- **Noticed while:** PR #599 review — OOB array access via `class_id` as index into `std::array<T, 8>`.
- **Current state:** `mission_state_tick.h` passes `DetectedObjectList` from IPC directly to `avoider.avoid()` without validating `class_id` bounds. The avoider now has an in-loop guard (`if (ci >= kPerClassCount) continue`), but defense-in-depth says validate at the system boundary too.
- **Proposed fix:** Add a `validate()` method to `DetectedObjectList` or a free function that clamps/filters invalid class IDs before passing to downstream consumers. Log a warning when invalid IDs are received from IPC.
- **When worth doing:** When touching IPC types or adding new consumers of DetectedObjectList.

#### 14. CameraIntrinsics needs lens distortion model for real hardware (#601)
- **Priority:** P3
- **Category:** architecture
- **Noticed while:** implementing E1 `CpuSemanticProjector` (PR #602)
- **Symptom:** `CameraIntrinsics` only has pinhole parameters (fx, fy, cx, cy). Real drone cameras with 120–150° FoV lenses have significant barrel distortion — pixels near image edges can be off by 10–30px, causing 0.5–2m 3D position errors in the volumetric map.
- **Fix:** Extend `CameraIntrinsics` with Brown-Conrady distortion coefficients (k1, k2, p1, p2, k3). Apply undistortion in `CpuSemanticProjector::backproject()` via iterative Newton's method. If fisheye lens selected, add Kannala-Brandt KB4 model.
- **Trigger:** Implement when nav camera hardware is selected and calibration data exists. Zero impact in simulation (all coefficients default to 0).
- **Issue:** #601

### 2026-04-20

#### 10. `COSYS_SIMULATION_ARCHITECTURE.md` — parallel to the existing Gazebo doc

- **Priority:** P3
- **Category:** docs (architecture reference)
- **Noticed while:** filing issue #594 (GT emitter) — the existing `docs/architecture/SIMULATION_ARCHITECTURE.md` is Gazebo-only despite Cosys now being comparable in complexity (HAL backends, segmentation, `simSpawnObject` scene population, scenarios #29/#30, GT emitter).
- **Current state:** Cosys architecture is scattered across ADR-011 (the "why"), `docs/guides/COSYS_SETUP.md` (the "how-to-install"), and inline comments in `common/hal/src/cosys_*.cpp`. No single reference doc for the runtime architecture.
- **Proposed fix:** create `docs/architecture/COSYS_SIMULATION_ARCHITECTURE.md` mirroring the Gazebo doc's structure — HAL-mapping, RPC-surface inventory, scenario population via `simSpawnObject`, segmentation pipeline, GT-emitter integration, known limitations. Cross-link from the Gazebo doc and from COSYS_SETUP.md.
- **When worth doing:** after the #594 (GT emitter) + #573 (baseline capture) work stabilises the Cosys-side patterns — then we document what actually shipped rather than chasing a moving target.

#### 5. Stage-name constants (eliminate magic-string drift across P2/P4/tests)

- **Priority:** P3
- **Category:** architecture (benchmark harness consistency)
- **Source:** deferred from `review-code-quality` agent on PR #593
- **Current state:** Stage names like `"Detect"`, `"Track"`, `"Fuse"`, `"PlannerLoop"`, `"GeofenceCheck"`, `"FaultEval"` appear as string literals at each `ScopedLatency` site across `process2_perception/src/main.cpp`, `process4_mission_planner/src/main.cpp`, and `tests/test_latency_profiler_dump.cpp`. A rename at one site wouldn't break the build — the baseline harness would silently see a new stage name while the old name disappears.
- **Proposed fix:** `namespace drone::cfg_key::benchmark::stage_names { inline constexpr std::string_view DETECT = "Detect"; ... }` in `util/config_keys.h`. All three locations (P2, P4, test) use the constants.
- **When worth doing:** when a third consumer (dashboard / CI gating) needs to reference the same stage names, OR when a rename has to happen for real (e.g. renaming `"Detect"` to `"Detection"` to match a schema).

#### 6. `LatencyTrace::stage` as `char[N]` for trivially-copyable ring (perf)

- **Priority:** P3
- **Category:** test-infra (benchmark harness performance)
- **Source:** deferred from `review-performance` agent on PR #593 (also `review-performance` on PR #591, documented in DR-020)
- **Current state:** `LatencyTrace` holds a `std::string stage`. SSO keeps typical stage names (≤15 chars) allocation-free, but every `record()` call still does a `slot.stage.assign()` under the mutex — ~10 ns of unnecessary work per call on the hot path.
- **Proposed fix:** switch `LatencyTrace::stage` to `char stage[32]` (or `std::array<char, 32>`). Makes `LatencyTrace` trivially copyable, allows `memcpy`-based bulk snapshot in `traces()`, eliminates SSO dependency.
- **When worth doing:** when a hot-path consumer records at >1 kHz (current max is 30 Hz), or when a certification audit flags the SSO assumption as an unbounded-allocation risk.
- **Trade-off:** silent truncation of stage names longer than 31 chars — manageable with a static assert on each stage-name literal.

#### 7. TSan run for `AllRecordsLandUnderConcurrentWorkers` test

- **Priority:** P3
- **Category:** test-infra (coverage)
- **Source:** deferred from `review-test-quality` agent on PR #593
- **Current state:** The 6-thread × 500-record stress test guards against count mismatches but can't catch memory-ordering bugs without ThreadSanitizer. CI runs the test under default GCC; no TSan pipeline runs on each PR.
- **Proposed fix:** add a `bash deploy/run_ci_local.sh --job TSAN` step that runs `test_latency_profiler*` under TSan, and require it pass before landing changes to `common/util/include/util/latency_profiler.h`.
- **When worth doing:** when a real TSan finding slips through to main, or as part of a broader CI-coverage uplift.

#### 8. `[[likely]]` / `[[unlikely]]` on the profiler gate

- **Priority:** P3
- **Category:** code quality (micro-optimisation)
- **Source:** deferred from `review-performance` agent on PR #593
- **Current state:** `if (profiler) bench.emplace(...)` — branch predictor handles it perfectly after the first few frames, but an explicit `[[unlikely]]` on the null-branch would improve code layout in the default (disabled) path.
- **Proposed fix:** `if (profiler_ptr) [[unlikely]] { ... }` at each of the 6 call sites.
- **When worth doing:** when profiling shows the disabled-path overhead is measurable. Currently it's sub-nanosecond — not worth the readability cost.

#### 9. `= nullptr` default on `LatencyProfiler*` thread-function parameters

- **Priority:** P3
- **Category:** api-ergonomics
- **Source:** deferred from `review-api-contract` agent on PR #593
- **Current state:** P2's `inference_thread`, `tracker_thread`, `fusion_thread` all take `LatencyProfiler* profiler` as the final positional parameter with no default value. Every current caller passes it explicitly, but a future caller could forget and get a compile error instead of a safe-by-default null.
- **Proposed fix:** add `= nullptr` to the parameter in the function declaration.
- **When worth doing:** when a second caller of any of these thread functions appears (e.g. a test harness that wants to run the thread function with a mock detector).

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
