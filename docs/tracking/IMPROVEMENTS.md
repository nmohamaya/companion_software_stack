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

### 2026-05-15 (PR #775 review deferrals — test/code-quality P2/P3 follow-ups)

#### DISARM-vs-FSM ordering not asserted in Layer 4 timeout tests

- **P2** (`tests/test_mission_state_tick.cpp`) — `Layer4NeverSettlesFiresTimeoutAndDisarms` asserts both `fsm.state() == IDLE` AND `fc_calls[0].cmd == DISARM`, but never verifies DISARM was emitted BEFORE the FSM transitioned.  If a future change reorders to `fsm.on_landed(); send_fc(DISARM)`, motors stay armed in IDLE state for one tick — silent safety regression that both assertions still pass.  Add a sequence-counter assertion or use a mock `FCSendFn` that captures the FSM state at send-time.  **When to do it:** next time the Layer 4 tests are touched, or before any reordering of `tick_preflight`'s timeout-fired path.  **Cross-ref:** PR #775 review-test-quality P2.

#### No warn→timeout single-timeline test

- **P2** (`tests/test_mission_state_tick.cpp`) — `WarnThresholdElapsesButNotYetTimeout` and `Layer1NeverReleasesFiresTimeoutAndDisarms` test the two boundaries independently.  No test walks t=0 → t=warn (still PREFLIGHT) → t=timeout (DISARM) on one fixture.  A regression that resets the warn-tracking timer at the warn-log boundary would not be caught.  Add `Layer1WarnPromotionThenTimeoutOnSameTimeline` that exercises both boundaries in sequence.  **Cross-ref:** PR #775 review-test-quality P2.

#### Outer-state-exit detector not isolated for non-timeout PREFLIGHT exit

- **P2** (`tests/test_mission_state_tick.cpp`) — `ReentryToPreflightStartsFreshTimer` exercises re-entry after the timeout fires, but the "GCS aborts PREFLIGHT mid-window before timeout, then re-enters PREFLIGHT" path is not isolated.  Add `PreflightTimerResetsOnNonTimeoutExitViaGCSAbort`.  **Cross-ref:** PR #775 review-test-quality P2 + test-unit P3 (analogous gap).

#### Weak thread-safety assertion in PlannerStallHandler concurrency test

- **P3** (`tests/test_planner_stall_handler.cpp`) — `ConsumeEventIsThreadSafeAcrossWatchdogAndPlanningLoop` asserts `EXPECT_GE(consumed.load(), 1)` which is satisfied by any single successful consume — far too loose.  A regression from `release/acquire` → `relaxed` would almost certainly still pass on x86-64 (TSO).  Tighten to `consumed >= kIterations / 10` and run under TSan in CI to make ordering bugs detectable.  **Cross-ref:** PR #775 review-test-quality P3.

#### LatencyProfiler trace copy under mutex (string allocations)

- **P3** (`common/util/include/util/latency_profiler.h`) — `collect_traces_locked()` copies up to 4096 `LatencyTrace` objects (each with a `std::string stage`) while holding `mtx_`.  Currently bounded + watchdog-thread-only so non-blocking, but if the profiler is ever extended to higher trace rates, consider `string_view` or fixed-size char array in `LatencyTrace` to eliminate per-trace allocation.  **Cross-ref:** PR #775 review-performance P3.

#### `StateTickConfig` field count growth

- **P3** (`process4_mission_planner/include/planner/mission_state_tick.h`) — `StateTickConfig` is at 17 fields across 3 logical groups (basic FSM, Layer 1/4 debounce, preflight timeout).  Codebase pattern: split when fields exceed ~20.  When the next preflight/Layer-N config knob is added, consider extracting `StateTickConfig::PreflightConfig` nested struct.  **Cross-ref:** PR #775 review-code-quality P3.

### 2026-05-15 (recovered from stash@{0} — scenario-runner / stack-coverage audit, originally 2026-05-01 / 2026-04-30)

#### `tests/run_scenario.sh` missing `requires_cosys_airsim` skip gate

- **P2** — `tests/run_scenario.sh` skips scenarios with `"requires_gazebo": true` but has no equivalent gate for `"requires_cosys_airsim": true`.  Scenarios 29, 30, and 33 set `requires_cosys_airsim` (not `requires_gazebo`), so `--all` will attempt to launch them on any machine, fail at stack startup (no Cosys-AirSim backend), and count them as failed rather than skipping cleanly.  Fix: add a second skip block in `run_scenario.sh` (around line 354) that reads `requires_cosys_airsim` and exits 0 with a SKIP message, mirroring the existing Gazebo gate.  Until fixed, always pass `--tier 1` to limit the run to genuine Tier 1 scenarios.

#### 17 scenarios missing `_comment_static_obstacles` audit annotation

- **P3** — 17 of 30 scenario JSON files have `"static_obstacles": []` without a `_comment_static_obstacles` key explaining why the array is empty.  The audit policy (CLAUDE.md "Stack Coverage Audit") requires an explanatory comment for every empty array.  Without it, a reviewer cannot distinguish "intentional: fault-injection scenario" from "accidental: HD-map was forgotten."  Add `_comment_static_obstacles` to scenarios 03, 04, 06, 07, 09, 10, 11, 12, 13, 14, 15, 16, 21, 23, 24, 25, 28.  Templates: `"_comment_static_obstacles": "Empty — fault-injection/smoke-test scenario; mission is interrupted before planner is significantly exercised."` for fault-injection group; `"Empty — no HD-map; perception-driven avoidance at runtime."` for Gazebo perception group (21, 28).  Verified present and correct in 01, 17, 18, 19, 27, 29, 30, 33 (9 scenarios already compliant).  Note: scenario numbering may have shifted since the original audit (April 30) — verify the offender list against current scenario IDs before fixing.

### 2026-05-13 (PR #752 review-fix follow-ups)

#### Decide whether VIO `LOST` state should also withhold pose publication

- **P2** — `process3_slam_vio_nav/src/main.cpp::vio_pipeline_thread`.  PR #752 added a guard that suppresses `pose_buffer.write()` while `output.health == VIOHealth::INITIALIZING`.  The other unhealthy state — `VIOHealth::LOST` ("tracking lost — need re-initialization") — currently still publishes the (now-stale) pose.  Downstream the planner's `FaultManager` checks pose age (`stale_pose_ns`) but does NOT check `Pose::quality`, so a backend that keeps emitting fresh-stamped LOST poses with `quality=0` does not trigger `FAULT_POSE_STALE` and the consumer has no signal that pose is unreliable.
  - **Pros of expanding the guard to LOST:**
    - Semantically equivalent to INITIALIZING — consumer can't trust the pose.
    - Cleaner FAULT_POSE_STALE triggering at the planner.
  - **Cons:**
    - Behaviour change beyond PR #752's stated scope ("INITIALIZING pose-publish guard").
    - Consumers that DID look at `Pose::quality` would lose visibility.
  - **Suggested fix:** open a follow-up issue + DR-NNN evaluating both options.  If we expand to LOST, the guard becomes `output.health == INITIALIZING || output.health == LOST` and the constant becomes a small enum / named bool.
  - **When to do it:** when fault-response semantics next change, or when a scenario reproduces VIO-LOST mid-flight and exhibits the missing FAULT_POSE_STALE.
  - **Source:** PR #752 Copilot review — `process3_slam_vio_nav/src/main.cpp:198/214` comment-vs-code mismatch.  PR #752 addressed the immediate concern by tightening the comment + log to say "INITIALIZING only" rather than "until DEGRADED/NOMINAL".

---

### 2026-05-13 (PR #752 implementation — proactive findings)

#### Refactor `vio_pipeline_thread` for unit testability

- **P3** — `test-infra` — `process3_slam_vio_nav/src/main.cpp::vio_pipeline_thread` is currently a 100+ line free function that captures `pose_buffer`, calls `backend.process_frame()`, and writes to the buffer.  No unit test directly exercises this function — coverage is via scenario integration tests only.  PR #752 (this PR) added an `output.health == INITIALIZING` guard inside the function but couldn't add a focused unit test for the new branch because the function isn't structured for testability (depends on a thread-local `PoseDoubleBuffer` reference + `Diagnostics` + ambient running flag).
  - **Suggested fix:** extract the per-frame decision logic (the "should I publish this output?" call) into a free function `should_publish_vio_output(const VIOOutput&) -> bool` or a thin method on a struct.  The pipeline thread calls the helper and acts on the result.  Unit tests then cover the helper directly with synthetic `VIOOutput` instances spanning all health states (INITIALIZING, DEGRADED, NOMINAL, LOST).  The thread function becomes a thin glue layer that doesn't need its own unit test.
  - **When to do it:** opportunistic — when next touching `vio_pipeline_thread` for another reason, or when adding the next per-output gating rule (likely Layer 4 RTF-aware logic from #746 if it lands here).
  - **Affected:** test coverage gap for P3 publish-thread logic.  Today the contract "don't publish during INITIALIZING" is enforced inline but only exercised by scenario tests.  Cheap-to-add unit test would catch a regression at PR time instead of scenario-sweep time.

---

### 2026-05-13 (PR #750 review-fix follow-ups)

#### `ZenohSubscriber` SFINAE-branch test coverage gap

- **P2** — The new wrapper-level stale-message filter in `common/ipc/include/ipc/zenoh_subscriber.h::on_sample()` is gated by `if constexpr (has_validate<T>::value)` then `if constexpr (has_timestamp_ns<T>::value)`.  Four code paths exist:
  1. `validate=Y, timestamp_ns=Y` — exercised by `ZenohStaleMessageFilter.*` tests in `tests/test_zenoh_coverage.cpp` (and most safety-critical IPC types).
  2. `validate=Y, timestamp_ns=N` — no current IPC type matches; no test.
  3. `validate=N, timestamp_ns=Y` — `ThreadHealth` matches but is publisher-side only (never subscribed via this wrapper); no test.
  4. `validate=N, timestamp_ns=N` — no current IPC type matches; no test.
  - **Why:** A future contributor who adds a non-validating timestamped type and relies on the constructor docstring will not be alerted that the filter silently fails to apply to their type.  Branch coverage is currently theoretical-only for paths 2 / 3 / 4.
  - **Suggested fix:** add 1–2 tests in `tests/test_zenoh_coverage.cpp::ZenohStaleMessageFilter` using a synthetic test-only type (e.g. `struct NoValidateWithTs { uint64_t timestamp_ns; }`) that exercises path 3 and asserts the message passes through unfiltered.  Path 2 is harder to test meaningfully without a `validate()` implementation; path 4 is the no-op baseline already covered by happy-path subscriber tests.
  - **When to do it:** next change that touches `on_sample()`, or when adding a new IPC type that opts out of the filter.

#### Duplicate `### Fix #51` entry in BUG_FIXES.md

- **P3** — Two entries share the heading `### Fix #51`: one is "Gazebo SITL Broken After Ubuntu System Update" (chronologically earlier), the other is "Depth Anything V2 ONNX Model Incompatible with OpenCV DNN".  Renumbering one of them (probably the later one) keeps the audit trail unambiguous.
  - **When to do it:** any quiet window; trivial sed-replace + index update.

---

### 2026-05-13 (PR #744 review-fix follow-ups)

#### `tests/lib_check_contacts.py` — no pytest coverage

- **P2** — The Python state machine that parses `gz topic` text format and classifies drone-vs-obstacle pairs is currently only verified by its in-the-loop behaviour during a scenario run.  A regression in `parse_contacts()` (e.g. the `---` delimiter reset added in PR #744 review fixes, or the `is_allowlisted` substring rule) could silently produce phantom or missing events without any unit-test signal.
  - **Why:** the helper is a small but load-bearing piece of cold-start observability — a quietly-broken parser would re-introduce the false-PASS class this gate exists to prevent.
  - **Suggested fix:** add `tests/test_lib_check_contacts.py` with pytest cases for: (a) synthetic clean log → exit 0; (b) synthetic single-contact log → exit 1 with the expected pair; (c) allowlist suppression; (d) truncated `contact { ... }` block followed by `---` → no phantom event; (e) missing file → exit 2.  Wire into `tests/run_tests.sh` if pytest is on the CI path.
  - **When to do it:** next time the parser is touched, or as part of the Tier-1 follow-up gate that will share the same state machine.

---

### 2026-05-13 (PR #741 review-fix follow-ups — backlog items deferred from the review)

These are valid suggestions surfaced by the [9-agent + Copilot review of PR #741](https://github.com/nmohamaya/companion_software_stack/pull/741#issuecomment-4440678222) that we chose to defer to backlog rather than fix in the immediate follow-up PR #743.  Each has a defined "when to do it" trigger so it doesn't get forgotten.

**Audit trail note:** these are review-comment-derived but routed to IMPROVEMENTS.md rather than DESIGN_RATIONALE.md because each is a **valid-but-deferred backlog item** ("yes will do later") rather than a **considered-and-declined design decision** ("we evaluated both paths and chose this one").  See CLAUDE.md §"Where deferred items are logged" (clarified in PR #743 to distinguish these two flavours).

#### Sibling `cfg_key` constants for `preflight_arm_retry_s` / `preflight_wait_log_s` absent

- **P3** — `architecture` — PR #741's review (api-contract agent) noted that `process4_mission_planner/include/planner/mission_state_tick.h::StateTickConfig` exposes three preflight tunables (`preflight_arm_retry_s`, `preflight_wait_log_s`, `preflight_armable_stable_s`) but only the third has a corresponding `cfg_key::mission_planner::PREFLIGHT_ARMABLE_STABLE_S` constant (added by PR #741).  The other two are read via hardcoded string literals or implicit defaults.  This is pre-existing asymmetry from PR #717 — not introduced by #741 — but is now visible.
  - **Suggested fix:** add `PREFLIGHT_ARM_RETRY_S` and `PREFLIGHT_WAIT_LOG_S` constants to `common/util/include/util/config_keys.h` under `namespace mission_planner`, and wire them into `main.cpp` where the values are read.
  - **When to do it:** bundle with #734's "tracking-doc cleanup + IMPROVEMENTS↔DR-NNN re-homing" or any other config-hygiene pass.  Not worth a standalone PR.
  - **Affected:** documentation consistency only — production behaviour is unchanged because `cfg.get<>` already handles missing keys via fallback.

#### `preflight_armable_stable_s` not propagated to `config/hardware*.json`

- **P3** — `docs / config` — PR #741's review (security agent) noted that the new `preflight_armable_stable_s = 3.0` JSON entry exists only in `config/default.json`, not in `config/hardware.json`, `config/hardware_orin.json`, or `config/hardware_edge.json`.  Operators inspecting a hardware-specific config can't see the key or tune it without knowing to look in `default.json`.
  - **Suggested fix:** add the key to all three hardware configs.  But: real-hardware tuning values (Orin vs edge vs Jetson Nano) may differ from Gazebo's 3.0s and **aren't empirically known yet**.  Propagating now risks locking in wrong values.
  - **When to do it:** when real-hardware testing produces empirical evidence about EKF2 settling time on each platform.  Until then, the `cfg.get<float>(..., 3.0f)` fallback is safe.
  - **Affected:** operator visibility only — no functional impact.

#### Repeated positional `StateTickConfig` constructor in test fixtures

- **P3** — `test-infra` — PR #741's review (code-quality agent) noted that `StateTickConfig c{10.0f, 1.5f, 0.5f, 5}` (positional brace-init for the first 4 fields) is repeated in 3 sites in `tests/test_mission_state_tick.cpp`: `make_default_test_config()`, the lambda init for `MissionStateTickDebounceTest`, and the free `ZeroWindowDisablesDebounce` test.  Minor DRY violation — if the `StateTickConfig` field ordering ever changes, all three sites need updating but only the first is obvious.
  - **Suggested fix:** extract a single `make_state_tick_config(float armable_stable_s)` helper that all three sites use.  Or, more ambitiously, a broader test-helper extraction across the planner test suite (other test files repeat similar fixture patterns).
  - **When to do it:** bundle with any other test-infra cleanup pass, or when a 4th similar construction site appears.
  - **Affected:** test maintainability only.

---

### 2026-05-13 (PR #741 — proactive safety / test-discipline findings during #740-A implementation)

Two items noticed while implementing the ARM-gate debounce (PR #741, epic #740 / #727 Layer 1). Both are test-discipline / drift-hygiene, not flight-safety, so they're logged here rather than escalated. Adding them surfaces patterns worth catching during future review.

#### Test-fixture ordering with `ScopedMockClock`

- **P3** — `test-infra` — `ScopedMockClock` MUST be declared as a fixture member BEFORE the unit-under-test, because the UUT's constructor may query `drone::util::get_clock()` at construction time and capture the production clock if the override isn't installed yet. Member-init order is declaration order; violating this silently breaks mock-driven tests in a way that's hard to debug (mock advances do nothing, tests look like they should pass but the UUT sees real wall-clock time).
  - **Suggested fix:** document the pattern in `docs/reference/CPP_PATTERNS_GUIDE.md` §5.7 (done in this PR), and check during review whenever a new `ScopedMockClock`-using test class is added.
  - **Affected:** every test fixture that injects `ScopedMockClock`. Currently rare but will become common as we migrate more time-dependent code to `drone::util::get_clock()` (see next item).

#### `std::chrono::steady_clock::now()` direct usage in process[1-7]_* code

- **P3** — `architecture` — 35+ direct uses of `std::chrono::steady_clock::now()` remain in `process4_mission_planner/` alone (other processes likely similar). These are grandfathered under the new §5.7 rule but each one blocks unit-test mockability of the surrounding logic.
  - **Suggested next step:** opportunistic migration when touching the surrounding code (the rule already says this). One-shot bulk migration as a separate refactor PR is also viable but would touch many files; review-pass-1 fault-recovery would likely raise concerns about subtle behaviour change so the touched-when-edited approach is safer.
  - **Affected:** `mission_state_tick.h` (~10 sites), `fault_manager.cpp` (~8), `geofence.cpp`, `gcs_command_handler.h`, etc.

#### `tests/TESTS.md` total drift

- **P3** — `docs` — `tests/TESTS.md` top table claims 2074 base tests on `feature/perception-v2-integration` HEAD. My `ctest -N` on `feature/cold-start-hardening` HEAD (= main = `629bdcc`) shows 2058 base. That's a 16-test gap between docs and reality, pre-existing the PR-A changes. Either tests aren't being compiled in a standard build, or TESTS.md is stale.
  - **Suggested next step:** reconciliation pass in a separate PR — run `ctest -N` against a fresh `main` build, walk the per-suite counts, and update TESTS.md to match. Bonus: add a CI gate that diffs TESTS.md against `ctest -N` output.
  - **Affected:** documentation accuracy only, no functional impact.

### 2026-05-13 (PR #735 ADR-014 — TWO classes of pre-existing breakage inherited from PR #729 perception-v2 merge)

Caught while opening PR #735 (ADR-014 — SWVIO algorithm-selection + FTO §9). Format-check failed on PR-#735's branch but the failures are **inherited from `main`**, not introduced by PR #735. After fixing format-check, a **second** class of pre-existing breakage emerged — sanitizer test failures that the format-check failure had been **masking**.

#### CI — format-check gating masked deeper sanitizer-test failures

- **P1** (upgraded from P2 after second failure class discovered) — Two distinct classes of pre-existing CI breakage on `main`, inherited from PR #729 (~92 commits). The first masked the second.

  **Class 1 — clang-format-18 violations** (visible): 16 files across `common/hal/`, `process3_slam_vio_nav/`, `process4_mission_planner/`, `tests/` were committed without a final clang-format pass during the PR #729 merge. Format-check fails on every PR branching off `main` until fixed.

  **Class 2 — sanitizer-incompatible tests** (MASKED by Class 1): the build matrix on `main` is **gated on format-check passing** — when format-check fails, the build matrix is skipped entirely. This means three test failures under sanitizer builds were completely hidden until PR #735's format-fix unmasked them:
  - `LatencyProfiler.OverheadUnderBudget` — fails under TSan (timing budget incompatible with TSan instrumentation overhead)
  - `LatencyProfiler.ConcurrentReadersDoNotRaceWriters` — fails under UBSan
  - `Performance.LargeFrameUnder100ms` — fails under ASan + TSan + UBSan (latency budget incompatible with sanitizer overhead)

  These are timing-sensitive performance / observability tests that don't tolerate sanitizer instrumentation overhead. Likely need `GTEST_SKIP_IF_SANITIZED()` guards or sanitizer-aware budget scaling — see [DR-022](DESIGN_RATIONALE.md) discussion of latency-profiler sanitizer interaction if it exists.

- **Evidence:**
  - PR #735's format-fix commit `c070965` made format-check go green
  - Once format-check passed, the build matrix RAN and revealed the three sanitizer test failures
  - `main` HEAD CI (run 25792496533, sha `629bdcc`) shows `format-check: FAILURE` + `build (matrix): SKIPPED` — confirming the matrix never ran on main with the current code
  - PR #735 itself does NOT introduce any of these failures; its diff is docs-only ADR + mechanical whitespace fixes

- **Why this is P1 not P2:**
  - **CI gate ordering masking real bugs.** Format-check failure didn't just slow downstream PRs; it actively concealed a second class of breakage from anyone looking at main's CI dashboard. "Format-check failed, sanitizer status unknown" is materially worse than "format-check failed, sanitizer status also failed" because the unknown status doesn't trigger a fix-it instinct.
  - **Sanitizer breakage on a safety-critical codebase is not optional.** TSan / ASan / UBSan exist specifically to catch data races, memory issues, and undefined behaviour that would cause loss of vehicle in production. Letting sanitizer tests fail silently is a direct safety regression.
  - **Discovery latency.** This breakage landed via PR #729 on 2026-05-13 morning and was only discovered later the same day when PR #735 happened to fix the format-check. Without that accidental sequence, sanitizer breakage could have stayed masked for weeks.

- **Suggested fix:**
  1. **Restructure CI workflow so format-check and the build matrix run in parallel**, both required for merge. Format-check failures shouldn't mask test failures.
  2. **Confirm format-check AND every sanitizer build are required status checks on `main`** branch protection — if any are non-required, fix that.
  3. **For large integration PRs (>20 commits or any wave-merge)**, require the full CI matrix (not just format-check) to pass on the integration branch's tip before the squash/merge button activates.
  4. **Add a pre-merge "rebase test" job** for any PR whose base branch has moved >10 commits since the PR's last push — catches the inheritance-from-stale-main pattern.
  5. **Separately, fix the three sanitizer test failures** — likely via `GTEST_SKIP_IF_SANITIZED()` guards, conditional latency budgets, or test-runner annotations. Track in a dedicated GitHub issue.

- **Why it matters:** trust in CI is binary. If a green PR can inherit a red main, contributors stop trusting the green. Worse: if a red CI dashboard masks a second class of red further down, the visible failure becomes a misleading indicator of true state. Equivalent to a fire alarm that activates after the fire is out *and* obscures the burning room behind a door no one opens.

- **Owner:** `feature-infra-platform` (CI / GitHub Actions workflow). Two follow-ups: (a) CI workflow restructure (P1); (b) sanitizer test fixes (P1).

- **Tracker:** to be filed as a GitHub issue.

---

### 2026-05-11 (#712 cleanup — residual concerns observed during scenario 18 testing)

Items observed in the field during the #710 → #712 work but **outside the cleanup scope**.  Both pre-date commit 3 (they were observable across multiple commits today on the integration branch) and reproduce on a clean tree.

#### Gazebo SITL — rotor-spin-up delay regression

- **P1** — Scenario 18 (and likely 02/17) on the integration branch shows a **~5 s delay between PX4 arming and rotor engagement** at takeoff.  On `main` (commit `389089a`), rotors engage immediately.  Symptom: drone sometimes hits the ground before climbing, sometimes climbs in a degraded EKF state that destabilises later flight.
  - **Evidence:** main scenario 18 run `2026-05-07_102520_PASS` shows clean immediate rotor spin-up; every integration-branch run today (commits 42e7af9, aa69d53, e6bf478) shows the 5 s delay.
  - **Bisect range:** between `main` (`389089a`, Apr 21) and `c7d64fb` (Apr 30, our previous bisect's known-good baseline).  This range was never bisected — our earlier bisect started at `c7d64fb` and found PR #657 was the *avoider* regression, but the rotor-delay regression is older and separate.
  - **Affected:** PX4 SITL takeoff stability in Gazebo scenarios.  Cosys-AirSim scenarios unaffected (different FC link path — `cosys_rpc` vs `mavlink`).
  - **Suggested next step:** bisect `main..c7d64fb` on scenario 18 using the same methodology as the original #710 bisect.  Likely culprits to prioritise: anything touching `deploy/launch_gazebo.sh`, `common/hal/src/gazebo_*.cpp`, or `process5_comms/` initialisation order.
  - **Tracker:** will be filed as a follow-up issue (#712 PR description references it).

#### Gazebo runtime perception — ghost obstacles in DetectedObjectList

- **P2** — Scenario 17 and 18 (sensor-driven, no HD-map) observe more obstacles in `[Avoider] considered=N` than there are physical objects in the scene.  Run `2026-05-07_101527_FAIL` on scenario 17 had `considered=6 active=5` with only 4 physical cylinders within range; commit 3 era scenario 18 runs show `considered=7-8` in similar geometries.
  - **Root cause hypothesis:** camera+radar UKF fusion producing duplicate tracks of the same physical obstacle from radar multipath, ByteTrack retaining low-confidence tracks longer than expected, or `min_confidence` filtering being too lenient at the avoider's input gate.
  - **Effect with PR #657 active (pre-#712):** AABB-aware avoider engaged early and pushed the drone away from ghost clusters, masking the over-publishing.
  - **Effect with PR #657 removed (post-#712):** centroid-distance avoider engages later; drone passes closer; more ghosts accumulate; planner grid saturates → STUCK / hover-fallback.
  - **Suggested next step:** add diagnostic counters per detection source (camera-only vs radar-only vs camera+radar) to identify which path inflates `considered`; tighten the UKF orphan/duplicate gates.
  - **Tracker:** will be filed as a follow-up issue.

### 2026-05-05 (PR #704 second-round review — deferred follow-ups)

Items from the second 9-agent review pass on PR #704, after the first-round fixes landed.  Top-4 items (config-key unit mismatch, fault propagation, header scrub) addressed in commit 593d38b.  The items below are correct-but-deferred:

#### Performance — RadarFovGate::tick_residency hot path

- **P2** — `tick_residency()` allocates a fresh `std::unordered_set<GridCell> live` every tick (10 Hz × ~5K cells = ~40-80 KB allocator round-trip per tick).  Promote `live_scratch_` to a class member and `clear()` between ticks (matches the `by_color_pool_` pattern already used in CosysSegmentation).
- **P2** — `first_seen_ns_.find(c)` followed by `first_seen_ns_[c] = now_ns` does two hash computations per cell.  Replace with `first_seen_ns_.try_emplace(c, now_ns)` for one hash on the hit path.
- **P3** — `query_cell()` does `query()` + two separate `find()` calls on `residency_` and `first_seen_ns_`.  Cache iterators or fuse into one map type to avoid the extra hash on the per-promotion-candidate path.

#### Test quality — integration test wall-clock dependence

- **P2** — `test_occupancy_grid_cross_veto.cpp::AgeCapEvict_FiresIndependentOfResidency` and the Row-1/Row-2 tests use `now_real_ns()` for radar freshness, but `insert_voxels` reads `steady_clock::now()` internally.  On a heavily-loaded CI box the gap could exceed `radar_max_staleness_ns` and Row 2 *agree* test could flip from `Promote` to `DeferToDynamic`.  Real fix: inject a clock into `OccupancyGrid3D` (`ScopedMockClock` pattern) so integration tests don't depend on real time.
- **P3** — `GateRadiusBoundaryRespectsLeq` uses hardcoded `gate_r=3.85f` recomputed by hand.  Derive from `q.gate_radius_m` of a prior `query()` call so the boundary tracks the implementation.
- **P3** — `NanQuaternionPose_QueryReturnsConservativeVeto` only asserts `SUCCEED()`.  Tighten to `EXPECT_FALSE(q.in_fov)` + `EXPECT_TRUE(q.radar_stale)` per the conservative-veto contract.
- **P3** — `NanVoxelPosition_ClampedDropped_DoesNotReachGate` asserts `s.clamped_dropped + s.out_of_bounds > 0` (an OR).  Pin which branch fires.
- **P3** — `make_gazebo_full_vio` returns `nullptr` silently on `is_err()`.  Capture `r.error().to_string()` into `ADD_FAILURE()` before returning nullptr so the test diagnostic surfaces the real error.
- **P3** — `tests/TESTS.md` does not yet list per-file entries for `test_cross_veto_decision.cpp`, `test_radar_fov_gate.cpp`, or `test_occupancy_grid_cross_veto.cpp`.  Total count was bumped but the per-file rows are missing.

#### Concurrency / API hygiene

- **P3** — `cosys_echo_backend.h:130-141` and `cosys_groundtruth_radar.h:141-145` start the polling thread while holding `mutex_`.  No deadlock today (thread doesn't reach the lock during init), but holding `mutex_` across `std::thread` construction is a smell.  Move the thread-spawn outside the lock.
- **P3** — `cosys_echo_backend.h::scan_count_.fetch_add(…, memory_order_release)` vs `cosys_groundtruth_radar.h::scan_count_.fetch_add(…, memory_order_acq_rel)` — same logical operation, inconsistent ordering.  Align both to `release` or document the choice.
- **P3** — `GazeboVIOBackend` uses `memory_order_relaxed` on `health_` while `GazeboFullVIOBackend` uses `release/acquire`.  Pre-existing inconsistency, low risk.  Either upgrade or add a one-line justification.
- **P3** — `process3_slam_vio_nav/include/slam/ivio_backend.h:33` `<stdexcept>` include is now dead (no `std::runtime_error` after the throw was removed).  clang-tidy `misc-include-cleaner` will flag it.
- **P3** — `process3_slam_vio_nav/src/main.cpp:286 + 384` register two threads with the same `ScopedHeartbeat("pose_publisher", true)` name.  Only one runs per build but the registry slot name collision makes watchdog reports ambiguous.  Rename the passthrough variant.

#### Security — input confinement gaps

- **P2** — `tests/run_scenario_cosys.sh` accepts `--base-config` and the scenario positional arg without confining either to project root.  An attacker or misconfigured CI could supply `--base-config /etc/shadow` or `../../some_other_project/config.json` and have it read + merged before `preflight_model_paths` runs.  Apply the same `pathlib.resolve().relative_to(PROJECT_DIR)` check that already runs against `model_path` entries.
- **P2** — `cosys_echo_backend.h::data.groundtruth.size() < n_points` silently passes unnamed returns through the include/exclude name filter without warning.  Add a one-shot warn (matching `partial_point_warned_` pattern) so a malicious or buggy Cosys plugin returning truncated groundtruth is observable.
- **P3** — `tools/check_voxel_on_target.py::load_scene_xy` and `load_scenario_static_obstacles` cast `obj["x"]`/`obj["y"]` to `float()` without try/except.  A misconfigured scene file with string values raises an unhandled traceback.
- **P3** — `tests/run_scenario_cosys.sh:752` uses `python3 -c "...${AIRSIM_SETTINGS_DIR}..."` instead of the heredoc `python3 - "${path}" <<'PYEOF'` pattern used elsewhere in the file.  Inconsistent quoting.

#### API contract polish

- **P3** — `radar_fov_gate.h:404` `residency_` private-member comment says "Reset on FOV exit" — directly contradicts the intentional fix from this PR (residency now persists).  Update to match current behaviour.
- **P3** — `occupancy_grid_3d.h:209` `VoxelInsertStats` `@return` doxygen lists 6 of 9 fields (missing `cross_veto_deferred`, `fov_silence_promoted`, `age_cap_evicted`).  Update the doxygen.
- **P3** — `total_single_modality_promoted()` aggregate getter name conflates two distinct severity classes.  Rename to `total_escape_hatch_promoted()` or remove and update the two log call sites.
- **P3** — API.md IVIOBackend factory note "(PR #704: was `unique_ptr` + exception, now Result-based...)" is informal historical commentary in the permanent reference table.  Replace with stable semantics: "Returns `Err` on unknown backend or missing client; never throws."
- **P3** — `cosys_name_filter.h` whitespace trim uses manual `while` loops; could be `tok.erase(0, tok.find_first_not_of(" \t"))` and the symmetric trailing form.

#### Scenario / runner

- **P3** — `run_scenario_cosys.sh:1153` voxel check fires only when `SCENE_FILE` is present.  Future scenarios that enable `path_a.diag.trace_voxels=true` but omit `scenario.scene.file` skip the check silently.  Add a fallback warn.

### 2026-05-05 (PR #704 review — deferred follow-ups)

Items from the 9-agent review pass on PR #704 (Cosys ground-truth perception baseline) deemed correct-but-deferred or out-of-scope.  Addressed-in-PR items are listed in the PR's Review Fixes table.

#### Cosys segmentation backend

- **P2** (architectural) — Frame synchronisation between Scene RGB (P1 publishes, P2 consumes via `frame_data` parameter) and Segmentation (`CosysSegmentationBackend::infer()` re-fetches via `simGetImages`).  Different RPC calls = potentially different simulator ticks; on a moving drone this misaligns masks vs depth.  Proper fix: have P1 publish Scene + Segmentation in one `simGetImages` call, P2 consumes both from one wire type.  Workaround in this PR: timestamp output from the segmentation response so downstream knows when masks were captured.  See `common/hal/include/hal/cosys_segmentation_backend.h::infer()`.
- **P2** (perf) — Mask-buffer pooling.  Each `ObjAccum::mask` is `std::move`d into the `InferenceOutput`, so masks don't survive across frames.  At 1080p × 30 Hz × 10 objects this is ~600 MB/s heap churn.  Proper fix: separate `mask_pool_` of `vector<vector<uint8_t>>` swapped in/out of each `ObjAccum`.  Out of scope for this PR (touches the move-semantics of `InferenceDetection::mask`).
- **P3** — `infer_count_` is plain `uint64_t` with modulo-100 logging.  At >584 years of inference it wraps; theoretical, not blocking.

#### Cosys Echo / GT-radar backends

- **P2** (perf) — `DRONE_LOG_INFO` on the polling thread.  spdlog is mutex-protected, technically a CLAUDE.md "observability on flight-critical thread" violation.  Long-term: lock-free buffer drained by a dedicated IO thread.  Documented in DESIGN_RATIONALE.md as DR-026.

#### Diagnostic tools

- **P3** — `tools/diag/cosys_scene_inventory.py::propose_placement()` heuristic: `0.5 * max(scale.x, scale.y)` assumes unit-mesh actors.  Arbitrary UE5 Static Mesh actors need `simGetMeshPositionVertexBuffers`-based bounding box.  Not needed for the current Blocks-scene workflow.
- **P3** — `tools/diag/planner_grid_overlay.py` could emit absolute paths (clickable in terminals) instead of relative for the overlay-saved-to filename.

#### Test infrastructure

- **P3** — `FallbackBehaviourTest.SearchFailureKeepsLastGoodPath` (`tests/test_dstar_lite_planner.cpp:1450`) is pre-existing flaky/broken — `cached_path()` is empty after a failed plan when the test expects the prior path to persist.  Not introduced by PR #704; needs a dedicated fix-and-investigate session.

#### Documentation

- **P3** — Issue #621: API.md `IDepthEstimator` / `CpuSemanticProjector` / `IInferenceBackend` sections are still missing.  The new `cosys_segmentation_backend` factory tag was added inline to the IRadar table for now, but a proper IInferenceBackend section should be authored alongside the Issue #621 work.

### 2026-04-30 (#645 scenario-33 review-driven backlog — deferred items)

The 14-PR scenario-33 fix stack (PRs #646–#666) drove orchestrator reviews on the merged PRs.  P1 + most P2 fixes landed in batches (#667–#672).  The items below were deemed correct-but-deferred — risk-of-regression items (e.g. tuning thresholds that scenario 33 now relies on) and lower-impact hygiene items.

#### #661 — radar-confirmed promotion bypass

- **P2** (deferred — regression risk): `radar_bypass` triggers on `obj.radar_update_count > 0`.  Reviewer recommends raising to ≥3 (matching `radar_promotion_hits_`) to require the same multi-frame confirmation that the non-bypass path uses.  Risk: scenario 33 currently passes with the lenient gate; tightening it could re-introduce hover-fallbacks if radar tracks take 3 frames to mature.  Need a re-run of scenario 33 with the tighter gate to confirm no regression before landing.  See `process4_mission_planner/include/planner/occupancy_grid_3d.h:519`.
- **P2** (deferred — config tuning): scenario 33 has `max_static_cells: 0` (unlimited) **and** radar bypass enabled.  Without a cap, a radar fault that produced spurious tracks could flood the static layer.  Pre-bypass this was safe because `promotion_paused_=true` blocked the detector path entirely; post-bypass the radar path can write unbounded.  Set a defensive cap (e.g. 2000) once we have telemetry on real-world radar track counts.  See `config/scenarios/33_non_coco_obstacles.json:190`.
- **P2** (deferred — IPC contract change): `DetectedObject::validate()` doesn't bound `class_id` (could be any int) or `radar_update_count` (UINT32_MAX passes the `>0` gate).  Adding bounds requires touching the IPC wire contract and all serialiser/deserialiser tests.  Worth doing in a dedicated PR, not bundled with the scenario-33 fixes.  See `common/ipc/include/ipc/ipc_types.h:81-90`.

#### #658 — perception altitude/mask filters

- **P2** (deferred — config-validator wiring): four new perception config keys (`perception.path_a.altitude_filter.{min_z_m,max_z_m}`, `perception.path_a.mask_size_filter.{min_area_px,max_area_px}`) are not registered in `common/util/include/util/config_validator.h`.  The validator is the startup gate that catches typos before flight.  Add the four keys with `[0, 100]` m bounds for altitude and `[0, 4_000_000]` px for mask area.

### 2026-04-30 (scenario-33 forensics)

- **P2** — comms `fc_tx_thread` calls `latency_tracker_.record()` (mutex-protected observability) from inside `receive()`. PR #674 P2 review correctly flagged this as P2 per CLAUDE.md: "Mutex-protected observability primitives ... SHOULD NOT be called from flight-critical or real-time threads ... without documented justification." `fc_tx_thread` runs at 40 Hz and drives the SimpleFlight heartbeat — flight-critical. Tier rule: buffer into a lock-free primitive (`LatencyTracker` / `SPSCRing` / `TripleBuffer`) and let a dedicated IO thread drain into the shared observability. **Note:** the earlier claim that `record()` ran "inside `data_mutex_`" was factually wrong — the `lock_guard` in `zenoh_subscriber.h::receive()` goes out of scope at line 89, and `record()` is called at line 105 (after the lock release). The priority is still P2 per the project rule for any mutex-protected observability on a flight-critical thread, regardless of where in receive() the call sits.

### 2026-04-30 (scenario-33 keystone-fix follow-ups, from #652)

Surfaced while debugging the cube-collision failure mode through PRs #646/#647/#649/#650/#651.  PR #651 (SimpleFlight 60 ms heartbeat) was the keystone fix — the prior four PRs were correct but had no observable effect on the SITL plant because SimpleFlight was seizing control on the api_goal_timeout.  Items below are observability and configurability gaps that would have surfaced #651 weeks earlier (or made it a config change instead of a code change).

- **P2** — Make Cosys SimpleFlight `api_goal_timeout` configurable via `settings.json`. Currently hard-coded at 60 ms in `third_party/cosys-airsim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware/Params.hpp:136`. **Why:** PR #651 ships a comms-side heartbeat to beat the 60 ms window, but it's still fragile — any future scheduling jitter, increased mission_planner load, or third-party timing drift can push us back over.  A configurable timeout (default 500 ms for SITL, keep 60 ms for hardware-fidelity scenarios) would eliminate the entire failure class.  **How:** vendored-patch our submodule fork to read the timeout from `MultirotorPawnSimApi`'s settings JSON (which already passes other params).  Document the patch in `deploy/airsim_setup.sh` so it survives a clean checkout.  Follow up with an upstream Cosys-AirSim PR.
- **P2** — Surface `final_clamp_count()` (avoider) and `hover_fallback_count()` (planner) in `run_report.txt`. **Why:** Both counters were added during this debug push (PRs #646 and #650) specifically so operators can see post-flight whether the safety floor / hover-fallback is firing.  Currently the run-report parser doesn't read them — the only way to find their values is to grep the log manually.  Defeats half the diagnostic value.  **How:** add two lines to the run_report parser (`tests/scenarios/run_report_template.py` or wherever the parser lives) that scrape the last `[Avoider]`/`[PlanBase]` log line and surface the counters in the run-report observations section.
- **P2** — Add a scenario-runner check that `[UE5]` log does NOT contain "API call was not received". **Why:** This message is emitted by the simulator HUD AND visible only on screen during a manual run — easy to miss in a long mission.  Would have surfaced PR #651's root cause weeks earlier instead of through manual UI observation.  **How:** add a `must_not_contain` pattern to the scenario pass criteria scanning `cosys_ue5.log`.  Apply to all SITL scenarios, not just 33.
- **P3** — `fc_tx_thread` loop period and heartbeat threshold should be config-driven, not hard-coded.  PR #651 hard-coded `25 ms` loop and `40 ms` heartbeat for SITL.  **Why:** real-hardware FC links may want different cadences; future SITL backends may have different timeout windows; benchmark-harness scenarios should be able to suppress the heartbeat to test failure-mode handling.  **How:** add `comms.fc_tx.send_period_ms` to default.json with derived `kHeartbeatPeriod = std::max(period * 1.5, 30 ms)` so the two stay coupled.
- **P3** — Rename `direct_fallback_` flag and `using_direct_fallback()` method on `IGridPlanner` (and all derived classes / log lines / tests).  Kept in PR #650 to limit blast radius.  **Why:** the legacy name is a permanent landmine — anyone reading the code reasonably concludes "drone is flying a direct line through obstacles" when the actual behaviour is hover.  Cost 30 minutes of debugging in one of the early scenario-33 sessions.  **How:** mechanical rename to `is_no_path_hovering_` / `is_no_path_hovering()`.  Touches `IGridPlanner`, `GridPlannerBase`, `DStarLitePlanner`, `mission_state_tick.h`, ~10 unit-test sites.

### 2026-04-30 (#643 supersede — items not yet forward-ported)

PR #643 ("tracker tuning + diagnostics for scenario-33 ID-explosion") was branched on 2026-04-30 morning before the keystone fixes (#651/#660/#658/#659/#642 phase 4) landed.  Its scenario-config diff is now a regression vs current scenario 33 (would revert validated values for `position_clamp_m`, `min_confidence`, `instance_promotion_observations`, `max_static_cells`, `allow_radar_promotion`, `inflation_radius_m`, and the altitude/mask filters).  Closed as superseded.  Two remaining items captured here:

- **P2 (clock-jitter robustness, independent of perception accuracy)** — `VoxelInstanceTracker` ageing currently uses caller-supplied `now_ns` (P2 frame timestamp), not `steady_clock::now()`.  Under irregular frame delivery (load spikes, late callbacks), ageing fires at the wrong time.  Switching to `steady_clock::now()` decouples ageing from upstream timestamp jitter — but the naive switch conflicts with the #670 test pattern that drives ageing by passing future-`now_ns` values.  Right path: dependency-inject a clock function (default `steady_clock::now`) so production gets monotonic ageing while tests keep a fake clock.  Stands on its own jitter-robustness justification, separate from perception-accuracy.
- **P3 (observability for upstream-perception regressions)** — Tracker diagnostic counters: `last_aged_out_count_`, `last_match_failure_count_`, plus `last_aged_out=N, match_failures=M` in the P2 `[VoxelCluster]` log line.  These are the measurement that surfaces upstream-perception drift: high mint rate / frequent ageing = SAM confidence floor too lenient, projector emitting NaN voxels, or cluster-size minimum too low.  Surface when the next perception-accuracy regression needs investigation.

**Deliberately not carried forward:** #643's hypothesis to bump `tracker.max_match_distance_m` from 3.0 m to 8.0 m.  That was a band-aid for upstream noise (centroid drift exceeded 3 m/frame on the failing run).  Keystone fixes #658 (altitude filter) / #659 (mask size filter) / #660 (`min_confidence: 0.7`, `position_clamp_m: 30`) / #642 (instance gate) directly attack the centroid-instability source, so the gate should *not* need to be relaxed.  **Principle:** when the tracker mints excess IDs, fix upstream perception, never widen the association gate — a wider gate trades real-correctness (two pillars 5 m apart could merge into one tracked instance) for masking a perception bug.

### 2026-04-27 (#638 review-driven backlog)

The four-PR voxel-clustering stack (#639 / #640 / #641 / #642) had ~70 review findings across 8 reviewer agents.  P1 + high-value P2 fixes landed in the respective PRs; the items below were deemed correct-but-deferred (proactive backlog).

#### Phase 1 (#639) — voxel clusterer

- **P2** (declined for now): `uf_union` lacks union-by-rank; header docstring claims O(N·α(N)) but only O(N log N) holds without rank.  At our N (~5000 voxels/frame) the asymptotic gap is invisible, but the docstring should either be tightened or rank should be added.  Choose one when N grows or when a profiler shows the union pass dominating.  See DR-037.
- **P2**: extract `cluster_lidar_points()` analogue from `assign_instance_ids()` into a unit-testable free function so the inner Union-Find logic is exercised independently of the void-returning entry point.
- **P3**: replace `unordered_map<int,int> root_size` and `unordered_map<int,uint32_t> root_to_id` with flat `std::vector<int>` indexed by root.  Eliminates ~10k hash ops/frame at N=5000.
- **P3**: pre-`reserve()` `cell_to_voxel` to `N * 2` to reduce rehash/chaining cost in the 27-cell neighbour loop (135k lookups/frame at N=5000).
- **P3**: `assign_instance_ids` could return `Result<uint32_t>` (cluster count + error channel) instead of `void` — would also obviate the `count_clusters()` helper's second pass.
- **P3**: `count_clusters()` now O(N) distinct-id scan (correct, was max-id which broke under non-compact Phase 2 IDs).  Performance-equivalent at our N; flag if profiling shows it.
- **P3**: tests don't cover the Chebyshev=1 boundary in y/z axes — only along x.  A bug like `for (int dy = 0; dy <= 0; ...)` would not be caught.  Add a 3D adjacency test.
- **P3**: tests don't cover `min_pts=1` singleton-cluster boundary, single-voxel input, exact-eps boundary, or negative eps_m / min_pts.
- **P3**: `tracks/TESTS.md` not yet updated with `test_voxel_clusterer.cpp` (11 tests after #639 fixes) and the new baseline count.
- **P3**: `EnabledWritesOneJsonlLinePerBatch` style: tests use `instance_id = 0xDEADBEEFu` marker pattern only on the two disable tests; consider applying to other tests for consistency / future-proofing.
- **P3**: `pack_cell_key` 16-bit truncation now bounded by the kMaxCellIndex clamp at the call site (P1-A fix), but the function itself still silently truncates if called from elsewhere.  Either widen to 21 bits/axis (covers ±1M cells in 63 bits), or rename to `pack_cell_key_unchecked` to make the precondition explicit.

#### Phase 2 (#640) — voxel instance tracker

- **P2** (declined for now): hidden allocations in `update()` (4 local collections per call).  At N≤30 tracks the allocator pressure is ~40 alloc/s, well within budget.  Mirror the `VoxelClusterScratch` pattern when a profiler shows it matters.  See DR-038.
- **P3**: `Track` struct `observation_count` semantics — counts total re-associations, not consecutive frames.  If Phase 4 needs consecutive semantics for promotion gating, add a `consecutive_count` field.
- **P3**: `tracks_view` cache-hostile pointer-chasing pattern in inner match loop; a struct-of-vectors flat array would be faster at N=30 once profiled.
- **P3**: `next_stable_id_` wraparound check (today only triggers after 4B mints, but compounds with NaN-config DOS — handled defensively elsewhere; keep an explicit `++` wraparound assert as an additional layer).
- **P3**: split `update()` into `compute_candidates_` / `associate_to_tracks_` / `rewrite_voxel_ids_` private methods so association can be tested independently.
- **P3**: `default.json` missing `path_a.cluster` and `path_a.tracker` defaults; `cfg.get<>` falls back to ctor defaults silently.  Add the keys for visibility/auditability.
- **P3**: deterministic iteration via `std::map` for `candidates` and `frame_local_to_stable` would remove a class of test-flake risk (unordered_map order is implementation-defined).
- **P4**: `[[maybe_unused]]` on the `_` structured binding to silence potential `-Wshadow` warnings.

#### Phase 3 (#641) — instance-aware grid promotion

- **P2** (declined for now): `OccupancyGrid3D` ctor now takes 14 positional parameters.  Refactor to a `GridConfig` struct mirror.  Out-of-scope for #641; large blast radius.  See DR-039.
- **P3**: `instances_` map can grow unboundedly within a single mission leg.  `clear_instance_state()` is now wired to RTL/LAND (P1 fix), but a within-leg LRU/TTL eviction would bound worst-case memory in long-duration missions.
- **P3**: `tracked_instance_count()` / `promoted_instance_count()` accessors should be `noexcept` and `[[nodiscard]]` for consistency with the rest of the file.
- **P3**: structured binding `[id, rec]` doesn't use `id` — switch to `for (const auto& kv : instances_)` or `[[maybe_unused]]`.
- **P3**: `InstanceRecord` struct with one `uint32_t observation_count` field is borderline over-engineered today; keep for Phase 4+ extensibility (last_seen_frame_seq, total_voxels, etc.).
- **P3**: `set_promotion_paused()` / new instance-gate startup interaction in `mission_state_tick.h:79` overwrites startup pause every NAVIGATE tick.  Pre-existing pattern, not Phase 3-introduced; flag for separate refactor.
- **P3**: Add fault-injection scenario for the instance-promotion gate (mid-flight P2 restart should not bypass the freshly-cleared instance counters).

#### Phase 4 (#642) — scenario 33 enable

- **P2** (declined for now): No CI test that scenario JSON files load + reach consumers.  Parametrise `tests/test_config_validator.cpp` over `config/scenarios/*.json` to catch typos at unit-test time, not scenario-run time.  See DR-040.
- **P3**: PR body explicit binary-version dependency (Phase 1+2+3 must all be present).  Already addressed by stacked-PR merge order docs but a startup self-check log in P2/P4 for "instance_id is non-zero" would surface mismatches at runtime.
- **P3**: `_comment_instance_gate` operator note should include the ~1m promotion-latency-at-cruise figure for tuners.
- **P4**: `max_match_distance_m=3.0` is 15× the per-frame motion baseline — possibly too loose for nearby distinct pillars.  Tighten to 1.5 m if Phase 2 traces show ID flapping.

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
  3. Document the workaround near `FindAirSim.cmake` and in `docs/tutorials/DEV_MACHINE_SETUP.md`.
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

### 2026-05-16 — PR #776 rollup review deferrals (cold-start hardening epic)

The PR #776 (integration→main rollup of the cold-start hardening epic) review surfaced these items as P3 — valid suggestions we agreed with but chose not to bundle into the rollup. Each entry preserves the originating-review audit trail.

#### 1. `std::to_string` in pose-stale escalation log (perf, hot path)

- **Priority:** P3
- **Category:** code quality / performance
- **Source:** deferred from `review-performance` agent on PR #776 rollup review (pre-existing — not introduced by the epic, but flagged in the merged diff)
- **Current state:** `process4_mission_planner/include/planner/fault_manager.h` emits `DRONE_LOG_WARN("Pose stale " + std::to_string(...) + "ms")` inside the 50 Hz fault-evaluate path. The string concatenation allocates per evaluate-cycle even when the fault is not active.
- **Proposed fix:** switch to `DRONE_LOG_WARN("Pose stale {}ms", ms)` (fmt-style structured logging, already supported by the spdlog backend) — zero allocation on the non-trigger path.
- **When worth doing:** when fault_manager telemetry shows the per-tick allocation cost in a profiler, or as part of a broader DRONE_LOG_* fmt-migration sweep.
- **Why deferred:** pre-existing code, not introduced by the cold-start epic. Fixing in this rollup would expand the diff with no functional change to the epic itself.

#### 2. `FAULT_FC_` vs `FAULT_PLANNER_` prefix inconsistency in fault catalogue

- **Priority:** P3
- **Category:** api-contract / naming
- **Source:** deferred from `review-api-contract` agent on PR #776 rollup review
- **Current state:** `common/ipc/include/ipc/ipc_types.h` fault flags use mixed prefixes — `FAULT_FC_PREFLIGHT_TIMEOUT`, `FAULT_FC_LINK_LOST` (FC-prefixed because the cause is the flight controller side) versus `FAULT_PLANNER_STALL`, `FAULT_VIO_LOST` (subsystem-prefixed). A reader can't infer the convention from the names alone.
- **Proposed fix:** either drop the `FC_` infix to make all flags subsystem-prefixed (`FAULT_PREFLIGHT_TIMEOUT`, `FAULT_LINK_LOST`), OR add a doc-comment table at the top of the FaultFlags enum mapping each flag to its subsystem/owner. Option B is the no-code-churn fix.
- **When worth doing:** when a new fault flag lands and the author has to pause to decide which convention to follow, OR if a GCS-side telemetry consumer is built that needs to filter by subsystem.
- **Why deferred:** renaming touches >30 callsites across P4, P5, P7 + GCS protocol + scenario JSON expectations. Out of scope for a rollup PR.

#### 3. `docs/tracking/BUG_FIXES.md` Fix #718 entry — file list completeness

- **Priority:** P3
- **Category:** docs (audit trail accuracy)
- **Source:** deferred from `review-api-contract` agent on PR #776 rollup review
- **Current state:** the Fix #718 entry in BUG_FIXES.md lists the planner_stall_handler files but omits the FaultManager constructor additions (`POSE_STALE_TIMEOUT_MS`, etc.) and the IPC type additions (FAULT_PLANNER_STALL bit).
- **Proposed fix:** append the missing files to the Fix #718 entry's "Files changed" list.
- **When worth doing:** in the next docs sweep, or when an audit (cert, post-mortem) traces a finding back through BUG_FIXES.md and the missing files matter.
- **Why deferred:** BUG_FIXES.md is for the audit trail, not the diff. Cross-referencing files is a polish item.

#### 4. `tests/TESTS.md` delta footnote attribution precision

- **Priority:** P3
- **Category:** docs (test inventory)
- **Source:** deferred from `test-unit` agent on PR #776 rollup review
- **Current state:** the latest TESTS.md update notes "+18 tests for cold-start hardening" but doesn't break down which test file added which count. A future maintainer can't tell from the footnote whether the 18 came from PR #743 (Layer 1+2) or PR #763 (Layer 4) or PR #775 (Layer 5).
- **Proposed fix:** split the cold-start delta into per-PR footnotes:
  - PR #741/#743 — Layer 1 ARM debounce (+N tests in `test_mission_state_tick.cpp`)
  - PR #744/#750 — Layer 3 contact-sensor gate (+N tests in `test_contact_sensor_gate.cpp`)
  - PR #752 — Layer 2 P3 INITIALIZING-skip (+N tests in `test_p3_pose_init.cpp`)
  - PR #763 — Layer 4 settle gate (+N tests in `test_post_arm_settle.cpp`)
  - PR #775 — Layer 5 planner-stall (+N tests in `test_planner_stall_handler.cpp`)
- **When worth doing:** in the next TESTS.md sweep, or when a maintainer trying to git-bisect a test-count regression needs the attribution.
- **Why deferred:** TESTS.md format is currently rolled-up-by-month. Per-PR attribution is a format change, not a rollup change.

#### 5. End-to-end Layer 1→4 chain composition integration test

- **Priority:** P3
- **Category:** test-infra (integration coverage gap)
- **Source:** deferred from `test-scenario` agent on PR #776 rollup review
- **Current state:** each cold-start layer has unit tests for its individual gate (Layer 1 ARM debounce, Layer 2 INITIALIZING-skip, Layer 3 contact-sensor + stale-pose, Layer 4 attitude/velocity settle). There is no single integration test that runs a process-level fixture through all five gates in sequence with realistic cold-start timing (EKF2 flicker → INITIALIZING resolve → contact-sensor settle → ARM → settle gate → TAKEOFF).
- **Proposed fix:** add `tests/integration/test_cold_start_chain.cpp` that drives a `MissionStateTick` instance through the full pre-flight sequence with a mocked `FCState` source that replays the recorded cold-start traces from the 5/5 smoke sweep on PR #763.
- **When worth doing:** when a real bug crosses two layers (e.g. Layer 4 settles but Layer 2 hasn't really resolved) and we wish we had a regression test for the chain. File as a follow-up issue under epic #727 closeout.
- **Why deferred:** integration test infrastructure (fixture, traces) is non-trivial; the 5/5 Gazebo sweep already validated the chain end-to-end at the scenario level for the epic merge.
- **Action:** file as a new GitHub issue tagged `area:tests` `epic:727-followup` after the rollup merges.

---

## Resolved

### 2026-05-13

- **P3** — `COSYS_SIMULATION_ARCHITECTURE.md` (Cosys-specific Tier 3 runtime arch doc, sibling to the Gazebo `SIMULATION_ARCHITECTURE.md`) created. Resolved by #745 Phase F. Covers HAL backend mapping, scenarios #29/#30/#33, Tier 2 vs Tier 3 differences, build gating. The ground-truth-emitter section that pairs with the perception pipeline is intentionally deferred to the (currently gitignored) `perception_v2_detailed_design.md` and cross-linked.

### 2026-04-30

- **P2** — `ZenohSubscriber<T>::receive()` recorded misleading IPC latency on quiet topics (sticky `has_data_` + stale `timestamp_ns_` made polled latency grow at 1 s/sec wall-clock).  Fix: only record a latency sample when the timestamp has changed since the previous `receive()`.  Resolved by PR #644 (cherry-picked forward).
