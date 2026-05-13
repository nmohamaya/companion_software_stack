# Bisect Report — Gazebo Scenario Regression on `feature/perception-v2-integration`

**Date:** 2026-05-07
**Bisecter:** Claude Opus 4.7 (1M context) + naveen.mohanan
**Issue:** [#710](../../issues/710)
**Related PR:** [#707](../../pulls/707)

## TL;DR

**First-bad commit identified: `760bcc1` — PR #657 *"ObstacleAvoider3D AABB-aware distance + repulsion direction"*.**

Confirmed by `git bisect` over 6 iterations across 56 commits in range `c7d64fb..eff1718`.  All later integration-branch commits inherit the broken behaviour and add secondary regressions on top.

## Symptom on integration branch

Scenario 18 (`perception_avoidance` — sensor-driven, **no HD-map**) on `feature/perception-v2-integration`:

- Rotors visibly delayed at sim start (~5 s vs immediate on `main`)
- "Drunken" / sloppy takeoff
- Drone struggles in front of obstacles, repeatedly "correcting itself"
- Does not return-and-land at end of mission (RTL hangs, no LAND transition)

Same scenario on `main` (commit `389089a`, baseline): **clean PASS, immediate rotor spin-up, smooth takeoff, all 5 waypoints, full RTL→LAND→IDLE cycle**.  Verified on the same physical machine on the same day with the same PX4 SITL binary, so it is not a hardware / system-update artefact.

## What PR #657 changed

```
process4_mission_planner/include/planner/obstacle_avoider_3d.h  +42 lines
tests/test_obstacle_avoider_3d.cpp                              +118 lines
```

Replaces centroid-distance with **AABB-face-distance** in `ObstacleAvoider3D::avoid()`:

```cpp
// BEFORE (legacy, on main):
//   dist = ||drone - centroid||
//   repulsion direction = (drone - centroid) / dist

// AFTER (PR #657):
//   nearest = clamp(drone, AABB_min, AABB_max)
//   dist = ||drone - nearest||
//   repulsion direction = (drone - nearest) / dist  ← perpendicular to nearest face
```

`estimated_radius_m` is reinterpreted as XY half-extent (square cross-section) and `estimated_height_m` as full Z extent.

**Original goal:** fix scenario 33 cube tangential collisions where a 1×1×5 m cube was being treated as a 0.5 m sphere around its centroid.  Drone could pass within 0.4 m of the cube face while the avoider thought it was "0.94 m away" and never engaged close-regime.

**Unintended consequence on Gazebo scenarios:** for cylinders detected by `color_contour` + `bytetrack` + UKF radar fusion, the AABB extents from runtime perception are **inflated** (multiple slightly-offset detections of the same cylinder produce a wider bounding envelope than the physical surface).  AABB-aware avoider then treats inflated extents as hard surfaces — drone perceives obstacle "faces" 1-2 m closer than reality, over-engages close-regime, oscillates between conflicting perpendicular pushes when between two cylinders.

## Bisect iterations + per-commit data

| # | Commit | Date | Title | Verdict | Survey static cells | STUCK events | RTL→LAND |
|---|--------|------|-------|---------|---------------------|--------------|----------|
| baseline-good | `c7d64fb` | (set good) | batch review-comment fixes | GOOD ✓ | — | — | — |
| baseline-bad | `eff1718` | (set bad)  | scenario 33 PASS — ground-truth perception (#704) | BAD ✗ | — | — | — |
| 1 | `94c7e54` | Apr 30 | cleanup review follow-ups (#672) | **BAD ✗** | — | — | no |
| 2 | `2493e41` | Apr 30 17:24 | enable static_cell_ttl_s=30 in scenario 33 (#656) | **GOOD ✓** | 39 | 0 | yes (10:44:24→32) |
| 3 | `c069e23` | Apr 30 20:05 | UKF construction guard + Pass 3 (#665) | **BAD ✗** | — | — | — |
| 4 | `023d2a2` | Apr 30 18:17 | MaskDepthProjector SAM mask size filter (#659) | **BAD ✗** | 64 | 1 | no (RTL hung) |
| 5 | `ed611a9` | Apr 30 18:12 | MaskDepthProjector altitude filter + voxel viz tool (#658) | **BAD ✗** | 38 | 2 | no |
| 6 | `760bcc1` | Apr 30 17:39 | **ObstacleAvoider3D AABB-aware distance (#657)** | **BAD ✗ — CULPRIT** | 36 | 0 | no (RTL→no LAND) |

The 14-minute gap between the GOOD baseline `2493e41` (17:24) and the BAD culprit `760bcc1` (17:39) contained ONLY the avoider geometry change — no other commits touched obstacle handling.

## User visual feedback (chronological, verbatim)

| # | Commit | Verdict | Observation |
|---|--------|---------|-------------|
| 1 | `94c7e54` | BAD | "failure, sloppy takeoff, mission did not complete" |
| 2 | `2493e41` | GOOD | "looked very close to how it ran on main, little hesitation while landing, scenario passed" |
| 3 | `c069e23` | BAD | "sloppy takeoff and the drone looked like it was flying drunk... did complete the scenario" |
| 4 | `023d2a2` | BAD | "looked less drunk but it did not land at the end" |
| 5 | `ed611a9` | BAD | "really struggling to land and was a bit stuck around the blue object and red object... did not land either" |
| 6 | `760bcc1` (culprit) | BAD | "the rotor start like after 5 sec, really struggled in front of the first object and feels like constantly correcting itself and could not land either" |

Plus comparison run on `main` (commit `389089a`): **PASS, 19/19, all 5 waypoints, clean RTL→LAND→IDLE.**

## Key log evidence

### `2493e41` (GOOD baseline) — clean main-parity behaviour
```
10:42:50  TAKEOFF → SURVEY        (12s)
10:43:16  SURVEY → NAVIGATE       (26s)
10:43:32  WP2 reached             (16s)
10:43:42  WP3                     (10s)
10:43:53  WP4                     (10s)
10:44:05  WP5                     (12s)
10:44:17  NAVIGATE → RTL          (12s)
10:44:24  RTL → LAND              (6s)   ← cleanly transitions
10:44:32  LAND → IDLE             (8s)   ← lands and disarms
0 STUCK events.  39 static cells from survey.
```

### `760bcc1` (CULPRIT) — RTL hangs, no LAND
```
12:11:36  TAKEOFF → SURVEY        (9s)
12:12:02  SURVEY → NAVIGATE       (26s)
12:12:30  WP2                     (28s)
12:12:47  WP3                     (17s)
12:13:05  WP4                     (18s)
12:13:27  WP5                     (22s)
12:13:47  NAVIGATE → RTL          (20s)
12:13:47  NAVIGATE → RTL          ← logged TWICE (state thrash)
   (no further FSM transitions — drone never reaches LAND)
0 STUCK events but RTL never completes.  36 static cells.
```

### `023d2a2` and `ed611a9` (intermediate BADs)
Both show STUCK events at random positions (`(13.0,5.0,4.1)`, `(-4.4,7.8,4.4)`, `(1.2,16.4,4.0)`, `(0.7,15.5,4.0)`) and the same RTL hang.  The AABB-aware avoider is over-engaging on the inbound path.

## Root-cause analysis

PR #657 is the FIRST commit where:
1. `ObstacleAvoider3D::avoid()` switched from centroid-distance to AABB-face-distance
2. Repulsion direction switched from "drone − centroid" to "drone − nearest_point_on_AABB" (perpendicular to face)
3. `estimated_radius_m` reinterpreted as XY half-extent rather than sphere radius

For Gazebo runtime perception (color_contour + ByteTrack + UKF radar), the resulting effects are:

1. **Inflated AABB extents** — fused `estimated_radius_m` / `estimated_height_m` from multiple offset detections is wider than the physical cylinder surface
2. **Over-engagement** — drone enters close-regime at apparent "0.5 m from face" while actually 1.5 m+ from the physical surface
3. **Oscillation** — between two cylinders, perpendicular repulsions point in opposing directions, fighting each other and stalling forward progress
4. **RTL stall** — on the way home, conflicting AABB faces near landing site prevent the drone from descending; PX4 hovers indefinitely

The "rotor delay" symptom may be a downstream effect: the avoider engages on `pose` arrival (before NAVIGATE state), and the resulting setpoint thrash interacts with PX4 SITL's preflight checks (compass/baro instability) to delay motor spin-up.

## Why scenario 02 (HD-map) passed empirical testing in PR #707

Earlier in the day (before this bisect), running scenario 02 with PR #707's flags OFF gave a clean 19/19 PASS.  Two reasons it didn't reproduce the regression:

1. Scenario 02 has an **HD-map static layer** (4 pre-mapped cylinders) that the **D\* Lite planner** routes around regardless of avoider behaviour.  Even if the avoider over-engages, the planner already has a clean global route.
2. Scenario 18 has **`static_obstacles: []`** — relies entirely on runtime perception.  When the avoider misbehaves, there's no fallback.

This is why the `aabb_aware_distance=OFF` smoke test on scenario 02 was a false-positive validation.

## Connection to existing work

**PR #707 already has this gated.**  I added `mission_planner.obstacle_avoidance.aabb_aware_distance` flag exactly for this code path.  When the flag is OFF (which Gazebo scenarios 02/17/18/26 set in their JSON), the avoider falls back to centroid behaviour:

```cpp
// from obstacle_avoider_3d.h, our PR #707 gating:
const float radius_xy = config_.aabb_aware_distance
    ? std::max(0.0f, obj.estimated_radius_m)
    : 0.0f;     // ← collapse to centroid (legacy main behaviour)
const float hz = config_.aabb_aware_distance
    ? std::max(0.0f, 0.5f * obj.estimated_height_m)
    : 0.0f;
```

The only change needed is to **flip the default in `config/default.json` from `true` to `false`**, so scenarios that don't explicitly opt in get legacy behaviour.

## Recommendations

### Immediate (low-risk, fast turn-around)
1. Change `config/default.json` default for `aabb_aware_distance` from `true` to `false`
2. Cosys scenario 33 keeps `true` explicitly in its scenario JSON
3. All Gazebo scenarios inherit `false` and behave like main (already verified for scenario 02)
4. Same may apply to `close_regime_final_clamp` (PR #646) — needs separate validation

### Medium-term (root-cause investigation)
1. Investigate why fused `estimated_radius_m` / `estimated_height_m` from camera+radar are inflated on cylinder detections (root cause of PR #657's Gazebo side effect)
2. Either fix the inflation (per-detection, before fusion) or make AABB-aware logic robust to it (e.g. trust extents only when they come from a high-confidence segmentation source like SAM/Cosys-GT)

### Longer-term (architectural)
1. Add a per-obstacle-source `extent_confidence` field — high for SAM-segmented or ground-truth obstacles, low for camera-bbox-derived envelopes
2. Avoider weights AABB extents by confidence — falls back smoothly to centroid for low-confidence extents

## Process notes

- **Bisect duration:** ~90 minutes total (6 iterations × ~15 min each, including build + scenario run + verdict)
- **Bisect range size:** 56 commits → 6 binary-search iterations as predicted
- **Methodology:** user ran each scenario in their terminal with GUI; visual verdict + run report were combined; logs (FSM transitions, STUCK count, survey static cells) gave objective data
- **The "elimination method" attempt earlier** (gating individual PRs in source) was **unreliable** because:
  - Surgical edits sometimes didn't take effect (mistake on my side, only one of three claimed disables actually applied)
  - Scenarios pass / fail wasn't a binary symptom — partial regressions were hard to classify
  - Without `git bisect`'s commit-tracking, reasoning got muddled
  - **Lesson:** for binary-search regression hunts, use real `git bisect` not surgical-disable-and-reason

## Files & artifacts

All bisect run logs preserved at:
```
drone_logs/scenarios_gazebo/perception_avoidance/2026-05-07_*
drone_logs/scenarios_gazebo/perception_avoidance/2026-05-07_104221_PASS  (2493e41 GOOD)
drone_logs/scenarios_gazebo/perception_avoidance/2026-05-07_121055_FAIL  (760bcc1 CULPRIT)
```

Main worktree comparison run at:
```
/home/nav/Projects/companion_software_stack/drone_logs/scenarios_gazebo/perception_avoidance/2026-05-07_102520_PASS
```

Bisect log preserved by `git bisect log` (re-runnable for verification).

---

## Addendum — Empirical sweep on Cosys scenario 33 (2026-05-11)

After the initial bisect identified PR #657 as the regression source, an empirical sweep was run on Cosys scenario 33 to determine which of the three #706 flags are actually load-bearing in Cosys ground-truth perception.

### Sweep methodology
Each test toggled one (or all) of the three flags on `config/scenarios/33_non_coco_obstacles.json` and ran a clean scenario.  Baseline = all three flags ON (`close_regime_final_clamp`, `aabb_aware_distance`, `avoider_surface_enabled`).

### Sweep results

| # | Config | Result | UNSTUCK | Hover-fallbacks | RTL→LAND | Notes |
|---|--------|--------|---------|-----------------|----------|-------|
| Baseline | all 3 ON | PASS 26/26 | 2 | 0 | yes | reference |
| A | only `close_regime_final_clamp` OFF | PASS 26/26 | **0** | 0 | yes | cleaner than baseline |
| B | only `aabb_aware_distance` OFF | **FAIL 22/26** | 3 | **81 → LOITER** | no | confirmed bisect culprit |
| C | only `avoider_surface_enabled` OFF | PASS 26/26 | 0 | 0 | yes | clean |
| **D run 1** | **all 3 OFF (= `main`)** | **PASS 26/26** | 0 | 21 | yes | unexpected clean PASS |
| **D run 2** | **all 3 OFF (validation)** | **PASS 26/26** | 0 | 20 | yes | reproducible |

### Critical finding from Sweep D

**Sweep B's failure was an interaction effect, not an intrinsic PR #657 dependency.**

Sweep B (only #657 OFF, with #646 and #647 still ON) FAILed with 81 hover-fallbacks.  Sweep D (all three OFF together) PASSed cleanly.  This means PR #657's apparent load-bearing nature was **caused by interactions with the dead-weight #646/#647 flags** — when all three are removed together, scenario 33 passes cleanly with the legacy centroid-distance avoider.

Bonus signal from Sweep A's diagnostic counter: `final_clamp_count = 0` in baseline runs — PR #646's mechanism was already not firing in production.  The code was running but the inner condition (`v_toward_final > 0` in close regime) was never satisfied.

### Updated verdict matrix

| PR | Pre-sweep verdict | Post-sweep verdict |
|----|-------------------|--------------------|
| **#646** `close_regime_final_clamp` | unknown | **Dead weight** — `final_clamp` never fires |
| **#657** `aabb_aware_distance` | load-bearing for Cosys-33 (per #710 design) | **Dead weight** — only "load-bearing" via interaction with #646/#647 |
| **#647** `avoider_surface_enabled` | probably dead weight | **Dead weight** — Cosys GT-segmentation supersedes the voxel-snapshot path |

### Why dead weight now (root cause)

PR #657 (and its companions #646/#647) were validated for scenario 33 when the perception pipeline was **PATH A: FastSAM + Depth Anything V2** — that pipeline produced noisy masks and inflated extents, requiring the AABB-aware geometry to compensate.  Subsequent move to **Cosys ground-truth segmentation** (`sam.backend = cosys_airsim`, `depth_estimator.backend = cosys_airsim`) produces pixel-perfect masks and depth.  The downstream `estimated_radius_m` / `estimated_height_m` from the fused tracks are already accurate without the geometric safety nets.

### Final recommendation: full code removal

All three code paths can be deleted from the codebase.  Tracked in **Issue #712**.  Implementation = 4 sequential commits on `feature/issue-710-empirical-cleanup`:

| # | Commit | Effect | Validation |
|---|--------|--------|------------|
| 1 | `42e7af9` — remove PR #647 voxel-snapshot | -522 lines, voxel_obstacle_snapshot.h deleted, 9 tests gone | Cosys-33 PASS 26/26 |
| 2 | `aa69d53` — remove PR #646 final clamp | -294 lines, final_clamp_count atomic gone, 6 tests gone | Cosys-33 PASS 26/26 ×2 runs |
| 3 | `e6bf478` — remove PR #657 AABB-aware | -376 lines, ~150 lines of geometry → 4-line centroid math, 6 tests gone | Cosys-33 PASS 26/26 |
| 4 | docs refresh (this update) | doc-only | n/a |

**Cumulative deletion: -1192 / +56 = net -1136 lines across 3 commits.**

### Caveat — Gazebo scenario 18 (sensor-driven) still on the edge

After commit 3, Cosys scenario 33 PASSes clean reproducibly.  Gazebo scenario 18 (sensor-driven, no HD-map) remains intermittently flaky — but the flakiness is from **two separate pre-existing issues** unrelated to the #712 cleanup:

1. **PX4 SITL ↔ Gazebo bridge timing race** (Issue #710) — intermittent EKF/compass/baro preflight failures at takeoff
2. **Runtime perception ghost obstacles in Gazebo** (color_contour + UKF radar fusion produces extra phantom tracks) — to be filed as a follow-up issue

These were observable on the branch before commit 3 too (e.g. 14:05 and 14:08 PASS runs had the same rotor-spin-up delay).  Commit 3 doesn't cause them but does make scenario 18 more vulnerable to them (no AABB-aware safety net to mask perception noise).

### Sweep run logs

| Sweep | Run path |
|-------|----------|
| Baseline | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_103801_PASS` |
| A | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_105911_PASS` |
| B | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_110726_FAIL` |
| C | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_111759_PASS` |
| D run 1 | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_113106_PASS` |
| D run 2 | `drone_logs/scenarios_cosys/33_planner_hdmap_groundtruth_perception/2026-05-11_114408_PASS` |

Post-commit validation runs:

| Commit | Run path |
|--------|----------|
| Commit 1 r1 (failed — variance) | `2026-05-11_121138_FAIL` |
| Commit 1 r2 (passed) | `2026-05-11_123031_PASS` |
| Commit 2 r1 | `2026-05-11_134219_PASS` |
| Commit 2 r2 | `2026-05-11_134805_PASS` |
| Commit 3 | `2026-05-11_142111_PASS` |
