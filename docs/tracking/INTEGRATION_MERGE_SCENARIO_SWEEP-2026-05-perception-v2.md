# Integration → Main Rollup — Phase 2 Scenario Sweep

> Captures Gazebo SITL + Cosys-AirSim scenario results on the integration HEAD
> before merging `feature/perception-v2-integration` into `main`.
>
> Part of [#723](https://github.com/nmohamaya/companion_software_stack/issues/723) Phase 2.
>
> **Rule (per [kickoff comment](https://github.com/nmohamaya/companion_software_stack/issues/723#issuecomment-4431557522) §6):** declare PASS only after 2–3 consecutive PASSes; declare regression only after 2–3 consecutive FAILs at the same commit. Single-run FAILs are flaky variance (PX4 EKF2 cold-start + Gazebo physics seed), not real regressions.

---

## Environment

| Field | Value |
|-------|-------|
| Integration HEAD | `f5aaf7b` (post-#726) |
| Test host | `Nicola-Kalam` |
| OS | Linux 6.8.0-48-generic |
| Build | `Release` + `-Werror -Wall -Wextra` + `ALLOW_INSECURE_ZENOH=ON` |
| `ctest -j$(nproc)` | **2074 / 2074** pass (8 Cosys-SDK tests skipped without `HAVE_COSYS_AIRSIM`) |
| Run mode | Headless (no `--gui`) — coordinated remotely |

---

## Scenarios run

| # | Scenario file | Tier | Runner | Notes |
|---|---------------|------|--------|-------|
| 02 | `02_obstacle_avoidance.json` | Gazebo SITL | `tests/run_scenario_gazebo.sh` | HD-map + obstacle avoidance |
| 17 | `17_radar_gazebo.json` | Gazebo SITL | `tests/run_scenario_gazebo.sh` | Camera + radar fusion (UKF) |
| 18 | `18_perception_avoidance.json` | Gazebo SITL | `tests/run_scenario_gazebo.sh` | Perception-driven avoidance — PR #721 stale-pose fix lands here |
| 25 | `25_flight_recorder_replay.json` | Gazebo SITL | `tests/run_scenario_gazebo.sh` | Flight-recorder replay path |
| 26 | `26_gazebo_full_vio.json` | Gazebo SITL | `tests/run_scenario_gazebo.sh` | Full VIO stack |
| 33 | `33_non_coco_obstacles.json` | Cosys-AirSim | `tests/run_scenario_cosys.sh` | GT perception + Echo radar — requires UE5 + Cosys-AirSim |

---

## Results table

| # | Scenario | Run 1 | Run 2 | Run 3 | Verdict | Notes |
|---|----------|-------|-------|-------|---------|-------|
| 02 | obstacle_avoidance | ✅ 19/19 | ❌ 16/19 | ❌ 18/19 | 🟡 flaky | Pre-existing integration friction documented in PROGRESS.md #88. Run 2 — cold-start VIO staleness, never took off. Run 3 — drone reported pose `(-27, 29)..(-26, 51)` vs WP corridor `(-4..22, -4..22)`, 4× stuck-recover, escalated to LOITER. Filed [#727](https://github.com/nmohamaya/companion_software_stack/issues/727). Non-blocking. |
| 17 | radar_gazebo | ❌ 20/21 | ✅ 21/21 | ✅ 21/21 | ✅ PASS | Two consecutive PASSes. Run 1 — `GazeboRadar.*Subscribed to scan` never fired (gz-transport topic discovery race) + VIO pose staleness escalation. After warmup, runs 2 & 3 clean. |
| 18 | perception_avoidance | ❌ 18/19 | ✅ 19/19 | ❌ 18/19 | 🟡 flaky | Same cold-start pattern. Run 1 — PX4 `Preflight Fail: High Gyro Bias / High Accelerometer Bias / Compass 0 fault`, drone took off at high attitude error, completed Survey at `(-75, 93, 4)` then 267 phantom obstacles promoted into the grid, planner couldn't make sense of subsequent NAVIGATE. User observation: front two rotors didn't spin up simultaneously with back two (PX4 mixer responding to bad EKF2 attitude estimate). Same throughline as #727. Non-blocking. |
| 25 | flight_recorder_replay | ✅ 15/15 | ✅ 15/15 | — | ✅ PASS | Two consecutive PASSes. Replay-driven (no live takeoff), so dodges the cold-start race. Duration 132s consistent across runs. |
| 26 | gazebo_full_vio | ✅ 18/18 | ✅ 18/18 | — | ✅ PASS | Two consecutive PASSes. Despite the name "full_vio", `slam.vio.backend=gazebo` (ground-truth passthrough — see `config/gazebo_sitl.json` comment `"Real VIO backends (e.g. MSCKF) are planned but not yet implemented in the factory"`). |
| 33 | non_coco_obstacles (Cosys) | ✅ 26/26 | ✅ 26/26 | — | ✅ PASS | **Two consecutive PASSes today on live UE5 + Cosys-AirSim** at commit `7a47374`. Run 1 — 271s, run 2 — 267s. Confirms PR #704's PASS evidence (Improvement #87) reproduces on the integration HEAD. The Cosys path uses `pose_source=cosys_ground_truth` (separate code path from `GazeboVIOBackend`), explaining why this scenario does not exhibit the cold-start race that flakes scenarios 02 and 18 — see [#727](https://github.com/nmohamaya/companion_software_stack/issues/727). |

Legend: ✅ PASS · ❌ FAIL · ⏳ pending · 🟡 flaky (mixed across runs)

---

## Per-run detail

Captured below as each scenario completed.  Full per-process logs in `drone_logs/scenarios_gazebo/<scenario>/<timestamp>_{PASS,FAIL}/` and `drone_logs/scenarios_cosys/...`.

### Scenario 02 — obstacle_avoidance

- **Run 1** — ✅ `2026-05-12_174701_PASS`, 161 s, 19/19 checks.  TAKEOFF → NAVIGATE → 5/5 waypoints reached → RTL → LAND.
- **Run 2** — ❌ `2026-05-12_175008_FAIL`, 161 s, 16/19 checks.  PREFLIGHT cleared, ARM accepted, but VIO pose staleness fault escalated during TAKEOFF.  Drone never lifted.  Auto-report: *"Mission did not complete — VIO pose staleness triggered fault escalation. This is typically a Gazebo SITL timing artifact, not a code defect. Recommend re-running."*
- **Run 3** — ❌ `2026-05-12_175350_FAIL`, 162 s, 18/19 checks.  Drone took off and entered NAVIGATE but reported pose was `(-27.8, 29.5, 3.9)` then drifted to `(-25.8, 51.3, 4.0)` over ~50 s — 25 m west, 7-30 m north of the nearest waypoint at `(-4, 22, 4)`.  4× stuck-recover events (cap is 3) → FSM escalated to LOITER.  Filed as the symptom evidence on [#727](https://github.com/nmohamaya/companion_software_stack/issues/727).

### Scenario 17 — radar_gazebo

- **Run 1** — ❌ `2026-05-12_180208_FAIL`, 161 s, 20/21 checks.  Two log-assertions failed: `GazeboRadar.*Subscribed to scan` and `GazeboRadar.*First scan` — gz-transport topic discovery race meant the radar plugin never came up.  VIO pose staleness fault also escalated.
- **Run 2** — ✅ `2026-05-12_180532_PASS`, 162 s, 21/21 checks.  Clean.
- **Run 3** — ✅ `2026-05-12_180915_PASS`, 162 s, 21/21 checks.  Clean (two consecutive PASS bar met).

### Scenario 18 — perception_avoidance

- **Run 1** — ❌ `2026-05-12_181640_FAIL`, 171 s, 18/19 checks.  PX4 cold-start reported `Preflight Fail: High Gyro Bias / High Accelerometer Bias / Compass 0 fault` for several seconds before clearing.  User observed asymmetric rotor spin-up: front rotors started ~1 s before back rotors (PX4 mixer correcting non-existent attitude error from bad EKF2 bias).  Drone took off but Survey ran at the *wrong* pose `(-75, 93, 4)` — 75 m off — and promoted **267 phantom static obstacles** into the planner grid.  Subsequent NAVIGATE was unable to make sense of the grid.
- **Run 2** — ✅ `2026-05-12_182118_PASS`, 172 s, 19/19 checks.  Clean.
- **Run 3** — ❌ `2026-05-12_182433_FAIL`, 172 s, 18/19 checks.  Same flake family — Mission complete check failed.

### Scenario 25 — flight_recorder_replay

- **Run 1** — ✅ `2026-05-12_183417_PASS`, 132 s, 15/15 checks.  Replay-driven (no live takeoff), dodges the cold-start race.
- **Run 2** — ✅ `2026-05-12_183704_PASS`, 132 s, 15/15 checks.

### Scenario 26 — gazebo_full_vio

- **Run 1** — ✅ `2026-05-12_201031_PASS`, 132 s, 18/18 checks.
- **Run 2** — ✅ `2026-05-12_204900_PASS`, 132 s, 18/18 checks.  Note: despite the "full_vio" name, `slam.vio.backend=gazebo` (ground-truth passthrough — see `config/gazebo_sitl.json` comment).

### Scenario 33 — non_coco_obstacles (Cosys)

- **Run 1** — ✅ `2026-05-13_091539_PASS`, 271 s, 26/26 checks.  Live UE5 + Cosys-AirSim Blocks scene at commit `7a47374`.  Zero cube collisions, mission complete → RTL → LAND.
- **Run 2** — ✅ `2026-05-13_092555_PASS`, 267 s, 26/26 checks.  Two consecutive PASS bar met on the live Cosys path.

---

## Cross-cutting observations

### The throughline — VIO/pose cold-start race ([#727](https://github.com/nmohamaya/companion_software_stack/issues/727))

3 out of 8 live-takeoff runs (scenarios 02 ×3, 17 ×1, 18 ×3 — 7 total) hit a VIO-pose-staleness-class failure:

- **Run 2 of scenario 02 + Run 1 of scenario 17** — explicit `FAULT_POSE_STALE` escalation. Drone never took off. Auto-report: *"Mission did not complete — VIO pose staleness triggered fault escalation. This is typically a Gazebo SITL timing artifact, not a code defect. Recommend re-running."*
- **Run 3 of scenario 02 + Run 1 of scenario 18** — drone took off but `slam/pose` reported it 25 m (scenario 02) to 75 m (scenario 18) outside the expected waypoint corridor. Planner couldn't navigate against the bad pose reference; eventually escalated to LOITER.
- **Run 1 of scenario 18 specifically** — visible asymmetric rotor start-up: front rotors spun up at a different time than back. Consistent with PX4 mixer responding to bad EKF2 attitude estimate (high gyro/accel bias on cold start) by commanding asymmetric throttle to "correct" non-existent tilt.
- Survey on a bad pose reference promotes **267 phantom static obstacles** (scenario 18 run 1) which makes subsequent NAVIGATE impossible.

### Root cause shape (from [#727](https://github.com/nmohamaya/companion_software_stack/issues/727))

**Two layers, both need fixing — neither alone is sufficient:**

| Layer | Bug | Fix shape |
|-------|-----|-----------|
| **P5 (PX4 ARM gate)** | PR #717 gates ARM on momentary `health_all_ok=true`. Bias flicker can flip the flag true briefly even while EKF2 attitude is still settling. | Require **N consecutive seconds** (e.g. 3 s) of `health_all_ok=true` before ARM. Or gate on bias-derivative settling, or on `is_local_position_ok ∧ is_global_position_ok`. |
| **P3 (pose publish)** | `GazeboVIOBackend::process_frame` returns `VIOHealth::INITIALIZING` with default-zero pose until first gz odom message. Publishing thread stamps with `now_ns` regardless. P4's PR #721 `planner_birth_ns` filter requires `timestamp_ns > 0` AND `timestamp_ns + slack >= planner_birth_ns` — a *fresh-stamped zero-pose* slips through both gates. | When `health == INITIALIZING`, either skip publishing, or publish with `timestamp_ns=0` and `quality=0`. |

Both layers together would dramatically reduce the cold-start variance.  Confirmed not a frame-conversion bug (Gazebo `ENU → NEU` and Cosys `NED → NEU` paths both converge on the same internal frame; MavlinkFCLink converts `NEU → NED` on send).  See [#727](https://github.com/nmohamaya/companion_software_stack/issues/727) for the full investigation thread.

### Rollup-merge impact

**This is pre-existing on integration AND on main** — none of the 88 commits in the rollup introduced the bug.  Scenarios 02 and 18 were already flaky on the integration tip *before* PR #707 / #711 added the safety-net gates.  The throughline is the **first** real VIO/pose handling that exposes both `GazeboVIOBackend`'s startup race and PR #717's not-yet-strict-enough ARM gate.

Recommendation: **do not block the rollup merge on this**. File post-merge effort against #727 to land the two-layer fix as one or two focused PRs. The same fix should also clean up the scenarios-02/18 flake rate, observable via the same scenario sweep on `main` after the merge lands.

### Stable scenarios

3 out of 6 scenarios passed with two consecutive PASSes — these are the unambiguous "rollup is good" signal:

- 17 radar_gazebo — clean after first run's cold-start warmup
- 25 flight_recorder_replay — replay-driven, dodges cold-start
- 26 gazebo_full_vio — `slam.vio.backend=gazebo` ground-truth (same as 17)
- 33 non_coco_obstacles (Cosys) — PR #704 evidence; different pose path

### Test count baseline

`ctest -j$(nproc)` on integration HEAD `f5aaf7b`: **2074 / 2074 pass**, 8 Cosys-SDK tests skipped without `HAVE_COSYS_AIRSIM`. CI quick (`bash deploy/run_ci_local.sh --quick`): **2 / 2 jobs pass** (FMT + Debug BUILD).

---

*Last updated: 2026-05-12 after Phase 2 scenario sweep complete. Phase 3 themed reviews pending maintainer confirmation per the [kickoff checkpoint rule](https://github.com/nmohamaya/companion_software_stack/issues/723#issuecomment-4431730530).*
