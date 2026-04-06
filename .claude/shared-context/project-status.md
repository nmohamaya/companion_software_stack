# Project Status

Current state of the project. Updated each session. Read at session start for alignment.

**Last updated:** 2026-04-06

## Key Metrics

- **Test count:** 1259 (baseline in `tests/TESTS.md` — verify after every build)
- **Scenario configs:** 25+ in `config/scenarios/`
- **Processes:** 7 (P1-P7), 21 threads total
- **Open PRs:** 0
- **Open issues:** ~15 active

## Recently Completed

- **Epic #263 (Autonomous Intelligence):** Wave 1 complete (8 PRs merged via integration branch). Includes YOLOv8, collision recovery, D* Lite fix, gimbal tracking, dynamic obstacle prediction, flight recorder, radar-only tracks, P3 rate clamping.
- **Multi-agent pipeline (#360):** Full `--pipeline` mode with 5 human checkpoints, cross-agent context injection, test+review agents running in parallel. PRs #359, #361, #362, #364 merged.
- **Audit PRs (#353-356):** Code quality, CI improvements, replay dispatch module. Test count 1259.
- **Epic #237 (Sensor Fusion):** Radar integration complete, camera+radar UKF fusion, 4-tier depth estimation.

## Active Epics

### Epic #263 — Autonomous Intelligence & Sim Fidelity
- **Status:** Wave 1 complete. Waves 2-3 pending.
- **Wave 2:** #191 (Gazebo stereo VIO) — depends on #220 (merged)
- **Wave 3:** #254 (covariance quality) + #255 (remove IVisualFrontend) — depends on #191
- **Integration branch:** `integration/epic-256-autonomous-intelligence`
- **Critical path:** #191 → #254

### Epic #300 — Platform Modularity & Adaptability
- **Status:** Planning complete. Score 34/60 (57%).
- **Strengths:** HAL (5/5), factories (5/5), error handling (5/5)
- **Gaps:** logging (2/5), clock (2/5), multi-vehicle (1/5)
- **Sub-epics:** A (ILogger+IClock+Config), B (CMake+TopicResolver), C (ProcessBuilder+EventBus), D (ISerializer+PluginLoader), E (Docker+cleanup)
- **Wave 1:** Sub-Epic A (foundational, no deps) + Sub-Epic E (independent)

## Blocking Bugs

| Issue | Problem | Impact |
|-------|---------|--------|
| #339 | Occupancy grid over-promotion causes corridor blockage | D* Lite failures, waypoint unreachable |
| #340 | False cell promotion from ground features | 616 ghost cells with 4 real obstacles |
| #338 | Goal snapping prefers nearest-to-start not nearest-to-goal | Suboptimal paths |
| #337 | No yaw-towards-travel for forward camera | Obstacle visibility gaps |

These are all related to the perception→planning pipeline. #339/#340 are the highest priority — the false promotion bug causes cascading failures.

## Priority Queue (Next Work)

1. **#339/#340** — Fix false cell promotion (root cause in color_contour + depth clamping)
2. **#191** — Gazebo stereo VIO (Wave 2 of Epic #263, unblocks #254)
3. **#337/#338** — Planner improvements (yaw-towards-travel, goal snapping)
4. **#335** — Per-customer deployment scripts (Epic #300-E5)
5. **Epic #300 Wave 1** — ILogger, IClock, Config keys (foundational modularity)

## Architecture Decisions in Effect

- **Integration branches** for multi-issue features — keeps main demo-ready
- **Pipeline workflow** (`deploy-issue.sh --pipeline`) — standard for all new work
- **HAL backend selection** via config — always maximize stack coverage in simulation
- **Camera=bearing, radar=range** fusion architecture — covariance-driven UKF
- **Don't fix code bugs by changing config** — investigate root cause first

## Workarounds in Place

- Scenario 02: `promotion_hits=0` (disables promotion, HD-map has all obstacles)
- Scenario 18: `max_static_cells=800` (caps promoted cells)
- DIAG logging gated to `spdlog::debug` (PR #355) — hidden at default INFO
- `TripleBufferTest.HighContentionStress` is a known flaky test — not caused by your changes
