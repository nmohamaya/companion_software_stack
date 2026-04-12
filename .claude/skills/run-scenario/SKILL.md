<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

---
name: run-scenario
description: Run Gazebo SITL scenario integration tests with structured analysis — launch, monitor, parse results, diagnose failures, compare runs
argument-hint: "<scenario-number|all|--list> [--compare <run1> <run2>] [--gui] [--tier <1|2>]"
---

# /run-scenario — Scenario Integration Test Runner with Analysis

Run scenario integration tests and provide structured analysis of results. This skill wraps `tests/run_scenario.sh` (Tier 1 simulated + Tier 2 Gazebo) and `tests/run_scenario_gazebo.sh` (Tier 2 Gazebo-only) with intelligent monitoring, result parsing, failure diagnosis, and cross-run comparison.

## Arguments

Parse `$ARGUMENTS` to extract:
- **scenario number** (e.g., `02`, `18`) — runs that specific scenario
- **`all`** — runs all 25 scenarios
- **`--list`** — list available scenarios without running
- **`--tier 1`** or **`--tier 2`** — run only Tier 1 (20 simulated) or Tier 2 (5 Gazebo) scenarios
- **`--compare <path1> <path2>`** — compare two previous run directories side-by-side
- **`--gui`** — launch Gazebo with 3D visualizer window
- **`--verbose`** — extra detail in output
- **`--dry-run`** — parse scenario config, show plan, don't execute

If no arguments, show the scenario list and ask what to run.

## Steps

### Step 1: List / Validate Scenarios

If `--list` or no arguments:
```bash
ls config/scenarios/*.json | sort
```

Parse each scenario's JSON to extract name, tier, description, and timeout. Present as table:

```
Available Scenarios (25 total: 20 Tier 1, 5 Tier 2):

| # | Name | Tier | Timeout | Description |
|---|------|------|---------|-------------|
| 01 | nominal_mission | 2 | 120s | Basic takeoff → navigate → RTL |
| 02 | obstacle_avoidance | 2 | 150s | D* Lite through 4-obstacle HD-map field |
| ... | ... | ... | ... | ... |
```

If a specific scenario number was given, verify the config file exists:
```bash
ls config/scenarios/<number>_*.json
```

### Step 2: Pre-Flight Checks

Before launching:

1. **Build exists:** Check `build/bin/` contains all 7 process binaries
2. **PX4 available:** Check `$PX4_DIR` exists (default: `~/PX4-Autopilot`)
3. **Gazebo available:** Check `gz sim --version` works
4. **No stale processes:** Check for running drone processes (`pgrep -f "video_capture|perception|slam_vio|mission_planner|comms|payload_manager|system_monitor"`)

If any check fails, report and ask whether to proceed or fix first.

### Step 3: Launch Scenario

Choose the correct script based on tier:

**Single scenario (any tier):**
```bash
# Tier 1 (simulated, no Gazebo needed):
./tests/run_scenario.sh config/scenarios/<number>_<name>.json [--verbose]

# Tier 2 (Gazebo SITL required):
./tests/run_scenario_gazebo.sh config/scenarios/<number>_<name>.json [--gui] [--verbose]
```

Check the scenario JSON's `scenario.tier` field to determine which script to use. Tier 1 scenarios have `"requires_gazebo": false`, Tier 2 have `"requires_gazebo": true`.

**Running multiple scenarios (`--all` or `--tier`):**
```bash
# All Tier 1 scenarios (no Gazebo):
./tests/run_scenario.sh --all --tier 1

# All Tier 2 scenarios (Gazebo):
./tests/run_scenario_gazebo.sh --all [--gui]

# All tiers:
./tests/run_scenario.sh --all --tier 1 && ./tests/run_scenario_gazebo.sh --all [--gui]
```

**Monitoring during execution:**
- Show periodic status updates (the script logs to `drone_logs/scenarios_gazebo/`)
- Report when key milestones are reached (TAKEOFF, NAVIGATE, waypoint advances, RTL, LAND)
- Alert immediately on EMERGENCY_LAND, OBSTACLE COLLISION, or process crash

### Step 4: Parse Results

After the scenario completes, read the structured outputs:

1. **Run metadata:** `<run_dir>/run_metadata.json` — result, duration, pass/fail counts
2. **Run report:** `<run_dir>/run_report.txt` — human-readable analysis from `lib_scenario_logging.sh`
3. **Process logs:** Individual process logs in the run directory

Present a structured summary:

```
═══ Scenario Result: <name> ═══

Status: PASS (12/12 checks passed)
Duration: 87s (timeout: 150s)
Git: abc1234 (feature/issue-381-deploy-issue-skill)

--- Mission Timeline ---
00:05  PREFLIGHT → TAKEOFF
00:13  TAKEOFF → SURVEY (+8s)
00:15  SURVEY → NAVIGATE (+2s)
00:22  Advanced to waypoint 1/5 (+7s)
00:38  Advanced to waypoint 2/5 (+16s)
00:55  Advanced to waypoint 3/5 (+17s)
01:09  Advanced to waypoint 4/5 (+14s)
01:18  Advanced to waypoint 5/5 (+9s)
01:18  NAVIGATE → RTL (+0s)
01:25  Mission complete

--- Perception ---
Detections: 847 total, 4 unique tracks
Radar: 312 returns, 289 after ground filter
Fusion: UKF active, 4 tracks fused

--- Occupancy Grid ---
Dynamic peak: 12 cells
Static peak: 24 cells (HD-map: 16, promoted: 8)
Promotion events: 3

--- Navigation ---
Planner: D* Lite, 6 replans
Avoider: ObstacleAvoider3D, 14 corrections (max 0.8 m/s)
Min obstacle distance: 3.2m (safe — min_distance: 2.0m)

--- Verification Checks (12/12) ---
✓ "Path planner: DStarLitePlanner" found
✓ "Obstacle avoider: ObstacleAvoider3D" found
✓ "backend: bytetrack" found
✓ All 7 processes alive at end
✓ No EMERGENCY_LAND
✓ No OBSTACLE COLLISION
✓ Mission complete
[...]
```

### Step 5: Failure Diagnosis

If the scenario **FAILED**, perform root cause analysis:

1. **Which checks failed?** List each failed verification check with context
2. **Last process state:** For each process, show the last 10 log lines before failure
3. **FSM state at failure:** What state was the mission planner in?
4. **Perception state:** Were detections being generated? Was fusion active?
5. **Common failure patterns:**
   - "Mission did NOT complete" + no crash → stuck in a state (check waypoint progress)
   - "EMERGENCY_LAND" → fault escalation (check fault events in log)
   - "OBSTACLE COLLISION" → avoidance failure (check min distance, grid state)
   - Process not alive → crash (check for SIGSEGV/SIGABRT in log)
   - Timeout → too slow (check replanning frequency, perception latency)

Present diagnosis:
```
--- Failure Diagnosis ---
Root cause: Mission stuck at waypoint 3/5 for >60s
Evidence:
  - Last FSM transition: NAVIGATE at 00:55
  - No waypoint advance after 00:55
  - D* Lite replanned 47 times (normal: 5-10) — path oscillation
  - Occupancy grid had 156 static cells (max_static_cells: 800 not hit, but high density)
Likely issue: False cell promotion near waypoint 3 blocking all paths
Suggestion: Check perception logs for ground detections being promoted
```

### Step 6: Compare Runs (if --compare)

If `--compare` was specified, load both run directories and present side-by-side:

```
═══ Comparison: <run1> vs <run2> ═══

| Metric | Run 1 | Run 2 | Delta |
|--------|-------|-------|-------|
| Result | PASS | FAIL | ▼ |
| Duration | 87s | 142s (timeout) | +55s |
| Waypoints | 5/5 | 3/5 | -2 |
| D* replans | 6 | 47 | +41 |
| Static cells peak | 24 | 156 | +132 |
| Min obstacle dist | 3.2m | 1.1m | -2.1m |
| Detections | 847 | 1203 | +356 |
| Promotions | 3 | 28 | +25 |

Key differences:
1. Run 2 has 6.5x more cell promotions — likely false promotion bug
2. Run 2 has 8x more D* replans — path oscillation from dense grid
3. Run 2 min distance 1.1m < 2.0m threshold — near-collision
```

### Step 7: Summary and Next Steps

After all scenarios complete (especially for `--all`):

```
═══ Scenario Summary ═══

| # | Scenario | Result | Duration | Notes |
|---|----------|--------|----------|-------|
| 01 | nominal_mission | PASS | 78s | |
| 02 | obstacle_avoidance | PASS | 87s | |
| 03 | battery_degradation | PASS | 45s | |
| ... | ... | ... | ... | ... |
| 18 | perception_avoidance | FAIL | 142s | Stuck at WP3 |

Total: 24/25 PASS, 1 FAIL
Failed: #18 perception_avoidance (false cell promotion)
```

Suggest next steps:
- If all pass: "All scenarios green. Ready for `/production-readiness` audit."
- If failures: "Suggest filing issues for failures: `/create-issue bug: <failure description>`"
- If comparing: "Regression detected in scenario #18 between commits X and Y"

If the user provided arguments, use them as context: $ARGUMENTS
