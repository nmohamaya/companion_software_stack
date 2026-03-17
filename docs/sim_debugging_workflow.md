# Simulation Debugging Workflow

A structured approach to diagnosing failures in Gazebo SITL scenario tests. This guide walks through systematic log analysis to identify functional and integration issues in the companion stack.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Root Cause Categories](#root-cause-categories)
3. [Step-by-Step Debugging Methodology](#step-by-step-debugging-methodology)
4. [Example: Functional Error — Path Planner Timeout](#example-functional-error--path-planner-timeout)
5. [Example: Config Mismatch — Wrong VIO Backend in Gazebo](#example-config-mismatch--wrong-vio-backend-in-gazebo)
6. [Common Error Patterns](#common-error-patterns)
7. [Log File Reference](#log-file-reference)

---

## Quick Start

**Before running any commands in this guide, run this setup block first:**

```bash
# REQUIRED: cd to project root — all paths in this guide are relative to it
cd "$(git rev-parse --show-toplevel)"

# Set LOG_DIR based on how you launched the stack:
#
#   Manual run (launch_gazebo.sh):
LOG_DIR=drone_logs
#
#   Automated Gazebo scenario (run_scenario_gazebo.sh):
#LOG_DIR=drone_logs/scenarios_gazebo/obstacle_avoidance
#
#   Automated Tier 1 scenario (run_scenario.sh):
#LOG_DIR=drone_logs/scenarios/obstacle_avoidance
#
# Uncomment the one that matches your run. To see what's available:
ls drone_logs/
ls drone_logs/scenarios_gazebo/ 2>/dev/null
ls drone_logs/scenarios/ 2>/dev/null
```

When a scenario test fails:

```bash
# 1. Run the failing scenario
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json
# -> Look for: "15 passed, 1 failed, 16 total"

# 2. Set LOG_DIR to point at the logs from that run
LOG_DIR=drone_logs/scenarios_gazebo/obstacle_avoidance

# 3. Follow the step-by-step debugging process below
```

---

## Root Cause Categories

Before diving into logs, understand what could fail:

| Category | Indicator | Example |
|----------|-----------|---------|
| **Config** | Wrong backend loaded, stale config key | VIO falls back to "simulated" instead of "gazebo" |
| **Timeout** | Process killed after N seconds | Mission planner hangs, scenario timeout exceeded |
| **Functional** | Log pattern expected but missing | Expected "EXECUTING" state never reached |
| **Crash** | Segfault, assertion, abort | Memory corruption, null pointer dereference |
| **Deadlock** | Process alive but stuck (no new logs) | IPC subscriber blocked, thread stuck |
| **Resource** | Out of memory, file descriptor exhausted | Perception logs grow unbounded |
| **Integration** | Expected IPC message never published | Pose not received, commands not executed |

---

## Step-by-Step Debugging Methodology

> All commands below assume you have completed the [Quick Start](#quick-start) setup:
> `cd` to the project root and `LOG_DIR` set to the correct log directory.

### **Step 1: Verify the Failure Mode**

```bash
tail -20 "${LOG_DIR}/combined.log" | grep -E "passed|failed"
```

**What to look for:**
- Total passed/failed count
- Which specific assertion failed (if shown)

### **Step 2: Identify Timestamp of Failure**

```bash
tail -100 "${LOG_DIR}/combined.log" | grep -iE "error|fatal"
```

**Example output:**
```
[2026-03-14 16:40:29.821] [system_monitor] [error] [t:205384] [SysMon] Process DIED: mission_planner
```

**Action:** Note the timestamp (16:40:29 in this case). This is your failure point.

### **Step 3: Look at Launcher/Scenario Runner Output**

```bash
tail -50 "${LOG_DIR}/launcher.log"
```

**What to look for:**
- Process launch success/failure
- Timeout messages
- "Killed" signal (indicates forceful termination)
- MAVLink/PX4 connection status

**Example suspicious pattern:**
```
[SITL] Waiting for PX4 MAVLink heartbeat on udp://:14540...
# ... nothing happens, timeout, killed
```

### **Step 4: Check the Primary Process Logs**

Based on the scenario, check the most relevant process:

```bash
# For mission/planning failures:
strings "${LOG_DIR}/mission_planner.log" | \
  grep -iE "waypoint|path|obstacle|execute" | tail -20

# For perception failures:
strings "${LOG_DIR}/perception.log" | \
  grep -iE "detector|track|fuse" | tail -20

# For comms failures:
strings "${LOG_DIR}/comms.log" | \
  grep -iE "mavlink|heartbeat|fc_state" | tail -20
```

**Why `strings`?** Binary log files contain formatting codes. `strings` extracts readable text.

### **Step 5: Trace IPC Message Flow**

Check if critical IPC messages were published:

```bash
grep -iE "pose|detected_objects|fc_state|mission_status" \
  "${LOG_DIR}/combined.log" | tail -20
```

**What to look for:**
- Is `/slam_pose` being published? (P3 -> P4)
- Are `/detected_objects` messages flowing? (P2 -> P4)
- Is `/fc_state` being received? (P5 -> P4)

**Example healthy pattern:**
```
[2026-03-14 16:39:15.234] [slam_vio_nav] [info] [t:205463] Published pose #1500
[2026-03-14 16:39:15.235] [mission_planner] [info] [t:205488] Received pose: (1.23, 4.56, 2.10)
```

### **Step 6: Check Process Liveness at Failure Time**

Look for the last log entry of each process around the failure timestamp:

```bash
for log in "${LOG_DIR}"/*.log; do
  echo "=== $(basename "$log") ==="
  tail -1 "$log"
done
```

**What to look for:**
- Processes still logging after failure timestamp? -> They stayed alive
- Processes silent before failure? -> They crashed
- Timestamp gap? -> Indicates freeze

**Example output:**
```
=== mission_planner.log ===
[2026-03-14 16:40:25.122] [mission_planner] [info] [t:205488] Loop tick 400

=== perception.log ===
[2026-03-14 16:40:28.891] [perception] [info] [t:205463] Fusion complete
```

If mission_planner stopped at 16:40:25 and failure was at 16:40:29, it hung for 4 seconds.

### **Step 7: Check Scenario Pass Criteria**

Look at what the scenario required:

```bash
# View expected log patterns (set SCENARIO_NUM to the scenario number, e.g. 02)
SCENARIO_NUM=02
grep -A 20 '"pass_criteria"' config/scenarios/${SCENARIO_NUM}_*.json
```

**Then verify each criterion:**

```bash
grep "Path planner: DStarLitePlanner" "${LOG_DIR}/combined.log"
grep "EXECUTING" "${LOG_DIR}/combined.log"
grep "Mission complete" "${LOG_DIR}/combined.log"

# Check for forbidden patterns
grep -i "EMERGENCY_LAND" "${LOG_DIR}/combined.log"
grep -i "COLLISION" "${LOG_DIR}/combined.log"
```

**Example failure diagnosis:**
```
$ grep "Mission complete" combined.log
[2026-03-14 16:39:52.005] Mission complete

$ grep "EXECUTING" combined.log
(no results)
[FAIL] Expected "EXECUTING" state never logged
```

### **Step 8: Verify Config Was Applied Correctly**

Many SITL failures trace back to config mismatches — stale keys, wrong base config, or overrides that don't reach the intended process.

```bash
# 1. Check which config the scenario runner actually used
python3 -m json.tool "${LOG_DIR}/merged_config.json" | head -80

# 2. Verify critical backend selections in the merged config
python3 -c "
import json, sys
cfg = json.load(open(sys.argv[1]))
print('VIO backend:      ', cfg.get('slam',{}).get('vio',{}).get('backend','NOT SET'))
print('IMU backend:      ', cfg.get('slam',{}).get('imu',{}).get('backend','NOT SET'))
print('FC backend:       ', cfg.get('comms',{}).get('mavlink',{}).get('backend','NOT SET'))
print('Detector backend: ', cfg.get('perception',{}).get('detector',{}).get('backend','NOT SET'))
print('Planner backend:  ', cfg.get('mission_planner',{}).get('path_planner',{}).get('backend','NOT SET'))
print('Avoider backend:  ', cfg.get('mission_planner',{}).get('obstacle_avoider',{}).get('backend','NOT SET'))
print('IPC backend:      ', cfg.get('ipc_backend','NOT SET'))
" "${LOG_DIR}/merged_config.json"

# 3. Compare base configs for drift (gazebo.json vs gazebo_sitl.json)
diff <(python3 -c "import json; print(json.dumps(json.load(open('config/gazebo.json')), indent=2, sort_keys=True))") \
     <(python3 -c "import json; print(json.dumps(json.load(open('config/gazebo_sitl.json')), indent=2, sort_keys=True))")

# 4. Verify process logs confirm the expected backend was loaded
strings "${LOG_DIR}/slam_vio_nav.log" | grep -i "backend"
strings "${LOG_DIR}/mission_planner.log" | grep -iE "Path planner:|Obstacle avoider:"
strings "${LOG_DIR}/comms.log" | grep -i "FC link:"
```

**What to look for:**
- `VIO backend: NOT SET` -> P3 will fall back to `"simulated"`, which generates a fake pose unrelated to the real drone position in Gazebo. This is the most common cause of "drone doesn't navigate" in SITL. (See Fix #25, Fix #41 in `docs/BUG_FIXES.md`.)
- Backend mismatch between `gazebo.json` (used by `launch_gazebo.sh`) and `gazebo_sitl.json` (used by `run_scenario_gazebo.sh`) — these files must be kept in sync.
- Stale config keys (e.g. `slam.visual_frontend` instead of `slam.vio`) — the process code reads a specific key path; if the config uses a different name, the value is silently ignored and the default is used.

### **Step 9: Investigate Root Cause**

Based on findings above, run targeted searches:

**If timeout suspected:**
```bash
SCENARIO_NUM=02
grep "timeout_s" config/scenarios/${SCENARIO_NUM}_*.json

# Calculate actual duration from logs
echo "First log:" "$(head -1 "${LOG_DIR}/combined.log" | grep -oP '\d{2}:\d{2}:\d{2}')"
echo "Last log:" "$(tail -1 "${LOG_DIR}/combined.log" | grep -oP '\d{2}:\d{2}:\d{2}')"
```

**If functional issue suspected:**
```bash
strings "${LOG_DIR}/mission_planner.log" | \
  grep -iE "FSM|state transition|waypoint" | head -30
```

**If deadlock suspected:**
```bash
# Look for repeated log patterns (indicates stuck loop)
strings "${LOG_DIR}/perception.log" | \
  sort | uniq -c | sort -rn | head -10
# High counts + same timestamp = likely deadlock
```

---

## Example: Functional Error — Path Planner Timeout

### Scenario Description

Running `config/scenarios/02_obstacle_avoidance.json`:
- Expected: Drone navigates 7 waypoints using D* Lite planner, avoids obstacles
- Failure: "2 passed, 14 failed" — missing critical log patterns

### Debugging Session

**Step 1: Check failure**
```bash
LOG_DIR=drone_logs/scenarios_gazebo/obstacle_avoidance
tail -5 "${LOG_DIR}/combined.log" | grep -E "passed|failed"
# -> Results: 2 passed, 14 failed, 16 total
```

**Step 2: Identify failure point**
```bash
grep -iE "error|fatal" "${LOG_DIR}/combined.log" | head -5
# -> [2026-03-14 16:41:55.334] [mission_planner] [error] [t:205488] [Planner] No path found
#      after 30 tries - timeout exceeded
```

**Step 3: Root cause confirmed**

Path planner timed out trying to find a path around obstacles within 30 iterations.

**Step 4: Examine planner loop**
```bash
strings "${LOG_DIR}/mission_planner.log" | \
  grep -iE "astar|dstar|path|iteration" | tail -30
```

Example output:
```
[Planner] Planning path from (0.1, 0.2, 5.0) to (1.0, 7.0, 3.0)
[Planner] D* Lite iteration 1: open_set=45, closed=12
[Planner] D* Lite iteration 2: open_set=42, closed=18
[Planner] D* Lite iteration 3: open_set=38, closed=25
...
[Planner] D* Lite iteration 29: open_set=2, closed=890
[Planner] D* Lite iteration 30: TIMEOUT
[Planner] No path found after 30 tries — timeout exceeded
```

**Step 5: Diagnosis**

The planner was expanding too many nodes (closed set growing to 890) without finding the goal. Likely causes:
1. Heuristic too weak (overestimate issue)
2. Obstacle placement blocks all paths
3. Goal unreachable

**Step 6: Verify via config**
```bash
grep -A 5 '"static_obstacles"' config/scenarios/02_obstacle_avoidance.json | head -10
```

```
{"x": 1, "y": 7, "radius_m": 0.75, "height_m": 5.0},
{"x": 2, "y": 13, "radius_m": 0.75, "height_m": 4.0},
```

**Step 7: Check if waypoints are reachable**
```bash
grep '"waypoints"' config/scenarios/02_obstacle_avoidance.json -A 7 | head -15
```

```
{"x": 1, "y": 7, "z": 3, ... "Straight at RED box — must avoid"},
{"x": 2, "y": 13, "z": 3, ... "Straight at BLUE cylinder — must avoid"},
```

**Aha!** First waypoint is at (1, 7) — **the same location as a RED obstacle** with radius 0.75 m. The planner cannot reach a goal inside an obstacle!

### Fix

Edit `config/scenarios/02_obstacle_avoidance.json`:

```json
"waypoints": [
    {"x": 1.5, "y": 7.5, "z": 3, ...},  // Offset from obstacle center
    {"x": 2.5, "y": 13.5, "z": 3, ...},
    ...
]
```

### Validation

Re-run scenario:
```bash
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json
# Expected: 16 passed, 0 failed
```

---

## Example: Config Mismatch — Wrong VIO Backend in Gazebo

### Scenario Description

Running scenario 02 (obstacle avoidance) via `launch_gazebo.sh`:
- Expected: Drone flies toward obstacles and avoids them using D* Lite planner
- Actual: Drone takes off but flies erratically, never reaches waypoints

### Debugging Session

**Step 1: Check which VIO backend was loaded**
```bash
# For manual launch_gazebo.sh runs, logs are in drone_logs/ directly:
LOG_DIR=drone_logs
strings "${LOG_DIR}/slam_vio_nav.log" | grep -i "backend"
# -> VIO backend: SimulatedVIOBackend (sim_speed=2.0 m/s)
# WRONG! Should be GazeboVIOBackend for Gazebo SITL
```

**Step 2: Check the config that was used**

`launch_gazebo.sh` defaults to `config/gazebo.json`. Inspect it:
```bash
python3 -c "
import json
cfg = json.load(open('config/gazebo.json'))
print('slam.vio.backend:', cfg.get('slam',{}).get('vio',{}).get('backend','NOT SET'))
print('slam keys:', list(cfg.get('slam',{}).keys()))
"
```

If this prints `slam.vio.backend: NOT SET`, the config is missing the `slam.vio` section. P3 reads `slam.vio.backend` and defaults to `"simulated"` when not found.

**Step 3: Compare with the working config**
```bash
diff <(python3 -c "import json; print(json.dumps(json.load(open('config/gazebo.json')).get('slam',{}), indent=2, sort_keys=True))") \
     <(python3 -c "import json; print(json.dumps(json.load(open('config/gazebo_sitl.json')).get('slam',{}), indent=2, sort_keys=True))")
```

This reveals mismatched keys — e.g. `visual_frontend` (stale) vs `vio` (correct).

**Step 4: Fix the config**

Rename the stale key to match what the code reads:
```json
"slam": {
    "vio": {
        "backend": "gazebo",
        "gz_topic": "/model/x500_companion_0/odometry"
    }
}
```

**Step 5: Verify the fix**
```bash
./deploy/launch_gazebo.sh --gui
# Drone should now navigate toward obstacles and avoid them
```

### Key Lesson

There are two Gazebo config files that must stay in sync:
- `config/gazebo.json` — used by `deploy/launch_gazebo.sh` (manual runs)
- `config/gazebo_sitl.json` — used by `tests/run_scenario_gazebo.sh` (automated)

When a config key is renamed, both files must be updated. See Fix #25 and Fix #41 in `docs/BUG_FIXES.md`.

---

## Common Error Patterns

### Pattern 1: Missing IPC Message

**Log appearance:**
```
[mission_planner] Loop tick 100 — no pose available (0 drops)
[mission_planner] Loop tick 101 — no pose available (0 drops)
[mission_planner] Loop tick 102 — timeout waiting for pose
```

**Diagnosis:**
- P3 (SLAM/VIO) not publishing pose
- Check `slam_vio_nav.log` for crashes or stuck threads

**Fix:**
```bash
strings "${LOG_DIR}/slam_vio_nav.log" | \
  grep -iE "visual frontend|imu|pose" | tail -5
```

### Pattern 2: Process Killed After Exceeded Timeout

**Log appearance:**
```
launcher.log:
[Stack] All processes launched. PIDs: ... (150s elapsed, timeout=100s)
launcher.log: Killed (signal 9)
```

**Diagnosis:**
- Scenario completed but validation took too long
- Timeout setting too tight

**Fix:**
```json
// In scenario config:
{"timeout_s": 150}  // Increase from 100
```

### Pattern 3: Deadlock (Process Alive but Not Logging)

**Log appearance:**
```
perception.log: [2026-03-14 16:40:15.234] [perception] [info] Processing frame #500
perception.log: [2026-03-14 16:40:15.235] [perception] [info] Publishing detections
(... 30 seconds of silence ...)
```

**Diagnosis:**
- Thread stuck waiting for lock or IPC message
- Check for circular IPC dependency (P A -> P B -> P C -> P A)

**Fix:**
```bash
# Run with ThreadSanitizer to detect deadlocks
bash deploy/build.sh --tsan
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json
```

### Pattern 4: High CPU / Memory Explosion

**Log appearance:**
```
perception.log: [2026...] [Frame] Published 1000 frames (ring buffer 99% full)
perception.2.log: Growing from 1MB to 500MB in 20s
```

**Diagnosis:**
- Downstream (mission planner) not consuming poses fast enough
- Perception thread looping faster than publishing

**Fix:**
```json
{
  "perception": {
    "detector": {"max_detections": 32}  // Reduce work per frame
  }
}
```

### Pattern 5: Config Key Mismatch (Silent Backend Fallback)

**Log appearance:**
```
slam_vio_nav.log: VIO backend: SimulatedVIOBackend (sim_speed=2.0 m/s)
# Expected: VIO backend: GazeboVIOBackend (for Gazebo SITL runs)
```

**Diagnosis:**
- Config file uses a stale or misnamed key that P3 doesn't read
- P3 falls back to `"simulated"` VIO, generating a fake pose
- The drone appears to take off and arm, but never navigates correctly
- The mission planner's velocity commands are based on the wrong position

**Identify:**
```bash
# Check if slam.vio.backend is set in the config being used
python3 -c "
import json
cfg = json.load(open('config/gazebo.json'))
vio = cfg.get('slam',{}).get('vio',{}).get('backend','NOT SET')
print('slam.vio.backend:', vio)
if vio == 'NOT SET':
    print('WARNING: P3 will fall back to simulated VIO!')
    print('Check for stale keys:')
    for k in cfg.get('slam',{}).keys():
        print(f'  slam.{k}')
"
```

**Fix:** Ensure the config has `slam.vio.backend: "gazebo"` (not under a different key name). Compare with the known-good `config/gazebo_sitl.json`.

---

## Log File Reference

| File | Process | Contains | Use for |
|------|---------|----------|---------|
| `combined.log` | All | Merged stdout from all 7 processes | Top-level flow, IPC patterns |
| `mission_planner.log` | P4 | Path planning, FSM, waypoint execution | Mission logic failures |
| `perception.log` | P2 | Detection, tracking, fusion | Vision failures |
| `slam_vio_nav.log` | P3 | VIO, odometry, pose estimation | Localization issues |
| `comms.log` | P5 | FC heartbeat, GCS commands, telemetry | Flight controller comms |
| `system_monitor.log` | P7 | Process health, CPU/memory, restarts | Process crashes, resource limits |
| `video_capture.log` | P1 | Camera frames, frame drops | Video source issues |
| `payload_manager.log` | P6 | Gimbal, payload commands | Gimbal/camera failures |
| `launcher.log` | External | PX4, Gazebo startup, process PIDs | Launch-time failures |
| `merged_config.json` | External | Effective config (after merges) | Verify actual test parameters |

---

## Quick Troubleshooting Flowchart

```
Scenario Failed
  |
  +-> Drone takes off but doesn't navigate / flies randomly?
  |    +-> Check VIO backend: strings ${LOG_DIR}/slam_vio_nav.log | grep backend
  |         +-> "SimulatedVIOBackend": CONFIG BUG — slam.vio.backend missing
  |         +-> "GazeboVIOBackend": VIO is correct, check planner/avoider
  |
  +-> Processes killed after N seconds?
  |    +-> Check launcher.log for "Killed"
  |         +-> YES: Increase timeout_s in scenario config
  |         +-> NO: Go to "Process Liveness"
  |
  +-> Expected log pattern not found?
  |    +-> grep "expected_pattern" combined.log
  |         +-> Found late in logs: Process slow to reach state
  |         +-> Not found: Functional logic failure
  |              +-> Check mission_planner.log for path planning / FSM issues
  |
  +-> Forbidden pattern found (EMERGENCY_LAND, COLLISION)?
  |    +-> Check waypoint/obstacle placement
  |         +-> Adjust config_overrides in scenario
  |
  +-> Process liveness check failed?
       +-> Check if all 7 processes logged after test completion
            +-> Process silent: Check its .log for crashes
            +-> All logged: Check scenario runner validation timing
```

---

## Next Steps

- Read the full [Gazebo integration guide](gazebo_setup.md)
- For custom scenarios, see [scenario config format](../config/scenarios/)
- To add a new scenario, copy an existing one and adjust `_comment`, `timeout_s`, waypoints, and pass_criteria
