# Simulation Debugging Workflow

A structured approach to diagnosing failures in Gazebo SITL scenario tests. This guide walks through systematic log analysis to identify functional and integration issues in the companion stack.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Root Cause Categories](#root-cause-categories)
3. [Step-by-Step Debugging Methodology](#step-by-step-debugging-methodology)
4. [Example: Functional Error — Path Planner Timeout](#example-functional-error--path-planner-timeout)
5. [Common Error Patterns](#common-error-patterns)
6. [Log File Reference](#log-file-reference)

---

## Quick Start

When a scenario test fails:

```bash
# 1. Check the error summary
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json
# → Look for: "15 passed, 1 failed, 16 total"

# 2. Note the log directory
# → Usually: drone_logs/scenarios_gazebo/<scenario_name>/

# 3. Follow the step-by-step debugging process below
```

---

## Root Cause Categories

Before diving into logs, understand what could fail:

| Category | Indicator | Example |
|----------|-----------|---------|
| **Timeout** | Process killed after N seconds | Mission planner hangs, scenario timeout exceeded |
| **Functional** | Log pattern expected but missing | Expected "EXECUTING" state never reached |
| **Crash** | Segfault, assertion, abort | Memory corruption, null pointer dereference |
| **Deadlock** | Process alive but stuck (no new logs) | IPC subscriber blocked, thread stuck |
| **Resource** | Out of memory, file descriptor exhausted | Perception logs grow unbounded |
| **Integration** | Expected IPC message never published | Pose not received, commands not executed |

---

## Step-by-Step Debugging Methodology

### **Step 1: Verify the Failure Mode**

```bash
cd /home/nmohanan31/NM/Projects/companion_software_stack

# Check which tests passed/failed
tail -20 drone_logs/scenarios_gazebo/<scenario>/combined.log | grep -E "passed|failed"
```

**What to look for:**
- Total passed/failed count
- Which specific assertion failed (if shown)

### **Step 2: Identify Timestamp of Failure**

```bash
# Find the exact moment things went wrong
tail -100 drone_logs/scenarios_gazebo/<scenario>/combined.log | grep -iE "error|fatal|failed"
```

**Example output:**
```
[2026-03-14 16:40:29.821] [system_monitor] [error] [t:205384] [SysMon] Process DIED: mission_planner
```

**Action:** Note the timestamp (16:40:29 in this case). This is your failure point.

### **Step 3: Look at Launcher/Scenario Runner Output**

```bash
tail -50 drone_logs/scenarios_gazebo/<scenario>/launcher.log
```

**What to look for:**
- Process launch success/failure
- Timeout messages
- "Killed" signal (indicates forceful termination)
- MAVLink/PX4 connection status

**Example suspicious pattern:**
```
[SITL] Waiting for PX4 MAVLink heartbeat on udp://:14540...
# … nothing happens, timeout, killed
```

### **Step 4: Check the Primary Process Logs**

Based on the scenario, check the most relevant process:

```bash
# For mission/planning failures:
strings drone_logs/scenarios_gazebo/<scenario>/mission_planner.log | \
  grep -iE "waypoint|path|obstacle|execute" | tail -20

# For perception failures:
strings drone_logs/scenarios_gazebo/<scenario>/perception.log | \
  grep -iE "detector|track|fuse" | tail -20

# For comms failures:
strings drone_logs/scenarios_gazebo/<scenario>/comms.log | \
  grep -iE "mavlink|heartbeat|fc_state" | tail -20
```

**Why `strings`?** Binary log files contain formatting codes. `strings` extracts readable text.

### **Step 5: Trace IPC Message Flow**

Check if critical IPC messages were published:

```bash
# Search entire combined log for key patterns
grep -iE "pose|detected_objects|fc_state|mission_status" \
  drone_logs/scenarios_gazebo/<scenario>/combined.log | tail -20
```

**What to look for:**
- Is `/slam_pose` being published? (P3 → P4)
- Are `/detected_objects` messages flowing? (P2 → P4)
- Is `/fc_state` being received? (P5 → P4)

**Example healthy pattern:**
```
[2026-03-14 16:39:15.234] [slam_vio_nav] [info] [t:205463] Published pose #1500
[2026-03-14 16:39:15.235] [mission_planner] [info] [t:205488] Received pose: (1.23, 4.56, 2.10)
```

### **Step 6: Check Process Liveness at Failure Time**

Look for the last log entry of each process around the failure timestamp:

```bash
# Extract last entry from each process log at the failure time
for log in drone_logs/scenarios_gazebo/<scenario>/*.log; do
  echo "=== $(basename $log) ===" 
  tail -1 "$log"
done
```

**What to look for:**
- Processes still logging after failure timestamp? → They stayed alive
- Processes silent before failure? → They crashed
- Timestamp gap? → Indicates freeze

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
# View expected log patterns
grep -A 20 '"pass_criteria"' config/scenarios/<scenario_number>.json
```

**Then verify each criterion:**

```bash
# Check for required log patterns
grep "Path planner: dstar" drone_logs/scenarios_gazebo/<scenario>/combined.log
grep "EXECUTING" drone_logs/scenarios_gazebo/<scenario>/combined.log
grep "Mission complete" drone_logs/scenarios_gazebo/<scenario>/combined.log

# Check for forbidden patterns
grep -i "EMERGENCY_LAND" drone_logs/scenarios_gazebo/<scenario>/combined.log
grep -i "COLLISION" drone_logs/scenarios_gazebo/<scenario>/combined.log
```

**Example failure diagnosis:**
```
$ grep "Mission complete" combined.log
[2026-03-14 16:39:52.005] Mission complete ✓

$ grep "EXECUTING" combined.log
(no results)
[FAIL] Expected "EXECUTING" state never logged
```

### **Step 8: Investigate Root Cause**

Based on findings above, run targeted searches:

**If timeout suspected:**
```bash
# Check scenario duration config
grep "timeout_s" config/scenarios/<number>.json

# Calculate actual duration from logs
echo "First log:" $(head -1 drone_logs/scenarios_gazebo/<scenario>/combined.log | grep -oP '\d{2}:\d{2}:\d{2}')
echo "Last log:" $(tail -1 drone_logs/scenarios_gazebo/<scenario>/combined.log | grep -oP '\d{2}:\d{2}:\d{2}')
```

**If functional issue suspected:**
```bash
# Follow mission planner state transitions
strings drone_logs/scenarios_gazebo/<scenario>/mission_planner.log | \
  grep -iE "FSM|state transition|waypoint" | head -30
```

**If deadlock suspected:**
```bash
# Look for repeated log patterns (indicates stuck loop)
strings drone_logs/scenarios_gazebo/<scenario>/perception.log | \
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
$ tail -5 drone_logs/scenarios_gazebo/obstacle_avoidance/combined.log | grep -E "passed|failed"
Results: 2 passed, 14 failed, 16 total
```

**Step 2: Identify failure point**
```bash
$ grep -i "error\|fatal" drone_logs/scenarios_gazebo/obstacle_avoidance/combined.log | head -5
[2026-03-14 16:41:55.334] [mission_planner] [error] [t:205488] [Planner] No path found 
  after 30 tries — timeout exceeded
```

**Step 3: Root cause confirmed**

Path planner timed out trying to find a path around obstacles within 30 iterations.

**Step 4: Examine planner loop**
```bash
$ strings drone_logs/scenarios_gazebo/obstacle_avoidance/mission_planner.log | \
  grep -iE "astar|path|iteration" | tail -30

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
$ grep -A 5 '"static_obstacles"' config/scenarios/02_obstacle_avoidance.json | head -10
...
{"x": 1, "y": 7, "radius_m": 0.75, "height_m": 5.0},
{"x": 2, "y": 13, "radius_m": 0.75, "height_m": 4.0},
...
```

**Step 7: Check if waypoints are reachable**
```bash
$ grep '"waypoints"' config/scenarios/02_obstacle_avoidance.json -A 7 | head -15
...
{"x": 1, "y": 7, "z": 3, ... "Straight at RED box — must avoid"},
{"x": 2, "y": 13, "z": 3, ... "Straight at BLUE cylinder — must avoid"},
...
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
# Verify P3 is healthy
strings drone_logs/scenarios_gazebo/<scenario>/slam_vio_nav.log | \
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
- Check for circular IPC dependency (P A → P B → P C → P A)

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
  │
  ├─→ Processes killed after N seconds?
  │    └─→ Check launcher.log for "Killed"
  │         └─→ YES: Increase timeout_s in scenario config
  │         └─→ NO: Go to "Process Liveness"
  │
  ├─→ Expected log pattern not found?
  │    └─→ grep "expected_pattern" combined.log
  │         └─→ Found late in logs: Process slow to reach state
  │         └─→ Not found: Functional logic failure
  │              └─→ Check mission_planner.log for path planning / FSM issues
  │
  ├─→ Forbidden pattern found (EMERGENCY_LAND, COLLISION)?
  │    └─→ Check waypoint/obstacle placement
  │         └─→ Adjust config_overrides in scenario
  │
  └─→ Process liveness check failed?
       └─→ Check if all 7 processes logged after test completion
            └─→ Process silent: Check its .log for crashes
            └─→ All logged: Check scenariorunner validation timing
```

---

## Next Steps

- Read the full [Gazebo integration guide](gazebo_setup.md)
- For custom scenarios, see [scenario config format](../config/scenarios/)
- To add a new scenario, copy an existing one and adjust `_comment`, `timeout_s`, waypoints, and pass_criteria
