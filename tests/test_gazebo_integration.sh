#!/usr/bin/env bash
# tests/test_gazebo_integration.sh
# ════════════════════════════════════════════════════════════
# Closed-loop integration test: PX4 SITL + Gazebo + companion stack.
#
# Launches everything, runs for a verification window, checks that
# each process is alive and producing expected output, then shuts down.
#
# Usage:
#   ./tests/test_gazebo_integration.sh           # full test
#   TIMEOUT=60 ./tests/test_gazebo_integration.sh  # custom timeout
#
# Exit codes:
#   0  All checks passed
#   1  One or more checks failed
#   2  Setup / launch failure
#
# Prerequisites:
#   - PX4-Autopilot built (make px4_sitl_default)
#   - Gazebo Harmonic installed
#   - Companion stack built with Gazebo + MAVSDK support
# ════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"
DEPLOY_DIR="${PROJECT_DIR}/deploy"
CONFIG_FILE="${PROJECT_DIR}/config/gazebo.json"
LOG_DIR="/tmp/drone_integration_test_logs"
TIMEOUT="${TIMEOUT:-120}"

# ── Colours ──────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No colour

PASS=0
FAIL=0
TOTAL=0

check() {
    local desc="$1"
    local result="$2"
    TOTAL=$((TOTAL + 1))
    if [[ "$result" -eq 0 ]]; then
        echo -e "  ${GREEN}✓${NC} ${desc}"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}✗${NC} ${desc}"
        FAIL=$((FAIL + 1))
    fi
}

# ── Pre-flight checks ───────────────────────────────────────
echo "════════════════════════════════════════════════════════"
echo "  Gazebo Integration Test"
echo "  Timeout  : ${TIMEOUT}s"
echo "  Config   : ${CONFIG_FILE}"
echo "  Log dir  : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

if [[ ! -d "$BIN_DIR" ]]; then
    echo -e "${RED}ERROR: Build directory not found. Build the project first.${NC}"
    exit 2
fi
if [[ ! -f "${DEPLOY_DIR}/launch_gazebo.sh" ]]; then
    echo -e "${RED}ERROR: launch_gazebo.sh not found.${NC}"
    exit 2
fi

# Clean previous test logs
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

# ── Step 1: Run unit tests first ─────────────────────────────
echo ""
echo "Phase 1: Running unit tests..."
pushd "${PROJECT_DIR}/build" > /dev/null
if ctest --output-on-failure --timeout 30 > "${LOG_DIR}/ctest.log" 2>&1; then
    UNIT_RESULT=0
else
    UNIT_RESULT=1
fi
popd > /dev/null

UNIT_COUNT=$(grep -c "Test #" "${LOG_DIR}/ctest.log" 2>/dev/null || echo "0")
UNIT_PASS=$(grep "tests passed" "${LOG_DIR}/ctest.log" 2>/dev/null | grep -oP '\d+(?= tests passed)' || echo "0")
check "Unit tests pass (${UNIT_PASS}/${UNIT_COUNT})" $UNIT_RESULT

# ── Step 2: Launch PX4 + Gazebo + stack ──────────────────────
echo ""
echo "Phase 2: Launching PX4 SITL + Gazebo + companion stack..."

export PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"

# Check PX4 is available
if [[ ! -d "$PX4_DIR" ]]; then
    echo -e "${YELLOW}SKIP: PX4-Autopilot not found at ${PX4_DIR}${NC}"
    echo -e "${YELLOW}      Set PX4_DIR or install PX4. Skipping SITL tests.${NC}"
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo -e "  Results: ${GREEN}${PASS} passed${NC}, ${FAIL} failed, ${TOTAL} total"
    echo "  (SITL tests skipped — PX4 not available)"
    echo "════════════════════════════════════════════════════════"
    [[ $FAIL -eq 0 ]] && exit 0 || exit 1
fi

if [[ ! -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
    echo -e "${YELLOW}SKIP: PX4 SITL not built. Run 'make px4_sitl_default' first.${NC}"
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo -e "  Results: ${GREEN}${PASS} passed${NC}, ${FAIL} failed, ${TOTAL} total"
    echo "  (SITL tests skipped — PX4 not built)"
    echo "════════════════════════════════════════════════════════"
    [[ $FAIL -eq 0 ]] && exit 0 || exit 1
fi

# Ensure system libstdc++ is used
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

STACK_PIDS=()

cleanup_test() {
    echo ""
    echo "Cleaning up test processes..."
    for pid in "${STACK_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${STACK_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    # Clean SHM
    rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
          /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
          /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
          /dev/shm/system_health 2>/dev/null || true
}
trap cleanup_test EXIT INT TERM

# Clean stale SHM
rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
      /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health 2>/dev/null || true

# ── Launch PX4 SITL ──────────────────────────────────────────
echo "  Starting PX4 SITL..."
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL="x500"
export PX4_GZ_WORLD_FILE="${PROJECT_DIR}/sim/worlds/test_world.sdf"
export HEADLESS=1

pushd "$PX4_DIR" > /dev/null
./build/px4_sitl_default/bin/px4 \
    -d ./build/px4_sitl_default/etc \
    -s ./build/px4_sitl_default/etc/init.d-posix/rcS \
    > "${LOG_DIR}/px4_sitl.log" 2>&1 &
PX4_PID=$!
STACK_PIDS+=($PX4_PID)
popd > /dev/null

# Wait for PX4 to start
echo "  Waiting for PX4 startup..."
sleep 10

check "PX4 SITL process alive" $(kill -0 $PX4_PID 2>/dev/null; echo $?)

# ── Launch companion processes ───────────────────────────────
CONFIG_ARG="--config ${CONFIG_FILE}"
PROCESSES=("system_monitor" "video_capture" "comms" "perception"
           "slam_vio_nav" "mission_planner" "payload_manager")

for proc in "${PROCESSES[@]}"; do
    echo "  Starting ${proc}..."
    "${BIN_DIR}/${proc}" ${CONFIG_ARG} \
        > "${LOG_DIR}/${proc}.log" 2>&1 &
    STACK_PIDS+=($!)
    sleep 0.5
done

echo "  All processes launched. Letting stack run for verification..."

# ── Step 3: Verification window ──────────────────────────────
# Let the stack run for a reasonable time
VERIFY_TIME=20
echo ""
echo "Phase 3: Running for ${VERIFY_TIME}s verification window..."
sleep "$VERIFY_TIME"

echo ""
echo "Phase 4: Checking results..."
echo ""

# ── Check all 7 companion processes are alive ────────────────
echo "Process liveness:"
for i in "${!PROCESSES[@]}"; do
    pid_idx=$((i + 1))  # offset by 1 because PX4 is at index 0
    pid=${STACK_PIDS[$pid_idx]}
    proc=${PROCESSES[$i]}
    if kill -0 "$pid" 2>/dev/null; then
        check "${proc} (PID ${pid}) alive" 0
    else
        check "${proc} (PID ${pid}) alive" 1
    fi
done

# ── Check log files exist and have content ───────────────────
echo ""
echo "Log file verification:"
for proc in "${PROCESSES[@]}"; do
    logfile="${LOG_DIR}/${proc}.log"
    if [[ -f "$logfile" ]] && [[ -s "$logfile" ]]; then
        check "${proc}.log has content" 0
    else
        check "${proc}.log has content" 1
    fi
done

# ── Check SHM segments exist ────────────────────────────────
echo ""
echo "SHM segment verification:"
SHM_SEGMENTS=("drone_mission_camera" "drone_stereo_camera"
              "drone_detected_objects" "drone_slam_pose"
              "drone_mission_status" "drone_fc_state"
              "drone_system_health")
for seg in "${SHM_SEGMENTS[@]}"; do
    if [[ -f "/dev/shm/${seg}" ]]; then
        check "SHM: ${seg}" 0
    else
        # Try without drone_ prefix (legacy naming)
        base="${seg#drone_}"
        if [[ -f "/dev/shm/${base}" ]]; then
            check "SHM: ${base}" 0
        else
            check "SHM: ${seg}" 1
        fi
    fi
done

# ── Check specific log markers ──────────────────────────────
echo ""
echo "Functional verification:"

# P1: Video capture opened cameras
if grep -qi "opened\|subscribed\|capturing\|GazeboCamera" "${LOG_DIR}/video_capture.log" 2>/dev/null; then
    check "P1: video_capture initialised cameras" 0
else
    check "P1: video_capture initialised cameras" 1
fi

# P3: SLAM/VIO received IMU data
if grep -qi "IMU\|imu.*init\|GazeboIMU\|IMUReader" "${LOG_DIR}/slam_vio_nav.log" 2>/dev/null; then
    check "P3: slam_vio_nav initialised IMU" 0
else
    check "P3: slam_vio_nav initialised IMU" 1
fi

# P5: Comms connected to PX4
if grep -qi "connect\|mavlink\|heartbeat\|MavlinkFCLink" "${LOG_DIR}/comms.log" 2>/dev/null; then
    check "P5: comms connected to MAVLink" 0
else
    check "P5: comms connected to MAVLink" 1
fi

# P7: System monitor collecting data
if grep -qi "health\|monitor\|cpu\|temperature" "${LOG_DIR}/system_monitor.log" 2>/dev/null; then
    check "P7: system_monitor reporting health" 0
else
    check "P7: system_monitor reporting health" 1
fi

# PX4 SITL output
if grep -qi "ready\|armed\|commander\|ekf" "${LOG_DIR}/px4_sitl.log" 2>/dev/null; then
    check "PX4: SITL initialised" 0
else
    check "PX4: SITL initialised" 1
fi

# ── Results ──────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
if [[ $FAIL -eq 0 ]]; then
    echo -e "  ${GREEN}ALL CHECKS PASSED${NC}: ${PASS}/${TOTAL}"
else
    echo -e "  ${RED}FAILURES${NC}: ${FAIL}/${TOTAL} checks failed"
    echo "  Logs available at: ${LOG_DIR}/"
fi
echo "════════════════════════════════════════════════════════"

[[ $FAIL -eq 0 ]] && exit 0 || exit 1
