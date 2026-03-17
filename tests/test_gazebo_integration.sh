#!/usr/bin/env bash
# tests/test_gazebo_integration.sh
# ════════════════════════════════════════════════════════════
# Smoke / liveness integration test for PX4 SITL + Gazebo +
# companion stack.
#
# Launches the full stack via deploy/launch_gazebo.sh, then
# verifies that all 7 processes are alive and producing
# expected IPC / log output.
#
# This is NOT a closed-loop waypoint mission test — it verifies
# that all processes start correctly and communicate via SHM,
# gz-transport, and MAVLink.
#
# Usage:
#   ./tests/test_gazebo_integration.sh           # full test
#   VERIFY_TIME=30 ./tests/test_gazebo_integration.sh
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
set -uo pipefail
# Note: -e is intentionally omitted so we can handle errors
# ourselves and report pass/fail rather than aborting.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"
DEPLOY_DIR="${PROJECT_DIR}/deploy"
CONFIG_FILE="${PROJECT_DIR}/config/gazebo_sitl.json"
LOG_DIR="/tmp/drone_integration_test_logs"
VERIFY_TIME="${VERIFY_TIME:-20}"

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
echo "  Gazebo Smoke / Liveness Integration Test"
echo "  Verify   : ${VERIFY_TIME}s"
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

# ── Phase 1: Run unit tests ─────────────────────────────────
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
UNIT_PASS=$(awk '/tests passed/ { for (i = 1; i <= NF; i++) if ($i ~ /^[0-9]+$/) { print $i; exit } }' "${LOG_DIR}/ctest.log" 2>/dev/null)
UNIT_PASS=${UNIT_PASS:-0}
check "Unit tests pass (${UNIT_PASS}/${UNIT_COUNT})" $UNIT_RESULT

# ── Phase 2: Launch PX4 + Gazebo + stack via launch script ───
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

LAUNCHER_PID=""

cleanup_test() {
    echo ""
    echo "Cleaning up test..."
    # Send SIGINT to launcher, which triggers its own cleanup trap
    if [[ -n "$LAUNCHER_PID" ]] && kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        kill -SIGINT "$LAUNCHER_PID" 2>/dev/null || true
        # Give the launcher time to run its cleanup
        sleep 3
        # Force-kill if still alive
        if kill -0 "$LAUNCHER_PID" 2>/dev/null; then
            kill -SIGKILL "$LAUNCHER_PID" 2>/dev/null || true
        fi
    fi
    wait "$LAUNCHER_PID" 2>/dev/null || true
}
trap cleanup_test EXIT INT TERM

# Launch everything via the launch script.
# Set LOG_DIR and CONFIG_FILE so the launcher writes logs where we expect.
export LOG_DIR
export CONFIG_FILE
"${DEPLOY_DIR}/launch_gazebo.sh" > "${LOG_DIR}/launcher.log" 2>&1 &
LAUNCHER_PID=$!

echo "  Launcher PID: ${LAUNCHER_PID}"
echo "  Waiting for stack startup (~35s)..."

# Wait for PX4 + processes to start.
# The launcher waits up to 30s for MAVLink, then starts 7 processes.
sleep 35

# Verify launcher is still alive (meaning nothing crashed immediately)
if kill -0 "$LAUNCHER_PID" 2>/dev/null; then
    check "Launcher still running after startup" 0
else
    check "Launcher still running after startup" 1
    echo -e "${RED}  Launcher exited prematurely. Check ${LOG_DIR}/launcher.log${NC}"
fi

# ── Phase 3: Verification window ────────────────────────────
echo ""
echo "Phase 3: Running for ${VERIFY_TIME}s verification window..."
sleep "$VERIFY_TIME"

echo ""
echo "Phase 4: Checking results..."
echo ""

# ── Check log files exist and have content ───────────────────
echo "Log file verification:"
PROCESSES=("system_monitor" "video_capture" "comms" "perception"
           "slam_vio_nav" "mission_planner" "payload_manager")

for proc in "${PROCESSES[@]}"; do
    logfile="${LOG_DIR}/${proc}.log"
    if [[ -f "$logfile" ]] && [[ -s "$logfile" ]]; then
        check "${proc}.log has content" 0
    else
        check "${proc}.log has content" 1
    fi
done

# ── Check SHM segments exist ────────────────────────────────
# Names from common/ipc/include/ipc/shm_types.h::shm_names
echo ""
echo "SHM segment verification:"
SHM_SEGMENTS=("drone_mission_cam" "drone_stereo_cam"
              "detected_objects" "slam_pose"
              "mission_status" "fc_state"
              "system_health")
for seg in "${SHM_SEGMENTS[@]}"; do
    if [[ -f "/dev/shm/${seg}" ]]; then
        check "SHM: ${seg}" 0
    else
        check "SHM: ${seg}" 1
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

# Launcher log shows all processes launched
if grep -qi "All processes launched" "${LOG_DIR}/launcher.log" 2>/dev/null; then
    check "Launcher: all 7 processes started" 0
else
    check "Launcher: all 7 processes started" 1
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
