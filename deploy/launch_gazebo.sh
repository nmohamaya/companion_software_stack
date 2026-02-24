#!/usr/bin/env bash
# deploy/launch_gazebo.sh — Launch PX4 SITL + Gazebo + companion stack.
#
# Usage:
#   ./deploy/launch_gazebo.sh                    # headless (default)
#   ./deploy/launch_gazebo.sh --gui              # with Gazebo GUI
#   ./deploy/launch_gazebo.sh --log-level debug  # pass args to stack
#
# Environment variables (override defaults):
#   PX4_DIR          Path to PX4-Autopilot   (default: ~/PX4-Autopilot)
#   GZ_WORLD         SDF world file          (default: sim/worlds/test_world.sdf)
#   CONFIG_FILE      JSON config             (default: config/gazebo.json)
#
# Prerequisites:
#   - PX4-Autopilot built for SITL (make px4_sitl_default)
#   - Gazebo Harmonic installed
#   - MAVSDK 2.x installed
#   - Companion stack built with -DHAVE_MAVSDK -DHAVE_GAZEBO
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"

# ── Defaults ──────────────────────────────────────────────────
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
GZ_WORLD="${GZ_WORLD:-${PROJECT_DIR}/sim/worlds/test_world.sdf}"
CONFIG_FILE="${CONFIG_FILE:-${PROJECT_DIR}/config/gazebo.json}"
PX4_MODEL="x500"
HEADLESS=1
EXTRA_ARGS=""
LOG_DIR="/tmp/drone_logs"

# ── Parse arguments ──────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --gui)      HEADLESS=0 ;;
        *)          EXTRA_ARGS="${EXTRA_ARGS} ${arg}" ;;
    esac
done

# ── Validate paths ───────────────────────────────────────────
if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: Build directory not found at ${BIN_DIR}."
    echo "       Run: cmake -S . -B build && cmake --build build"
    exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4-Autopilot not found at ${PX4_DIR}."
    echo "       Set PX4_DIR env or clone PX4-Autopilot."
    exit 1
fi
if [[ ! -f "$GZ_WORLD" ]]; then
    echo "ERROR: World file not found: ${GZ_WORLD}"
    exit 1
fi
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "ERROR: Config file not found: ${CONFIG_FILE}"
    exit 1
fi

# Ensure system libstdc++ is used instead of Anaconda's older version
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ── Clean stale SHM ──────────────────────────────────────────
rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
      /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health 2>/dev/null || true
mkdir -p "$LOG_DIR"

echo "════════════════════════════════════════════════════════"
echo "  Drone Companion Stack — Gazebo SITL Launch"
echo "  PX4        : ${PX4_DIR}"
echo "  World      : ${GZ_WORLD}"
echo "  Config     : ${CONFIG_FILE}"
echo "  Headless   : ${HEADLESS}"
echo "  Binaries   : ${BIN_DIR}"
echo "  Logs       : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    # Kill any leftover Gazebo / PX4 processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# ── Step 1: Start PX4 SITL + Gazebo ─────────────────────────
echo ""
echo "[SITL] Starting PX4 SITL with Gazebo (model=${PX4_MODEL})..."

export PX4_SYS_AUTOSTART=4001   # x500 quad
export PX4_GZ_MODEL="${PX4_MODEL}"
export PX4_GZ_WORLD_FILE="${GZ_WORLD}"

if [[ "$HEADLESS" -eq 1 ]]; then
    export HEADLESS=1
fi

pushd "$PX4_DIR" > /dev/null
# Use the PX4 SITL launch mechanism
./build/px4_sitl_default/bin/px4 \
    -d ./build/px4_sitl_default/etc \
    -s ./build/px4_sitl_default/etc/init.d-posix/rcS \
    > "${LOG_DIR}/px4_sitl.log" 2>&1 &
PX4_PID=$!
PIDS+=($PX4_PID)
popd > /dev/null

echo "[SITL] PX4 PID: ${PX4_PID}"

# ── Step 2: Wait for MAVLink heartbeat ───────────────────────
echo "[SITL] Waiting for PX4 MAVLink heartbeat on udp://:14540..."
MAX_WAIT=30
WAITED=0
while [[ $WAITED -lt $MAX_WAIT ]]; do
    if ss -ulnp 2>/dev/null | grep -q ":14540" || \
       ss -ulnp 2>/dev/null | grep -q ":14580"; then
        echo "[SITL] MAVLink port detected after ${WAITED}s"
        break
    fi
    sleep 1
    WAITED=$((WAITED + 1))
done
if [[ $WAITED -ge $MAX_WAIT ]]; then
    echo "WARNING: MAVLink port not detected after ${MAX_WAIT}s — continuing anyway"
fi
# Additional settle time for PX4 to fully initialise
sleep 3

# ── Step 3: Launch companion stack ───────────────────────────
CONFIG_ARG="--config ${CONFIG_FILE}"

echo ""
echo "[Stack] Launching 7 companion processes..."

echo "  [1/7] system_monitor"
"${BIN_DIR}/system_monitor" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/system_monitor.log" 2>&1 &
PIDS+=($!)
sleep 0.3

echo "  [2/7] video_capture"
"${BIN_DIR}/video_capture" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/video_capture.log" 2>&1 &
PIDS+=($!)
sleep 0.3

echo "  [3/7] comms"
"${BIN_DIR}/comms" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/comms.log" 2>&1 &
PIDS+=($!)
sleep 1  # comms needs time to connect to PX4

echo "  [4/7] perception"
"${BIN_DIR}/perception" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/perception.log" 2>&1 &
PIDS+=($!)
sleep 0.3

echo "  [5/7] slam_vio_nav"
"${BIN_DIR}/slam_vio_nav" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/slam_vio_nav.log" 2>&1 &
PIDS+=($!)
sleep 0.3

echo "  [6/7] mission_planner"
"${BIN_DIR}/mission_planner" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/mission_planner.log" 2>&1 &
PIDS+=($!)
sleep 0.3

echo "  [7/7] payload_manager"
"${BIN_DIR}/payload_manager" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/payload_manager.log" 2>&1 &
PIDS+=($!)

echo ""
echo "All processes launched. PIDs: ${PIDS[*]}"
echo "Logs: ${LOG_DIR}/"
echo "Press Ctrl+C to stop everything."
echo ""

# ── Step 4: Monitor ──────────────────────────────────────────
# Wait for any process to exit, then trigger shutdown
wait -n "${PIDS[@]}" 2>/dev/null || true
echo "A process exited — shutting down stack."
