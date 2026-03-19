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
#   CONFIG_FILE      JSON config             (default: config/gazebo_sitl.json)
#   LOG_DIR          Log output directory    (default: <project>/drone_logs)
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
CONFIG_FILE="${CONFIG_FILE:-${PROJECT_DIR}/config/gazebo_sitl.json}"
LOG_DIR="${LOG_DIR:-${PROJECT_DIR}/drone_logs}"
PX4_MODEL="x500_companion"
HEADLESS=1
EXTRA_ARGS=""

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

PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_DIR}/build/px4_sitl_default/etc"
PX4_ROOTFS="${PX4_DIR}/build/px4_sitl_default/rootfs"

if [[ ! -x "$PX4_BIN" ]]; then
    echo "ERROR: PX4 SITL binary not found at ${PX4_BIN}."
    echo "       Run: cd ${PX4_DIR} && make px4_sitl_default"
    exit 1
fi
if [[ ! -d "$PX4_ETC" ]]; then
    echo "ERROR: PX4 SITL config dir not found at ${PX4_ETC}."
    exit 1
fi
if [[ ! -d "$PX4_ROOTFS" ]]; then
    echo "ERROR: PX4 SITL rootfs dir not found at ${PX4_ROOTFS}."
    echo "       Run: cd ${PX4_DIR} && make px4_sitl_default"
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

# Register custom model path so PX4/Gazebo can find x500_companion
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/sim/models:${PX4_DIR}/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

mkdir -p "$LOG_DIR"
export DRONE_LOG_DIR="$LOG_DIR"

echo "════════════════════════════════════════════════════════"
echo "  Drone Companion Stack — Gazebo SITL Launch"
echo "  PX4        : ${PX4_DIR}"
echo "  World      : ${GZ_WORLD}"
echo "  Config     : ${CONFIG_FILE}"
echo "  Model      : ${PX4_MODEL}"
echo "  Headless   : ${HEADLESS}"
echo "  Binaries   : ${BIN_DIR}"
echo "  Logs       : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

PX4_PID=""
COMPANION_PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all processes..."
    # 0. Stop GUI client
    if [[ -n "${GUI_PID:-}" ]]; then
        kill -SIGTERM "$GUI_PID" 2>/dev/null || true
    fi
    # 1. Stop companion processes by tracked PID
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGKILL "$pid" 2>/dev/null || true
    done
    # 2. Stop PX4 and its children (includes Gazebo sim server)
    if [[ -n "$PX4_PID" ]]; then
        # Kill children first (Gazebo server, bridges, etc.)
        pkill -SIGINT -P "$PX4_PID" 2>/dev/null || true
        sleep 1
        kill -SIGINT "$PX4_PID" 2>/dev/null || true
        sleep 1
        kill -SIGKILL "$PX4_PID" 2>/dev/null || true
        pkill -SIGKILL -P "$PX4_PID" 2>/dev/null || true
    fi
    # 3. Clean SHM segments
    clean_shm
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# ── Step 1: Start PX4 SITL + Gazebo ─────────────────────────
echo ""
echo "[SITL] Starting PX4 SITL with Gazebo (model=${PX4_MODEL})..."

export PX4_SYS_AUTOSTART=4001   # x500 quad airframe
export PX4_SIM_MODEL="${PX4_MODEL}"
export PX4_GZ_WORLD="test_world"

# Explicitly export HEADLESS for both cases so that inherited
# environment values don't override the --gui / default choice.
if [[ "$HEADLESS" -eq 1 ]]; then
    export HEADLESS=1
else
    export HEADLESS=0
fi

pushd "$PX4_ROOTFS" > /dev/null
# PX4 must run from its rootfs directory so that px4-rc.gzsim can
# find and source ./gz_env.sh, which sets PX4_GZ_WORLDS, PX4_GZ_MODELS,
# and GZ_SIM_RESOURCE_PATH. It then starts gz sim with the world from
# ${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf and spawns the model from
# ${PX4_GZ_MODELS}/${PX4_SIM_MODEL}/model.sdf.
# The x500_companion model and test_world.sdf are symlinked into
# PX4's gz directories; see deploy section in the README.
"$PX4_BIN" \
    -d "$PX4_ETC" \
    -s "${PX4_ETC}/init.d-posix/rcS" \
    > "${LOG_DIR}/px4_sitl.log" 2>&1 &
PX4_PID=$!
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

# ── Step 2b: Launch Gazebo GUI client (if --gui) ────────────
GUI_PID=""
if [[ "$HEADLESS" -eq 0 ]]; then
    echo ""
    echo "[GUI] Launching Gazebo GUI client..."
    GUI_CONFIG="${PROJECT_DIR}/sim/gui.config"
    # Use clean environment to avoid Snap/Conda library conflicts
    env -i \
        HOME="$HOME" \
        DISPLAY="${DISPLAY:-:1}" \
        WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
        XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
        PATH="/usr/bin:/usr/local/bin:/bin" \
        GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}" \
        gz sim -g --gui-config "$GUI_CONFIG" \
        > "${LOG_DIR}/gz_gui.log" 2>&1 &
    GUI_PID=$!
    echo "[GUI] Gazebo GUI PID: ${GUI_PID}"

    # Give the GUI time to connect and render, then tell camera to
    # follow the drone model so it stays in view during flight.
    (
        sleep 8
        echo "[GUI] Requesting camera to follow drone..."
        # Set follow offset: 6m behind, 3m above
        env -i \
            HOME="$HOME" \
            PATH="/usr/bin:/usr/local/bin:/bin" \
            gz service -s /gui/follow/offset \
            --reqtype gz.msgs.Vector3d \
            --reptype gz.msgs.Boolean \
            --timeout 5000 \
            --req "x: -6, y: 0, z: 3" \
            > /dev/null 2>&1 || true
        # Start following the drone model
        env -i \
            HOME="$HOME" \
            PATH="/usr/bin:/usr/local/bin:/bin" \
            gz service -s /gui/follow \
            --reqtype gz.msgs.StringMsg \
            --reptype gz.msgs.Boolean \
            --timeout 5000 \
            --req "data: 'x500_companion_0'" \
            > /dev/null 2>&1 || true
        echo "[GUI] Camera follow request sent"
    ) &
fi

# ── Step 3: Launch companion stack ───────────────────────────
CONFIG_ARG="--config ${CONFIG_FILE}"

echo ""
echo "[Stack] Launching 7 companion processes..."

echo "  [1/7] system_monitor"
"${BIN_DIR}/system_monitor" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/system_monitor.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [2/7] video_capture"
"${BIN_DIR}/video_capture" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/video_capture.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [3/7] comms"
"${BIN_DIR}/comms" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/comms.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 1  # comms needs time to connect to PX4

echo "  [4/7] perception"
"${BIN_DIR}/perception" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/perception.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [5/7] slam_vio_nav"
"${BIN_DIR}/slam_vio_nav" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/slam_vio_nav.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [6/7] mission_planner"
"${BIN_DIR}/mission_planner" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/mission_planner.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [7/7] payload_manager"
"${BIN_DIR}/payload_manager" ${CONFIG_ARG} ${EXTRA_ARGS} \
    > "${LOG_DIR}/payload_manager.log" 2>&1 &
COMPANION_PIDS+=($!)

echo ""
echo "All processes launched. PIDs: PX4=${PX4_PID} Companion=${COMPANION_PIDS[*]}"
echo "Logs: ${LOG_DIR}/"
echo "Press Ctrl+C to stop everything."
echo ""

# ── Step 4: Monitor ──────────────────────────────────────────
# Wait for any process to exit, then trigger shutdown with its exit code.
# FIXME(BUG-29): PX4_PID is included in ALL_PIDS, so a normal PX4 exit
# (mission complete, landing, SIGINT from a previous scenario runner) tears
# down the entire companion stack and kills the Gazebo GUI immediately.
# Fix: monitor companion processes only; handle PX4 exit separately.
# GitHub: https://github.com/nmohamaya/companion_software_stack/issues/129
# See docs/BUG_FIXES.md — Bug #29 (OPEN).
ALL_PIDS=("$PX4_PID" "${COMPANION_PIDS[@]}")
wait -n "${ALL_PIDS[@]}" 2>/dev/null && EXIT_CODE=0 || EXIT_CODE=$?

echo "A process exited (code=${EXIT_CODE}) — shutting down stack."
exit "$EXIT_CODE"
