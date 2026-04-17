#!/usr/bin/env bash
# deploy/launch_cosys.sh — Launch Cosys-AirSim UE5 + PX4 SITL + companion stack.
#
# Usage:
#   ./deploy/launch_cosys.sh                       # headless (default, dev profile)
#   ./deploy/launch_cosys.sh --gui                 # with UE5 rendering window
#   ./deploy/launch_cosys.sh --profile cloud       # cloud config overlay
#   ./deploy/launch_cosys.sh --env Neighborhood    # UE5 environment (default: Blocks)
#   ./deploy/launch_cosys.sh --log-level debug     # pass args to stack
#
# Environment variables (override defaults):
#   COSYS_DIR        Path to Cosys-AirSim          (default: third_party/cosys-airsim if submodule exists, else ~/Cosys-AirSim)
#   PX4_DIR          Path to PX4-Autopilot          (default: ~/PX4-Autopilot)
#   CONFIG_FILE      JSON config (overrides profile) (default: from --profile)
#   LOG_DIR          Log output directory            (default: <project>/drone_logs)
#   COSYS_RPC_PORT   RPC port                        (default: 41451)
#
# Prerequisites:
#   - Cosys-AirSim built with UE5 (or pre-built binary)
#   - PX4-Autopilot built for SITL (make px4_sitl_default)
#   - NVIDIA GPU with >= 8 GB VRAM
#   - Companion stack built with -DHAVE_COSYS_AIRSIM
#
# Issue: #463 (part of Epic #459)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"

# ── Defaults ──────────────────────────────────────────────────
# Try to detect Cosys-AirSim from submodule first, then fall back to ~/Cosys-AirSim
if [[ -d "${PROJECT_DIR}/third_party/cosys-airsim" ]]; then
    COSYS_DIR="${COSYS_DIR:-${PROJECT_DIR}/third_party/cosys-airsim}"
else
    COSYS_DIR="${COSYS_DIR:-${HOME}/Cosys-AirSim}"
fi
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-${PROJECT_DIR}/drone_logs}"
COSYS_RPC_PORT="${COSYS_RPC_PORT:-41451}"
HEADLESS=1
PROFILE="dev"
UE5_ENV="Blocks"
EXTRA_ARGS=""

# ── Parse arguments ──────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            HEADLESS=0
            shift
            ;;
        --profile)
            PROFILE="${2:-}"
            if [[ -z "$PROFILE" ]]; then
                echo "ERROR: --profile requires an argument (dev|cloud)"
                exit 1
            fi
            shift 2
            ;;
        --env)
            UE5_ENV="${2:-}"
            if [[ -z "$UE5_ENV" ]]; then
                echo "ERROR: --env requires a UE5 environment name"
                exit 1
            fi
            shift 2
            ;;
        *)
            EXTRA_ARGS="${EXTRA_ARGS} ${1}"
            shift
            ;;
    esac
done

# ── Resolve config file from profile ────────────────────────
if [[ -z "${CONFIG_FILE:-}" ]]; then
    case "$PROFILE" in
        dev)
            CONFIG_FILE="${PROJECT_DIR}/config/cosys_airsim_dev.json"
            ;;
        cloud)
            CONFIG_FILE="${PROJECT_DIR}/config/cosys_airsim.json"
            ;;
        *)
            echo "ERROR: Unknown profile '${PROFILE}'. Use 'dev' or 'cloud'."
            exit 1
            ;;
    esac
fi

# ── Validate paths ───────────────────────────────────────────
if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: Build directory not found at ${BIN_DIR}."
    echo "       Run: bash deploy/build.sh"
    exit 1
fi
if [[ ! -d "$COSYS_DIR" ]]; then
    echo "ERROR: Cosys-AirSim not found at ${COSYS_DIR}."
    echo "       Options:"
    echo "         1. Initialize submodule: git submodule update --init third_party/cosys-airsim"
    echo "         2. Set COSYS_DIR env: export COSYS_DIR=/path/to/cosys-airsim"
    echo "         3. Install: bash deploy/install_cosys.sh"
    exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4-Autopilot not found at ${PX4_DIR}."
    echo "       Set PX4_DIR env or clone PX4-Autopilot."
    exit 1
fi
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "ERROR: Config file not found: ${CONFIG_FILE}"
    exit 1
fi

# Locate PX4 SITL binary
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

# ── GPU check ────────────────────────────────────────────────
if ! command -v nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-smi not found. NVIDIA GPU required for Cosys-AirSim."
    exit 1
fi
# Check VRAM (need >= 8 GB for UE5 rendering)
VRAM_MB=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
if [[ -n "$VRAM_MB" ]] && [[ "$VRAM_MB" -lt 8000 ]]; then
    echo "ERROR: GPU VRAM is ${VRAM_MB} MB — Cosys-AirSim requires >= 8 GB."
    echo "       Available GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
    exit 1
fi

# Ensure system libstdc++ is used instead of Anaconda's older version
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

mkdir -p "$LOG_DIR"
export DRONE_LOG_DIR="$LOG_DIR"

echo "════════════════════════════════════════════════════════"
echo "  Drone Companion Stack — Cosys-AirSim SITL Launch"
echo "  Cosys-AirSim : ${COSYS_DIR}"
echo "  PX4          : ${PX4_DIR}"
echo "  UE5 Env      : ${UE5_ENV}"
echo "  Config       : ${CONFIG_FILE}"
echo "  Profile      : ${PROFILE}"
echo "  Headless     : ${HEADLESS}"
echo "  RPC Port     : ${COSYS_RPC_PORT}"
echo "  Binaries     : ${BIN_DIR}"
echo "  Logs         : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

UE5_PID=""
PX4_PID=""
COMPANION_PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all processes..."
    # 1. Stop companion processes
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGKILL "$pid" 2>/dev/null || true
    done
    # 2. Stop PX4
    if [[ -n "$PX4_PID" ]]; then
        pkill -SIGINT -P "$PX4_PID" 2>/dev/null || true
        sleep 1
        kill -SIGINT "$PX4_PID" 2>/dev/null || true
        sleep 1
        kill -SIGKILL "$PX4_PID" 2>/dev/null || true
        pkill -SIGKILL -P "$PX4_PID" 2>/dev/null || true
    fi
    # 3. Stop UE5/Cosys-AirSim
    if [[ -n "$UE5_PID" ]]; then
        kill -SIGTERM "$UE5_PID" 2>/dev/null || true
        sleep 2
        kill -SIGKILL "$UE5_PID" 2>/dev/null || true
    fi
    # 4. Clean SHM segments
    rm -f /dev/shm/zenoh_shm_* 2>/dev/null || true
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# ── Step 1: Start Cosys-AirSim (UE5) ────────────────────────
echo ""
echo "[UE5] Starting Cosys-AirSim (env=${UE5_ENV})..."

# Locate UE5 binary — check common build output locations
UE5_BIN=""
CANDIDATE_PATHS=(
    "${COSYS_DIR}/Unreal/Environments/${UE5_ENV}/Binaries/Linux/${UE5_ENV}"
    "${COSYS_DIR}/Unreal/Environments/${UE5_ENV}/Binaries/Linux/${UE5_ENV}-Linux-Shipping"
    "${COSYS_DIR}/build/${UE5_ENV}/${UE5_ENV}"
    "${COSYS_DIR}/${UE5_ENV}/LinuxNoEditor/${UE5_ENV}.sh"
    "${COSYS_DIR}/${UE5_ENV}/${UE5_ENV}.sh"
)
for candidate in "${CANDIDATE_PATHS[@]}"; do
    if [[ -x "$candidate" ]]; then
        UE5_BIN="$candidate"
        break
    fi
done

if [[ -z "$UE5_BIN" ]]; then
    echo "ERROR: Could not find UE5 binary for environment '${UE5_ENV}'."
    echo "       Searched:"
    for candidate in "${CANDIDATE_PATHS[@]}"; do
        echo "         ${candidate}"
    done
    echo "       Set COSYS_DIR to the Cosys-AirSim installation root."
    exit 1
fi

UE5_ARGS=()
if [[ "$HEADLESS" -eq 1 ]]; then
    UE5_ARGS+=(-RenderOffScreen)
fi
# Pass AirSim settings path if cosys_settings.json exists
COSYS_SETTINGS="${PROJECT_DIR}/config/cosys_settings.json"
if [[ -f "$COSYS_SETTINGS" ]]; then
    # Cosys-AirSim reads ~/Documents/AirSim/settings.json by default;
    # symlink or copy our project settings there.
    AIRSIM_SETTINGS_DIR="${HOME}/Documents/AirSim"
    mkdir -p "$AIRSIM_SETTINGS_DIR"
    cp "$COSYS_SETTINGS" "${AIRSIM_SETTINGS_DIR}/settings.json"
    echo "[UE5] Deployed settings to ${AIRSIM_SETTINGS_DIR}/settings.json"
fi

"$UE5_BIN" "${UE5_ARGS[@]}" > "${LOG_DIR}/cosys_ue5.log" 2>&1 &
UE5_PID=$!
echo "[UE5] PID: ${UE5_PID}"

# ── Step 2: Wait for RPC port ────────────────────────────────
echo "[UE5] Waiting for RPC port ${COSYS_RPC_PORT}..."
MAX_WAIT=60
WAITED=0
while [[ $WAITED -lt $MAX_WAIT ]]; do
    if ss -tlnp 2>/dev/null | grep -q ":${COSYS_RPC_PORT}"; then
        echo "[UE5] RPC port ${COSYS_RPC_PORT} ready after ${WAITED}s"
        break
    fi
    # Check UE5 hasn't crashed
    if ! kill -0 "$UE5_PID" 2>/dev/null; then
        echo "ERROR: Cosys-AirSim process died — check ${LOG_DIR}/cosys_ue5.log"
        tail -20 "${LOG_DIR}/cosys_ue5.log" 2>/dev/null || true
        exit 1
    fi
    sleep 1
    WAITED=$((WAITED + 1))
done
if [[ $WAITED -ge $MAX_WAIT ]]; then
    echo "WARNING: RPC port ${COSYS_RPC_PORT} not detected after ${MAX_WAIT}s — continuing anyway"
fi
# Additional settle time for UE5 to fully initialise
sleep 3

# ── Step 3: Start PX4 SITL ──────────────────────────────────
echo ""
echo "[SITL] Starting PX4 SITL..."

export PX4_SYS_AUTOSTART=4001   # x500 quad airframe
export PX4_SIM_MODEL="x500"
# Use none_iris for SITL without Gazebo — PX4 provides the sim interface,
# Cosys-AirSim provides the physics/sensors via its own MAVLink bridge.
export PX4_SIM_HOSTNAME=127.0.0.1

pushd "$PX4_ROOTFS" > /dev/null
"$PX4_BIN" \
    -d "$PX4_ETC" \
    -s "${PX4_ETC}/init.d-posix/rcS" \
    > "${LOG_DIR}/px4_sitl.log" 2>&1 &
PX4_PID=$!
popd > /dev/null

echo "[SITL] PX4 PID: ${PX4_PID}"

# ── Step 4: Wait for MAVLink heartbeat ───────────────────────
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
sleep 3

# ── Step 5: Launch companion stack ───────────────────────────
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
echo "All processes launched."
echo "PIDs: UE5=${UE5_PID} PX4=${PX4_PID} Companion=${COMPANION_PIDS[*]}"
echo "Logs: ${LOG_DIR}/"
echo "Press Ctrl+C to stop everything."
echo ""

# ── Step 6: Monitor ──────────────────────────────────────────
# Background UE5 watcher — logs when UE5 exits.
(
    wait "$UE5_PID" 2>/dev/null
    UE5_EXIT=$?
    echo "[UE5-watch] Cosys-AirSim exited (code=${UE5_EXIT})."
) &
UE5_WATCHER_PID=$!

# Background PX4 watcher — logs when PX4 exits.
(
    wait "$PX4_PID" 2>/dev/null
    PX4_EXIT=$?
    echo "[PX4-watch] PX4 exited (code=${PX4_EXIT}) — companions continue running."
) &
PX4_WATCHER_PID=$!

# Wait for the first companion process to exit.
wait -n "${COMPANION_PIDS[@]}" 2>/dev/null && EXIT_CODE=0 || EXIT_CODE=$?

echo "Companion process exited (code=${EXIT_CODE}) — shutting down stack."
kill "$UE5_WATCHER_PID" 2>/dev/null || true
kill "$PX4_WATCHER_PID" 2>/dev/null || true
exit "$EXIT_CODE"
