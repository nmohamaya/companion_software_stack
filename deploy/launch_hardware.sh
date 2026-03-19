#!/usr/bin/env bash
# deploy/launch_hardware.sh — Launch companion stack on real drone hardware.
#
# Connects to a Pixhawk flight controller over serial (or UDP), launches
# all 7 companion processes with the hardware config, and provides
# clean shutdown on Ctrl+C or process failure.
#
# Usage:
#   ./deploy/launch_hardware.sh                           # defaults
#   ./deploy/launch_hardware.sh --config config/my_drone.json
#   ./deploy/launch_hardware.sh --log-level debug
#   ./deploy/launch_hardware.sh --dry-run                 # validate only, don't start
#
# Environment variables (override defaults):
#   CONFIG_FILE      JSON config         (default: config/hardware.json)
#   LOG_DIR          Log output dir      (default: <project>/drone_logs)
#   FC_DEVICE        Serial device       (overrides config comms.mavlink.uri)
#
# Prerequisites:
#   - Companion stack built (deploy/build.sh)
#   - MAVSDK 2.x installed
#   - Flight controller connected via USB or UART
#   - User in 'dialout' group for serial access (sudo usermod -aG dialout $USER)
#
# Issue: #26
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"

# ── Defaults ──────────────────────────────────────────────────
CONFIG_FILE="${CONFIG_FILE:-${PROJECT_DIR}/config/hardware.json}"
LOG_DIR="${LOG_DIR:-${PROJECT_DIR}/drone_logs}"
DRY_RUN=0
EXTRA_ARGS=""

# ── Parse arguments ──────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        *)
            EXTRA_ARGS="${EXTRA_ARGS} ${1}"
            shift
            ;;
    esac
done

# ── Validate build ───────────────────────────────────────────
if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: Build directory not found at ${BIN_DIR}."
    echo "       Run: cmake -S . -B build && cmake --build build"
    exit 1
fi

REQUIRED_BINS=(system_monitor video_capture comms perception slam_vio_nav mission_planner payload_manager)
for bin in "${REQUIRED_BINS[@]}"; do
    if [[ ! -x "${BIN_DIR}/${bin}" ]]; then
        echo "ERROR: Binary not found: ${BIN_DIR}/${bin}"
        echo "       Run: deploy/build.sh"
        exit 1
    fi
done

# ── Validate config ──────────────────────────────────────────
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "ERROR: Config file not found: ${CONFIG_FILE}"
    echo "       Available configs:"
    ls -1 "${PROJECT_DIR}/config/" 2>/dev/null | sed 's/^/         /'
    exit 1
fi

# ── Validate JSON syntax ─────────────────────────────────────
if command -v python3 &>/dev/null; then
    if ! python3 -c "import json, sys; json.load(open(sys.argv[1]))" "$CONFIG_FILE" 2>/dev/null; then
        echo "ERROR: Invalid JSON in ${CONFIG_FILE}"
        exit 1
    fi
    echo "[OK] Config JSON valid: ${CONFIG_FILE}"
elif command -v jq &>/dev/null; then
    if ! jq empty "${CONFIG_FILE}" 2>/dev/null; then
        echo "ERROR: Invalid JSON in ${CONFIG_FILE}"
        exit 1
    fi
    echo "[OK] Config JSON valid: ${CONFIG_FILE}"
else
    echo "[WARN] Neither python3 nor jq found — skipping JSON validation"
fi

# ── Detect FC serial device ──────────────────────────────────
# If FC_DEVICE env is set, it overrides whatever is in the config.
# Otherwise, try to auto-detect common Pixhawk USB serial devices.
detect_fc_device() {
    # Check for common Pixhawk USB devices
    local candidates=(
        "/dev/ttyACM0"    # Pixhawk via USB (most common)
        "/dev/ttyACM1"    # Secondary USB
        "/dev/ttyUSB0"    # USB-to-serial adapter
        "/dev/ttyTHS1"    # Jetson Orin Nano UART1
        "/dev/ttyAMA0"    # Raspberry Pi UART
        "/dev/serial0"    # RPi serial alias
    )
    for dev in "${candidates[@]}"; do
        if [[ -c "$dev" ]]; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

if [[ -n "${FC_DEVICE:-}" ]]; then
    echo "[OK] FC device (override): ${FC_DEVICE}"
    if [[ ! -c "$FC_DEVICE" ]]; then
        echo "ERROR: FC_DEVICE=${FC_DEVICE} does not exist or is not a character device"
        exit 1
    fi
elif DETECTED_DEV=$(detect_fc_device); then
    echo "[OK] FC device (auto-detected): ${DETECTED_DEV}"
else
    echo "[WARN] No serial FC device found — will use URI from config"
    echo "       If connecting over UDP, this is expected."
fi

# ── Check serial permissions ─────────────────────────────────
check_serial_perms() {
    local dev="$1"
    if [[ ! -c "$dev" ]]; then
        return 0  # device doesn't exist yet, skip check
    fi
    if [[ -r "$dev" && -w "$dev" ]]; then
        echo "[OK] Serial permissions: ${dev}"
        return 0
    fi
    local dev_group
    dev_group=$(stat -c '%G' "$dev" 2>/dev/null || echo "unknown")
    echo "ERROR: Cannot read/write ${dev} (group: ${dev_group})"
    echo "       Fix: sudo usermod -aG ${dev_group} \$USER && logout/login"
    echo "       For more advanced setups, configure persistent serial access via udev rules or system-specific device access controls."
    return 1
}

SERIAL_DEV="${FC_DEVICE:-${DETECTED_DEV:-}}"
if [[ -n "$SERIAL_DEV" ]]; then
    check_serial_perms "$SERIAL_DEV" || exit 1

    # If a serial FC device is set/detected, try to override comms.mavlink.uri
    # in the config so that MAVSDK connects to the correct device.
    if command -v jq >/dev/null 2>&1; then
        # Read existing URI (may be empty or non-serial).
        CURRENT_URI="$(jq -r '.comms.mavlink.uri // empty' "${CONFIG_FILE}" 2>/dev/null || echo "")"
        if [[ -n "${CURRENT_URI}" && "${CURRENT_URI}" == serial://* ]]; then
            # Preserve baud rate suffix (e.g. :921600).
            BAUD="${CURRENT_URI##*:}"
            NEW_URI="serial://${SERIAL_DEV}:${BAUD}"

            TEMP_CONFIG="$(mktemp --suffix=.hardware.json)"
            if jq --arg uri "${NEW_URI}" '.comms.mavlink.uri = $uri' "${CONFIG_FILE}" > "${TEMP_CONFIG}"; then
                CONFIG_FILE="${TEMP_CONFIG}"
                echo "[OK] Overriding comms.mavlink.uri to: ${NEW_URI}"
                echo "     Using temporary config: ${CONFIG_FILE}"
            else
                echo "[WARN] Failed to write temporary config for MAVLink URI override; using original ${CONFIG_FILE}"
                rm -f "${TEMP_CONFIG}" || true
            fi
        else
            echo "[WARN] FC device set (${SERIAL_DEV}) but config comms.mavlink.uri is not serial://*; leaving URI unchanged."
        fi
    else
        echo "[WARN] jq not found; cannot override comms.mavlink.uri based on FC_DEVICE/SERIAL_DEV."
    fi
fi

# ── Check MAVSDK availability ─────────────────────────────────
# The comms binary needs MAVSDK linked. Verify at runtime by checking
# if the binary can at least print usage without missing-lib errors.
if ! ldd "${BIN_DIR}/comms" 2>/dev/null | grep -q "libmavsdk"; then
    echo "[WARN] comms binary may not have MAVSDK linked."
    echo "       Ensure you built with -DHAVE_MAVSDK=ON"
fi

# ── Validate system resources ─────────────────────────────────
echo ""
echo "── System Check ──────────────────────────────────"

# Disk space (warn if < 500 MB free on log/project partition)
# Use PROJECT_DIR for the check since LOG_DIR may not exist yet.
AVAIL_KB=$(df --output=avail "${PROJECT_DIR}" 2>/dev/null | tail -1 | tr -d ' ' || echo "0")
if [[ "$AVAIL_KB" -lt 512000 ]]; then
    echo "[WARN] Low disk space: $(( AVAIL_KB / 1024 )) MB free"
else
    echo "[OK] Disk space: $(( AVAIL_KB / 1024 )) MB free"
fi

# CPU temperature (warn if > 70°C before we even start)
if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
    TEMP_MC=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$(( TEMP_MC / 1000 ))
    if [[ "$TEMP_C" -gt 70 ]]; then
        echo "[WARN] CPU temperature: ${TEMP_C}°C (high before flight)"
    else
        echo "[OK] CPU temperature: ${TEMP_C}°C"
    fi
fi

# Memory
MEM_AVAIL_KB=$(grep MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo "0")
if [[ "$MEM_AVAIL_KB" -lt 256000 ]]; then
    echo "[WARN] Low memory: $(( MEM_AVAIL_KB / 1024 )) MB available"
else
    echo "[OK] Memory: $(( MEM_AVAIL_KB / 1024 )) MB available"
fi

echo "───────────────────────────────────────────────────"

# ── Dry-run exit ──────────────────────────────────────────────
if [[ "$DRY_RUN" -eq 1 ]]; then
    echo ""
    echo "[DRY RUN] All pre-flight checks passed. Would launch with:"
    echo "  Config  : ${CONFIG_FILE}"
    echo "  Logs    : ${LOG_DIR}"
    echo "  FC dev  : ${SERIAL_DEV:-<from config>}"
    echo ""
    echo "Run without --dry-run to start the stack."
    exit 0
fi

# ── Ensure system libstdc++ ───────────────────────────────────
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Also handle aarch64 (Jetson/RPi)
if [[ -d "/usr/lib/aarch64-linux-gnu" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
fi

mkdir -p "$LOG_DIR"
export DRONE_LOG_DIR="$LOG_DIR"

# ── Banner ────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  Drone Companion Stack — Hardware Launch"
echo "  Config   : ${CONFIG_FILE}"
echo "  FC device: ${SERIAL_DEV:-<from config URI>}"
echo "  Binaries : ${BIN_DIR}"
echo "  Logs     : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

CONFIG_ARGS=(--config "${CONFIG_FILE}")
EXTRA_ARGS_ARR=()
if [[ -n "${EXTRA_ARGS}" ]]; then
    # shellcheck disable=SC2206  # intentional word-splitting of user-supplied flags
    EXTRA_ARGS_ARR=(${EXTRA_ARGS})
fi
COMPANION_PIDS=()
MONITOR_PID=""

cleanup() {
    echo ""
    echo "[Shutdown] Stopping all processes..."
    # Stop monitor first to avoid log noise during shutdown
    if [[ -n "${MONITOR_PID}" ]]; then
        kill "$MONITOR_PID" 2>/dev/null || true
    fi
    # Send SIGINT first for graceful shutdown
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null || true
    done
    sleep 2
    # Force-kill stragglers
    for pid in "${COMPANION_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    # Clean SHM
    clean_shm
    echo "[Shutdown] All processes stopped."
}
trap cleanup EXIT INT TERM

# ── Launch companion stack ────────────────────────────────────
# Order matches dependency graph (same as launch_gazebo.sh):
# 1. System Monitor (no deps)
# 2. Video Capture (no deps)
# 3. Comms (no deps — connects to FC)
# 4. Perception (needs Video Capture IPC)
# 5. SLAM/VIO/Nav (needs Video Capture + IMU IPC)
# 6. Mission Planner (needs SLAM + Perception + Comms IPC)
# 7. Payload Manager (needs Mission Planner IPC)
# IPC backend (SHM or Zenoh) is determined by the config file.

echo ""
echo "[Stack] Launching 7 companion processes..."

echo "  [1/7] system_monitor"
"${BIN_DIR}/system_monitor" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/system_monitor.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [2/7] video_capture"
"${BIN_DIR}/video_capture" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/video_capture.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [3/7] comms (connecting to FC...)"
"${BIN_DIR}/comms" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/comms.log" 2>&1 &
COMPANION_PIDS+=($!)
# Give comms extra time to establish MAVLink connection
sleep 3

# Verify comms actually connected to FC (not just alive)
COMMS_CONNECTED=0
if ! kill -0 "${COMPANION_PIDS[2]}" 2>/dev/null; then
    echo "ERROR: comms process died — check ${LOG_DIR}/comms.log"
    echo "       Common causes:"
    echo "         - FC not connected on ${SERIAL_DEV:-<config URI>}"
    echo "         - Wrong baud rate"
    echo "         - Missing dialout group membership"
    tail -5 "${LOG_DIR}/comms.log" 2>/dev/null || true
    exit 1
fi
# Check log for definitive connection status (up to 10s total)
for i in $(seq 1 7); do
    if grep -qi "connected to autopilot\|system discovered\|mavlink.*connected" "${LOG_DIR}/comms.log" 2>/dev/null; then
        COMMS_CONNECTED=1
        break
    fi
    if grep -qi "connection.*failed\|error.*connect\|timeout" "${LOG_DIR}/comms.log" 2>/dev/null; then
        echo "ERROR: comms reported connection failure — check ${LOG_DIR}/comms.log"
        tail -10 "${LOG_DIR}/comms.log" 2>/dev/null || true
        exit 1
    fi
    sleep 1
done
if [[ "$COMMS_CONNECTED" -eq 1 ]]; then
    echo "  [3/7] comms connected to FC"
else
    echo "  [3/7] comms alive (connection status not confirmed in log — continuing)"
fi

echo "  [4/7] perception"
"${BIN_DIR}/perception" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/perception.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [5/7] slam_vio_nav"
"${BIN_DIR}/slam_vio_nav" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/slam_vio_nav.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [6/7] mission_planner"
"${BIN_DIR}/mission_planner" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/mission_planner.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [7/7] payload_manager"
"${BIN_DIR}/payload_manager" "${CONFIG_ARGS[@]}" "${EXTRA_ARGS_ARR[@]}" \
    > "${LOG_DIR}/payload_manager.log" 2>&1 &
COMPANION_PIDS+=($!)

echo ""
echo "All 7 processes launched."
echo "PIDs: ${COMPANION_PIDS[*]}"
echo "Logs: ${LOG_DIR}/"
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║  Press Ctrl+C to stop all processes             ║"
echo "║  tail -f ${LOG_DIR}/mission_planner.log  ║"
echo "║  tail -f ${LOG_DIR}/comms.log            ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── Monitor ──────────────────────────────────────────────────
# Print a periodic status line while running
(
    while true; do
        sleep 30
        ALIVE=0
        for pid in "${COMPANION_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                ALIVE=$((ALIVE + 1))
            fi
        done
        if [[ "$ALIVE" -lt 7 ]]; then
            echo "[Monitor] WARNING: only ${ALIVE}/7 processes alive"
        fi
    done
) &
MONITOR_PID=$!

# Wait for any companion process to exit, then trigger shutdown
wait -n "${COMPANION_PIDS[@]}" 2>/dev/null && EXIT_CODE=0 || EXIT_CODE=$?
kill "$MONITOR_PID" 2>/dev/null || true

echo "A process exited (code=${EXIT_CODE}) — shutting down stack."
exit "$EXIT_CODE"
