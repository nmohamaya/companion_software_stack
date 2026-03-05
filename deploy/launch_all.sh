#!/usr/bin/env bash
# deploy/launch_all.sh — Launch all 7 processes in correct order.
#
# Two modes:
#   --supervised   P7 (system_monitor) forks+execs P1–P6 internally.
#                  The script only manages one process.
#   (default)      The script launches all 7 processes directly (legacy).
#
# Usage: ./deploy/launch_all.sh [--supervised] [--config path] [--sim] [--log-level debug]
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"

if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: Build directory not found. Run deploy/build.sh first."
    exit 1
fi

# Check for --supervised and --config flags, collect extra args in an array
SUPERVISED=false
CONFIG_PATH=""
extra_args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --supervised)  SUPERVISED=true ;;
        --config)
            CONFIG_PATH="${2:-}"
            if [[ -z "$CONFIG_PATH" ]]; then
                echo "ERROR: --config requires a path argument."
                exit 1
            fi
            if [[ ! -f "$CONFIG_PATH" ]]; then
                echo "ERROR: Config file not found: ${CONFIG_PATH}"
                exit 1
            fi
            extra_args+=("--config" "$CONFIG_PATH")
            shift  # consume the path
            ;;
        *)  extra_args+=("$1") ;;
    esac
    shift
done

LOG_DIR="${PROJECT_DIR}/drone_logs"

# Ensure system libstdc++ is used instead of Anaconda's older version
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Clean stale legacy SHM segments from previous runs.
# Only relevant for the legacy POSIX SHM IPC backend (ipc_backend=shm).
# When using Zenoh (ipc_backend=zenoh), these /dev/shm/drone_* segments are not used,
# though Zenoh may still create its own POSIX SHM pool segments under /dev/shm.
rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
      /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health 2>/dev/null || true

mkdir -p "$LOG_DIR"
export DRONE_LOG_DIR="$LOG_DIR"

echo "══════════════════════════════════════════"
echo "  Drone Companion Stack — Launching"
echo "  Binaries : ${BIN_DIR}"
echo "  Logs     : ${LOG_DIR}"
echo "  Args     : ${extra_args[*]:-}"
[[ -n "$CONFIG_PATH" ]] && echo "  Config   : ${CONFIG_PATH}"
echo "══════════════════════════════════════════"

PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done
    # Wait for graceful shutdown
    sleep 2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

# ═══════════════════════════════════════════════════════════
# Supervised mode: P7 fork+execs P1–P6 internally
# ═══════════════════════════════════════════════════════════
if [[ "$SUPERVISED" == "true" ]]; then
    echo "  Mode     : SUPERVISED (P7 manages P1–P6)"
    echo "══════════════════════════════════════════"
    echo ""
    echo "Starting system_monitor in supervisor mode..."
    "${BIN_DIR}/system_monitor" --supervised "${extra_args[@]}" &
    PIDS+=($!)
    echo "system_monitor PID: ${PIDS[0]}"

    # Health check: wait for SHM health segment to appear (up to 10s)
    echo -n "Waiting for health segment..."
    HEALTH_READY=false
    for _ in $(seq 1 20); do
        if [[ -f /dev/shm/system_health ]]; then
            HEALTH_READY=true
            break
        fi
        echo -n "."
        sleep 0.5
    done
    if [[ "$HEALTH_READY" == "true" ]]; then
        echo -e " ready!"
    else
        echo -e " timeout (health segment not found — supervisor may still be starting)"
    fi

    echo "Press Ctrl+C to stop the stack."
    echo ""
    wait "${PIDS[0]}" 2>/dev/null || true
    echo "system_monitor exited."
    exit 0
fi

# ═══════════════════════════════════════════════════════════
# Legacy mode: script launches all 7 processes directly
# ═══════════════════════════════════════════════════════════
echo "  Mode     : LEGACY (script manages all 7 processes)"
echo "══════════════════════════════════════════"

# Launch order matches dependency graph:

echo "[1/7] Starting system_monitor..."
"${BIN_DIR}/system_monitor" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[2/7] Starting video_capture..."
"${BIN_DIR}/video_capture" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[3/7] Starting comms..."
"${BIN_DIR}/comms" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[4/7] Starting perception..."
"${BIN_DIR}/perception" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[5/7] Starting slam_vio_nav..."
"${BIN_DIR}/slam_vio_nav" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[6/7] Starting mission_planner..."
"${BIN_DIR}/mission_planner" "${extra_args[@]}" &
PIDS+=($!)
sleep 0.5

echo "[7/7] Starting payload_manager..."
"${BIN_DIR}/payload_manager" "${extra_args[@]}" &
PIDS+=($!)

echo ""
echo "All 7 processes launched."
echo "PIDs: ${PIDS[*]}"
echo "Press Ctrl+C to stop all processes."
echo ""

# Wait for any process to exit
wait -n "${PIDS[@]}" 2>/dev/null || true
echo "A process exited — shutting down stack."
