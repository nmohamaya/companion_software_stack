#!/usr/bin/env bash
# deploy/launch_all.sh — Launch all 7 processes in correct order.
# Usage: ./deploy/launch_all.sh [--sim] [--log-level debug]
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"

if [[ ! -d "$BIN_DIR" ]]; then
    echo "ERROR: Build directory not found. Run deploy/build.sh first."
    exit 1
fi

EXTRA_ARGS="${*}"
LOG_DIR="${PROJECT_DIR}/drone_logs"

# Ensure system libstdc++ is used instead of Anaconda's older version
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Clean stale SHM segments from previous runs.
# Only relevant when using the POSIX SHM IPC backend (ipc_backend=shm).
# When using Zenoh (ipc_backend=zenoh), no /dev/shm segments are created.
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
echo "  Args     : ${EXTRA_ARGS}"
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

# Launch order matches dependency graph:
# 1. System Monitor (no deps)
# 2. Video Capture (no deps)
# 3. Comms (no deps)
# 4. Perception (needs Video Capture)
# 5. SLAM/VIO/Nav (needs Video Capture)
# 6. Mission Planner (needs SLAM, Perception, Comms)
# 7. Payload Manager (needs Mission Planner)

echo "[1/7] Starting system_monitor..."
"${BIN_DIR}/system_monitor" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[2/7] Starting video_capture..."
"${BIN_DIR}/video_capture" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[3/7] Starting comms..."
"${BIN_DIR}/comms" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[4/7] Starting perception..."
"${BIN_DIR}/perception" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[5/7] Starting slam_vio_nav..."
"${BIN_DIR}/slam_vio_nav" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[6/7] Starting mission_planner..."
"${BIN_DIR}/mission_planner" ${EXTRA_ARGS} &
PIDS+=($!)
sleep 0.5

echo "[7/7] Starting payload_manager..."
"${BIN_DIR}/payload_manager" ${EXTRA_ARGS} &
PIDS+=($!)

echo ""
echo "All 7 processes launched."
echo "PIDs: ${PIDS[*]}"
echo "Press Ctrl+C to stop all processes."
echo ""

# Wait for any process to exit
wait -n "${PIDS[@]}" 2>/dev/null || true
echo "A process exited — shutting down stack."
