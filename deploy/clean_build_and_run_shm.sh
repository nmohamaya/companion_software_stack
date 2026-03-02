#!/usr/bin/env bash
# deploy/clean_build_and_run_shm.sh
# Clean-build the companion stack with POSIX SHM backend and launch the
# Gazebo SITL simulation.
#
# Usage:
#   bash deploy/clean_build_and_run_shm.sh          # headless
#   bash deploy/clean_build_and_run_shm.sh --gui    # with Gazebo 3-D GUI
#
# What it does:
#   1. Kill leftover PX4/Gazebo/companion processes & stale SHM segments
#   2. Clean-build (Release, Zenoh OFF)
#   3. Run unit tests
#   4. Launch Gazebo SITL flight with config/gazebo.json (SHM IPC)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

GUI_FLAG=""
for arg in "$@"; do
    [[ "$arg" == "--gui" ]] && GUI_FLAG="--gui"
done

# ── Step 1: Kill leftover processes & SHM ────────────────────
echo "═══ [1/4] Cleaning up old processes & shared memory ═══"
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "build/bin/$p" 2>/dev/null || true
done
sleep 2
rm -f /dev/shm/drone_* /dev/shm/detected_* /dev/shm/slam_* \
      /dev/shm/mission_* /dev/shm/trajectory_* /dev/shm/payload_* \
      /dev/shm/fc_* /dev/shm/gcs_* /dev/shm/system_* 2>/dev/null || true
echo "  Done."

# ── Step 2: Clean build (SHM — Zenoh OFF) ────────────────────
echo ""
echo "═══ [2/4] Clean build (Release, IPC = POSIX SHM) ═══"
rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_ZENOH=OFF \
      ..
cmake --build . -j"$(nproc)"
cd "$PROJECT_DIR"
echo "  Build complete — binaries in build/bin/"

# ── Step 3: Run tests ────────────────────────────────────────
echo ""
echo "═══ [3/4] Running unit tests ═══"
ctest --test-dir build --output-on-failure -j"$(nproc)"
echo "  All tests passed."

# ── Step 4: Launch Gazebo SITL (SHM) ─────────────────────────
echo ""
echo "═══ [4/4] Launching Gazebo SITL simulation (SHM backend) ═══"
if [[ -n "$GUI_FLAG" ]]; then
    echo "  Mode   : GUI (3-D visualisation + chase-cam)"
else
    echo "  Mode   : Headless"
fi
echo "  Config : config/gazebo.json"
echo "  Logs   : drone_logs/"
echo "  Press Ctrl+C to stop."
echo ""

export CONFIG_FILE="${PROJECT_DIR}/config/gazebo.json"
exec bash deploy/launch_gazebo.sh $GUI_FLAG
