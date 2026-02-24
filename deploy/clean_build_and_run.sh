#!/usr/bin/env bash
# deploy/clean_build_and_run.sh
# Clean build the companion stack from scratch and launch the Gazebo SITL sim.
#
# Usage:
#   bash deploy/clean_build_and_run.sh          # headless
#   bash deploy/clean_build_and_run.sh --gui    # with Gazebo 3D GUI
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

GUI_FLAG=""
for arg in "$@"; do
    [[ "$arg" == "--gui" ]] && GUI_FLAG="--gui"
done

# ── Step 1: Kill any leftover processes ──────────────────────
echo "═══ [1/4] Cleaning up old processes & shared memory ═══"
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "$p" 2>/dev/null || true
done
sleep 2
rm -f /dev/shm/drone_* /dev/shm/detected_* /dev/shm/slam_* \
      /dev/shm/mission_* /dev/shm/trajectory_* /dev/shm/payload_* \
      /dev/shm/fc_* /dev/shm/gcs_* /dev/shm/system_* 2>/dev/null
echo "  Done."

# ── Step 2: Clean build ─────────────────────────────────────
echo ""
echo "═══ [2/4] Clean build (Release) ═══"
rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
cd "$PROJECT_DIR"
echo "  Build complete — binaries in build/bin/"

# ── Step 3: Run tests ───────────────────────────────────────
echo ""
echo "═══ [3/4] Running tests ═══"
ctest --test-dir build --output-on-failure -j"$(nproc)"
echo "  All tests passed."

# ── Step 4: Launch simulation ───────────────────────────────
echo ""
echo "═══ [4/4] Launching Gazebo SITL simulation ═══"
if [[ -n "$GUI_FLAG" ]]; then
    echo "  Mode: GUI (3D visualisation + chase-cam)"
else
    echo "  Mode: Headless"
fi
echo "  Logs: /tmp/drone_logs/"
echo "  Press Ctrl+C to stop."
echo ""

exec bash deploy/launch_gazebo.sh $GUI_FLAG
