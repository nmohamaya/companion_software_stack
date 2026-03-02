#!/usr/bin/env bash
# deploy/clean_build_and_run_zenoh.sh
# Clean-build the companion stack with Zenoh IPC backend and launch the
# Gazebo SITL simulation.
#
# Usage:
#   bash deploy/clean_build_and_run_zenoh.sh          # headless
#   bash deploy/clean_build_and_run_zenoh.sh --gui    # with Gazebo 3-D GUI
#
# What it does:
#   1. Kill leftover PX4/Gazebo/companion processes & stale SHM segments
#   2. Clean-build (Release, Zenoh ON)
#   3. Run unit tests
#   4. Launch Gazebo SITL flight with config/gazebo_zenoh.json (Zenoh IPC)
#
# Prerequisites:
#   - zenohc ≥ 1.0 installed  (apt: libzenohc libzenohc-dev)
#   - zenoh-cpp headers        (https://github.com/eclipse-zenoh/zenoh-cpp)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

GUI_FLAG=""
for arg in "$@"; do
    [[ "$arg" == "--gui" ]] && GUI_FLAG="--gui"
done

# ── Step 0: Verify zenohc is installed ────────────────────────
if ! pkg-config --exists zenohc 2>/dev/null; then
    echo "ERROR: zenohc not found. Install it first:"
    echo "  sudo apt install libzenohc libzenohc-dev"
    echo "  or see: https://github.com/eclipse-zenoh/zenoh-c/releases"
    exit 1
fi
ZENOH_VER="$(pkg-config --modversion zenohc 2>/dev/null || echo 'unknown')"
echo "[OK] zenohc ${ZENOH_VER} found"

# ── Step 1: Kill leftover processes & SHM ────────────────────
echo ""
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

# ── Step 2: Clean build (Zenoh ON) ───────────────────────────
echo ""
echo "═══ [2/4] Clean build (Release, IPC = Zenoh) ═══"
rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_ZENOH=ON \
      -DALLOW_INSECURE_ZENOH=ON \
      ..
cmake --build . -j"$(nproc)"
cd "$PROJECT_DIR"
echo "  Build complete — binaries in build/bin/"

# ── Step 3: Run tests ────────────────────────────────────────
echo ""
echo "═══ [3/4] Running unit tests ═══"
ctest --test-dir build --output-on-failure -j"$(nproc)"
echo "  All tests passed."

# ── Step 4: Launch Gazebo SITL (Zenoh) ────────────────────────
echo ""
echo "═══ [4/4] Launching Gazebo SITL simulation (Zenoh backend) ═══"
if [[ -n "$GUI_FLAG" ]]; then
    echo "  Mode   : GUI (3-D visualisation + chase-cam)"
else
    echo "  Mode   : Headless"
fi
echo "  Config : config/gazebo_zenoh.json"
echo "  Logs   : drone_logs/"
echo "  Press Ctrl+C to stop."
echo ""

export CONFIG_FILE="${PROJECT_DIR}/config/gazebo_zenoh.json"
exec bash deploy/launch_gazebo.sh $GUI_FLAG
