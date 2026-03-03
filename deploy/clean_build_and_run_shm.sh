#!/usr/bin/env bash
# deploy/clean_build_and_run_shm.sh
# Clean-build the companion stack with POSIX SHM backend and launch the
# Gazebo SITL simulation.
#
# Usage:
#   bash deploy/clean_build_and_run_shm.sh               # headless
#   bash deploy/clean_build_and_run_shm.sh --gui          # with Gazebo 3-D GUI
#   bash deploy/clean_build_and_run_shm.sh --asan         # + AddressSanitizer
#   bash deploy/clean_build_and_run_shm.sh --tsan         # + ThreadSanitizer
#   bash deploy/clean_build_and_run_shm.sh --ubsan        # + UBSan
#   bash deploy/clean_build_and_run_shm.sh --coverage     # + code coverage
#
# What it does:
#   1. Kill leftover PX4/Gazebo/companion processes & stale SHM segments
#   2. Clean-build (Release by default, Debug if sanitizer/coverage, SHM IPC)
#   3. Run unit tests
#   4. Launch Gazebo SITL flight with config/gazebo.json (SHM IPC)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

GUI_FLAG=""
SANITIZER=""
ENABLE_COVERAGE="OFF"

for arg in "$@"; do
    case "$arg" in
        --gui)       GUI_FLAG="--gui" ;;
        --asan)      SANITIZER="asan" ;;
        --tsan)      SANITIZER="tsan" ;;
        --ubsan)     SANITIZER="ubsan" ;;
        --coverage)  ENABLE_COVERAGE="ON" ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--gui] [--asan|--tsan|--ubsan] [--coverage]"
            exit 1
            ;;
    esac
done

# Build type: Debug if sanitizer or coverage requested, Release otherwise
BUILD_TYPE="Release"
EXTRA_CMAKE_FLAGS=""
if [[ -n "$SANITIZER" ]]; then
    BUILD_TYPE="Debug"
    case "$SANITIZER" in
        asan)  EXTRA_CMAKE_FLAGS="-DENABLE_ASAN=ON" ;;
        tsan)  EXTRA_CMAKE_FLAGS="-DENABLE_TSAN=ON" ;;
        ubsan) EXTRA_CMAKE_FLAGS="-DENABLE_UBSAN=ON" ;;
    esac
fi
if [[ "$ENABLE_COVERAGE" == "ON" ]]; then
    BUILD_TYPE="Debug"
    EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS} -DENABLE_COVERAGE=ON"
fi

# ── Step 1: Kill leftover processes & SHM ────────────────────
echo "═══ [1/4] Cleaning up old processes & shared memory ═══"
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "build/bin/$p" 2>/dev/null || true
done
sleep 2
# Remove only the explicit SHM segments this stack creates (see shm_types.h::shm_names).
# Avoid broad globs that could delete unrelated /dev/shm objects.
SHM_SEGMENTS=(
    /dev/shm/drone_mission_cam
    /dev/shm/drone_stereo_cam
    /dev/shm/detected_objects
    /dev/shm/slam_pose
    /dev/shm/mission_status
    /dev/shm/trajectory_cmd
    /dev/shm/payload_commands
    /dev/shm/fc_commands
    /dev/shm/fc_state
    /dev/shm/gcs_commands
    /dev/shm/payload_status
    /dev/shm/system_health
)
rm -f "${SHM_SEGMENTS[@]}" 2>/dev/null || true
echo "  Done."

# ── Step 2: Clean build (SHM — Zenoh OFF) ────────────────────
echo ""
echo "═══ [2/4] Clean build (${BUILD_TYPE}, IPC = POSIX SHM) ═══"
[[ -n "$SANITIZER" ]]            && echo "  Sanitizer: ${SANITIZER}"
[[ "$ENABLE_COVERAGE" == "ON" ]] && echo "  Coverage : ON"
rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      -DENABLE_ZENOH=OFF \
      ${EXTRA_CMAKE_FLAGS} \
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
