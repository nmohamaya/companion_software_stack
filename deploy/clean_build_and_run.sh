#!/usr/bin/env bash
# deploy/clean_build_and_run.sh
# Clean-build the companion stack and launch the Gazebo SITL simulation.
#
# Usage:
#   bash deploy/clean_build_and_run.sh               # headless
#   bash deploy/clean_build_and_run.sh --gui          # with Gazebo 3-D GUI
#   bash deploy/clean_build_and_run.sh --asan         # + AddressSanitizer
#   bash deploy/clean_build_and_run.sh --ubsan        # + UBSan
#   bash deploy/clean_build_and_run.sh --coverage     # + code coverage
#
# NOTE: --tsan is intentionally omitted because the zenohc library triggers
#       TSan false-positives in its internal threading.
#
# What it does:
#   1. Kill leftover PX4/Gazebo/companion processes
#   2. Clean-build (Release by default, Debug if sanitizer/coverage)
#   3. Run unit tests
#   4. Launch Gazebo SITL flight with config/gazebo.json
#
# Prerequisites:
#   - zenohc ≥ 1.0 installed  (apt: libzenohc libzenohc-dev)
#   - zenoh-cpp headers        (https://github.com/eclipse-zenoh/zenoh-cpp)
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
        --tsan)
            echo "ERROR: --tsan is not supported (false-positives in zenohc internal threading)."
            echo "  Use --asan or --ubsan instead."
            exit 1
            ;;
        --ubsan)     SANITIZER="ubsan" ;;
        --coverage)  ENABLE_COVERAGE="ON" ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--gui] [--asan|--ubsan] [--coverage]"
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
        ubsan) EXTRA_CMAKE_FLAGS="-DENABLE_UBSAN=ON" ;;
    esac
fi
if [[ "$ENABLE_COVERAGE" == "ON" ]]; then
    BUILD_TYPE="Debug"
    EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS} -DENABLE_COVERAGE=ON"
fi

# ── Step 0: Verify zenohc is installed ────────────────────────
if ! pkg-config --exists zenohc 2>/dev/null; then
    echo "ERROR: zenohc not found. Install it first:"
    echo "  sudo apt install libzenohc libzenohc-dev"
    echo "  or see: https://github.com/eclipse-zenoh/zenoh-c/releases"
    exit 1
fi
ZENOH_VER="$(pkg-config --modversion zenohc 2>/dev/null || echo 'unknown')"
echo "[OK] zenohc ${ZENOH_VER} found"

# ── Step 1: Kill leftover processes ───────────────────────────
echo ""
echo "═══ [1/4] Cleaning up old processes ═══"
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "build/bin/$p" 2>/dev/null || true
done
sleep 2
echo "  Done."

# ── Step 2: Clean build ──────────────────────────────────────
echo ""
echo "═══ [2/4] Clean build (${BUILD_TYPE}) ═══"
[[ -n "$SANITIZER" ]]            && echo "  Sanitizer: ${SANITIZER}"
[[ "$ENABLE_COVERAGE" == "ON" ]] && echo "  Coverage : ON"
# Determine Zenoh security configuration:
# - If ZENOH_CONFIG_PATH is set, use it (secure build).
# - Otherwise, fall back to ALLOW_INSECURE_ZENOH (dev/test only).
ZENOH_SECURITY_FLAGS=""
if [[ -n "${ZENOH_CONFIG_PATH:-}" ]]; then
    if [[ ! -f "$ZENOH_CONFIG_PATH" ]]; then
        echo "ERROR: ZENOH_CONFIG_PATH='${ZENOH_CONFIG_PATH}' does not exist."
        exit 1
    fi
    echo "  Using secure Zenoh config: ${ZENOH_CONFIG_PATH}"
    ZENOH_SECURITY_FLAGS="-DZENOH_CONFIG_PATH=${ZENOH_CONFIG_PATH}"
else
    echo "  WARNING: No ZENOH_CONFIG_PATH set — using insecure mode (dev/test only)."
    ZENOH_SECURITY_FLAGS="-DALLOW_INSECURE_ZENOH=ON"
fi

rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      ${ZENOH_SECURITY_FLAGS} \
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

# ── Step 4: Launch Gazebo SITL ────────────────────────────────
echo ""
echo "═══ [4/4] Launching Gazebo SITL simulation ═══"
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
