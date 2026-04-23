#!/usr/bin/env bash
# deploy/clean_run_build_cosys.sh
# Clean-build the companion stack and launch a Cosys-AirSim (UE5) scenario.
#
# Usage:
#   bash deploy/clean_run_build_cosys.sh                        # default: scenario 33 + GUI
#   bash deploy/clean_run_build_cosys.sh --scenario 30          # pick a different scenario
#   bash deploy/clean_run_build_cosys.sh --headless             # no UE5 3D window
#   bash deploy/clean_run_build_cosys.sh --no-tests             # skip unit tests
#   bash deploy/clean_run_build_cosys.sh --asan                 # + AddressSanitizer
#   bash deploy/clean_run_build_cosys.sh --ubsan                # + UBSan
#   bash deploy/clean_run_build_cosys.sh --coverage             # + code coverage
#   bash deploy/clean_run_build_cosys.sh --scenario-config PATH # absolute scenario json
#
# NOTE: --tsan is intentionally omitted because the zenohc library triggers
#       TSan false-positives in its internal threading.
#
# What it does:
#   1. Kill leftover UE5/PX4/companion processes
#   2. Clean-build (Release by default, Debug if sanitizer/coverage)
#   3. Run unit tests (unless --no-tests)
#   4. Launch tests/run_scenario_cosys.sh with the selected scenario
#
# Prerequisites:
#   - zenohc ≥ 1.0 installed  (apt: libzenohc libzenohc-dev)
#   - Cosys-AirSim checkout with a pre-built environment (e.g., Blocks)
#   - NVIDIA GPU + drivers for UE5
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

SCENARIO_NUM="33"
SCENARIO_CONFIG=""
GUI_FLAG="--gui"
SANITIZER=""
ENABLE_COVERAGE="OFF"
RUN_TESTS=true
VERBOSE_FLAG="--verbose"

usage() {
    sed -n '2,20p' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario)          SCENARIO_NUM="$2"; shift ;;
        --scenario-config)   SCENARIO_CONFIG="$2"; shift ;;
        --headless)          GUI_FLAG="" ;;
        --gui)               GUI_FLAG="--gui" ;;
        --quiet)             VERBOSE_FLAG="" ;;
        --no-tests)          RUN_TESTS=false ;;
        --asan)              SANITIZER="asan" ;;
        --tsan)
            echo "ERROR: --tsan is not supported (false-positives in zenohc internal threading)."
            echo "  Use --asan or --ubsan instead."
            exit 1
            ;;
        --ubsan)             SANITIZER="ubsan" ;;
        --coverage)          ENABLE_COVERAGE="ON" ;;
        -h|--help)           usage; exit 0 ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

# Resolve scenario config path
if [[ -z "$SCENARIO_CONFIG" ]]; then
    # Match scenario number prefix against config/scenarios/<N>_*.json
    matches=( "${PROJECT_DIR}/config/scenarios/${SCENARIO_NUM}_"*.json )
    if [[ ${#matches[@]} -ne 1 || ! -f "${matches[0]}" ]]; then
        echo "ERROR: Could not resolve scenario '${SCENARIO_NUM}' in config/scenarios/"
        echo "  Try --scenario-config <path> or list with:"
        echo "    ls config/scenarios/ | head"
        exit 1
    fi
    SCENARIO_CONFIG="${matches[0]}"
fi
if [[ ! -f "$SCENARIO_CONFIG" ]]; then
    echo "ERROR: Scenario config not found: ${SCENARIO_CONFIG}"
    exit 1
fi

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
# UE5 / Cosys-AirSim
pkill -9 -f "UnrealEditor"   2>/dev/null || true
pkill -9 -f "Blocks-Linux"   2>/dev/null || true
pkill -9 -f "AirSimEnv"      2>/dev/null || true
# PX4 (defensive: Tier 3 uses SimpleFlight, but stale PX4 could still hold ports)
pkill -9 -f "px4_sitl"                   2>/dev/null || true
pkill -9 -f "PX4-Autopilot/build/px4"    2>/dev/null || true
# Companion stack
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor \
         cosys_telemetry_poller cosys_populate_scene; do
    pkill -f "build/bin/$p" 2>/dev/null || true
done
sleep 2
echo "  Done."

# ── Step 2: Clean build ──────────────────────────────────────
echo ""
echo "═══ [2/4] Clean build (${BUILD_TYPE}) ═══"
[[ -n "$SANITIZER" ]]            && echo "  Sanitizer: ${SANITIZER}"
[[ "$ENABLE_COVERAGE" == "ON" ]] && echo "  Coverage : ON"
# Determine Zenoh security configuration
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
if [[ "$RUN_TESTS" == "true" ]]; then
    echo ""
    echo "═══ [3/4] Running unit tests ═══"
    ctest --test-dir build --output-on-failure -j"$(nproc)"
    echo "  All tests passed."
else
    echo ""
    echo "═══ [3/4] Skipping unit tests (--no-tests) ═══"
fi

# ── Step 4: Launch Cosys-AirSim scenario ─────────────────────
echo ""
echo "═══ [4/4] Launching Cosys-AirSim scenario ═══"
if [[ -n "$GUI_FLAG" ]]; then
    echo "  Mode     : GUI (UE5 3-D viewport)"
else
    echo "  Mode     : Headless (-RenderOffScreen)"
fi
echo "  Scenario : ${SCENARIO_CONFIG}"
echo "  Logs     : drone_logs/scenarios_cosys/<scenario>/..."
echo "  Press Ctrl+C to stop."
echo ""

# Use exec so SIGINT from the user goes straight to the runner's trap
exec bash "${PROJECT_DIR}/tests/run_scenario_cosys.sh" \
    "$SCENARIO_CONFIG" \
    $GUI_FLAG \
    $VERBOSE_FLAG
