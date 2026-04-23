#!/usr/bin/env bash
# deploy/clean_run_build_cosys.sh
# Clean-build the companion stack and smoke-test that all 7 processes start.
#
# Scope: this script is a *build + process-startup* verifier, not a scenario
# runner.  It does NOT launch UE5 / Cosys-AirSim / Gazebo.  To actually run a
# scenario against Cosys-AirSim, use:
#
#     ./tests/run_scenario_cosys.sh config/scenarios/33_non_coco_obstacles.json --gui --verbose
#
# Usage:
#   bash deploy/clean_run_build_cosys.sh               # default: clean build + smoke test
#   bash deploy/clean_run_build_cosys.sh --no-tests    # skip ctest (saves ~60 s)
#   bash deploy/clean_run_build_cosys.sh --asan        # + AddressSanitizer
#   bash deploy/clean_run_build_cosys.sh --ubsan       # + UBSan
#   bash deploy/clean_run_build_cosys.sh --coverage    # + code coverage
#   bash deploy/clean_run_build_cosys.sh --smoke-seconds 15  # override 8 s wait
#   bash deploy/clean_run_build_cosys.sh --no-smoke    # skip smoke test (build + ctest only)
#
# NOTE: --tsan is intentionally omitted because the zenohc library triggers
#       TSan false-positives in its internal threading.
#
# What it does:
#   1. Kill leftover UE5/PX4/companion processes
#   2. Clean-build (Release by default, Debug if sanitizer/coverage)
#   3. Run unit tests (unless --no-tests)
#   4. Launch the 7 companion processes via deploy/launch_all.sh (simulated
#      backends — no UE5, no Gazebo), wait for them to come up, verify every
#      process is alive, then shut them down cleanly.
#
# Exit codes:
#   0  clean build + all 7 processes alive
#   1  build or tests or smoke test failed
#
# Prerequisites:
#   - zenohc ≥ 1.0 installed (apt: libzenohc libzenohc-dev)
#   - NVIDIA GPU + drivers optional (only needed for scenario runs, not here)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

SANITIZER=""
ENABLE_COVERAGE="OFF"
RUN_TESTS=true
RUN_SMOKE=true
SMOKE_SECONDS=8

usage() {
    sed -n '2,28p' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-tests)      RUN_TESTS=false ;;
        --no-smoke)      RUN_SMOKE=false ;;
        --smoke-seconds) SMOKE_SECONDS="$2"; shift ;;
        --asan)          SANITIZER="asan" ;;
        --tsan)
            echo "ERROR: --tsan is not supported (false-positives in zenohc internal threading)."
            echo "  Use --asan or --ubsan instead."
            exit 1
            ;;
        --ubsan)         SANITIZER="ubsan" ;;
        --coverage)      ENABLE_COVERAGE="ON" ;;
        -h|--help)       usage; exit 0 ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

# Build type: Debug if sanitizer or coverage requested, Release otherwise.
BUILD_TYPE="Release"
EXTRA_CMAKE_FLAGS=()
if [[ -n "$SANITIZER" ]]; then
    BUILD_TYPE="Debug"
    case "$SANITIZER" in
        asan)  EXTRA_CMAKE_FLAGS+=("-DENABLE_ASAN=ON") ;;
        ubsan) EXTRA_CMAKE_FLAGS+=("-DENABLE_UBSAN=ON") ;;
    esac
fi
if [[ "$ENABLE_COVERAGE" == "ON" ]]; then
    BUILD_TYPE="Debug"
    EXTRA_CMAKE_FLAGS+=("-DENABLE_COVERAGE=ON")
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
# UE5 / Cosys-AirSim (defensive — scripts like this often run after a failed scenario)
pkill -9 -f "UnrealEditor"   2>/dev/null || true
pkill -9 -f "Blocks-Linux"   2>/dev/null || true
pkill -9 -f "AirSimEnv"      2>/dev/null || true
# PX4 (defensive)
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
# Zenoh security configuration — array so paths with spaces survive the cmake invocation.
ZENOH_SECURITY_FLAGS=()
if [[ -n "${ZENOH_CONFIG_PATH:-}" ]]; then
    if [[ ! -f "$ZENOH_CONFIG_PATH" ]]; then
        echo "ERROR: ZENOH_CONFIG_PATH='${ZENOH_CONFIG_PATH}' does not exist."
        exit 1
    fi
    echo "  Using secure Zenoh config: ${ZENOH_CONFIG_PATH}"
    ZENOH_SECURITY_FLAGS+=("-DZENOH_CONFIG_PATH=${ZENOH_CONFIG_PATH}")
else
    echo "  WARNING: No ZENOH_CONFIG_PATH set — using insecure mode (dev/test only)."
    ZENOH_SECURITY_FLAGS+=("-DALLOW_INSECURE_ZENOH=ON")
fi

rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      "${ZENOH_SECURITY_FLAGS[@]}" \
      "${EXTRA_CMAKE_FLAGS[@]}" \
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

# ── Step 4: Smoke-test the 7 companion processes ─────────────
if [[ "$RUN_SMOKE" != "true" ]]; then
    echo ""
    echo "═══ [4/4] Skipping smoke test (--no-smoke) ═══"
    echo ""
    echo "DONE — clean build + ctest.  To run a scenario:"
    echo "  ./tests/run_scenario_cosys.sh config/scenarios/33_non_coco_obstacles.json --gui --verbose"
    exit 0
fi

echo ""
echo "═══ [4/4] Smoke test: start 7 processes (simulated backends) ═══"
SMOKE_LOG="${PROJECT_DIR}/drone_logs/smoke_test_$(date +%Y-%m-%d_%H%M%S).log"
mkdir -p "$(dirname "$SMOKE_LOG")"
echo "  Log   : ${SMOKE_LOG}"
echo "  Wait  : ${SMOKE_SECONDS}s for the stack to settle"
echo ""

# Launch the 7 processes.  launch_all.sh uses simulated backends by default —
# no UE5, no Gazebo, no external simulator — exactly what we want to verify
# that the stack compiled cleanly and every binary starts without crashing.
bash "${SCRIPT_DIR}/launch_all.sh" > "$SMOKE_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup_smoke() {
    # Stop launch_all and any lingering process it spawned.
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
    pkill -9 -f "build/bin/(video_capture|perception|slam_vio_nav|mission_planner|comms|payload_manager|system_monitor)" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    # Clean up stale Zenoh shm (safe — we just killed everything that would hold them).
    rm -f /dev/shm/zenoh_shm_* 2>/dev/null || true
}
trap cleanup_smoke EXIT INT TERM

# Poll for every 0.5s until every process is alive or we time out.
SMOKE_DEADLINE=$(( $(date +%s) + SMOKE_SECONDS ))
PROCS=(video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor)
while true; do
    ALL_UP=true
    for p in "${PROCS[@]}"; do
        if ! pgrep -f "build/bin/$p" >/dev/null; then
            ALL_UP=false; break
        fi
    done
    if [[ "$ALL_UP" == "true" ]]; then break; fi
    if [[ $(date +%s) -ge $SMOKE_DEADLINE ]]; then break; fi
    sleep 0.5
done

# Final check + per-process report.
echo "  ┌─────────────────────────────────────────────────"
MISSING=0
for p in "${PROCS[@]}"; do
    if pgrep -f "build/bin/$p" >/dev/null; then
        echo "  │  ✓ $p"
    else
        echo "  │  ✗ $p NOT RUNNING"
        MISSING=$((MISSING + 1))
    fi
done
echo "  └─────────────────────────────────────────────────"
echo ""

if [[ $MISSING -eq 0 ]]; then
    echo "SMOKE TEST: PASS — all 7 processes are alive."
    echo ""
    echo "Next: run a scenario"
    echo "  ./tests/run_scenario_cosys.sh config/scenarios/33_non_coco_obstacles.json --gui --verbose"
    exit 0
else
    echo "SMOKE TEST: FAIL — ${MISSING}/7 processes did not start."
    echo ""
    echo "Last 30 lines of ${SMOKE_LOG}:"
    tail -30 "$SMOKE_LOG" 2>/dev/null || true
    exit 1
fi
