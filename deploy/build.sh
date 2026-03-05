#!/usr/bin/env bash
# deploy/build.sh — Build the drone companion stack.
#
# Usage:
#   bash deploy/build.sh                 # Release, SHM backend
#   bash deploy/build.sh --zenoh         # Release, Zenoh backend
#   bash deploy/build.sh Debug           # Debug, SHM backend
#   bash deploy/build.sh Debug --zenoh   # Debug, Zenoh backend
#   bash deploy/build.sh --clean         # Delete build/ first
#   bash deploy/build.sh --asan          # Debug + AddressSanitizer
#   bash deploy/build.sh --tsan          # Debug + ThreadSanitizer
#   bash deploy/build.sh --ubsan         # Debug + UndefinedBehaviorSanitizer
#   bash deploy/build.sh --coverage      # Debug + gcov code coverage
#   bash deploy/build.sh --format-check  # Check clang-format (no build)
#   bash deploy/build.sh --test          # Build + run all tests
#   bash deploy/build.sh --test-filter watchdog  # Build + run module tests
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

BUILD_DIR="${PROJECT_DIR}/build"
BUILD_TYPE="Release"
ENABLE_ZENOH="OFF"
CLEAN=0
SANITIZER=""
ENABLE_COVERAGE="OFF"
FORMAT_CHECK=0
RUN_TESTS=0
TEST_FILTER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --zenoh)         ENABLE_ZENOH="ON" ;;
        --clean)         CLEAN=1 ;;
        --asan)          SANITIZER="asan" ;;
        --tsan)          SANITIZER="tsan" ;;
        --ubsan)         SANITIZER="ubsan" ;;
        --coverage)      ENABLE_COVERAGE="ON" ;;
        --format-check)  FORMAT_CHECK=1 ;;
        --test)          RUN_TESTS=1 ;;
        --test-filter)
            RUN_TESTS=1
            TEST_FILTER="${2:-}"
            if [[ -z "$TEST_FILTER" ]]; then
                echo "ERROR: --test-filter requires a module name."
                echo "  Example: $0 --test-filter watchdog"
                exit 1
            fi
            shift  # consume the module name
            ;;
        --test-filter=*)
            RUN_TESTS=1
            TEST_FILTER="${1#*=}"
            if [[ -z "$TEST_FILTER" ]]; then
                echo "ERROR: --test-filter requires a module name."
                echo "  Example: $0 --test-filter watchdog"
                exit 1
            fi
            ;;
        Debug|Release|RelWithDebInfo|MinSizeRel) BUILD_TYPE="$1" ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [Debug|Release] [--zenoh] [--clean] [--asan|--tsan|--ubsan] [--coverage] [--format-check] [--test] [--test-filter MODULE]"
            exit 1
            ;;
    esac
    shift
done

# ── Format check (standalone — exits after) ──────────────────
if [[ "$FORMAT_CHECK" -eq 1 ]]; then
    echo "═══ Checking code formatting (clang-format-18) ═══"
    cd "$PROJECT_DIR"
    if ! command -v clang-format-18 &>/dev/null; then
        # Fall back to plain clang-format if version-suffixed not found
        if command -v clang-format &>/dev/null; then
            echo "  WARNING: clang-format-18 not found, using $(clang-format --version)"
            FORMATTER="clang-format"
        else
            echo "ERROR: clang-format-18 not found. Install: sudo apt install clang-format-18"
            exit 1
        fi
    else
        FORMATTER="clang-format-18"
    fi
    ERRORS=0
    find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 \
      | xargs -0 "$FORMATTER" --dry-run --Werror 2>&1 || ERRORS=$?
    if [[ "$ERRORS" -ne 0 ]]; then
        echo ""
        echo "FAILED: Some files need formatting. Auto-fix with:"
        echo "  find common process[1-7]_* tests \\( -name '*.h' -o -name '*.cpp' \\) -print0 | xargs -0 $FORMATTER -i"
        exit 1
    fi
    echo "  All files formatted correctly."
    exit 0
fi

# ── Sanitizer flags ──────────────────────────────────────────
SANITIZER_FLAGS=""
if [[ -n "$SANITIZER" ]]; then
    BUILD_TYPE="Debug"  # Sanitizers need debug info
    case "$SANITIZER" in
        asan)  SANITIZER_FLAGS="-DENABLE_ASAN=ON" ;;
        tsan)  SANITIZER_FLAGS="-DENABLE_TSAN=ON" ;;
        ubsan) SANITIZER_FLAGS="-DENABLE_UBSAN=ON" ;;
    esac
fi

# ── Coverage flags ───────────────────────────────────────────
COVERAGE_FLAGS=""
if [[ "$ENABLE_COVERAGE" == "ON" ]]; then
    BUILD_TYPE="Debug"  # Coverage needs debug info for accurate line mapping
    COVERAGE_FLAGS="-DENABLE_COVERAGE=ON"
fi

# Zenoh backend configuration
ZENOH_FLAGS=""
if [[ "$ENABLE_ZENOH" == "ON" ]]; then
    if ! pkg-config --exists zenohc 2>/dev/null; then
        echo "ERROR: --zenoh requested but zenohc not found."
        echo "  Install: sudo apt install libzenohc libzenohc-dev"
        exit 1
    fi
    # Prefer secure config if ZENOH_CONFIG_PATH is provided; fall back to
    # insecure for dev/test.
    if [[ -n "${ZENOH_CONFIG_PATH:-}" ]]; then
        if [[ ! -f "$ZENOH_CONFIG_PATH" ]]; then
            echo "ERROR: ZENOH_CONFIG_PATH='${ZENOH_CONFIG_PATH}' does not exist."
            exit 1
        fi
        ZENOH_FLAGS="-DENABLE_ZENOH=ON -DZENOH_CONFIG_PATH=${ZENOH_CONFIG_PATH}"
    else
        echo "NOTE: No ZENOH_CONFIG_PATH set — using insecure mode (dev/test only)."
        ZENOH_FLAGS="-DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON"
    fi
fi

echo "══════════════════════════════════════════"
echo "  Building Drone Companion Stack"
echo "  Build type : ${BUILD_TYPE}"
echo "  IPC backend: $([ "$ENABLE_ZENOH" = "ON" ] && echo "Zenoh" || echo "SHM")"
[[ -n "$SANITIZER" ]]          && echo "  Sanitizer  : ${SANITIZER}"
[[ "$ENABLE_COVERAGE" == "ON" ]] && echo "  Coverage   : ON"
echo "══════════════════════════════════════════"

if [[ "$CLEAN" -eq 1 ]]; then
    echo "Removing build/..."
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# shellcheck disable=SC2086
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
      ${ZENOH_FLAGS} \
      ${SANITIZER_FLAGS} \
      ${COVERAGE_FLAGS} \
      ..
cmake --build . -j"$(nproc)"

echo ""
echo "Build complete! Binaries in: ${BUILD_DIR}/bin/"
ls -la "${BUILD_DIR}/bin/"

# ── Post-build: run tests if requested ───────────────────────
if [[ "$RUN_TESTS" -eq 1 ]]; then
    echo ""
    echo "═══ Running tests ═══"
    TEST_RUNNER="${PROJECT_DIR}/tests/run_tests.sh"
    TEST_ARGS=()
    if [[ -n "$TEST_FILTER" ]]; then
        TEST_ARGS+=("$TEST_FILTER")
    fi
    # Pass --no-build to avoid a redundant rebuild — build.sh already built.
    TEST_ARGS+=("--no-build")
    if [[ "$ENABLE_COVERAGE" == "ON" ]]; then
        TEST_ARGS+=("--coverage")
    fi
    bash "$TEST_RUNNER" "${TEST_ARGS[@]}"
fi

# ── Post-build: coverage report helper ───────────────────────
if [[ "$ENABLE_COVERAGE" == "ON" && "$RUN_TESTS" -eq 0 ]]; then
    echo ""
    echo "Coverage build ready.  To generate a report after running tests:"
    echo "  cd ${BUILD_DIR}"
    echo "  ctest --output-on-failure"
    echo "  lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch"
    echo "  lcov --remove coverage.info '/usr/*' '*/tests/*' --output-file coverage_filtered.info"
    echo "  genhtml coverage_filtered.info --output-directory coverage_html"
    echo "  echo \"Open: \${PWD}/coverage_html/index.html\""
fi
