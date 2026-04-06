#!/usr/bin/env bash
# deploy/run_ci_local.sh — Run the same checks as GitHub Actions CI, locally.
#
# Usage:
#   bash deploy/run_ci_local.sh            # Run all CI jobs (format + build + sanitizers + coverage)
#   bash deploy/run_ci_local.sh --quick    # Format + build (no sanitizers, no coverage)
#   bash deploy/run_ci_local.sh --job FMT  # Run a single job by tag
#
# Jobs (tags):
#   FMT       clang-format-18 check
#   BUILD     Debug build + test (Zenoh)
#   ASAN      AddressSanitizer
#   TSAN      ThreadSanitizer
#   UBSAN     UBSanitizer
#   COV       Coverage build + lcov report
#
# The script mirrors the CI matrix in .github/workflows/ci.yml.
# If you want to save the results, pipe to a file: bash deploy/run_ci_local.sh 2>&1 | tee ci_results.log
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_DIR}/build"

# ── Colors ───────────────────────────────────────────────────
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    BOLD='\033[1m'
    RESET='\033[0m'
else
    RED='' GREEN='' BOLD='' RESET=''
fi

# ── Parse arguments ──────────────────────────────────────────
MODE="all"          # all | quick | single
SINGLE_JOB=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --quick|-q)   MODE="quick" ;;
        --job|-j)     MODE="single"; SINGLE_JOB="$2"; shift ;;
        --help|-h)
            head -17 "$0" | tail -15
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${RESET}"
            echo "Run: $0 --help"
            exit 1
            ;;
    esac
    shift
done

# ── Job definitions ──────────────────────────────────────────
PASS=0
FAIL=0
TOTAL=0
FAILED_JOBS=()

run_job() {
    local tag="$1"
    local label="$2"
    shift 2

    TOTAL=$((TOTAL + 1))
    echo ""
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${BOLD}  [${tag}] ${label}${RESET}"
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo ""

    local start
    start=$(date +%s)

    if "$@"; then
        local elapsed=$(( $(date +%s) - start ))
        echo -e "${GREEN}${BOLD}  ✓ [${tag}] PASSED${RESET} (${elapsed}s)"
        PASS=$((PASS + 1))
    else
        local elapsed=$(( $(date +%s) - start ))
        echo -e "${RED}${BOLD}  ✗ [${tag}] FAILED${RESET} (${elapsed}s)"
        FAIL=$((FAIL + 1))
        FAILED_JOBS+=("$tag")
    fi
}

should_run() {
    local tag="$1"
    if [[ "$MODE" == "single" ]]; then
        [[ "$SINGLE_JOB" == "$tag" ]]
    elif [[ "$MODE" == "quick" ]]; then
        [[ "$tag" == "FMT" || "$tag" == "BUILD" ]]
    else
        return 0  # all
    fi
}

# ── Helper: build + test ────────────────────────────────────
# shellcheck disable=SC2317  # called indirectly via run_job
build_and_test() {
    local build_type="$1"
    shift
    # Remaining args are extra cmake flags
    local cmake_flags=("$@")

    # Determine ctest exclusions (mirrors CI)
    local ctest_exclude=""
    for flag in "${cmake_flags[@]}"; do
        if [[ "$flag" == "-DENABLE_TSAN=ON" ]]; then
            ctest_exclude="Zenoh|Mavlink|Yolo|Liveliness|MessageBus"
        fi
    done

    echo "  cmake -B build -DCMAKE_BUILD_TYPE=${build_type} -DALLOW_INSECURE_ZENOH=ON ${cmake_flags[*]}"
    # shellcheck disable=SC2086
    cmake -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE="${build_type}" \
        -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
        -DALLOW_INSECURE_ZENOH=ON \
        "${cmake_flags[@]}" 2>&1

    echo "  cmake --build build -j$(nproc)"
    cmake --build "${BUILD_DIR}" -j"$(nproc)" 2>&1

    echo ""
    if [[ -n "$ctest_exclude" ]]; then
        echo "  ctest (excluding: ${ctest_exclude})"
        ctest --test-dir "${BUILD_DIR}" --output-on-failure -j"$(nproc)" -E "${ctest_exclude}"
    else
        echo "  ctest --output-on-failure"
        ctest --test-dir "${BUILD_DIR}" --output-on-failure -j"$(nproc)"
    fi
}

# ── Job: FMT ─────────────────────────────────────────────────
# shellcheck disable=SC2317  # called indirectly via run_job
job_fmt() {
    cd "$PROJECT_DIR"
    find common process[1-7]_* tests tools \( -name '*.h' -o -name '*.cpp' \) -print0 \
        | xargs -0 clang-format-18 --dry-run --Werror 2>&1
    echo "  All files formatted correctly."
}

# ── Job: Build + test ────────────────────────────────────────
# shellcheck disable=SC2317  # called indirectly via run_job
job_build()    { cd "$PROJECT_DIR" && build_and_test Debug; }
# shellcheck disable=SC2317
job_asan()     { cd "$PROJECT_DIR" && build_and_test Debug -DENABLE_ASAN=ON; }
# shellcheck disable=SC2317
job_tsan()     { cd "$PROJECT_DIR" && build_and_test Debug -DENABLE_TSAN=ON; }
# shellcheck disable=SC2317
job_ubsan()    { cd "$PROJECT_DIR" && build_and_test Debug -DENABLE_UBSAN=ON; }

# ── Helper: coverage report generation (mirrors CI) ─────────
# shellcheck disable=SC2317  # called indirectly via run_job
generate_coverage_report() {
    echo ""
    echo "  Capturing coverage data..."
    lcov --capture --directory "${BUILD_DIR}" --output-file "${BUILD_DIR}/coverage-raw.info" \
        --ignore-errors mismatch,negative --rc lcov_branch_coverage=1 2>/dev/null
    lcov --remove "${BUILD_DIR}/coverage-raw.info" \
        '/usr/*' '*/googletest/*' '*/gtest/*' '*/test/*' '*/tests/*' '*_test.cpp' '*_test.h' \
        --output-file "${BUILD_DIR}/coverage.info" \
        --ignore-errors unused --rc lcov_branch_coverage=1 2>/dev/null

    # Print summary
    echo ""
    echo "  Coverage summary:"
    lcov --list "${BUILD_DIR}/coverage.info" --rc lcov_branch_coverage=1 2>/dev/null | tail -5
    echo ""

    # Enforce coverage threshold (mirrors CI)
    local lcov_summary
    lcov_summary=$(lcov --summary "${BUILD_DIR}/coverage.info" 2>&1) || true
    local coverage
    coverage=$(echo "$lcov_summary" | sed -n 's/.*lines\.\.\.\.\.\.: \([0-9.]*\)%.*/\1/p')
    if [[ -z "$coverage" ]] || ! echo "$coverage" | grep -Eq '^[0-9]+(\.[0-9]+)?$'; then
        echo -e "${RED}  ERROR: Failed to parse line coverage from lcov summary${RESET}"
        echo "  lcov output was:"
        echo "$lcov_summary"
        return 1
    fi
    echo "  Line coverage: ${coverage}%"
    if awk "BEGIN { exit !(${coverage} < 70.0) }"; then
        echo -e "${RED}  ERROR: Line coverage ${coverage}% is below 70% threshold${RESET}"
        return 1
    fi
    echo -e "${GREEN}  Coverage threshold (70%) met.${RESET}"

    # Generate HTML report
    if command -v genhtml &>/dev/null; then
        echo ""
        echo "  Generating HTML report..."
        genhtml "${BUILD_DIR}/coverage.info" \
            --output-directory "${BUILD_DIR}/coverage-report" \
            --title "Companion Stack Coverage" \
            --rc genhtml_branch_coverage=1 \
            --legend --highlight --demangle-cpp 2>/dev/null
        echo "  Coverage report: ${BUILD_DIR}/coverage-report/index.html"
    else
        echo "  genhtml not found — raw coverage in ${BUILD_DIR}/coverage.info"
    fi
}

# ── Job: Coverage ────────────────────────────────────────────
# shellcheck disable=SC2317  # called indirectly via run_job
job_cov() {
    cd "$PROJECT_DIR"

    # Configure + build (without running tests — avoid double test run)
    echo "  cmake -B build -DCMAKE_BUILD_TYPE=Debug -DALLOW_INSECURE_ZENOH=ON -DENABLE_COVERAGE=ON"
    cmake -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
        -DALLOW_INSECURE_ZENOH=ON \
        -DENABLE_COVERAGE=ON 2>&1

    echo "  cmake --build build -j$(nproc)"
    cmake --build "${BUILD_DIR}" -j"$(nproc)" 2>&1

    # Zero counters before the single test run
    echo ""
    echo "  Zeroing counters..."
    lcov --zerocounters --directory "${BUILD_DIR}" 2>/dev/null

    # Run tests once for clean coverage
    echo "  ctest --output-on-failure"
    ctest --test-dir "${BUILD_DIR}" --output-on-failure -j"$(nproc)"

    generate_coverage_report
}

# ── Run selected jobs ────────────────────────────────────────
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}  Local CI Runner  (mode: ${MODE})${RESET}"
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"

START_ALL=$(date +%s)

should_run "FMT"   && run_job "FMT"   "Format check (clang-format-18)"     job_fmt
should_run "BUILD" && run_job "BUILD" "Build + test (Debug)"               job_build
should_run "ASAN"  && run_job "ASAN"  "Build + test (ASan)"                job_asan
should_run "TSAN"  && run_job "TSAN"  "Build + test (TSan)"                job_tsan
should_run "UBSAN" && run_job "UBSAN" "Build + test (UBSan)"               job_ubsan
should_run "COV"   && run_job "COV"   "Coverage (Debug + lcov)"            job_cov

# ── Summary ──────────────────────────────────────────────────
ELAPSED_ALL=$(( $(date +%s) - START_ALL ))

echo ""
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}  CI Summary${RESET}"
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo -e "  Total : ${TOTAL}"
echo -e "  Passed: ${GREEN}${PASS}${RESET}"
if [[ "$FAIL" -gt 0 ]]; then
    echo -e "  Failed: ${RED}${FAIL}${RESET}  (${FAILED_JOBS[*]})"
else
    echo -e "  Failed: 0"
fi
echo -e "  Time  : ${ELAPSED_ALL}s"
echo ""

if [[ "$FAIL" -gt 0 ]]; then
    echo -e "${RED}${BOLD}✗ CI would FAIL${RESET}"
    exit 1
else
    echo -e "${GREEN}${BOLD}✓ CI would PASS${RESET}"
    exit 0
fi
