#!/usr/bin/env bash
# deploy/view_coverage.sh — Build with coverage, run tests, generate & view HTML report.
#
# Usage:
#   bash deploy/view_coverage.sh            # Full pipeline: build → test → report (SHM)
#   bash deploy/view_coverage.sh --zenoh     # Full pipeline with Zenoh backend
#   bash deploy/view_coverage.sh --open      # Same, but auto-open report in browser
#   bash deploy/view_coverage.sh --report    # Skip build/test, just regenerate report
#   bash deploy/view_coverage.sh --summary   # Print terminal summary only (no HTML)
#
# Prerequisites: gcc/g++ (gcov), lcov, genhtml (from lcov package)
#   sudo apt install lcov
#
# Output:
#   build/coverage-report/index.html  — Full HTML coverage report
#   build/coverage.info               — lcov tracefile (for CI upload)
#
# With --zenoh:
#   build/coverage-report-zenoh/index.html
#   build/coverage-zenoh.info
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_DIR}/build"
COVERAGE_DIR="${BUILD_DIR}/coverage-report"
RAW_INFO="${BUILD_DIR}/coverage-raw.info"
FILTERED_INFO="${BUILD_DIR}/coverage.info"

# ── Parse arguments ──────────────────────────────────────────
AUTO_OPEN=0
REPORT_ONLY=0
SUMMARY_ONLY=0
ENABLE_ZENOH=0

for arg in "$@"; do
    case "$arg" in
        --open)     AUTO_OPEN=1 ;;
        --report)   REPORT_ONLY=1 ;;
        --summary)  SUMMARY_ONLY=1 ;;
        --zenoh)    ENABLE_ZENOH=1 ;;
        -h|--help)
            head -15 "$0" | tail -14
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--open] [--report] [--summary]"
            exit 1
            ;;
    esac
done

# ── Zenoh-aware paths ────────────────────────────────────────
if [[ "$ENABLE_ZENOH" -eq 1 ]]; then
    COVERAGE_DIR="${BUILD_DIR}/coverage-report-zenoh"
    RAW_INFO="${BUILD_DIR}/coverage-raw-zenoh.info"
    FILTERED_INFO="${BUILD_DIR}/coverage-zenoh.info"
    REPORT_TITLE="Companion Stack Coverage (Zenoh)"
else
    REPORT_TITLE="Companion Stack Coverage"
fi

# ── Dependency check ─────────────────────────────────────────
check_deps() {
    local missing=()
    for cmd in cmake gcc lcov genhtml; do
        if ! command -v "$cmd" &>/dev/null; then
            missing+=("$cmd")
        fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        echo "ERROR: Missing required tools: ${missing[*]}"
        echo "  Install with: sudo apt install build-essential cmake lcov"
        exit 1
    fi
}

# ── Step 1: Build with coverage ──────────────────────────────
build_with_coverage() {
    echo "══════════════════════════════════════════"
    if [[ "$ENABLE_ZENOH" -eq 1 ]]; then
        echo "  Step 1/4: Building with coverage (Zenoh)"
    else
        echo "  Step 1/4: Building with coverage (SHM)"
    fi
    echo "══════════════════════════════════════════"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    local cmake_extra_flags=()
    if [[ "$ENABLE_ZENOH" -eq 1 ]]; then
        cmake_extra_flags+=(-DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON)
    fi

    cmake -DCMAKE_BUILD_TYPE=Debug \
          -DENABLE_COVERAGE=ON \
          "${cmake_extra_flags[@]}" \
          ..
    cmake --build . -j"$(nproc)"
    echo "  Build complete."
}

# ── Step 2: Zero counters & run tests ────────────────────────
run_tests() {
    echo ""
    echo "══════════════════════════════════════════"
    echo "  Step 2/4: Zeroing counters"
    echo "══════════════════════════════════════════"
    lcov --zerocounters --directory "$BUILD_DIR"

    echo ""
    echo "══════════════════════════════════════════"
    echo "  Step 3/4: Running tests"
    echo "══════════════════════════════════════════"
    cd "$BUILD_DIR"
    ctest --output-on-failure -j"$(nproc)" || true
    echo "  Tests complete."
}

# ── Step 3: Capture & filter coverage ────────────────────────
capture_coverage() {
    echo ""
    echo "══════════════════════════════════════════"
    echo "  Step 4/4: Capturing coverage data"
    echo "══════════════════════════════════════════"

    # Capture raw counters
    lcov --capture --directory "$BUILD_DIR" \
         --output-file "$RAW_INFO" \
         --ignore-errors mismatch \
         --rc lcov_branch_coverage=1

    # Remove system headers, test code, third-party libraries
    lcov --remove "$RAW_INFO" \
         '/usr/*' \
         '*/googletest/*' \
         '*/gtest/*' \
         '*/test/*' \
         '*/tests/*' \
         '*_test.cpp' \
         '*_test.h' \
         --output-file "$FILTERED_INFO" \
         --ignore-errors unused \
         --rc lcov_branch_coverage=1

    echo "  Coverage data written to: ${FILTERED_INFO}"
}

# ── Print terminal summary ───────────────────────────────────
print_summary() {
    echo ""
    echo "══════════════════════════════════════════"
    echo "  Coverage Summary"
    echo "══════════════════════════════════════════"
    lcov --list "$FILTERED_INFO" --rc lcov_branch_coverage=1
    echo ""

    # Extract overall percentages
    local line_pct branch_pct
    line_pct=$(lcov --summary "$FILTERED_INFO" --rc lcov_branch_coverage=1 2>&1 \
               | grep -oP 'lines\.*:\s*\K[\d.]+%' || echo "N/A")
    branch_pct=$(lcov --summary "$FILTERED_INFO" --rc lcov_branch_coverage=1 2>&1 \
                 | grep -oP 'branches\.*:\s*\K[\d.]+%' || echo "N/A")
    echo "  Line coverage  : ${line_pct}"
    echo "  Branch coverage: ${branch_pct}"
}

# ── Generate HTML report ─────────────────────────────────────
generate_html() {
    echo ""
    echo "  Generating HTML report..."
    rm -rf "$COVERAGE_DIR"
    genhtml "$FILTERED_INFO" \
            --output-directory "$COVERAGE_DIR" \
            --title "$REPORT_TITLE" \
            --rc genhtml_branch_coverage=1 \
            --legend \
            --highlight \
            --demangle-cpp

    echo ""
    echo "══════════════════════════════════════════"
    echo "  HTML report: ${COVERAGE_DIR}/index.html"
    echo "══════════════════════════════════════════"
}

# ── Open in browser ──────────────────────────────────────────
open_report() {
    local url="file://${COVERAGE_DIR}/index.html"
    if command -v xdg-open &>/dev/null; then
        xdg-open "$url" 2>/dev/null &
    elif command -v open &>/dev/null; then
        open "$url"
    else
        echo "  Open manually: $url"
    fi
}

# ── Main ─────────────────────────────────────────────────────
main() {
    check_deps

    if [[ "$REPORT_ONLY" -eq 1 ]]; then
        # Skip build & test — just regenerate from existing .gcda files
        if [[ ! -d "$BUILD_DIR" ]]; then
            echo "ERROR: No build/ directory. Run without --report first."
            exit 1
        fi
        capture_coverage
    elif [[ "$SUMMARY_ONLY" -eq 0 ]]; then
        build_with_coverage
        run_tests
        capture_coverage
    fi

    # Always print terminal summary when coverage data exists
    if [[ -f "$FILTERED_INFO" ]]; then
        print_summary
    elif [[ "$SUMMARY_ONLY" -eq 1 && -f "$FILTERED_INFO" ]]; then
        print_summary
    else
        echo "ERROR: No coverage data found. Run without --summary first."
        exit 1
    fi

    # Generate HTML unless --summary only
    if [[ "$SUMMARY_ONLY" -eq 0 ]]; then
        generate_html
        if [[ "$AUTO_OPEN" -eq 1 ]]; then
            open_report
        else
            echo ""
            echo "  To open in browser:"
            echo "    xdg-open ${COVERAGE_DIR}/index.html"
        fi
    fi

    echo ""
    echo "Done."
}

main "$@"
