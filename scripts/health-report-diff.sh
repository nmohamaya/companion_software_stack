#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

echo "=========================================="
echo "  Health Report"
echo "=========================================="
echo ""

# --- Test count ---
CURRENT_TESTS=""
if [[ -d "$PROJECT_DIR/build" ]]; then
    CURRENT_TESTS=$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null \
        | grep "Total Tests:" | awk '{print $NF}' || true)
fi

BASELINE_TESTS=""
if [[ -f "$PROJECT_DIR/tests/TESTS.md" ]]; then
    BASELINE_TESTS=$(grep -oP '\*\*\d+\*\*' "$PROJECT_DIR/tests/TESTS.md" \
        | head -1 | tr -d '*' || true)
fi

echo -n "Test count:  "
if [[ -n "$CURRENT_TESTS" && -n "$BASELINE_TESTS" ]]; then
    if [[ "$CURRENT_TESTS" -ge "$BASELINE_TESTS" ]]; then
        echo -e "${GREEN}${CURRENT_TESTS}${RESET} (baseline: ${BASELINE_TESTS}) OK"
    else
        echo -e "${RED}${CURRENT_TESTS}${RESET} (baseline: ${BASELINE_TESTS}) BELOW BASELINE"
    fi
elif [[ -n "$CURRENT_TESTS" ]]; then
    echo -e "${YELLOW}${CURRENT_TESTS}${RESET} (no baseline found)"
elif [[ -n "$BASELINE_TESTS" ]]; then
    echo -e "${RED}no build found${RESET} (baseline: ${BASELINE_TESTS})"
else
    echo -e "${RED}no data available${RESET}"
fi

# --- Coverage ---
echo -n "Coverage:    "
if [[ -f "$PROJECT_DIR/build/coverage.info" ]]; then
    COV_PCT=$(lcov --summary "$PROJECT_DIR/build/coverage.info" 2>&1 \
        | grep -oP 'lines\.*:\s*\K[\d.]+' || true)
    if [[ -n "$COV_PCT" ]]; then
        COV_INT=${COV_PCT%.*}
        if [[ "$COV_INT" -ge 70 ]]; then
            echo -e "${GREEN}${COV_PCT}%${RESET}"
        else
            echo -e "${YELLOW}${COV_PCT}%${RESET} (below 70%)"
        fi
    else
        echo -e "${YELLOW}coverage.info exists but could not parse${RESET}"
    fi
else
    echo -e "${YELLOW}no coverage.info found${RESET} (run: deploy/build.sh --coverage)"
fi

# --- Build status ---
echo -n "Build:       "
if [[ -d "$PROJECT_DIR/build" ]]; then
    # Check if build directory has any executables
    BIN_COUNT=$(find "$PROJECT_DIR/build/bin" -type f -executable 2>/dev/null | wc -l || echo 0)
    if [[ "$BIN_COUNT" -gt 0 ]]; then
        # Try a quick cmake check
        if cmake --build "$PROJECT_DIR/build" --target help >/dev/null 2>&1; then
            echo -e "${GREEN}configured${RESET} (${BIN_COUNT} executables in build/bin/)"
        else
            echo -e "${YELLOW}build dir exists but cmake not configured${RESET}"
        fi
    else
        echo -e "${YELLOW}build dir exists, no executables found${RESET}"
    fi
else
    echo -e "${RED}no build directory${RESET}"
fi

echo ""
