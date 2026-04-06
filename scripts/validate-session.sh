#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ── Colors ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

# ── Usage ───────────────────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage: $(basename "$0") [--branch <branch>]

Post-session hallucination detector. Verifies build, tests, includes, and
diff sanity after an agent session.

Options:
  --branch <branch>   Branch to validate (default: current branch)
EOF
    exit "${1:-1}"
}

# ── Parse arguments ─────────────────────────────────────────────────────────
BRANCH=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --branch)
            if [[ $# -lt 2 ]]; then
                echo -e "${RED}Error: --branch requires a value${RESET}" >&2
                exit 1
            fi
            BRANCH="$2"
            shift 2
            ;;
        --help|-h)
            usage 0
            ;;
        *)
            echo -e "${RED}Error: unexpected argument '$1'${RESET}" >&2
            usage 1
            ;;
    esac
done

if [[ -z "$BRANCH" ]]; then
    BRANCH="$(git -C "$PROJECT_DIR" branch --show-current 2>/dev/null || echo "HEAD")"
fi

echo -e "${BOLD}=== Session Validation: ${BRANCH} ===${RESET}"
echo ""

# Track overall result
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

result_pass() {
    echo -e "  ${GREEN}PASS${RESET}  $1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

result_warn() {
    echo -e "  ${YELLOW}WARN${RESET}  $1"
    WARN_COUNT=$((WARN_COUNT + 1))
}

result_fail() {
    echo -e "  ${RED}FAIL${RESET}  $1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

# ── 1. Build verification ──────────────────────────────────────────────────
echo -e "${BOLD}[1/5] Build verification${RESET}"

if [[ ! -d "$PROJECT_DIR/build" ]]; then
    result_fail "No build/ directory — run cmake first"
else
    BUILD_OUTPUT="$(cmake --build "$PROJECT_DIR/build" -j"$(nproc)" 2>&1)" || {
        result_fail "Build failed"
        echo "$BUILD_OUTPUT" | tail -20
        echo ""
    }

    if [[ $? -eq 0 || -z "${BUILD_OUTPUT##*Built target*}" ]]; then
        # Check for warnings in build output (since -Werror turns them to errors,
        # a successful build means zero warnings, but double-check)
        WARNING_COUNT="$(echo "$BUILD_OUTPUT" | grep -ci "warning:" || true)"
        if [[ "$WARNING_COUNT" -gt 0 ]]; then
            result_warn "Build succeeded but $WARNING_COUNT warning(s) detected"
        else
            result_pass "Build succeeded (zero warnings)"
        fi
    fi
fi

echo ""

# ── 2. Test count ───────────────────────────────────────────────────────────
echo -e "${BOLD}[2/5] Test count verification${RESET}"

ACTUAL_COUNT="0"
if [[ -d "$PROJECT_DIR/build" ]]; then
    ACTUAL_COUNT="$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null \
        | grep "Total Tests:" | awk '{print $NF}' || echo "0")"
fi

# Extract expected count from tests/TESTS.md
EXPECTED_COUNT=""
TESTS_MD="$PROJECT_DIR/tests/TESTS.md"
if [[ -f "$TESTS_MD" ]]; then
    # Look for a number after "Total" or a prominent test count pattern
    EXPECTED_COUNT="$(grep -oP '(?i)(?:total[^0-9]*|baseline[^0-9]*|count[^0-9]*)\K[0-9]+' "$TESTS_MD" \
        | head -1 || true)"
    # Fallback: look for **NNNN** pattern (bold number)
    if [[ -z "$EXPECTED_COUNT" ]]; then
        EXPECTED_COUNT="$(grep -oP '\*\*([0-9]{3,})\*\*' "$TESTS_MD" \
            | head -1 | tr -d '*' || true)"
    fi
fi

if [[ -z "$EXPECTED_COUNT" ]]; then
    result_warn "Test count: ${ACTUAL_COUNT} (could not determine expected baseline from TESTS.md)"
elif [[ "$ACTUAL_COUNT" -eq "$EXPECTED_COUNT" ]]; then
    result_pass "Test count: ${ACTUAL_COUNT} matches expected ${EXPECTED_COUNT}"
elif [[ "$ACTUAL_COUNT" -gt "$EXPECTED_COUNT" ]]; then
    result_pass "Test count: ${ACTUAL_COUNT} (${EXPECTED_COUNT} expected — tests were added)"
else
    result_fail "Test count: ${ACTUAL_COUNT} < expected ${EXPECTED_COUNT} — tests may be missing"
fi

echo ""

# ── 3. Test execution ──────────────────────────────────────────────────────
echo -e "${BOLD}[3/5] Test execution${RESET}"

if [[ ! -d "$PROJECT_DIR/build" ]]; then
    result_fail "Cannot run tests — no build directory"
else
    TEST_OUTPUT="$(ctest --test-dir "$PROJECT_DIR/build" --output-on-failure -j"$(nproc)" 2>&1)" || true

    TESTS_FAILED="$(echo "$TEST_OUTPUT" | grep -oP '[1-9][0-9]* tests? failed' || true)"
    TESTS_PASSED="$(echo "$TEST_OUTPUT" | grep -oP '[0-9]+% tests passed' || true)"

    if [[ -n "$TESTS_FAILED" ]]; then
        result_fail "Test failures: ${TESTS_FAILED}"
        # Show failed test names
        echo "$TEST_OUTPUT" | grep -E "^\s*[0-9]+.*\*\*\*Failed" | head -10 || true
    elif echo "$TEST_OUTPUT" | grep -q "100% tests passed"; then
        result_pass "All tests passed (${TESTS_PASSED})"
    elif [[ "$ACTUAL_COUNT" == "0" ]]; then
        result_warn "No tests found to run"
    else
        result_pass "Tests completed: ${TESTS_PASSED}"
    fi
fi

echo ""

# ── 4. Include verification ────────────────────────────────────────────────
echo -e "${BOLD}[4/5] Include verification${RESET}"

# Get diff between main and current branch
DIFF_INCLUDES="$(git -C "$PROJECT_DIR" diff main...HEAD 2>/dev/null \
    | grep -oP '^\+.*#include\s+"([^"]+)"' \
    | grep -oP '"[^"]+"' \
    | tr -d '"' \
    | sort -u || true)"

if [[ -z "$DIFF_INCLUDES" ]]; then
    result_pass "No new #include directives in diff"
else
    MISSING_INCLUDES=()
    CHECKED=0

    while IFS= read -r inc; do
        [[ -z "$inc" ]] && continue
        CHECKED=$((CHECKED + 1))

        FOUND=false
        # Check common include paths
        for search_dir in \
            "$PROJECT_DIR/common" \
            "$PROJECT_DIR/process1_video_capture" \
            "$PROJECT_DIR/process2_perception" \
            "$PROJECT_DIR/process3_slam_vio_nav" \
            "$PROJECT_DIR/process4_mission_planner" \
            "$PROJECT_DIR/process5_comms" \
            "$PROJECT_DIR/process6_payload_manager" \
            "$PROJECT_DIR/process7_system_monitor" \
            "$PROJECT_DIR/tests"; do

            if find "$search_dir" -path "*/$inc" -print -quit 2>/dev/null | grep -q .; then
                FOUND=true
                break
            fi
            # Also check include/ subdirectory pattern
            if find "$search_dir" -path "*/include/$inc" -print -quit 2>/dev/null | grep -q .; then
                FOUND=true
                break
            fi
        done

        # Check if it might be a system/third-party header (heuristic)
        if [[ "$FOUND" == "false" ]]; then
            # Skip common system/third-party prefixes
            case "$inc" in
                zenoh/*|Eigen/*|opencv2/*|gz/*|mavsdk/*|spdlog/*|nlohmann/*|gtest/*|gmock/*)
                    FOUND=true  # third-party, skip
                    ;;
            esac
        fi

        if [[ "$FOUND" == "false" ]]; then
            MISSING_INCLUDES+=("$inc")
        fi
    done <<< "$DIFF_INCLUDES"

    if [[ ${#MISSING_INCLUDES[@]} -eq 0 ]]; then
        result_pass "All ${CHECKED} new includes resolved"
    else
        result_fail "${#MISSING_INCLUDES[@]} include(s) not found:"
        for mi in "${MISSING_INCLUDES[@]}"; do
            echo -e "    ${RED}-${RESET} $mi"
        done
    fi
fi

echo ""

# ── 5. Diff sanity ─────────────────────────────────────────────────────────
echo -e "${BOLD}[5/5] Diff sanity${RESET}"

# Check if a PR exists for this branch
PR_NUMBER="$(gh pr list --head "$BRANCH" --json number --jq '.[0].number' 2>/dev/null || true)"

if [[ -z "$PR_NUMBER" || "$PR_NUMBER" == "null" ]]; then
    result_warn "No PR found for branch '$BRANCH' — skipping PR body vs diff comparison"
else
    echo -e "  ${CYAN}INFO${RESET}  PR #${PR_NUMBER} found for branch"

    # Get actual diff stats
    DIFF_STAT="$(git -C "$PROJECT_DIR" diff --stat main...HEAD 2>/dev/null || true)"
    FILES_CHANGED="$(echo "$DIFF_STAT" | tail -1 | grep -oP '[0-9]+ files? changed' || echo "unknown")"
    INSERTIONS="$(echo "$DIFF_STAT" | tail -1 | grep -oP '[0-9]+ insertions?' || echo "0 insertions")"
    DELETIONS="$(echo "$DIFF_STAT" | tail -1 | grep -oP '[0-9]+ deletions?' || echo "0 deletions")"

    echo -e "  ${CYAN}INFO${RESET}  Diff stats: ${FILES_CHANGED}, ${INSERTIONS}, ${DELETIONS}"

    # Get PR body
    PR_BODY="$(gh pr view "$PR_NUMBER" --json body --jq '.body' 2>/dev/null || true)"

    if [[ -n "$PR_BODY" ]]; then
        # Basic sanity: check if PR mentions files that don't exist in the diff
        PR_MENTIONED_FILES="$(echo "$PR_BODY" | grep -oP '[a-zA-Z0-9_/]+\.(cpp|h|md)' | sort -u || true)"
        DIFF_FILES="$(git -C "$PROJECT_DIR" diff --name-only main...HEAD 2>/dev/null | sort -u || true)"

        PHANTOM_FILES=()
        while IFS= read -r pf; do
            [[ -z "$pf" ]] && continue
            # Check if this file is mentioned in the diff or exists in the repo
            if ! echo "$DIFF_FILES" | grep -qF "$pf"; then
                if ! [[ -f "$PROJECT_DIR/$pf" ]]; then
                    PHANTOM_FILES+=("$pf")
                fi
            fi
        done <<< "$PR_MENTIONED_FILES"

        if [[ ${#PHANTOM_FILES[@]} -eq 0 ]]; then
            result_pass "PR body file references check out"
        else
            result_warn "PR body mentions ${#PHANTOM_FILES[@]} file(s) not in diff or repo:"
            for pf in "${PHANTOM_FILES[@]}"; do
                echo -e "    ${YELLOW}-${RESET} $pf"
            done
        fi
    else
        result_warn "PR body is empty — nothing to cross-check"
    fi
fi

echo ""

# ── Final verdict ───────────────────────────────────────────────────────────
echo -e "${BOLD}=== Verdict ===${RESET}"
echo -e "  Passed: ${GREEN}${PASS_COUNT}${RESET}"
echo -e "  Warns:  ${YELLOW}${WARN_COUNT}${RESET}"
echo -e "  Failed: ${RED}${FAIL_COUNT}${RESET}"
echo ""

if [[ $FAIL_COUNT -gt 0 ]]; then
    echo -e "  ${RED}${BOLD}FAIL${RESET} — ${FAIL_COUNT} check(s) failed. Investigate before merging."
    exit 1
elif [[ $WARN_COUNT -gt 0 ]]; then
    echo -e "  ${YELLOW}${BOLD}WARN${RESET} — All critical checks passed, ${WARN_COUNT} warning(s) to review."
    exit 0
else
    echo -e "  ${GREEN}${BOLD}PASS${RESET} — All checks passed."
    exit 0
fi
