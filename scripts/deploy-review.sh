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
Usage: $(basename "$0") <pr-number> [--all]
       $(basename "$0") --status <pr-number>

Launches review agents against a PR. By default launches memory-safety and
security reviewers, plus concurrency and fault-recovery if the diff warrants it.

Options:
  --all              Force all 4 review agents regardless of diff content
  --dry-run          Show routing decision without launching agents
  --status <pr>      Show last review results for a PR without launching agents
EOF
    exit "${1:-1}"
}

# ── Status display function ────────────────────────────────────────────────
show_status() {
    local pr="$1"

    # Fetch the latest "Automated Safety Review" comment from the PR
    local comments
    comments="$(gh pr view "$pr" --comments --json comments \
        --jq '.comments[] | select(.body | test("Automated Safety Review")) | {createdAt, body}' \
        2>/dev/null)" || {
        echo -e "${RED}Error: failed to fetch PR #${pr} — check gh auth and PR number${RESET}" >&2
        exit 1
    }

    if [[ -z "$comments" ]]; then
        echo -e "PR #${pr}  Last review: ${YELLOW}none${RESET}"
        echo -e "Status: ${YELLOW}NO_REVIEW${RESET}"
        exit 0
    fi

    # Get the last matching comment (most recent)
    local last_comment
    last_comment="$(echo "$comments" | tail -1)"

    local review_date
    review_date="$(echo "$last_comment" | jq -r '.createdAt // empty' 2>/dev/null | cut -dT -f1)"
    if [[ -z "$review_date" ]]; then
        review_date="unknown"
    fi

    local body
    body="$(echo "$last_comment" | jq -r '.body // empty' 2>/dev/null)"

    # Extract reviewer list
    local reviewers_line
    reviewers_line="$(echo "$body" | grep -oP '(?<=\*\*Review agents:\*\* ).*' || true)"
    if [[ -z "$reviewers_line" ]]; then
        reviewers_line="unknown"
    fi

    # Extract tester list
    local testers_line
    testers_line="$(echo "$body" | grep -oP '(?<=\*\*Test agents:\*\* ).*' || true)"
    if [[ -z "$testers_line" ]]; then
        testers_line="none"
    fi

    # Count findings by severity (P1, P2, P3, P4)
    local p1 p2 p3 p4
    p1="$(echo "$body" | grep -ciP '\bP1\b' || true)"
    p2="$(echo "$body" | grep -ciP '\bP2\b' || true)"
    p3="$(echo "$body" | grep -ciP '\bP3\b' || true)"
    p4="$(echo "$body" | grep -ciP '\bP4\b' || true)"

    # Determine overall status
    local status status_color
    if [[ "$p1" -gt 0 ]]; then
        status="NEEDS_FIX"
        status_color="$RED"
    elif [[ "$p2" -gt 0 ]]; then
        status="NEEDS_FIX"
        status_color="$YELLOW"
    else
        status="PASS"
        status_color="$GREEN"
    fi

    # Display
    echo -e "PR #${pr}  Last review: ${BOLD}${review_date}${RESET}"
    echo -e "Reviewers: ${CYAN}${reviewers_line}${RESET}"
    echo -e "Testers:   ${CYAN}${testers_line}${RESET}"
    echo -e "Findings:  ${p1} P1, ${p2} P2, ${p3} P3, ${p4} P4"
    echo -e "Status:    ${status_color}${status}${RESET}"
    exit 0
}

# ── Parse arguments ─────────────────────────────────────────────────────────
PR=""
FORCE_ALL=false
DRY_RUN=false
STATUS_MODE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)      FORCE_ALL=true; shift ;;
        --dry-run)  DRY_RUN=true; shift ;;
        --help|-h)  usage 0 ;;
        --status)
            STATUS_MODE=true
            shift
            if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
                PR="$1"
                shift
            fi
            ;;
        *)
            if [[ -z "$PR" ]]; then
                PR="$1"
            else
                echo -e "${RED}Error: unexpected argument '$1'${RESET}" >&2
                usage 1
            fi
            shift
            ;;
    esac
done

if [[ -z "$PR" ]]; then
    echo -e "${RED}Error: PR number is required${RESET}" >&2
    usage 1
fi

if ! [[ "$PR" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Error: PR number must be numeric, got '$PR'${RESET}" >&2
    exit 1
fi

# If --status mode, show status and exit
if [[ "$STATUS_MODE" == "true" ]]; then
    show_status "$PR"
fi

# ── 1. Fetch PR diff ───────────────────────────────────────────────────────
echo -e "${BOLD}Fetching PR #${PR} diff...${RESET}"

DIFF="$(gh pr diff "$PR" 2>/dev/null)" || {
    echo -e "${RED}Error: failed to fetch PR #${PR} — check gh auth and PR number${RESET}" >&2
    exit 1
}

DIFF_LINES="$(echo "$DIFF" | wc -l)"
echo -e "  Diff size: ${DIFF_LINES} lines"
echo ""

# ── 2. Determine which reviewers to launch ──────────────────────────────────
REVIEWERS=("review-memory-safety" "review-security")

NEED_CONCURRENCY=false
NEED_FAULT_RECOVERY=false

if [[ "$FORCE_ALL" == "true" ]]; then
    NEED_CONCURRENCY=true
    NEED_FAULT_RECOVERY=true
else
    # Check for concurrency patterns
    if echo "$DIFF" | grep -qE 'atomic|mutex|thread|lock_guard|condition_variable|memory_order'; then
        NEED_CONCURRENCY=true
    fi

    # Check for fault-recovery related files
    if echo "$DIFF" | grep -qE 'process4_|process5_|process7_|watchdog|fault|recovery'; then
        NEED_FAULT_RECOVERY=true
    fi
fi

if [[ "$NEED_CONCURRENCY" == "true" ]]; then
    REVIEWERS+=("review-concurrency")
fi

if [[ "$NEED_FAULT_RECOVERY" == "true" ]]; then
    REVIEWERS+=("review-fault-recovery")
fi

echo -e "${BOLD}Reviewers to launch:${RESET}"
for r in "${REVIEWERS[@]}"; do
    echo -e "  ${CYAN}${r}${RESET}"
done
echo ""


# Also determine test agents
TESTERS=("test-unit")
if echo "$DIFF" | grep -qE 'common/ipc|common/hal|config/scenarios|gazebo'; then
    TESTERS+=("test-scenario")
fi

echo ""
echo -e "${BOLD}Test agents to launch:${RESET}"
for t in "${TESTERS[@]}"; do
    echo -e "  ${CYAN}${t}${RESET}"
done
echo ""

if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "${YELLOW}${BOLD}DRY RUN${RESET} — would launch ${#REVIEWERS[@]} reviewers + ${#TESTERS[@]} test agents for PR #${PR}"
    echo ""
    echo "Routing reasons:"
    echo "  memory-safety:  always"
    echo "  security:       always"
    [[ "$NEED_CONCURRENCY" == "true" ]] && echo "  concurrency:    diff contains atomic/mutex/thread patterns"
    [[ "$NEED_FAULT_RECOVERY" == "true" ]] && echo "  fault-recovery: diff touches P4/P5/P7 or watchdog/fault code"
    echo "  test-unit:      always"
    [[ "${#TESTERS[@]}" -gt 1 ]] && echo "  test-scenario:  diff touches IPC/HAL/Gazebo configs"
    exit 0
fi

# ── 3. Launch all agents in parallel ───────────────────────────────────────
SESSION_LOG_DIR="$PROJECT_DIR/tasks/sessions"
mkdir -p "$SESSION_LOG_DIR"

TIMESTAMP="$(date +%Y-%m-%d-%H%M)"

# Combined list of all agents (reviewers + testers)
ALL_AGENTS=("${REVIEWERS[@]}" "${TESTERS[@]}")
PIDS=()
LOGS=()

DIFF_FILES="$(echo "$DIFF" | grep '^diff --git' | sed 's/diff --git a\///' | sed 's/ b\/.*//' || true)"
DIFF_TRUNCATED="$(echo "$DIFF" | head -500 || true)"

# Fetch PR branch for test agents to check out
PR_BRANCH="$(gh pr view "$PR" --json headRefName --jq '.headRefName' 2>/dev/null || true)"
if [[ -z "$PR_BRANCH" ]]; then
    echo -e "${RED}Error:${RESET} Unable to determine branch name for PR #${PR}."
    echo "  Verify the PR exists and gh auth is configured."
    exit 1
fi

REVIEW_PROMPT="Review PR #${PR}.

Changed files:
${DIFF_FILES}

Diff (first 500 lines):
${DIFF_TRUNCATED}

Provide a detailed safety review focused on your domain. Post findings as a structured list with file:line, severity (P1-P4), and fix suggestion."

TEST_UNIT_PROMPT="Verify PR #${PR} (branch: ${PR_BRANCH}).

Changed files:
${DIFF_FILES}

Your job:
1. Check out the PR branch: git checkout ${PR_BRANCH}
2. Build the project: bash deploy/build.sh
3. Run all unit tests: ./tests/run_tests.sh
4. Verify test count matches or exceeds the baseline in tests/TESTS.md: ctest -N --test-dir build | grep 'Total Tests:'
5. Run tests for modules touched by the diff (look at changed file paths to determine modules)
6. Check formatting: git diff --name-only | xargs clang-format-18 --dry-run --Werror 2>&1

Report:
- Build result (pass/fail, any warnings)
- Test results (total tests, passed, failed, skipped)
- Test count vs baseline (regression?)
- Any test failures with details (test name, assertion, output)
- Module-specific test results for changed modules
- Formatting issues if any"

TEST_SCENARIO_PROMPT="Verify PR #${PR} (branch: ${PR_BRANCH}) with scenario integration tests.

Changed files:
${DIFF_FILES}

Your job:
1. Check out the PR branch: git checkout ${PR_BRANCH}
2. Build the project: bash deploy/build.sh
3. Run simulated scenario tests: tests/run_scenario.sh
4. If the diff touches IPC, HAL, or Gazebo configs, also run: tests/run_scenario_gazebo.sh
5. Verify scenario configs use full stack coverage (A* planner, 3D obstacle avoider, populated static_obstacles) unless the scenario specifically tests something else

Report:
- Scenario results (which passed, which failed)
- Any scenario regressions vs previous runs
- Config issues (downgraded backends, empty static_obstacles where inappropriate)
- Stack coverage assessment"

echo -e "${BOLD}Launching ${#ALL_AGENTS[@]} agents in parallel...${RESET}"

for agent in "${ALL_AGENTS[@]}"; do
    LOG_FILE="${SESSION_LOG_DIR}/${TIMESTAMP}-review-pr${PR}-${agent}.log"
    LOGS+=("$LOG_FILE")

    echo -e "  Launching ${CYAN}${agent}${RESET} (log: $(basename "$LOG_FILE"))"

    # Resolve model and prompt for this agent
    case "$agent" in
        test-unit)
            _MODEL="claude-sonnet-4-6"
            _PROMPT="$TEST_UNIT_PROMPT"
            ;;
        test-scenario)
            _MODEL="claude-sonnet-4-6"
            _PROMPT="$TEST_SCENARIO_PROMPT"
            ;;
        ops-github)
            _MODEL="claude-haiku-4-5-20251001"
            _PROMPT="$REVIEW_PROMPT"
            ;;
        *)
            _MODEL="claude-opus-4-6"
            _PROMPT="$REVIEW_PROMPT"
            ;;
    esac

    (
        # Review agents are read-only (acceptEdits prevents permission prompts).
        # Test agents need acceptEdits to run build/test commands.
        claude --model "$_MODEL" --agent "$agent" \
            --permission-mode acceptEdits \
            -p "$_PROMPT" \
            > "$LOG_FILE" 2>&1
    ) &
    PIDS+=($!)
done

echo ""
echo -e "${BOLD}Waiting for all agents to complete...${RESET}"

# Wait for all background processes
FAILED=0
for i in "${!PIDS[@]}"; do
    if ! wait "${PIDS[$i]}"; then
        echo -e "  ${YELLOW}WARN${RESET}  ${ALL_AGENTS[$i]} exited with non-zero status"
        FAILED=$((FAILED + 1))
    else
        echo -e "  ${GREEN}DONE${RESET}  ${ALL_AGENTS[$i]}"
    fi
done

echo ""

# ── 4. Collect and consolidate outputs ─────────────────────────────────────
echo -e "${BOLD}Consolidating findings...${RESET}"

CONSOLIDATED="## Automated Safety Review — PR #${PR}

**Review agents:** ${REVIEWERS[*]}
**Test agents:** ${TESTERS[*]}
**Date:** $(date +%Y-%m-%d)

---
"

# Review agent findings
CONSOLIDATED+="
# Safety Review Findings
"

for i in "${!REVIEWERS[@]}"; do
    reviewer="${REVIEWERS[$i]}"
    # Find the log index in ALL_AGENTS
    for j in "${!ALL_AGENTS[@]}"; do
        if [[ "${ALL_AGENTS[$j]}" == "$reviewer" ]]; then
            log_file="${LOGS[$j]}"
            break
        fi
    done

    CONSOLIDATED+="
### ${reviewer}

"
    if [[ -f "$log_file" && -s "$log_file" ]]; then
        REVIEW_CONTENT="$(tail -200 "$log_file")"
        CONSOLIDATED+="${REVIEW_CONTENT}

"
    else
        CONSOLIDATED+="_No output captured._

"
    fi
done

# Test agent results
CONSOLIDATED+="
# Test Verification Results
"

for tester in "${TESTERS[@]}"; do
    for j in "${!ALL_AGENTS[@]}"; do
        if [[ "${ALL_AGENTS[$j]}" == "$tester" ]]; then
            log_file="${LOGS[$j]}"
            break
        fi
    done

    CONSOLIDATED+="
### ${tester}

"
    if [[ -f "$log_file" && -s "$log_file" ]]; then
        TEST_CONTENT="$(tail -200 "$log_file")"
        CONSOLIDATED+="${TEST_CONTENT}

"
    else
        CONSOLIDATED+="_No output captured._

"
    fi
done

CONSOLIDATED+="---
_Generated by deploy-review.sh — ${#REVIEWERS[@]} reviewers + ${#TESTERS[@]} test agents_"

# ── 5. Post consolidated review as PR comment ──────────────────────────────
echo -e "Posting consolidated review to PR #${PR}..."

if gh pr comment "$PR" --body "$CONSOLIDATED" 2>/dev/null; then
    echo -e "  ${GREEN}DONE${RESET}  Review posted to PR #${PR}"
else
    echo -e "  ${RED}FAIL${RESET}  Could not post review comment"
    echo ""
    echo "Consolidated review saved locally. Logs:"
    for log_file in "${LOGS[@]}"; do
        echo "  $log_file"
    done
    exit 1
fi

echo ""
echo -e "${BOLD}Review complete${RESET}"
echo "  PR:        #${PR}"
echo "  Reviewers: ${#REVIEWERS[@]}"
echo "  Testers:   ${#TESTERS[@]}"
echo "  Failed:    ${FAILED}"
echo "  Logs:      ${SESSION_LOG_DIR}/${TIMESTAMP}-review-pr${PR}-*.log"
