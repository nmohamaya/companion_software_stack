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

Launches review agents against a PR. By default launches memory-safety and
security reviewers, plus concurrency and fault-recovery if the diff warrants it.

Options:
  --all       Force all 4 review agents regardless of diff content
  --dry-run   Show routing decision without launching agents
EOF
    exit "${1:-1}"
}

# ── Parse arguments ─────────────────────────────────────────────────────────
PR=""
FORCE_ALL=false
DRY_RUN=false

for arg in "$@"; do
    case "$arg" in
        --all)      FORCE_ALL=true ;;
        --dry-run)  DRY_RUN=true ;;
        --help|-h)  usage 0 ;;
        *)
            if [[ -z "$PR" ]]; then
                PR="$arg"
            else
                echo -e "${RED}Error: unexpected argument '$arg'${RESET}" >&2
                usage 1
            fi
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

# ── 3. Launch reviewers in parallel ─────────────────────────────────────────
SESSION_LOG_DIR="$PROJECT_DIR/tasks/sessions"
mkdir -p "$SESSION_LOG_DIR"

TIMESTAMP="$(date +%Y-%m-%d-%H%M)"
PIDS=()
LOGS=()

REVIEW_PROMPT_TEMPLATE="Review PR #${PR}. Read the diff and provide a detailed safety review focused on your domain. Post findings as a structured list with file:line, severity (P1-P4), and fix suggestion."

for reviewer in "${REVIEWERS[@]}"; do
    LOG_FILE="${SESSION_LOG_DIR}/${TIMESTAMP}-review-pr${PR}-${reviewer}.log"
    LOGS+=("$LOG_FILE")

    echo -e "  Launching ${CYAN}${reviewer}${RESET} (log: $(basename "$LOG_FILE"))"

    (
        "$PROJECT_DIR/scripts/start-agent.sh" "$reviewer" "$REVIEW_PROMPT_TEMPLATE" \
            --skip-preflight \
            > "$LOG_FILE" 2>&1
    ) &
    PIDS+=($!)
done

echo ""
echo -e "${BOLD}Waiting for all reviewers to complete...${RESET}"

# Wait for all background processes
FAILED=0
for i in "${!PIDS[@]}"; do
    if ! wait "${PIDS[$i]}"; then
        echo -e "  ${YELLOW}WARN${RESET}  ${REVIEWERS[$i]} exited with non-zero status"
        FAILED=$((FAILED + 1))
    else
        echo -e "  ${GREEN}DONE${RESET}  ${REVIEWERS[$i]}"
    fi
done

echo ""

# ── 4. Collect and consolidate review outputs ──────────────────────────────
echo -e "${BOLD}Consolidating review findings...${RESET}"

CONSOLIDATED="## Automated Safety Review — PR #${PR}

**Reviewers:** ${REVIEWERS[*]}
**Date:** $(date +%Y-%m-%d)

---
"

for i in "${!REVIEWERS[@]}"; do
    reviewer="${REVIEWERS[$i]}"
    log_file="${LOGS[$i]}"

    CONSOLIDATED+="
### ${reviewer}

"
    if [[ -f "$log_file" && -s "$log_file" ]]; then
        # Take last 200 lines (the findings, not the startup boilerplate)
        REVIEW_CONTENT="$(tail -200 "$log_file")"
        CONSOLIDATED+="${REVIEW_CONTENT}

"
    else
        CONSOLIDATED+="_No output captured._

"
    fi
done

CONSOLIDATED+="---
_Generated by deploy-review.sh_"

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
echo "  Failed:    ${FAILED}"
echo "  Logs:      ${SESSION_LOG_DIR}/${TIMESTAMP}-review-pr${PR}-*.log"
