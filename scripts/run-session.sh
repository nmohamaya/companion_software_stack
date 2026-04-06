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
Usage: $(basename "$0") <role> "task description" [options]

Full session orchestrator — runs pre-flight, health baseline, agent,
post-session validation, and changelog update with output logging.

Options:
  --issue <number>  Link a GitHub issue number to the session
EOF
    exit "${1:-1}"
}

if [[ $# -lt 2 ]]; then
    usage 1
fi

ROLE="$1"
TASK="$2"
shift 2

# ── Parse remaining flags ──────────────────────────────────────────────────
ISSUE=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)
            if [[ $# -lt 2 ]]; then
                echo -e "${RED}Error: --issue requires a value${RESET}" >&2
                exit 1
            fi
            ISSUE="$2"
            shift 2
            ;;
        *)
            echo -e "${RED}Error: unexpected argument '$1'${RESET}" >&2
            usage 1
            ;;
    esac
done

SESSION_START="$(date +%s)"
SESSION_TIMESTAMP="$(date +%Y-%m-%d-%H%M)"
SESSION_LOG_DIR="$PROJECT_DIR/tasks/sessions"
mkdir -p "$SESSION_LOG_DIR"
SESSION_LOG="$SESSION_LOG_DIR/${SESSION_TIMESTAMP}-${ROLE}.log"

# ── Helper: log to both console and file ────────────────────────────────────
log() {
    echo -e "$@" | tee -a "$SESSION_LOG"
}

log "${BOLD}=== Session: ${ROLE} @ ${SESSION_TIMESTAMP} ===${RESET}"
log ""

# ── 1. Pre-flight ──────────────────────────────────────────────────────────
log "${BOLD}[1/6] Pre-flight checks${RESET}"

# Git state
BRANCH="$(git -C "$PROJECT_DIR" branch --show-current 2>/dev/null || echo "detached")"
log "  Branch: ${CYAN}${BRANCH}${RESET}"

UNCOMMITTED=""
if [[ -n "$(git -C "$PROJECT_DIR" status --porcelain 2>/dev/null)" ]]; then
    UNCOMMITTED="$(git -C "$PROJECT_DIR" status --porcelain 2>/dev/null | wc -l)"
    log "  ${YELLOW}WARN${RESET}  $UNCOMMITTED uncommitted file(s)"
else
    log "  ${GREEN}PASS${RESET}  Working tree clean"
fi

# Behind remote
BEHIND="$(git -C "$PROJECT_DIR" rev-list --count HEAD..@{upstream} 2>/dev/null || echo "N/A")"
if [[ "$BEHIND" != "N/A" && "$BEHIND" -gt 0 ]]; then
    log "  ${YELLOW}WARN${RESET}  Branch is $BEHIND commit(s) behind remote"
fi

# Active work conflicts
ACTIVE_WORK="$PROJECT_DIR/tasks/active-work.md"
if [[ -f "$ACTIVE_WORK" ]]; then
    log "  ${CYAN}INFO${RESET}  Active work file exists — check for conflicts manually"
else
    log "  ${CYAN}INFO${RESET}  No active-work.md found"
fi

# Build existence
if [[ -d "$PROJECT_DIR/build/bin" ]]; then
    log "  ${GREEN}PASS${RESET}  Build directory exists"
else
    log "  ${YELLOW}WARN${RESET}  No build/bin/ — build may be needed"
fi

# Record starting commit for later comparison
START_COMMIT="$(git -C "$PROJECT_DIR" rev-parse HEAD 2>/dev/null || echo "unknown")"

log ""

# ── 2. Health baseline ─────────────────────────────────────────────────────
log "${BOLD}[2/6] Health baseline${RESET}"

BASELINE_TEST_COUNT="0"
if [[ -d "$PROJECT_DIR/build" ]]; then
    BASELINE_TEST_COUNT="$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null \
        | grep "Total Tests:" | awk '{print $NF}' || echo "0")"
fi
log "  Test count (baseline): ${BASELINE_TEST_COUNT}"

# Quick format scan of changed files
CHANGED_FILES="$(git -C "$PROJECT_DIR" diff --name-only HEAD 2>/dev/null \
    | grep -E '\.(cpp|h)$' || true)"
if [[ -n "$CHANGED_FILES" ]]; then
    FORMAT_ISSUES=0
    while IFS= read -r f; do
        if [[ -f "$PROJECT_DIR/$f" ]] && command -v clang-format-18 &>/dev/null; then
            if ! clang-format-18 --dry-run --Werror "$PROJECT_DIR/$f" 2>/dev/null; then
                FORMAT_ISSUES=$((FORMAT_ISSUES + 1))
            fi
        fi
    done <<< "$CHANGED_FILES"
    if [[ $FORMAT_ISSUES -gt 0 ]]; then
        log "  ${YELLOW}WARN${RESET}  $FORMAT_ISSUES file(s) with format issues"
    else
        log "  ${GREEN}PASS${RESET}  Changed files pass format check"
    fi
else
    log "  ${CYAN}INFO${RESET}  No changed C++ files to format-check"
fi

log ""

# ── 3. Launch agent ────────────────────────────────────────────────────────
log "${BOLD}[3/6] Launching agent${RESET}"
log "  Role: ${ROLE}"
log "  Task: ${TASK:0:80}${TASK:80:+...}"
log ""

AGENT_EXIT=0
"$PROJECT_DIR/scripts/start-agent.sh" "$ROLE" "$TASK" --skip-preflight 2>&1 \
    | tee -a "$SESSION_LOG" \
    || AGENT_EXIT=$?

log ""

# ── 4. Post-session validation ─────────────────────────────────────────────
log "${BOLD}[4/6] Post-session validation${RESET}"

VALIDATE_EXIT=0
VALIDATE_OUTPUT="$("$PROJECT_DIR/scripts/validate-session.sh" --branch "$BRANCH" 2>&1)" || VALIDATE_EXIT=$?
echo "$VALIDATE_OUTPUT" | tee -a "$SESSION_LOG"

if [[ $VALIDATE_EXIT -eq 0 ]]; then
    if echo "$VALIDATE_OUTPUT" | grep -q "WARN"; then
        VALIDATE_RESULT="WARN"
    else
        VALIDATE_RESULT="PASS"
    fi
else
    VALIDATE_RESULT="FAIL"
fi
log "  Validation result: ${VALIDATE_RESULT}"
log ""

# ── 5. Post-session metrics ────────────────────────────────────────────────
log "${BOLD}[5/6] Post-session report${RESET}"

SESSION_END="$(date +%s)"
ELAPSED=$(( SESSION_END - SESSION_START ))
ELAPSED_MIN=$(( ELAPSED / 60 ))
ELAPSED_SEC=$(( ELAPSED % 60 ))

# New test count
POST_TEST_COUNT="0"
if [[ -d "$PROJECT_DIR/build" ]]; then
    POST_TEST_COUNT="$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null \
        | grep "Total Tests:" | awk '{print $NF}' || echo "0")"
fi

# Uncommitted files
POST_UNCOMMITTED="$(git -C "$PROJECT_DIR" status --porcelain 2>/dev/null | wc -l || echo "0")"

# Commits since session start
NEW_COMMITS="$(git -C "$PROJECT_DIR" rev-list --count "${START_COMMIT}..HEAD" 2>/dev/null || echo "0")"

log "  Tests:     ${BASELINE_TEST_COUNT} -> ${POST_TEST_COUNT}"
log "  Uncommitted files: ${POST_UNCOMMITTED}"
log "  New commits:       ${NEW_COMMITS}"
log "  Elapsed:           ${ELAPSED_MIN}m ${ELAPSED_SEC}s"
log "  Agent exit code:   ${AGENT_EXIT}"
log "  Validation:        ${VALIDATE_RESULT}"
log ""

# Detect PR number for changelog entry
PR_NUMBER="$(gh pr list --head "$BRANCH" --json number --jq '.[0].number' 2>/dev/null || true)"
PR_REF=""
if [[ -n "$PR_NUMBER" && "$PR_NUMBER" != "null" ]]; then
    PR_REF="PR #${PR_NUMBER}"
else
    PR_REF="no PR"
fi

# Resolve model name from start-agent.sh role mapping
case "$ROLE" in
    test-unit|test-scenario) SESSION_MODEL="sonnet" ;;
    ops-github)              SESSION_MODEL="haiku" ;;
    *)                       SESSION_MODEL="opus" ;;
esac

# ── 6. Append to agent changelog ───────────────────────────────────────────
log "${BOLD}[6/6] Updating agent changelog${RESET}"

CHANGELOG="$PROJECT_DIR/tasks/agent-changelog.md"
if [[ ! -f "$CHANGELOG" ]]; then
    echo "# Agent Changelog" > "$CHANGELOG"
    echo "" >> "$CHANGELOG"
fi

SESSION_DATE="$(date +%Y-%m-%d)"
ISSUE_REF=""
if [[ -n "$ISSUE" ]]; then
    ISSUE_REF=" | #${ISSUE}"
fi

cat >> "$CHANGELOG" <<EOF

### ${SESSION_DATE} | ${ROLE} | ${SESSION_MODEL} | ${PR_REF}${ISSUE_REF}
- **Task:** ${TASK:0:120}
- **Branch:** ${BRANCH}
- **Tests:** ${BASELINE_TEST_COUNT} -> ${POST_TEST_COUNT}
- **Commits:** ${NEW_COMMITS}
- **Duration:** ${ELAPSED_MIN}m ${ELAPSED_SEC}s
- **Validation:** ${VALIDATE_RESULT}
- **Exit:** ${AGENT_EXIT}
- **Log:** tasks/sessions/$(basename "$SESSION_LOG")
EOF

log "  ${GREEN}DONE${RESET}  Appended to tasks/agent-changelog.md"
log "  Session log: ${SESSION_LOG}"
log ""

exit "$AGENT_EXIT"
