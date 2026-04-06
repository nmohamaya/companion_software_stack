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

# ── Role → Model mapping ───────────────────────────────────────────────────
declare -A ROLE_MODEL=(
    [tech-lead]="claude-opus-4-6"
    [feature-perception]="claude-opus-4-6"
    [feature-nav]="claude-opus-4-6"
    [feature-integration]="claude-opus-4-6"
    [feature-infra-core]="claude-opus-4-6"
    [feature-infra-platform]="claude-opus-4-6"
    [review-memory-safety]="claude-opus-4-6"
    [review-concurrency]="claude-opus-4-6"
    [review-fault-recovery]="claude-opus-4-6"
    [review-security]="claude-opus-4-6"
    [test-unit]="claude-sonnet-4-6"
    [test-scenario]="claude-sonnet-4-6"
    [ops-github]="claude-haiku-4-5-20251001"
)

declare -A ROLE_TIER=(
    [tech-lead]="opus"
    [feature-perception]="opus"
    [feature-nav]="opus"
    [feature-integration]="opus"
    [feature-infra-core]="opus"
    [feature-infra-platform]="opus"
    [review-memory-safety]="opus"
    [review-concurrency]="opus"
    [review-fault-recovery]="opus"
    [review-security]="opus"
    [test-unit]="sonnet"
    [test-scenario]="sonnet"
    [ops-github]="haiku"
)

ALL_ROLES=(
    tech-lead
    feature-perception
    feature-nav
    feature-integration
    feature-infra-core
    feature-infra-platform
    review-memory-safety
    review-concurrency
    review-fault-recovery
    review-security
    test-unit
    test-scenario
    ops-github
)

# ── Usage ───────────────────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage: $(basename "$0") <role> "task description" [options]

Options:
  --list            Print all available roles and exit
  --dry-run         Run agent in read-only analysis mode
  --skip-preflight  Skip pre-flight checks

Roles: ${ALL_ROLES[*]}
EOF
    exit "${1:-1}"
}

# ── List roles ──────────────────────────────────────────────────────────────
list_roles() {
    printf "\n${BOLD}%-28s %-10s %s${RESET}\n" "ROLE" "TIER" "MODEL"
    printf "%-28s %-10s %s\n" "---" "---" "---"
    for role in "${ALL_ROLES[@]}"; do
        printf "%-28s %-10s %s\n" "$role" "${ROLE_TIER[$role]}" "${ROLE_MODEL[$role]}"
    done
    echo ""
    exit 0
}

# ── Parse arguments ─────────────────────────────────────────────────────────
DRY_RUN=false
SKIP_PREFLIGHT=false
ROLE=""
TASK=""

for arg in "$@"; do
    case "$arg" in
        --list)       list_roles ;;
        --dry-run)    DRY_RUN=true ;;
        --skip-preflight) SKIP_PREFLIGHT=true ;;
        --help|-h)    usage 0 ;;
        *)
            if [[ -z "$ROLE" ]]; then
                ROLE="$arg"
            elif [[ -z "$TASK" ]]; then
                TASK="$arg"
            else
                echo -e "${RED}Error: unexpected argument '$arg'${RESET}" >&2
                usage 1
            fi
            ;;
    esac
done

if [[ -z "$ROLE" ]]; then
    echo -e "${RED}Error: role is required${RESET}" >&2
    usage 1
fi

if [[ -z "$TASK" && "$DRY_RUN" == "false" ]]; then
    echo -e "${RED}Error: task description is required${RESET}" >&2
    usage 1
fi

# ── Validate role ───────────────────────────────────────────────────────────
MODEL="${ROLE_MODEL[$ROLE]:-}"
if [[ -z "$MODEL" ]]; then
    echo -e "${RED}Error: unknown role '$ROLE'${RESET}" >&2
    echo "Available roles: ${ALL_ROLES[*]}"
    exit 1
fi

# ── Pre-flight checks ──────────────────────────────────────────────────────
if [[ "$SKIP_PREFLIGHT" == "false" ]]; then
    echo -e "${BOLD}Pre-flight checks${RESET}"

    # Agent file exists
    AGENT_FILE="$PROJECT_DIR/.claude/agents/${ROLE}.md"
    if [[ -f "$AGENT_FILE" ]]; then
        echo -e "  ${GREEN}PASS${RESET}  Agent file: ${ROLE}.md"
    else
        echo -e "  ${RED}FAIL${RESET}  Agent file not found: $AGENT_FILE"
        exit 1
    fi

    # Git repo
    if git -C "$PROJECT_DIR" rev-parse --git-dir &>/dev/null; then
        echo -e "  ${GREEN}PASS${RESET}  Git repository"
    else
        echo -e "  ${RED}FAIL${RESET}  Not a git repository"
        exit 1
    fi

    # Current branch
    BRANCH="$(git -C "$PROJECT_DIR" branch --show-current 2>/dev/null || echo "detached")"
    echo -e "  ${CYAN}INFO${RESET}  Branch: $BRANCH"

    # Uncommitted changes
    if [[ -n "$(git -C "$PROJECT_DIR" status --porcelain 2>/dev/null)" ]]; then
        echo -e "  ${YELLOW}WARN${RESET}  Uncommitted changes detected"
    else
        echo -e "  ${GREEN}PASS${RESET}  Working tree clean"
    fi

    # Build exists
    if [[ -d "$PROJECT_DIR/build/bin" ]]; then
        echo -e "  ${GREEN}PASS${RESET}  Build directory exists"
    else
        echo -e "  ${YELLOW}WARN${RESET}  No build/bin/ directory — build may be needed"
    fi

    # clang-format-18
    if command -v clang-format-18 &>/dev/null; then
        echo -e "  ${GREEN}PASS${RESET}  clang-format-18 available"
    else
        echo -e "  ${YELLOW}WARN${RESET}  clang-format-18 not found in PATH"
    fi

    echo ""
fi

# ── Tool restriction notes ─────────────────────────────────────────────────
case "$ROLE" in
    review-*)
        echo -e "${CYAN}NOTE${RESET}  Review agents are read-only (enforced by agent file)"
        ;;
    ops-github)
        echo -e "${CYAN}NOTE${RESET}  ops-github: Bash restricted to gh CLI (enforced by agent file)"
        ;;
esac

# ── Build task prompt ───────────────────────────────────────────────────────
PROMPT="$TASK"
if [[ "$DRY_RUN" == "true" ]]; then
    PROMPT="${PROMPT:+${PROMPT} }READ-ONLY mode: do not edit files, only analyze and plan"
fi

# ── Export role environment variable ────────────────────────────────────────
export CLAUDE_AGENT_ROLE="$ROLE"

# ── Print summary ──────────────────────────────────────────────────────────
TASK_DISPLAY="${TASK:0:80}"
if [[ ${#TASK} -gt 80 ]]; then
    TASK_DISPLAY="${TASK_DISPLAY}..."
fi

MODE="normal"
if [[ "$DRY_RUN" == "true" ]]; then
    MODE="dry-run (read-only)"
fi

echo -e "${BOLD}Launching agent${RESET}"
echo -e "  Role:  ${CYAN}${ROLE}${RESET}"
echo -e "  Model: ${ROLE_TIER[$ROLE]} (${MODEL})"
echo -e "  Task:  ${TASK_DISPLAY}"
echo -e "  Mode:  ${MODE}"
echo ""

# ── Launch ──────────────────────────────────────────────────────────────────
# Launch the Claude CLI with the specified model and agent definition.
# Agent files in .claude/agents/ are auto-discovered by name (without .md extension).
# Use -p (print mode) for non-interactive/scripted runs, or interactive mode for
# hands-on sessions.

CMD=(claude --model "$MODEL" --agent "$ROLE")

if [[ -n "$PROMPT" ]]; then
    # Non-interactive: pass prompt via -p (print mode)
    CMD+=(-p "$PROMPT")
else
    # Interactive: open a session with the agent's role context
    true  # no extra flags needed
fi

exec "${CMD[@]}"
