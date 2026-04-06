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
Usage: $(basename "$0") <issue-number> [--dry-run]

Fetches a GitHub issue, routes it to the appropriate agent role based on
labels, creates a worktree, and launches the agent.
EOF
    exit "${1:-1}"
}

# ── Parse arguments ─────────────────────────────────────────────────────────
ISSUE=""
DRY_RUN=false

for arg in "$@"; do
    case "$arg" in
        --dry-run)  DRY_RUN=true ;;
        --help|-h)  usage 0 ;;
        *)
            if [[ -z "$ISSUE" ]]; then
                ISSUE="$arg"
            else
                echo -e "${RED}Error: unexpected argument '$arg'${RESET}" >&2
                usage 1
            fi
            ;;
    esac
done

if [[ -z "$ISSUE" ]]; then
    echo -e "${RED}Error: issue number is required${RESET}" >&2
    usage 1
fi

# Validate issue number is numeric
if ! [[ "$ISSUE" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Error: issue number must be numeric, got '$ISSUE'${RESET}" >&2
    exit 1
fi

# ── 1. Fetch issue from GitHub ──────────────────────────────────────────────
echo -e "${BOLD}Fetching issue #${ISSUE}...${RESET}"

ISSUE_JSON="$(gh issue view "$ISSUE" --json title,body,labels,number 2>/dev/null)" || {
    echo -e "${RED}Error: failed to fetch issue #${ISSUE} — check gh auth and issue number${RESET}" >&2
    exit 1
}

TITLE="$(echo "$ISSUE_JSON" | jq -r '.title')"
BODY="$(echo "$ISSUE_JSON" | jq -r '.body // ""')"
LABELS="$(echo "$ISSUE_JSON" | jq -r '.labels[].name' 2>/dev/null || true)"

echo -e "  Title:  ${CYAN}${TITLE}${RESET}"
echo -e "  Labels: ${LABELS:-<none>}"
echo ""

# ── 2. Route to agent role based on labels ──────────────────────────────────
ROLE=""
IS_BUG=false
IS_REFACTOR=false
IS_PERF=false
DOMAIN_LABEL=""

# Scan labels
while IFS= read -r label; do
    [[ -z "$label" ]] && continue
    case "$label" in
        bug)              IS_BUG=true ;;
        refactor)         IS_REFACTOR=true ;;
        performance)      IS_PERF=true ;;
        perception)       DOMAIN_LABEL="perception" ;;
        nav-planning)     DOMAIN_LABEL="nav" ;;
        comms|integration) DOMAIN_LABEL="integration" ;;
        common|infra|infrastructure|modularity) DOMAIN_LABEL="infra-core" ;;
        platform|deploy|ci|bsp) DOMAIN_LABEL="infra-platform" ;;
        safety-audit)     ROLE="review-memory-safety" ;;
        security-audit)   ROLE="review-security" ;;
        test-coverage)    ROLE="test-unit" ;;
        cross-domain)     ROLE="tech-lead" ;;
    esac
done <<< "$LABELS"

# Resolve role from domain if not directly set
if [[ -z "$ROLE" ]]; then
    if [[ "$IS_REFACTOR" == "true" || "$IS_PERF" == "true" ]]; then
        # Refactor/performance: use domain-specific feature agent
        if [[ -n "$DOMAIN_LABEL" ]]; then
            ROLE="feature-${DOMAIN_LABEL}"
        else
            echo -e "${RED}Error: refactor/performance issue needs a domain label (perception, nav-planning, comms, common, etc.)${RESET}" >&2
            echo "  Add a domain label to issue #${ISSUE} and re-run."
            exit 1
        fi
    elif [[ -n "$DOMAIN_LABEL" ]]; then
        ROLE="feature-${DOMAIN_LABEL}"
    else
        echo -e "${RED}Error: cannot determine agent role — no recognized domain label found${RESET}" >&2
        echo ""
        echo "Recognized labels for routing:"
        echo "  perception, nav-planning, comms, integration, common, infra, modularity,"
        echo "  platform, deploy, ci, safety-audit, security-audit, test-coverage, cross-domain"
        echo ""
        echo "Add one of these labels to issue #${ISSUE} and re-run."
        exit 1
    fi
fi

# ── 3. Determine branch name ───────────────────────────────────────────────
# Sanitize title for branch name
SLUG="$(echo "$TITLE" | tr '[:upper:]' '[:lower:]' | tr ' ' '-' | tr -cd 'a-z0-9-' | head -c 40)"
# Remove trailing hyphens
SLUG="${SLUG%-}"

if [[ "$IS_BUG" == "true" ]]; then
    BRANCH="fix/issue-${ISSUE}-${SLUG}"
else
    BRANCH="feat/issue-${ISSUE}-${SLUG}"
fi

# ── 4. Dry-run or execute ──────────────────────────────────────────────────
echo -e "${BOLD}Routing decision${RESET}"
echo -e "  Issue:  #${ISSUE} — ${TITLE}"
echo -e "  Role:   ${CYAN}${ROLE}${RESET}"
echo -e "  Branch: ${BRANCH}"
echo ""

if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "${YELLOW}DRY RUN${RESET} — would perform:"
    echo "  1. git worktree add .claude/worktrees/issue-${ISSUE} -b ${BRANCH} main"
    echo "  2. Update tasks/active-work.md"
    echo "  3. Launch: start-agent.sh ${ROLE} <issue prompt>"
    exit 0
fi

# ── Create worktree ────────────────────────────────────────────────────────
WORKTREE_DIR="$PROJECT_DIR/.claude/worktrees/issue-${ISSUE}"

if [[ -d "$WORKTREE_DIR" ]]; then
    echo -e "${YELLOW}WARN${RESET}  Worktree already exists: $WORKTREE_DIR"
    echo "  Re-using existing worktree."
else
    echo -e "Creating worktree..."
    git -C "$PROJECT_DIR" worktree add "$WORKTREE_DIR" -b "$BRANCH" main 2>&1 || {
        # Branch may already exist — try without -b
        echo -e "${YELLOW}WARN${RESET}  Branch may already exist, trying checkout..."
        git -C "$PROJECT_DIR" worktree add "$WORKTREE_DIR" "$BRANCH" 2>&1 || {
            echo -e "${RED}Error: failed to create worktree${RESET}" >&2
            exit 1
        }
    }
    echo -e "  ${GREEN}DONE${RESET}  Worktree: $WORKTREE_DIR"
fi

# ── Update active-work.md ──────────────────────────────────────────────────
ACTIVE_WORK="$PROJECT_DIR/tasks/active-work.md"
if [[ ! -f "$ACTIVE_WORK" ]]; then
    echo "# Active Work" > "$ACTIVE_WORK"
    echo "" >> "$ACTIVE_WORK"
fi

TIMESTAMP="$(date +%Y-%m-%d\ %H:%M)"
cat >> "$ACTIVE_WORK" <<EOF

## Issue #${ISSUE} — ${TITLE}
- **Agent:** ${ROLE}
- **Branch:** ${BRANCH}
- **Worktree:** .claude/worktrees/issue-${ISSUE}
- **Started:** ${TIMESTAMP}
- **Status:** in-progress
EOF

echo -e "  ${GREEN}DONE${RESET}  Updated tasks/active-work.md"

# ── Construct prompt ────────────────────────────────────────────────────────
PROMPT="Issue #${ISSUE}: ${TITLE}

${BODY}

Work in this worktree. The branch is '${BRANCH}'. Implement the issue, add tests, and ensure the build passes."

# ── Launch agent in the worktree ────────────────────────────────────────────
echo ""
cd "$WORKTREE_DIR"
exec "$PROJECT_DIR/scripts/start-agent.sh" "$ROLE" "$PROMPT" --skip-preflight
