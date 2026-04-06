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
Usage: $(basename "$0") <issue-number> [options]

Fetches a GitHub issue, routes it to the appropriate agent role based on
labels, creates a worktree, and launches the agent.

Options:
  --base <branch>   Base branch to branch from (default: main)
                    Use for integration branches: --base integration/epic-263
  --auto            Agent works autonomously, then writes AGENT_REPORT.md
                    for you to review. You accept, reject, or request changes.
  --headless        Non-interactive print mode (for CI with pre-configured perms)
  --dry-run         Print routing decision without executing
  --help, -h        Show this help

Modes:
  (default)   Interactive — you approve changes and converse in real time
  --auto      Autonomous — agent works, writes report, you review afterward
EOF
    exit "${1:-1}"
}

# ── Preflight checks ───────────────────────────────────────────────────────
for cmd in gh jq; do
    if ! command -v "$cmd" &>/dev/null; then
        echo -e "${RED}Error: '$cmd' is required but not installed${RESET}" >&2
        exit 1
    fi
done

# ── Parse arguments ─────────────────────────────────────────────────────────
ISSUE=""
DRY_RUN=false
HEADLESS=false
AUTO=false
BASE_BRANCH="main"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)   DRY_RUN=true; shift ;;
        --headless)  HEADLESS=true; shift ;;
        --auto)      AUTO=true; shift ;;
        --base)
            if [[ $# -lt 2 ]]; then
                echo -e "${RED}Error: --base requires a branch name${RESET}" >&2
                exit 1
            fi
            BASE_BRANCH="$2"
            shift 2
            ;;
        --help|-h)  usage 0 ;;
        *)
            if [[ -z "$ISSUE" ]]; then
                ISSUE="$1"
            else
                echo -e "${RED}Error: unexpected argument '$1'${RESET}" >&2
                usage 1
            fi
            shift
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
# Sanitize body: strip control characters and cap length to prevent prompt injection
BODY="$(echo "$ISSUE_JSON" | jq -r '.body // ""' | tr -cd '[:print:]\n\t' | head -c 4096)"
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
DOMAIN_PRI=0

# Scan labels
while IFS= read -r label; do
    [[ -z "$label" ]] && continue
    case "$label" in
        bug)              IS_BUG=true ;;
        refactor)         IS_REFACTOR=true ;;
        performance)      IS_PERF=true ;;
        # Domain labels: highest priority wins. Specific labels (ipc, perception)
        # beat broad labels (platform, infrastructure). Priority: 3=specific, 2=mid, 1=broad.
        perception|domain:perception) if [[ "${DOMAIN_PRI:-0}" -lt 3 ]]; then DOMAIN_LABEL="perception"; DOMAIN_PRI=3; fi ;;
        nav-planning|domain:nav)     if [[ "${DOMAIN_PRI:-0}" -lt 3 ]]; then DOMAIN_LABEL="nav"; DOMAIN_PRI=3; fi ;;
        comms|domain:comms)          if [[ "${DOMAIN_PRI:-0}" -lt 3 ]]; then DOMAIN_LABEL="integration"; DOMAIN_PRI=3; fi ;;
        ipc)                         if [[ "${DOMAIN_PRI:-0}" -lt 3 ]]; then DOMAIN_LABEL="infra-core"; DOMAIN_PRI=3; fi ;;
        integration|domain:integration) if [[ "${DOMAIN_PRI:-0}" -lt 2 ]]; then DOMAIN_LABEL="integration"; DOMAIN_PRI=2; fi ;;
        common|infra|infrastructure|modularity|domain:infra-core) if [[ "${DOMAIN_PRI:-0}" -lt 2 ]]; then DOMAIN_LABEL="infra-core"; DOMAIN_PRI=2; fi ;;
        platform|deploy|ci|bsp|domain:infra-platform) if [[ "${DOMAIN_PRI:-0}" -lt 1 ]]; then DOMAIN_LABEL="infra-platform"; DOMAIN_PRI=1; fi ;;
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
    BRANCH="feature/issue-${ISSUE}-${SLUG}"
fi

# ── 4. Dry-run or execute ──────────────────────────────────────────────────
echo -e "${BOLD}Routing decision${RESET}"
echo -e "  Issue:  #${ISSUE} — ${TITLE}"
echo -e "  Role:   ${CYAN}${ROLE}${RESET}"
echo -e "  Base:   ${BASE_BRANCH}"
echo -e "  Branch: ${BRANCH}"
echo ""

if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "${YELLOW}DRY RUN${RESET} — would perform:"
    echo "  1. git worktree add .claude/worktrees/issue-${ISSUE} -b ${BRANCH} ${BASE_BRANCH}"
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
    git -C "$PROJECT_DIR" worktree add "$WORKTREE_DIR" -b "$BRANCH" "$BASE_BRANCH" 2>&1 || {
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
PR_TARGET_NOTE=""
if [[ "$BASE_BRANCH" != "main" ]]; then
    PR_TARGET_NOTE="

IMPORTANT: This branch was created from '${BASE_BRANCH}' (not main). When creating a PR, target '${BASE_BRANCH}' as the base branch, not main."
fi

PROMPT="Issue #${ISSUE}: ${TITLE}

${BODY}

Work in this worktree. The branch is '${BRANCH}'.${PR_TARGET_NOTE} Implement the issue, add tests, and ensure the build passes."

# ── Launch agent in the worktree ────────────────────────────────────────────
echo ""
cd "$WORKTREE_DIR"

if [[ "$AUTO" == "true" ]]; then
    # Auto mode: agent works autonomously, writes a report, user reviews.
    REPORT_FILE="$WORKTREE_DIR/AGENT_REPORT.md"

    AUTO_PROMPT="${PROMPT}

IMPORTANT — AUTO MODE INSTRUCTIONS:
1. Implement the issue fully (code, tests, build verification).
2. Do NOT create a PR or push — leave changes as local commits.
3. When done, write AGENT_REPORT.md in the worktree root with this structure:

# Change Report — Issue #${ISSUE}

## Summary
One paragraph describing what was done.

## Files Changed
| File | Change | Reason |
|------|--------|--------|

## Tests Added/Modified
List of test changes with expected behavior.

## Decisions Made
Any non-obvious design decisions and why.

## Risks / Review Attention
Anything the reviewer should look closely at.

## Build & Test Status
Whether build passed, test count before/after, any failures."

    # Resolve model from role
    declare -A _ROLE_MODEL=(
        [tech-lead]="claude-opus-4-6" [feature-perception]="claude-opus-4-6"
        [feature-nav]="claude-opus-4-6" [feature-integration]="claude-opus-4-6"
        [feature-infra-core]="claude-opus-4-6" [feature-infra-platform]="claude-opus-4-6"
        [review-memory-safety]="claude-opus-4-6" [review-concurrency]="claude-opus-4-6"
        [review-fault-recovery]="claude-opus-4-6" [review-security]="claude-opus-4-6"
        [test-unit]="claude-sonnet-4-6" [test-scenario]="claude-sonnet-4-6"
        [ops-github]="claude-haiku-4-5-20251001"
    )
    MODEL="${_ROLE_MODEL[$ROLE]:-claude-opus-4-6}"

    echo -e "${CYAN}Auto mode${RESET} — agent working autonomously..."
    echo -e "  Report: ${REPORT_FILE}"
    echo ""

    # Phase 1: Agent works autonomously
    claude --model "$MODEL" --agent "$ROLE" \
        --permission-mode acceptEdits \
        -p "$AUTO_PROMPT" || true

    # Phase 2: Show report and enter review loop
    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
    echo -e "${BOLD}  Agent work complete — review phase${RESET}"
    echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
    echo ""

    if [[ -f "$REPORT_FILE" ]]; then
        cat "$REPORT_FILE"
    else
        echo -e "${YELLOW}WARN${RESET}  No AGENT_REPORT.md found — showing git diff instead"
        git -C "$WORKTREE_DIR" diff --stat HEAD~1 2>/dev/null || \
            git -C "$WORKTREE_DIR" diff --stat 2>/dev/null || true
    fi

    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
    echo ""

    # Review loop
    while true; do
        echo -e "  ${GREEN}[a]ccept${RESET}  — approve changes, ready for PR"
        echo -e "  ${YELLOW}[c]hanges${RESET} — request changes (opens interactive session)"
        echo -e "  ${RED}[r]eject${RESET}  — discard all changes"
        echo ""
        read -rp "  Your verdict: " VERDICT

        case "$VERDICT" in
            a|accept)
                echo ""
                echo -e "  ${GREEN}Accepted.${RESET} Changes are committed in: ${WORKTREE_DIR}"
                echo -e "  Next steps:"
                echo -e "    cd $WORKTREE_DIR"
                echo -e "    git push -u origin $BRANCH"
                echo -e "    gh pr create --base $BASE_BRANCH"
                exit 0
                ;;
            c|changes)
                echo ""
                echo -e "  ${CYAN}Opening interactive session${RESET} — tell the agent what to change."
                echo -e "  The agent has all its previous context.\n"
                exec claude --model "$MODEL" --agent "$ROLE" --continue
                ;;
            r|reject)
                echo ""
                read -rp "  This will reset all changes. Are you sure? [y/N] " CONFIRM
                if [[ "$CONFIRM" =~ ^[Yy]$ ]]; then
                    git -C "$WORKTREE_DIR" checkout . 2>/dev/null || true
                    git -C "$WORKTREE_DIR" clean -fd 2>/dev/null || true
                    echo -e "  ${RED}Changes discarded.${RESET}"
                    exit 1
                fi
                ;;
            *)
                echo -e "  ${RED}Invalid choice.${RESET} Enter a, c, or r."
                ;;
        esac
    done

elif [[ "$HEADLESS" == "true" ]]; then
    # Headless: non-interactive print mode (no auto-approve — will hang on prompts)
    exec "$PROJECT_DIR/scripts/start-agent.sh" "$ROLE" "$PROMPT" --skip-preflight
else
    # Interactive (default): agent gets issue context, you can approve changes and converse
    exec "$PROJECT_DIR/scripts/start-agent.sh" "$ROLE" "$PROMPT" --skip-preflight --interactive
fi
