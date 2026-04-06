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
  --pipeline        Full guided pipeline: agent work → validation → PR → review
                    agents → fix → push. 5 human checkpoints, rest automated.
  --headless        Non-interactive print mode (for CI with pre-configured perms)
  --dry-run         Print routing decision without executing
  --help, -h        Show this help

Modes:
  (default)    Interactive — you approve changes and converse in real time
  --auto       Autonomous — agent works, writes report, you review afterward
  --pipeline   End-to-end guided pipeline with 5 approval checkpoints
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
PIPELINE=false
BASE_BRANCH="main"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)   DRY_RUN=true; shift ;;
        --headless)  HEADLESS=true; shift ;;
        --auto)      AUTO=true; shift ;;
        --pipeline)  PIPELINE=true; shift ;;
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

# Validate mutually exclusive modes
MODE_COUNT=0
[[ "$AUTO" == "true" ]] && MODE_COUNT=$((MODE_COUNT + 1))
[[ "$HEADLESS" == "true" ]] && MODE_COUNT=$((MODE_COUNT + 1))
[[ "$PIPELINE" == "true" ]] && MODE_COUNT=$((MODE_COUNT + 1))
if [[ $MODE_COUNT -gt 1 ]]; then
    echo -e "${RED}Error: --auto, --headless, and --pipeline are mutually exclusive${RESET}" >&2
    exit 1
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

MODE_LABEL="interactive"
[[ "$AUTO" == "true" ]] && MODE_LABEL="auto"
[[ "$HEADLESS" == "true" ]] && MODE_LABEL="headless"
[[ "$PIPELINE" == "true" ]] && MODE_LABEL="pipeline"
echo -e "  Mode:   ${MODE_LABEL}"
echo ""

if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "${YELLOW}DRY RUN${RESET} — would perform:"
    echo "  1. git worktree add .claude/worktrees/issue-${ISSUE} -b ${BRANCH} ${BASE_BRANCH}"
    echo "  2. Update tasks/active-work.md"
    if [[ "$PIPELINE" == "true" ]]; then
        echo "  3. Agent works autonomously (writes AGENT_REPORT.md)"
        echo "  4. CP1: Review changes (accept / request changes / reject)"
        echo "  5. validate-session.sh (build, tests, hallucination detection)"
        echo "  6. CP2: Commit approval (commit / back / abort)"
        echo "  7. git push + gh pr create --base ${BASE_BRANCH}"
        echo "  8. CP3: PR preview (create / edit / back / abort)"
        echo "  9. deploy-review.sh (parallel review agents)"
        echo "  10. CP4: Review findings (accept / fix / back / reject)"
        echo "  11. Feature agent fixes P1/P2 findings"
        echo "  12. CP5: Final push (push / re-review / back / abort)"
        echo "  13. cleanup-branches.sh"
    else
        echo "  3. Launch: start-agent.sh ${ROLE} <issue prompt> (mode: ${MODE_LABEL})"
    fi
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
                echo -e "    gh pr create --base $BASE_BRANCH --body \"Closes #${ISSUE}\""
                echo ""
                echo -e "  ${YELLOW}IMPORTANT:${RESET} PR body MUST include 'Closes #${ISSUE}' to link the issue."
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

elif [[ "$PIPELINE" == "true" ]]; then
    # ── Pipeline mode ──────────────────────────────────────────────────────
    # Full guided pipeline: agent work → validation → PR → review → fix → push
    # 5 human checkpoints, everything else automated.

    trap 'echo -e "\n${YELLOW}Pipeline interrupted.${RESET} Worktree preserved at: $WORKTREE_DIR"; exit 130' INT TERM

    # ── Helper functions ───────────────────────────────────────────────────
    pipeline_header() {
        echo ""
        echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
        echo -e "${BOLD}  $1${RESET}"
        echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
        echo ""
    }

    pipeline_options() {
        # Print colored option list. Args: pairs of "key" "description"
        while [[ $# -ge 2 ]]; do
            local key="$1" desc="$2"; shift 2
            echo -e "  ${CYAN}[${key}]${RESET} ${desc}"
        done
        echo ""
    }

    # ── Role → Model mapping ───────────────────────────────────────────────
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

    REPORT_FILE="$WORKTREE_DIR/AGENT_REPORT.md"
    VALIDATION_OUTPUT=""
    VALIDATION_EXIT=0
    PR_NUMBER=""
    PR_URL=""
    PR_TITLE=""
    PR_BODY=""
    REVIEW_FINDINGS=""

    AUTO_PROMPT="${PROMPT}

IMPORTANT — PIPELINE MODE INSTRUCTIONS:
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

    # ── State machine ──────────────────────────────────────────────────────
    STATE="AGENT_WORK"

    while true; do
        case "$STATE" in

        # ── AGENT_WORK: Agent works autonomously ──────────────────────────
        AGENT_WORK)
            pipeline_header "Phase 1/5 — Agent Working (${ROLE})"
            echo -e "  Model:  ${MODEL}"
            echo -e "  Issue:  #${ISSUE} — ${TITLE}"
            echo -e "  Mode:   autonomous (writes AGENT_REPORT.md)"
            echo ""

            claude --model "$MODEL" --agent "$ROLE" \
                --permission-mode acceptEdits \
                -p "$AUTO_PROMPT" || true

            STATE="CP1"
            ;;

        # ── CP1: Changes Review ───────────────────────────────────────────
        CP1)
            pipeline_header "CHECKPOINT 1/5 — Changes Review"

            if [[ -f "$REPORT_FILE" ]]; then
                cat "$REPORT_FILE"
            else
                echo -e "${YELLOW}WARN${RESET}  No AGENT_REPORT.md — showing diff instead"
                git diff --stat HEAD 2>/dev/null || true
            fi

            echo ""
            echo -e "${BOLD}--- Diff Summary ---${RESET}"
            git diff --stat "${BASE_BRANCH}...HEAD" 2>/dev/null || \
                git diff --stat 2>/dev/null || true
            echo ""

            pipeline_options \
                "a" "accept — proceed to validation" \
                "c" "changes — open interactive session to request modifications" \
                "r" "reject — discard all changes and abort"

            read -rp "  Your choice: " CHOICE
            case "$CHOICE" in
                a|accept)  STATE="VALIDATE" ;;
                c|changes)
                    echo ""
                    echo -e "  ${CYAN}Opening interactive session...${RESET}"
                    echo -e "  Tell the agent what to change. Exit when done (Ctrl+C or /exit).\n"
                    claude --model "$MODEL" --agent "$ROLE" --continue || true
                    STATE="CP1"  # loop back to review updated changes
                    ;;
                r|reject)
                    read -rp "  Discard all changes? [y/N] " CONFIRM
                    if [[ "$CONFIRM" =~ ^[yY]$ ]]; then
                        STATE="ABORT"
                    fi
                    ;;
                *) echo -e "  ${RED}Invalid choice.${RESET}" ;;
            esac
            ;;

        # ── VALIDATE: Run hallucination detection ─────────────────────────
        VALIDATE)
            pipeline_header "Running Validation (validate-session.sh)"

            VALIDATION_OUTPUT="$("$PROJECT_DIR/scripts/validate-session.sh" --branch "$BRANCH" 2>&1)" || true
            VALIDATION_EXIT=$?
            echo "$VALIDATION_OUTPUT"

            STATE="CP2"
            ;;

        # ── CP2: Commit Approval ──────────────────────────────────────────
        CP2)
            pipeline_header "CHECKPOINT 2/5 — Commit Approval"

            echo -e "${BOLD}--- Validation Results ---${RESET}"
            echo "$VALIDATION_OUTPUT"
            echo ""

            if [[ $VALIDATION_EXIT -ne 0 ]]; then
                echo -e "  ${RED}WARNING: Validation had failures.${RESET} You may still commit (your judgment)."
            else
                echo -e "  ${GREEN}Validation passed.${RESET}"
            fi
            echo ""

            pipeline_options \
                "c" "commit — stage all changes and commit" \
                "b" "back — return to CP1 (changes review)" \
                "a" "abort — exit pipeline"

            read -rp "  Your choice: " CHOICE
            case "$CHOICE" in
                c|commit)
                    echo ""
                    echo -e "  Committing changes..."
                    git add -A
                    git commit -m "$(cat <<COMMITEOF
feat(#${ISSUE}): ${TITLE}

Pipeline commit for issue #${ISSUE}.
Agent role: ${ROLE}

Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
COMMITEOF
)" || {
                        echo -e "  ${YELLOW}WARN${RESET}  Nothing to commit (changes may already be committed)"
                    }
                    echo -e "  ${GREEN}DONE${RESET}"
                    STATE="PR_CREATE"
                    ;;
                b|back)  STATE="CP1" ;;
                a|abort) STATE="ABORT" ;;
                *) echo -e "  ${RED}Invalid choice.${RESET}" ;;
            esac
            ;;

        # ── PR_CREATE: Push and generate PR ───────────────────────────────
        PR_CREATE)
            pipeline_header "Pushing and Preparing PR"

            echo -e "  Pushing to origin/${BRANCH}..."
            git push -u origin "$BRANCH" 2>&1 || true
            echo -e "  ${GREEN}DONE${RESET}"
            echo ""

            # Generate PR title and body from AGENT_REPORT.md
            if [[ -z "$PR_TITLE" ]]; then
                PR_TITLE="feat(#${ISSUE}): ${TITLE}"
            fi
            if [[ -z "$PR_BODY" ]]; then
                summary_text=""
                if [[ -f "$REPORT_FILE" ]]; then
                    summary_text="$(sed -n '/^## Summary/,/^## /p' "$REPORT_FILE" | head -n -1 | tail -n +2 || true)"
                fi
                PR_BODY="## Summary
${summary_text:-See AGENT_REPORT.md for details.}

## Changes
$(git diff --stat "${BASE_BRANCH}...HEAD" 2>/dev/null || true)

## Test plan
- [ ] Build passes with zero warnings
- [ ] All tests pass (verify count matches baseline)
- [ ] Review agent findings addressed

Closes #${ISSUE}

🤖 Generated with [Claude Code](https://claude.com/claude-code) via pipeline mode"
            fi

            STATE="CP3"
            ;;

        # ── CP3: PR Preview ───────────────────────────────────────────────
        CP3)
            pipeline_header "CHECKPOINT 3/5 — PR Preview"

            echo -e "${BOLD}Title:${RESET} ${PR_TITLE}"
            echo ""
            echo -e "${BOLD}Body:${RESET}"
            echo "$PR_BODY"
            echo ""

            if [[ -n "$PR_NUMBER" ]]; then
                echo -e "  ${CYAN}PR #${PR_NUMBER} already exists.${RESET}"
                pipeline_options \
                    "u" "update — edit PR title/body via gh pr edit" \
                    "s" "skip — proceed to review agents" \
                    "b" "back — return to CP2" \
                    "a" "abort — exit pipeline"
            else
                pipeline_options \
                    "c" "create — create the PR on GitHub" \
                    "e" "edit — modify title or body before creating" \
                    "b" "back — return to CP2" \
                    "a" "abort — exit pipeline"
            fi

            read -rp "  Your choice: " CHOICE
            case "$CHOICE" in
                c|create)
                    echo ""
                    echo -e "  Creating PR..."
                    PR_URL="$(gh pr create --base "$BASE_BRANCH" \
                        --title "$PR_TITLE" \
                        --body "$PR_BODY" 2>&1)" || {
                        echo -e "  ${RED}Failed to create PR${RESET}"
                        echo "  $PR_URL"
                        STATE="CP3"
                        continue
                    }
                    PR_NUMBER="$(echo "$PR_URL" | grep -oP '/pull/\K[0-9]+' || true)"
                    echo -e "  ${GREEN}DONE${RESET}  ${PR_URL}"
                    STATE="REVIEW"
                    ;;
                u|update)
                    if [[ -n "$PR_NUMBER" ]]; then
                        read -rp "  New title (Enter to keep): " NEW_TITLE
                        [[ -n "$NEW_TITLE" ]] && PR_TITLE="$NEW_TITLE"
                        echo -e "  Updating PR #${PR_NUMBER}..."
                        gh pr edit "$PR_NUMBER" --title "$PR_TITLE" --body "$PR_BODY" 2>&1 | grep -v "Projects (classic)" || true
                        echo -e "  ${GREEN}DONE${RESET}"
                        STATE="REVIEW"
                    fi
                    ;;
                e|edit)
                    read -rp "  New title (Enter to keep '${PR_TITLE}'): " NEW_TITLE
                    [[ -n "$NEW_TITLE" ]] && PR_TITLE="$NEW_TITLE"
                    echo -e "  (Body will use AGENT_REPORT.md summary — edit on GitHub after creation if needed)"
                    STATE="CP3"  # re-show preview
                    ;;
                s|skip) STATE="REVIEW" ;;
                b|back) STATE="CP2" ;;
                a|abort) STATE="ABORT" ;;
                *) echo -e "  ${RED}Invalid choice.${RESET}" ;;
            esac
            ;;

        # ── REVIEW: Deploy review agents ──────────────────────────────────
        REVIEW)
            pipeline_header "Deploying Review Agents (deploy-review.sh)"

            if [[ -z "$PR_NUMBER" ]]; then
                echo -e "  ${RED}No PR number — skipping review agents.${RESET}"
                STATE="CP5"
                continue
            fi

            echo -e "  Launching review agents for PR #${PR_NUMBER}..."
            echo ""
            "$PROJECT_DIR/scripts/deploy-review.sh" "$PR_NUMBER" 2>&1 || true
            echo ""

            # Fetch the consolidated review comment
            REVIEW_FINDINGS="$(gh api "repos/{owner}/{repo}/issues/${PR_NUMBER}/comments" \
                --jq 'map(select(.body | test("Automated Safety Review"))) | last | .body' 2>/dev/null || true)"

            STATE="CP4"
            ;;

        # ── CP4: Review Findings ──────────────────────────────────────────
        CP4)
            pipeline_header "CHECKPOINT 4/5 — Safety Review Findings"

            if [[ -n "$REVIEW_FINDINGS" ]]; then
                echo "$REVIEW_FINDINGS"
            else
                echo -e "  ${YELLOW}No review findings found.${RESET} Check PR #${PR_NUMBER} comments manually."
            fi
            echo ""

            pipeline_options \
                "a" "accept as-is — no fixes needed, proceed to final push" \
                "f" "fix — feed findings to feature agent for fixing" \
                "b" "back — return to PR preview (CP3)" \
                "r" "reject — abort pipeline"

            read -rp "  Your choice: " CHOICE
            case "$CHOICE" in
                a|accept) STATE="CP5" ;;
                f|fix)    STATE="FIX_AND_REVALIDATE" ;;
                b|back)   STATE="CP3" ;;
                r|reject) STATE="ABORT" ;;
                *) echo -e "  ${RED}Invalid choice.${RESET}" ;;
            esac
            ;;

        # ── FIX_AND_REVALIDATE: Agent fixes review findings ──────────────
        FIX_AND_REVALIDATE)
            pipeline_header "Fixing Review Findings"

            # Extract P1/P2 findings for the fix prompt
            FIX_PROMPT="The review agents found these issues on PR #${PR_NUMBER}:

${REVIEW_FINDINGS}

Fix all P1 (critical) and P2 (high) findings. Address P3 items where reasonable.
Do NOT create a PR or push — just fix the code and commit."

            echo -e "  Launching feature agent to fix findings..."
            echo ""
            claude --model "$MODEL" --agent "$ROLE" \
                --permission-mode acceptEdits \
                --continue \
                -p "$FIX_PROMPT" || true

            echo ""
            echo -e "  ${BOLD}Re-validating after fixes...${RESET}"
            VALIDATION_OUTPUT="$("$PROJECT_DIR/scripts/validate-session.sh" --branch "$BRANCH" 2>&1)" || true
            VALIDATION_EXIT=$?
            echo "$VALIDATION_OUTPUT"

            # Commit and push fixes
            echo ""
            echo -e "  Committing fixes..."
            git add -A
            git commit -m "$(cat <<FIXEOF
fix(#${ISSUE}): address review findings

Pipeline auto-fix for PR #${PR_NUMBER} review comments.

Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
FIXEOF
)" || echo -e "  ${YELLOW}WARN${RESET}  Nothing new to commit"

            echo -e "  Pushing fixes..."
            git push 2>&1 || true
            echo -e "  ${GREEN}DONE${RESET}"

            STATE="CP5"
            ;;

        # ── CP5: Final Push ───────────────────────────────────────────────
        CP5)
            pipeline_header "CHECKPOINT 5/5 — Final Summary"

            echo -e "${BOLD}--- Commit History ---${RESET}"
            git log --oneline "${BASE_BRANCH}..HEAD" 2>/dev/null || \
                git log --oneline -5
            echo ""

            if [[ $VALIDATION_EXIT -ne 0 ]]; then
                echo -e "  ${YELLOW}Last validation had warnings/failures.${RESET}"
            else
                echo -e "  ${GREEN}Last validation passed.${RESET}"
            fi

            if [[ -n "$PR_URL" ]]; then
                echo -e "  PR: ${PR_URL}"
            fi
            echo ""

            pipeline_options \
                "d" "done — finalize and clean up" \
                "r" "re-review — run review agents again" \
                "b" "back — return to review findings (CP4)" \
                "a" "abort — exit pipeline"

            read -rp "  Your choice: " CHOICE
            case "$CHOICE" in
                d|done)
                    # Ensure everything is pushed
                    git push 2>/dev/null || true
                    STATE="CLEANUP"
                    ;;
                r|re-review) STATE="REVIEW" ;;
                b|back)      STATE="CP4" ;;
                a|abort)     STATE="ABORT" ;;
                *) echo -e "  ${RED}Invalid choice.${RESET}" ;;
            esac
            ;;

        # ── CLEANUP: Final cleanup ────────────────────────────────────────
        CLEANUP)
            pipeline_header "Pipeline Complete"

            # Verify PR links to issue (enforce issue↔PR association)
            if [[ -n "$PR_NUMBER" ]]; then
                PR_BODY_CHECK="$(gh pr view "$PR_NUMBER" --json body --jq '.body' 2>/dev/null || true)"
                if echo "$PR_BODY_CHECK" | grep -qiE "(closes|fixes|resolves) #${ISSUE}"; then
                    echo -e "  ${GREEN}VERIFIED${RESET}  PR #${PR_NUMBER} links to issue #${ISSUE}"
                else
                    echo -e "  ${RED}WARNING${RESET}  PR #${PR_NUMBER} does NOT link issue #${ISSUE}!"
                    echo -e "  Adding 'Closes #${ISSUE}' to PR body..."
                    UPDATED_BODY="${PR_BODY_CHECK}

Closes #${ISSUE}"
                    gh pr edit "$PR_NUMBER" --body "$UPDATED_BODY" 2>&1 | grep -v "Projects (classic)" || true
                    echo -e "  ${GREEN}FIXED${RESET}  PR body updated with issue link."
                fi
            fi
            echo ""

            echo -e "  ${GREEN}Issue:${RESET}    #${ISSUE} — ${TITLE}"
            echo -e "  ${GREEN}Branch:${RESET}   ${BRANCH}"
            [[ -n "$PR_URL" ]] && echo -e "  ${GREEN}PR:${RESET}       ${PR_URL}"
            echo -e "  ${GREEN}Role:${RESET}     ${ROLE}"
            echo -e "  ${GREEN}Model:${RESET}    ${MODEL}"
            echo ""
            echo -e "  ${BOLD}Next step:${RESET} Review and merge the PR in GitHub UI."
            echo ""

            # Offer cleanup (don't force it — user may want the worktree)
            read -rp "  Clean up merged branches/worktrees? [y/N] " CLEANUP_CHOICE
            if [[ "$CLEANUP_CHOICE" =~ ^[yY]$ ]]; then
                "$PROJECT_DIR/scripts/cleanup-branches.sh" --yes 2>&1 || true
            fi

            exit 0
            ;;

        # ── ABORT: Exit pipeline ──────────────────────────────────────────
        ABORT)
            pipeline_header "Pipeline Aborted"

            echo -e "  Worktree preserved at: ${WORKTREE_DIR}"
            echo -e "  Branch: ${BRANCH}"
            [[ -n "$PR_URL" ]] && echo -e "  PR: ${PR_URL} (still open)"
            echo ""
            echo -e "  You can resume manually:"
            echo -e "    cd $WORKTREE_DIR"
            echo -e "    bash $PROJECT_DIR/scripts/start-agent.sh $ROLE --interactive"
            exit 1
            ;;

        *)
            echo -e "${RED}BUG: Unknown state '$STATE'${RESET}" >&2
            exit 2
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
