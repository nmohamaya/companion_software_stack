#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

AGENT=""
TEAM=false
SINCE=""

usage() {
    echo "Usage: agent-dashboard.sh [--agent <role>] [--team] [--since <date>]"
    echo ""
    echo "Options:"
    echo "  --agent <role>   Per-agent report (e.g., perception, nav, comms, infra)"
    echo "  --team           Aggregate team report"
    echo "  --since <date>   Filter by date (e.g., '2026-03-01', '2 weeks ago')"
    echo ""
    echo "Examples:"
    echo "  agent-dashboard.sh --agent perception"
    echo "  agent-dashboard.sh --team --since '2026-04-01'"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --agent)
            AGENT="$2"
            shift 2
            ;;
        --team)
            TEAM=true
            shift
            ;;
        --since)
            SINCE="$2"
            shift 2
            ;;
        --help|-h)
            usage
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            ;;
        *)
            # Positional arg: treat as --agent <role>
            AGENT="$1"
            shift
            ;;
    esac
done

# Default to --team mode when no arguments given
if [[ -z "$AGENT" && "$TEAM" != true ]]; then
    TEAM=true
fi

CHANGELOG="$PROJECT_DIR/tasks/agent-changelog.md"

agent_report() {
    local role="$1"

    echo -e "${BOLD}=========================================="
    echo -e "  Agent Report: ${CYAN}${role}${RESET}${BOLD}"
    echo -e "==========================================${RESET}"
    echo ""

    # Sessions from changelog
    local sessions=0
    if [[ -f "$CHANGELOG" ]]; then
        sessions=$(grep -ci "$role" "$CHANGELOG" 2>/dev/null || echo 0)
    fi
    echo -e "  ${BOLD}Sessions referenced:${RESET}  $sessions entries in changelog"

    # Commits from git log
    local since_arg=""
    if [[ -n "$SINCE" ]]; then
        since_arg="--since=$SINCE"
    fi

    local commits=0
    if [[ -n "$since_arg" ]]; then
        commits=$(git log "$since_arg" --all --oneline --grep="$role" 2>/dev/null | wc -l || echo 0)
        # Also count from branch names
        local branch_commits
        branch_commits=$(git log "$since_arg" --all --oneline --source 2>/dev/null \
            | grep -i "$role" | wc -l || echo 0)
        commits=$((commits > branch_commits ? commits : branch_commits))
    else
        commits=$(git log --all --oneline --grep="$role" 2>/dev/null | wc -l || echo 0)
    fi
    echo -e "  ${BOLD}Commits:${RESET}              $commits"

    # PRs (via gh if available)
    local prs="N/A"
    if command -v gh &>/dev/null; then
        prs=$(gh pr list --state all --search "$role" --limit 100 --json number 2>/dev/null \
            | grep -c '"number"' || echo 0)
    fi
    echo -e "  ${BOLD}PRs:${RESET}                  $prs"

    # Parse changelog for details
    if [[ -f "$CHANGELOG" ]]; then
        local tests_added safety_issues hallucination_flags
        tests_added=$(grep -i "$role" "$CHANGELOG" 2>/dev/null \
            | grep -oiP 'tests?\s*added[:\s]*\d+|\+\d+\s*tests?' \
            | grep -oP '\d+' | paste -sd+ - | bc 2>/dev/null || echo 0)
        safety_issues=$(grep -i "$role" "$CHANGELOG" 2>/dev/null \
            | grep -ciE 'safety|critical|vulnerability' || echo 0)

        local pass_count warn_count fail_count
        pass_count=$(grep -i "$role" "$CHANGELOG" 2>/dev/null | grep -c 'PASS' || echo 0)
        warn_count=$(grep -i "$role" "$CHANGELOG" 2>/dev/null | grep -c 'WARN' || echo 0)
        fail_count=$(grep -i "$role" "$CHANGELOG" 2>/dev/null | grep -c 'FAIL' || echo 0)

        echo -e "  ${BOLD}Tests added:${RESET}          $tests_added"
        echo -e "  ${BOLD}Safety issues:${RESET}        $safety_issues"
        echo -e "  ${BOLD}Hallucination flags:${RESET}  ${GREEN}PASS=$pass_count${RESET} ${YELLOW}WARN=$warn_count${RESET} ${RED}FAIL=$fail_count${RESET}"
    else
        echo -e "  ${YELLOW}(No agent-changelog.md found at $CHANGELOG)${RESET}"
    fi

    echo ""
}

team_report() {
    echo -e "${BOLD}=========================================="
    echo -e "  Team Dashboard"
    echo -e "==========================================${RESET}"
    echo ""

    # Pipeline throughput: issues closed
    local since_filter=""
    if [[ -n "$SINCE" ]]; then
        since_filter="--search \"closed:>=$SINCE\""
    fi

    if command -v gh &>/dev/null; then
        echo -e "${BOLD}--- Pipeline Throughput ---${RESET}"
        local closed_issues
        if [[ -n "$SINCE" ]]; then
            closed_issues=$(gh issue list --state closed --search "closed:>=$SINCE" --limit 500 --json number 2>/dev/null \
                | grep -c '"number"' || echo 0)
        else
            closed_issues=$(gh issue list --state closed --limit 500 --json number 2>/dev/null \
                | grep -c '"number"' || echo 0)
        fi
        echo -e "  Issues closed:      ${GREEN}${closed_issues}${RESET}"

        local open_issues
        open_issues=$(gh issue list --state open --limit 500 --json number 2>/dev/null \
            | grep -c '"number"' || echo 0)
        echo -e "  Issues open:        ${YELLOW}${open_issues}${RESET}"

        local open_prs
        open_prs=$(gh pr list --state open --limit 100 --json number 2>/dev/null \
            | grep -c '"number"' || echo 0)
        echo -e "  PRs open:           ${CYAN}${open_prs}${RESET}"
        echo ""

        # Epic progress
        echo -e "${BOLD}--- Epic Progress ---${RESET}"
        for epic_num in 284 300; do
            local epic_body
            epic_body=$(gh issue view "$epic_num" --json body --jq '.body' 2>/dev/null || true)
            if [[ -n "$epic_body" ]]; then
                local checked unchecked total
                checked=$(echo "$epic_body" | grep -c '\[x\]' || echo 0)
                unchecked=$(echo "$epic_body" | grep -c '\[ \]' || echo 0)
                total=$((checked + unchecked))
                if [[ "$total" -gt 0 ]]; then
                    local pct=$((checked * 100 / total))
                    echo -e "  Epic #${epic_num}:  ${checked}/${total} tasks (${pct}%)"
                else
                    echo -e "  Epic #${epic_num}:  no checkbox tasks found"
                fi
            else
                echo -e "  Epic #${epic_num}:  ${YELLOW}could not fetch${RESET}"
            fi
        done
        echo ""
    else
        echo -e "${YELLOW}gh CLI not available -- skipping GitHub metrics${RESET}"
        echo ""
    fi

    # Cost breakdown by role from changelog
    if [[ -f "$CHANGELOG" ]]; then
        echo -e "${BOLD}--- Activity by Role (from changelog) ---${RESET}"
        for role in perception nav integration infra-core infra-platform docs; do
            local count
            count=$(grep -ci "$role" "$CHANGELOG" 2>/dev/null || echo 0)
            if [[ "$count" -gt 0 ]]; then
                printf "  %-20s %d entries\n" "$role" "$count"
            fi
        done
        echo ""

        echo -e "${BOLD}--- Safety Issue Trend ---${RESET}"
        local safety_total
        safety_total=$(grep -ciE 'safety|critical|vulnerability' "$CHANGELOG" 2>/dev/null || echo 0)
        if [[ "$safety_total" -gt 0 ]]; then
            echo -e "  Total safety-related entries: ${RED}${safety_total}${RESET}"
        else
            echo -e "  Total safety-related entries: ${GREEN}0${RESET}"
        fi
        echo ""
    else
        echo -e "${YELLOW}No agent-changelog.md found -- skipping changelog metrics${RESET}"
        echo ""
    fi

    # Git commit summary
    echo -e "${BOLD}--- Git Commit Summary ---${RESET}"
    local since_git=""
    if [[ -n "$SINCE" ]]; then
        since_git="--since=$SINCE"
    fi

    local total_commits agent_commits
    if [[ -n "$since_git" ]]; then
        total_commits=$(git log "$since_git" --oneline 2>/dev/null | wc -l)
        agent_commits=$(git log "$since_git" --format="%b" 2>/dev/null \
            | grep -c "Co-Authored-By.*Claude" || echo 0)
    else
        total_commits=$(git log --oneline 2>/dev/null | wc -l)
        agent_commits=$(git log --format="%b" 2>/dev/null \
            | grep -c "Co-Authored-By.*Claude" || echo 0)
    fi

    echo -e "  Total commits:  $total_commits"
    echo -e "  Agent commits:  $agent_commits"
    if [[ "$total_commits" -gt 0 ]]; then
        local ratio=$((agent_commits * 100 / total_commits))
        echo -e "  Agent ratio:    ${ratio}%"
    fi
    echo ""
}

if [[ -n "$AGENT" ]]; then
    agent_report "$AGENT"
fi

if [[ "$TEAM" == true ]]; then
    team_report
fi
