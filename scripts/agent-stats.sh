#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

# Defaults
SINCE_ARG="--since=30 days ago"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)
            SINCE_ARG=""
            shift
            ;;
        --since)
            SINCE_ARG="--since=$2"
            shift 2
            ;;
        *)
            echo "Usage: agent-stats.sh [--all | --since <period>]"
            exit 1
            ;;
    esac
done

# Collect all commits
if [[ -n "$SINCE_ARG" ]]; then
    TOTAL_COMMITS=$(git log "$SINCE_ARG" --oneline 2>/dev/null | wc -l)
else
    TOTAL_COMMITS=$(git log --oneline 2>/dev/null | wc -l)
fi

# Collect agent commits (those with Co-Authored-By: Claude)
if [[ -n "$SINCE_ARG" ]]; then
    AGENT_LOG=$(git log "$SINCE_ARG" --format="%H %s%n%(trailers:key=Co-Authored-By,valueonly)" 2>/dev/null)
else
    AGENT_LOG=$(git log --format="%H %s%n%(trailers:key=Co-Authored-By,valueonly)" 2>/dev/null)
fi

# Parse agent commits into arrays
declare -A MODEL_COUNTS TYPE_COUNTS ROLE_COUNTS
AGENT_COMMIT_COUNT=0

while IFS= read -r line; do
    [[ -z "$line" ]] && continue

    # Lines with a hash at the start are commit lines
    if [[ "$line" =~ ^[0-9a-f]{40}\  ]]; then
        CURRENT_SUBJECT="${line#* }"
        CURRENT_HASH="${line%% *}"
        continue
    fi

    # Co-Authored-By lines
    if [[ "$line" == *"Claude"* ]]; then
        AGENT_COMMIT_COUNT=$((AGENT_COMMIT_COUNT + 1))

        # Extract model
        MODEL="Unknown"
        if [[ "$line" == *"Opus"* ]]; then
            MODEL="Opus"
        elif [[ "$line" == *"Sonnet"* ]]; then
            MODEL="Sonnet"
        elif [[ "$line" == *"Haiku"* ]]; then
            MODEL="Haiku"
        fi
        MODEL_COUNTS[$MODEL]=$(( ${MODEL_COUNTS[$MODEL]:-0} + 1 ))

        # Extract type from commit message prefix
        TYPE="other"
        if [[ "$CURRENT_SUBJECT" =~ ^(feat|fix|refactor|test|docs|chore|perf)\( ]]; then
            TYPE="${BASH_REMATCH[1]}"
        elif [[ "$CURRENT_SUBJECT" =~ ^(feat|fix|refactor|test|docs|chore|perf): ]]; then
            TYPE="${BASH_REMATCH[1]}"
        fi
        TYPE_COUNTS[$TYPE]=$(( ${TYPE_COUNTS[$TYPE]:-0} + 1 ))

        # Extract role from branch name via commit hash
        BRANCH_NAME=$(git branch --contains "$CURRENT_HASH" --format='%(refname:short)' 2>/dev/null | head -1)
        ROLE="unknown"
        case "$BRANCH_NAME" in
            *perception*|*camera*|*detector*)  ROLE="perception" ;;
            *nav*|*mission*|*slam*|*planner*)  ROLE="nav" ;;
            *comms*|*monitor*|*integration*)   ROLE="integration" ;;
            *infra*|*common*|*modularity*)     ROLE="infra-core" ;;
            *deploy*|*ci*|*platform*)          ROLE="infra-platform" ;;
            *docs*|*doc*)                      ROLE="docs" ;;
            *fix*|*bug*)                       ROLE="bugfix" ;;
        esac
        ROLE_COUNTS[$ROLE]=$(( ${ROLE_COUNTS[$ROLE]:-0} + 1 ))
    fi
done <<< "$AGENT_LOG"

# Print results
echo "=========================================="
echo "  Agent Commit Statistics"
if [[ -n "$SINCE_ARG" ]]; then
    echo "  Period: ${SINCE_ARG#--since=}"
else
    echo "  Period: all time"
fi
echo "=========================================="
echo ""

echo "--- By Model ---"
for model in "${!MODEL_COUNTS[@]}"; do
    printf "  %-12s %d\n" "$model" "${MODEL_COUNTS[$model]}"
done | sort -t' ' -k2 -rn
echo ""

echo "--- By Type ---"
for typ in "${!TYPE_COUNTS[@]}"; do
    printf "  %-12s %d\n" "$typ" "${TYPE_COUNTS[$typ]}"
done | sort -t' ' -k2 -rn
echo ""

echo "--- By Role ---"
for role in "${!ROLE_COUNTS[@]}"; do
    printf "  %-16s %d\n" "$role" "${ROLE_COUNTS[$role]}"
done | sort -t' ' -k2 -rn
echo ""

echo "--- Summary ---"
printf "  Agent commits:  %d / %d total  " "$AGENT_COMMIT_COUNT" "$TOTAL_COMMITS"
if [[ "$TOTAL_COMMITS" -gt 0 ]]; then
    PCT=$(( AGENT_COMMIT_COUNT * 100 / TOTAL_COMMITS ))
    echo "(${PCT}%)"
else
    echo "(N/A)"
fi
