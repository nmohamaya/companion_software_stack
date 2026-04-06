#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_DIR="$PROJECT_DIR/tasks/sessions"

SUMMARY=false
LOGFILE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --summary)
            SUMMARY=true
            shift
            ;;
        *)
            LOGFILE="$1"
            shift
            ;;
    esac
done

parse_logfile() {
    local file="$1"
    local basename
    basename=$(basename "$file")

    # Extract role from filename: YYYY-MM-DD-HHMM-<role>.log
    local role="unknown"
    if [[ "$basename" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{4}-(.+)\.log$ ]]; then
        role="${BASH_REMATCH[1]}"
    fi

    # Extract date from filename
    local session_date="unknown"
    if [[ "$basename" =~ ^([0-9]{4}-[0-9]{2}-[0-9]{2}) ]]; then
        session_date="${BASH_REMATCH[1]}"
    fi

    # Duration: use file modification time - creation time approximation
    local first_ts last_ts duration="unknown"
    first_ts=$(head -1 "$file" 2>/dev/null | grep -oP '\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}' || true)
    last_ts=$(tail -1 "$file" 2>/dev/null | grep -oP '\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}' || true)
    if [[ -n "$first_ts" && -n "$last_ts" ]]; then
        local start_epoch end_epoch
        start_epoch=$(date -d "$first_ts" +%s 2>/dev/null || echo 0)
        end_epoch=$(date -d "$last_ts" +%s 2>/dev/null || echo 0)
        if [[ "$start_epoch" -gt 0 && "$end_epoch" -gt 0 ]]; then
            local diff=$((end_epoch - start_epoch))
            duration="$((diff / 60))m $((diff % 60))s"
        fi
    fi
    if [[ "$duration" == "unknown" ]]; then
        # Fallback to file timestamps
        local mod_time create_time
        mod_time=$(stat -c %Y "$file" 2>/dev/null || echo 0)
        create_time=$(stat -c %W "$file" 2>/dev/null || echo 0)
        if [[ "$create_time" -gt 0 && "$mod_time" -gt 0 && "$mod_time" -ge "$create_time" ]]; then
            local diff=$((mod_time - create_time))
            duration="$((diff / 60))m $((diff % 60))s"
        fi
    fi

    # Count metrics
    local files_changed test_runs errors
    files_changed=$(grep -ciE 'Edit|Write|created|modified' "$file" 2>/dev/null || echo 0)
    test_runs=$(grep -ciE 'ctest|test.*pass|test.*fail' "$file" 2>/dev/null || echo 0)
    errors=$(grep -ciE 'error|Error|ERROR|failed|FAIL' "$file" 2>/dev/null || echo 0)

    # Tool call counts
    local tool_read tool_edit tool_write tool_bash tool_grep tool_glob
    tool_read=$(grep -c 'Read' "$file" 2>/dev/null || echo 0)
    tool_edit=$(grep -c 'Edit' "$file" 2>/dev/null || echo 0)
    tool_write=$(grep -c 'Write' "$file" 2>/dev/null || echo 0)
    tool_bash=$(grep -c 'Bash' "$file" 2>/dev/null || echo 0)
    tool_grep=$(grep -c 'Grep' "$file" 2>/dev/null || echo 0)
    tool_glob=$(grep -c 'Glob' "$file" 2>/dev/null || echo 0)

    printf "%-30s  %-14s  %-10s  %-8s\n" "File" "Role" "Duration" "Date"
    printf "%-30s  %-14s  %-10s  %-8s\n" "$(echo "$basename" | cut -c1-30)" "$role" "$duration" "$session_date"
    echo ""
    echo "  Files changed:  $files_changed"
    echo "  Test runs:      $test_runs"
    echo "  Errors:         $errors"
    echo "  Tool calls:     Read=$tool_read Edit=$tool_edit Write=$tool_write Bash=$tool_bash Grep=$tool_grep Glob=$tool_glob"
    echo ""
}

if [[ "$SUMMARY" == true ]]; then
    echo "=========================================="
    echo "  Session Metrics Summary"
    echo "=========================================="
    echo ""

    if [[ ! -d "$SESSION_DIR" ]]; then
        echo "No sessions directory found at $SESSION_DIR"
        exit 0
    fi

    LOG_COUNT=0
    for f in "$SESSION_DIR"/*.log; do
        [[ -f "$f" ]] || continue
        LOG_COUNT=$((LOG_COUNT + 1))
        parse_logfile "$f"
        echo "---"
    done

    if [[ "$LOG_COUNT" -eq 0 ]]; then
        echo "No session logs found in $SESSION_DIR"
    else
        echo "Total sessions: $LOG_COUNT"
    fi
elif [[ -n "$LOGFILE" ]]; then
    if [[ ! -f "$LOGFILE" ]]; then
        echo "File not found: $LOGFILE"
        exit 1
    fi
    parse_logfile "$LOGFILE"
else
    echo "Usage: parse-session-metrics.sh [<logfile>] [--summary]"
    echo ""
    echo "  <logfile>   Parse a specific session log file"
    echo "  --summary   Parse all logs in tasks/sessions/"
    exit 1
fi
