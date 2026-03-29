#!/usr/bin/env bash
# tests/cleanup_old_runs.sh
# ═══════════════════════════════════════════════════════════════
# Manual retention cleanup for timestamped scenario run directories.
#
# Deletes run directories older than N days.  NEVER runs automatically
# — manual invocation only.
#
# Usage:
#   ./tests/cleanup_old_runs.sh                    # List runs >30 days old
#   ./tests/cleanup_old_runs.sh --days 7           # List runs >7 days old
#   ./tests/cleanup_old_runs.sh --delete           # Delete runs >30 days old
#   ./tests/cleanup_old_runs.sh --days 14 --delete # Delete runs >14 days old
#
# Options:
#   --days <N>     Retention period in days (default: 30)
#   --delete       Actually delete (default: dry-run / list only)
#   --log-dir <p>  Base log directory (default: drone_logs/)
#
# Exit codes:
#   0  Success
#   1  Error
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

RETENTION_DAYS=30
DO_DELETE=false
LOG_DIR="${PROJECT_DIR}/drone_logs"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --days)     RETENTION_DAYS="$2"; shift ;;
        --delete)   DO_DELETE=true ;;
        --log-dir)  LOG_DIR="$2"; shift ;;
        -h|--help)
            echo "Usage: $0 [--days N] [--delete] [--log-dir <path>]"
            echo ""
            echo "Lists (or deletes) scenario run directories older than N days."
            echo "Default: 30 days, dry-run (list only)."
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1"
            exit 1
            ;;
    esac
    shift
done

CUTOFF_EPOCH=$(date -d "${RETENTION_DAYS} days ago" +%s 2>/dev/null || \
               date -v-${RETENTION_DAYS}d +%s 2>/dev/null || echo "0")

if [[ "$CUTOFF_EPOCH" == "0" ]]; then
    echo "ERROR: Could not compute cutoff date"
    exit 1
fi

CUTOFF_DATE=$(date -d "@${CUTOFF_EPOCH}" +"%Y-%m-%d" 2>/dev/null || \
              date -r "${CUTOFF_EPOCH}" +"%Y-%m-%d" 2>/dev/null || echo "unknown")

echo "Retention policy : keep runs newer than ${RETENTION_DAYS} days (cutoff: ${CUTOFF_DATE})"
echo "Log directory    : ${LOG_DIR}"
echo "Mode             : $(if $DO_DELETE; then echo 'DELETE'; else echo 'DRY RUN (list only)'; fi)"
echo ""

TOTAL=0
TOTAL_SIZE=0
DELETED=0

# Search for timestamped run directories: YYYY-MM-DD_HHMMSS_*
for runner_dir in "${LOG_DIR}/scenarios" "${LOG_DIR}/scenarios_gazebo"; do
    [[ -d "$runner_dir" ]] || continue

    for scenario_dir in "${runner_dir}"/*/; do
        [[ -d "$scenario_dir" ]] || continue
        scenario_name=$(basename "$scenario_dir")

        for run_dir in "${scenario_dir}"????-??-??_*; do
            [[ -d "$run_dir" ]] || continue
            # Skip the 'latest' symlink
            [[ -L "$run_dir" ]] && continue

            run_basename=$(basename "$run_dir")
            # Extract date from directory name: YYYY-MM-DD
            run_date=$(echo "$run_basename" | grep -oP '^\d{4}-\d{2}-\d{2}' || continue)

            run_epoch=$(date -d "$run_date" +%s 2>/dev/null || \
                        date -j -f "%Y-%m-%d" "$run_date" +%s 2>/dev/null || echo "0")

            if [[ "$run_epoch" -gt 0 && "$run_epoch" -lt "$CUTOFF_EPOCH" ]]; then
                TOTAL=$((TOTAL + 1))
                run_size=$(du -sh "$run_dir" 2>/dev/null | cut -f1 || echo "?")

                if $DO_DELETE; then
                    echo "  DELETE: ${run_dir} (${run_size})"
                    rm -rf "$run_dir"
                    DELETED=$((DELETED + 1))
                else
                    echo "  OLD: ${run_dir} (${run_size})"
                fi
            fi
        done
    done
done

echo ""
if $DO_DELETE; then
    echo "Deleted ${DELETED} run(s) older than ${RETENTION_DAYS} days."
else
    echo "Found ${TOTAL} run(s) older than ${RETENTION_DAYS} days."
    if [[ $TOTAL -gt 0 ]]; then
        echo "Run with --delete to remove them."
    fi
fi
