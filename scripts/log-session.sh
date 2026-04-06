#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -lt 2 ]]; then
    echo "Usage: log-session.sh <role> <command...>"
    echo "  Captures session transcript to tasks/sessions/"
    exit 1
fi

ROLE="$1"
shift

SESSION_DIR="$PROJECT_DIR/tasks/sessions"
mkdir -p "$SESSION_DIR"

LOGFILE="$SESSION_DIR/$(date +%Y-%m-%d-%H%M)-${ROLE}.log"

echo "=== Session: role=$ROLE date=$(date -Iseconds) ===" | tee "$LOGFILE"
echo "=== Command: $* ===" | tee -a "$LOGFILE"
echo "---" | tee -a "$LOGFILE"

"$@" 2>&1 | tee -a "$LOGFILE"
EXIT_CODE=${PIPESTATUS[0]}

echo "---" | tee -a "$LOGFILE"
echo "=== Session ended: $(date -Iseconds) exit=$EXIT_CODE ===" | tee -a "$LOGFILE"
echo "Log file: $LOGFILE"

exit "$EXIT_CODE"
