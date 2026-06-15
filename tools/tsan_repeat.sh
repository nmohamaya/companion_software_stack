#!/usr/bin/env bash
# tools/tsan_repeat.sh — Build + repeatedly run test targets under ThreadSanitizer.
#
# Why: data races are probabilistic. A single run — including CI's one TSan
# leg — can miss them. For signal-handler / atomic / lock-free changes the
# house rule (ADR-016 "Tool-First Verification", tasks/lessons.md) is a
# dedicated TSan build in a SEPARATE dir + N repeated runs of the affected
# targets. This script automates that (it is Tier-0 tooling from ADR-016).
#
# Usage:
#   tools/tsan_repeat.sh [-n RUNS] [-d BUILD_DIR] [-j JOBS] TARGET [TARGET ...]
#
# Example:
#   tools/tsan_repeat.sh -n 5 test_stack_trace_capture test_planner_stall_handler
#
# Exit: non-zero on the first run that reports a race/failure (the run uses
# TSAN_OPTIONS=halt_on_error=1). BUILD_DIR defaults to build-tsan/, kept
# separate from build/ so Release and TSan objects never mix (tasks/lessons.md).
set -uo pipefail

RUNS=5
BUILD_DIR="build-tsan"
JOBS="$(nproc 2>/dev/null || echo 4)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

while getopts "n:d:j:h" opt; do
    case "$opt" in
        n) RUNS="$OPTARG" ;;
        d) BUILD_DIR="$OPTARG" ;;
        j) JOBS="$OPTARG" ;;
        h) sed -n '2,21p' "$0"; exit 0 ;;
        *) exit 2 ;;
    esac
done
shift $((OPTIND - 1))

if [ "$#" -eq 0 ]; then
    echo "error: need at least one test target (run with -h for usage)" >&2
    exit 2
fi

cd "$ROOT"

echo "== Configuring TSan build in $BUILD_DIR =="
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Debug -DENABLE_TSAN=ON -DALLOW_INSECURE_ZENOH=ON >/dev/null

echo "== Building targets: $* =="
cmake --build "$BUILD_DIR" --target "$@" -j"$JOBS" >/dev/null

fail=0
for t in "$@"; do
    bin="$BUILD_DIR/bin/$t"
    if [ ! -x "$bin" ]; then
        echo "MISSING: $bin (is '$t' a test target?)" >&2
        fail=1
        continue
    fi
    ok=0
    for i in $(seq 1 "$RUNS"); do
        log="/tmp/tsan_${t}_${i}.log"
        if TSAN_OPTIONS="halt_on_error=1 history_size=4" "$bin" >"$log" 2>&1; then
            ok=$((ok + 1))
        else
            echo "FAIL: $t — run $i/$RUNS"
            grep -E "ThreadSanitizer|data race|WARNING|FATAL|FAILED" "$log" | head -20
            fail=1
            break
        fi
    done
    [ "$ok" -eq "$RUNS" ] && echo "PASS: $t — $RUNS/$RUNS TSan-clean"
done

if [ "$fail" -eq 0 ]; then
    echo "ALL CLEAN"
else
    echo "RACES/FAILURES PRESENT" >&2
    exit 1
fi
