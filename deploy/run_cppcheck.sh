#!/usr/bin/env bash
# deploy/run_cppcheck.sh — static analysis via cppcheck (ADR-016 Tier-0).
#
# Complements clang-tidy with a second engine (cppcheck finds a different
# class of issues: uninitialised members, dead branches, some buffer math).
# Advisory in CI (continue-on-error) until the backlog is triaged; the script
# itself exits non-zero on findings (--error-exitcode=1) so it is honest when
# run locally or promoted to a hard gate later.
#
# Usage: deploy/run_cppcheck.sh [PATH ...]   (default: common + process dirs)
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

if ! command -v cppcheck >/dev/null 2>&1; then
    echo "cppcheck not installed — skipping (install: apt-get install cppcheck)"
    exit 0
fi

PATHS=("$@")
if [ "${#PATHS[@]}" -eq 0 ]; then
    PATHS=(common process1_* process2_* process3_* process4_* process5_* process6_* process7_*)
fi

echo "== cppcheck $(cppcheck --version) =="
# warning,performance,portability only — 'style'/'information' are too noisy for a gate.
# Suppress include-resolution noise (cppcheck does not see the full -I set) and
# header-only false positives. inline-suppr honours // cppcheck-suppress in code.
cppcheck \
    --std=c++17 \
    --enable=warning,performance,portability \
    --inline-suppr \
    --quiet \
    --error-exitcode=1 \
    --suppress=missingInclude \
    --suppress=missingIncludeSystem \
    --suppress=unmatchedSuppression \
    --suppress=unusedFunction \
    --suppress=unknownMacro \
    -I common/util/include \
    -I common/ipc/include \
    -I common/hal/include \
    "${PATHS[@]}"

rc=$?
if [ "$rc" -eq 0 ]; then
    echo "cppcheck: clean."
fi
exit "$rc"
