#!/usr/bin/env bash
# deploy/run_clang_tidy.sh — clang-tidy via the project .clang-tidy (ADR-016 Tier-0).
#
# clang-tidy needs a compilation database (compile_commands.json); this script
# generates one with a cheap cmake configure if absent, then runs tidy on the
# changed .cpp translation units (default) or all of them (--all).
#
# Advisory in CI (continue-on-error) until the existing backlog is burned down;
# promote to a hard gate once `--all` is clean. Front-runs review-code-quality
# and parts of review-memory-safety (modernize/bugprone/cppcoreguidelines).
#
# Usage:
#   deploy/run_clang_tidy.sh [--changed BASE]   # changed .cpp vs BASE (default origin/main)
#   deploy/run_clang_tidy.sh --all              # every .cpp under common/ + process dirs
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

MODE="changed"
BASE="origin/main"
case "${1:-}" in
    --changed) BASE="${2:-origin/main}" ;;
    --all) MODE="all" ;;
    --help | -h) sed -n '2,18p' "$0"; exit 0 ;;
esac

TIDY="$(command -v clang-tidy-18 || command -v clang-tidy || true)"
if [ -z "$TIDY" ]; then
    echo "clang-tidy not installed — skipping (install: apt-get install clang-tidy-18)"
    exit 0
fi

CDB="build/compile_commands.json"
if [ ! -f "$CDB" ]; then
    echo "== generating compile_commands.json (cmake configure) =="
    if ! cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DALLOW_INSECURE_ZENOH=ON >/dev/null 2>&1; then
        echo "cmake configure failed — cannot run clang-tidy; skipping" >&2
        exit 0
    fi
fi

if [ "$MODE" = "all" ]; then
    mapfile -t FILES < <(git ls-files 'common/**/*.cpp' 'process*_*/**/*.cpp')
else
    mapfile -t FILES < <(git diff --name-only --diff-filter=d "$BASE"...HEAD -- 'common/**/*.cpp' 'process*_*/**/*.cpp')
fi

if [ "${#FILES[@]}" -eq 0 ]; then
    echo "clang-tidy: no C++ translation units to check ($MODE mode)."
    exit 0
fi

echo "== clang-tidy on ${#FILES[@]} file(s) ($MODE mode) =="
rc=0
for f in "${FILES[@]}"; do
    echo "--- $f"
    "$TIDY" -p build --quiet "$f" || rc=1
done
[ "$rc" -eq 0 ] && echo "clang-tidy: clean."
exit "$rc"
