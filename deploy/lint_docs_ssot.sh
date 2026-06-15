#!/usr/bin/env bash
# deploy/lint_docs_ssot.sh — fail if docs hardcode the test-count TOTAL.
#
# Test counts are a Single Source of Truth: they live ONLY in tests/TESTS.md
# (CLAUDE.md §Single Sources of Truth). Every other doc must defer to it
# ("see tests/TESTS.md") rather than restating a number that drifts the moment
# a test is added. This linter is the Tier-0 gate from ADR-016 that makes the
# rule mechanical instead of relying on a reviewer (or Copilot) to spot drift.
#
# Modes:
#   deploy/lint_docs_ssot.sh                 # full audit of all tracked *.md (manual)
#   deploy/lint_docs_ssot.sh --changed BASE  # only ADDED lines vs BASE (CI / pre-commit)
#
# --changed lints only lines this change introduces, so it blocks NEW hardcoded
# totals without failing on pre-existing backlog. CI passes BASE=origin/<base>.
#
# What counts as a violation (3+ digit counts; small deltas like "+5 tests" are
# intentionally NOT matched — those are immutable historical facts, not totals):
#   - "1461 tests" / "1461 C++ tests"
#   - "Total Tests: 1461"
#   - "2131 -> 2136" / "2131 → 2136"  (running-total transitions)
# tests/TESTS.md is always exempt (it is the canonical source).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

MODE="full"
BASE=""
case "${1:-}" in
    --changed) MODE="changed"; BASE="${2:-origin/main}" ;;
    --help | -h) sed -n '2,25p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown arg: $1 (see --help)" >&2; exit 2 ;;
esac

EXEMPT="tests/TESTS.md"

# ERE: a hardcoded test-count total.
PATTERN='([0-9]{3,}[[:space:]]*(\+[[:space:]]*)?(C\+\+[[:space:]]*)?tests?\b)|([Tt]otal[[:space:]]+[Tt]ests?:?[[:space:]]*[0-9]{3,})|([0-9]{3,}[[:space:]]*(->|→|–>)[[:space:]]*[0-9]{3,})'

violations=0
report() {
    echo "  $1"
    violations=$((violations + 1))
}

if [ "$MODE" = "full" ]; then
    while IFS= read -r f; do
        [ "$f" = "$EXEMPT" ] && continue
        while IFS=: read -r ln text; do
            report "$f:$ln: $(echo "$text" | sed 's/^[[:space:]]*//')"
        done < <(grep -nE "$PATTERN" "$f" 2>/dev/null || true)
    done < <(git ls-files '*.md')
else
    cur=""
    while IFS= read -r line; do
        case "$line" in
            "+++ b/"*) cur="${line#+++ b/}" ;;
            "+"*)
                [ "$cur" = "$EXEMPT" ] && continue
                content="${line#+}"
                if printf '%s' "$content" | grep -qE "$PATTERN"; then
                    report "$cur (added): $(printf '%s' "$content" | sed 's/^[[:space:]]*//')"
                fi
                ;;
        esac
    done < <(git diff --unified=0 "$BASE"...HEAD -- '*.md' 2>/dev/null || true)
fi

if [ "$violations" -gt 0 ]; then
    echo ""
    echo "FAIL: $violations hardcoded test-count total(s) found."
    echo "Test counts are SSOT in tests/TESTS.md (CLAUDE.md §Single Sources of Truth)."
    echo "Defer to it (e.g. 'see tests/TESTS.md') instead of restating the number;"
    echo "use a delta ('+N tests') if you must describe a change, not a running total."
    exit 1
fi
echo "doc-SSOT lint: clean ($MODE mode)."
