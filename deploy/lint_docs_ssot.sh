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
#   deploy/lint_docs_ssot.sh                 # full audit of tracked *.md (manual)
#   deploy/lint_docs_ssot.sh --changed BASE  # only ADDED lines vs BASE (CI)
#   deploy/lint_docs_ssot.sh --staged        # only ADDED staged lines (pre-commit)
#
# --changed/--staged lint only the lines a change introduces, so they block NEW
# hardcoded totals without failing on pre-existing backlog. CI passes
# BASE=origin/<base>.
#
# What counts as a violation (3+ digit counts; small deltas like "+5 tests" are
# intentionally NOT matched — those are immutable point-in-time facts):
#   - "1461 tests" / "1461 C++ tests"              (but NOT "1400 test lines/files/...")
#   - "Total Tests: 1461"
#   - "2131 -> 2136" / "2131 → 2136" near "test"   (NOT "809 -> 366 lines", resolutions, constants)
#
# Exemptions:
#   - tests/TESTS.md           — always (canonical source).
#   - In FULL mode only: append-only / point-in-time historical records whose
#     counts are frozen facts (PROGRESS.md, BUG_FIXES.md, CI_ISSUES.md,
#     ROADMAP.md, docs/adr/**). The changed/staged gate STILL scans these, so a
#     NEW entry that restates a running total is still caught.
#
# Requires GNU grep -P (PCRE); available on Linux/CI.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

MODE="full"
BASE=""
case "${1:-}" in
    --changed) MODE="changed"; BASE="${2:-origin/main}" ;;
    --staged) MODE="staged" ;;
    --help | -h) sed -n '2,30p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown arg: $1 (see --help)" >&2; exit 2 ;;
esac

# PCRE: a hardcoded test-count total.
#  A: "NNN[+] [C++] tests" — excluding "test lines/files/binaries/suites/cases/stages"
#  B: "Total Tests: NNN"
#  C: a "NNN -> NNN" / "NNN → NNN" transition within ~40 chars of the word "test"
PATTERN_A='(?<!#)\b[0-9]{3,}\+?\s*(?:C\+\+\s*)?tests?\b(?!\s*(?:lines|files|binaries|suites|cases|stages))'
PATTERN_B='[Tt]otal\s+[Tt]ests?:?\s*[0-9]{3,}'
PATTERN_C='(?:(?i:tests?)\b.{0,40}[0-9]{3,}\s*(?:->|→)\s*[0-9]{3,}|[0-9]{3,}\s*(?:->|→)\s*[0-9]{3,}.{0,40}(?i:tests?))'
PATTERN="${PATTERN_A}|${PATTERN_B}|${PATTERN_C}"

is_exempt() {
    local f="$1"
    [ "$f" = "tests/TESTS.md" ] && return 0
    if [ "$MODE" = "full" ]; then
        case "$f" in
            docs/tracking/PROGRESS.md | docs/tracking/BUG_FIXES.md | \
                docs/tracking/CI_ISSUES.md | docs/tracking/ROADMAP.md) return 0 ;;
            docs/adr/*) return 0 ;;
        esac
    fi
    return 1
}

violations=0
report() {
    echo "  $1"
    violations=$((violations + 1))
}

if [ "$MODE" = "full" ]; then
    while IFS= read -r f; do
        is_exempt "$f" && continue
        while IFS=: read -r ln text; do
            report "$f:$ln: $(echo "$text" | sed 's/^[[:space:]]*//')"
        done < <(grep -nP "$PATTERN" "$f" 2>/dev/null || true)
    done < <(git ls-files '*.md')
else
    if [ "$MODE" = "staged" ]; then
        DIFFCMD=(git diff --cached --unified=0 -- '*.md')
    else
        DIFFCMD=(git diff --unified=0 "$BASE...HEAD" -- '*.md')
    fi
    cur=""
    while IFS= read -r line; do
        case "$line" in
            "+++ b/"*) cur="${line#+++ b/}" ;;
            "+"*)
                is_exempt "$cur" && continue
                content="${line#+}"
                if printf '%s' "$content" | grep -qP "$PATTERN"; then
                    report "$cur (added): $(printf '%s' "$content" | sed 's/^[[:space:]]*//')"
                fi
                ;;
        esac
    done < <("${DIFFCMD[@]}" 2>/dev/null || true)
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
