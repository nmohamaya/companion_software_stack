#!/usr/bin/env bash
# Tests for scripts/start-agent.sh CLI flags
# Note: run_tests.sh handles C++ tests; this shell test is run separately
# or via: bash tests/test_start_agent.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT="$SCRIPT_DIR/scripts/start-agent.sh"

PASS=0
FAIL=0

assert_eq() {
    local label="$1" expected="$2" actual="$3"
    if [[ "$expected" == "$actual" ]]; then
        echo "  PASS  $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL  $label"
        echo "        expected: '$expected'"
        echo "        actual:   '$actual'"
        FAIL=$((FAIL + 1))
    fi
}

assert_contains() {
    local label="$1" needle="$2" haystack="$3"
    if [[ "$haystack" == *"$needle"* ]]; then
        echo "  PASS  $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL  $label"
        echo "        expected to contain: '$needle'"
        echo "        actual: '$haystack'"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== tests/test_start_agent.sh ==="

# ── --version flag ─────────────────────────────────────────────────────────
echo ""
echo "--- --version flag ---"

VERSION_OUTPUT="$(bash "$SCRIPT" --version 2>&1)" || true
VERSION_EXIT=$?
assert_eq "--version exits 0" "0" "$VERSION_EXIT"
assert_eq "--version prints version string" "multi-agent-pipeline v1.0" "$VERSION_OUTPUT"

# ── --help flag ────────────────────────────────────────────────────────────
echo ""
echo "--- --help flag ---"

HELP_OUTPUT="$(bash "$SCRIPT" --help 2>&1)" || true
HELP_EXIT=$?
assert_eq "--help exits 0" "0" "$HELP_EXIT"
assert_contains "--help mentions --version" "--version" "$HELP_OUTPUT"
assert_contains "--help mentions --list" "--list" "$HELP_OUTPUT"

# ── --list flag ────────────────────────────────────────────────────────────
echo ""
echo "--- --list flag ---"

LIST_OUTPUT="$(bash "$SCRIPT" --list 2>&1)" || true
LIST_EXIT=$?
assert_eq "--list exits 0" "0" "$LIST_EXIT"
assert_contains "--list shows tech-lead" "tech-lead" "$LIST_OUTPUT"

# ── Summary ────────────────────────────────────────────────────────────────
echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="

if [[ "$FAIL" -gt 0 ]]; then
    exit 1
fi
exit 0
