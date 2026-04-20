#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# Safety-Critical C++ Audit Script
# Drone Companion Software Stack
#
# Scans production code against 29 safety rules from CLAUDE.md.
# Run from repo root:  bash deploy/safety_audit.sh [report_path]
#
# Exit codes: 0 = all pass, 1 = violations found
# ═══════════════════════════════════════════════════════════════════
#
# ── HOW IT WORKS ────────────────────────────────────────────────
#
# WHAT THIS SCRIPT DOES
#   Performs a static grep-based audit of all production C++ code
#   against the 28 safety-critical rules defined in CLAUDE.md under
#   "Safety-Critical C++ Practices". It checks 16 AVOID rules and
#   12 PREFER rules, then produces:
#     1. Coloured terminal output (PASS/FAIL/WARN per rule)
#     2. A markdown report file (default: safety_audit_report.md)
#
# USAGE
#   bash deploy/safety_audit.sh                  # default report path
#   bash deploy/safety_audit.sh /tmp/report.md   # custom report path
#
#   The script auto-detects the repo root relative to its own location,
#   so it works from any working directory.
#
# SEARCH SCOPE
#   Only production directories are scanned:
#     common/  process1_*  process2_*  process3_*  process4_*
#     process5_*  process6_*  process7_*
#   Test files (tests/), build artefacts, and third-party deps are
#   excluded so the audit focuses on code that ships on the drone.
#
# RULES CHECKED
#   AVOID rules (flagged as FAIL if found):
#     1  memcpy/memset/memmove      8  atoi/atof/strtol
#     2  Raw new/delete              9  Bare std::thread
#     3  shared_ptr (non-external)  10  using namespace in headers
#     4  C-style casts              13  exit/abort in library code
#     5  reinterpret_cast w/o       14  Unscoped enum
#        static_assert              16  Global mutable state
#     6  volatile                   21  memory_order_relaxed
#     7  goto
#    30  Signed→unsigned casts on
#        durations/sizes (warn)
#    31  Mutex-protected observability
#        on flight-critical threads (warn)
#
#   PREFER rules (flagged as FAIL/WARN if missing):
#    17  [[nodiscard]] on Result    24  override keyword
#    20  RAII lock/unlock only      26  Fixed-width types in IPC
#    23  = delete on non-copyable   27  Default member initializers
#    28  noexcept on move ops       29  Strong types over bare floats
#
# HOW TO INTERPRET OUTPUT
#   Terminal colours:
#     GREEN  (PASS) — rule satisfied, no issues
#     RED    (FAIL) — violation found, must fix
#     YELLOW (WARN) — potential issue, manual review needed
#
#   The markdown report contains the same results table plus a
#   "Violation Details" section with exact file:line locations for
#   every FAIL/WARN, so you can jump straight to the code.
#
# HOW TO ADD A NEW RULE
#   1. Pick the next unused rule number
#   2. Add a section like the existing rules:
#        # ── Rule N: description ──────────────────────────
#        hits=$(grep_prod_count 'pattern')
#        if [ "$hits" -eq 0 ]; then
#            log_pass N "Description" "Zero violations"
#        else
#            log_fail N "Description" "$hits occurrences"
#            add_detail "### Rule N: ...\n\`\`\`"
#            add_detail "$(grep_prod 'pattern')"
#            add_detail "\`\`\`\n"
#        fi
#   3. Use log_warn instead of log_fail for heuristic checks
#   4. Update the rule list in this HOW IT WORKS section
#
# HELPER FUNCTIONS
#   grep_prod(pattern)       — grep production .h/.cpp, return lines
#   grep_prod_count(pattern) — same, return count only
#   grep_headers(pattern)    — grep only .h files
#   log_pass/fail/warn(num, desc, detail) — print + append to report
#   add_detail(text)         — append to violation details section
#
# LIMITATIONS
#   - Grep-based, not AST-aware: may have false positives/negatives
#     (e.g., patterns in comments or string literals)
#   - Comments and strings are filtered heuristically (grep -v '//')
#     but not perfectly — clang-tidy covers the remaining edge cases
#   - Some PREFER rules use threshold heuristics (e.g., "at least 10
#     = delete declarations") rather than exhaustive analysis
#   - Integer conversion hazards (Rule 30) use keyword heuristics —
#     can only catch casts near "timeout"/"duration"/"count" keywords.
#     Cannot detect: unsigned subtraction underflow (a - b where a < b),
#     timestamp addition overflow, duration multiplication overflow,
#     or negative-count-to-size casts without keyword context.
#     The review-memory-safety agent covers all 6 sub-patterns with
#     full context-aware analysis.
#   - For a complete static analysis, run clang-tidy-18 in addition
#     to this script
#
# ════════════════════════════════════════════════════════════════
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# ── Colours ──────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Counters ─────────────────────────────────────────────────────
PASS=0
FAIL=0
WARN=0
TOTAL=0

# ── Output helpers ───────────────────────────────────────────────
REPORT_FILE="${1:-safety_audit_report.md}"
REPORT=""

log_pass() {
    ((PASS++)) || true
    ((TOTAL++)) || true
    printf "  ${GREEN}✅ PASS${NC}  Rule %2d: %s\n" "$1" "$2"
    REPORT+="| $1 | $2 | ✅ PASS | $3 |\n"
}

log_fail() {
    ((FAIL++)) || true
    ((TOTAL++)) || true
    printf "  ${RED}❌ FAIL${NC}  Rule %2d: %s\n" "$1" "$2"
    REPORT+="| $1 | $2 | ❌ FAIL | $3 |\n"
}

log_warn() {
    ((WARN++)) || true
    ((TOTAL++)) || true
    printf "  ${YELLOW}⚠️  WARN${NC}  Rule %2d: %s\n" "$1" "$2"
    REPORT+="| $1 | $2 | ⚠️ WARN | $3 |\n"
}

# ── Search scope (production code only) ─────────────────────────
PROD_DIRS="common process1_video_capture process2_perception process3_slam_vio_nav process4_mission_planner process5_comms process6_payload_manager process7_system_monitor"
PROD_GLOBS=""
for d in $PROD_DIRS; do
    PROD_GLOBS="$PROD_GLOBS --include='*.h' --include='*.cpp'"
done

# Helper: grep production code, return count
grep_prod() {
    local pattern="$1"
    local extra="${2:-}"
    grep -rn "$pattern" $PROD_DIRS --include='*.h' --include='*.cpp' $extra 2>/dev/null || true
}

grep_prod_count() {
    grep_prod "$1" "${2:-}" | wc -l
}

# Helper: grep headers only
grep_headers() {
    local pattern="$1"
    grep -rn "$pattern" $PROD_DIRS --include='*.h' 2>/dev/null || true
}

DETAILS=""
add_detail() {
    DETAILS+="$1\n"
}

# Count non-empty lines in a variable (safe for empty strings)
count_lines() {
    local text="$1"
    if [ -z "$text" ]; then
        echo 0
    else
        echo "$text" | grep -c . 2>/dev/null || echo 0
    fi
}

# ═══════════════════════════════════════════════════════════════════
printf "\n${BOLD}${CYAN}═══════════════════════════════════════════════════════════${NC}\n"
printf "${BOLD}  Safety-Critical C++ Audit${NC}\n"
printf "${BOLD}${CYAN}═══════════════════════════════════════════════════════════${NC}\n\n"
printf "${BOLD}AVOID Rules${NC}\n\n"
# ═══════════════════════════════════════════════════════════════════

# ── Rule 1: memcpy/memset/memmove ───────────────────────────────
# Exclude comments (// and ///) and string literals ("...")
memcpy_hits=$(grep_prod '\bmemcpy\b\|\bmemset\b\|\bmemmove\b' | grep -v '//\|".*memcpy\|".*memset\|".*memmove' || true)
hits=$(count_lines "$memcpy_hits")
if [ "$hits" -eq 0 ]; then
    log_pass 1 "No memcpy/memset/memmove" "Zero uses in production code"
else
    log_fail 1 "memcpy/memset/memmove found" "$hits occurrences"
    add_detail "### Rule 1: memcpy/memset/memmove\n\`\`\`"
    add_detail "$memcpy_hits"
    add_detail "\`\`\`\n"
fi

# ── Rule 2: Raw new/delete ──────────────────────────────────────
# Exclude comments, string literals, smart ptr patterns, and singleton pattern
raw_new=$(grep_prod '\bnew \b' | grep -v '//' | grep -v 'make_unique\|make_shared\|placement new\|operator new' | grep -v '\.md:' | grep -v '".*new ' || true)
raw_delete=$(grep_prod '\bdelete \b\|\bdelete\[' | grep -v '//' | grep -v '".*delete' || true)
hits=$(count_lines "$raw_new$raw_delete")
if [ "$hits" -eq 0 ]; then
    log_pass 2 "No raw new/delete" "Zero violations"
else
    log_fail 2 "Raw new/delete found" "$hits occurrences"
    add_detail "### Rule 2: Raw new/delete\n\`\`\`"
    add_detail "$raw_new$raw_delete"
    add_detail "\`\`\`\n"
fi

# ── Rule 3: shared_ptr in non-external code ─────────────────────
# Exclude external library contracts and comments
shared_hits=$(grep_prod 'shared_ptr' | grep -v 'spdlog\|mavsdk\|MAVSDK\|zenoh\|Zenoh\|callback\|// external\|///' | grep -v '//' || true)
hits=$(count_lines "$shared_hits")
if [ "$hits" -eq 0 ]; then
    log_pass 3 "No shared_ptr in non-external code" "All uses are external library contracts"
else
    log_warn 3 "shared_ptr in non-external code" "$hits — review if justified"
    add_detail "### Rule 3: shared_ptr (review needed)\n\`\`\`"
    add_detail "$shared_hits"
    add_detail "\`\`\`\n"
fi

# ── Rule 4: C-style casts ──────────────────────────────────────
# Pattern: (type)expr — match (int), (float), (double), (void*), (char*), (uint8_t*), (size_t) etc.
# Exclude (void)param — standard C++ idiom for unused parameters
c_casts=$(grep_prod '([[:space:]]*\(int\|float\|double\|char\|uint8_t\|uint16_t\|uint32_t\|uint64_t\|int8_t\|int16_t\|int32_t\|int64_t\|size_t\|unsigned\)[[:space:]]*\*\?[[:space:]]*)' | grep -v '//' | grep -v 'static_cast\|dynamic_cast\|reinterpret_cast\|const_cast' || true)
hits=$(count_lines "$c_casts")
if [ "$hits" -eq 0 ]; then
    log_pass 4 "No C-style casts" "Zero violations"
else
    log_fail 4 "C-style casts found" "$hits occurrences"
    add_detail "### Rule 4: C-style casts\n\`\`\`"
    add_detail "$c_casts"
    add_detail "\`\`\`\n"
fi

# ── Rule 5: reinterpret_cast without static_assert ──────────────
reinterpret_count=$(grep_prod_count '\breinterpret_cast\b')
static_assert_count=$(grep_prod_count 'static_assert.*trivially_copyable')
if [ "$reinterpret_count" -gt 0 ] && [ "$static_assert_count" -eq 0 ]; then
    log_fail 5 "reinterpret_cast without static_assert" "$reinterpret_count casts, 0 static_asserts"
else
    log_pass 5 "reinterpret_cast properly guarded" "$reinterpret_count casts, $static_assert_count static_asserts"
fi

# ── Rule 6: volatile ────────────────────────────────────────────
hits=$(grep_prod_count '\bvolatile\b')
if [ "$hits" -eq 0 ]; then
    log_pass 6 "No volatile" "Zero violations"
else
    log_fail 6 "volatile found" "$hits occurrences"
    add_detail "### Rule 6: volatile\n\`\`\`"
    add_detail "$(grep_prod '\bvolatile\b')"
    add_detail "\`\`\`\n"
fi

# ── Rule 7: goto ────────────────────────────────────────────────
hits=$(grep_prod_count '\bgoto\b')
if [ "$hits" -eq 0 ]; then
    log_pass 7 "No goto" "Zero violations"
else
    log_fail 7 "goto found" "$hits occurrences"
    add_detail "### Rule 7: goto\n\`\`\`"
    add_detail "$(grep_prod '\bgoto\b')"
    add_detail "\`\`\`\n"
fi

# ── Rule 8: atoi/atof/strtol ───────────────────────────────────
hits=$(grep_prod_count '\batoi\b\|\batof\b\|\bstrtol\b\|\bstrtod\b\|\bstrtoul\b')
if [ "$hits" -eq 0 ]; then
    log_pass 8 "No atoi/atof/strtol" "Zero violations"
else
    log_fail 8 "Unsafe string conversion found" "$hits occurrences"
    add_detail "### Rule 8: atoi/atof/strtol\n\`\`\`"
    add_detail "$(grep_prod '\batoi\b\|\batof\b\|\bstrtol\b\|\bstrtod\b\|\bstrtoul\b')"
    add_detail "\`\`\`\n"
fi

# ── Rule 9: Bare std::thread ───────────────────────────────────
thread_count=$(grep_prod_count 'std::thread\b')
join_count=$(grep_prod_count '\.join()\|\.joinable()')
if [ "$thread_count" -gt 0 ] && [ "$join_count" -eq 0 ]; then
    log_fail 9 "Bare std::thread without join" "$thread_count threads, 0 joins"
else
    log_pass 9 "std::thread properly joined" "$thread_count threads, $join_count join/joinable calls"
fi

# ── Rule 10: using namespace in headers ─────────────────────────
using_ns=$(grep_headers 'using namespace ' | grep -v '//' || true)
hits=$(count_lines "$using_ns")
if [ "$hits" -eq 0 ]; then
    log_pass 10 "No 'using namespace' in headers" "Zero violations"
else
    log_fail 10 "'using namespace' in headers" "$hits occurrences"
    add_detail "### Rule 10: using namespace in headers\n\`\`\`"
    add_detail "$using_ns"
    add_detail "\`\`\`\n"
fi

# ── Rule 13: exit/abort/terminate ───────────────────────────────
exit_hits=$(grep_prod '\bexit\s*(\|\babort\s*(\|std::terminate\s*(' | grep -v '//' | grep -v 'EXIT_SUCCESS\|EXIT_FAILURE\|_exit\b' || true)
# Allow exit() in main() only
exit_in_lib=$(echo "$exit_hits" | grep -v 'main\.cpp' || true)
hits=$(count_lines "$exit_in_lib")
if [ "$hits" -eq 0 ]; then
    log_pass 13 "No exit/abort in library code" "Only in main() (acceptable)"
else
    log_fail 13 "exit/abort in library code" "$hits occurrences"
    add_detail "### Rule 13: exit/abort in library code\n\`\`\`"
    add_detail "$exit_in_lib"
    add_detail "\`\`\`\n"
fi

# ── Rule 14: Unscoped enum ─────────────────────────────────────
unscoped=$(grep_prod '^[[:space:]]*enum [^c]' | grep -v 'enum class\|enum struct\|//' || true)
hits=$(count_lines "$unscoped")
if [ "$hits" -eq 0 ]; then
    log_pass 14 "No unscoped enums" "All use enum class"
else
    log_fail 14 "Unscoped enum found" "$hits occurrences"
    add_detail "### Rule 14: Unscoped enums\n\`\`\`"
    add_detail "$unscoped"
    add_detail "\`\`\`\n"
fi

# ── Rule 16: Global mutable state ──────────────────────────────
# Static non-const variables (excluding function-local statics which are OK)
global_mut=$(grep_prod '^static [^c]' | grep -v 'const \|constexpr \|inline \|static_assert\|static void\|static auto\|static Result\|static std::string\|static bool\|static int\|// ' || true)
hits=$(count_lines "$global_mut")
if [ "$hits" -eq 0 ]; then
    log_pass 16 "No global mutable state" "Zero violations"
else
    log_warn 16 "Global mutable state found" "$hits — review if thread-safe"
    add_detail "### Rule 16: Global mutable state (review needed)\n\`\`\`"
    add_detail "$global_mut"
    add_detail "\`\`\`\n"
fi

# ── Rule 21: memory_order_relaxed ───────────────────────────────
relaxed_count=$(grep_prod_count 'memory_order_relaxed')
if [ "$relaxed_count" -eq 0 ]; then
    log_pass 21 "No memory_order_relaxed" "None used"
else
    log_warn 21 "memory_order_relaxed used" "$relaxed_count occurrences — verify each is justified"
    add_detail "### Rule 21: memory_order_relaxed (review needed)\n\`\`\`"
    add_detail "$(grep_prod 'memory_order_relaxed')"
    add_detail "\`\`\`\n"
fi

# ── Rule 30: Integer conversion hazards (signed→unsigned) ──────
# Detect static_cast<uint*> near timeout/duration/count/size keywords
# without a preceding std::max clamp.  Grep-based approximation —
# the review-memory-safety agent does the full context-aware check.
# Sub-check A: static_cast<uint64_t> applied to chrono or timeout expressions
signed_to_unsigned=$(grep_prod 'static_cast<uint64_t>\|static_cast<uint32_t>\|static_cast<size_t>' | grep -i 'timeout\|duration\|elapsed\|deadline\|sleep\|count()' | grep -v 'std::max\|std::clamp\|>= 0\|> 0\|//' || true)
# Sub-check B: narrowing cast static_cast<uint32_t>(sizeof — rarely a problem but flag it
narrowing_sizeof=$(grep_prod 'static_cast<uint32_t>(sizeof' | grep -v '//' || true)
int_hazard_hits="$signed_to_unsigned$narrowing_sizeof"
hits=$(count_lines "$int_hazard_hits")
if [ "$hits" -eq 0 ]; then
    log_pass 30 "No unguarded signed→unsigned casts" "Duration/size casts properly guarded"
else
    log_warn 30 "Possible signed→unsigned hazards" "$hits — verify clamp before cast"
    add_detail "### Rule 30: Integer conversion hazards (review needed)\n\`\`\`"
    add_detail "$int_hazard_hits"
    add_detail "\`\`\`\n"
fi

# ── Rule 31: Mutex-protected observability on flight-critical threads ──
# Flags every production-code use of a known MUTEX-PROTECTED observability
# primitive. Each hit requires EITHER (a) manual confirmation that the
# caller runs on a non-real-time thread (periodic logger, scenario-end
# dump, etc.) OR (b) a DR-NNN entry in docs/guides/DESIGN_RATIONALE.md
# documenting the hazard analysis: priority isolation + bounded hold-time
# + config gating. See CLAUDE.md § Concurrency tiering → "Observability
# on flight-critical threads."
#
# OBS_PATTERNS lists only primitives that actually take a std::mutex in
# their record/emit path. Lock-free primitives like FrameDiagnostics,
# LatencyTracker (single-threaded), SPSCRing, TripleBuffer are explicitly
# NOT in this list — they are safe on hot paths. Extend OBS_PATTERNS when
# a new mutex-protected observability primitive lands.
OBS_PATTERNS='\bScopedLatency\b\|\bLatencyProfiler\b'
obs_hits=$(grep_prod "$OBS_PATTERNS" | grep -v '//\|latency_profiler\.h' || true)
hits=$(count_lines "$obs_hits")
if [ "$hits" -eq 0 ]; then
    log_pass 31 "No mutex-protected observability in production code" "Zero call sites — add audit review if this changes"
else
    log_warn 31 "Mutex-protected observability in production code" "$hits — verify caller is non-RT OR a DR entry exists in DESIGN_RATIONALE.md"
    add_detail "### Rule 31: Mutex-protected observability (requires non-RT caller or DR-NNN justification)\n\`\`\`"
    add_detail "$obs_hits"
    add_detail "\`\`\`\n"
fi

# ═══════════════════════════════════════════════════════════════════
printf "\n${BOLD}PREFER Rules${NC}\n\n"
# ═══════════════════════════════════════════════════════════════════

# ── Rule 17: [[nodiscard]] on Result returns ────────────────────
# Find virtual functions returning Result/VIOResult without [[nodiscard]]
missing_nodiscard=$(grep_headers 'virtual.*Result<\|virtual.*VIOResult<' | grep -v '\[\[nodiscard\]\]' | grep -v '//' || true)
hits=$(count_lines "$missing_nodiscard")
if [ "$hits" -eq 0 ]; then
    log_pass 17 "[[nodiscard]] on all Result returns" "All virtual Result methods annotated"
else
    log_fail 17 "Missing [[nodiscard]] on Result returns" "$hits violations"
    add_detail "### Rule 17: Missing [[nodiscard]]\n\`\`\`"
    add_detail "$missing_nodiscard"
    add_detail "\`\`\`\n"
fi

# ── Rule 20: RAII lock/unlock ───────────────────────────────────
bare_lock=$(grep_prod '\.lock()\|\.unlock()' | grep -v 'lock_guard\|unique_lock\|shared_lock\|scoped_lock\|//' | grep -v '\.joinable\|try_lock' || true)
hits=$(count_lines "$bare_lock")
if [ "$hits" -eq 0 ]; then
    log_pass 20 "RAII lock/unlock only" "Zero bare lock/unlock calls"
else
    log_fail 20 "Bare lock/unlock calls" "$hits violations"
    add_detail "### Rule 20: Bare lock/unlock\n\`\`\`"
    add_detail "$bare_lock"
    add_detail "\`\`\`\n"
fi

# ── Rule 23: = delete on non-copyable ───────────────────────────
delete_count=$(grep_prod_count '= delete')
if [ "$delete_count" -ge 10 ]; then
    log_pass 23 "= delete on non-copyable types" "$delete_count delete declarations found"
else
    log_warn 23 "Few = delete declarations" "$delete_count — review resource-owning classes"
fi

# ── Rule 24: override on virtuals ───────────────────────────────
# Check for virtual methods in derived classes without override
# This is approximate — a full check requires clang-tidy
override_count=$(grep_prod_count '\boverride\b')
if [ "$override_count" -ge 20 ]; then
    log_pass 24 "override keyword used" "$override_count override declarations"
else
    log_warn 24 "Few override keywords" "$override_count — run clang-tidy modernize-use-override"
fi

# ── Rule 26: Fixed-width types in IPC structs ───────────────────
# Exclude constexpr constants, loop variables, and comments — only flag struct member fields
bare_int_ipc=$(grep -n '\bint \b\|\blong \b\|\bunsigned \b' common/ipc/include/ipc/ipc_types.h common/ipc/include/ipc/wire_format.h 2>/dev/null | grep -v '//' | grep -v 'uint\|int8_t\|int16_t\|int32_t\|int64_t\|constexpr\|for (\|for(' || true)
hits=$(count_lines "$bare_int_ipc")
if [ "$hits" -eq 0 ]; then
    log_pass 26 "Fixed-width types in IPC structs" "No bare int/long in wire data"
else
    log_fail 26 "Bare int/long in IPC structs" "$hits occurrences"
    add_detail "### Rule 26: Bare int/long in IPC\n\`\`\`"
    add_detail "$bare_int_ipc"
    add_detail "\`\`\`\n"
fi

# ── Rule 27: Default member initializers ────────────────────────
# Check IPC types for uninitialized members (heuristic: struct members without = or {})
# This is approximate
init_count=$(grep -c '= \|{.*}' common/ipc/include/ipc/ipc_types.h 2>/dev/null || echo 0)
if [ "$init_count" -ge 30 ]; then
    log_pass 27 "Default member initializers" "$init_count initialized members in ipc_types.h"
else
    log_warn 27 "Few default initializers" "$init_count — review struct members"
fi

# ── Rule 28: noexcept on move constructors/operators ────────────
# Find move constructors/operators missing noexcept
move_no_noexcept=$(grep_headers '&&)\s*[{;=]' | grep -v 'noexcept\|//\|delete\|template\|forward\|static_cast\|std::move' || true)
# Also check explicitly: Type(Type&&) and operator=(Type&&) without noexcept
move_ctors=$(grep_headers '([A-Za-z_]*&&[[:space:]]*)' | grep -v 'noexcept\|//\|template\|forward\|static_cast\|const\|std::' || true)
combined_move="$move_no_noexcept$move_ctors"
hits=$(count_lines "$combined_move")
if [ "$hits" -eq 0 ]; then
    log_pass 28 "noexcept on move operations" "All move ctors/operators are noexcept"
else
    log_warn 28 "Move operations possibly missing noexcept" "$hits — review move ctors/operators"
    add_detail "### Rule 28: Move operations missing noexcept (review needed)\n\`\`\`"
    add_detail "$combined_move"
    add_detail "\`\`\`\n"
fi

# ── Rule 29: Strong types over bare float params ───────────────
# Heuristic: find functions with 3+ consecutive float/double params (confusion risk)
multi_float=$(grep_prod 'float [a-z_]*, float [a-z_]*, float\|double [a-z_]*, double [a-z_]*, double' | grep -v '//' || true)
hits=$(count_lines "$multi_float")
if [ "$hits" -eq 0 ]; then
    log_pass 29 "No confusable bare float param groups" "No 3+ consecutive float/double params"
else
    log_warn 29 "Bare float param groups (confusion risk)" "$hits — consider strong types (Meters, Radians)"
    add_detail "### Rule 29: Bare float param groups (review needed)\n\`\`\`"
    add_detail "$multi_float"
    add_detail "\`\`\`\n"
fi

# ═══════════════════════════════════════════════════════════════════
printf "\n${BOLD}${CYAN}═══════════════════════════════════════════════════════════${NC}\n"
printf "${BOLD}  Summary${NC}\n"
printf "${BOLD}${CYAN}═══════════════════════════════════════════════════════════${NC}\n\n"
# ═══════════════════════════════════════════════════════════════════

printf "  ${GREEN}Passed:${NC}  %d\n" "$PASS"
printf "  ${RED}Failed:${NC}  %d\n" "$FAIL"
printf "  ${YELLOW}Warnings:${NC} %d\n" "$WARN"
printf "  ${BOLD}Total:${NC}   %d rules checked\n\n" "$TOTAL"

if [ "$FAIL" -eq 0 ]; then
    printf "  ${GREEN}${BOLD}✓ Safety audit PASSED${NC}\n\n"
else
    printf "  ${RED}${BOLD}✗ Safety audit FAILED — %d violation(s) found${NC}\n\n" "$FAIL"
fi

# ═══════════════════════════════════════════════════════════════════
# Generate markdown report
# ═══════════════════════════════════════════════════════════════════
{
    echo "# Safety-Critical C++ Audit Report"
    echo ""
    echo "**Date:** $(date '+%Y-%m-%d %H:%M')"
    echo "**Repo:** $(git remote get-url origin 2>/dev/null || echo 'local')"
    echo "**Branch:** $(git branch --show-current 2>/dev/null || echo 'unknown')"
    echo "**Commit:** $(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
    echo ""
    echo "## Summary"
    echo ""
    echo "| Result | Count |"
    echo "|--------|-------|"
    echo "| ✅ Passed | $PASS |"
    echo "| ❌ Failed | $FAIL |"
    echo "| ⚠️ Warnings | $WARN |"
    echo "| **Total** | **$TOTAL** |"
    echo ""
    echo "## Results"
    echo ""
    echo "| # | Rule | Status | Details |"
    echo "|---|------|--------|---------|"
    echo -e "$REPORT"
    echo ""
    if [ -n "$DETAILS" ]; then
        echo "## Violation Details"
        echo ""
        echo -e "$DETAILS"
    fi
    echo "---"
    echo ""
    echo "*Generated by \`deploy/safety_audit.sh\`. Rules defined in CLAUDE.md under \"Safety-Critical C++ Practices\".*"
} > "$REPORT_FILE"

printf "  Report written to: ${CYAN}%s${NC}\n\n" "$REPORT_FILE"

# Exit with failure if violations found
[ "$FAIL" -eq 0 ]
