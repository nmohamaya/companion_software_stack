#!/usr/bin/env bash
# tests/run_scenario.sh
# ═══════════════════════════════════════════════════════════════
# Scenario-Driven Integration Test Runner
#
# Orchestrates: launch → inject faults → verify → report
#
# Usage:
#   ./tests/run_scenario.sh <scenario_json>
#   ./tests/run_scenario.sh config/scenarios/03_battery_degradation.json
#   ./tests/run_scenario.sh --list                  # list available scenarios
#   ./tests/run_scenario.sh --all                   # run all Tier 1 scenarios
#   ./tests/run_scenario.sh --all --tier 2          # run all (requires Gazebo)
#
# Options:
#   --base-config <path>    Base config to merge with (default: config/default.json)
#   --log-dir <path>        Log output directory (default: /tmp/drone_scenario_logs)
#   --timeout <seconds>     Override scenario timeout
#   --dry-run               Parse scenario, show plan, but don't execute
#   --verbose               Extra verbose output
#   --tier <1|2>            Filter by tier when using --all
#
# Exit codes:
#   0  All checks passed
#   1  One or more checks failed
#   2  Setup / launch failure
# ═══════════════════════════════════════════════════════════════
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"
SCENARIOS_DIR="${PROJECT_DIR}/config/scenarios"
DEPLOY_DIR="${PROJECT_DIR}/deploy"
FAULT_INJECTOR="${BIN_DIR}/fault_injector"
DEFAULT_BASE_CONFIG="${PROJECT_DIR}/config/default.json"
DEFAULT_LOG_DIR="/tmp/drone_scenario_logs"

# ── Colours ───────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── State ─────────────────────────────────────────────────────
PASS=0
FAIL=0
TOTAL=0
LAUNCHER_PID=""
SCENARIO_NAME=""

# ── Parse args ────────────────────────────────────────────────
SCENARIO_FILE=""
BASE_CONFIG="$DEFAULT_BASE_CONFIG"
LOG_DIR="$DEFAULT_LOG_DIR"
TIMEOUT_OVERRIDE=""
DRY_RUN=false
VERBOSE=false
RUN_ALL=false
TIER_FILTER=""
LIST_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --list)         LIST_ONLY=true ;;
        --all)          RUN_ALL=true ;;
        --base-config)  BASE_CONFIG="$2"; shift ;;
        --log-dir)      LOG_DIR="$2"; shift ;;
        --timeout)      TIMEOUT_OVERRIDE="$2"; shift ;;
        --dry-run)      DRY_RUN=true ;;
        --verbose)      VERBOSE=true ;;
        --tier)         TIER_FILTER="$2"; shift ;;
        -h|--help)
            echo "Usage: $0 <scenario_json> [options]"
            echo "       $0 --list"
            echo "       $0 --all [--tier 1]"
            echo ""
            echo "Options:"
            echo "  --base-config <path>  Base config (default: config/default.json)"
            echo "  --log-dir <path>      Log directory (default: /tmp/drone_scenario_logs)"
            echo "  --timeout <seconds>   Override scenario timeout"
            echo "  --dry-run             Parse and show plan only"
            echo "  --verbose             Extra output"
            echo "  --tier <1|2>          Filter by tier"
            echo "  --list                List available scenarios"
            echo "  --all                 Run all matching scenarios"
            exit 0
            ;;
        *)
            if [[ -z "$SCENARIO_FILE" ]]; then
                SCENARIO_FILE="$1"
            else
                echo -e "${RED}ERROR: Unknown argument: $1${NC}"
                exit 2
            fi
            ;;
    esac
    shift
done

# ── Helpers ───────────────────────────────────────────────────

# JSON query helper (uses python3 since jq may not be installed)
# Variables are passed via sys.argv to avoid shell injection into Python code.
json_get() {
    local file="$1"
    local query="$2"
    python3 - "$file" "$query" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    data = json.load(f)
keys = sys.argv[2].split('.')
val = data
for k in keys:
    if isinstance(val, dict) and k in val:
        val = val[k]
    else:
        val = ''
        break
print(val if not isinstance(val, (dict, list)) else json.dumps(val))
PYEOF
}

json_get_array() {
    local file="$1"
    local query="$2"
    python3 - "$file" "$query" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    data = json.load(f)
keys = sys.argv[2].split('.')
val = data
for k in keys:
    if isinstance(val, dict) and k in val:
        val = val[k]
    else:
        val = []
        break
if isinstance(val, list):
    for item in val:
        if isinstance(item, dict):
            print(json.dumps(item))
        else:
            print(item)
PYEOF
}

# Merge scenario config_overrides into base config
merge_configs() {
    local base="$1"
    local scenario="$2"
    local output="$3"
    python3 - "$base" "$scenario" "$output" <<'PYEOF'
import json, sys

def deep_merge(base, override):
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result

with open(sys.argv[1]) as f:
    base_cfg = json.load(f)
with open(sys.argv[2]) as f:
    scenario_cfg = json.load(f)

overrides = scenario_cfg.get('config_overrides', {})
merged = deep_merge(base_cfg, overrides)

with open(sys.argv[3], 'w') as f:
    json.dump(merged, f, indent=4)
PYEOF
}

check() {
    local desc="$1"
    local result="$2"
    TOTAL=$((TOTAL + 1))
    if [[ "$result" -eq 0 ]]; then
        echo -e "  ${GREEN}✓${NC} ${desc}"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}✗${NC} ${desc}"
        FAIL=$((FAIL + 1))
    fi
}

# ── List scenarios ────────────────────────────────────────────
list_scenarios() {
    echo -e "${BOLD}Available Scenarios:${NC}"
    echo ""
    printf "%-4s %-30s %-6s %-8s %s\n" "#" "Name" "Tier" "Gazebo" "Description"
    echo "───────────────────────────────────────────────────────────────────────────────────"
    for f in "${SCENARIOS_DIR}"/[0-9]*.json; do
        [[ -f "$f" ]] || continue
        local name tier requires_gazebo desc
        name=$(json_get "$f" "scenario.name")
        tier=$(json_get "$f" "scenario.tier")
        requires_gazebo=$(json_get "$f" "scenario.requires_gazebo")
        desc=$(json_get "$f" "scenario.description")
        local num
        num=$(basename "$f" | cut -d_ -f1)
        local gazebo_str
        gazebo_str=$([[ "$requires_gazebo" == "True" ]] && echo "yes" || echo "no")

        # Truncate description
        if [[ ${#desc} -gt 50 ]]; then
            desc="${desc:0:47}..."
        fi
        printf "%-4s %-30s %-6s %-8s %s\n" "$num" "$name" "$tier" "$gazebo_str" "$desc"
    done
    echo ""
}

if [[ "$LIST_ONLY" == "true" ]]; then
    list_scenarios
    exit 0
fi

# ── Validate inputs ──────────────────────────────────────────
if [[ "$RUN_ALL" == "false" && -z "$SCENARIO_FILE" ]]; then
    echo -e "${RED}ERROR: Specify a scenario file or --all / --list${NC}"
    echo "Usage: $0 <scenario_json> [options]"
    exit 2
fi

if [[ ! -d "$BIN_DIR" ]]; then
    echo -e "${RED}ERROR: Build directory not found. Build the project first.${NC}"
    exit 2
fi

# ── Run all scenarios ─────────────────────────────────────────
if [[ "$RUN_ALL" == "true" ]]; then
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  Scenario Runner — Running All Scenarios${NC}"
    [[ -n "$TIER_FILTER" ]] && echo -e "  Tier filter: ${TIER_FILTER}"
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo ""

    TOTAL_SCENARIOS=0
    PASSED_SCENARIOS=0
    FAILED_SCENARIOS=0

    for f in "${SCENARIOS_DIR}"/[0-9]*.json; do
        [[ -f "$f" ]] || continue

        if [[ -n "$TIER_FILTER" ]]; then
            tier=$(json_get "$f" "scenario.tier")
            [[ "$tier" != "$TIER_FILTER" ]] && continue
        fi

        TOTAL_SCENARIOS=$((TOTAL_SCENARIOS + 1))
        scenario_name=$(json_get "$f" "scenario.name")
        echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "${BOLD}  Running: ${scenario_name}${NC}"
        echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

        # Run each scenario in a subshell so state is isolated
        args=("$f" --base-config "$BASE_CONFIG" --log-dir "$LOG_DIR")
        [[ -n "$TIMEOUT_OVERRIDE" ]] && args+=(--timeout "$TIMEOUT_OVERRIDE")
        [[ "$DRY_RUN" == "true" ]] && args+=(--dry-run)
        [[ "$VERBOSE" == "true" ]] && args+=(--verbose)

        if "$0" "${args[@]}"; then
            PASSED_SCENARIOS=$((PASSED_SCENARIOS + 1))
        else
            FAILED_SCENARIOS=$((FAILED_SCENARIOS + 1))
        fi
        echo ""
    done

    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo -e "  ${BOLD}Summary:${NC} ${PASSED_SCENARIOS}/${TOTAL_SCENARIOS} scenarios passed"
    if [[ $FAILED_SCENARIOS -gt 0 ]]; then
        echo -e "  ${RED}${FAILED_SCENARIOS} scenario(s) FAILED${NC}"
    else
        echo -e "  ${GREEN}ALL SCENARIOS PASSED${NC}"
    fi
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    [[ $FAILED_SCENARIOS -eq 0 ]] && exit 0 || exit 1
fi

# ═══════════════════════════════════════════════════════════════
# Single scenario execution
# ═══════════════════════════════════════════════════════════════

if [[ ! -f "$SCENARIO_FILE" ]]; then
    # Try relative to SCENARIOS_DIR
    if [[ -f "${SCENARIOS_DIR}/${SCENARIO_FILE}" ]]; then
        SCENARIO_FILE="${SCENARIOS_DIR}/${SCENARIO_FILE}"
    else
        echo -e "${RED}ERROR: Scenario file not found: ${SCENARIO_FILE}${NC}"
        exit 2
    fi
fi

# Extract scenario metadata
SCENARIO_NAME=$(json_get "$SCENARIO_FILE" "scenario.name")
SCENARIO_DESC=$(json_get "$SCENARIO_FILE" "scenario.description")
SCENARIO_TIER=$(json_get "$SCENARIO_FILE" "scenario.tier")
SCENARIO_TIMEOUT=$(json_get "$SCENARIO_FILE" "scenario.timeout_s")
REQUIRES_GAZEBO=$(json_get "$SCENARIO_FILE" "scenario.requires_gazebo")

[[ -n "$TIMEOUT_OVERRIDE" ]] && SCENARIO_TIMEOUT="$TIMEOUT_OVERRIDE"

# Sanitize SCENARIO_NAME — allow only [A-Za-z0-9_-] to prevent path-traversal
SCENARIO_NAME_SAFE=$(echo "$SCENARIO_NAME" | tr -cd 'A-Za-z0-9_-')
if [[ "$SCENARIO_NAME_SAFE" != "$SCENARIO_NAME" ]]; then
    echo -e "${YELLOW}WARNING: Scenario name sanitized: '${SCENARIO_NAME}' → '${SCENARIO_NAME_SAFE}'${NC}"
    SCENARIO_NAME="$SCENARIO_NAME_SAFE"
fi
if [[ -z "$SCENARIO_NAME" ]]; then
    echo -e "${RED}ERROR: Scenario name is empty after sanitization${NC}"
    exit 2
fi

SCENARIO_LOG_DIR="${LOG_DIR}/${SCENARIO_NAME}"
rm -rf "$SCENARIO_LOG_DIR"
mkdir -p "$SCENARIO_LOG_DIR"

echo ""
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}${NC}"
echo -e "  ${SCENARIO_DESC}"
echo -e "  Tier     : ${SCENARIO_TIER}"
echo -e "  Timeout  : ${SCENARIO_TIMEOUT}s"
echo -e "  Gazebo   : ${REQUIRES_GAZEBO}"
echo -e "  Logs     : ${SCENARIO_LOG_DIR}"
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# ── Gazebo requirement check ─────────────────────────────────
if [[ "$REQUIRES_GAZEBO" == "True" ]]; then
    echo ""
    echo -e "${YELLOW}SKIP: Scenario requires Gazebo SITL (Tier 2).${NC}"
    echo -e "${YELLOW}      Run with Gazebo launch_gazebo.sh for full test.${NC}"
    exit 0
fi

# ── Merge configs ─────────────────────────────────────────────
MERGED_CONFIG="${SCENARIO_LOG_DIR}/merged_config.json"
echo ""
echo "Phase 1: Merging configs..."
merge_configs "$BASE_CONFIG" "$SCENARIO_FILE" "$MERGED_CONFIG"
echo -e "  ${GREEN}✓${NC} Config merged → ${MERGED_CONFIG}"

# ── Dry run ───────────────────────────────────────────────────
if [[ "$DRY_RUN" == "true" ]]; then
    echo ""
    echo -e "${YELLOW}DRY RUN — showing execution plan:${NC}"
    echo ""
    echo "Base config    : ${BASE_CONFIG}"
    echo "Scenario file  : ${SCENARIO_FILE}"
    echo "Merged config  : ${MERGED_CONFIG}"
    echo ""
    echo "Fault injection sequence:"
    json_get_array "$SCENARIO_FILE" "fault_sequence.steps" | while read -r step; do
        action=$(echo "$step" | python3 -c "import json,sys; d=json.load(sys.stdin); print(d.get('action',''))" 2>/dev/null)
        delay=$(echo "$step" | python3 -c "import json,sys; d=json.load(sys.stdin); print(d.get('delay_s',0))" 2>/dev/null)
        comment=$(echo "$step" | python3 -c "import json,sys; d=json.load(sys.stdin); print(d.get('_comment',''))" 2>/dev/null)
        echo "  +${delay}s → ${action}: ${comment}"
    done
    echo ""
    echo "Pass criteria:"
    json_get_array "$SCENARIO_FILE" "pass_criteria.log_contains" | while read -r item; do
        echo "  log MUST contain: ${item}"
    done
    json_get_array "$SCENARIO_FILE" "pass_criteria.log_must_not_contain" | while read -r item; do
        echo "  log must NOT contain: ${item}"
    done
    echo ""
    echo -e "${YELLOW}DRY RUN complete — no processes were launched.${NC}"
    exit 0
fi

# Ensure system libstdc++ is used
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ── Launch the stack ──────────────────────────────────────────
echo ""
echo "Phase 2: Launching companion stack..."

# Clean stale SHM
rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
      /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health /dev/shm/fc_commands /dev/shm/mission_upload 2>/dev/null || true

cleanup_scenario() {
    echo ""
    echo "Cleaning up scenario..."
    if [[ -n "$LAUNCHER_PID" ]] && kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        kill -SIGINT "$LAUNCHER_PID" 2>/dev/null || true
        sleep 2
        if kill -0 "$LAUNCHER_PID" 2>/dev/null; then
            kill -SIGKILL "$LAUNCHER_PID" 2>/dev/null || true
        fi
    fi
    # Also kill any child processes
    pkill -f "build/bin/(video_capture|perception|slam_vio_nav|mission_planner|comms|payload_manager|system_monitor)" 2>/dev/null || true
    wait "$LAUNCHER_PID" 2>/dev/null || true
}
trap cleanup_scenario EXIT INT TERM

# Launch via deploy script with merged config
"${DEPLOY_DIR}/launch_all.sh" --config "$MERGED_CONFIG" > "${SCENARIO_LOG_DIR}/launcher.log" 2>&1 &
LAUNCHER_PID=$!

echo -e "  Launcher PID: ${LAUNCHER_PID}"

# Wait for the stack to start (check for SHM segments)
echo -n "  Waiting for SHM segments."
STARTUP_OK=false
for _ in $(seq 1 30); do
    if [[ -f /dev/shm/fc_state ]] && [[ -f /dev/shm/system_health ]]; then
        STARTUP_OK=true
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

if [[ "$STARTUP_OK" == "true" ]]; then
    check "Stack started (SHM segments ready)" 0
else
    check "Stack started (SHM segments ready)" 1
    echo -e "${RED}  Stack failed to start. Check ${SCENARIO_LOG_DIR}/launcher.log${NC}"
fi

# Brief stabilisation delay
sleep 3

# ── Timeout enforcement ──────────────────────────────────────
# Record the scenario start time.  Before each phase we check whether
# the configured timeout has been exceeded.
SCENARIO_START=$SECONDS

check_deadline() {
    if [[ -n "$SCENARIO_TIMEOUT" && "$SCENARIO_TIMEOUT" -gt 0 ]] 2>/dev/null; then
        local elapsed=$(( SECONDS - SCENARIO_START ))
        if [[ $elapsed -ge $SCENARIO_TIMEOUT ]]; then
            echo -e "${RED}TIMEOUT: scenario exceeded ${SCENARIO_TIMEOUT}s (elapsed ${elapsed}s)${NC}"
            exit 2
        fi
    fi
}

# ── Phase 3: Fault injection ─────────────────────────────────
check_deadline
echo ""
echo "Phase 3: Executing fault injection sequence..."

STEPS_JSON=$(json_get "$SCENARIO_FILE" "fault_sequence.steps")
if [[ "$STEPS_JSON" != "[]" && -n "$STEPS_JSON" ]]; then
    if [[ -x "$FAULT_INJECTOR" ]]; then
        # Write the fault sequence to a temp file for the fault_injector to process
        FAULT_SEQ_FILE="${SCENARIO_LOG_DIR}/fault_sequence.json"
        python3 - "$SCENARIO_FILE" "$FAULT_SEQ_FILE" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    data = json.load(f)
seq = data.get('fault_sequence', {})
with open(sys.argv[2], 'w') as f:
    json.dump(seq, f, indent=2)
PYEOF
        echo -e "  Fault sequence: ${FAULT_SEQ_FILE}"
        "${FAULT_INJECTOR}" sequence "$FAULT_SEQ_FILE" 2>&1 | tee "${SCENARIO_LOG_DIR}/fault_injector.log"
        check "Fault injection sequence completed" $?
    else
        echo -e "  ${YELLOW}WARNING: fault_injector not found at ${FAULT_INJECTOR}${NC}"
        echo -e "  ${YELLOW}Build with: cmake --build build --target fault_injector${NC}"
        check "Fault injector available" 1
    fi
else
    echo -e "  ${CYAN}(no fault injection steps for this scenario)${NC}"
fi

# ── Phase 4: Collection window ────────────────────────────────
check_deadline
COLLECTION_TIME=5
echo ""
echo "Phase 4: Post-injection collection (${COLLECTION_TIME}s)..."
sleep "$COLLECTION_TIME"

# Capture process logs from launcher output
# In legacy mode, all output goes to launcher.log

# ── Phase 5: Verification ────────────────────────────────────
check_deadline
echo ""
echo "Phase 5: Verification..."

# All log output is in launcher.log
COMBINED_LOG="${SCENARIO_LOG_DIR}/launcher.log"

# Use process substitution (< <(...)) instead of pipes so that
# PASS/FAIL/TOTAL counters are updated in the current shell.

# Check log_contains
echo ""
echo "Log-contains checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qi "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log contains: ${pattern}" 0
    else
        check "Log missing: ${pattern}" 1
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.log_contains")

# Check log_must_not_contain
echo ""
echo "Log-must-not-contain checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qi "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log unexpectedly contains: ${pattern}" 1
    else
        check "Log correctly does NOT contain: ${pattern}" 0
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.log_must_not_contain")

# Check processes alive
echo ""
echo "Process liveness checks:"
while read -r proc; do
    [[ -z "$proc" ]] && continue
    if pgrep -f "build/bin/${proc}" > /dev/null 2>&1; then
        check "Process alive: ${proc}" 0
    else
        check "Process NOT alive: ${proc}" 1
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.processes_alive")

# Check SHM segments
echo ""
echo "SHM segment checks:"
while read -r seg; do
    [[ -z "$seg" ]] && continue
    if [[ -f "/dev/shm/${seg}" ]]; then
        check "SHM: ${seg}" 0
    else
        check "SHM missing: ${seg}" 1
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.shm_segments_exist")

# ── Results ───────────────────────────────────────────────────
echo ""
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}${NC}"
echo -e "  Logs: ${SCENARIO_LOG_DIR}"
echo -e "  Results: ${PASS} passed, ${FAIL} failed, ${TOTAL} total"
if [[ $FAIL -gt 0 ]]; then
    echo -e "  ${RED}FAILED${NC}"
else
    echo -e "  ${GREEN}PASSED${NC}"
fi
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# Exit based on verification results + launcher health
if [[ $FAIL -gt 0 ]]; then
    exit 1
elif ! kill -0 "$LAUNCHER_PID" 2>/dev/null; then
    echo -e "${RED}Launcher process died unexpectedly${NC}"
    exit 1
else
    exit 0
