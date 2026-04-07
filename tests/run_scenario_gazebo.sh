#!/usr/bin/env bash
# tests/run_scenario_gazebo.sh
# ═══════════════════════════════════════════════════════════════
# Tier 2 Gazebo SITL Scenario Runner
#
# Launches PX4 SITL + Gazebo + companion stack, injects faults,
# and verifies pass criteria against real MAVLink + Gazebo sensors.
#
# Usage:
#   ./tests/run_scenario_gazebo.sh <scenario_json>
#   ./tests/run_scenario_gazebo.sh config/scenarios/01_nominal_mission.json
#   ./tests/run_scenario_gazebo.sh --list
#   ./tests/run_scenario_gazebo.sh --all
#   ./tests/run_scenario_gazebo.sh --all --gui       # with Gazebo 3D window
#
# Options:
#   --base-config <path>    Base config (default: config/gazebo_sitl.json)
#   --log-dir <path>        Log output directory (default: drone_logs/scenarios_gazebo)
#   --timeout <seconds>     Override scenario timeout
#   --dry-run               Parse scenario, show plan, don't execute
#   --verbose               Extra verbose output
#   --gui                   Launch Gazebo GUI (3D visualisation)
#   --json-logs             Enable structured JSON log output
#   --ipc <shm|zenoh>       Override IPC backend (default: from base config)
#
# Environment variables:
#   PX4_DIR                 Path to PX4-Autopilot (default: ~/PX4-Autopilot)
#   GZ_WORLD                Path to world SDF (default: sim/worlds/test_world.sdf)
#
# Exit codes:
#   0  All checks passed
#   1  One or more checks failed
#   2  Setup / launch failure
# ═══════════════════════════════════════════════════════════════
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/lib_scenario_logging.sh"
BIN_DIR="${PROJECT_DIR}/build/bin"
SCENARIOS_DIR="${PROJECT_DIR}/config/scenarios"
DEPLOY_DIR="${PROJECT_DIR}/deploy"
FAULT_INJECTOR="${BIN_DIR}/fault_injector"
DEFAULT_BASE_CONFIG="${PROJECT_DIR}/config/gazebo_sitl.json"
DEFAULT_LOG_DIR="${PROJECT_DIR}/drone_logs/scenarios_gazebo"

# ── PX4 / Gazebo paths ───────────────────────────────────────
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
GZ_WORLD="${GZ_WORLD:-${PROJECT_DIR}/sim/worlds/test_world.sdf}"

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
LIST_ONLY=false
GUI_FLAG=""
JSON_LOGS=""
IPC_BACKEND=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --list)         LIST_ONLY=true ;;
        --all)          RUN_ALL=true ;;
        --base-config)  BASE_CONFIG="$2"; shift ;;
        --log-dir)      LOG_DIR="$2"; shift ;;
        --timeout)      TIMEOUT_OVERRIDE="$2"; shift ;;
        --dry-run)      DRY_RUN=true ;;
        --verbose)      VERBOSE=true ;;
        --gui)          GUI_FLAG="--gui" ;;
        --json-logs)    JSON_LOGS="--json-logs" ;;
        --ipc)
            IPC_BACKEND="$2"
            if [[ "$IPC_BACKEND" != "shm" && "$IPC_BACKEND" != "zenoh" ]]; then
                echo -e "${RED}ERROR: --ipc must be 'shm' or 'zenoh'${NC}"
                exit 2
            fi
            shift ;;
        -h|--help)
            echo "Usage: $0 <scenario_json> [options]"
            echo "       $0 --list"
            echo "       $0 --all [--gui]"
            echo ""
            echo "Options:"
            echo "  --base-config <path>  Base config (default: config/gazebo_sitl.json)"
            echo "  --log-dir <path>      Log directory (default: drone_logs/scenarios_gazebo)"
            echo "  --timeout <seconds>   Override scenario timeout"
            echo "  --dry-run             Parse and show plan only"
            echo "  --verbose             Extra output"
            echo "  --gui                 Launch Gazebo 3D GUI"
            echo "  --json-logs           Enable structured JSON log output"
            echo "  --ipc <shm|zenoh>     Override IPC transport backend"
            echo "  --list                List available scenarios"
            echo "  --all                 Run all Tier 2 scenarios"
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

# ── Prerequisite checks ──────────────────────────────────────
check_prerequisites() {
    local ok=true
    PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
    if [[ ! -x "$PX4_BIN" ]]; then
        echo -e "${RED}ERROR: PX4 SITL binary not found at ${PX4_BIN}${NC}"
        echo -e "${RED}       Build PX4: cd ${PX4_DIR} && make px4_sitl_default${NC}"
        ok=false
    fi
    if ! command -v gz &>/dev/null; then
        echo -e "${RED}ERROR: 'gz' command not found — install Gazebo Harmonic${NC}"
        ok=false
    fi
    if [[ ! -f "$GZ_WORLD" ]]; then
        echo -e "${RED}ERROR: World file not found: ${GZ_WORLD}${NC}"
        ok=false
    fi
    if [[ ! -d "$BIN_DIR" ]]; then
        echo -e "${RED}ERROR: Build directory not found. Build the project first.${NC}"
        ok=false
    fi
    if [[ ! -f "$BASE_CONFIG" ]]; then
        echo -e "${RED}ERROR: Base config not found: ${BASE_CONFIG}${NC}"
        ok=false
    fi
    [[ "$ok" == "false" ]] && exit 2
}

# ── List scenarios ────────────────────────────────────────────
list_scenarios() {
    echo -e "${BOLD}Available Scenarios (Tier 2 — Gazebo SITL):${NC}"
    echo ""
    printf "%-4s %-30s %-6s %s\n" "#" "Name" "Tier" "Description"
    echo "───────────────────────────────────────────────────────────────────────────────────"
    for f in "${SCENARIOS_DIR}"/[0-9]*.json; do
        [[ -f "$f" ]] || continue
        local name tier desc
        name=$(json_get "$f" "scenario.name")
        tier=$(json_get "$f" "scenario.tier")
        desc=$(json_get "$f" "scenario.description")
        local num
        num=$(basename "$f" | cut -d_ -f1)
        if [[ ${#desc} -gt 50 ]]; then
            desc="${desc:0:47}..."
        fi
        printf "%-4s %-30s %-6s %s\n" "$num" "$name" "$tier" "$desc"
    done
    echo ""
    echo "All scenarios run in Gazebo SITL mode (PX4 + MAVLink + Gazebo sensors)."
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

check_prerequisites

# ── Run all scenarios ─────────────────────────────────────────
if [[ "$RUN_ALL" == "true" ]]; then
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  Gazebo SITL Scenario Runner — All Scenarios${NC}"
    echo -e "  PX4    : ${PX4_DIR}"
    echo -e "  World  : ${GZ_WORLD}"
    echo -e "  Config : ${BASE_CONFIG}"
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo ""

    TOTAL_SCENARIOS=0
    PASSED_SCENARIOS=0
    FAILED_SCENARIOS=0

    for f in "${SCENARIOS_DIR}"/[0-9]*.json; do
        [[ -f "$f" ]] || continue

        TOTAL_SCENARIOS=$((TOTAL_SCENARIOS + 1))
        scenario_name=$(json_get "$f" "scenario.name")
        echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "${BOLD}  Running: ${scenario_name}${NC}"
        echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

        args=("$f" --base-config "$BASE_CONFIG" --log-dir "$LOG_DIR")
        [[ -n "$TIMEOUT_OVERRIDE" ]] && args+=(--timeout "$TIMEOUT_OVERRIDE")
        [[ -n "$IPC_BACKEND" ]] && args+=(--ipc "$IPC_BACKEND")
        [[ -n "$GUI_FLAG" ]] && args+=("$GUI_FLAG")
        [[ -n "$JSON_LOGS" ]] && args+=(--json-logs)
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

[[ -n "$TIMEOUT_OVERRIDE" ]] && SCENARIO_TIMEOUT="$TIMEOUT_OVERRIDE"

# Gazebo scenarios need longer timeouts (PX4 boot + takeoff = ~20s)
# Enforce a minimum of 120s unless explicitly overridden
if [[ -z "$TIMEOUT_OVERRIDE" ]] && [[ -n "$SCENARIO_TIMEOUT" ]] && [[ "$SCENARIO_TIMEOUT" -lt 120 ]] 2>/dev/null; then
    echo -e "  ${YELLOW}Bumping timeout from ${SCENARIO_TIMEOUT}s to 120s (Gazebo minimum)${NC}"
    SCENARIO_TIMEOUT=120
fi

# Sanitize SCENARIO_NAME
SCENARIO_NAME_SAFE=$(echo "$SCENARIO_NAME" | tr -cd 'A-Za-z0-9_-')
if [[ "$SCENARIO_NAME_SAFE" != "$SCENARIO_NAME" ]]; then
    echo -e "${YELLOW}WARNING: Scenario name sanitized: '${SCENARIO_NAME}' → '${SCENARIO_NAME_SAFE}'${NC}"
    SCENARIO_NAME="$SCENARIO_NAME_SAFE"
fi
if [[ -z "$SCENARIO_NAME" ]]; then
    echo -e "${RED}ERROR: Scenario name is empty after sanitization${NC}"
    exit 2
fi

RUN_START_EPOCH=$(date +%s)
SCENARIO_LOG_DIR=$(create_run_dir "$LOG_DIR" "$SCENARIO_NAME")

echo ""
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}  (Gazebo SITL)${NC}"
echo -e "  ${SCENARIO_DESC}"
echo -e "  Tier     : ${SCENARIO_TIER}"
echo -e "  Timeout  : ${SCENARIO_TIMEOUT}s"
echo -e "  PX4      : ${PX4_DIR}"
echo -e "  World    : $(basename "$GZ_WORLD")"
echo -e "  GUI      : ${GUI_FLAG:-off}"
echo -e "  Logs     : ${SCENARIO_LOG_DIR}"
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# ── Merge configs ─────────────────────────────────────────────
MERGED_CONFIG="${SCENARIO_LOG_DIR}/merged_config.json"
echo ""
echo "Phase 1: Merging configs..."
merge_configs "$BASE_CONFIG" "$SCENARIO_FILE" "$MERGED_CONFIG"

# Apply --ipc override
if [[ -n "$IPC_BACKEND" ]]; then
    python3 - "$MERGED_CONFIG" "$IPC_BACKEND" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    cfg = json.load(f)
cfg['ipc_backend'] = sys.argv[2]
with open(sys.argv[1], 'w') as f:
    json.dump(cfg, f, indent=4)
PYEOF
fi
echo -e "  ${GREEN}✓${NC} Config merged → ${MERGED_CONFIG} (ipc=${IPC_BACKEND:-default})"

# Resolve effective IPC backend
EFFECTIVE_IPC="${IPC_BACKEND}"
if [[ -z "$EFFECTIVE_IPC" ]]; then
    EFFECTIVE_IPC=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('ipc_backend','shm'))" "$MERGED_CONFIG" 2>/dev/null || echo "shm")
fi

# ── Dry run ───────────────────────────────────────────────────
if [[ "$DRY_RUN" == "true" ]]; then
    echo ""
    echo -e "${YELLOW}DRY RUN — showing execution plan:${NC}"
    echo ""
    echo "Base config    : ${BASE_CONFIG}"
    echo "Scenario file  : ${SCENARIO_FILE}"
    echo "Merged config  : ${MERGED_CONFIG}"
    echo "Launcher       : deploy/launch_gazebo.sh"
    echo "PX4 model      : x500_companion"
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

# Ensure system libstdc++ is used (arch-conditional for cross-platform support)
SYS_ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
case "$SYS_ARCH" in
    amd64|x86_64)  SYS_LIB_DIR="/usr/lib/x86_64-linux-gnu" ;;
    arm64|aarch64) SYS_LIB_DIR="/usr/lib/aarch64-linux-gnu" ;;
    *)             SYS_LIB_DIR="" ;;
esac
if [[ -n "$SYS_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="${SYS_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# ── Launch PX4 + Gazebo + companion stack ─────────────────────
echo ""
echo "Phase 2: Launching PX4 SITL + Gazebo + companion stack..."

# Clean stale SHM
rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
      /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health /dev/shm/fc_commands /dev/shm/mission_upload \
      /dev/shm/fault_overrides 2>/dev/null || true

# Kill any leftover PX4/Gazebo processes from a previous run
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
# Only kill PX4 process if it's our SITL instance (not system services)
pkill -9 -f "px4.*sitl" 2>/dev/null || true
sleep 1

cleanup_scenario() {
    echo ""
    echo "Cleaning up Gazebo scenario..."
    # If run directory still has _RUNNING suffix, rename to _ABORTED
    if [[ -n "${SCENARIO_LOG_DIR:-}" && "$SCENARIO_LOG_DIR" == *_RUNNING ]]; then
        SCENARIO_LOG_DIR=$(abort_run_dir "$SCENARIO_LOG_DIR")
    fi
    if [[ -n "$LAUNCHER_PID" ]] && kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        kill -SIGINT "$LAUNCHER_PID" 2>/dev/null || true
        sleep 3
        if kill -0 "$LAUNCHER_PID" 2>/dev/null; then
            kill -SIGKILL "$LAUNCHER_PID" 2>/dev/null || true
        fi
    fi
    # Send SIGTERM to companion processes, PX4, and Gazebo
    pkill -f "build/bin/video_capture" 2>/dev/null || true
    pkill -f "build/bin/perception" 2>/dev/null || true
    pkill -f "build/bin/slam_vio_nav" 2>/dev/null || true
    pkill -f "build/bin/mission_planner" 2>/dev/null || true
    pkill -f "build/bin/comms" 2>/dev/null || true
    pkill -f "build/bin/payload_manager" 2>/dev/null || true
    pkill -f "build/bin/system_monitor" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz" 2>/dev/null || true
    pkill -f "px4.*sitl" 2>/dev/null || true
    sleep 2
    # Force kill any stragglers that ignored SIGTERM
    pkill -9 -f "build/bin/video_capture" 2>/dev/null || true
    pkill -9 -f "build/bin/perception" 2>/dev/null || true
    pkill -9 -f "build/bin/slam_vio_nav" 2>/dev/null || true
    pkill -9 -f "build/bin/mission_planner" 2>/dev/null || true
    pkill -9 -f "build/bin/comms" 2>/dev/null || true
    pkill -9 -f "build/bin/payload_manager" 2>/dev/null || true
    pkill -9 -f "build/bin/system_monitor" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "px4.*sitl" 2>/dev/null || true
    wait "$LAUNCHER_PID" 2>/dev/null || true
    # Wait until UDP port 14540 is fully released (prevents stale-socket race)
    for _pw in $(seq 1 30); do
        if ! ss -ulnp 2>/dev/null | grep -q ":14540"; then
            break
        fi
        sleep 0.5
    done
    # Clean SHM
    rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
          /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
          /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
          /dev/shm/system_health /dev/shm/fc_commands /dev/shm/mission_upload \
          /dev/shm/fault_overrides 2>/dev/null || true
}
trap cleanup_scenario EXIT INT TERM

# Launch via launch_gazebo.sh with merged config
LOG_DIR="$SCENARIO_LOG_DIR" \
CONFIG_FILE="$MERGED_CONFIG" \
PX4_DIR="$PX4_DIR" \
GZ_WORLD="$GZ_WORLD" \
    "${DEPLOY_DIR}/launch_gazebo.sh" $GUI_FLAG $JSON_LOGS \
    > "${SCENARIO_LOG_DIR}/launcher.log" 2>&1 &
LAUNCHER_PID=$!

echo -e "  Launcher PID: ${LAUNCHER_PID}"

# Wait for PX4 MAVLink + companion stack startup
echo -n "  Waiting for PX4 + stack startup"
STARTUP_OK=false
for _ in $(seq 1 60); do
    if ! kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        echo ""
        echo -e "  ${RED}Launcher exited prematurely${NC}"
        break
    fi
    # Check if all 7 companion processes are running
    if pgrep -f "build/bin/mission_planner" > /dev/null 2>&1 && \
       pgrep -f "build/bin/comms" > /dev/null 2>&1 && \
       pgrep -f "build/bin/system_monitor" > /dev/null 2>&1; then
        STARTUP_OK=true
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

if [[ "$STARTUP_OK" == "true" ]]; then
    check "PX4 + Gazebo + Stack started" 0
    # Extra settle time for PX4 to fully boot and start publishing telemetry
    echo -e "  ${CYAN}Settling (5s) — waiting for MAVLink telemetry...${NC}"
    sleep 5
else
    check "PX4 + Gazebo + Stack started" 1
    echo -e "${RED}  Stack failed to start. Check:${NC}"
    echo -e "${RED}    ${SCENARIO_LOG_DIR}/launcher.log${NC}"
    echo -e "${RED}    ${SCENARIO_LOG_DIR}/px4_sitl.log${NC}"
fi

# ── Timeout enforcement ──────────────────────────────────────
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
        check "Fault injector available" 1
    fi
else
    echo -e "  ${CYAN}(no fault injection steps for this scenario)${NC}"
fi

# ── Phase 4: Collection window ────────────────────────────────
check_deadline
COLLECTION_TIME=10
# Use remaining timeout budget for collection (leave 10s for verification)
if [[ -n "$SCENARIO_TIMEOUT" && "$SCENARIO_TIMEOUT" -gt 0 ]] 2>/dev/null; then
    ELAPSED=$(( SECONDS - SCENARIO_START ))
    REMAINING=$(( SCENARIO_TIMEOUT - ELAPSED - 10 ))
    if [[ $REMAINING -gt $COLLECTION_TIME ]]; then
        COLLECTION_TIME=$REMAINING
    fi
fi
echo ""
echo "Phase 4: Post-injection collection (${COLLECTION_TIME}s)..."
sleep "$COLLECTION_TIME"

# ── Phase 5: Verification ────────────────────────────────────
check_deadline
echo ""
echo "Phase 5: Verification..."

# Gazebo launcher writes individual log files.  Combine them for
# pattern matching.  Also include the launcher wrapper log.
COMBINED_LOG="${SCENARIO_LOG_DIR}/combined.log"
cat "${SCENARIO_LOG_DIR}"/*.log > "$COMBINED_LOG" 2>/dev/null || true

# Check log_contains
echo ""
echo "Log-contains checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qai "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log contains: ${pattern}" 0
    else
        check "Log missing: ${pattern}" 1
        if [[ "$VERBOSE" == "true" ]]; then
            echo -e "    ${YELLOW}(searched $(wc -l < "$COMBINED_LOG") lines across all logs)${NC}"
        fi
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.log_contains")

# Check log_must_not_contain
echo ""
echo "Log-must-not-contain checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qai "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log unexpectedly contains: ${pattern}" 1
        if [[ "$VERBOSE" == "true" ]]; then
            echo -e "    ${YELLOW}First match:${NC}"
            grep -ai "$pattern" "$COMBINED_LOG" | head -1 | sed 's/^/    /'
        fi
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

# Check SHM segments (only meaningful for SHM backend)
echo ""
echo "SHM segment checks:"
if [[ "$EFFECTIVE_IPC" == "zenoh" ]]; then
    echo "  (skipped — Zenoh transport does not use POSIX SHM)"
else
    while read -r seg; do
        [[ -z "$seg" ]] && continue
        if [[ -f "/dev/shm/${seg}" ]]; then
            check "SHM: ${seg}" 0
        else
            check "SHM missing: ${seg}" 1
        fi
    done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.shm_segments_exist")
fi

# PX4-specific checks
echo ""
echo "PX4 SITL checks:"
if pgrep -f "px4" > /dev/null 2>&1; then
    check "PX4 process alive" 0
else
    check "PX4 process alive" 1
fi

# ── Results ───────────────────────────────────────────────────
echo ""
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}  (Gazebo SITL)${NC}"
echo -e "  Logs: ${SCENARIO_LOG_DIR}"
echo -e "  Results: ${PASS} passed, ${FAIL} failed, ${TOTAL} total"
if [[ $FAIL -gt 0 ]]; then
    echo -e "  ${RED}FAILED${NC}"
else
    echo -e "  ${GREEN}PASSED${NC}"
fi
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# ── Finalize run: rename dir, generate report, update index ──
RUN_END_EPOCH=$(date +%s)
SCENARIO_LOG_DIR=$(finalize_run_dir "$SCENARIO_LOG_DIR" "$PASS" "$FAIL")

SCENARIO_BASE_DIR="${LOG_DIR}/${SCENARIO_NAME}"
update_latest_symlink "$SCENARIO_BASE_DIR" "$(basename "$SCENARIO_LOG_DIR")"

write_run_metadata "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "$SCENARIO_NAME" "gazebo" "$SCENARIO_FILE" \
    "$RUN_START_EPOCH" "$RUN_END_EPOCH" \
    "$PASS" "$FAIL" "$TOTAL" "$PROJECT_DIR"

REPORT_FILE=$(generate_run_report "$SCENARIO_LOG_DIR" "$SCENARIO_NAME" "gazebo" \
    "$PASS" "$FAIL" "$TOTAL" "$SCENARIO_FILE" "$PROJECT_DIR")

append_to_index "${LOG_DIR}/runs.jsonl" \
    "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "${SCENARIO_NAME}/$(basename "$SCENARIO_LOG_DIR")"

echo ""
echo -e "  ${CYAN}Report : ${REPORT_FILE}${NC}"
echo -e "  ${CYAN}Logs   : ${SCENARIO_LOG_DIR}${NC}"

if [[ $FAIL -gt 0 ]]; then
    exit 1
elif ! kill -0 "$LAUNCHER_PID" 2>/dev/null; then
    echo -e "${RED}Launcher process died unexpectedly${NC}"
    exit 1
else
    exit 0
fi
