#!/usr/bin/env bash
# tests/run_scenario_cosys.sh
# ═══════════════════════════════════════════════════════════════
# Tier 3 Cosys-AirSim (UE5) Scenario Runner
#
# Launches Cosys-AirSim UE5 + companion stack, injects faults,
# and verifies pass criteria against real RPC sensors.
#
# Usage:
#   ./tests/run_scenario_cosys.sh <scenario_json>
#   ./tests/run_scenario_cosys.sh config/scenarios/29_cosys_perception.json
#   ./tests/run_scenario_cosys.sh --list
#   ./tests/run_scenario_cosys.sh --all
#   ./tests/run_scenario_cosys.sh config/scenarios/29_cosys_perception.json --gui --verbose
#
# Options:
#   --base-config <path>    Base config (default: config/cosys_airsim.json)
#   --log-dir <path>        Log output directory (default: drone_logs/scenarios_cosys)
#   --timeout <seconds>     Override scenario timeout
#   --dry-run               Parse scenario, show plan, don't execute
#   --verbose               Extra verbose output
#   --gui                   Launch UE5 with a visible 3D window (default: headless)
#   --json-logs             Enable structured JSON log output
#   --ipc <shm|zenoh>       Override IPC backend (default: from base config)
#   --env <name>            UE5 environment name (default: Blocks)
#
# Environment variables:
#   COSYS_DIR    Path to Cosys-AirSim (default: third_party/cosys-airsim if present)
#   UE5_DIR      Path to UnrealEngine   (default: /opt/UnrealEngine)
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
DEFAULT_BASE_CONFIG="${PROJECT_DIR}/config/cosys_airsim.json"
DEFAULT_LOG_DIR="${PROJECT_DIR}/drone_logs/scenarios_cosys"
COSYS_RPC_PORT="${COSYS_RPC_PORT:-41451}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_PID=""

# ── Cosys-AirSim / UE5 paths ─────────────────────────────────
if [[ -d "${PROJECT_DIR}/third_party/cosys-airsim" ]]; then
    COSYS_DIR="${COSYS_DIR:-${PROJECT_DIR}/third_party/cosys-airsim}"
else
    COSYS_DIR="${COSYS_DIR:-${HOME}/Cosys-AirSim}"
fi
UE5_DIR="${UE5_DIR:-/opt/UnrealEngine}"
UE5_ENV="Blocks"

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
GUI=false
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
        --gui)          GUI=true ;;
        --json-logs)    JSON_LOGS="--json-logs" ;;
        --env)          UE5_ENV="$2"; shift ;;
        --ipc)
            IPC_BACKEND="$2"
            if [[ "$IPC_BACKEND" != "shm" && "$IPC_BACKEND" != "zenoh" ]]; then
                echo -e "${RED}ERROR: --ipc must be 'shm' or 'zenoh'${NC}"
                exit 2
            fi
            shift ;;
        -h|--help)
            sed -n '2,30p' "$0"
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
        print(json.dumps(item) if isinstance(item, dict) else item)
PYEOF
}

merge_configs() {
    # Three-layer merge: default.json → cosys overlay → scenario config_overrides
    # cosys_airsim.json is an overlay, not a complete config — must start from default.json
    local cosys_overlay="$1"
    local scenario="$2"
    local output="$3"
    local default_cfg="${PROJECT_DIR}/config/default.json"
    python3 - "$default_cfg" "$cosys_overlay" "$scenario" "$output" <<'PYEOF'
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
    merged = json.load(f)           # layer 1: default.json
with open(sys.argv[2]) as f:
    cosys_cfg = json.load(f)
merged = deep_merge(merged, cosys_cfg)  # layer 2: cosys_airsim.json overlay
with open(sys.argv[3]) as f:
    scenario_cfg = json.load(f)
overrides = scenario_cfg.get('config_overrides', {})
merged = deep_merge(merged, overrides)  # layer 3: scenario config_overrides

with open(sys.argv[4], 'w') as f:
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

# ── Prerequisite checks ───────────────────────────────────────
check_prerequisites() {
    local ok=true
    if [[ ! -d "$COSYS_DIR" ]]; then
        echo -e "${RED}ERROR: Cosys-AirSim not found at ${COSYS_DIR}${NC}"
        echo -e "${RED}       Set COSYS_DIR or init submodule:${NC}"
        echo -e "${RED}         git submodule update --init third_party/cosys-airsim${NC}"
        ok=false
    fi
    if ! command -v nvidia-smi &>/dev/null; then
        echo -e "${RED}ERROR: nvidia-smi not found — NVIDIA GPU required for UE5${NC}"
        ok=false
    fi
    if [[ ! -d "$BIN_DIR" ]]; then
        echo -e "${RED}ERROR: Build directory not found. Run: bash deploy/build.sh${NC}"
        ok=false
    fi
    if [[ ! -f "$BASE_CONFIG" ]]; then
        echo -e "${RED}ERROR: Base config not found: ${BASE_CONFIG}${NC}"
        ok=false
    fi
    [[ "$ok" == "false" ]] && exit 2
}

# ── Find UE5 binary ───────────────────────────────────────────
find_ue5_binary() {
    local env_name="$1"
    local candidates=(
        "${COSYS_DIR}/Unreal/Environments/${env_name}/Binaries/Linux/${env_name}-Linux-Shipping"
        "${COSYS_DIR}/Unreal/Environments/${env_name}/Binaries/Linux/${env_name}"
        "${COSYS_DIR}/${env_name}/LinuxNoEditor/${env_name}.sh"
        "${COSYS_DIR}/${env_name}/${env_name}.sh"
    )
    for candidate in "${candidates[@]}"; do
        if [[ -x "$candidate" ]]; then
            echo "$candidate"
            return 0
        fi
    done

    # No shipping binary — use UE5 editor to run the project
    local editor="${UE5_DIR}/Engine/Binaries/Linux/UnrealEditor"
    local uproject="${COSYS_DIR}/Unreal/Environments/${env_name}/${env_name}.uproject"
    if [[ -x "$editor" && -f "$uproject" ]]; then
        echo "editor:${editor}:${uproject}"
        return 0
    fi

    return 1
}

# ── List scenarios ────────────────────────────────────────────
list_scenarios() {
    echo -e "${BOLD}Available Scenarios (Tier 3 — Cosys-AirSim / UE5):${NC}"
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
    echo "Tier 3 scenarios require UE5 + Cosys-AirSim (GPU required)."
    echo ""
}

if [[ "$LIST_ONLY" == "true" ]]; then
    list_scenarios
    exit 0
fi

# ── Validate inputs ───────────────────────────────────────────
if [[ "$RUN_ALL" == "false" && -z "$SCENARIO_FILE" ]]; then
    echo -e "${RED}ERROR: Specify a scenario file or --all / --list${NC}"
    echo "Usage: $0 <scenario_json> [options]"
    exit 2
fi

check_prerequisites

# ── Run all scenarios ─────────────────────────────────────────
if [[ "$RUN_ALL" == "true" ]]; then
    echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  Cosys-AirSim Scenario Runner — All Scenarios${NC}"
    echo -e "  Cosys-AirSim : ${COSYS_DIR}"
    echo -e "  Config       : ${BASE_CONFIG}"
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
        [[ "$GUI" == "true" ]] && args+=(--gui)
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

# UE5 boot is slow — enforce a minimum of 180s
if [[ -z "$TIMEOUT_OVERRIDE" ]] && [[ -n "$SCENARIO_TIMEOUT" ]] && [[ "$SCENARIO_TIMEOUT" -lt 180 ]] 2>/dev/null; then
    echo -e "  ${YELLOW}Bumping timeout from ${SCENARIO_TIMEOUT}s to 180s (UE5 minimum)${NC}"
    SCENARIO_TIMEOUT=180
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
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}  (Cosys-AirSim / UE5)${NC}"
echo -e "  ${SCENARIO_DESC}"
echo -e "  Tier      : ${SCENARIO_TIER}"
echo -e "  Timeout   : ${SCENARIO_TIMEOUT}s"
echo -e "  Cosys-AirSim : ${COSYS_DIR}"
echo -e "  UE5 Env   : ${UE5_ENV}"
echo -e "  GUI       : ${GUI}"
echo -e "  RPC Port  : ${COSYS_RPC_PORT}"
echo -e "  Logs      : ${SCENARIO_LOG_DIR}"
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# ── Merge configs ─────────────────────────────────────────────
MERGED_CONFIG="${SCENARIO_LOG_DIR}/merged_config.json"
echo ""
echo "Phase 1: Merging configs..."
merge_configs "$BASE_CONFIG" "$SCENARIO_FILE" "$MERGED_CONFIG"

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

EFFECTIVE_IPC="${IPC_BACKEND}"
if [[ -z "$EFFECTIVE_IPC" ]]; then
    EFFECTIVE_IPC=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('ipc_backend','zenoh'))" "$MERGED_CONFIG" 2>/dev/null || echo "zenoh")
fi

# ── Dry run ───────────────────────────────────────────────────
if [[ "$DRY_RUN" == "true" ]]; then
    echo ""
    echo -e "${YELLOW}DRY RUN — showing execution plan:${NC}"
    echo ""
    echo "Base config    : ${BASE_CONFIG}"
    echo "Scenario file  : ${SCENARIO_FILE}"
    echo "Merged config  : ${MERGED_CONFIG}"
    echo "UE5 Env        : ${UE5_ENV}"
    echo "GUI            : ${GUI}"
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

# ── System libstdc++ (avoid Anaconda shadowing) ───────────────
SYS_ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
case "$SYS_ARCH" in
    amd64|x86_64)  SYS_LIB_DIR="/usr/lib/x86_64-linux-gnu" ;;
    arm64|aarch64) SYS_LIB_DIR="/usr/lib/aarch64-linux-gnu" ;;
    *)             SYS_LIB_DIR="" ;;
esac
if [[ -n "$SYS_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="${SYS_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

mkdir -p "$SCENARIO_LOG_DIR"

# ── Pre-flight: kill stale sim processes to free ports ────────
# Without this, a zombie UE5/PX4 from a previous run holds TCP 4560 or UDP
# 14540, causing "bind: Address already in use" or "PX4 server not running".
echo ""
echo "Pre-flight: clearing stale simulator processes..."
STALE_COUNT=0
for pat in "UnrealEditor" "Blocks-Linux" "AirSimEnv" "px4_sitl" "build/bin/px4" "PX4-Autopilot/build/px4_sitl"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        STALE_COUNT=$((STALE_COUNT + 1))
        pkill -9 -f "$pat" 2>/dev/null || true
    fi
done
if [[ $STALE_COUNT -gt 0 ]]; then
    sleep 2
    echo -e "  ${YELLOW}Killed ${STALE_COUNT} stale process group(s)${NC}"
fi
# Release stuck TCP/UDP ports (best-effort)
for port in 4560 14540 14541 14550 14580 14581 18570 41451; do
    if ss -tlnp 2>/dev/null | grep -q ":${port} " || ss -ulnp 2>/dev/null | grep -q ":${port} "; then
        echo -e "  ${YELLOW}Port ${port} still held — check manually if scenario fails to bind${NC}"
    fi
done

# ── PIDs ──────────────────────────────────────────────────────
UE5_PID=""
COMPANION_PIDS=()
SCENE_STAMP=""    # Populated if scenario declares scene.file and spawn succeeds

cleanup_scenario() {
    echo ""
    echo "Cleaning up Cosys-AirSim scenario..."
    if [[ -n "${SCENARIO_LOG_DIR:-}" && "$SCENARIO_LOG_DIR" == *_RUNNING ]]; then
        SCENARIO_LOG_DIR=$(abort_run_dir "$SCENARIO_LOG_DIR")
    fi
    # Destroy RPC-spawned scene objects while UE5 is still alive.
    # Best-effort: failures are non-fatal (stamp file left behind for manual cleanup).
    if [[ -n "$SCENE_STAMP" && -f "$SCENE_STAMP" ]] \
       && [[ -x "${BIN_DIR}/cosys_populate_scene" ]] \
       && [[ -n "$UE5_PID" ]] && kill -0 "$UE5_PID" 2>/dev/null; then
        echo "  Destroying spawned scene objects..."
        "${BIN_DIR}/cosys_populate_scene" destroy --stamp "$SCENE_STAMP" \
            >> "${SCENARIO_LOG_DIR}/scene_populate.log" 2>&1 || true
    fi
    # Stop companion processes
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${COMPANION_PIDS[@]}"; do
        kill -SIGKILL "$pid" 2>/dev/null || true
    done
    # Stop UE5
    if [[ -n "$UE5_PID" ]] && kill -0 "$UE5_PID" 2>/dev/null; then
        kill -SIGTERM "$UE5_PID" 2>/dev/null || true
        sleep 3
        kill -SIGKILL "$UE5_PID" 2>/dev/null || true
    fi
    pkill -f "build/bin/video_capture"   2>/dev/null || true
    pkill -f "build/bin/perception"      2>/dev/null || true
    pkill -f "build/bin/slam_vio_nav"    2>/dev/null || true
    pkill -f "build/bin/mission_planner" 2>/dev/null || true
    pkill -f "build/bin/comms"           2>/dev/null || true
    pkill -f "build/bin/payload_manager" 2>/dev/null || true
    pkill -f "build/bin/system_monitor"  2>/dev/null || true
    # Stop PX4 SITL
    if [[ -n "$PX4_PID" ]] && kill -0 "$PX4_PID" 2>/dev/null; then
        kill -SIGTERM "$PX4_PID" 2>/dev/null || true
        sleep 2
        kill -SIGKILL "$PX4_PID" 2>/dev/null || true
    fi
    pkill -f "px4.*sitl\|px4_sitl" 2>/dev/null || true
    rm -f /dev/shm/zenoh_shm_* 2>/dev/null || true
    echo "Cleanup complete."
}
trap cleanup_scenario EXIT INT TERM

# ── Phase 2: Launch UE5 ───────────────────────────────────────
echo ""
echo "Phase 2: Launching Cosys-AirSim (UE5 env=${UE5_ENV})..."

# Deploy our AirSim settings so UE5 picks up the correct camera/vehicle config
COSYS_SETTINGS="${PROJECT_DIR}/config/cosys_settings.json"
AIRSIM_SETTINGS_DIR="${HOME}/Documents/AirSim"
mkdir -p "$AIRSIM_SETTINGS_DIR"
if [[ -f "$COSYS_SETTINGS" ]]; then
    cp "$COSYS_SETTINGS" "${AIRSIM_SETTINGS_DIR}/settings.json"
    # When running with --gui, override ViewMode so the drone is visible in the viewport.
    # NoDisplay (headless) shows a black screen with no camera feed.
    if [[ "$GUI" == "true" ]]; then
        python3 -c "
import json, sys
with open(sys.argv[1]) as f:
    s = json.load(f)
s['ViewMode'] = 'FlyWithMe'
with open(sys.argv[1], 'w') as f:
    json.dump(s, f, indent=4)
" "${AIRSIM_SETTINGS_DIR}/settings.json"
    fi
    echo -e "  ${GREEN}✓${NC} AirSim settings deployed to ${AIRSIM_SETTINGS_DIR}/settings.json (ViewMode=$(python3 -c \"import json; print(json.load(open('${AIRSIM_SETTINGS_DIR}/settings.json')).get('ViewMode','default'))\"))"
else
    echo -e "  ${YELLOW}WARNING: ${COSYS_SETTINGS} not found — using existing settings.json${NC}"
fi
# Deploy materials.csv (silences stencil initialization warning; header-only = no GPU lidar materials)
COSYS_MATERIALS="${PROJECT_DIR}/config/materials.csv"
if [[ -f "$COSYS_MATERIALS" ]]; then
    cp "$COSYS_MATERIALS" "${AIRSIM_SETTINGS_DIR}/materials.csv"
    echo -e "  ${GREEN}✓${NC} materials.csv deployed"
elif [[ ! -f "${AIRSIM_SETTINGS_DIR}/materials.csv" ]]; then
    echo "material_name,material_id,r,g,b" > "${AIRSIM_SETTINGS_DIR}/materials.csv"
    echo -e "  ${GREEN}✓${NC} materials.csv created (empty)"
fi

UE5_BIN_INFO=$(find_ue5_binary "$UE5_ENV") || {
    echo -e "${RED}ERROR: Cannot find UE5 binary for env '${UE5_ENV}'${NC}"
    echo -e "${RED}       COSYS_DIR=${COSYS_DIR}${NC}"
    echo -e "${RED}       UE5_DIR=${UE5_DIR}${NC}"
    exit 2
}

IS_EDITOR_MODE=false
if [[ "$UE5_BIN_INFO" == editor:* ]]; then
    # Editor mode: UnrealEditor <project.uproject> -game
    # -game makes the editor auto-start the level (no Play button needed)
    IFS=':' read -r _tag ue5_editor ue5_uproject <<< "$UE5_BIN_INFO"
    UE5_CMD=("$ue5_editor" "$ue5_uproject")
    IS_EDITOR_MODE=true
else
    UE5_CMD=("$UE5_BIN_INFO")
fi

UE5_ARGS=()
if [[ "$IS_EDITOR_MODE" == "true" ]]; then
    # -game: auto-starts the level (equivalent to pressing the Play button)
    UE5_ARGS+=(-game)
fi
if [[ "$GUI" == "false" ]]; then
    UE5_ARGS+=(-RenderOffScreen)
else
    UE5_ARGS+=(-windowed -ResX=1280 -ResY=720)
fi

echo -e "  Binary: ${UE5_CMD[*]}"
echo -e "  Args  : ${UE5_ARGS[*]:-none}"

"${UE5_CMD[@]}" "${UE5_ARGS[@]}" \
    > "${SCENARIO_LOG_DIR}/cosys_ue5.log" 2>&1 &
UE5_PID=$!
echo -e "  ${GREEN}✓${NC} UE5 launched (PID=${UE5_PID})"

# ── Wait for RPC port ─────────────────────────────────────────
echo -n "  Waiting for RPC port ${COSYS_RPC_PORT}"
UE5_READY=false
MAX_WAIT=120
WAITED=0
while [[ $WAITED -lt $MAX_WAIT ]]; do
    if ss -tlnp 2>/dev/null | grep -q ":${COSYS_RPC_PORT}"; then
        UE5_READY=true
        break
    fi
    if ! kill -0 "$UE5_PID" 2>/dev/null; then
        echo ""
        echo -e "  ${RED}ERROR: UE5 process died — check ${SCENARIO_LOG_DIR}/cosys_ue5.log${NC}"
        tail -20 "${SCENARIO_LOG_DIR}/cosys_ue5.log" 2>/dev/null || true
        exit 2
    fi
    echo -n "."
    sleep 1
    WAITED=$((WAITED + 1))
done
echo ""

if [[ "$UE5_READY" == "true" ]]; then
    check "Cosys-AirSim RPC port ${COSYS_RPC_PORT} ready (${WAITED}s)" 0
else
    echo -e "  ${YELLOW}WARNING: RPC port not detected after ${MAX_WAIT}s — continuing anyway${NC}"
    check "Cosys-AirSim RPC port ${COSYS_RPC_PORT} ready" 1
fi

# Additional settle: UE5 needs time to finish loading the level after the port opens
echo -e "  ${CYAN}Settling (5s) — waiting for level load...${NC}"
sleep 5

# ── Wait for HIL TCP port (AirSim server on 4560) ────────────
# AirSim opens the RPC port (41451) during plugin init, but the HIL TCP server
# (port 4560) is only opened once the vehicle is spawned in the game. Launching
# PX4 before 4560 is bound causes simulator_mavlink to fail its initial connect.
echo -n "  Waiting for AirSim HIL TCP port 4560"
HIL_READY=false
for _ in $(seq 1 30); do
    if ss -tlnp 2>/dev/null | grep -q ":4560 "; then
        HIL_READY=true
        break
    fi
    if ! kill -0 "$UE5_PID" 2>/dev/null; then
        echo ""
        echo -e "  ${RED}ERROR: UE5 died before HIL server came up${NC}"
        exit 2
    fi
    echo -n "."
    sleep 1
done
echo ""
if [[ "$HIL_READY" == "true" ]]; then
    check "AirSim HIL TCP port 4560 ready" 0
else
    echo -e "  ${YELLOW}WARNING: HIL TCP port 4560 not detected after 30s — PX4 may fail to connect${NC}"
    echo -e "  ${YELLOW}Hint: in GUI mode, ensure the game is actually playing (press Play if in editor)${NC}"
    check "AirSim HIL TCP port 4560 ready" 1
fi

# ── Launch PX4 SITL ──────────────────────────────────────────
echo ""
echo "Phase 2b: Launching PX4 SITL..."
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_DIR}/build/px4_sitl_default/etc"
PX4_ROOTFS="${PX4_DIR}/build/px4_sitl_default/rootfs"
if [[ ! -x "$PX4_BIN" ]]; then
    echo -e "  ${RED}ERROR: PX4 SITL binary not found at ${PX4_BIN}${NC}"
    echo -e "  ${RED}       Run: cd ${PX4_DIR} && make px4_sitl_default${NC}"
    exit 2
fi
mkdir -p "${PX4_ROOTFS}"
# PX4_SIM_MODEL=none_iris → autostart 10016 (none_iris, no Gazebo). Do NOT set
# PX4_SYS_AUTOSTART — it overrides PX4_SIM_MODEL and 4001 maps to 4001_gz_x500.
export PX4_SIM_MODEL="none_iris" # none = no Gazebo; AirSim provides physics/sensors
export PX4_SIM_HOSTNAME=127.0.0.1
pushd "${PX4_ROOTFS}" > /dev/null
"${PX4_BIN}" \
    -d "${PX4_ETC}" \
    -s "${PX4_ETC}/init.d-posix/rcS" \
    > "${SCENARIO_LOG_DIR}/px4_sitl.log" 2>&1 &
PX4_PID=$!
popd > /dev/null
echo -e "  ${GREEN}✓${NC} PX4 SITL launched (PID=${PX4_PID})"

# Wait for PX4 MAVLink UDP port 14540
echo -n "  Waiting for PX4 MAVLink port 14540"
PX4_READY=false
for _ in $(seq 1 30); do
    if ss -ulnp 2>/dev/null | grep -q ":14540"; then
        PX4_READY=true
        break
    fi
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo ""
        echo -e "  ${RED}ERROR: PX4 died — check ${SCENARIO_LOG_DIR}/px4_sitl.log${NC}"
        tail -10 "${SCENARIO_LOG_DIR}/px4_sitl.log" 2>/dev/null || true
        exit 2
    fi
    echo -n "."
    sleep 1
done
echo ""
if [[ "$PX4_READY" == "true" ]]; then
    check "PX4 SITL MAVLink port 14540 ready" 0
else
    echo -e "  ${YELLOW}WARNING: MAVLink port not detected after 30s — continuing${NC}"
    check "PX4 SITL MAVLink port 14540 ready" 1
fi

# ── Phase 2c: Populate scene (RPC-spawned obstacles) ─────────
# If the scenario declares `scene.file`, call cosys_populate_scene to spawn
# the listed assets via simSpawnObject RPC. Part of epic #480 (proving-ground).
# Non-fatal: if the binary is missing or UE5 rejects a spawn, we log WARN and
# continue — the scenario can still pass with a subset of objects.
SCENE_FILE=$(json_get "$SCENARIO_FILE" "scenario.scene.file")
if [[ -n "$SCENE_FILE" && "$SCENE_FILE" != "None" ]]; then
    # Resolve relative scene paths against PROJECT_DIR
    if [[ "$SCENE_FILE" != /* ]]; then
        SCENE_FILE="${PROJECT_DIR}/${SCENE_FILE}"
    fi
    echo ""
    echo "Phase 2c: Populating scene from ${SCENE_FILE}..."
    if [[ ! -f "$SCENE_FILE" ]]; then
        echo -e "  ${YELLOW}WARNING: scene file not found — skipping spawn${NC}"
    elif [[ ! -x "${BIN_DIR}/cosys_populate_scene" ]]; then
        echo -e "  ${YELLOW}WARNING: ${BIN_DIR}/cosys_populate_scene not built — skipping spawn${NC}"
        echo -e "  ${YELLOW}         Rebuild with Cosys-AirSim support to enable populated scenes.${NC}"
    else
        SCENE_STAMP="${SCENARIO_LOG_DIR}/spawned.txt"
        if "${BIN_DIR}/cosys_populate_scene" spawn \
                --scene "$SCENE_FILE" --stamp "$SCENE_STAMP" \
                > "${SCENARIO_LOG_DIR}/scene_populate.log" 2>&1; then
            SPAWNED_COUNT=$(wc -l < "$SCENE_STAMP" 2>/dev/null || echo 0)
            echo -e "  ${GREEN}✓${NC} Spawned ${SPAWNED_COUNT} scene objects"
        else
            echo -e "  ${YELLOW}WARNING: scene populate failed — see scene_populate.log${NC}"
            tail -5 "${SCENARIO_LOG_DIR}/scene_populate.log" 2>/dev/null || true
            SCENE_STAMP=""  # Don't try to destroy nothing on cleanup
        fi
    fi
fi

# ── Phase 3: Launch companion stack ──────────────────────────
echo ""
echo "Phase 3: Launching 7 companion processes..."

# Clean stale SHM
rm -f /dev/shm/zenoh_shm_* /dev/shm/drone_* \
      /dev/shm/detected_objects /dev/shm/slam_pose \
      /dev/shm/mission_status /dev/shm/trajectory_cmd \
      /dev/shm/payload_commands /dev/shm/fc_state \
      /dev/shm/gcs_commands /dev/shm/payload_status \
      /dev/shm/system_health /dev/shm/fc_commands \
      /dev/shm/mission_upload /dev/shm/fault_overrides 2>/dev/null || true

CONFIG_ARG="--config ${MERGED_CONFIG}"

echo "  [1/7] system_monitor"
"${BIN_DIR}/system_monitor" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/system_monitor.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [2/7] video_capture"
"${BIN_DIR}/video_capture" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/video_capture.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [3/7] comms"
"${BIN_DIR}/comms" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/comms.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 1

echo "  [4/7] perception"
"${BIN_DIR}/perception" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/perception.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [5/7] slam_vio_nav"
"${BIN_DIR}/slam_vio_nav" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/slam_vio_nav.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [6/7] mission_planner"
"${BIN_DIR}/mission_planner" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/mission_planner.log" 2>&1 &
COMPANION_PIDS+=($!)
sleep 0.3

echo "  [7/7] payload_manager"
"${BIN_DIR}/payload_manager" ${CONFIG_ARG} \
    > "${SCENARIO_LOG_DIR}/payload_manager.log" 2>&1 &
COMPANION_PIDS+=($!)

echo ""

# Wait for the stack to be up
echo -n "  Waiting for stack startup"
STARTUP_OK=false
for _ in $(seq 1 30); do
    if ! kill -0 "$UE5_PID" 2>/dev/null; then
        echo ""
        echo -e "  ${RED}UE5 crashed during stack startup${NC}"
        break
    fi
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
    check "Companion stack started (7 processes)" 0
    echo -e "  ${CYAN}Settling (5s) — waiting for RPC handshake...${NC}"
    sleep 5
else
    check "Companion stack started (7 processes)" 1
    echo -e "${RED}  Stack failed to start. Check logs in: ${SCENARIO_LOG_DIR}/${NC}"
fi

# ── Timeout enforcement ───────────────────────────────────────
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

# ── Phase 4: Fault injection ──────────────────────────────────
check_deadline
echo ""
echo "Phase 4: Executing fault injection sequence..."

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
        "${FAULT_INJECTOR}" sequence "$FAULT_SEQ_FILE" 2>&1 | tee "${SCENARIO_LOG_DIR}/fault_injector.log"
        check "Fault injection sequence completed" $?
    else
        echo -e "  ${YELLOW}WARNING: fault_injector not found at ${FAULT_INJECTOR}${NC}"
        check "Fault injector available" 1
    fi
else
    echo -e "  ${CYAN}(no fault injection steps for this scenario)${NC}"
fi

# ── Phase 5: Collection window ────────────────────────────────
check_deadline
COLLECTION_TIME=10
if [[ -n "$SCENARIO_TIMEOUT" && "$SCENARIO_TIMEOUT" -gt 0 ]] 2>/dev/null; then
    ELAPSED=$(( SECONDS - SCENARIO_START ))
    REMAINING=$(( SCENARIO_TIMEOUT - ELAPSED - 10 ))
    if [[ $REMAINING -gt $COLLECTION_TIME ]]; then
        COLLECTION_TIME=$REMAINING
    fi
fi
echo ""
echo "Phase 5: Post-injection collection (${COLLECTION_TIME}s)..."
sleep "$COLLECTION_TIME"

# ── Phase 6: Verification ─────────────────────────────────────
check_deadline
echo ""
echo "Phase 6: Verification..."

COMBINED_LOG="${SCENARIO_LOG_DIR}/combined.log"
cat "${SCENARIO_LOG_DIR}"/*.log > "$COMBINED_LOG" 2>/dev/null || true

# Also pull UE5 console log for getCamera error messages
UE5_LOG_DIR="${HOME}/.config/Epic/UnrealEngine/5.4/Saved/Logs"
if [[ -d "$UE5_LOG_DIR" ]]; then
    ls -t "${UE5_LOG_DIR}"/*.log 2>/dev/null | head -1 | xargs -I{} cat {} >> "$COMBINED_LOG" 2>/dev/null || true
fi

echo ""
echo "Log-contains checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qai "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log contains: ${pattern}" 0
    else
        check "Log missing: ${pattern}" 1
        if [[ "$VERBOSE" == "true" ]]; then
            echo -e "    ${YELLOW}(searched $(wc -l < "$COMBINED_LOG") lines)${NC}"
        fi
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.log_contains")

echo ""
echo "Log-must-not-contain checks:"
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qai "$pattern" "$COMBINED_LOG" 2>/dev/null; then
        check "Log unexpectedly contains: ${pattern}" 1
        if [[ "$VERBOSE" == "true" ]]; then
            grep -ai "$pattern" "$COMBINED_LOG" | head -1 | sed 's/^/    /'
        fi
    else
        check "Log correctly does NOT contain: ${pattern}" 0
    fi
done < <(json_get_array "$SCENARIO_FILE" "pass_criteria.log_must_not_contain")

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

echo ""
echo "UE5 / RPC checks:"
if kill -0 "$UE5_PID" 2>/dev/null; then
    check "Cosys-AirSim (UE5) still running" 0
else
    check "Cosys-AirSim (UE5) still running" 1
fi
if ss -tlnp 2>/dev/null | grep -q ":${COSYS_RPC_PORT}"; then
    check "RPC port ${COSYS_RPC_PORT} still open" 0
else
    check "RPC port ${COSYS_RPC_PORT} still open" 1
fi

# ── Results ───────────────────────────────────────────────────
echo ""
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "  ${BOLD}Scenario: ${SCENARIO_NAME}  (Cosys-AirSim / UE5)${NC}"
echo -e "  Logs: ${SCENARIO_LOG_DIR}"
echo -e "  Results: ${PASS} passed, ${FAIL} failed, ${TOTAL} total"
if [[ $FAIL -gt 0 ]]; then
    echo -e "  ${RED}FAILED${NC}"
else
    echo -e "  ${GREEN}PASSED${NC}"
fi
echo -e "${BOLD}════════════════════════════════════════════════════${NC}"

# ── Finalize ──────────────────────────────────────────────────
RUN_END_EPOCH=$(date +%s)
SCENARIO_LOG_DIR=$(finalize_run_dir "$SCENARIO_LOG_DIR" "$PASS" "$FAIL")

SCENARIO_BASE_DIR="${LOG_DIR}/${SCENARIO_NAME}"
update_latest_symlink "$SCENARIO_BASE_DIR" "$(basename "$SCENARIO_LOG_DIR")"

write_run_metadata "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "$SCENARIO_NAME" "cosys_airsim" "$SCENARIO_FILE" \
    "$RUN_START_EPOCH" "$RUN_END_EPOCH" \
    "$PASS" "$FAIL" "$TOTAL" "$PROJECT_DIR"

REPORT_FILE=$(generate_run_report "$SCENARIO_LOG_DIR" "$SCENARIO_NAME" "cosys_airsim" \
    "$PASS" "$FAIL" "$TOTAL" "$SCENARIO_FILE" "$PROJECT_DIR")

append_to_index "${LOG_DIR}/runs.jsonl" \
    "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "${SCENARIO_NAME}/$(basename "$SCENARIO_LOG_DIR")"

echo ""
echo -e "  ${CYAN}Report : ${REPORT_FILE}${NC}"
echo -e "  ${CYAN}Logs   : ${SCENARIO_LOG_DIR}${NC}"

if [[ $FAIL -gt 0 ]]; then
    exit 1
else
    exit 0
fi
