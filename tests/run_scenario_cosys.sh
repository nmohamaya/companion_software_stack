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
#   --base-config <path>    Base config (default: resolved from the scenario).
#                           Resolution precedence:
#                             1. this flag, if given
#                             2. scenario.base_config inside the scenario JSON
#                             3. config/cosys_airsim_dev.json, if it exists
#                             4. config/cosys_airsim.json (cloud profile fallback)
#                           Paths must stay within PROJECT_DIR (no ../ escape).
#   --log-dir <path>        Log output directory (default: drone_logs/scenarios_cosys)
#   --timeout <seconds>     Override scenario timeout
#   --dry-run               Parse scenario, show plan, don't execute
#   --verbose               Extra verbose output
#   --gui                   Launch UE5 with a visible 3D window (default: headless)
#   --json-logs             Enable structured JSON log output
#   --ipc <shm|zenoh>       Override IPC backend (default: from base config)
#   --env <name>            UE5 environment name (default: Blocks)
#
# Scenario JSON fields (auto-detected by the runner):
#   scenario.base_config    Optional hint pointing at the base config overlay
#                           that best matches this scenario (e.g.
#                           "config/cosys_airsim_dev.json" for the dev profile).
#                           Lets a scenario be launched without --base-config.
#                           Path must stay inside PROJECT_DIR.
#   perception.path_a.diag.trace_voxels   Opt-in voxel trace (Issue #612).
#                           The runner rewrites trace_path to
#                           <scenario_log_dir>/path_a_voxel_trace.jsonl so every
#                           run is self-contained.
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
# Default base config: picked at scenario-load time.  Order of precedence:
#   1. --base-config CLI flag (explicit override)
#   2. scenario.base_config field inside the scenario JSON (self-describing)
#   3. config/cosys_airsim_dev.json  (dev profile — preferred for local runs)
#   4. config/cosys_airsim.json      (cloud profile — fallback)
DEFAULT_BASE_CONFIG=""   # populated after SCENARIO_FILE is resolved
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
BASE_CONFIG=""   # empty = auto-resolve after scenario file is known
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

confine_to_project() {
    # Return 0 iff $1 resolves to a path under $PROJECT_DIR.  Rejects symlink
    # escapes and ../-escapes even if the intermediate path doesn't exist yet.
    # Portable realpath(1) may not support --relative-base; use Python instead.
    local candidate="$1"
    python3 - "$candidate" "$PROJECT_DIR" <<'PYEOF' >/dev/null 2>&1
import os, sys
candidate, root = sys.argv[1], sys.argv[2]
root_real = os.path.realpath(root)
# Use realpath so symlink targets are evaluated; fall back to normpath if the
# candidate does not exist yet (realpath still resolves parents).
cand_real = os.path.realpath(candidate)
if not (cand_real == root_real or cand_real.startswith(root_real + os.sep)):
    sys.exit(1)
PYEOF
}

resolve_base_config() {
    # Resolve the base config path against a scenario file.
    # Precedence: caller-provided override → scenario.base_config → dev → cloud.
    #
    # Security: the `scenario.base_config` hint is read from a JSON file we do
    # NOT control (users can ship arbitrary scenarios).  A malicious hint like
    # `"base_config": "../../etc/passwd"` would otherwise be parsed by
    # merge_configs() and passed into every companion process.  Confine all
    # resolved paths to $PROJECT_DIR; reject anything that escapes.
    local scenario_file="$1"
    local override="$2"
    if [[ -n "$override" ]]; then
        if ! confine_to_project "$override"; then
            echo -e "${RED}ERROR: --base-config '${override}' escapes project root${NC}" >&2
            return 2
        fi
        echo "$override"
        return 0
    fi
    local hint=""
    if [[ -f "$scenario_file" ]]; then
        hint=$(json_get "$scenario_file" "scenario.base_config" 2>/dev/null || echo "")
    fi
    if [[ -n "$hint" && "$hint" != "None" ]]; then
        if [[ "$hint" != /* ]]; then
            hint="${PROJECT_DIR}/${hint}"
        fi
        if ! confine_to_project "$hint"; then
            echo -e "${RED}ERROR: scenario.base_config '${hint}' escapes project root${NC}" >&2
            return 2
        fi
        echo "$hint"
        return 0
    fi
    if [[ -f "${PROJECT_DIR}/config/cosys_airsim_dev.json" ]]; then
        echo "${PROJECT_DIR}/config/cosys_airsim_dev.json"
        return 0
    fi
    echo "${PROJECT_DIR}/config/cosys_airsim.json"
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
    if [[ -n "$BASE_CONFIG" && ! -f "$BASE_CONFIG" ]]; then
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

        args=("$f" --log-dir "$LOG_DIR")
        # Only forward --base-config if user explicitly set one; otherwise let
        # the recursive call resolve per-scenario (scenario.base_config → dev → cloud).
        [[ -n "$BASE_CONFIG" ]] && args+=(--base-config "$BASE_CONFIG")
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

# Resolve base config now that we know which scenario is running.
# resolve_base_config returns exit code 2 on path-confinement failure; the
# error message has already gone to stderr.
if ! BASE_CONFIG=$(resolve_base_config "$SCENARIO_FILE" "$BASE_CONFIG"); then
    exit 2
fi
if [[ ! -f "$BASE_CONFIG" ]]; then
    echo -e "${RED}ERROR: Base config not found: ${BASE_CONFIG}${NC}"
    exit 2
fi
echo -e "${CYAN}  Base config: ${BASE_CONFIG}${NC}"

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

# Prefix the log-dir name with the scenario number pulled from the filename
# (e.g. "33_non_coco_obstacles.json" → "33_non_coco_obstacles").  Makes run
# directories immediately grep-able / sortable by scenario number.
SCENARIO_FILE_BASE=$(basename "$SCENARIO_FILE" .json)
SCENARIO_NUM_PREFIX=$(echo "$SCENARIO_FILE_BASE" | grep -oE '^[0-9]+' || true)
if [[ -n "$SCENARIO_NUM_PREFIX" && "$SCENARIO_NAME" != "${SCENARIO_NUM_PREFIX}_"* ]]; then
    SCENARIO_DIR_NAME="${SCENARIO_NUM_PREFIX}_${SCENARIO_NAME}"
else
    SCENARIO_DIR_NAME="$SCENARIO_NAME"
fi

RUN_START_EPOCH=$(date +%s)
SCENARIO_LOG_DIR=$(create_run_dir "$LOG_DIR" "$SCENARIO_DIR_NAME")

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

# Route per-run diagnostic artifacts into the scenario log directory so every
# run stays self-contained (#612). Only rewrites fields the scenario opted in.
python3 - "$MERGED_CONFIG" "$SCENARIO_LOG_DIR" <<'PYEOF'
import json, os, sys
cfg_path, log_dir = sys.argv[1], sys.argv[2]
with open(cfg_path) as f:
    cfg = json.load(f)
diag = cfg.get("perception", {}).get("path_a", {}).get("diag")
if isinstance(diag, dict) and diag.get("trace_voxels"):
    diag["trace_path"] = os.path.join(log_dir, "path_a_voxel_trace.jsonl")
with open(cfg_path, "w") as f:
    json.dump(cfg, f, indent=4)
PYEOF

EFFECTIVE_IPC="${IPC_BACKEND}"
if [[ -z "$EFFECTIVE_IPC" ]]; then
    EFFECTIVE_IPC=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('ipc_backend','zenoh'))" "$MERGED_CONFIG" 2>/dev/null || echo "zenoh")
fi

# ── Preflight: every model_path in the merged config must exist (Issue #625) ──
# Without this check, a missing ONNX file produces a silent in-process error
# (e.g. `[OpenCvYoloDetector] Failed to load model`) and the scenario runs to
# completion with the affected stage degraded.  The pass_criteria failures look
# identical to genuine logic bugs, so debugging takes 30+ minutes per missing
# file.  Fail fast here instead, with a pointer to the right download script.
# Implementation lives in lib_scenario_logging.sh so cosys + gazebo runners
# share the same hardened check (PR #628 review-code-quality P2: deduplication;
# review-security P3: path-traversal + json-parse hardening).
MISSING_MODELS=$(preflight_model_paths "$MERGED_CONFIG" "$PROJECT_DIR")
if [[ -n "$MISSING_MODELS" ]]; then
    echo -e "  ${RED}✗ Preflight failed: missing model file(s) referenced by scenario config${NC}" >&2
    echo "" >&2
    echo "$MISSING_MODELS" | while IFS=$'\t' read -r key path; do
        echo "    config key:  $key" >&2
        echo "    expected at: $path" >&2
        case "$path" in
            *yolov8n*)             echo "    → run: bash models/download_yolov8n.sh"          >&2 ;;
            *yolov8s*)             echo "    → run: bash models/download_yolov8n.sh # use 'n' variant" >&2 ;;
            *yolov8*visdrone*)     echo "    → run: bash models/download_yolov8n_visdrone.sh"  >&2 ;;
            *fastsam*)             echo "    → run: bash models/download_fastsam.sh"           >&2 ;;
            *depth_anything_v2*)   echo "    → run: bash models/download_depth_anything_v2.sh" >&2 ;;
            *)                     echo "    → no known download script; check models/ for the matching .sh" >&2 ;;
        esac
        echo "" >&2
    done
    echo -e "  ${YELLOW}Tip:${NC} models/ is gitignored — every fresh checkout needs the download scripts run." >&2
    echo "" >&2
    SCENARIO_LOG_DIR=$(abort_run_dir "$SCENARIO_LOG_DIR")
    exit 1
fi
echo -e "  ${GREEN}✓${NC} Preflight: all referenced model files present"

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
    # Stop telemetry poller first so its JSONL flushes before UE5 goes away.
    if [[ -n "${TELEMETRY_PID:-}" ]] && kill -0 "$TELEMETRY_PID" 2>/dev/null; then
        kill -SIGTERM "$TELEMETRY_PID" 2>/dev/null || true
        sleep 1
        kill -SIGKILL "$TELEMETRY_PID" 2>/dev/null || true
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
    VIEW_MODE=$(python3 -c "import json; print(json.load(open('${AIRSIM_SETTINGS_DIR}/settings.json')).get('ViewMode', 'default'))" 2>/dev/null || echo "default")
    echo -e "  ${GREEN}✓${NC} AirSim settings deployed to ${AIRSIM_SETTINGS_DIR}/settings.json (ViewMode=${VIEW_MODE})"
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

# NOTE: Tier 3 uses AirSim's built-in SimpleFlight controller (not PX4 HIL).
# SimpleFlight is driven via AirSim RPC (port 41451) — no PX4, no MAVLink, no
# HIL TCP port 4560. Companion stack uses CosysFCLink via MultirotorRpcLibClient.
# PX4+Cosys HIL integration is broken upstream (PX4 #24033, AirSim #5018).
# See ADR-011 amendment for #490, config/cosys_settings.json comment.
echo ""
echo "Phase 2b: SimpleFlight controller active (no PX4 needed for Tier 3)"
echo -e "  ${CYAN}Settling (5s) for SimpleFlight to stabilize...${NC}"
sleep 5

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

# ── Sim-side telemetry poller (Issue #607 / #612) ────────────
# Polls simGetCollisionInfo + simGetGroundTruthKinematics at 10 Hz and
# writes JSONL alongside the other run logs.  Complements the perception-
# side voxel trace: `cosys_telemetry.jsonl` carries ground-truth drone pose
# (vs SLAM estimate) and UE5 physics collision events (vs no-collision-log-
# at-all today).
TELEMETRY_POLLER="${BIN_DIR}/cosys_telemetry_poller"
TELEMETRY_PID=""
if [[ -x "$TELEMETRY_POLLER" ]]; then
    "$TELEMETRY_POLLER" \
        --out "${SCENARIO_LOG_DIR}/cosys_telemetry.jsonl" \
        --collisions "${SCENARIO_LOG_DIR}/collisions.log" \
        --host "127.0.0.1" --port "${COSYS_RPC_PORT}" --rate_hz 10 \
        > "${SCENARIO_LOG_DIR}/cosys_telemetry_poller.log" 2>&1 &
    TELEMETRY_PID=$!
    echo -e "  ${CYAN}Telemetry poller launched (pid=${TELEMETRY_PID})${NC}"
else
    echo -e "  ${YELLOW}WARNING: ${TELEMETRY_POLLER} not built — no collision / GT-pose logs${NC}"
fi

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

# Issue #705 follow-up — gracefully stop companion processes BEFORE building
# combined.log so the cat captures the final Mission-complete / RTL → LAND
# log lines.  Run 2026-05-05_153302 had the cat fire ~2 s before
# mission_planner finished writing "Mission complete — RTL", making the
# runner spuriously report `_FAIL` despite zero collisions and full
# waypoint progression.
echo "  Stopping companion processes (SIGINT, then SIGKILL after 2 s)..."
for pid in "${COMPANION_PIDS[@]}"; do
    kill -SIGINT "$pid" 2>/dev/null || true
done
sleep 2
for pid in "${COMPANION_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
        kill -SIGKILL "$pid" 2>/dev/null || true
    fi
done
# Brief flush window for any in-flight log writes (spdlog flushes on signal,
# but the file system may not have synced yet).
sleep 1

COMBINED_LOG="${SCENARIO_LOG_DIR}/combined.log"
cat "${SCENARIO_LOG_DIR}"/*.log > "$COMBINED_LOG" 2>/dev/null || true

# Also pull UE5 console log for getCamera error messages
UE5_LOG_DIR="${HOME}/.config/Epic/UnrealEngine/5.4/Saved/Logs"
if [[ -d "$UE5_LOG_DIR" ]]; then
    ls -t "${UE5_LOG_DIR}"/*.log 2>/dev/null | head -1 | xargs -I{} cat {} >> "$COMBINED_LOG" 2>/dev/null || true
fi

echo ""
echo "Log-contains checks:"
# Issue #698 — use -F (fixed-string) so that bracketed log prefixes like
# [FSM], [Planner], [CosysGroundTruthRadar] match LITERAL text rather than
# being interpreted as regex character classes.  Previously e.g. "[FSM]
# Advanced to waypoint 6/6" silently matched only when one of {F,S,M}
# happened to appear before "] Advanced..." in the line — fragile and
# scenario-specific.  pass_criteria.log_contains is conceptually a
# literal-text contract, not a regex one (the explicit-regex use case is
# served by validation.checks[].pattern).
while read -r pattern; do
    [[ -z "$pattern" ]] && continue
    if grep -qaiF "$pattern" "$COMBINED_LOG" 2>/dev/null; then
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
    if grep -qaiF "$pattern" "$COMBINED_LOG" 2>/dev/null; then
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

# ── PATH A voxel-on-target regression check (PR #614 review) ──
# Addresses the review-test-quality P1: without this check, a Fix #55
# regression (wrong camera extrinsic → voxels 8–12 m off) would not fail
# the scenario — every `pass_criteria.log_contains` marker fires at init,
# so a broken pipeline still "passes" until the LANDED gate.
VOXEL_TRACE="${SCENARIO_LOG_DIR}/path_a_voxel_trace.jsonl"
VOXEL_CHECK="${PROJECT_DIR}/tools/check_voxel_on_target.py"
if [[ -f "$VOXEL_TRACE" && -x "$VOXEL_CHECK" && -n "$SCENE_FILE" && -f "$SCENE_FILE" ]]; then
    echo ""
    echo "PATH A voxel-on-target check:"
    # Pull threshold from scenario JSON if present; otherwise use the default.
    VOX_MIN_RATIO=$(json_get "$SCENARIO_FILE" "pass_criteria.voxel_on_target_ratio_min")
    VOX_RADIUS=$(json_get "$SCENARIO_FILE" "pass_criteria.voxel_on_target_radius_m")
    VOX_ARGS=(--trace "$VOXEL_TRACE" --scene "$SCENE_FILE")
    [[ -n "$VOX_MIN_RATIO" && "$VOX_MIN_RATIO" != "None" ]] && VOX_ARGS+=(--min-ratio "$VOX_MIN_RATIO")
    [[ -n "$VOX_RADIUS"    && "$VOX_RADIUS"    != "None" ]] && VOX_ARGS+=(--radius-m "$VOX_RADIUS")
    if python3 "$VOXEL_CHECK" "${VOX_ARGS[@]}" 2>&1 | tee "${SCENARIO_LOG_DIR}/voxel_on_target.log"; then
        check "Voxel-on-target ratio within threshold" 0
    else
        check "Voxel-on-target ratio within threshold" 1
    fi
fi
# ── Diagnostic overlays (best-effort — never fail the run) ────
# Both overlays read run-local logs and write PNGs into the run dir so
# they get captured by the report and the post-run finalize.
SCENE_OVERLAY="${PROJECT_DIR}/tools/diag/scene_overlay.py"
GRID_OVERLAY="${PROJECT_DIR}/tools/diag/planner_grid_overlay.py"
if [[ -x "$SCENE_OVERLAY" ]]; then
    python3 "$SCENE_OVERLAY" "$SCENARIO_LOG_DIR" "$SCENARIO_FILE" \
        > "${SCENARIO_LOG_DIR}/scene_overlay.log" 2>&1 \
        && echo "  Generated: scene_overlay.png" \
        || echo "  WARN: scene_overlay.py failed (see scene_overlay.log)"
fi
if [[ -x "$GRID_OVERLAY" ]]; then
    python3 "$GRID_OVERLAY" "$SCENARIO_LOG_DIR" "$SCENARIO_FILE" \
        > "${SCENARIO_LOG_DIR}/planner_grid_overlay.log" 2>&1 \
        && echo "  Generated: planner_grid_overlay.png" \
        || echo "  WARN: planner_grid_overlay.py failed (see planner_grid_overlay.log)"
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

SCENARIO_BASE_DIR="${LOG_DIR}/${SCENARIO_DIR_NAME}"
update_latest_symlink "$SCENARIO_BASE_DIR" "$(basename "$SCENARIO_LOG_DIR")"

write_run_metadata "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "$SCENARIO_NAME" "cosys_airsim" "$SCENARIO_FILE" \
    "$RUN_START_EPOCH" "$RUN_END_EPOCH" \
    "$PASS" "$FAIL" "$TOTAL" "$PROJECT_DIR"

REPORT_FILE=$(generate_run_report "$SCENARIO_LOG_DIR" "$SCENARIO_NAME" "cosys_airsim" \
    "$PASS" "$FAIL" "$TOTAL" "$SCENARIO_FILE" "$PROJECT_DIR")

append_to_index "${LOG_DIR}/runs.jsonl" \
    "${SCENARIO_LOG_DIR}/run_metadata.json" \
    "${SCENARIO_DIR_NAME}/$(basename "$SCENARIO_LOG_DIR")"

echo ""
echo -e "  ${CYAN}Report : ${REPORT_FILE}${NC}"
echo -e "  ${CYAN}Logs   : ${SCENARIO_LOG_DIR}${NC}"

if [[ $FAIL -gt 0 ]]; then
    exit 1
else
    exit 0
fi
