#!/usr/bin/env bash
# tests/test_zenoh_e2e.sh
# ════════════════════════════════════════════════════════════════════
# End-to-End Zenoh IPC Smoke Test
#
# Launches all 7 processes with ipc_backend=zenoh (simulated hardware
# backends), verifies:
#
#   1. All processes start and stay alive
#   2. Zenoh session initialises (log markers)
#   3. Data flows through pub/sub channels (log markers)
#   4. Liveliness tokens are declared (log markers)
#   5. Process-death detection works (kill one → P7 detects it)
#   6. Graceful shutdown on SIGINT
#
# Usage:
#   ./tests/test_zenoh_e2e.sh                    # default 15s verify
#   VERIFY_TIME=30 ./tests/test_zenoh_e2e.sh     # longer window
#
# Exit codes:
#   0  All checks passed
#   1  One or more checks failed
#   2  Setup / build not ready
#
# Prerequisites:
#   - Project built with -DENABLE_ZENOH=ON
#   - No PX4 / Gazebo required (all backends simulated)
# ════════════════════════════════════════════════════════════════════
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="${PROJECT_DIR}/build/bin"
CONFIG_FILE="${PROJECT_DIR}/config/zenoh_e2e.json"
LOG_DIR="/tmp/zenoh_e2e_test_logs"
VERIFY_TIME="${VERIFY_TIME:-15}"

# ── Colours ──────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
FAIL=0
TOTAL=0

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

# ── Pre-flight checks ───────────────────────────────────────
echo "════════════════════════════════════════════════════════"
echo "  Zenoh E2E Smoke Test"
echo "  Verify   : ${VERIFY_TIME}s"
echo "  Config   : ${CONFIG_FILE}"
echo "  Log dir  : ${LOG_DIR}"
echo "════════════════════════════════════════════════════════"

if [[ ! -d "$BIN_DIR" ]]; then
    echo -e "${RED}ERROR: Build directory '${BIN_DIR}' not found.${NC}"
    echo "  Run: cd build && cmake .. -DENABLE_ZENOH=ON && make -j\$(nproc)"
    exit 2
fi

PROCESSES=("system_monitor" "video_capture" "comms" "perception"
           "slam_vio_nav" "mission_planner" "payload_manager")

for proc in "${PROCESSES[@]}"; do
    if [[ ! -x "${BIN_DIR}/${proc}" ]]; then
        echo -e "${RED}ERROR: Binary '${proc}' not found in ${BIN_DIR}${NC}"
        exit 2
    fi
done

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo -e "${RED}ERROR: Config file '${CONFIG_FILE}' not found.${NC}"
    exit 2
fi

# Verify config uses zenoh backend
if ! grep -q '"ipc_backend".*:.*"zenoh"' "$CONFIG_FILE"; then
    echo -e "${RED}ERROR: Config does not use ipc_backend=zenoh${NC}"
    exit 2
fi

# ── Clean up ─────────────────────────────────────────────────
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

# Ensure system libstdc++ is preferred
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ── Process management ───────────────────────────────────────
declare -A PIDS           # process name → PID
declare -A LAUNCH_ORDER   # process name → launch index

cleanup() {
    echo ""
    echo -e "${CYAN}Shutting down all processes...${NC}"
    for proc in "${PROCESSES[@]}"; do
        local pid="${PIDS[$proc]:-}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    for proc in "${PROCESSES[@]}"; do
        local pid="${PIDS[$proc]:-}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -SIGKILL "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

launch() {
    local proc="$1"
    local idx="$2"
    echo -e "  [${idx}/7] Starting ${CYAN}${proc}${NC}..."
    "${BIN_DIR}/${proc}" --config "$CONFIG_FILE" --log-level debug \
        > "${LOG_DIR}/${proc}.log" 2>&1 &
    PIDS[$proc]=$!
    LAUNCH_ORDER[$proc]=$idx
}

# ═════════════════════════════════════════════════════════════
# Phase 1: Launch all 7 processes
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 1: Launching all 7 processes with Zenoh backend...${NC}"

launch "system_monitor"   1
sleep 0.5
launch "video_capture"    2
sleep 0.5
launch "comms"            3
sleep 0.5
launch "perception"       4
sleep 0.5
launch "slam_vio_nav"     5
sleep 0.5
launch "mission_planner"  6
sleep 0.5
launch "payload_manager"  7

echo "  All 7 processes launched."
echo "  PIDs: $(for p in "${PROCESSES[@]}"; do echo -n "$p=${PIDS[$p]} "; done)"

# ═════════════════════════════════════════════════════════════
# Phase 2: Verification window — let processes run
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 2: Running for ${VERIFY_TIME}s verification window...${NC}"
sleep "$VERIFY_TIME"

# ═════════════════════════════════════════════════════════════
# Phase 3: Check all processes still alive
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 3: Process liveness checks...${NC}"

for proc in "${PROCESSES[@]}"; do
    if kill -0 "${PIDS[$proc]}" 2>/dev/null; then
        check "Process alive: ${proc} (PID ${PIDS[$proc]})" 0
    else
        wait "${PIDS[$proc]}" 2>/dev/null
        exit_code=$?
        check "Process alive: ${proc} (exited with code ${exit_code})" 1
    fi
done

# ═════════════════════════════════════════════════════════════
# Phase 4: Zenoh backend selection
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 4: Zenoh backend initialisation...${NC}"

for proc in "${PROCESSES[@]}"; do
    logfile="${LOG_DIR}/${proc}.log"
    if grep -qi "Selected backend: Zenoh\|MessageBusFactory.*Zenoh" "$logfile" 2>/dev/null; then
        check "Zenoh backend: ${proc}" 0
    else
        check "Zenoh backend: ${proc}" 1
    fi
done

# ═════════════════════════════════════════════════════════════
# Phase 5: Liveliness token declaration
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 5: Liveliness token checks...${NC}"

for proc in "${PROCESSES[@]}"; do
    logfile="${LOG_DIR}/${proc}.log"
    if grep -qi "Token declared.*drone/alive/" "$logfile" 2>/dev/null; then
        check "Liveliness token: ${proc}" 0
    else
        check "Liveliness token: ${proc}" 1
    fi
done

# P7 should be monitoring liveliness
if grep -qi "Monitor started\|Liveliness.*Monitor" "${LOG_DIR}/system_monitor.log" 2>/dev/null; then
    check "Liveliness monitor active in system_monitor" 0
else
    check "Liveliness monitor active in system_monitor" 1
fi

# ═════════════════════════════════════════════════════════════
# Phase 6: Data flow verification (log markers)
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 6: Data flow verification...${NC}"

# P1 → publishing video frames
if grep -qi "MissionCam\|publish\|frame\|capture" "${LOG_DIR}/video_capture.log" 2>/dev/null; then
    check "P1: video_capture producing frames" 0
else
    check "P1: video_capture producing frames" 1
fi

# P2 → perception processing
if grep -qi "detect\|track\|fusion\|Perception" "${LOG_DIR}/perception.log" 2>/dev/null; then
    check "P2: perception processing" 0
else
    check "P2: perception processing" 1
fi

# P3 → SLAM producing poses
if grep -qi "pose\|VIO\|SLAM\|IMU\|keyframe" "${LOG_DIR}/slam_vio_nav.log" 2>/dev/null; then
    check "P3: slam_vio_nav producing poses" 0
else
    check "P3: slam_vio_nav producing poses" 1
fi

# P4 → mission planner running FSM
if grep -qi "FSM\|mission\|waypoint\|state\|IDLE\|PREFLIGHT" "${LOG_DIR}/mission_planner.log" 2>/dev/null; then
    check "P4: mission_planner FSM active" 0
else
    check "P4: mission_planner FSM active" 1
fi

# P5 → comms connected (simulated)
if grep -qi "mavlink\|heartbeat\|comms\|connected\|FC\|GCS" "${LOG_DIR}/comms.log" 2>/dev/null; then
    check "P5: comms active" 0
else
    check "P5: comms active" 1
fi

# P6 → payload manager running
if grep -qi "gimbal\|payload\|plugin\|Payload" "${LOG_DIR}/payload_manager.log" 2>/dev/null; then
    check "P6: payload_manager active" 0
else
    check "P6: payload_manager active" 1
fi

# P7 → system monitor reporting health
if grep -qi "CPU=\|SysMon\|health\|monitor" "${LOG_DIR}/system_monitor.log" 2>/dev/null; then
    check "P7: system_monitor reporting health" 0
else
    check "P7: system_monitor reporting health" 1
fi

# ═════════════════════════════════════════════════════════════
# Phase 7: Process-death detection (kill one → P7 detects it)
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 7: Process-death detection test...${NC}"

# Pick a non-critical process to kill: payload_manager
KILL_PROC="payload_manager"
KILL_PID="${PIDS[$KILL_PROC]}"

if kill -0 "$KILL_PID" 2>/dev/null; then
    echo "  Killing ${KILL_PROC} (PID ${KILL_PID})..."
    kill -SIGKILL "$KILL_PID" 2>/dev/null || true
    wait "$KILL_PID" 2>/dev/null || true

    # Give system_monitor time to detect the death via liveliness
    echo "  Waiting 5s for death detection..."
    sleep 5

    # Check P7 logs for death detection
    if grep -qi "DIED.*${KILL_PROC}\|Process DIED.*payload" "${LOG_DIR}/system_monitor.log" 2>/dev/null; then
        check "Death detected: ${KILL_PROC}" 0
    else
        # Also check for the liveliness DELETE event
        if grep -qi "DELETE.*payload\|death.*payload" "${LOG_DIR}/system_monitor.log" 2>/dev/null; then
            check "Death detected: ${KILL_PROC}" 0
        else
            check "Death detected: ${KILL_PROC}" 1
        fi
    fi
else
    check "Death detection: ${KILL_PROC} already dead before test" 1
fi

# Verify the other 6 processes survived the kill
echo ""
echo -e "${CYAN}Phase 7b: Remaining processes still alive after kill...${NC}"

for proc in "${PROCESSES[@]}"; do
    if [[ "$proc" == "$KILL_PROC" ]]; then
        continue  # skip the one we killed
    fi
    if kill -0 "${PIDS[$proc]}" 2>/dev/null; then
        check "Still alive after kill: ${proc}" 0
    else
        check "Still alive after kill: ${proc}" 1
    fi
done

# ═════════════════════════════════════════════════════════════
# Phase 8: Graceful shutdown
# ═════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}Phase 8: Graceful shutdown test...${NC}"

# Send SIGINT to all remaining processes
for proc in "${PROCESSES[@]}"; do
    if [[ "$proc" == "$KILL_PROC" ]]; then
        continue
    fi
    local_pid="${PIDS[$proc]}"
    if kill -0 "$local_pid" 2>/dev/null; then
        kill -SIGINT "$local_pid" 2>/dev/null || true
    fi
done

# Wait up to 5 seconds for graceful exit
SHUTDOWN_OK=true
for i in $(seq 1 50); do
    ALL_DEAD=true
    for proc in "${PROCESSES[@]}"; do
        if [[ "$proc" == "$KILL_PROC" ]]; then
            continue
        fi
        if kill -0 "${PIDS[$proc]}" 2>/dev/null; then
            ALL_DEAD=false
        fi
    done
    if $ALL_DEAD; then
        break
    fi
    sleep 0.1
done

for proc in "${PROCESSES[@]}"; do
    if [[ "$proc" == "$KILL_PROC" ]]; then
        continue
    fi
    if kill -0 "${PIDS[$proc]}" 2>/dev/null; then
        check "Graceful shutdown: ${proc}" 1
        SHUTDOWN_OK=false
        kill -SIGKILL "${PIDS[$proc]}" 2>/dev/null || true
    else
        wait "${PIDS[$proc]}" 2>/dev/null
        exit_code=$?
        if [[ "$exit_code" -eq 0 ]] || [[ "$exit_code" -eq 130 ]]; then
            check "Graceful shutdown: ${proc} (exit ${exit_code})" 0
        else
            check "Graceful shutdown: ${proc} (exit ${exit_code})" 1
        fi
    fi
done

# Clear the trap — we already cleaned up
trap - EXIT INT TERM

# ═════════════════════════════════════════════════════════════
# Results
# ═════════════════════════════════════════════════════════════
echo ""
echo "════════════════════════════════════════════════════════"
if [[ $FAIL -eq 0 ]]; then
    echo -e "  ${GREEN}ALL CHECKS PASSED${NC}: ${PASS}/${TOTAL}"
else
    echo -e "  ${RED}FAILURES${NC}: ${FAIL}/${TOTAL} checks failed"
    echo "  Logs available at: ${LOG_DIR}/"
    echo "  Tail a failing process: tail -50 ${LOG_DIR}/<process>.log"
fi
echo "════════════════════════════════════════════════════════"

[[ $FAIL -eq 0 ]] && exit 0 || exit 1
