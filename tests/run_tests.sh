#!/usr/bin/env bash
# tests/run_tests.sh — Modular test runner for the Companion Software Stack.
#
# Run all tests or filter by module:
#   ./tests/run_tests.sh              # all tests
#   ./tests/run_tests.sh ipc          # SHM + Zenoh IPC tests
#   ./tests/run_tests.sh watchdog     # thread heartbeat, health, policy, graph, process manager
#   ./tests/run_tests.sh perception   # P2 perception (tracker, fusion, detectors)
#   ./tests/run_tests.sh mission      # P4 mission FSM + fault manager
#   ./tests/run_tests.sh comms        # P5 comms (MavlinkSim, GCSLink)
#   ./tests/run_tests.sh hal          # HAL backends (simulated, Gazebo, MAVLink)
#   ./tests/run_tests.sh payload      # P6 payload manager (GimbalController)
#   ./tests/run_tests.sh monitor      # P7 system monitor (SysInfo, process health)
#   ./tests/run_tests.sh util         # config, result, latency, json log, correlation
#   ./tests/run_tests.sh quick        # fast unit tests (skip fork/exec, Zenoh, E2E)
#   ./tests/run_tests.sh zenoh-e2e     # Zenoh end-to-end integration (shell script)
#   ./tests/run_tests.sh gazebo-e2e    # Gazebo SITL integration (shell script)
#   ./tests/run_tests.sh list         # list available modules
#
# Options:
#   --verbose       Show individual test output
#   --parallel N    Run with N parallel jobs (default: nproc)
#   --repeat N      Repeat tests N times (stress testing)
#   --build         Rebuild before running tests
#   --zenoh         Build with Zenoh backend (combines with --build)
#   --asan          Rebuild with AddressSanitizer, then test
#   --tsan          Rebuild with ThreadSanitizer, then test
#   --ubsan         Rebuild with UBSan, then test
#   --coverage      Rebuild with coverage, test, generate report
#
# Examples:
#   ./tests/run_tests.sh watchdog --verbose
#   ./tests/run_tests.sh quick --parallel 4
#   ./tests/run_tests.sh --asan
#   ./tests/run_tests.sh ipc --repeat 5
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_DIR}/build"
DEPLOY_DIR="${PROJECT_DIR}/deploy"

# ── Colors ───────────────────────────────────────────────────
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    BLUE='\033[0;34m'
    BOLD='\033[1m'
    RESET='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' BOLD='' RESET=''
fi

# ── Module → ctest filter mapping ────────────────────────────
# Each module maps to a regex matching GTest suite names.
declare -A MODULE_FILTERS=(
    [ipc]="ShmIPCTest|SPSCRingTest|ShmMessageBusTest|ShmPublisherTest|ShmSubscriberTest|ShmSystemHealth|MessageBusFactory|ZenohMessageBus|ZenohPublisher|ZenohSubscriber|ZenohPubSub|ZenohSession|ZenohShmProvider|ZenohShmPublish|ZenohServiceChannel|ZenohMigration|ZenohTopicMapping|ServiceChannelInterface|ConfigAwareFactory|WireFormat|WireHeaderBackcompat|WireHeaderV2"
    [watchdog]="ThreadHeartbeatTest|ThreadHealthPublisherTest|ShmThreadHealthStruct|RestartPolicy|StackStatus|ProcessConfig|ProcessGraph|ProcessManagerTest|ProcessHealthEntry"
    [perception]="KalmanBoxTrackerTest|HungarianSolverTest|MultiObjectTrackerTest|FusionEngineTest|ColorContourDetectorTest|OpenCvYoloDetectorTest|DetectorFactoryTest|YoloFactoryTest|YoloModelTest|ComponentBBoxTest|CocoMappingTest|HsvRangeTest|RgbToHsvTest|UnionFindTest"
    [mission]="MissionFSMTest|FaultManagerTest"
    [comms]="MavlinkSim|GCSLink"
    [hal]="HALFactoryTest|SimulatedCameraTest|SimulatedIMUTest|SimulatedFCLinkTest|SimulatedGCSLinkTest|SimulatedGimbalTest|GazeboCameraTest|GazeboIMUTest|MavlinkFCLinkTest"
    [payload]="GimbalController"
    [monitor]="SysInfo|ProcessHealthEntry"
    [util]="ConfigTest|ConfigValidatorTest|ArgParserJsonTest|ResultTest|VoidResultTest|ErrorTest|ErrorCodeTest|LatencyTrackerTest|JsonSinkTest|JsonSinkMtTest|JsonEscapeTest|LevelToStrTest|FormatTimestampTest|LogConfigJsonTest|CorrelationContext|ScopedCorrelation|JsonCorrelation|ShmCorrelation"
    [interfaces]="VisualFrontendTest|PathPlannerTest|ObstacleAvoiderTest|ProcessMonitorTest"
    [zenoh]="ZenohMessageBus|ZenohPublisher|ZenohSubscriber|ZenohPubSub|ZenohSession|ZenohShmProvider|ZenohShmPublish|ZenohServiceChannel|ZenohMigration|ZenohTopicMapping|ZenohNetworkConfig|LivelinessConstants|LivelinessExtract|LivelinessMonitor|LivelinessToken"
    [network]="ZenohNetworkConfig|LivelinessConstants|LivelinessExtract|LivelinessMonitor|LivelinessToken|WireFormat|WireHeaderBackcompat|WireHeaderV2"
)

# Quick mode: exclude slow tests (fork/exec, Zenoh sessions, E2E)
QUICK_EXCLUDE="ProcessManagerTest|ZenohMessageBus|ZenohPublisher|ZenohSubscriber|ZenohPubSub|ZenohSession|ZenohShmProvider|ZenohShmPublish|ZenohServiceChannel|LivelinessConstants|LivelinessExtract|LivelinessMonitor|LivelinessToken|ZenohNetworkConfig|ConfigAwareFactory|ConfigValidatorTest"

# ── Parse arguments ──────────────────────────────────────────
MODULE=""
VERBOSE=0
PARALLEL="$(nproc)"
REPEAT=1
DO_BUILD=0
SANITIZER=""
COVERAGE=0
ENABLE_ZENOH=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --verbose|-v)     VERBOSE=1 ;;
        --parallel|-j)    PARALLEL="$2"; shift ;;
        --repeat|-r)      REPEAT="$2"; shift ;;
        --build|-b)       DO_BUILD=1 ;;
        --zenoh)          ENABLE_ZENOH=1; DO_BUILD=1 ;;
        --asan)           SANITIZER="asan"; DO_BUILD=1 ;;
        --tsan)           SANITIZER="tsan"; DO_BUILD=1 ;;
        --ubsan)          SANITIZER="ubsan"; DO_BUILD=1 ;;
        --coverage)       COVERAGE=1; DO_BUILD=1 ;;
        --help|-h)
            head -35 "$0" | tail -33
            exit 0
            ;;
        list)
            echo -e "${BOLD}Available test modules:${RESET}"
            echo ""
            printf "  %-14s %s\n" "MODULE" "DESCRIPTION"
            printf "  %-14s %s\n" "──────" "───────────"
            printf "  %-14s %s\n" "ipc" "SHM + Zenoh IPC primitives, message bus, wire format"
            printf "  %-14s %s\n" "watchdog" "Thread heartbeat, health publisher, restart policy, process graph, supervisor"
            printf "  %-14s %s\n" "perception" "Kalman tracker, fusion engine, color contour, YOLOv8 detector"
            printf "  %-14s %s\n" "mission" "Mission FSM state machine, FaultManager degradation"
            printf "  %-14s %s\n" "comms" "MavlinkSim and GCSLink"
            printf "  %-14s %s\n" "hal" "Simulated, Gazebo, and MAVLink HAL backends"
            printf "  %-14s %s\n" "payload" "GimbalController servo simulation"
            printf "  %-14s %s\n" "monitor" "P7 system monitor (CPU/memory/thermal, process health)"
            printf "  %-14s %s\n" "util" "Config, Result<T,E>, latency tracker, JSON log sink, correlation"
            printf "  %-14s %s\n" "interfaces" "IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor"
            printf "  %-14s %s\n" "zenoh" "All Zenoh-specific tests (pub/sub, SHM, services, liveliness)"
            printf "  %-14s %s\n" "network" "Network transport, wire format, liveliness"
            printf "  %-14s %s\n" "quick" "All fast unit tests (excludes fork/exec, Zenoh session, E2E)"
            printf "  %-14s %s\n" "zenoh-e2e" "Zenoh end-to-end integration smoke test (shell script)"
            printf "  %-14s %s\n" "gazebo-e2e" "Gazebo SITL integration smoke test (shell script)"
            echo ""
            exit 0
            ;;
        -*)
            echo -e "${RED}Unknown option: $1${RESET}"
            echo "Run: $0 --help"
            exit 1
            ;;
        *)
            if [[ -n "$MODULE" ]]; then
                echo -e "${RED}Only one module can be specified at a time.${RESET}"
                exit 1
            fi
            MODULE="$1"
            ;;
    esac
    shift
done

# ── Validate module ──────────────────────────────────────────
if [[ -n "$MODULE" && "$MODULE" != "quick" && "$MODULE" != "zenoh-e2e" && "$MODULE" != "gazebo-e2e" ]]; then
    if [[ -z "${MODULE_FILTERS[$MODULE]+x}" ]]; then
        echo -e "${RED}Unknown module: ${MODULE}${RESET}"
        echo "Run: $0 list"
        exit 1
    fi
fi

# ── Build if requested ───────────────────────────────────────
if [[ "$DO_BUILD" -eq 1 ]]; then
    echo -e "${BLUE}${BOLD}═══ Building ═══${RESET}"
    BUILD_ARGS=("Debug")
    if [[ "$ENABLE_ZENOH" -eq 1 ]]; then
        BUILD_ARGS+=("--zenoh")
    fi
    if [[ -n "$SANITIZER" ]]; then
        BUILD_ARGS+=("--${SANITIZER}")
    fi
    if [[ "$COVERAGE" -eq 1 ]]; then
        BUILD_ARGS+=("--coverage")
    fi
    bash "${DEPLOY_DIR}/build.sh" "${BUILD_ARGS[@]}"
    echo ""
fi

# ── Verify build directory exists ────────────────────────────
if [[ ! -d "$BUILD_DIR" ]]; then
    echo -e "${RED}Build directory not found. Run deploy/build.sh first or use --build.${RESET}"
    exit 1
fi

# ── Shell-based E2E test modules ─────────────────────────────
# These bypass ctest and run the integration scripts directly.
if [[ "$MODULE" == "zenoh-e2e" ]]; then
    echo -e "${BOLD}══════════════════════════════════════════${RESET}"
    echo -e "${BOLD}  Zenoh End-to-End Integration Test${RESET}"
    echo -e "${BOLD}══════════════════════════════════════════${RESET}"
    echo ""
    if [[ ! -x "${SCRIPT_DIR}/test_zenoh_e2e.sh" ]]; then
        echo -e "${RED}test_zenoh_e2e.sh not found or not executable.${RESET}"
        exit 1
    fi
    exec bash "${SCRIPT_DIR}/test_zenoh_e2e.sh"
fi

if [[ "$MODULE" == "gazebo-e2e" ]]; then
    echo -e "${BOLD}══════════════════════════════════════════${RESET}"
    echo -e "${BOLD}  Gazebo SITL Integration Test${RESET}"
    echo -e "${BOLD}══════════════════════════════════════════${RESET}"
    echo ""
    if [[ ! -x "${SCRIPT_DIR}/test_gazebo_integration.sh" ]]; then
        echo -e "${RED}test_gazebo_integration.sh not found or not executable.${RESET}"
        exit 1
    fi
    exec bash "${SCRIPT_DIR}/test_gazebo_integration.sh"
fi

# ── Build ctest arguments ────────────────────────────────────
CTEST_ARGS=(
    --test-dir "$BUILD_DIR"
    --output-on-failure
    -j "$PARALLEL"
)

if [[ "$VERBOSE" -eq 1 ]]; then
    CTEST_ARGS+=(--verbose)
fi

if [[ "$REPEAT" -gt 1 ]]; then
    CTEST_ARGS+=(--repeat until-fail:"$REPEAT")
fi

# Apply module filter
if [[ -n "$MODULE" ]]; then
    if [[ "$MODULE" == "quick" ]]; then
        CTEST_ARGS+=(-E "$QUICK_EXCLUDE")
    else
        CTEST_ARGS+=(-R "${MODULE_FILTERS[$MODULE]}")
    fi
fi

# ── Run tests ────────────────────────────────────────────────
echo -e "${BOLD}══════════════════════════════════════════${RESET}"
echo -e "${BOLD}  Companion Stack — Test Runner${RESET}"
if [[ -n "$MODULE" ]]; then
    echo -e "  Module   : ${BLUE}${MODULE}${RESET}"
else
    echo -e "  Module   : ${BLUE}all${RESET}"
fi
echo -e "  Parallel : ${PARALLEL} jobs"
[[ "$REPEAT" -gt 1 ]] && echo -e "  Repeat   : ${REPEAT}x"
[[ -n "$SANITIZER" ]] && echo -e "  Sanitizer: ${YELLOW}${SANITIZER}${RESET}"
[[ "$COVERAGE" -eq 1 ]] && echo -e "  Coverage : ${YELLOW}ON${RESET}"
echo -e "${BOLD}══════════════════════════════════════════${RESET}"
echo ""

# Count matching tests using the same filter args
COUNT_ARGS=(--test-dir "$BUILD_DIR" -N)
if [[ -n "$MODULE" ]]; then
    if [[ "$MODULE" == "quick" ]]; then
        COUNT_ARGS+=(-E "$QUICK_EXCLUDE")
    else
        COUNT_ARGS+=(-R "${MODULE_FILTERS[$MODULE]}")
    fi
fi
TEST_COUNT=$(ctest "${COUNT_ARGS[@]}" 2>&1 | grep -oP 'Total Tests:\s*\K\d+' || echo "?")
echo -e "Running ${BOLD}${TEST_COUNT}${RESET} tests..."
echo ""

START_TIME=$(date +%s)

if ctest "${CTEST_ARGS[@]}"; then
    ELAPSED=$(( $(date +%s) - START_TIME ))
    echo ""
    echo -e "${GREEN}${BOLD}✓ All ${TEST_COUNT} tests passed${RESET} (${ELAPSED}s)"
    EXIT_CODE=0
else
    ELAPSED=$(( $(date +%s) - START_TIME ))
    echo ""
    echo -e "${RED}${BOLD}✗ Some tests failed${RESET} (${ELAPSED}s)"
    EXIT_CODE=1
fi

# ── Coverage report ──────────────────────────────────────────
if [[ "$COVERAGE" -eq 1 && "$EXIT_CODE" -eq 0 ]]; then
    echo ""
    echo -e "${BLUE}${BOLD}═══ Generating coverage report ═══${RESET}"
    cd "$BUILD_DIR"
    lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch 2>/dev/null
    lcov --remove coverage.info '/usr/*' '*/tests/*' '*/build/*' \
         --output-file coverage_filtered.info 2>/dev/null
    if command -v genhtml &>/dev/null; then
        genhtml coverage_filtered.info --output-directory coverage_html 2>/dev/null
        echo -e "${GREEN}Coverage report: ${BUILD_DIR}/coverage_html/index.html${RESET}"
    else
        echo -e "${YELLOW}genhtml not found — raw coverage in ${BUILD_DIR}/coverage_filtered.info${RESET}"
    fi
fi

exit $EXIT_CODE
