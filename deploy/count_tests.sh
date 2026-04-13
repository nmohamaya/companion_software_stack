#!/usr/bin/env bash
# deploy/count_tests.sh — Accurate test count audit
#
# Counts TEST/TEST_F macros in source files and cross-references against
# ctest registered tests. Reports per-file, per-category, per-suite, and
# total counts with compile-guard awareness.
#
# Usage:
#   bash deploy/count_tests.sh              # Full report
#   bash deploy/count_tests.sh --summary    # One-line summary only
#   bash deploy/count_tests.sh --check      # Compare against TESTS.md baseline
#
# This script is the source of truth for test counts. Use it to verify
# tests/TESTS.md is accurate before merge.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TESTS_DIR="$PROJECT_ROOT/tests"
BUILD_DIR="$PROJECT_ROOT/build"

# Colors (if terminal supports it)
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    BOLD='\033[1m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BOLD='' NC=''
fi

# ── Count TEST macros in a file ──────────────────────────────
count_tests_in_file() {
    local file="$1"
    local c
    c=$(grep -c '^TEST\b\|^TEST_F\b' "$file" 2>/dev/null) || true
    echo "${c:-0}"
}

# ── Detect compile guards ────────────────────────────────────
detect_guards() {
    local file="$1"
    local guards=""
    grep -q "HAVE_PLUGINS" "$file" 2>/dev/null && guards="${guards}PLUGINS " || true
    grep -q "HAVE_GAZEBO" "$file" 2>/dev/null && guards="${guards}GAZEBO " || true
    grep -q "HAVE_MAVSDK" "$file" 2>/dev/null && guards="${guards}MAVSDK " || true
    echo "${guards:-none}"
}

# ── Categorize a test file ───────────────────────────────────
categorize_file() {
    local basename="$1"
    case "$basename" in
        test_spsc_ring*|test_message_bus*|test_zenoh_ipc*|test_zenoh_coverage*|test_zenoh_liveliness*|test_zenoh_network*|test_ipc_validation*|test_correlation*|test_topic_resolver*|test_serializer*)
            echo "IPC" ;;
        test_hal*|test_gazebo_camera*|test_gazebo_imu*|test_gazebo_radar*|test_mavlink_fc_link*|test_radar_hal*|test_plugin_loader*|test_plugin_mock*)
            echo "HAL" ;;
        test_kalman_tracker*|test_bytetrack_tracker*|test_fusion_engine*|test_color_contour*|test_opencv_yolo*|test_simulated_detector*)
            echo "P2 Perception" ;;
        test_feature_extractor*|test_stereo_matcher*|test_imu_preintegrator*|test_vio_backend*|test_rate_clamp*)
            echo "P3 SLAM/VIO" ;;
        test_mission_fsm*|test_fault_manager*|test_static_obstacle*|test_gcs_command*|test_fault_response*|test_mission_state*|test_dstar_lite*|test_obstacle_avoider*|test_collision_recovery*|test_obstacle_prediction*|test_geofence*|test_event_bus*)
            echo "P4 Mission" ;;
        test_comms*)
            echo "P5 Comms" ;;
        test_payload_manager*|test_gimbal_auto*)
            echo "P6 Payload" ;;
        test_system_monitor*|test_process_manager*|test_process_context*)
            echo "P7 Monitor" ;;
        test_thread_heartbeat*|test_thread_health*|test_restart_policy*|test_process_graph*)
            echo "Watchdog" ;;
        test_config*|test_result*|test_json_log*|test_latency*|test_triple_buffer*|test_diagnostic*|test_sd_notify*|test_iclock*|test_ilogger*|test_replay_dispatch*)
            echo "Utility" ;;
        test_flight_recorder*)
            echo "Flight Recorder" ;;
        test_process_interfaces*)
            echo "Cross-Cutting" ;;
        *)
            echo "Other" ;;
    esac
}

# ── Main ─────────────────────────────────────────────────────

MODE="${1:-full}"

# Per-file counts
total_source=0
total_guarded_files=0
declare -A category_tests
declare -A category_files

echo -e "${BOLD}=== Test Count Audit ===${NC}"
echo ""

if [ "$MODE" != "--summary" ]; then
    printf "${BOLD}%-45s %6s  %-15s  %s${NC}\n" "File" "Tests" "Category" "Guards"
    printf "%-45s %6s  %-15s  %s\n" "---------------------------------------------" "------" "---------------" "----------"
fi

for f in "$TESTS_DIR"/test_*.cpp; do
    basename=$(basename "$f")
    count=$(count_tests_in_file "$f")
    guards=$(detect_guards "$f")
    category=$(categorize_file "$basename")
    total_source=$((total_source + count))

    # Accumulate per-category
    category_tests[$category]=$(( ${category_tests[$category]:-0} + count ))
    if [ "$count" -gt 0 ]; then
        category_files[$category]=$(( ${category_files[$category]:-0} + 1 ))
    fi

    if [ "$guards" != "none" ]; then
        total_guarded_files=$((total_guarded_files + 1))
    fi

    if [ "$MODE" != "--summary" ] && [ "$count" -gt 0 ]; then
        if [ "$guards" != "none" ]; then
            printf "${YELLOW}%-45s %6d  %-15s  %s${NC}\n" "$basename" "$count" "$category" "$guards"
        else
            printf "%-45s %6d  %-15s  %s\n" "$basename" "$count" "$category" "$guards"
        fi
    fi
done

echo ""

# ctest count (if build exists)
ctest_total=0
if [ -d "$BUILD_DIR" ]; then
    ctest_line=$(ctest -N --test-dir "$BUILD_DIR" 2>/dev/null | grep "Total Tests:") || true
    if [ -n "$ctest_line" ]; then
        ctest_total=$(echo "$ctest_line" | awk '{print $NF}')
    fi
fi

# File counts
total_files=$(ls "$TESTS_DIR"/test_*.cpp 2>/dev/null | wc -l)
files_with_tests=0
for f in "$TESTS_DIR"/test_*.cpp; do
    c=$(count_tests_in_file "$f")
    if [ "$c" -gt 0 ]; then
        files_with_tests=$((files_with_tests + 1))
    fi
done
helper_files=$((total_files - files_with_tests))
guarded_diff=$((total_source - ctest_total))

# ── Category breakdown ───────────────────────────────────────
echo -e "${BOLD}=== By Category ===${NC}"
echo ""
printf "  ${BOLD}%-20s %6s  %5s${NC}\n" "Category" "Tests" "Files"
printf "  %-20s %6s  %5s\n" "--------------------" "------" "-----"

# Sort categories by test count (descending) using TAB delimiter for multi-word names
while IFS=$'\t' read -r tests cat; do
    files=${category_files[$cat]:-0}
    printf "  %-20s %6d  %5d\n" "$cat" "$tests" "$files"
done < <(for k in "${!category_tests[@]}"; do printf '%d\t%s\n' "${category_tests[$k]}" "$k"; done | sort -rn)
printf "  %-20s %6s  %5s\n" "--------------------" "------" "-----"
printf "  ${BOLD}%-20s %6d  %5d${NC}\n" "TOTAL" "$total_source" "$files_with_tests"

echo ""

echo -e "${BOLD}=== Summary ===${NC}"
echo ""
echo "  Source TEST macros:        $total_source"
echo "  ctest registered:          $ctest_total"
echo "  Compile-guarded (delta):   $guarded_diff"
echo ""
echo "  Test files (total):        $total_files"
echo "  Files with tests:          $files_with_tests"
echo "  Helper files (0 tests):    $helper_files"
echo "  Files with compile guards: $total_guarded_files"

# ── --check mode: compare against TESTS.md baseline ─────────
if [ "$MODE" = "--check" ]; then
    echo ""
    echo -e "${BOLD}=== Baseline Check ===${NC}"

    tests_md="$PROJECT_ROOT/tests/TESTS.md"
    if [ ! -f "$tests_md" ]; then
        echo -e "${RED}ERROR: tests/TESTS.md not found${NC}"
        exit 1
    fi

    # Extract baseline from the Total row (third column = test count)
    # Row format: | **Total** | **65 C++ + 5 shell** | **1461 + 42 + 250+** | |
    baseline=$(grep -F '| **Total**' "$tests_md" | head -1 | awk -F'|' '{print $4}' | grep -oP '\d+' | head -1) || true
    if [ -z "$baseline" ]; then
        echo -e "${YELLOW}WARNING: Could not parse baseline from TESTS.md${NC}"
    else
        echo "  TESTS.md baseline:  $baseline"
        echo "  ctest actual:       $ctest_total"
        if [ "$ctest_total" -eq "$baseline" ]; then
            echo -e "  ${GREEN}MATCH${NC}"
        else
            diff=$((ctest_total - baseline))
            echo -e "  ${RED}MISMATCH: delta = $diff${NC}"
            echo "  Update tests/TESTS.md or investigate missing/extra tests."
            exit 1
        fi
    fi

    # Also check CLAUDE.md baseline
    claude_md="$PROJECT_ROOT/CLAUDE.md"
    if [ -f "$claude_md" ]; then
        claude_baseline=$(grep -oP 'currently \*\*(\d+)\*\*' "$claude_md" | grep -oP '\d+' | head -1) || true
        if [ -n "$claude_baseline" ]; then
            echo ""
            echo "  CLAUDE.md baseline: $claude_baseline"
            if [ "$ctest_total" -eq "$claude_baseline" ]; then
                echo -e "  ${GREEN}MATCH${NC}"
            else
                echo -e "  ${YELLOW}STALE: CLAUDE.md says $claude_baseline, actual is $ctest_total${NC}"
            fi
        fi
    fi
fi

echo ""

# ── Per-suite breakdown (full mode only) ─────────────────────
if [ "$MODE" = "full" ] || [ "$MODE" = "--detail" ]; then
    echo -e "${BOLD}=== Top 30 Test Suites ===${NC}"
    echo ""
    grep -rh '^TEST\b\|^TEST_F\b' "$TESTS_DIR"/test_*.cpp 2>/dev/null \
        | sed 's/TEST_F(\|TEST(//' \
        | sed 's/,.*//' \
        | sort | uniq -c | sort -rn \
        | head -30
    remaining=$(grep -rh '^TEST\b\|^TEST_F\b' "$TESTS_DIR"/test_*.cpp 2>/dev/null \
        | sed 's/TEST_F(\|TEST(//' \
        | sed 's/,.*//' \
        | sort -u | wc -l)
    echo "  ... ($remaining suites total)"
fi
