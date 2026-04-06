#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

BASE="${1:-origin/main}"

# Get changed files
CHANGED_FILES=$(git diff --name-only "$BASE"...HEAD 2>/dev/null || true)
if [[ -z "$CHANGED_FILES" ]]; then
    echo "No changed files detected against $BASE"
    exit 0
fi

# Detect role from branch name
BRANCH=$(git branch --show-current 2>/dev/null || echo "")
ROLE=""
case "$BRANCH" in
    *perception*)                           ROLE="feature-perception" ;;
    *nav*|*mission*|*slam*)                 ROLE="feature-nav" ;;
    *comms*|*integration*|*monitor*)        ROLE="feature-integration" ;;
    *infra*|*common*|*modularity*)          ROLE="feature-infra-core" ;;
    *platform*|*deploy*|*ci-*|*cert*)       ROLE="feature-infra-platform" ;;
esac

if [[ -z "$ROLE" ]]; then
    echo "No agent role detected from branch name, skipping boundary check"
    exit 0
fi

echo "Branch: $BRANCH"
echo "Detected role: $ROLE"
echo "Base ref: $BASE"
echo ""

# Always-allowed patterns (all roles)
ALWAYS_ALLOWED=(
    "docs/"
    "tasks/"
    "CLAUDE.md"
    "tests/TESTS.md"
    "tests/CMakeLists.txt"
)

# Role-specific allowed patterns
declare -a ALLOWED
case "$ROLE" in
    feature-perception)
        ALLOWED=(
            "process1_video_capture/"
            "process2_perception/"
            "common/hal/include/hal/*camera*"
            "common/hal/include/hal/*detector*"
            "tests/test_*perception*"
            "tests/test_*camera*"
            "tests/test_*detector*"
            "tests/test_*tracker*"
            "tests/test_*fusion*"
        )
        ;;
    feature-nav)
        ALLOWED=(
            "process3_slam_vio_nav/"
            "process4_mission_planner/"
            "common/hal/include/hal/*planner*"
            "common/hal/include/hal/*avoider*"
            "tests/test_*mission*"
            "tests/test_*slam*"
            "tests/test_*nav*"
            "tests/test_*planner*"
        )
        ;;
    feature-integration)
        ALLOWED=(
            "process5_comms/"
            "process6_payload_manager/"
            "process7_system_monitor/"
            "common/ipc/"
            "common/hal/include/hal/*fc_link*"
            "common/hal/include/hal/*gcs*"
            "common/hal/include/hal/*gimbal*"
            "common/hal/include/hal/*imu*"
            "tests/test_*comms*"
            "tests/test_*ipc*"
            "tests/test_*hal*"
            "tests/test_*monitor*"
        )
        ;;
    feature-infra-core)
        ALLOWED=(
            "common/util/"
            "common/recorder/"
            "CMakeLists.txt"
            "*/CMakeLists.txt"
            "config/default.json"
            "tests/test_*util*"
            "tests/test_*config*"
            "tests/test_*replay*"
        )
        ;;
    feature-infra-platform)
        ALLOWED=(
            "deploy/"
            "scripts/"
            ".github/"
            "boards/"
            "config/customers/"
        )
        ;;
esac

# Check each changed file
VIOLATIONS=()

file_matches_pattern() {
    local file="$1"
    local pattern="$2"

    # Handle glob patterns using bash pattern matching
    # Convert glob to a check: if pattern ends with /, it's a directory prefix
    if [[ "$pattern" == */ ]]; then
        [[ "$file" == "$pattern"* ]] && return 0
    elif [[ "$pattern" == *"*"* ]]; then
        # Use bash extended globbing for wildcard patterns
        # shellcheck disable=SC2254
        case "$file" in
            $pattern) return 0 ;;
        esac
    else
        [[ "$file" == "$pattern" ]] && return 0
    fi
    return 1
}

while IFS= read -r file; do
    [[ -z "$file" ]] && continue

    MATCHED=false

    # Check always-allowed
    for pattern in "${ALWAYS_ALLOWED[@]}"; do
        if file_matches_pattern "$file" "$pattern"; then
            MATCHED=true
            break
        fi
    done

    if [[ "$MATCHED" == true ]]; then
        continue
    fi

    # Check role-specific
    for pattern in "${ALLOWED[@]}"; do
        if file_matches_pattern "$file" "$pattern"; then
            MATCHED=true
            break
        fi
    done

    if [[ "$MATCHED" != true ]]; then
        VIOLATIONS+=("$file")
    fi
done <<< "$CHANGED_FILES"

# Report results
if [[ ${#VIOLATIONS[@]} -eq 0 ]]; then
    echo "All ${#CHANGED_FILES[@]} changed files are within $ROLE boundaries."
    exit 0
else
    echo "BOUNDARY VIOLATIONS for role $ROLE:"
    echo ""
    for v in "${VIOLATIONS[@]}"; do
        echo "  - $v"
    done
    echo ""
    echo "${#VIOLATIONS[@]} file(s) outside allowed boundaries for $ROLE"
    exit 1
fi
