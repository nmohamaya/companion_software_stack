#!/usr/bin/env bash
# deploy/setup.sh — Complete setup orchestrator for the Drone Companion Software Stack
#
# Usage:
#   bash deploy/setup.sh                     # Interactive: prompt for each step
#   bash deploy/setup.sh --auto              # Fully automated (install all, build, test, launch)
#   bash deploy/setup.sh --deps-only         # Install dependencies and exit
#   bash deploy/setup.sh --build-test        # Build + test only
#
# This script handles:
#   1. System checks (OS version, dependencies)
#   2. Dependency installation (interactive or automated)
#   3. Clean build with verification
#   4. Test validation (927 tests expected)
#   5. Launch demo (optional)
#
# Purpose: Simplify the "git clone → working build" flow for new developers

set -euo pipefail

# ── Colours & helpers ─────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No colour

info()    { echo -e "${CYAN}[INFO]${NC} $*"; }
success() { echo -e "${GREEN}[  ✓ ]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail()    { echo -e "${RED}[FAIL]${NC} $*"; exit 1; }
header()  { echo -e "\n${BOLD}════════════════════════════════════════════════════════${NC}"; echo -e "${BOLD}  $*${NC}"; echo -e "${BOLD}════════════════════════════════════════════════════════${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Parse arguments ──────────────────────────────────────────
MODE="interactive"  # interactive | auto | deps-only | build-test
LAUNCH_AFTER=0      # 1 = launch gazebo after build

for arg in "$@"; do
    case "$arg" in
        --auto)        MODE="auto"; LAUNCH_AFTER=0 ;;
        --auto-launch) MODE="auto"; LAUNCH_AFTER=1 ;;
        --deps-only)   MODE="deps-only" ;;
        --build-test)  MODE="build-test" ;;
        --help|-h)
            cat <<EOF
Setup orchestrator for Companion Software Stack

Usage:
  bash deploy/setup.sh                     Interactive mode
  bash deploy/setup.sh --auto              Automated (install + build + test)
  bash deploy/setup.sh --auto-launch       Automated + launch Gazebo SITL
  bash deploy/setup.sh --deps-only         Install dependencies only
  bash deploy/setup.sh --build-test        Build + test (assume deps installed)

Automated mode installs:
  - Core dependencies (always)
  - Optional: OpenCV, MAVSDK, Gazebo, Zenoh

Then:
  - Clean builds project
  - Runs all 927 tests
  - Optionally launches Gazebo SITL demo

Expected test count: 927 (if < 927, clean rebuild required)

EOF
            exit 0
            ;;
        *)
            fail "Unknown argument: $arg"
            ;;
    esac
done

# ── Sudo helper ──────────────────────────────────────────────
if [[ "$(id -u)" -eq 0 ]]; then
    sudo() { "$@"; }
else
    command -v sudo &>/dev/null || fail "This script requires 'sudo'. Run as root or install sudo."
fi

# ═════════════════════════════════════════════════════════════
# STEP 1: System checks
# ═════════════════════════════════════════════════════════════

step_system_checks() {
    header "STEP 1: System Checks"
    
    # Detect OS
    if ! command -v lsb_release &>/dev/null; then
        warn "lsb_release not found; assuming Ubuntu 22.04+"
    else
        OS_CODENAME=$(lsb_release -cs)
        OS_VERSION=$(lsb_release -rs)
        info "Detected: Ubuntu $OS_VERSION ($OS_CODENAME)"
        
        if ! [[ "$OS_CODENAME" =~ ^(jammy|noble)$ ]]; then
            warn "Ubuntu $OS_CODENAME not officially tested (22.04 LTS [jammy] or 24.04 LTS [noble] recommended)"
        fi
    fi
    
    # Detect architecture
    ARCH=$(uname -m)
    info "Architecture: $ARCH"
    if [[ "$ARCH" != "x86_64" ]] && [[ "$ARCH" != "aarch64" ]]; then
        warn "Unusual architecture: $ARCH (expected x86_64 or aarch64)"
    fi
    
    # Check resources
    RAM_GB=$(free -b | awk 'NR==2 {print int($2/1024/1024/1024)}')
    DISK_GB=$(df / | awk 'NR==2 {print int($4/1024/1024)}')
    CORES=$(nproc)
    
    info "Resources: ${RAM_GB}GB RAM, ${DISK_GB}GB free disk, $CORES CPU cores"
    
    if (( RAM_GB < 4 )); then
        warn "Less than 4GB RAM; build may be slow or fail"
    fi
    if (( DISK_GB < 8 )); then
        fail "Less than 8GB free disk space required"
    fi
    
    success "System check passed"
}

# ═════════════════════════════════════════════════════════════
# STEP 2: Install dependencies
# ═════════════════════════════════════════════════════════════

step_install_deps() {
    header "STEP 2: Install Dependencies"
    
    if [[ "$MODE" == "deps-only" ]] || [[ "$MODE" == "auto" ]] || [[ "$MODE" == "auto-launch" ]]; then
        # Invoke install_dependencies.sh with appropriate flags
        if [[ "$MODE" == "deps-only" ]]; then
            info "Interactive mode — will prompt for optional features"
            bash "$PROJECT_DIR/deploy/install_dependencies.sh"
        elif [[ "$MODE" == "auto" ]] || [[ "$MODE" == "auto-launch" ]]; then
            info "Installing all dependencies (core + optional)"
            bash "$PROJECT_DIR/deploy/install_dependencies.sh" --all
        fi
    elif [[ "$MODE" == "interactive" ]]; then
        # Check if deps are already installed
        if command -v cmake &>/dev/null && \
           dpkg -l | grep -q libspdlog-dev && \
           dpkg -l | grep -q libeigen3-dev && \
           dpkg -l | grep -q nlohmann-json3-dev && \
           dpkg -l | grep -q libgtest-dev; then
            success "Core dependencies already installed"
            read -rp "$(echo -e "${CYAN}?${NC}") Install optional dependencies? [y/N] " opt
            if [[ "$opt" =~ ^[Yy]$ ]]; then
                bash "$PROJECT_DIR/deploy/install_dependencies.sh"
            else
                success "Skipping optional dependencies"
            fi
        else
            warn "Some core dependencies missing"
            read -rp "$(echo -e "${CYAN}?${NC}") Run installer? [Y/n] " run_install
            if [[ "$run_install" != "n" ]]; then
                bash "$PROJECT_DIR/deploy/install_dependencies.sh"
            else
                fail "Core dependencies required to continue"
            fi
        fi
    elif [[ "$MODE" == "build-test" ]]; then
        info "Skipping dependency installation (build-test mode)"
    fi
    
    success "Dependency check complete"
}

# ═════════════════════════════════════════════════════════════
# STEP 3: Build
# ═════════════════════════════════════════════════════════════

step_build() {
    header "STEP 3: Build"
    
    info "Running clean build..."
    if ! bash "$PROJECT_DIR/deploy/build.sh" --clean; then
        fail "Build failed; see errors above"
    fi
    
    if [[ ! -f "$PROJECT_DIR/build/bin/video_capture" ]]; then
        fail "Build succeeded but binary not found; check CMake output"
    fi
    
    success "Build complete; binaries in build/bin/"
}

# ═════════════════════════════════════════════════════════════
# STEP 4: Test verification
# ═════════════════════════════════════════════════════════════

step_test() {
    header "STEP 4: Test Verification"
    
    # Check test count
    TEST_COUNT=$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null | grep -oP '(?<=Total Tests: )\d+' || echo "0")
    info "Found $TEST_COUNT tests (expected: 927)"
    
    if [[ "$TEST_COUNT" -ne 927 ]]; then
        warn "Test count mismatch (expected 927, got $TEST_COUNT)"
        warn "This can happen when CMake cache is stale from different build types"
        warn "Attempting clean rebuild..."
        bash "$PROJECT_DIR/deploy/build.sh" --clean
        TEST_COUNT=$(ctest -N --test-dir "$PROJECT_DIR/build" 2>/dev/null | grep -oP '(?<=Total Tests: )\d+' || echo "0")
        info "After rebuild: $TEST_COUNT tests"
        if [[ "$TEST_COUNT" -ne 927 ]]; then
            fail "Still got wrong test count after rebuild; contact developers"
        fi
    fi
    
    info "Running all tests (~1-2 minutes)..."
    if bash "$PROJECT_DIR/tests/run_tests.sh"; then
        success "All 927 tests passed ✓"
    else
        fail "Tests failed; see output above"
    fi
}

# ═════════════════════════════════════════════════════════════
# STEP 5: Optional launch
# ═════════════════════════════════════════════════════════════

step_launch() {
    header "STEP 5: Launch"
    
    local should_launch=0
    
    if [[ "$LAUNCH_AFTER" -eq 1 ]]; then
        should_launch=1
    elif [[ "$MODE" == "interactive" ]]; then
        read -rp "$(echo -e "${CYAN}?${NC}") Launch standalone demo? [Y/n] " launch_resp
        [[ "$launch_resp" != "n" ]] && should_launch=1
    fi
    
    if [[ "$should_launch" -eq 1 ]]; then
        info "Launching all 7 processes with simulated sensors..."
        info "  Logs: drone_logs/"
        info "  Press Ctrl+C to stop gracefully"
        bash "$PROJECT_DIR/deploy/launch_all.sh"
    else
        success "Setup complete! To launch, run:"
        echo "  bash deploy/launch_all.sh              # Standalone demo"
        echo "  bash deploy/launch_gazebo.sh --gui    # Gazebo SITL"
    fi
}

# ═════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════

main() {
    header "Companion Software Stack — Complete Setup"
    info "Mode: $MODE"
    info "Project directory: $PROJECT_DIR"
    
    # Check we're in the right place
    if [[ ! -f "$PROJECT_DIR/CMakeLists.txt" ]]; then
        fail "CMakeLists.txt not found; are you in the project root?"
    fi
    
    # Run appropriate steps based on mode
    step_system_checks
    
    if [[ "$MODE" != "build-test" ]]; then
        step_install_deps
    fi
    
    step_build
    step_test
    
    if [[ "$MODE" == "deps-only" ]]; then
        success "Dependencies installed. To build: bash deploy/build.sh"
    else
        if [[ "$MODE" == "auto" ]] || [[ "$MODE" == "auto-launch" ]]; then
            step_launch
        elif [[ "$MODE" == "interactive" ]]; then
            step_launch
        else
            # build-test mode
            success "Build and tests complete!"
            echo ""
            echo "Next steps:"
            echo "  1. Standalone:     bash deploy/launch_all.sh"
            echo "  2. Gazebo SITL:    bash deploy/launch_gazebo.sh --gui"
            echo "  3. Modify config:  nano config/default.json"
            echo "  4. Contribute:     see DEVELOPMENT_WORKFLOW.md"
        fi
    fi
    
    header "Setup Complete! ✓"
    echo ""
    echo "👉 Next: Read docs/guides/GETTING_STARTED.md for more info"
    echo "👉 Next: Check https://github.com/nmohamaya/companion_software_stack"
}

main "$@"
