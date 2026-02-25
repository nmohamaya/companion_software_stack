#!/usr/bin/env bash
# deploy/install_dependencies.sh — Install all dependencies for the Drone
# Companion Software Stack on a fresh Ubuntu 24.04 machine.
#
# Usage:
#   bash deploy/install_dependencies.sh              # Interactive — prompts for each optional dep
#   bash deploy/install_dependencies.sh --all        # Install everything (core + OpenCV + MAVSDK + Gazebo + PX4)
#   bash deploy/install_dependencies.sh --core-only  # Core dependencies only (no optional)
#
# What this installs:
#   CORE (always):  build-essential, cmake, spdlog, Eigen3, nlohmann-json, GTest
#   OPTIONAL:
#     OpenCV 4.10   — YOLOv8-nano object detection via OpenCV DNN module
#     MAVSDK 2.12   — MAVLink communication with PX4 flight controller
#     Gazebo Harmonic — 3D physics simulation (camera, IMU, odometry)
#     PX4 SITL      — Software-in-the-loop flight controller
#
# After running this script, build the stack with:
#   bash deploy/build.sh
#
# See INSTALL.md for detailed explanations and troubleshooting.
set -euo pipefail

# ── Colours & helpers ─────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No colour

info()    { echo -e "${CYAN}[INFO]${NC} $*"; }
success() { echo -e "${GREEN}[  OK]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail()    { echo -e "${RED}[FAIL]${NC} $*"; }
header()  { echo -e "\n${BOLD}══════════════════════════════════════════${NC}"; echo -e "${BOLD}  $*${NC}"; echo -e "${BOLD}══════════════════════════════════════════${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Parse arguments ──────────────────────────────────────────
MODE="interactive"  # interactive | all | core-only
OPENCV_SOURCE=1     # 1 = build from source (4.10), 0 = apt package (4.6)

for arg in "$@"; do
    case "$arg" in
        --all)        MODE="all" ;;
        --core-only)  MODE="core-only" ;;
        --opencv-apt) OPENCV_SOURCE=0 ;;
        --help|-h)
            echo "Usage: $0 [--all | --core-only] [--opencv-apt]"
            echo ""
            echo "  --all         Install all dependencies (no prompts)"
            echo "  --core-only   Install only required core dependencies"
            echo "  --opencv-apt  Use Ubuntu apt package for OpenCV (4.6) instead of building 4.10 from source"
            echo ""
            echo "With no flags, the script prompts for each optional dependency."
            exit 0
            ;;
        *)
            fail "Unknown argument: $arg"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# ── Prompt helper ────────────────────────────────────────────
ask_yes_no() {
    local prompt="$1"
    local default="${2:-y}"
    if [[ "$MODE" == "all" ]]; then
        return 0  # yes to everything
    fi
    if [[ "$MODE" == "core-only" ]]; then
        return 1  # no to optionals
    fi
    local yn
    if [[ "$default" == "y" ]]; then
        read -rp "$(echo -e "${CYAN}?${NC}") $prompt [Y/n] " yn
        yn="${yn:-y}"
    else
        read -rp "$(echo -e "${CYAN}?${NC}") $prompt [y/N] " yn
        yn="${yn:-n}"
    fi
    [[ "$yn" =~ ^[Yy] ]]
}

# ── Sudo helper: skip sudo when already root ─────────────────
if [[ "$(id -u)" -eq 0 ]]; then
    sudo() { "$@"; }          # already root — run commands directly
else
    if ! command -v sudo &>/dev/null; then
        fail "This script requires 'sudo' when not run as root. Please install sudo or re-run as root."
        exit 1
    fi
fi

# ── Pre-flight checks ────────────────────────────────────────
header "Drone Companion Stack — Dependency Installer"

if [[ "$(id -u)" -eq 0 ]]; then
    warn "Running as root. Sudo prompts will be skipped."
fi

# Check Ubuntu version
if command -v lsb_release &>/dev/null; then
    DISTRO="$(lsb_release -ds 2>/dev/null || echo 'unknown')"
    CODENAME="$(lsb_release -cs 2>/dev/null || echo 'unknown')"
    info "Detected OS: ${DISTRO} (${CODENAME})"
    if [[ "$CODENAME" != "noble" ]]; then
        warn "This script is tested on Ubuntu 24.04 (noble). Your system is '${CODENAME}'."
        warn "Things should still work on other recent Ubuntu/Debian versions."
    fi
else
    warn "lsb_release not found — cannot detect OS version."
fi

# Warn about Conda
if [[ -n "${CONDA_DEFAULT_ENV:-}" ]] || [[ -n "${CONDA_PREFIX:-}" ]]; then
    warn "Conda environment detected (${CONDA_DEFAULT_ENV:-unknown})."
    warn "Conda can cause libstdc++ conflicts. Consider running: conda deactivate"
    if ! ask_yes_no "Continue anyway?" "y"; then
        echo "Aborted. Deactivate Conda and re-run."
        exit 1
    fi
fi

echo ""
info "Project directory: ${PROJECT_DIR}"
info "Mode: ${MODE}"
echo ""

# Track what was installed for the summary
INSTALLED=()
SKIPPED=()

# ══════════════════════════════════════════════════════════════
#  STEP 1: Core build tools & required libraries
# ══════════════════════════════════════════════════════════════
header "Step 1/5: Core Dependencies (Required)"

info "Installing build tools and required libraries..."
sudo apt-get update -qq

sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    lsb-release \
    gnupg \
    libspdlog-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libgtest-dev

success "Core dependencies installed"
INSTALLED+=("Core (spdlog, Eigen3, nlohmann-json, GTest)")

# ══════════════════════════════════════════════════════════════
#  STEP 2: OpenCV (Optional)
# ══════════════════════════════════════════════════════════════
header "Step 2/5: OpenCV (Optional — YOLOv8 Object Detection)"

echo "  OpenCV's DNN module enables the YOLOv8-nano detector backend."
echo "  Without it, the stack uses ColorContourDetector (HSV + union-find)"
echo "  or SimulatedDetector (random bounding boxes)."
echo ""

INSTALL_OPENCV=false
if ask_yes_no "Install OpenCV?" "y"; then
    INSTALL_OPENCV=true
fi

if $INSTALL_OPENCV; then
    # Check if OpenCV is already installed
    EXISTING_OPENCV=""
    if pkg-config --exists opencv4 2>/dev/null; then
        EXISTING_OPENCV="$(pkg-config --modversion opencv4 2>/dev/null)"
        info "OpenCV ${EXISTING_OPENCV} already detected."
        if [[ "$EXISTING_OPENCV" == "4.10.0" ]]; then
            success "OpenCV 4.10.0 already installed — skipping."
            INSTALLED+=("OpenCV 4.10.0 (already present)")
            INSTALL_OPENCV=false
        else
            warn "Older OpenCV ${EXISTING_OPENCV} found."
            if [[ "$OPENCV_SOURCE" -eq 1 ]] && [[ "$MODE" != "core-only" ]]; then
                info "Will build OpenCV 4.10.0 from source (installs to /usr/local)."
            fi
        fi
    fi

    if $INSTALL_OPENCV; then
        if [[ "$OPENCV_SOURCE" -eq 1 ]]; then
            info "Building OpenCV 4.10.0 from source (this takes 10–20 minutes)..."

            # Install build dependencies
            sudo apt-get install -y --no-install-recommends \
                libgtk-3-dev \
                libavcodec-dev \
                libavformat-dev \
                libswscale-dev \
                libtbb-dev \
                libjpeg-dev \
                libpng-dev \
                libtiff-dev

            # Clone into /tmp
            OPENCV_BUILD_DIR="/tmp/opencv_build_$$"
            mkdir -p "$OPENCV_BUILD_DIR"
            cd "$OPENCV_BUILD_DIR"

            if [[ ! -d opencv ]]; then
                info "Cloning OpenCV 4.10.0..."
                git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
            fi
            if [[ ! -d opencv_contrib ]]; then
                info "Cloning OpenCV contrib 4.10.0..."
                git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git
            fi

            mkdir -p opencv/build && cd opencv/build

            info "Configuring OpenCV..."
            cmake \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_INSTALL_PREFIX=/usr/local \
                -DOPENCV_EXTRA_MODULES_PATH="${OPENCV_BUILD_DIR}/opencv_contrib/modules" \
                -DBUILD_LIST=core,imgproc,dnn,imgcodecs,highgui \
                -DBUILD_TESTS=OFF \
                -DBUILD_PERF_TESTS=OFF \
                -DBUILD_EXAMPLES=OFF \
                -DBUILD_opencv_python3=OFF \
                -DWITH_PROTOBUF=ON \
                -DBUILD_PROTOBUF=ON \
                -DOPENCV_DNN_OPENCL=OFF \
                ..

            info "Building OpenCV (using $(nproc) cores)..."
            make -j"$(nproc)"

            info "Installing OpenCV..."
            sudo make install
            sudo ldconfig

            # Verify
            if pkg-config --exists opencv4 2>/dev/null; then
                VER="$(pkg-config --modversion opencv4)"
                success "OpenCV ${VER} installed successfully"
                INSTALLED+=("OpenCV ${VER} (from source)")
            else
                warn "OpenCV installed but pkg-config doesn't see it."
                warn "CMake should still find it via /usr/local/lib/cmake/opencv4/"
                INSTALLED+=("OpenCV 4.10.0 (from source, no pkg-config)")
            fi

            # Cleanup
            cd "$PROJECT_DIR"
            rm -rf "$OPENCV_BUILD_DIR"
        else
            info "Installing OpenCV from Ubuntu apt (4.6.0)..."
            sudo apt-get install -y libopencv-dev
            success "OpenCV $(pkg-config --modversion opencv4 2>/dev/null || echo '4.6.0') installed from apt"
            INSTALLED+=("OpenCV (apt package)")
        fi
    fi

    # Download YOLOv8n model
    MODELS_DIR="${PROJECT_DIR}/models"
    if [[ ! -f "${MODELS_DIR}/yolov8n.onnx" ]]; then
        info "Downloading YOLOv8n ONNX model (12.8 MB)..."
        mkdir -p "$MODELS_DIR"
        wget -q --show-progress -O "${MODELS_DIR}/yolov8n.onnx" \
            "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.onnx" \
            || warn "Failed to download YOLOv8n model. You can download it later manually."
        if [[ -f "${MODELS_DIR}/yolov8n.onnx" ]]; then
            success "YOLOv8n model saved to models/yolov8n.onnx"
        fi
    else
        info "YOLOv8n model already exists at models/yolov8n.onnx"
    fi
else
    info "Skipping OpenCV."
    SKIPPED+=("OpenCV")
fi

# ══════════════════════════════════════════════════════════════
#  STEP 3: MAVSDK (Optional)
# ══════════════════════════════════════════════════════════════
header "Step 3/5: MAVSDK 2.x (Optional — PX4 MAVLink Communication)"

echo "  MAVSDK provides MAVLink communication with the PX4 flight controller."
echo "  It enables real arming, takeoff, landing, position commands, and telemetry."
echo "  Without it, the stack uses SimulatedFCLink (synthetic telemetry)."
echo "  NOTE: MAVSDK is not available via apt — it must be built from source (~15–30 min)."
echo ""

INSTALL_MAVSDK=false
if ask_yes_no "Install MAVSDK?" "y"; then
    INSTALL_MAVSDK=true
fi

if $INSTALL_MAVSDK; then
    # Check if already installed
    if [[ -f /usr/local/lib/libmavsdk.so ]]; then
        MAVSDK_VER="$(readelf -d /usr/local/lib/libmavsdk.so 2>/dev/null | grep -oP 'SONAME.*\[libmavsdk\.so\.\K[0-9.]+' || echo 'unknown')"
        info "MAVSDK already detected at /usr/local/lib/libmavsdk.so (version: ${MAVSDK_VER})"
        if ask_yes_no "Reinstall/upgrade MAVSDK?" "n"; then
            info "Proceeding with rebuild..."
        else
            success "Keeping existing MAVSDK installation."
            INSTALLED+=("MAVSDK (already present)")
            INSTALL_MAVSDK=false
        fi
    fi

    if $INSTALL_MAVSDK; then
        info "Building MAVSDK from source (this takes 15–30 minutes)..."

        # Install dependencies
        sudo apt-get install -y --no-install-recommends \
            libcurl4-openssl-dev \
            libjsoncpp-dev \
            libtinyxml2-dev

        MAVSDK_BUILD_DIR="/tmp/mavsdk_build_$$"
        mkdir -p "$MAVSDK_BUILD_DIR"
        cd "$MAVSDK_BUILD_DIR"

        info "Cloning MAVSDK (with submodules — this may take a few minutes)..."
        git clone https://github.com/mavlink/MAVSDK.git
        cd MAVSDK
        git checkout v2.12.12
        git submodule update --init --recursive

        info "Configuring MAVSDK..."
        cmake -B build \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DBUILD_SHARED_LIBS=ON

        info "Building MAVSDK (using $(nproc) cores)..."
        cmake --build build -j"$(nproc)"

        info "Installing MAVSDK..."
        sudo cmake --install build
        sudo ldconfig

        if [[ -f /usr/local/lib/libmavsdk.so ]]; then
            success "MAVSDK installed successfully"
            INSTALLED+=("MAVSDK 2.12.12 (from source)")
        else
            fail "MAVSDK installation may have failed — libmavsdk.so not found"
        fi

        # Cleanup
        cd "$PROJECT_DIR"
        rm -rf "$MAVSDK_BUILD_DIR"
    fi
else
    info "Skipping MAVSDK."
    SKIPPED+=("MAVSDK")
fi

# ══════════════════════════════════════════════════════════════
#  STEP 4: Gazebo Harmonic (Optional)
# ══════════════════════════════════════════════════════════════
header "Step 4/5: Gazebo Harmonic (Optional — 3D Physics Simulation)"

echo "  Gazebo Harmonic (gz-sim 8) provides physics-based simulation with"
echo "  rendered camera images, IMU data, and ground-truth odometry."
echo "  Combined with PX4 SITL, it creates a full simulated drone environment."
echo "  Without it, the stack uses synthetic data generators."
echo ""

INSTALL_GAZEBO=false
if ask_yes_no "Install Gazebo Harmonic?" "y"; then
    INSTALL_GAZEBO=true
fi

if $INSTALL_GAZEBO; then
    # Check if already installed
    if command -v gz &>/dev/null && gz sim --version &>/dev/null; then
        GZ_VER="$(gz sim --version 2>/dev/null | head -1)"
        info "Gazebo already detected: ${GZ_VER}"
        if ask_yes_no "Reinstall Gazebo?" "n"; then
            info "Proceeding with reinstall..."
        else
            success "Keeping existing Gazebo installation."
            # Still install dev headers if missing
            if ! dpkg -s libgz-transport13-dev &>/dev/null 2>&1; then
                info "Installing missing development headers..."
                sudo apt-get install -y libgz-transport13-dev libgz-msgs10-dev
            fi
            INSTALLED+=("Gazebo (already present)")
            INSTALL_GAZEBO=false
        fi
    fi

    if $INSTALL_GAZEBO; then
        info "Adding Gazebo package repository..."

        # Add OSRF GPG key
        sudo wget -q https://packages.osrfoundation.org/gazebo.gpg \
            -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

        # Add repo (HTTPS to reduce MITM/downgrade risk)
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

        sudo apt-get update -qq

        info "Installing Gazebo Harmonic (this may take a few minutes)..."
        sudo apt-get install -y gz-harmonic

        info "Installing Gazebo C++ development headers..."
        sudo apt-get install -y \
            libgz-transport13-dev \
            libgz-msgs10-dev

        if command -v gz &>/dev/null; then
            GZ_VER="$(gz sim --version 2>/dev/null | head -1)"
            success "Gazebo installed: ${GZ_VER}"
            INSTALLED+=("${GZ_VER}")
        else
            fail "Gazebo installation may have failed — 'gz' command not found"
        fi
    fi
else
    info "Skipping Gazebo."
    SKIPPED+=("Gazebo")
fi

# ══════════════════════════════════════════════════════════════
#  STEP 5: PX4 SITL (Optional — requires Gazebo + MAVSDK)
# ══════════════════════════════════════════════════════════════
header "Step 5/5: PX4 SITL (Optional — Flight Controller Simulator)"

echo "  PX4 SITL runs a real PX4 flight controller in software, connected"
echo "  to Gazebo for physics. Requires Gazebo and MAVSDK to be useful."
echo "  NOTE: PX4 build installs many packages via its own setup script"
echo "  and takes 20–40 minutes to compile."
echo ""

INSTALL_PX4=false
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"

# Detect whether Gazebo and MAVSDK are available (installed now or previously)
HAS_GAZEBO_NOW=false
HAS_MAVSDK_NOW=false
if $INSTALL_GAZEBO || [[ -n "${INSTALLED[*]}" && "${INSTALLED[*]}" == *"Gazebo"* ]]; then
    HAS_GAZEBO_NOW=true
fi
if ${INSTALL_MAVSDK:-false} || [[ -n "${INSTALLED[*]}" && "${INSTALLED[*]}" == *"MAVSDK"* ]]; then
    HAS_MAVSDK_NOW=true
fi

if $HAS_GAZEBO_NOW && $HAS_MAVSDK_NOW; then
    if ask_yes_no "Install/build PX4 SITL at ${PX4_DIR}?" "y"; then
        INSTALL_PX4=true
    fi
elif ! $HAS_GAZEBO_NOW && ! $HAS_MAVSDK_NOW; then
    info "Gazebo and MAVSDK were not installed — PX4 SITL is only useful with both."
    if ask_yes_no "Install PX4 SITL anyway?" "n"; then
        INSTALL_PX4=true
    fi
elif ! $HAS_GAZEBO_NOW; then
    info "Gazebo was not installed — PX4 SITL is only useful with Gazebo."
    if ask_yes_no "Install PX4 SITL anyway?" "n"; then
        INSTALL_PX4=true
    fi
else
    info "MAVSDK was not installed — PX4 SITL is only useful with MAVSDK."
    if ask_yes_no "Install PX4 SITL anyway?" "n"; then
        INSTALL_PX4=true
    fi
fi

if $INSTALL_PX4; then
    if [[ -d "$PX4_DIR" ]] && [[ -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
        info "PX4 already found at ${PX4_DIR} with SITL binary."
        if ask_yes_no "Re-clone and rebuild PX4?" "n"; then
            info "Proceeding with rebuild..."
        else
            success "Keeping existing PX4 installation."
            INSTALL_PX4=false
            INSTALLED+=("PX4 SITL (already present at ${PX4_DIR})")
        fi
    fi

    if $INSTALL_PX4; then
        info "Cloning PX4-Autopilot (with submodules — this takes a few minutes)..."
        if [[ -d "$PX4_DIR" ]]; then
            warn "Directory ${PX4_DIR} already exists. Pulling latest..."
            cd "$PX4_DIR"
            git pull || true
            git submodule update --init --recursive
        else
            cd "$(dirname "$PX4_DIR")"
            git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$(basename "$PX4_DIR")"
            cd "$PX4_DIR"
        fi

        info "Running PX4 setup script (installs build dependencies)..."
        # The PX4 script may prompt — pass --no-nuttx to skip NuttX for SITL-only
        bash Tools/setup/ubuntu.sh --no-nuttx || {
            warn "PX4 setup script returned non-zero. Trying to build anyway..."
        }

        info "Building PX4 SITL target (using $(nproc) cores — this takes 20–40 minutes)..."
        make -j"$(nproc)" px4_sitl_default

        if [[ -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
            success "PX4 SITL built successfully"
            INSTALLED+=("PX4 SITL (at ${PX4_DIR})")
        else
            fail "PX4 SITL build may have failed — binary not found"
        fi

        cd "$PROJECT_DIR"
    fi

    # Set up simulation asset symlinks
    if [[ -d "$PX4_DIR" ]]; then
        info "Setting up simulation asset symlinks..."

        GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
        GZ_MODELS_DIR="${PX4_DIR}/Tools/simulation/gz/models"
        mkdir -p "$GZ_WORLDS_DIR" "$GZ_MODELS_DIR"

        if [[ -d "${PROJECT_DIR}/sim/worlds" ]] && [[ -d "$GZ_WORLDS_DIR" ]]; then
            ln -sf "${PROJECT_DIR}/sim/worlds/test_world.sdf" \
                "${GZ_WORLDS_DIR}/test_world.sdf" 2>/dev/null && \
                success "Linked test_world.sdf into PX4 worlds" || \
                warn "Could not create world symlink"
        fi

        if [[ -d "${PROJECT_DIR}/sim/models/x500_companion" ]] && [[ -d "$GZ_MODELS_DIR" ]]; then
            ln -sf "${PROJECT_DIR}/sim/models/x500_companion" \
                "${GZ_MODELS_DIR}/x500_companion" 2>/dev/null && \
                success "Linked x500_companion model into PX4 models" || \
                warn "Could not create model symlink"
        fi
    fi
else
    info "Skipping PX4 SITL."
    SKIPPED+=("PX4 SITL")
fi

# ══════════════════════════════════════════════════════════════
#  BUILD THE COMPANION STACK
# ══════════════════════════════════════════════════════════════
header "Building Companion Stack"

cd "$PROJECT_DIR"

info "Configuring with CMake..."
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. 2>&1 | tee cmake_configure.log

echo ""
info "Building (using $(nproc) cores)..."
make -j"$(nproc)"

echo ""
info "Running tests..."
ctest --output-on-failure -j"$(nproc)" 2>&1 | tee ctest.log

cd "$PROJECT_DIR"

# ══════════════════════════════════════════════════════════════
#  SUMMARY
# ══════════════════════════════════════════════════════════════
header "Installation Complete"

echo ""
echo -e "${BOLD}Installed:${NC}"
for item in "${INSTALLED[@]}"; do
    echo -e "  ${GREEN}✓${NC} ${item}"
done

if [[ ${#SKIPPED[@]} -gt 0 ]]; then
    echo ""
    echo -e "${BOLD}Skipped (can install later by re-running this script):${NC}"
    for item in "${SKIPPED[@]}"; do
        echo -e "  ${YELLOW}○${NC} ${item}"
    done
fi

echo ""
echo -e "${BOLD}Next steps:${NC}"
echo "  1. Build:       bash deploy/build.sh"
echo "  2. Run tests:   cd build && ctest --output-on-failure"
echo "  3. Run stack:   bash deploy/launch_all.sh"
if $INSTALL_GAZEBO || [[ "${INSTALLED[*]}" == *"Gazebo"* ]]; then
    echo "  4. Run SITL:    bash deploy/launch_gazebo.sh --gui"
fi
echo ""
echo "  See INSTALL.md for troubleshooting and detailed documentation."
echo ""
