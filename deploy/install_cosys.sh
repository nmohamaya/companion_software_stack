#!/usr/bin/env bash
# deploy/install_cosys.sh — Install Cosys-AirSim and verify prerequisites.
#
# Usage:
#   bash deploy/install_cosys.sh                # Interactive install
#   bash deploy/install_cosys.sh --check-only   # Verify GPU and deps only
#   bash deploy/install_cosys.sh --prebuilt     # Download pre-built binaries
#
# Exit codes:
#   0 = success
#   1 = GPU insufficient (no NVIDIA GPU or < 8 GB VRAM)
#   2 = build/install failed
#   3 = prerequisites missing
#
# Environment variables:
#   COSYS_DIR        Install location (default: ~/Cosys-AirSim)
#   COSYS_BRANCH     Git branch       (default: main)
#
# Issue: #463 (part of Epic #459)
set -euo pipefail

# ── Colours & helpers ─────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC} $*"; }
success() { echo -e "${GREEN}[  OK]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail()    { echo -e "${RED}[FAIL]${NC} $*"; }
header()  { echo -e "\n${BOLD}══════════════════════════════════════════${NC}"; echo -e "${BOLD}  $*${NC}"; echo -e "${BOLD}══════════════════════════════════════════${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Defaults ──────────────────────────────────────────────────
COSYS_DIR="${COSYS_DIR:-${HOME}/Cosys-AirSim}"
COSYS_BRANCH="${COSYS_BRANCH:-main}"
COSYS_REPO="https://github.com/Cosys-Lab/Cosys-AirSim.git"
COSYS_RPC_PORT=41451
CHECK_ONLY=false
USE_PREBUILT=false

# ── Parse arguments ──────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --check-only)  CHECK_ONLY=true ;;
        --prebuilt)    USE_PREBUILT=true ;;
        --help|-h)
            echo "Usage: $0 [--check-only] [--prebuilt]"
            echo ""
            echo "  --check-only   Verify GPU capability and prerequisites only"
            echo "  --prebuilt     Download pre-built environment binaries"
            echo ""
            echo "Environment variables:"
            echo "  COSYS_DIR      Install location (default: ~/Cosys-AirSim)"
            echo "  COSYS_BRANCH   Git branch       (default: main)"
            exit 0
            ;;
    esac
done

# ══════════════════════════════════════════════════════════════
# Step 1: GPU Capability Check
# ══════════════════════════════════════════════════════════════
header "GPU Capability Check"

if ! command -v nvidia-smi &>/dev/null; then
    fail "nvidia-smi not found. NVIDIA GPU with >= 8 GB VRAM is required."
    fail "Install NVIDIA drivers: https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/"
    exit 1
fi

GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")
VRAM_MB=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ' || echo "0")
DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "unknown")

info "GPU: ${GPU_NAME}"
info "VRAM: ${VRAM_MB} MB"
info "Driver: ${DRIVER_VER}"

if [[ "$VRAM_MB" -lt 8000 ]]; then
    fail "GPU VRAM is ${VRAM_MB} MB — Cosys-AirSim requires >= 8 GB (8000 MB)."
    fail "UE5 rendering will not work reliably with less VRAM."
    exit 1
fi
success "GPU meets requirements (${VRAM_MB} MB VRAM)"

# Check CUDA toolkit
if command -v nvcc &>/dev/null; then
    CUDA_VER=$(nvcc --version 2>/dev/null | grep "release" | sed 's/.*release //' | sed 's/,.*//')
    success "CUDA toolkit: ${CUDA_VER}"
else
    warn "nvcc not found — CUDA toolkit may be needed for building"
fi

# ══════════════════════════════════════════════════════════════
# Step 2: System Prerequisites
# ══════════════════════════════════════════════════════════════
header "System Prerequisites"

MISSING_DEPS=()

# Check build tools
for cmd in cmake g++ git python3 pip3; do
    if command -v "$cmd" &>/dev/null; then
        success "${cmd} found"
    else
        MISSING_DEPS+=("$cmd")
        fail "${cmd} not found"
    fi
done

# Check Vulkan support (required by UE5)
if command -v vulkaninfo &>/dev/null; then
    success "Vulkan tools found"
elif dpkg -l 2>/dev/null | grep -q "libvulkan-dev"; then
    success "Vulkan dev libraries found"
else
    warn "Vulkan tools not found — UE5 requires Vulkan support"
    warn "Install: sudo apt install vulkan-tools libvulkan-dev"
fi

if [[ ${#MISSING_DEPS[@]} -gt 0 ]]; then
    fail "Missing dependencies: ${MISSING_DEPS[*]}"
    info "Install with: sudo apt install build-essential cmake git python3 python3-pip"
    if [[ "$CHECK_ONLY" == "true" ]]; then
        exit 3
    fi
fi

if [[ "$CHECK_ONLY" == "true" ]]; then
    echo ""
    success "All checks passed. System is ready for Cosys-AirSim."
    exit 0
fi

# ══════════════════════════════════════════════════════════════
# Step 3: Clone / Update Cosys-AirSim
# ══════════════════════════════════════════════════════════════
header "Cosys-AirSim Installation"

if [[ -d "$COSYS_DIR" ]]; then
    info "Cosys-AirSim already present at ${COSYS_DIR}"
    info "Updating to branch '${COSYS_BRANCH}'..."
    pushd "$COSYS_DIR" > /dev/null
    git fetch origin 2>/dev/null || warn "git fetch failed (network?)"
    git checkout "$COSYS_BRANCH" 2>/dev/null || warn "Branch checkout failed"
    git pull --ff-only 2>/dev/null || warn "git pull failed"
    popd > /dev/null
    success "Repository updated"
else
    info "Cloning Cosys-AirSim to ${COSYS_DIR}..."
    info "Repository: ${COSYS_REPO}"
    info "Branch: ${COSYS_BRANCH}"
    if ! git clone --branch "$COSYS_BRANCH" --depth 1 "$COSYS_REPO" "$COSYS_DIR"; then
        fail "git clone failed"
        exit 2
    fi
    success "Repository cloned"
fi

# ══════════════════════════════════════════════════════════════
# Step 4: Build or Download
# ══════════════════════════════════════════════════════════════
if [[ "$USE_PREBUILT" == "true" ]]; then
    header "Pre-built Environment Download"
    info "Pre-built Cosys-AirSim environments can be downloaded from:"
    info "  https://github.com/Cosys-Lab/Cosys-AirSim/releases"
    info ""
    info "Download the environment matching your platform and extract to:"
    info "  ${COSYS_DIR}/Unreal/Environments/<EnvName>/"
    info ""
    info "Common environments: Blocks, Neighborhood, LandscapeMountains"
    warn "Automated download not yet implemented — please download manually."
else
    header "Build Instructions"
    info "To build Cosys-AirSim from source:"
    info ""
    info "  1. Install Unreal Engine 5 (see Epic Games Launcher or build from source)"
    info "  2. Build the AirSim plugin:"
    info "     cd ${COSYS_DIR}"
    info "     ./setup.sh"
    info "     ./build.sh"
    info ""
    info "  3. Build a UE5 environment (e.g., Blocks):"
    info "     cd ${COSYS_DIR}/Unreal/Environments/Blocks"
    info "     # Open in UE5 editor and package for Linux"
    info ""
    info "For detailed instructions, see:"
    info "  ${COSYS_DIR}/docs/build_linux.md"
fi

# ══════════════════════════════════════════════════════════════
# Step 5: Deploy Settings
# ══════════════════════════════════════════════════════════════
header "Settings Deployment"

COSYS_SETTINGS="${PROJECT_DIR}/config/cosys_settings.json"
if [[ -f "$COSYS_SETTINGS" ]]; then
    AIRSIM_SETTINGS_DIR="${HOME}/Documents/AirSim"
    mkdir -p "$AIRSIM_SETTINGS_DIR"
    cp "$COSYS_SETTINGS" "${AIRSIM_SETTINGS_DIR}/settings.json"
    success "Deployed settings to ${AIRSIM_SETTINGS_DIR}/settings.json"
else
    warn "No cosys_settings.json found in config/ — using Cosys-AirSim defaults"
fi

# ══════════════════════════════════════════════════════════════
# Step 6: Verification
# ══════════════════════════════════════════════════════════════
header "Verification"

# Check if any built environment exists
ENV_FOUND=false
if [[ -d "${COSYS_DIR}/Unreal/Environments" ]]; then
    for env_dir in "${COSYS_DIR}/Unreal/Environments"/*/; do
        if [[ -d "$env_dir" ]]; then
            ENV_NAME=$(basename "$env_dir")
            success "Found environment: ${ENV_NAME}"
            ENV_FOUND=true
        fi
    done
fi
# Also check for standalone packaged environments
for env_script in "${COSYS_DIR}"/*/LinuxNoEditor/*.sh; do
    if [[ -x "$env_script" ]]; then
        success "Found packaged environment: $(dirname "$(dirname "$env_script")" | xargs basename)"
        ENV_FOUND=true
    fi
done 2>/dev/null

if [[ "$ENV_FOUND" == "false" ]]; then
    warn "No built UE5 environments found yet"
    warn "Build or download an environment before running launch_cosys.sh"
fi

# Check RPC port availability (should not be in use before launch)
if ss -tlnp 2>/dev/null | grep -q ":${COSYS_RPC_PORT}"; then
    warn "Port ${COSYS_RPC_PORT} is already in use — another Cosys-AirSim instance may be running"
else
    success "RPC port ${COSYS_RPC_PORT} is available"
fi

echo ""
header "Installation Complete"
info "Next steps:"
info "  1. Build/download a UE5 environment (if not done)"
info "  2. Build the companion stack: bash deploy/build.sh"
info "  3. Launch: bash deploy/launch_cosys.sh"
info "  4. Launch with GUI: bash deploy/launch_cosys.sh --gui"
echo ""
success "Cosys-AirSim setup complete."
exit 0
