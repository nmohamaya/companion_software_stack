#!/usr/bin/env bash
# deploy/build.sh — Build the drone companion stack.
#
# Usage:
#   bash deploy/build.sh                 # Release, SHM backend
#   bash deploy/build.sh --zenoh         # Release, Zenoh backend
#   bash deploy/build.sh Debug           # Debug, SHM backend
#   bash deploy/build.sh Debug --zenoh   # Debug, Zenoh backend
#   bash deploy/build.sh --clean         # Delete build/ first
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

BUILD_DIR="${PROJECT_DIR}/build"
BUILD_TYPE="Release"
ENABLE_ZENOH="OFF"
CLEAN=0

for arg in "$@"; do
    case "$arg" in
        --zenoh)  ENABLE_ZENOH="ON" ;;
        --clean)  CLEAN=1 ;;
        Debug|Release|RelWithDebInfo|MinSizeRel) BUILD_TYPE="$arg" ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [Debug|Release] [--zenoh] [--clean]"
            exit 1
            ;;
    esac
done

# Zenoh backend configuration
ZENOH_FLAGS=""
if [[ "$ENABLE_ZENOH" == "ON" ]]; then
    if ! pkg-config --exists zenohc 2>/dev/null; then
        echo "ERROR: --zenoh requested but zenohc not found."
        echo "  Install: sudo apt install libzenohc libzenohc-dev"
        exit 1
    fi
    # Prefer secure config if ZENOH_CONFIG_PATH is provided; fall back to
    # insecure for dev/test.
    if [[ -n "${ZENOH_CONFIG_PATH:-}" ]]; then
        if [[ ! -f "$ZENOH_CONFIG_PATH" ]]; then
            echo "ERROR: ZENOH_CONFIG_PATH='${ZENOH_CONFIG_PATH}' does not exist."
            exit 1
        fi
        ZENOH_FLAGS="-DENABLE_ZENOH=ON -DZENOH_CONFIG_PATH=${ZENOH_CONFIG_PATH}"
    else
        echo "NOTE: No ZENOH_CONFIG_PATH set — using insecure mode (dev/test only)."
        ZENOH_FLAGS="-DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON"
    fi
fi

echo "══════════════════════════════════════════"
echo "  Building Drone Companion Stack"
echo "  Build type : ${BUILD_TYPE}"
echo "  IPC backend: $([ "$ENABLE_ZENOH" = "ON" ] && echo "Zenoh" || echo "SHM")"
echo "══════════════════════════════════════════"

if [[ "$CLEAN" -eq 1 ]]; then
    echo "Removing build/..."
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" ${ZENOH_FLAGS} ..
cmake --build . -j"$(nproc)"

echo ""
echo "Build complete! Binaries in: ${BUILD_DIR}/bin/"
ls -la "${BUILD_DIR}/bin/"
