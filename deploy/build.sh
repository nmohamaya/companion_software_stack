#!/usr/bin/env bash
# deploy/build.sh — Build the drone companion stack
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

BUILD_DIR="${PROJECT_DIR}/build"
BUILD_TYPE="${1:-Release}"

echo "══════════════════════════════════════════"
echo "  Building Drone Companion Stack"
echo "  Build type: ${BUILD_TYPE}"
echo "══════════════════════════════════════════"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" ..
make -j"$(nproc)"

echo ""
echo "Build complete! Binaries in: ${BUILD_DIR}/bin/"
ls -la "${BUILD_DIR}/bin/"
