#!/usr/bin/env bash
# build_zenohc_deb.sh — Build zenoh-c .deb with shared-memory feature enabled.
#
# The upstream zenoh-c release packages omit the shared-memory cargo feature,
# which means our SHM zero-copy tests skip in CI. This script builds zenoh-c
# from source with ZENOHC_BUILD_WITH_SHARED_MEMORY=ON, packages it as a .deb,
# and outputs it for upload to GitHub Releases.
#
# Usage:
#   bash deploy/build_zenohc_deb.sh
#
# Prerequisites:
#   - Rust toolchain (rustup) — script installs the exact version from zenoh-c's rust-toolchain.toml
#   - CMake, build-essential, dpkg-deb
#
# After building, upload the .deb to a GitHub Release:
#   gh release create "zenohc-v${VERSION}" ./libzenohc-shm_*.deb \
#     --title "zenohc ${VERSION} with SHM" \
#     --notes "Custom zenoh-c build with shared-memory feature enabled for CI."
#
set -euo pipefail

ZENOH_VERSION="${ZENOH_VERSION:-1.7.2}"
WORK_DIR="/tmp/zenohc-shm-build"
ARCH="$(dpkg --print-architecture)"

echo "=== Building zenoh-c ${ZENOH_VERSION} with shared-memory for ${ARCH} ==="

# Clean previous build
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}"

# Clone zenoh-c at the pinned version
echo "--- Cloning zenoh-c ${ZENOH_VERSION} ---"
git clone --depth 1 --branch "${ZENOH_VERSION}" \
    https://github.com/eclipse-zenoh/zenoh-c.git "${WORK_DIR}/zenoh-c"

# Read the exact Rust toolchain version from zenoh-c's rust-toolchain.toml.
# This is critical — mismatched Rust versions cause opaque-type size assertion
# failures (see CI_ISSUES.md CI-003).
RUST_TOOLCHAIN_FILE="${WORK_DIR}/zenoh-c/rust-toolchain.toml"
if [ ! -f "${RUST_TOOLCHAIN_FILE}" ]; then
    echo "ERROR: rust-toolchain.toml not found in zenoh-c repo"
    exit 1
fi
RUST_CHANNEL=$(grep 'channel' "${RUST_TOOLCHAIN_FILE}" | head -1 | sed 's/.*"\(.*\)".*/\1/')
echo "--- Installing Rust toolchain: ${RUST_CHANNEL} ---"
rustup install "${RUST_CHANNEL}"
rustup default "${RUST_CHANNEL}"

# Build zenoh-c with shared-memory enabled
echo "--- Configuring CMake with ZENOHC_BUILD_WITH_SHARED_MEMORY=ON ---"
cmake -S "${WORK_DIR}/zenoh-c" -B "${WORK_DIR}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DZENOHC_BUILD_WITH_SHARED_MEMORY=ON \
    -DCMAKE_INSTALL_PREFIX=/usr

echo "--- Building (this takes several minutes) ---"
cmake --build "${WORK_DIR}/build" -j"$(nproc)"

# Install to staging directory for packaging
STAGING="${WORK_DIR}/staging"
DESTDIR="${STAGING}" cmake --install "${WORK_DIR}/build"

# Create .deb package
PKG_NAME="libzenohc-shm"
PKG_DIR="${WORK_DIR}/${PKG_NAME}_${ZENOH_VERSION}_${ARCH}"
mkdir -p "${PKG_DIR}/DEBIAN"

cat > "${PKG_DIR}/DEBIAN/control" <<CTRL
Package: ${PKG_NAME}
Version: ${ZENOH_VERSION}
Section: libs
Priority: optional
Architecture: ${ARCH}
Maintainer: Companion Software Stack CI <ci@example.com>
Description: zenoh-c ${ZENOH_VERSION} with shared-memory feature
 Custom build of zenoh-c with ZENOHC_BUILD_WITH_SHARED_MEMORY=ON.
 Provides PosixShmProvider for zero-copy IPC in the drone stack.
CTRL

# Copy installed files into package
cp -a "${STAGING}"/* "${PKG_DIR}/"

# Build the .deb
DEB_FILE="${PKG_NAME}_${ZENOH_VERSION}_${ARCH}.deb"
dpkg-deb --build "${PKG_DIR}" "${WORK_DIR}/${DEB_FILE}"

# Copy to current directory
cp "${WORK_DIR}/${DEB_FILE}" "./${DEB_FILE}"

echo ""
echo "=== SUCCESS ==="
echo "Output: ./${DEB_FILE}"
echo ""
echo "To upload to GitHub Releases:"
echo "  gh release create \"zenohc-v${ZENOH_VERSION}\" ./${DEB_FILE} \\"
echo "    --title \"zenohc ${ZENOH_VERSION} with SHM\" \\"
echo "    --notes \"Custom zenoh-c build with shared-memory feature enabled for CI.\""
echo ""
echo "To verify SHM symbols:"
echo "  dpkg -c ./${DEB_FILE} | grep libzenohc"
echo "  dpkg -I ./${DEB_FILE}"
