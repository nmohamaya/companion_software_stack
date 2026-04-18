#!/usr/bin/env bash
# third_party/apply_cosys_patches.sh
# Apply local patches to the Cosys-AirSim submodule. Idempotent.
# See issue #495 for the upstream bug these patches fix.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUBMODULE_DIR="${SCRIPT_DIR}/cosys-airsim"
PATCH_FILE="${SCRIPT_DIR}/cosys-airsim-simspawnobject-fix.patch"

if [[ ! -d "${SUBMODULE_DIR}/AirLib" ]]; then
    echo "ERROR: Cosys-AirSim submodule not initialised at ${SUBMODULE_DIR}"
    echo "Run: git submodule update --init third_party/cosys-airsim"
    exit 1
fi

if [[ ! -f "${PATCH_FILE}" ]]; then
    echo "ERROR: patch file missing: ${PATCH_FILE}"
    exit 1
fi

cd "${SUBMODULE_DIR}"

# Idempotent: skip if already applied
if git apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
    echo "[apply_cosys_patches] Patches already applied — skipping"
    exit 0
fi

if git apply --check "${PATCH_FILE}" 2>/dev/null; then
    git apply "${PATCH_FILE}"
    echo "[apply_cosys_patches] Applied simSpawnObject fix (issue #495)"
    echo "[apply_cosys_patches] Rebuild required: cd ${SUBMODULE_DIR} && ./build.sh"
else
    echo "ERROR: patch does not apply cleanly. The submodule may have been updated"
    echo "or the patch is stale. Manual intervention required."
    git apply --reject "${PATCH_FILE}" || true
    exit 2
fi
