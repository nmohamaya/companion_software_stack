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
else
    echo "ERROR: patch does not apply cleanly. The submodule may have been updated"
    echo "or the patch is stale. Manual intervention required."
    git apply --reject "${PATCH_FILE}" || true
    exit 2
fi

# build.sh syncs Unreal/Plugins/AirSim into the source tree, but the Blocks
# environment has its own per-project Plugin copy. Sync the patched files
# there too so UE5 picks them up when the project is opened/built.
BLOCKS_PLUGIN_SRC="${SUBMODULE_DIR}/Unreal/Environments/Blocks/Plugins/AirSim/Source"
if [[ -d "${BLOCKS_PLUGIN_SRC}" ]]; then
    cp "${SUBMODULE_DIR}/Unreal/Plugins/AirSim/Source/AirBlueprintLib.cpp" \
       "${BLOCKS_PLUGIN_SRC}/AirBlueprintLib.cpp"
    cp "${SUBMODULE_DIR}/Unreal/Plugins/AirSim/Source/WorldSimApi.cpp" \
       "${BLOCKS_PLUGIN_SRC}/WorldSimApi.cpp"
    # Force UE5 to rebuild the plugin .so on next project open
    rm -rf "${SUBMODULE_DIR}/Unreal/Environments/Blocks/Plugins/AirSim/Binaries"
    rm -rf "${SUBMODULE_DIR}/Unreal/Environments/Blocks/Plugins/AirSim/Intermediate"
    rm -rf "${SUBMODULE_DIR}/Unreal/Environments/Blocks/Binaries"
    rm -rf "${SUBMODULE_DIR}/Unreal/Environments/Blocks/Intermediate"
    echo "[apply_cosys_patches] Synced patched files to Blocks plugin + cleaned stale binaries"
fi

echo ""
echo "[apply_cosys_patches] Next steps:"
echo "  1. Rebuild AirLib:  cd ${SUBMODULE_DIR} && CMAKE_POLICY_VERSION_MINIMUM=3.5 ./build.sh"
echo "  2. Open Blocks.uproject in UE5 — choose 'Yes' to build missing modules"
echo "  3. Press Play to start the AirSim server"
