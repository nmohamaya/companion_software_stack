#!/usr/bin/env bash
# deploy/setup_blocks_assets.sh
# ═══════════════════════════════════════════════════════════════
# Import supplementary UE5 assets into the Blocks environment so
# scenario 30 (cosys_static) and follow-ups in epic #480 can spawn
# people, furniture and buildings via simSpawnObject RPC.
#
# Sources:
#   1. Cosys-AirSim DynamicObjects submodule — SK_Mannequin + AI
#      assets (YOLO "person"). Required for scenarios 30 and 31.
#   2. UE5 StarterContent — Architecture (walls/pillars) + Props
#      (chairs/couches/bushes). These are COCO classes ("chair",
#      "couch", "potted plant") that YOLOv8 will actually detect.
#
# Destination: Blocks/Content/Imported/  (new subtree, isolates imports
# from the pristine upstream Blocks content).
#
# Idempotent: rsync -a only copies changed files.
# No UE5 C++/source changes — no rebuild required. Relaunch Blocks
# once after running; UE5's asset registry picks up the new tree.
#
# Usage:
#   bash deploy/setup_blocks_assets.sh [--dry-run]
#
# Environment variables:
#   COSYS_DIR  Path to Cosys-AirSim (default: third_party/cosys-airsim)
#   UE5_DIR    Path to UnrealEngine   (default: /opt/UnrealEngine)
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

COSYS_DIR="${COSYS_DIR:-${PROJECT_DIR}/third_party/cosys-airsim}"
UE5_DIR="${UE5_DIR:-/opt/UnrealEngine}"
DRY_RUN=false

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "Unknown arg: $arg" >&2; exit 2 ;;
    esac
done

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

DYNAMIC_SRC="${COSYS_DIR}/Unreal/Environments/DynamicObjects/Content"
STARTER_SRC="${UE5_DIR}/Samples/StarterContent/Content/StarterContent"
DEST_ROOT="${COSYS_DIR}/Unreal/Environments/Blocks/Content/Imported"

echo "═══════════════════════════════════════════════════════════════"
echo "  Blocks asset import (Epic #480 Phase A, issue #479)"
echo "═══════════════════════════════════════════════════════════════"
echo "  DynamicObjects : ${DYNAMIC_SRC}"
echo "  StarterContent : ${STARTER_SRC}"
echo "  Destination    : ${DEST_ROOT}"
echo "  Dry run        : ${DRY_RUN}"
echo ""

# ── Sanity checks ───────────────────────────────────────────
missing=0
if [[ ! -d "$DYNAMIC_SRC/GroupedAI" ]]; then
    echo -e "  ${RED}ERROR: DynamicObjects/GroupedAI not found at ${DYNAMIC_SRC}${NC}"
    echo -e "  ${RED}       Check COSYS_DIR and git submodule status.${NC}"
    missing=$((missing + 1))
fi
if [[ ! -d "$STARTER_SRC/Architecture" ]] || [[ ! -d "$STARTER_SRC/Props" ]]; then
    echo -e "  ${RED}ERROR: UE5 StarterContent not found at ${STARTER_SRC}${NC}"
    echo -e "  ${RED}       Install StarterContent via UE5 setup, or set UE5_DIR.${NC}"
    missing=$((missing + 1))
fi
if [[ $missing -gt 0 ]]; then
    exit 2
fi

# ── Copy ────────────────────────────────────────────────────
RSYNC_FLAGS=("-a" "--update")
if [[ "$DRY_RUN" == "true" ]]; then
    RSYNC_FLAGS+=("--dry-run" "-v")
fi

copy_tree() {
    local src="$1" dest="$2" label="$3"
    if [[ ! -d "$src" ]]; then
        echo -e "  ${YELLOW}SKIP${NC} ${label}  (source missing: ${src})"
        return 0
    fi
    mkdir -p "$(dirname "$dest")"
    echo -n "  → ${label} ... "
    rsync "${RSYNC_FLAGS[@]}" "$src/" "$dest/" >/dev/null
    echo -e "${GREEN}ok${NC}"
}

echo "Copying DynamicObjects assets:"
copy_tree "${DYNAMIC_SRC}/GroupedAI"          "${DEST_ROOT}/GroupedAI"          "GroupedAI (SK_Mannequin, human_ai, animations)"

echo ""
echo "Copying UE5 StarterContent assets:"
copy_tree "${STARTER_SRC}/Architecture"       "${DEST_ROOT}/Architecture"       "Architecture (walls, pillars, doors, floors)"
copy_tree "${STARTER_SRC}/Props"              "${DEST_ROOT}/Props"              "Props (chairs, couches, bushes, lamps…)"
copy_tree "${STARTER_SRC}/Shapes"             "${DEST_ROOT}/Shapes"             "Shapes (basic meshes)"
copy_tree "${STARTER_SRC}/Materials"          "${DEST_ROOT}/Materials"          "Materials referenced by Architecture + Props"
copy_tree "${STARTER_SRC}/Textures"           "${DEST_ROOT}/Textures"           "Textures referenced by Materials"

echo ""
if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "${YELLOW}Dry run complete — no files written.${NC}"
else
    echo -e "${GREEN}Asset import complete.${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Launch Blocks once from UE5 so the asset registry indexes the new tree."
    echo "     (Our scenario runner relaunches UE5 each run — this happens automatically on first scenario execution.)"
    echo "  2. Run:"
    echo "       bash tests/run_scenario_cosys.sh config/scenarios/30_cosys_static.json \\"
    echo "            --base-config config/cosys_airsim_dev.json --gui --verbose"
fi
