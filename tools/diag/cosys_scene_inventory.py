#!/usr/bin/env python3
# Cosys-AirSim scene inventory + obstacle auto-placement.
#
# Workflow:
#   1. Start UE5 (Blocks scene) — Cosys RPC server on 127.0.0.1:41451
#   2. Run this script
#   3. It connects, lists every scene actor matching common obstacle prefixes
#      (TemplateCube_*, Cube_*, Wall_*, etc.), fetches each pose, and writes
#      tools/diag/blocks_default_inventory.json
#   4. With --place, it then proposes 5 obstacle slots that are at least
#      MIN_GAP_M from every default-scene obstacle AND from each other,
#      anchored on a corridor the drone needs to fly through.
#
# Why this exists:
#   The Blocks default UE5 scene contains dozens of TemplateCubes that we
#   collide with even though they're not in our spawned-obstacle list.
#   Placing our own obstacles blindly puts them on top of these defaults,
#   making path-planning impossible.  We need an auto-placer that respects
#   the existing scene geometry.

import argparse
import json
import math
import sys
from pathlib import Path

# Bootstrap the cosys-airsim Python client from third_party.
THIRD_PARTY = Path(__file__).resolve().parents[2] / "third_party" / "cosys-airsim" / "PythonClient"
if THIRD_PARTY.exists():
    sys.path.insert(0, str(THIRD_PARTY))

try:
    import cosysairsim as airsim
except ImportError as e:
    sys.exit(f"cannot import cosysairsim: {e}\n"
             f"hint: pip3 install --user msgpack-rpc-python  +  ensure UE5 is running")


# Patterns of actor names we care about (cubes / walls / etc.)
INTERESTING_PREFIXES = (
    "TemplateCube",
    "Cube",
    "Wall",
    "Pillar",
    "BP_",            # Unreal Blueprint
    "SM_",            # Static mesh
    "Box",
    "Plane",
    "Sphere",
)


def is_interesting(name: str) -> bool:
    return any(name.startswith(p) for p in INTERESTING_PREFIXES)


def query_inventory(host: str = "127.0.0.1", port: int = 41451):
    print(f"Connecting to Cosys-AirSim at {host}:{port} ...")
    client = airsim.MultirotorClient(ip=host, port=port)
    client.confirmConnection()
    print("Connected.")

    # Cosys exposes simListSceneObjects(name_regex) — empty/wildcard returns all.
    print("Listing all scene objects (this can take a few seconds)...")
    try:
        all_names = client.simListSceneObjects(".*")
    except Exception as e:
        sys.exit(f"simListSceneObjects failed: {e}")
    print(f"  Found {len(all_names)} total actors")

    targets = [n for n in all_names if is_interesting(n)]
    print(f"  {len(targets)} match obstacle prefixes (TemplateCube*/Cube*/Wall*/Pillar*/BP_*/SM_*/Box*/Plane*/Sphere*)")

    # AirSim uses NED world frame (X=N, Y=E, Z=Down).  Convert to NEU
    # (X=N, Y=E, Z=Up) to match the planner/scenario convention.
    inventory = []
    for i, name in enumerate(targets):
        try:
            pose = client.simGetObjectPose(name)
            scale = client.simGetObjectScale(name)
        except Exception as e:
            print(f"  ! skip {name}: {e}")
            continue
        ned_pos = pose.position
        if not (math.isfinite(ned_pos.x_val) and math.isfinite(ned_pos.y_val)
                and math.isfinite(ned_pos.z_val)):
            continue
        # NED → NEU
        neu_x = ned_pos.x_val   # North
        neu_y = ned_pos.y_val   # East
        neu_z = -ned_pos.z_val  # Up
        inventory.append({
            "name": name,
            "neu_x": round(neu_x, 3),
            "neu_y": round(neu_y, 3),
            "neu_z": round(neu_z, 3),
            "scale": [round(scale.x_val, 3), round(scale.y_val, 3), round(scale.z_val, 3)],
        })
        if (i + 1) % 25 == 0:
            print(f"  ... fetched {i + 1}/{len(targets)} poses")

    print(f"Fetched {len(inventory)} valid obstacle poses")
    return inventory


def save_inventory(inv, out_path: Path):
    out_path.write_text(json.dumps({"obstacles": inv}, indent=2))
    print(f"Wrote {out_path}")


def horizontal_distance(a, b):
    return math.hypot(a["neu_x"] - b["neu_x"], a["neu_y"] - b["neu_y"])


def propose_placement(inventory, min_gap_m: float = 8.0,
                      bbox=((-30.0, 30.0), (-30.0, 30.0))):
    """Find 5 obstacle slots at least min_gap_m from every default-scene actor
    AND from each other, inside the bbox, on a corridor the drone can navigate.

    bbox = ((north_min, north_max), (east_min, east_max))
    """
    print(f"\nLooking for 5 spaced slots (≥{min_gap_m} m clearance) in "
          f"N=[{bbox[0][0]}, {bbox[0][1]}] × E=[{bbox[1][0]}, {bbox[1][1]}] ...")

    # Inflate every default obstacle by min_gap_m to define a forbidden zone.
    forbidden = inventory[:]

    # Candidate grid: 1 m steps inside the bbox at the drone's altitude (~5 m).
    # Inventory positions are world-XY; we ignore Z for placement.
    candidates = []
    n_min, n_max = bbox[0]
    e_min, e_max = bbox[1]
    n = n_min
    while n <= n_max:
        e = e_min
        while e <= e_max:
            candidates.append({"neu_x": n, "neu_y": e})
            e += 1.0
        n += 1.0

    # Filter candidates clear of all default obstacles.
    clear = [c for c in candidates
             if all(horizontal_distance(c, d) >= min_gap_m for d in forbidden)]
    print(f"  {len(clear)}/{len(candidates)} candidate cells clear of default scene")
    if not clear:
        return []

    # Greedy spread: pick 5 candidates each ≥ min_gap_m from chosen ones,
    # biased toward forming an obstacle FIELD (not a single cluster) along a
    # ~30 m corridor stretching North.  Anchor: try near the centre.
    target_anchors = [
        ( 8.0, 12.0),  # WP3 vicinity
        (16.0, 20.0),
        (16.0,  6.0),
        (24.0, 14.0),
        ( 4.0,  4.0),
    ]
    chosen = []
    for anchor_n, anchor_e in target_anchors:
        anchor = {"neu_x": anchor_n, "neu_y": anchor_e}
        # Pick the closest clear candidate to this anchor that's also
        # ≥ min_gap_m from already-chosen.
        best = None
        best_dist = 1e9
        for c in clear:
            if any(horizontal_distance(c, x) < min_gap_m for x in chosen):
                continue
            d = horizontal_distance(c, anchor)
            if d < best_dist:
                best = c
                best_dist = d
        if best is not None:
            chosen.append(best)

    print(f"  Selected {len(chosen)} obstacle slots:")
    for i, c in enumerate(chosen):
        nearest_default = min((horizontal_distance(c, d), d["name"]) for d in forbidden)
        print(f"    slot {i + 1}: NEU=({c['neu_x']:.1f}, {c['neu_y']:.1f})  "
              f"nearest default = {nearest_default[1]} at {nearest_default[0]:.1f} m")
    return chosen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41451)
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).resolve().parent / "blocks_default_inventory.json")
    ap.add_argument("--place", action="store_true",
                    help="After querying, propose 5 obstacle slots in a clear region")
    ap.add_argument("--gap", type=float, default=8.0,
                    help="Minimum gap (m) between proposed slots and default obstacles")
    ap.add_argument("--bbox-n", type=float, nargs=2, default=[-30.0, 30.0])
    ap.add_argument("--bbox-e", type=float, nargs=2, default=[-30.0, 30.0])
    args = ap.parse_args()

    inv = query_inventory(args.host, args.port)
    save_inventory(inv, args.out)

    if args.place:
        slots = propose_placement(inv, args.gap, (tuple(args.bbox_n), tuple(args.bbox_e)))
        if slots:
            placement_path = args.out.parent / "proposed_placement.json"
            placement_path.write_text(json.dumps({"slots": slots, "min_gap_m": args.gap}, indent=2))
            print(f"\nWrote {placement_path}")


if __name__ == "__main__":
    main()
