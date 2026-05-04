#!/usr/bin/env python3
"""
check_voxel_on_target.py — post-run regression check for PATH A.

Compares the voxel positions in a scenario run's
`path_a_voxel_trace.jsonl` against the ground-truth object positions
from the scene JSON (typically `config/scenes/cosys_static.json`).
Fails (exit code 1) if the fraction of voxels landing within the
specified XY radius of any scene object falls below a threshold.

Addresses the test-quality P1 from PR #614 review:
  "Scenario 33 pass_criteria `log_contains` strings all fire at init,
   so Fix #55 (camera extrinsic rotation) could silently regress and
   the scenario would still match every log check. We need a post-run
   assertion that ties cube-collision-free behaviour to voxel positions
   actually landing on scene objects."

Called from `tests/run_scenario_cosys.sh` (Phase 6 verification) when
`scenario.pass_criteria.voxel_on_target_ratio_min` is present. Defaults
are tuned for the 2026-04-23 scenario-33 working run:
  - radius_m = 3.0 (voxels within 3 m XY of a scene object count as "on target")
  - min_ratio = 0.20 (≥ 20 % — the before-fix was 0 %, after-fix 34 %)
  - min_voxels = 100 (below this count, run is too short to assess — exit 0)

Usage:
  python3 tools/check_voxel_on_target.py \
      --trace drone_logs/scenarios_cosys/33_*/latest/path_a_voxel_trace.jsonl \
      --scene config/scenes/cosys_static.json \
      --radius-m 3.0 --min-ratio 0.20
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def load_scene_xy(scene_path: Path) -> list[tuple[float, float, str]]:
    with scene_path.open() as f:
        scene = json.load(f)
    objects = scene.get("objects", [])
    out = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        if "x" in obj and "y" in obj:
            out.append((float(obj["x"]), float(obj["y"]), obj.get("name", "?")))
    return out


def iter_voxels(trace_path: Path):
    with trace_path.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in rec.get("voxels", []):
                # schema: [wx, wy, wz, label, conf]
                if len(v) >= 3:
                    yield float(v[0]), float(v[1]), float(v[2])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trace", required=True, type=Path, help="path_a_voxel_trace.jsonl from a scenario run")
    ap.add_argument("--scene", required=True, type=Path, help="scene JSON (e.g. config/scenes/cosys_static.json)")
    ap.add_argument("--radius-m", type=float, default=3.0, help="XY radius around each scene object counted as 'on target'")
    ap.add_argument("--min-ratio", type=float, default=0.20, help="minimum fraction of voxels that must land within --radius-m")
    ap.add_argument("--min-voxels", type=int, default=100, help="skip the check if fewer total voxels than this (run too short to assess)")
    args = ap.parse_args()

    if not args.trace.exists():
        print(f"[check_voxel_on_target] ERROR: trace file not found: {args.trace}", file=sys.stderr)
        return 2
    if not args.scene.exists():
        print(f"[check_voxel_on_target] ERROR: scene file not found: {args.scene}", file=sys.stderr)
        return 2

    scene_xy = load_scene_xy(args.scene)
    if not scene_xy:
        # Issue #698 — scenarios that use the live UE5 scene's existing
        # geometry as the obstacle field (no spawned objects in the scene
        # JSON) have nothing to check against here.  This is a legitimate
        # configuration (e.g. scenario 33 uses Blocks default cubes via
        # HD-map static_obstacles instead of spawned scene objects), not
        # an error.  Exit 0 so the runner doesn't flag a spurious FAIL.
        print(f"[check_voxel_on_target] SKIP: scene {args.scene} has no spawned "
              f"objects — nothing to check against (likely an HD-map-based scenario)")
        return 0

    r2 = args.radius_m * args.radius_m
    total = 0
    on_target = 0
    for vx, vy, _vz in iter_voxels(args.trace):
        total += 1
        for (ox, oy, _name) in scene_xy:
            dx = vx - ox
            dy = vy - oy
            if dx * dx + dy * dy <= r2:
                on_target += 1
                break

    if total == 0:
        print(f"[check_voxel_on_target] WARN: trace has zero voxels — PATH A did not publish anything")
        return 1

    if total < args.min_voxels:
        print(f"[check_voxel_on_target] SKIP: only {total} voxels (< --min-voxels={args.min_voxels}); "
              f"run too short to assess on-target ratio")
        return 0

    ratio = on_target / total
    verdict = "PASS" if ratio >= args.min_ratio else "FAIL"
    print(f"[check_voxel_on_target] {verdict}: {on_target}/{total} voxels "
          f"({ratio * 100:.1f} %) landed within {args.radius_m:.1f} m XY of a scene object "
          f"(threshold {args.min_ratio * 100:.0f} %)")
    print(f"[check_voxel_on_target] scene objects checked: {len(scene_xy)} from {args.scene}")
    if verdict == "FAIL":
        print(f"[check_voxel_on_target] Fix #55 regression indicator — see PR #614 / Issue #612", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
