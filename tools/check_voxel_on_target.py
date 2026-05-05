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


def load_scenario_static_obstacles(scenario_path: Path) -> list[tuple[float, float, str]]:
    """Load scenario.json -> mission_planner.static_obstacles[] as XY targets.

    HD-map scenarios (no spawned scene objects, all obstacles defined as
    cylinders in static_obstacles) need an alternative source for the
    voxel-on-target regression check.  Each static_obstacle entry has
    {x, y, radius_m, ...} — we use (x, y) as the centre.  The cylinder
    radius is added to --radius-m at call time so a tight radius still
    catches voxels landing on the obstacle surface.
    """
    try:
        with scenario_path.open() as f:
            sc = json.load(f)
    except (OSError, json.JSONDecodeError):
        return []
    out = []
    obs = sc.get("mission_planner", {}).get("static_obstacles", [])
    for o in obs:
        if not isinstance(o, dict):
            continue
        if "x" in o and "y" in o:
            label = o.get("name") or o.get("label") or "static_obstacle"
            out.append((float(o["x"]), float(o["y"]), str(label),
                        float(o.get("radius_m", 0.0))))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trace", required=True, type=Path, help="path_a_voxel_trace.jsonl from a scenario run")
    ap.add_argument("--scene", required=True, type=Path, help="scene JSON (e.g. config/scenes/cosys_static.json)")
    ap.add_argument("--scenario", type=Path, default=None,
                    help="optional scenario JSON; if --scene has no spawned objects, "
                         "fall back to scenario.mission_planner.static_obstacles[] so "
                         "HD-map scenarios still get a voxel-on-target regression check "
                         "(PR #704 test-scenario review).")
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
    # `targets` is a normalised list of (x, y, label, extra_radius_m).
    # For spawned scene objects extra_radius_m=0; for HD-map static
    # obstacles we use the cylinder radius so a 3-m default still catches
    # surface returns.
    targets: list[tuple[float, float, str, float]] = [(x, y, n, 0.0) for (x, y, n) in scene_xy]
    if not targets and args.scenario is not None and args.scenario.exists():
        targets = load_scenario_static_obstacles(args.scenario)
        if targets:
            print(f"[check_voxel_on_target] HD-map fallback: using {len(targets)} static_obstacles "
                  f"from {args.scenario}")
    if not targets:
        # Truly nothing to check — both scene and scenario fallback empty.
        # Exit 0 so the runner doesn't flag a spurious FAIL on scenarios
        # that legitimately don't spawn anything (e.g. fault-injection
        # tests that just need the planner to ARM and hover).
        print(f"[check_voxel_on_target] SKIP: scene {args.scene} has no spawned objects "
              f"and no scenario static_obstacles fallback available — nothing to check against")
        return 0

    base_r2 = args.radius_m * args.radius_m
    total = 0
    on_target = 0
    for vx, vy, _vz in iter_voxels(args.trace):
        total += 1
        for (ox, oy, _name, extra_r) in targets:
            r = args.radius_m + extra_r  # cylinder surface gets the cylinder radius added
            r2 = r * r if extra_r > 0.0 else base_r2
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
          f"({ratio * 100:.1f} %) landed within {args.radius_m:.1f} m XY of a target "
          f"(threshold {args.min_ratio * 100:.0f} %)")
    print(f"[check_voxel_on_target] targets checked: {len(targets)} ("
          f"{len(scene_xy)} scene objects, {len(targets) - len(scene_xy)} HD-map fallback)")
    if verdict == "FAIL":
        print(f"[check_voxel_on_target] Fix #55 regression indicator — see PR #614 / Issue #612", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
