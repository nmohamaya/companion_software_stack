#!/usr/bin/env python3
# Per-object recall analyzer for scenario 33 sensor performance.
#
# For each spawned obstacle (ground truth), answer:
#   1. Did ANY radar track land within X m of it during the run?
#   2. Did ANY PATH A voxel land within X m of it?
#   3. What's the closest sensor hit (and time of that hit)?
#
# This is the diagnostic the user wants — recall per object, not aggregate
# error.  Aggregate error is misleading because the GT list excludes Blocks
# default-scene clutter.  Recall per spawned object is the true measure of
# "does our perception see what we placed".
#
# Output: text table + per-object PNG plots showing the obstacle as a
# central marker with all sensor hits within a configurable radius.

import json
import re
import sys
from pathlib import Path
from math import sqrt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    sys.exit("usage: per_object_recall.py <run_dir> [radius_m]")
run = Path(sys.argv[1])
radius_m = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

# ── Spawned obstacles (ground truth) — NEU world frame (X=North, Y=East) ──
# Per scene_populate.log: pillar_01 at (8, 12, 0)... where (x, y, z) is
# AirSim NED world meters.  Internal stack frame is NEU (X=North=NED.x,
# Y=East=NED.y, Z=Up=-NED.z).  So spawned obstacles in stack frame are:
SPAWNED = {
    "pillar_01": (8.0,  12.0),   # (north, east)
    "pillar_02": (22.0, 14.0),
    "wall_01":   (15.0, 10.0),
    "wall_02":   (15.0, 10.0),
    "chair_01":  (4.0,  18.0),
    "couch_01":  (20.0, 12.0),
    "bush_01":   (10.0, 5.0),
}

# TemplateCubes from collision log are in NED → reuse same (north, east) form
template_cubes = []
coll = run / "collisions.log"
if coll.exists():
    pat = re.compile(r"collision #\d+ with '(TemplateCube_[A-Za-z0-9_]+)'.*at \(([\d\.\-]+), ([\d\.\-]+),")
    seen = {}
    for line in coll.read_text().splitlines():
        m = pat.search(line)
        if m:
            name = m.group(1)
            x, y = float(m.group(2)), float(m.group(3))
            seen.setdefault(name, []).append((x, y))
    for name, pts in seen.items():
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        template_cubes.append((name, cx, cy))

# All GT obstacles to score (label, north, east)
all_gt = [(name, pos[0], pos[1]) for name, pos in SPAWNED.items()]
for name, x, y in template_cubes:
    all_gt.append((name, x, y))

# ── Radar tracks (UKF dormant pool) ──────────────────────────
radar_tracks = []
perc = run / "perception.log"
if perc.exists():
    pat = re.compile(r"\[(\d{2}:\d{2}:\d{2}\.\d{3})\].*radar-only.*at \(([-\d\.]+),([-\d\.]+)\)")
    pat2 = re.compile(r"\[\d{4}-\d{2}-\d{2} (\d{2}:\d{2}:\d{2}\.\d{3})\].*radar-only.*at \(([-\d\.]+),([-\d\.]+)\)")
    for line in perc.read_text().splitlines():
        m = pat2.search(line) or pat.search(line)
        if m:
            radar_tracks.append((m.group(1), float(m.group(2)), float(m.group(3))))

# ── PATH A voxels ────────────────────────────────────────────
voxel_xy_all = []  # (x_north, y_east) — sample
trace = run / "path_a_voxel_trace.jsonl"
if trace.exists():
    cap = 200000  # bigger sample for recall measurement
    with open(trace) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in rec.get("voxels", []):
                if len(v) >= 3 and len(voxel_xy_all) < cap:
                    voxel_xy_all.append((v[0], v[1]))

# ── Per-object recall ────────────────────────────────────────
def hits_within(xy_pts, target_x, target_y, r):
    """Return list of (xy, dist) for sensor points within r metres."""
    out = []
    for p in xy_pts:
        if isinstance(p, tuple) and len(p) == 3:
            _, x, y = p
        else:
            x, y = p[0], p[1]
        d = sqrt((x - target_x) ** 2 + (y - target_y) ** 2)
        if d <= r:
            out.append(((x, y), d))
    return sorted(out, key=lambda t: t[1])

print("=" * 78)
print(f"Per-object recall analysis  —  run {run.name}")
print(f"Radius for 'detected' classification: {radius_m:.1f} m")
print("=" * 78)
print()
print(f"{'Obstacle':16s} {'GT (N, E)':>14s} {'Radar hits':>10s} {'closest':>8s} {'Voxel hits':>11s} {'closest':>8s}")
print("-" * 78)

summary = []
for name, gt_x, gt_y in all_gt:
    radar_hits = hits_within([(rt[1], rt[2]) for rt in radar_tracks], gt_x, gt_y, radius_m)
    voxel_hits = hits_within(voxel_xy_all, gt_x, gt_y, radius_m)
    r_closest = radar_hits[0][1] if radar_hits else 9999.0
    v_closest = voxel_hits[0][1] if voxel_hits else 9999.0
    summary.append((name, gt_x, gt_y, len(radar_hits), r_closest, len(voxel_hits), v_closest))
    r_str = f"{r_closest:.2f}m" if radar_hits else "—"
    v_str = f"{v_closest:.2f}m" if voxel_hits else "—"
    print(f"{name:16s} ({gt_x:5.1f},{gt_y:5.1f})   {len(radar_hits):>9d} {r_str:>8s}   {len(voxel_hits):>10d} {v_str:>8s}")

print()
print("=== Recall summary ===")
spawned_names = set(SPAWNED.keys())
radar_recall_spawned = sum(1 for s in summary if s[0] in spawned_names and s[3] > 0)
voxel_recall_spawned = sum(1 for s in summary if s[0] in spawned_names and s[5] > 0)
n_spawned = len(spawned_names)
print(f"Spawned obstacles ({n_spawned}):")
print(f"  Radar saw  : {radar_recall_spawned}/{n_spawned}  ({100*radar_recall_spawned/n_spawned:.0f}%)")
print(f"  Voxels saw : {voxel_recall_spawned}/{n_spawned}  ({100*voxel_recall_spawned/n_spawned:.0f}%)")

# ── Per-object PNG: the obstacle as origin, sensor hits scattered around ──
out_dir = run / "per_object_plots"
out_dir.mkdir(exist_ok=True)

for name, gt_x, gt_y in all_gt:
    fig, ax = plt.subplots(figsize=(7, 7))
    # Plot a 2*radius_m window around the obstacle
    win = radius_m + 1.0
    ax.set_xlim(gt_x - win, gt_x + win)
    ax.set_ylim(gt_y - win, gt_y + win)
    ax.set_aspect("equal")
    # GT obstacle marker
    ax.plot(gt_x, gt_y, "*", markersize=24, color="green",
            markeredgecolor="darkgreen", markeredgewidth=1.5, label=f"{name} (GT)")
    # radius circle
    th = [i * 0.1 for i in range(63)]
    import math
    ax.plot([gt_x + radius_m * math.cos(t) for t in th],
            [gt_y + radius_m * math.sin(t) for t in th],
            "--", color="green", alpha=0.5, linewidth=1)
    # voxel hits
    voxel_hits = hits_within(voxel_xy_all, gt_x, gt_y, radius_m)
    if voxel_hits:
        ax.scatter([h[0][0] for h in voxel_hits], [h[0][1] for h in voxel_hits],
                   s=4, c="lightgrey", alpha=0.5,
                   label=f"PATH A voxels ({len(voxel_hits)})")
    # radar hits
    radar_hits = hits_within([(rt[1], rt[2]) for rt in radar_tracks], gt_x, gt_y, radius_m)
    if radar_hits:
        ax.scatter([h[0][0] for h in radar_hits], [h[0][1] for h in radar_hits],
                   s=60, c="red", marker="x", linewidths=1.5,
                   label=f"Radar tracks ({len(radar_hits)})")
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_title(f"{name}  GT=({gt_x:.1f}, {gt_y:.1f})  recall radius = {radius_m:.1f} m")
    ax.legend(loc="lower left", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"{name}.png", dpi=110)
    plt.close(fig)

print(f"\nSaved per-object plots to {out_dir}/")
print(f"Run dir: {run}")
