#!/usr/bin/env python3
# Sensor-vs-ground-truth analysis for scenario 33 run.
#
# Computes per-sensor (radar, PATH A voxels) localisation error against
# known obstacle positions from the spawn log + collision-inferred cubes.
#
# Outputs:
#   - text report on stdout
#   - PNG: top-down + 3D matplotlib plots (saved next to run dir)

import json
import re
import sys
from pathlib import Path
from collections import defaultdict
from math import sqrt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  — registers 3D projection

if len(sys.argv) < 2:
    print("usage: sensor_vs_gt.py <run_dir>")
    sys.exit(1)

run = Path(sys.argv[1])

# ── 1. Ground truth obstacles (ENU) ──────────────────────────
# Spawned positions logged as (NED_x_north, NED_y_east).  Convert to ENU:
#   ENU_east  = NED_y_east
#   ENU_north = NED_x_north
SPAWNED_NED = {
    "pillar_01": (8.0, 12.0),
    "pillar_02": (22.0, 14.0),
    "wall_01":   (15.0, 10.0),
    "wall_02":   (15.0, 10.0),
    "chair_01":  (4.0, 18.0),
    "couch_01":  (20.0, 12.0),
    "bush_01":   (10.0, 5.0),
}
SPAWNED_ENU = {n: (p[1], p[0]) for n, p in SPAWNED_NED.items()}

# TemplateCubes inferred from collisions.log (in NED → ENU)
template_cubes_enu = {}  # name → list of (e, n) collision points
coll_log = run / "collisions.log"
if coll_log.exists():
    pat = re.compile(r"collision #\d+ with '(TemplateCube_[A-Za-z0-9_]+)'.*at \(([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+)\)")
    for line in coll_log.read_text().splitlines():
        m = pat.search(line)
        if m:
            name, x, y = m.group(1), float(m.group(2)), float(m.group(3))
            template_cubes_enu.setdefault(name, []).append((y, x))  # (e, n)

# ── 2. Radar tracks (UKF dormant pool, world-frame) ──────────
radar_tracks = []  # list of (x_logged, y_logged) — frame TBD
perc_log = run / "perception.log"
if perc_log.exists():
    pat = re.compile(r"(?:New obstacle|Re-ID).*radar-only.*at \(([-\d\.]+),([-\d\.]+)\)")
    for line in perc_log.read_text().splitlines():
        m = pat.search(line)
        if m:
            radar_tracks.append((float(m.group(1)), float(m.group(2))))

# ── 3. PATH A voxels ─────────────────────────────────────────
# Each line: { "voxels": [[x, y, z, instance_id, conf, sem], ...] }
voxel_count = 0
voxel_xyz = []          # (x, y, z) ENU world
voxel_sample_max = 50000
trace = run / "path_a_voxel_trace.jsonl"
if trace.exists():
    with open(trace) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in rec.get("voxels", []):
                if len(v) >= 3:
                    voxel_count += 1
                    if len(voxel_xyz) < voxel_sample_max:
                        voxel_xyz.append((v[0], v[1], v[2]))

# ── 4. Drone trajectory from cosys_telemetry.jsonl gt.pos ────
drone_xyz = []
tele = run / "cosys_telemetry.jsonl"
if tele.exists():
    with open(tele) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            gt = rec.get("gt", {})
            pos = gt.get("pos")
            if pos and len(pos) >= 3:
                drone_xyz.append((pos[0], pos[1], pos[2]))

# ── 5. GT point list for distance calcs ──────────────────────
gt_points = [(name, pos) for name, pos in SPAWNED_ENU.items()]
for name, pts in template_cubes_enu.items():
    # use the centroid of all collision points for that cube name
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        gt_points.append((name, (cx, cy)))

def nearest_2d(p, pts):
    best = (None, 1e9)
    for name, pos in pts:
        d = sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2)
        if d < best[1]:
            best = (name, d)
    return best

def report(label, points2d, gt, near_thresh=2.0):
    if not points2d:
        print(f"  {label}: 0 detections")
        return
    errs = sorted(nearest_2d(p, gt)[1] for p in points2d)
    near = sum(1 for e in errs if e <= near_thresh)
    print(f"  {label}: n={len(points2d):6d}  near-hits (<{near_thresh}m)={near:5d} ({100*near/len(errs):.1f}%)")
    print(f"    median={errs[len(errs)//2]:6.2f} m   p25={errs[len(errs)//4]:6.2f} m   p75={errs[3*len(errs)//4]:6.2f} m   max={errs[-1]:6.2f} m")

# ── 6. Print text report ─────────────────────────────────────
print("=" * 70)
print(f"Run: {run.name}")
print("=" * 70)
print()
print("Ground truth (ENU east, north):")
for name, pos in SPAWNED_ENU.items():
    print(f"  {name:12s}  ({pos[0]:6.2f}, {pos[1]:6.2f})")
print(f"  TemplateCubes inferred from collisions: {len(template_cubes_enu)} unique")
for name, pts in template_cubes_enu.items():
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    print(f"    {name:30s} centroid=({cx:6.2f}, {cy:6.2f})  {len(pts)} hits")
print()

print("Sensor counts:")
print(f"  Radar dormant tracks: {len(radar_tracks)}")
print(f"  PATH A voxels       : {voxel_count} total ({len(voxel_xyz)} sampled)")
print(f"  Drone GT poses      : {len(drone_xyz)}")
print()

print("Radar→GT distance (frame hypotheses):")
report("  Hypothesis A (raw)            ", radar_tracks, gt_points)
report("  Hypothesis B (axis swap)      ", [(p[1], p[0]) for p in radar_tracks], gt_points)
report("  Hypothesis C (negate both)    ", [(-p[0], -p[1]) for p in radar_tracks], gt_points)
print()

print("PATH A voxels→GT distance (assume ENU as written):")
voxel_2d = [(v[0], v[1]) for v in voxel_xyz]
report("  Voxels (xy)                   ", voxel_2d, gt_points, near_thresh=3.0)
print()

# Top 10 nearest radar tracks (Hypothesis A — raw)
print("Top 10 radar tracks closest to GT (Hypothesis A — raw):")
ranked = sorted(((nearest_2d(p, gt_points), p) for p in radar_tracks), key=lambda x: x[0][1])
for (name, dist), p in ranked[:10]:
    print(f"  radar=({p[0]:6.2f}, {p[1]:6.2f})  → nearest GT={name:30s} dist={dist:5.2f} m")
print()

# ── 7. Visualisation ─────────────────────────────────────────
out_png = run / "sensor_vs_gt.png"

# Pick whichever frame hypothesis is best for plotting
def hyp_errs(pts):
    if not pts:
        return 1e9
    return sum(nearest_2d(p, gt_points)[1] for p in pts) / len(pts)

candidates = {
    "A (raw)":        radar_tracks,
    "B (axis swap)":  [(p[1], p[0]) for p in radar_tracks],
    "C (negate)":     [(-p[0], -p[1]) for p in radar_tracks],
}
best_label = min(candidates.keys(), key=lambda k: hyp_errs(candidates[k]))
radar_for_plot = candidates[best_label]
print(f"Best radar frame for plotting: Hypothesis {best_label}")

fig = plt.figure(figsize=(18, 9))

# ── Panel 1: top-down (XY plane) ──
ax1 = fig.add_subplot(1, 2, 1)
# voxels as small grey dots (sample down for plot speed)
if voxel_xyz:
    vp = voxel_xyz[::10]
    ax1.scatter([v[0] for v in vp], [v[1] for v in vp], s=0.5, c="lightgrey",
                alpha=0.3, label=f"PATH A voxels (n={voxel_count}, plotted {len(vp)})")
# radar tracks as red x
if radar_for_plot:
    ax1.scatter([p[0] for p in radar_for_plot], [p[1] for p in radar_for_plot],
                s=40, c="red", marker="x", linewidths=1.5,
                label=f"Radar tracks (n={len(radar_for_plot)})")
# drone trajectory
if drone_xyz:
    ax1.plot([d[0] for d in drone_xyz], [d[1] for d in drone_xyz], "-",
             color="blue", linewidth=1.0, alpha=0.6, label="Drone GT path")
# GT obstacles as big green circles
for name, pos in SPAWNED_ENU.items():
    ax1.plot(pos[0], pos[1], "o", markersize=14, color="green",
             markerfacecolor="none", markeredgewidth=2.0)
    ax1.annotate(name, (pos[0], pos[1]), fontsize=8,
                 xytext=(5, 5), textcoords="offset points", color="green")
# template cubes from collisions
for name, pts in template_cubes_enu.items():
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ax1.plot(cx, cy, "s", markersize=14, color="purple",
                 markerfacecolor="none", markeredgewidth=2.0)
        ax1.annotate(name.replace("TemplateCube_", "TC_"), (cx, cy), fontsize=7,
                     xytext=(5, -10), textcoords="offset points", color="purple")
# waypoints (from scenario)
waypoints_enu = [
    (0, 0, "WP1 home"),
    (10, 20, "WP2"),
    (25, 12, "WP3"),
    (10, 2, "WP4"),
    (0, 0, "WP5 home"),
]
for x, y, label in waypoints_enu:
    ax1.plot(x, y, "*", markersize=20, color="orange", markeredgecolor="black",
             markeredgewidth=0.5)
    ax1.annotate(label, (x, y), fontsize=8, xytext=(8, 0), textcoords="offset points",
                 color="orange", weight="bold")
ax1.set_xlabel("ENU East (m)")
ax1.set_ylabel("ENU North (m)")
ax1.set_title(f"Top-down view (radar frame: Hyp {best_label})")
ax1.legend(loc="lower left", fontsize=8)
ax1.grid(alpha=0.3)
ax1.set_aspect("equal")

# ── Panel 2: 3D ──
ax2 = fig.add_subplot(1, 2, 2, projection="3d")
if voxel_xyz:
    vp = voxel_xyz[::20]
    ax2.scatter([v[0] for v in vp], [v[1] for v in vp], [v[2] for v in vp],
                s=0.4, c="lightgrey", alpha=0.3, label="PATH A voxels (sampled)")
if drone_xyz:
    ax2.plot([d[0] for d in drone_xyz], [d[1] for d in drone_xyz],
             [d[2] for d in drone_xyz], "-", color="blue", linewidth=1.5,
             alpha=0.7, label="Drone GT path")
for name, pos in SPAWNED_ENU.items():
    ax2.scatter([pos[0]], [pos[1]], [2.5], s=120, c="green",
                edgecolors="darkgreen", linewidths=1.5)
    ax2.text(pos[0], pos[1], 3.0, name, fontsize=7, color="darkgreen")
for name, pts in template_cubes_enu.items():
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ax2.scatter([cx], [cy], [2.5], s=140, c="purple", marker="s",
                    edgecolors="indigo", linewidths=1.5)
ax2.set_xlabel("E")
ax2.set_ylabel("N")
ax2.set_zlabel("Up")
ax2.set_title("3D view")
ax2.legend(loc="upper left", fontsize=8)

plt.suptitle(f"Sensor vs GT — {run.name}", fontsize=12)
plt.tight_layout()
plt.savefig(out_png, dpi=120, bbox_inches="tight")
print(f"Saved {out_png}")
