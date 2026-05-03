#!/usr/bin/env python3
# Top-down GT-vs-sensor overlay for scenario 33.
#
# Plots, on a single 2D top-down view (NEU world frame, X=North, Y=East):
#   - All known DEFAULT BLOCKS scene cubes (extracted from collision logs)
#     drawn as rectangles with their actual extent
#   - Spawned obstacles from scenario config (if any) as green circles
#   - Drone trajectory (Cosys ground-truth pose) as a blue line
#   - Radar tracks (UKF dormant pool world coords) as red X
#   - PATH A voxel sample as light grey dots
#   - Waypoints from scenario config as orange stars
#
# Use after every scenario run to see at a glance where the sensors agree
# / disagree with ground truth.

import json
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    sys.exit("usage: scene_overlay.py <run_dir> [scenario_config.json]")

run = Path(sys.argv[1])
scenario_cfg = Path(sys.argv[2]) if len(sys.argv) > 2 else (
    Path(__file__).resolve().parents[2]
    / "config" / "scenarios" / "33_non_coco_obstacles.json")

# ── Default Blocks scene cubes (mined from past collision logs) ──────
# NEU coords (X=North, Y=East). bbox = (n_min, n_max, e_min, e_max).
DEFAULT_CUBES = {
    "Cube_7":  (16.9, 16.9,  4.5, 13.8),   # 10m wall along East
    "Cube_9":  (16.9, 26.6, 16.7, 24.1),   # 10x7 m block
    "Cube_62": (27.3, 35.2,  4.1,  4.1),   # 8m wall along North
    "Cube_66": (27.4, 32.3, 24.1, 24.1),   # 5m wall along North
}

# ── Spawned obstacles + waypoints from scenario config ───────────────
spawned = []
waypoints = []
if scenario_cfg.exists():
    cfg = json.loads(scenario_cfg.read_text())
    overrides = cfg.get("config_overrides", {})
    mp = overrides.get("mission_planner", {})
    for w in mp.get("waypoints", []):
        waypoints.append((w.get("x", 0.0), w.get("y", 0.0), w.get("z", 5.0)))
    # Spawned scene objects (NEU coords per scene_populate convention).
    scene_file = cfg.get("scenario", {}).get("scene", {}).get("file")
    if scene_file:
        sp = (Path(__file__).resolve().parents[2] / scene_file)
        if sp.exists():
            scene = json.loads(sp.read_text())
            for o in scene.get("objects", []):
                spawned.append({
                    "name": o.get("name", "?"),
                    "x": o.get("x", 0.0),
                    "y": o.get("y", 0.0),
                })

# ── Radar tracks ────────────────────────────────────────────────────
radar = []
plog = run / "perception.log"
if plog.exists():
    pat = re.compile(r"radar-only.*at \(([-\d\.]+),([-\d\.]+)\)")
    for line in plog.read_text().splitlines():
        m = pat.search(line)
        if m:
            radar.append((float(m.group(1)), float(m.group(2))))

# ── PATH A voxels (sampled) ─────────────────────────────────────────
voxels = []
trace = run / "path_a_voxel_trace.jsonl"
if trace.exists():
    cap = 100000
    with open(trace) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in rec.get("voxels", []):
                if len(v) >= 2 and len(voxels) < cap:
                    voxels.append((v[0], v[1]))

# ── Drone GT trajectory ─────────────────────────────────────────────
drone = []
tele = run / "cosys_telemetry.jsonl"
if tele.exists():
    with open(tele) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            pos = rec.get("gt", {}).get("pos")
            if pos and len(pos) >= 2:
                drone.append((pos[0], pos[1]))

# ── Plot ────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 12))

# Default cubes as filled purple rectangles with outlines.
for name, (n_min, n_max, e_min, e_max) in DEFAULT_CUBES.items():
    # x = East, y = North in plot — NEU view.  Using matplotlib's standard
    # coordinate frame: plot.x = East, plot.y = North.
    width = max(0.5, e_max - e_min)
    height = max(0.5, n_max - n_min)
    # Inflate by 1 m for collision-zone visualisation.
    rect = mpatches.Rectangle(
        (e_min, n_min), width, height,
        linewidth=2, edgecolor="purple", facecolor="purple", alpha=0.25,
        label=None,
    )
    ax.add_patch(rect)
    ax.text(e_min + width / 2, n_min + height / 2, name,
            ha="center", va="center", fontsize=9, color="purple", weight="bold")

# Voxels as light grey dots (sampled down for speed).
if voxels:
    vp = voxels[::10]
    ax.scatter([v[1] for v in vp], [v[0] for v in vp],
               s=1, c="lightgrey", alpha=0.4,
               label=f"PATH A voxels (n={len(voxels)} sampled, {len(vp)} plotted)")

# Radar tracks.
if radar:
    ax.scatter([r[1] for r in radar], [r[0] for r in radar],
               s=60, c="red", marker="x", linewidths=1.8,
               label=f"Radar tracks (n={len(radar)})")

# Spawned obstacles (green circles).
for o in spawned:
    ax.plot(o["y"], o["x"], "o", markersize=14, color="green",
            markerfacecolor="none", markeredgewidth=2.0)
    ax.annotate(o["name"], (o["y"], o["x"]), fontsize=8, color="green",
                xytext=(6, 6), textcoords="offset points")

# Drone trajectory.
if drone:
    ax.plot([d[1] for d in drone], [d[0] for d in drone], "-",
            color="blue", linewidth=1.4, alpha=0.7,
            label=f"Drone GT path (n={len(drone)})")
    if drone:
        ax.plot(drone[0][1], drone[0][0], "o", color="blue",
                markersize=10, markeredgecolor="navy", label="Start")
        ax.plot(drone[-1][1], drone[-1][0], "s", color="blue",
                markersize=10, markeredgecolor="navy", label="End")

# Waypoints.
for i, (n, e, z) in enumerate(waypoints):
    ax.plot(e, n, "*", markersize=22, color="orange",
            markeredgecolor="black", markeredgewidth=0.6)
    ax.annotate(f"WP{i + 1}", (e, n), fontsize=10, color="darkorange",
                weight="bold", xytext=(8, 0), textcoords="offset points")

# Frame setup.
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.set_title(f"Scene overlay — {run.name}\n"
             f"GT cubes (purple) | spawned (green ○) | radar (red ×) | voxels (grey ·) "
             f"| drone path (blue) | waypoints (orange ★)")
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.legend(loc="upper left", fontsize=9)

# Reasonable axis ranges covering the typical scenario 33 area.
all_x = [r[1] for r in radar] + [v[1] for v in voxels] + [d[1] for d in drone]
all_y = [r[0] for r in radar] + [v[0] for v in voxels] + [d[0] for d in drone]
all_x.extend([w[1] for w in waypoints])
all_y.extend([w[0] for w in waypoints])
for n_min, n_max, e_min, e_max in DEFAULT_CUBES.values():
    all_x.extend([e_min, e_max])
    all_y.extend([n_min, n_max])
if all_x and all_y:
    margin = 5
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

out = run / "scene_overlay.png"
fig.tight_layout()
fig.savefig(out, dpi=130, bbox_inches="tight")
print(f"Saved {out}")
print(f"\nQuick stats:")
print(f"  Drone GT samples : {len(drone)}")
print(f"  Radar tracks     : {len(radar)}")
print(f"  PATH A voxels    : {len(voxels)} sampled")
print(f"  Waypoints        : {len(waypoints)}")
print(f"  Default cubes    : {len(DEFAULT_CUBES)} (purple rectangles)")
print(f"  Spawned objects  : {len(spawned)}")
