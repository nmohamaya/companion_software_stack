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

# Some launchers (systemd, isolated subshells, processes that effectively
# spawn `python3 -S`) strip standard site paths.  Re-add the common ones so
# a pip-user-installed matplotlib AND its system-installed dependencies
# (e.g. `packaging` in /usr/lib/python3/dist-packages) both resolve.
import site
_pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
_fallback_paths = []
try:
    _fallback_paths.append(site.getusersitepackages())
except Exception:
    pass
_fallback_paths.extend([
    "/usr/lib/python3/dist-packages",
    f"/usr/lib/{_pyver}/dist-packages",
    f"/usr/local/lib/{_pyver}/dist-packages",
])
for _p in _fallback_paths:
    if _p and _p not in sys.path:
        sys.path.append(_p)

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

# ── Ground-truth scene obstacles ─────────────────────────────────────
# Issue #698 — replaces the legacy DEFAULT_CUBES dict (which was mined from
# collision logs and got the geometry wrong; e.g. Cube_4 + Cube_7 are
# actually halves of a single 10×10×10m stacked block, not separate strips).
# Source: tools/diag/cosys_scene_inventory.py dump from a live UE5 instance.
# Re-run the inventory script after any scene change.
DEFAULT_CUBES = {}  # populated below from blocks_default_inventory.json
INVENTORY = (Path(__file__).resolve().parent / "blocks_default_inventory.json")
if INVENTORY.exists():
    inv = json.loads(INVENTORY.read_text())
    for o in inv.get("obstacles", []):
        # Center + scale → axis-aligned bbox.  Unreal "scale" entries are in
        # 1m units when the source mesh is 1×1×1 m (TemplateCube_Rounded is).
        # bbox = (n_min, n_max, e_min, e_max) in NEU world coords.
        cx, cy = o.get("neu_x", 0.0), o.get("neu_y", 0.0)
        sx, sy, _sz = o.get("scale", [1.0, 1.0, 1.0])
        DEFAULT_CUBES[o["name"]] = (cx - sx / 2.0, cx + sx / 2.0,
                                    cy - sy / 2.0, cy + sy / 2.0)
else:
    # Legacy fallback (wrong, but keeps the tool working without inventory).
    DEFAULT_CUBES = {
        "Cube_7":  (16.9, 16.9,  4.5, 13.8),
        "Cube_9":  (16.9, 26.6, 16.7, 24.1),
        "Cube_62": (27.3, 35.2,  4.1,  4.1),
        "Cube_66": (27.4, 32.3, 24.1, 24.1),
    }
    print(f"WARN: {INVENTORY} not found — falling back to hardcoded 4-cube table",
          file=sys.stderr)

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

# Determine the mission-area bounding box first so we can filter the cube
# list to only those overlapping the visible region.
_xs = [r[1] for r in radar] + [v[1] for v in voxels] + [d[1] for d in drone]
_ys = [r[0] for r in radar] + [v[0] for v in voxels] + [d[0] for d in drone]
_xs.extend([w[1] for w in waypoints])
_ys.extend([w[0] for w in waypoints])
if _xs and _ys:
    _view = (min(_xs) - 8, max(_xs) + 8, min(_ys) - 8, max(_ys) + 8)  # e_min, e_max, n_min, n_max
else:
    _view = (-10, 40, -10, 40)

# Default cubes — filled purple rectangles, but only the ones that overlap
# the mission-area view box.  Skips far-field clutter (Blocks scene has
# 165 cubes spanning ±100 m).
def _bbox_overlaps(n_min, n_max, e_min, e_max, view):
    ev_min, ev_max, nv_min, nv_max = view
    return not (e_max < ev_min or e_min > ev_max or n_max < nv_min or n_min > nv_max)

drawn_cubes = 0
for name, (n_min, n_max, e_min, e_max) in DEFAULT_CUBES.items():
    if not _bbox_overlaps(n_min, n_max, e_min, e_max, _view):
        continue
    width  = max(0.5, e_max - e_min)
    height = max(0.5, n_max - n_min)
    rect = mpatches.Rectangle(
        (e_min, n_min), width, height,
        linewidth=2, edgecolor="purple", facecolor="purple", alpha=0.25,
        label=None,
    )
    ax.add_patch(rect)
    # Trim the long Cosys names ("TemplateCube_Rounded_NN" → "TC_NN") so
    # labels don't overlap on dense cube fields.
    short = name.replace("TemplateCube_Rounded_", "TC_")
    ax.text(e_min + width / 2, n_min + height / 2, short,
            ha="center", va="center", fontsize=8, color="purple", weight="bold")
    drawn_cubes += 1

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

# Bound the plot to the MISSION AREA (drone path + waypoints + sensor data),
# then draw only those GT cubes that actually overlap the view.  The Blocks
# scene has 165 cubes spread across ±100 m; without this, the plot zooms
# out to encompass cubes 80 m away from the mission and the drone+cubes in
# the relevant area become unreadable smears.
all_x = [r[1] for r in radar] + [v[1] for v in voxels] + [d[1] for d in drone]
all_y = [r[0] for r in radar] + [v[0] for v in voxels] + [d[0] for d in drone]
all_x.extend([w[1] for w in waypoints])
all_y.extend([w[0] for w in waypoints])
margin = 8
if all_x and all_y:
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
else:
    # Fallback: scenario-33-ish default if no run data.
    ax.set_xlim(-10, 40)
    ax.set_ylim(-10, 40)

out = run / "scene_overlay.png"
fig.tight_layout()
fig.savefig(out, dpi=130, bbox_inches="tight")
print(f"Saved {out}")
print(f"\nQuick stats:")
print(f"  Drone GT samples : {len(drone)}")
print(f"  Radar tracks     : {len(radar)}")
print(f"  PATH A voxels    : {len(voxels)} sampled")
print(f"  Waypoints        : {len(waypoints)}")
print(f"  Default cubes    : {drawn_cubes} drawn ({len(DEFAULT_CUBES)} known total)")
print(f"  Spawned objects  : {len(spawned)}")
