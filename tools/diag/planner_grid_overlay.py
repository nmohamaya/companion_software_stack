#!/usr/bin/env python3
# Planner-grid overlay — approximates the OccupancyGrid3D static layer that
# the planner actually consumes, by replaying voxel/radar promotion logic
# over the existing run logs.
#
# This is an APPROXIMATION (no instance-tracker state, no exact TTL replay,
# no cross-modal veto) but accurate enough to show "what cells did the
# planner think were obstacles?" — which is what matters for diagnosing
# why the drone won't move.
#
# Reads (from <run_dir>):
#   - path_a_voxel_trace.jsonl   — every voxel position emitted to the grid
#   - perception.log             — radar-only world-frame tracks
#   - cosys_telemetry.jsonl      — drone GT trajectory
# Reads (from scenario config — defaults to 33):
#   - mission_planner.occupancy_grid.resolution_m
#   - mission_planner.occupancy_grid.promotion_hits
#   - mission_planner.occupancy_grid.voxel_input.min_confidence
#   - mission_planner.occupancy_grid.inflation_radius_m
#
# Plots cells that received >= promotion_hits hits at >= min_confidence as
# the approximated static grid, plus the inflated halos that the planner
# would treat as blocked.

import json
import math
import re
import sys
from collections import defaultdict
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
    sys.exit("usage: planner_grid_overlay.py <run_dir> [scenario_config.json]")

run = Path(sys.argv[1])
scenario_cfg_path = Path(sys.argv[2]) if len(sys.argv) > 2 else (
    Path(__file__).resolve().parents[2]
    / "config" / "scenarios" / "33_non_coco_obstacles.json")

# Issue #698 — load cubes from the live-scene inventory dump rather than
# the legacy hardcoded table; see scene_overlay.py for rationale.
DEFAULT_CUBES = {}
INVENTORY = (Path(__file__).resolve().parent / "blocks_default_inventory.json")
if INVENTORY.exists():
    inv = json.loads(INVENTORY.read_text())
    for o in inv.get("obstacles", []):
        cx, cy = o.get("neu_x", 0.0), o.get("neu_y", 0.0)
        sx, sy, _sz = o.get("scale", [1.0, 1.0, 1.0])
        DEFAULT_CUBES[o["name"]] = (cx - sx / 2.0, cx + sx / 2.0,
                                    cy - sy / 2.0, cy + sy / 2.0)
else:
    DEFAULT_CUBES = {
        "Cube_7":  (16.9, 16.9,  4.5, 13.8),
        "Cube_9":  (16.9, 26.6, 16.7, 24.1),
        "Cube_62": (27.3, 35.2,  4.1,  4.1),
        "Cube_66": (27.4, 32.3, 24.1, 24.1),
    }
    print(f"WARN: {INVENTORY} not found — falling back to hardcoded 4-cube table",
          file=sys.stderr)

# ── Scenario config — pull grid params ──────────────────────────────
resolution_m = 2.0
promotion_hits = 10
min_confidence = 0.7
inflation_m = 1.5
waypoints = []
if scenario_cfg_path.exists():
    cfg = json.loads(scenario_cfg_path.read_text())
    mp = cfg.get("config_overrides", {}).get("mission_planner", {})
    og = mp.get("occupancy_grid", {})
    resolution_m = float(og.get("resolution_m", resolution_m))
    # Voxel pipeline uses `voxel_input.promotion_hits` (default 3 for PATH A);
    # the top-level `promotion_hits` is the *detected-objects* gate (default 10).
    # Earlier this overlay incorrectly read the detected-objects threshold,
    # so the orange "promoted" cells in the PNG didn't match what
    # insert_voxels() actually cements.  Read the voxel-input key first
    # (matches the path the trace records), fall back to top-level
    # `promotion_hits` only if voxel_input is absent (Copilot review on
    # PR #704).
    voxel_input_cfg = og.get("voxel_input", {})
    promotion_hits = int(voxel_input_cfg.get("promotion_hits",
                                             og.get("promotion_hits", promotion_hits)))
    inflation_m = float(og.get("inflation_radius_m", inflation_m))
    vi = og.get("voxel_input", {})
    min_confidence = float(vi.get("min_confidence", min_confidence))
    for w in mp.get("waypoints", []):
        waypoints.append((w.get("x", 0.0), w.get("y", 0.0), w.get("z", 5.0)))

# ── Voxel hits per cell ─────────────────────────────────────────────
# Voxel format from path_a_voxel_trace.jsonl: [x, y, z, instance_id, confidence]
# Where (x, y, z) is NEU world coords (X=N, Y=E, Z=Up).
# Group by 2D (N, E) cell — collapse Z so a vertical pillar shows as one
# blocking cell on the top-down view. The planner uses 3D cells but for
# top-down "where can the drone go" the projection is what matters.
hits_2d = defaultdict(int)             # (cn, ce) -> hit count (any conf, any z)
hits_2d_conf = defaultdict(int)        # (cn, ce) -> hits passing min_conf
trace = run / "path_a_voxel_trace.jsonl"
total_voxels = 0
# Existence guard: the runner now invokes this overlay for every Cosys
# scenario, but not every scenario emits a PATH A trace (HD-map-only
# tests, fault injection, etc.).  Without the guard `open()` raises
# FileNotFoundError and the overlay aborts before drawing the cubes
# and drone path that ARE available (Copilot review on PR #704).
if trace.exists():
    with open(trace) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in rec.get("voxels", []):
                if len(v) < 5:
                    continue
                n, e, _z, _inst, conf = v[0], v[1], v[2], v[3], v[4]
                cn = int(round(n / resolution_m))
                ce = int(round(e / resolution_m))
                hits_2d[(cn, ce)] += 1
                if conf >= min_confidence:
                    hits_2d_conf[(cn, ce)] += 1
                total_voxels += 1
else:
    print(f"[planner_grid_overlay] {trace.name} not found — generating partial overlay "
          f"(drone path + waypoints + scene cubes only)", file=sys.stderr)

# Cells that would have been promoted: confident hits >= promotion_hits.
promoted = {c for c, h in hits_2d_conf.items() if h >= promotion_hits}

# ── Radar tracks (for radar-side grid promotion approximation) ──────
# Radar promotes after radar_promotion_hits=3 default; we don't have per-cell
# hit counts here, so we just plot radar-track cells once any track lands.
radar_pts = []
plog = run / "perception.log"
if plog.exists():
    pat = re.compile(r"radar-only.*at \(([-\d\.]+),([-\d\.]+)\)")
    for line in plog.read_text().splitlines():
        m = pat.search(line)
        if m:
            radar_pts.append((float(m.group(1)), float(m.group(2))))
radar_cells = {(int(round(n / resolution_m)), int(round(e / resolution_m)))
               for n, e in radar_pts}

# ── Drone GT path ───────────────────────────────────────────────────
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
            if pos:
                drone.append((pos[0], pos[1]))

# ── Plot ────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(11, 11))

# Pre-compute mission-area bbox so we can clip the inventory cubes to the
# visible region (Blocks scene has 165 cubes spanning ±100 m; without this
# the plot zooms out far past the mission and mission-area cells become
# unreadable).
_xs = [d[1] for d in drone] + [w[1] for w in waypoints]
_ys = [d[0] for d in drone] + [w[0] for w in waypoints]
# Bring in the promoted cells too so the view encloses any phantom
# obstacle field that emerged from voxel/radar.  (`inflated` is computed
# below — `promoted` is its core subset and sufficient for the bbox.)
for (cn, ce) in promoted:
    _xs.append(ce * resolution_m)
    _ys.append(cn * resolution_m)
if _xs and _ys:
    _view = (min(_xs) - 8, max(_xs) + 8, min(_ys) - 8, max(_ys) + 8)
else:
    _view = (-10, 40, -10, 40)

def _bbox_overlaps(n_min, n_max, e_min, e_max, view):
    ev_min, ev_max, nv_min, nv_max = view
    return not (e_max < ev_min or e_min > ev_max or n_max < nv_min or n_min > nv_max)

drawn_cubes = 0
for name, (n_min, n_max, e_min, e_max) in DEFAULT_CUBES.items():
    if not _bbox_overlaps(n_min, n_max, e_min, e_max, _view):
        continue
    width = max(e_max - e_min, 0.5)
    height = max(n_max - n_min, 0.5)
    ax.add_patch(mpatches.Rectangle((e_min, n_min), width, height,
                                    facecolor="purple", alpha=0.18,
                                    edgecolor="purple"))
    short = name.replace("TemplateCube_Rounded_", "TC_")
    ax.text(e_min + width / 2, n_min + height / 2, short,
            ha="center", va="center", fontsize=8, color="purple", weight="bold")
    drawn_cubes += 1

# Voxel hit-density (light shade — every cell that ever saw any voxel).
for (cn, ce), h in hits_2d.items():
    if (cn, ce) in promoted:
        continue
    n = cn * resolution_m
    e = ce * resolution_m
    alpha = min(0.5, 0.05 + 0.04 * math.log1p(h))
    ax.add_patch(mpatches.Rectangle((e - resolution_m / 2, n - resolution_m / 2),
                                    resolution_m, resolution_m,
                                    facecolor="grey", alpha=alpha, linewidth=0))

# Promoted (approx) static cells — bright orange, with inflation halo.
inflation_cells = max(1, int(math.ceil(inflation_m / resolution_m)))
inflated = set()
for (cn, ce) in promoted:
    for di in range(-inflation_cells, inflation_cells + 1):
        for dj in range(-inflation_cells, inflation_cells + 1):
            if di * di + dj * dj <= inflation_cells * inflation_cells:
                inflated.add((cn + di, ce + dj))
# Inflation halo first (lighter)
for (cn, ce) in inflated - promoted:
    n = cn * resolution_m
    e = ce * resolution_m
    ax.add_patch(mpatches.Rectangle((e - resolution_m / 2, n - resolution_m / 2),
                                    resolution_m, resolution_m,
                                    facecolor="orange", alpha=0.18, linewidth=0))
# Promoted core
for (cn, ce) in promoted:
    n = cn * resolution_m
    e = ce * resolution_m
    ax.add_patch(mpatches.Rectangle((e - resolution_m / 2, n - resolution_m / 2),
                                    resolution_m, resolution_m,
                                    facecolor="orange", alpha=0.85,
                                    edgecolor="darkorange", linewidth=0.5))

# Radar-only cells (cyan ×) — these would also appear in the grid under
# allow_radar_promotion=true after radar_promotion_hits observations.
for (cn, ce) in radar_cells:
    n = cn * resolution_m
    e = ce * resolution_m
    ax.plot(e, n, "x", color="cyan", markersize=10, markeredgewidth=2)

# Drone trajectory.
if drone:
    ax.plot([d[1] for d in drone], [d[0] for d in drone], "-",
            color="blue", linewidth=1.4, alpha=0.7,
            label=f"Drone GT path (n={len(drone)})")
    ax.plot(drone[0][1], drone[0][0], "o", color="blue", markersize=10,
            markeredgecolor="navy", label="Start")
    ax.plot(drone[-1][1], drone[-1][0], "s", color="blue", markersize=10,
            markeredgecolor="navy", label="End")

# Waypoints.
for i, (n, e, _z) in enumerate(waypoints):
    ax.plot(e, n, "*", markersize=22, color="orange",
            markeredgecolor="black", markeredgewidth=0.6)
    ax.annotate(f"WP{i + 1}", (e, n), fontsize=10, color="darkorange",
                weight="bold", xytext=(8, 0), textcoords="offset points")

# Legend handles for the rectangle layers.
legend_handles = [
    mpatches.Patch(facecolor="orange", alpha=0.85, edgecolor="darkorange",
                   label=f"Promoted static cells (>= {promotion_hits} hits @ conf >= {min_confidence})"),
    mpatches.Patch(facecolor="orange", alpha=0.18,
                   label=f"Inflation halo (radius {inflation_m} m)"),
    mpatches.Patch(facecolor="grey", alpha=0.4,
                   label="Voxel-hit cells (sub-promotion)"),
    mpatches.Patch(facecolor="purple", alpha=0.4, label="GT cubes"),
    plt.Line2D([], [], marker="x", linestyle="None", color="cyan", markersize=10,
               label=f"Radar cells (n={len(radar_cells)})"),
]
if drone:
    legend_handles.append(plt.Line2D([], [], color="blue", linewidth=1.4,
                                     label="Drone GT path"))

ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.set_title(f"Planner-grid overlay — {run.name}\n"
             f"resolution={resolution_m} m, promotion_hits={promotion_hits}, "
             f"min_conf={min_confidence}, inflation={inflation_m} m\n"
             f"voxels={total_voxels:,}  |  promoted_cells={len(promoted)}  |  "
             f"inflated_cells={len(inflated)}  |  radar_cells={len(radar_cells)}")
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.legend(handles=legend_handles, loc="upper left", fontsize=8)

# Frame: cover drone + waypoints + promoted cells with margin.  Skip
# cubes that were view-clipped above — re-adding the full DEFAULT_CUBES
# inventory here pulls the limits back out to the full ±100 m Blocks
# scene and undoes the mission-area clipping (Copilot review on PR
# #704).
xs = [d[1] for d in drone] + [w[1] for w in waypoints]
ys = [d[0] for d in drone] + [w[0] for w in waypoints]
for (cn, ce) in inflated:
    xs.append(ce * resolution_m)
    ys.append(cn * resolution_m)
for n_min, n_max, e_min, e_max in DEFAULT_CUBES.values():
    if not _bbox_overlaps(n_min, n_max, e_min, e_max, _view):
        continue
    xs.extend([e_min, e_max])
    ys.extend([n_min, n_max])
if xs and ys:
    margin = 5
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)

out = run / "planner_grid_overlay.png"
fig.tight_layout()
fig.savefig(out, dpi=130, bbox_inches="tight")
print(f"Saved {out}")
print(f"\nQuick stats:")
print(f"  Voxels seen      : {total_voxels:,}")
print(f"  Cells with hits  : {len(hits_2d):,}")
print(f"  Cells >= conf    : {len(hits_2d_conf):,}")
print(f"  Promoted (approx): {len(promoted)}")
print(f"  Inflated halo    : {len(inflated)}")
print(f"  Radar cells      : {len(radar_cells)}")
print(f"  Drone GT samples : {len(drone)}")
