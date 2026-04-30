#!/usr/bin/env python3
"""Visualize PATH A voxels overlaid with the spawned scene obstacles.

Reads the latest scenario-33 run's `path_a_voxel_trace.jsonl` and
produces a 4-panel PNG showing top-down + side views + 3D perspective.

Dependencies:
    pip3 install matplotlib numpy
    # or: sudo apt install python3-matplotlib python3-numpy

Usage:
    python3 deploy/visualize_voxels.py
    python3 deploy/visualize_voxels.py /path/to/run_dir
"""

import json
import math
import os
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers projection)
    import numpy as np
    from matplotlib.patches import Rectangle, Circle
except ModuleNotFoundError as e:
    sys.exit(
        f"Missing dependency: {e.name}\n"
        "Install with:\n"
        "  pip3 install matplotlib numpy\n"
        "  # or: sudo apt install python3-matplotlib python3-numpy"
    )


# ── Spawned scene obstacles for scenario 33 (from scene_populate.log) ──
# (name, x, y, z_min, z_max, half_extent_x, half_extent_y, marker, color)
SCENE_OBSTACLES = [
    # pillars: 0.5m diameter (≈0.25m half), 5m tall (z 0..5)
    ("pillar_01",   8.0, 12.0, 0.0, 5.0, 0.25, 0.25, "P", "#1f77b4"),
    ("pillar_02",  22.0, 14.0, 0.0, 5.0, 0.25, 0.25, "P", "#1f77b4"),
    # walls 4×4: half-extent 2m (yaw shape varies but we draw as boxes)
    ("wall_01",    15.0, 10.0, 0.0, 4.0, 2.0,  0.5,  "W", "#d62728"),
    ("wall_02",    15.0, 10.0, 0.0, 4.0, 0.5,  2.0,  "W", "#d62728"),
    ("chair_01",    4.0, 18.0, 0.0, 1.0, 0.5,  0.5,  "C", "#9467bd"),
    ("couch_01",   20.0, 12.0, 0.0, 1.0, 1.0,  0.5,  "C", "#8c564b"),
    ("bush_01",    10.0,  5.0, 0.0, 1.0, 0.5,  0.5,  "B", "#2ca02c"),
    # The Blocks-environment background cubes the drone keeps hitting
    ("cube_9_obs", 22.4, 24.1, 0.0, 1.0, 0.5,  0.5,  "X", "#ff7f0e"),
    ("cube_66",    30.3, 24.1, 0.0, 1.0, 0.5,  0.5,  "X", "#ff7f0e"),
]


def find_latest_run(scenario_dir: Path) -> Path:
    runs = sorted(
        [d for d in scenario_dir.iterdir() if d.is_dir() and d.name.startswith("2026-")],
        key=lambda d: d.name,
        reverse=True,
    )
    for r in runs:
        if (r / "path_a_voxel_trace.jsonl").exists():
            return r
    raise SystemExit(f"No run with path_a_voxel_trace.jsonl under {scenario_dir}")


def parse_drone_trajectory(mission_log: Path):
    xs, ys, zs, ts = [], [], [], []
    if not mission_log.exists():
        return xs, ys, zs, ts
    with mission_log.open() as f:
        for line in f:
            if "PlanBase] pos=(" not in line:
                continue
            try:
                pos_str = line.split("pos=(")[1].split(")")[0]
                x, y, z = (float(v) for v in pos_str.split(","))
                xs.append(x); ys.append(y); zs.append(z)
                ts.append(line.split("]")[0].strip("[ "))
            except Exception:
                pass
    return xs, ys, zs, ts


def load_voxels(trace: Path, max_voxels: int = 200_000):
    """Subsample voxels uniformly to keep plot tractable."""
    xs, ys, zs, confs, classes = [], [], [], [], []
    with trace.open() as f:
        all_voxels = []
        for line in f:
            try:
                b = json.loads(line)
            except json.JSONDecodeError:
                continue
            for v in b.get("voxels", []):
                # voxel format: [x, y, z, semantic_label, confidence]
                all_voxels.append(v)
    n = len(all_voxels)
    if n == 0:
        return np.array(xs), np.array(ys), np.array(zs), np.array(confs), np.array(classes), n
    if n > max_voxels:
        step = n // max_voxels
        all_voxels = all_voxels[::step]
    for v in all_voxels:
        xs.append(v[0]); ys.append(v[1]); zs.append(v[2])
        classes.append(int(v[3]) if len(v) > 3 else 0)
        confs.append(float(v[4]) if len(v) > 4 else 1.0)
    return (np.array(xs), np.array(ys), np.array(zs),
            np.array(confs), np.array(classes), n)


def main():
    if len(sys.argv) > 1:
        run_dir = Path(sys.argv[1])
    else:
        scenario_dir = Path(
            "/home/nav/Projects/companion_software_stack_worktrees/"
            "perception-v2-integration/drone_logs/scenarios_cosys/"
            "33_non_coco_obstacles"
        )
        run_dir = find_latest_run(scenario_dir)
    print(f"Run dir: {run_dir}")

    trace = run_dir / "path_a_voxel_trace.jsonl"
    if not trace.exists():
        raise SystemExit(f"No trace at {trace}")

    xs, ys, zs, confs, classes, n_total = load_voxels(trace)
    print(f"Loaded {len(xs):,} voxels (subsampled from {n_total:,})")

    drone_xs, drone_ys, drone_zs, _ = parse_drone_trajectory(run_dir / "mission_planner.log")
    print(f"Drone trajectory points: {len(drone_xs)}")

    # ── Figure setup ───────────────────────────────────────────
    fig = plt.figure(figsize=(20, 14))
    fig.suptitle(
        f"PATH A voxels vs scene obstacles — {run_dir.name}\n"
        f"{n_total:,} voxels total ({len(xs):,} plotted)",
        fontsize=14,
    )

    # Common voxel scatter args
    kw = dict(c=zs, cmap="viridis", s=0.5, alpha=0.3)

    # ── Panel 1: Top-down (XY) ─────────────────────────────────
    ax1 = fig.add_subplot(2, 2, 1)
    sc1 = ax1.scatter(xs, ys, **kw)
    plt.colorbar(sc1, ax=ax1, label="Z (m)")
    # Scene obstacles as rectangles
    for name, ox, oy, _, _, hx, hy, marker, color in SCENE_OBSTACLES:
        ax1.add_patch(Rectangle((ox - hx, oy - hy), 2 * hx, 2 * hy,
                                fill=False, edgecolor=color, linewidth=2))
        ax1.text(ox, oy, name, fontsize=7, ha="center", va="center",
                 color=color, fontweight="bold")
    if drone_xs:
        ax1.plot(drone_xs, drone_ys, "r-", linewidth=1, alpha=0.7,
                 label="drone trajectory")
        ax1.plot(drone_xs[0], drone_ys[0], "go", markersize=10, label="start")
        ax1.plot(drone_xs[-1], drone_ys[-1], "rs", markersize=10, label="end")
    ax1.set_xlabel("X (m, world)"); ax1.set_ylabel("Y (m, world)")
    ax1.set_title("Top-down view (XY)")
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_xlim(-10, 50); ax1.set_ylim(-10, 50)

    # ── Panel 2: Side view (XZ) ────────────────────────────────
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.scatter(xs, zs, c=confs, cmap="plasma", s=0.5, alpha=0.3)
    for name, ox, _, z_min, z_max, hx, _, _, color in SCENE_OBSTACLES:
        ax2.add_patch(Rectangle((ox - hx, z_min), 2 * hx, z_max - z_min,
                                fill=False, edgecolor=color, linewidth=2))
    if drone_xs:
        ax2.plot(drone_xs, drone_zs, "r-", linewidth=1, alpha=0.7)
    ax2.axhline(y=0, color="brown", linewidth=1, alpha=0.5, label="ground")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
    ax2.set_title("Side view (XZ) — color = confidence")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(-10, 50); ax2.set_ylim(-2, 15)
    ax2.legend(loc="upper right", fontsize=8)

    # ── Panel 3: Side view (YZ) ────────────────────────────────
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.scatter(ys, zs, c=confs, cmap="plasma", s=0.5, alpha=0.3)
    for name, _, oy, z_min, z_max, _, hy, _, color in SCENE_OBSTACLES:
        ax3.add_patch(Rectangle((oy - hy, z_min), 2 * hy, z_max - z_min,
                                fill=False, edgecolor=color, linewidth=2))
    if drone_ys:
        ax3.plot(drone_ys, drone_zs, "r-", linewidth=1, alpha=0.7)
    ax3.axhline(y=0, color="brown", linewidth=1, alpha=0.5)
    ax3.set_xlabel("Y (m)"); ax3.set_ylabel("Z (m)")
    ax3.set_title("Side view (YZ) — color = confidence")
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(-10, 50); ax3.set_ylim(-2, 15)

    # ── Panel 4: 3D perspective ────────────────────────────────
    ax4 = fig.add_subplot(2, 2, 4, projection="3d")
    ax4.scatter(xs, ys, zs, c=zs, cmap="viridis", s=0.3, alpha=0.2)
    # Scene obstacles as 3D boxes
    for name, ox, oy, z_min, z_max, hx, hy, _, color in SCENE_OBSTACLES:
        # Draw 4 vertical edges for a quick 3D box outline
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                ax4.plot([ox + sx, ox + sx], [oy + sy, oy + sy],
                         [z_min, z_max], color=color, linewidth=1.5)
    if drone_xs:
        ax4.plot(drone_xs, drone_ys, drone_zs, "r-", linewidth=2,
                 alpha=0.8, label="drone")
    ax4.set_xlabel("X"); ax4.set_ylabel("Y"); ax4.set_zlabel("Z")
    ax4.set_title("3D perspective")
    ax4.set_xlim(-5, 50); ax4.set_ylim(-5, 50); ax4.set_zlim(0, 15)
    ax4.legend(loc="upper right", fontsize=8)

    out = run_dir / "voxels_vs_scene.png"
    plt.tight_layout()
    plt.savefig(out, dpi=120, bbox_inches="tight")
    print(f"\nSaved: {out}")
    print(f"Open it: xdg-open '{out}'  (or use any image viewer)")

    # Also produce a high-zoom on the cube cluster — most informative for
    # debugging the side-collisions.
    fig2, ax = plt.subplots(figsize=(12, 10))
    ax.scatter(xs, ys, c=zs, cmap="viridis", s=2, alpha=0.5)
    for name, ox, oy, _, _, hx, hy, marker, color in SCENE_OBSTACLES:
        ax.add_patch(Rectangle((ox - hx, oy - hy), 2 * hx, 2 * hy,
                                fill=False, edgecolor=color, linewidth=2))
        ax.text(ox, oy + hy + 0.3, name, fontsize=10, ha="center",
                color=color, fontweight="bold")
    if drone_xs:
        ax.plot(drone_xs, drone_ys, "r-", linewidth=1.5, alpha=0.7)
        ax.plot(drone_xs[0], drone_ys[0], "go", markersize=15, label="start")
        ax.plot(drone_xs[-1], drone_ys[-1], "rs", markersize=15, label="end")
    # Waypoints
    waypoints = [(0, 0), (10, 20), (25, 12), (10, 2), (0, 0)]
    wxs = [w[0] for w in waypoints]; wys = [w[1] for w in waypoints]
    ax.plot(wxs, wys, "y--", linewidth=2, alpha=0.6, label="planned waypoints")
    for i, (wx, wy) in enumerate(waypoints):
        ax.plot(wx, wy, "y*", markersize=20)
        ax.text(wx, wy - 1.5, f"WP{i + 1}", fontsize=11, ha="center",
                color="goldenrod", fontweight="bold")
    ax.set_xlim(-5, 35); ax.set_ylim(-5, 35)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m, world)"); ax.set_ylabel("Y (m, world)")
    ax.set_title(f"Cube cluster + waypoints (zoom) — {run_dir.name}")
    ax.legend(loc="upper right", fontsize=10)
    out2 = run_dir / "voxels_vs_scene_zoom.png"
    plt.tight_layout()
    plt.savefig(out2, dpi=120, bbox_inches="tight")
    print(f"Saved: {out2}")


if __name__ == "__main__":
    main()
