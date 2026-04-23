#!/usr/bin/env python3
"""tools/plot_voxel_trace.py — visualise PATH A voxel trace vs scene ground truth.

Inputs (any subset; at least the voxel trace is required):
  --voxel-trace    path_a_voxel_trace.jsonl produced by the P2 mask_projection_thread
                   when `perception.path_a.diag.trace_voxels` is true.
  --cosys-telemetry cosys_telemetry.jsonl  — ground-truth drone pose + collision ledger
                   produced by tools/cosys_telemetry_poller.
  --scene          config/scenes/*.json    — spawned-object ground truth.
  --out-prefix     prefix for output PNG files (e.g. drone_logs/.../analysis)

Produces:
  <prefix>_topdown.png    — top-down scatter of voxels + SLAM + Cosys-GT trajectories
                            + scene-object positions + collision points.
  <prefix>_depth_hist.png — depth distribution per batch (from voxel world pos vs
                            pose translation).
  <prefix>_pose_error.png — SLAM pose vs Cosys-GT pose error over time (if both
                            inputs present).

Usage:
  python3 tools/plot_voxel_trace.py \
      --voxel-trace drone_logs/.../path_a_voxel_trace.jsonl \
      --cosys-telemetry drone_logs/.../cosys_telemetry.jsonl \
      --scene config/scenes/cosys_static.json \
      --out-prefix drone_logs/.../analysis

Issue #612.  Reads JSONL line-by-line so it works on in-progress traces too.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from typing import Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _read_jsonl(path: pathlib.Path) -> List[dict]:
    records: List[dict] = []
    with path.open() as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"{path}:{line_no}: JSON error: {e}", file=sys.stderr)
    return records


def _voxels(trace: Iterable[dict]) -> np.ndarray:
    """Stack all voxel world positions across the trace. Shape (N, 3)."""
    rows = []
    for rec in trace:
        for v in rec.get("voxels", []):
            # [wx, wy, wz, label, conf]
            rows.append((v[0], v[1], v[2]))
    return np.asarray(rows, dtype=float).reshape(-1, 3)


def _trajectory(trace: Iterable[dict], pose_key: str = "pose_t") -> np.ndarray:
    """Drone trajectory from one of the traces.  Shape (T, 3)."""
    pts = []
    for rec in trace:
        p = rec.get(pose_key)
        if p:
            pts.append(p)
    return np.asarray(pts, dtype=float).reshape(-1, 3)


def _collisions(telemetry: Iterable[dict]) -> np.ndarray:
    """Impact points from fresh collision events. Shape (K, 3)."""
    pts = []
    for rec in telemetry:
        coll = rec.get("coll")
        if coll and coll.get("fresh"):
            pts.append(coll["pt"])
    return np.asarray(pts, dtype=float).reshape(-1, 3)


def _gt_trajectory(telemetry: Iterable[dict]) -> np.ndarray:
    pts = []
    for rec in telemetry:
        gt = rec.get("gt")
        if gt:
            pts.append(gt["pos"])
    return np.asarray(pts, dtype=float).reshape(-1, 3)


def _scene_objects(scene_path: pathlib.Path) -> List[Tuple[str, float, float, float]]:
    with scene_path.open() as f:
        data = json.load(f)
    out = []
    for obj in data.get("objects", []):
        out.append((obj.get("name", "?"), float(obj.get("x", 0)), float(obj.get("y", 0)),
                    float(obj.get("z", 0))))
    return out


def _plot_topdown(voxels: np.ndarray, slam_traj: np.ndarray, gt_traj: np.ndarray,
                  collisions: np.ndarray, scene: List[Tuple[str, float, float, float]],
                  out_path: pathlib.Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 10))
    if voxels.size:
        ax.scatter(voxels[:, 0], voxels[:, 1], c="tab:gray", s=3, alpha=0.25,
                   label=f"PATH A voxels ({len(voxels)})")
    if slam_traj.size:
        ax.plot(slam_traj[:, 0], slam_traj[:, 1], "b-", linewidth=1.5, alpha=0.7,
                label=f"SLAM trajectory ({len(slam_traj)} pts)")
        ax.scatter(slam_traj[0, 0], slam_traj[0, 1], c="blue", marker="o", s=60,
                   edgecolors="k", label="SLAM start")
    if gt_traj.size:
        ax.plot(gt_traj[:, 0], gt_traj[:, 1], "g--", linewidth=1.2, alpha=0.8,
                label=f"Cosys GT trajectory ({len(gt_traj)} pts)")
    if collisions.size:
        ax.scatter(collisions[:, 0], collisions[:, 1], c="red", marker="X", s=120,
                   edgecolors="k", zorder=5, label=f"Collision events ({len(collisions)})")
    for name, x, y, _z in scene:
        ax.scatter(x, y, c="orange", marker="s", s=150, edgecolors="k", zorder=4)
        ax.annotate(name, (x, y), xytext=(5, 5), textcoords="offset points", fontsize=8,
                    color="darkorange")
    ax.set_xlabel("X (world, m)")
    ax.set_ylabel("Y (world, m)")
    ax.set_title("PATH A voxel distribution — top-down view")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"  wrote {out_path}")


def _plot_depth_hist(trace: List[dict], out_path: pathlib.Path) -> None:
    """Distance of each voxel from the drone at that frame — proxy for DA V2 depth."""
    depths: List[float] = []
    for rec in trace:
        pose_t = rec.get("pose_t")
        if not pose_t:
            continue
        px, py, pz = pose_t
        for v in rec.get("voxels", []):
            dx = v[0] - px
            dy = v[1] - py
            dz = v[2] - pz
            depths.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    fig, ax = plt.subplots(figsize=(10, 6))
    if depths:
        ax.hist(depths, bins=60, color="tab:blue", edgecolor="k")
        ax.axvline(20.0, color="red", linestyle="--",
                   label="kMaxObstacleDepth (20 m filter)")
    ax.set_xlabel("||voxel − drone|| (m)")
    ax.set_ylabel("Voxel count")
    ax.set_title("PATH A voxel range distribution (sanity check on depth + filter)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"  wrote {out_path}")


def _plot_pose_error(trace: List[dict], telemetry: List[dict],
                     out_path: pathlib.Path) -> Optional[pathlib.Path]:
    """SLAM pose vs Cosys GT pose — Euclidean error over time."""
    if not trace or not telemetry:
        print("  pose-error plot skipped (needs both voxel trace + cosys telemetry)")
        return None

    # Simple nearest-neighbour time alignment on t_ns.
    slam = sorted((r["t_ns"], r.get("pose_t")) for r in trace if r.get("pose_t"))
    gt   = sorted((r["t_ns"], r["gt"]["pos"])  for r in telemetry if r.get("gt"))
    if not slam or not gt:
        print("  pose-error plot skipped (no timestamped poses)")
        return None

    slam_arr = np.asarray([(t, *p) for t, p in slam], dtype=float)
    gt_arr   = np.asarray([(t, *p) for t, p in gt], dtype=float)

    t0 = min(slam_arr[0, 0], gt_arr[0, 0])
    slam_t = (slam_arr[:, 0] - t0) / 1e9
    gt_t   = (gt_arr[:, 0] - t0) / 1e9
    errors: List[float] = []
    for i, t in enumerate(slam_t):
        j = int(np.argmin(np.abs(gt_t - t)))
        dp = slam_arr[i, 1:4] - gt_arr[j, 1:4]
        errors.append(float(np.linalg.norm(dp)))

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(slam_t, errors, "r-", linewidth=1.2)
    ax.set_xlabel("Time since first sample (s)")
    ax.set_ylabel("|SLAM − GT| (m)")
    ax.set_title("SLAM vs Cosys ground-truth pose error")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"  wrote {out_path}")
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Visualise PATH A voxel trace against scene ground truth.")
    ap.add_argument("--voxel-trace", required=True, type=pathlib.Path,
                    help="JSONL from P2 mask_projection_thread (perception.path_a.diag.trace_voxels)")
    ap.add_argument("--cosys-telemetry", type=pathlib.Path,
                    help="JSONL from cosys_telemetry_poller (GT pose + collisions)")
    ap.add_argument("--scene", type=pathlib.Path,
                    help="config/scenes/*.json with spawned-object positions")
    ap.add_argument("--out-prefix", type=pathlib.Path, default=pathlib.Path("voxel_trace"),
                    help="Prefix for output PNG files")
    args = ap.parse_args()

    if not args.voxel_trace.exists():
        print(f"error: voxel trace not found: {args.voxel_trace}", file=sys.stderr)
        return 2

    trace = _read_jsonl(args.voxel_trace)
    print(f"voxel trace: {len(trace)} batches, {sum(len(r.get('voxels', [])) for r in trace)} "
          "voxels total")

    telemetry: List[dict] = []
    if args.cosys_telemetry and args.cosys_telemetry.exists():
        telemetry = _read_jsonl(args.cosys_telemetry)
        print(f"cosys telemetry: {len(telemetry)} ticks, "
              f"{sum(1 for r in telemetry if r.get('coll', {}).get('fresh'))} fresh collisions")
    elif args.cosys_telemetry:
        print(f"warn: cosys telemetry not found: {args.cosys_telemetry}", file=sys.stderr)

    scene: List[Tuple[str, float, float, float]] = []
    if args.scene and args.scene.exists():
        scene = _scene_objects(args.scene)
        print(f"scene: {len(scene)} objects")
    elif args.scene:
        print(f"warn: scene file not found: {args.scene}", file=sys.stderr)

    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)

    voxels      = _voxels(trace)
    slam_traj   = _trajectory(trace, pose_key="pose_t")
    gt_traj     = _gt_trajectory(telemetry)
    collisions  = _collisions(telemetry)

    _plot_topdown(voxels, slam_traj, gt_traj, collisions, scene,
                  pathlib.Path(str(args.out_prefix) + "_topdown.png"))
    _plot_depth_hist(trace, pathlib.Path(str(args.out_prefix) + "_depth_hist.png"))
    _plot_pose_error(trace, telemetry,
                     pathlib.Path(str(args.out_prefix) + "_pose_error.png"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
