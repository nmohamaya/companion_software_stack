#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.

"""
tests/lib_check_proximity.py — Scenario flight-quality gate: proximity collision.

Issue #796 (belt-and-suspenders for #791).  An independent collision gate that
does NOT depend on gz contact-sensor plumbing: it compares the drone's
GROUND-TRUTH pose trace against the obstacle geometry parsed from the world SDF
and FAILs if the drone ever penetrates an obstacle's volume.

Why this exists alongside the #791 contact gate:
    The #791 gate relies on Gazebo's Contact system reporting static-vs-dynamic
    contacts, which is version-inconsistent.  This gate is sim-internal-
    independent (it only needs a pose stream + the world SDF) and is
    deterministically unit-testable, so a real strike is caught even if the
    contact mechanism regresses.

Inputs:
    - Pose log: a captured `gz topic -e -t /model/<drone>/odometry` stream
      (gz.msgs.Odometry text format).  The drone model runs
      `gz-sim-odometry-publisher-system`, so this topic is always published.
    - World SDF: `sim/worlds/test_world.sdf` — obstacle models
      (`<model name="obstacle_*">`) with cylinder/box collision geometry.

Collision model:
    The drone is a sphere of radius `--drone-radius` (default 0.35 m; x500 is
    ~0.47 m motor-to-motor).  For each pose sample we compute the signed
    clearance to each obstacle's solid (XY distance to the shape combined with
    the Z-overlap); clearance < 0 means the drone sphere penetrated the
    obstacle → collision.  A configurable near-miss band only WARNs.

Exit codes (mirroring lib_check_contacts.py):
    0  — no penetration (PASS; near-miss warnings do not fail)
    1  — at least one obstacle penetration detected (FAIL)
    2  — pose log / world SDF missing or unreadable (FAIL, infrastructure)
    3  — pose log empty but --expect-nonempty set: odometry not publishing
         (fail-closed) — cannot certify no-collision (Issue #796)
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ── World SDF obstacle parsing ───────────────────────────────────────────────
class Obstacle:
    """An obstacle solid from the world SDF.  `kind` is 'cylinder' or 'box'."""

    def __init__(self, name: str, x: float, y: float, z: float, kind: str, dims: dict):
        self.name = name
        self.x, self.y, self.z = x, y, z  # model-frame centre (SDF <pose>)
        self.kind = kind
        self.dims = dims  # cylinder: {radius, length}; box: {sx, sy, sz}

    def clearance(self, px: float, py: float, pz: float, drone_radius: float) -> float:
        """Signed clearance (m) from a drone sphere (centre p, radius
        drone_radius) to this obstacle's surface.  < 0 ⇒ penetration."""
        if self.kind == "cylinder":
            r = self.dims["radius"]
            half_h = self.dims["length"] / 2.0
            horiz = math.hypot(px - self.x, py - self.y)
            dxy_out = max(0.0, horiz - r)
            dz_out = max(0.0, abs(pz - self.z) - half_h)
            dist_to_surface = math.hypot(dxy_out, dz_out)
        else:  # box (axis-aligned; obstacles are unrotated)
            hx = self.dims["sx"] / 2.0
            hy = self.dims["sy"] / 2.0
            hz = self.dims["sz"] / 2.0
            dx_out = max(0.0, abs(px - self.x) - hx)
            dy_out = max(0.0, abs(py - self.y) - hy)
            dz_out = max(0.0, abs(pz - self.z) - hz)
            dist_to_surface = math.sqrt(dx_out * dx_out + dy_out * dy_out + dz_out * dz_out)
        return dist_to_surface - drone_radius


def _local_name(tag: str) -> str:
    """Strip any XML namespace from an element tag (SDF is namespace-free, but
    be defensive)."""
    return tag.rsplit("}", 1)[-1]


def parse_obstacles(sdf_path: Path, name_prefix: str = "obstacle_") -> list[Obstacle]:
    """Parse obstacle models (name starting with `name_prefix`) from the world
    SDF.  Returns their centre pose + cylinder/box dimensions.  Raises on a
    malformed SDF (treated as infrastructure failure by the caller)."""
    tree = ET.parse(sdf_path)
    obstacles: list[Obstacle] = []
    for model in tree.iter():
        if _local_name(model.tag) != "model":
            continue
        name = model.get("name", "")
        if not name.startswith(name_prefix):
            continue
        # Model-frame pose: "x y z roll pitch yaw" (obstacles are unrotated).
        pose_el = next(
            (c for c in model if _local_name(c.tag) == "pose"), None
        )
        if pose_el is None or not pose_el.text:
            continue
        parts = pose_el.text.split()
        if len(parts) < 3:
            continue
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        # First geometry under this model (collision or visual — identical here).
        cyl = next((e for e in model.iter() if _local_name(e.tag) == "cylinder"), None)
        box = next((e for e in model.iter() if _local_name(e.tag) == "box"), None)
        if cyl is not None:
            radius = float(next(e.text for e in cyl if _local_name(e.tag) == "radius"))
            length = float(next(e.text for e in cyl if _local_name(e.tag) == "length"))
            obstacles.append(Obstacle(name, x, y, z, "cylinder", {"radius": radius, "length": length}))
        elif box is not None:
            size_el = next(e for e in box if _local_name(e.tag) == "size")
            sx, sy, sz = (float(v) for v in size_el.text.split())
            obstacles.append(Obstacle(name, x, y, z, "box", {"sx": sx, "sy": sy, "sz": sz}))
    return obstacles


# ── Odometry pose-log parsing ────────────────────────────────────────────────
# gz.msgs.Odometry text format nests the position under `pose { position { ... } }`.
# We only need position x/y/z.  A small state machine extracts the position block
# of each message (delimited by `---`); `pose {`→`position {` then x/y/z floats.
_POSE_OPEN = re.compile(r"\s*pose\s*\{\s*$")
_POSITION_OPEN = re.compile(r"\s*position\s*\{\s*$")
_FLOAT_FIELD = re.compile(r"\s*([xyz]):\s*([-+0-9.eE]+)\s*$")


def parse_poses(log_path: Path) -> list[tuple[float, float, float]]:
    """Parse (x, y, z) samples from a gz.msgs.Odometry text-format capture."""
    poses: list[tuple[float, float, float]] = []
    in_pose = in_position = False
    cur: dict[str, float] = {}
    with log_path.open("r", errors="replace") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            if line.strip() == "---":
                in_pose = in_position = False
                cur = {}
                continue
            if _POSE_OPEN.match(line):
                in_pose = True
                continue
            if in_pose and _POSITION_OPEN.match(line):
                in_position = True
                cur = {}
                continue
            if in_position:
                m = _FLOAT_FIELD.match(line)
                if m:
                    cur[m.group(1)] = float(m.group(2))
                    if len(cur) == 3:
                        poses.append((cur["x"], cur["y"], cur["z"]))
                        in_position = in_pose = False
                        cur = {}
                    continue
                # End of the position block before all three fields (defensive).
                if line.strip() == "}":
                    in_position = False
    return poses


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Scenario flight-quality gate: detect drone-vs-obstacle penetration "
            "from a ground-truth odometry pose capture + the world SDF geometry "
            "(Issue #796, independent backstop for the #791 contact gate)."
        )
    )
    ap.add_argument("pose_log", type=Path, help="gz.msgs.Odometry text-format capture log.")
    ap.add_argument("--world-sdf", type=Path, required=True, help="World SDF with obstacle models.")
    ap.add_argument("--drone-radius", type=float, default=0.35, help="Drone sphere radius (m).")
    ap.add_argument(
        "--near-miss-margin",
        type=float,
        default=0.0,
        help="Extra clearance (m) below which a sample is reported as a near-miss WARN (not a FAIL).",
    )
    ap.add_argument(
        "--expect-nonempty",
        action="store_true",
        help="Fail-closed: treat an empty pose log as FAIL (exit 3), not PASS.",
    )
    ap.add_argument("--max-events", type=int, default=5, help="Max penetration reports to print.")
    args = ap.parse_args()

    if not args.pose_log.exists():
        print(f"[proximity] FAIL: pose log not found at {args.pose_log}.", file=sys.stderr)
        return 2
    if not args.world_sdf.exists():
        print(f"[proximity] FAIL: world SDF not found at {args.world_sdf}.", file=sys.stderr)
        return 2

    if args.pose_log.stat().st_size == 0:
        if args.expect_nonempty:
            print(
                f"[proximity] FAIL: pose log is EMPTY ({args.pose_log}) — odometry topic not "
                f"publishing; cannot certify no-collision (fail-closed, Issue #796).",
                file=sys.stderr,
            )
            return 3
        print(f"[proximity] PASS: no pose samples captured ({args.pose_log} is empty).")
        return 0

    try:
        obstacles = parse_obstacles(args.world_sdf)
    except Exception as exc:  # malformed SDF — infrastructure failure
        print(f"[proximity] FAIL: could not parse obstacles from {args.world_sdf}: {exc}",
              file=sys.stderr)
        return 2

    if not obstacles:
        print(f"[proximity] PASS: no obstacle models in {args.world_sdf} (nothing to hit).")
        return 0

    poses = parse_poses(args.pose_log)
    if not poses:
        # File non-empty but no parseable poses — treat like empty.
        if args.expect_nonempty:
            print(
                f"[proximity] FAIL: no parseable pose samples in {args.pose_log} — odometry "
                f"format unexpected; cannot certify no-collision (fail-closed).",
                file=sys.stderr,
            )
            return 3
        print(f"[proximity] PASS: no parseable pose samples in {args.pose_log}.")
        return 0

    # Worst (most negative) clearance per obstacle across the whole trace.
    penetrations: list[tuple[str, float, tuple[float, float, float]]] = []
    near_misses: list[tuple[str, float]] = []
    for obs in obstacles:
        worst = math.inf
        worst_p = (0.0, 0.0, 0.0)
        for px, py, pz in poses:
            c = obs.clearance(px, py, pz, args.drone_radius)
            if c < worst:
                worst, worst_p = c, (px, py, pz)
        if worst < 0.0:
            penetrations.append((obs.name, worst, worst_p))
        elif worst < args.near_miss_margin:
            near_misses.append((obs.name, worst))

    for name, clr in near_misses:
        print(f"[proximity] WARN: near-miss {clr:.2f} m from {name} (margin {args.near_miss_margin} m).")

    if not penetrations:
        print(
            f"[proximity] PASS: no obstacle penetration over {len(poses)} pose samples "
            f"(drone_radius={args.drone_radius} m, {len(obstacles)} obstacles)."
        )
        return 0

    print(
        f"[proximity] FAIL: drone penetrated {len(penetrations)} obstacle(s) "
        f"(ground-truth odometry vs {args.world_sdf.name}):"
    )
    for name, clr, (px, py, pz) in sorted(penetrations, key=lambda e: e[1])[: args.max_events]:
        print(f"  - {name}: penetration {(-clr):.2f} m at drone ({px:.1f},{py:.1f},{pz:.1f})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
