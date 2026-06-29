#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.

"""
tests/test_lib_check_proximity.py — deterministic tests for lib_check_proximity.py
(Issue #796 proximity collision gate).

Standalone-runnable (`python3 tests/test_lib_check_proximity.py`) AND
pytest-compatible (`test_*` functions).  CI's pytest job is scoped to
tests/test_orchestrator/, so this is primarily a runnable regression artifact +
the functional verification for the gate's geometry logic (cylinder, box,
Z-overlap, fail-closed).  Exercises the helper as a subprocess so it also covers
the CLI / exit-code contract the scenario runner depends on.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

HELPER = Path(__file__).resolve().parent / "lib_check_proximity.py"

# Minimal world: a cylinder (centre 10,10,3; r=1.0, len=6 → z 0..6) and a box
# (centre 5,3,5; size 1.5x1.5x10 → x±0.75, y±0.75, z 0..10).
_WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="t">
    <model name="obstacle_green_cylinder">
      <static>true</static>
      <pose>10 10 3.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>1.0</radius><length>6.0</length></cylinder></geometry>
        </collision>
      </link>
    </model>
    <model name="obstacle_orange_box">
      <static>true</static>
      <pose>5 3 5.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.5 1.5 10</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""


def _odom(samples: list[tuple[float, float, float]]) -> str:
    """Render (x,y,z) samples as gz.msgs.Odometry text format."""
    blocks = []
    for x, y, z in samples:
        blocks.append(
            f"pose {{\n  position {{\n    x: {x}\n    y: {y}\n    z: {z}\n  }}\n}}\n---"
        )
    return "\n".join(blocks) + "\n"


def _run(pose_text: str, world_text: str | None = _WORLD_SDF, *extra: str) -> int:
    with tempfile.TemporaryDirectory() as d:
        pose = Path(d) / "odom.log"
        pose.write_text(pose_text)
        world = Path(d) / "w.sdf"
        world.write_text(world_text if world_text is not None else _WORLD_SDF)
        cmd = [sys.executable, str(HELPER), str(pose), "--world-sdf", str(world), *extra]
        return subprocess.run(cmd, capture_output=True, text=True).returncode


def test_clear_trajectory_passes() -> None:
    # Flies the corridor well clear of both obstacles at cruise altitude.
    rc = _run(_odom([(0, 0, 4), (0, 18, 4), (20, 18, 4), (20, 8, 4)]))
    assert rc == 0, f"clear path must PASS, got {rc}"


def test_cylinder_penetration_fails() -> None:
    # Drone passes through the cylinder centre at z=3 (inside z 0..6, r=1.0).
    rc = _run(_odom([(0, 0, 4), (10, 10, 3), (20, 20, 4)]))
    assert rc == 1, f"cylinder hit must FAIL, got {rc}"


def test_box_penetration_fails() -> None:
    # Drone passes through the box centre (5,3,5) — inside x±0.75,y±0.75,z0..10.
    rc = _run(_odom([(5, 3, 5)]))
    assert rc == 1, f"box hit must FAIL, got {rc}"


def test_above_obstacle_passes() -> None:
    # Directly over the cylinder but above its 6 m top (z=8) → no Z-overlap.
    rc = _run(_odom([(10, 10, 8)]))
    assert rc == 0, f"flying above the obstacle must PASS, got {rc}"


def test_grazing_within_drone_radius_fails() -> None:
    # 0.2 m outside the cylinder wall (horiz dist r+0.2=1.2) but drone radius
    # 0.35 → sphere overlaps the wall → penetration.
    rc = _run(_odom([(10, 11.2, 3)]), _WORLD_SDF, "--drone-radius", "0.35")
    assert rc == 1, f"grazing within drone radius must FAIL, got {rc}"


def test_near_miss_is_warn_not_fail() -> None:
    # 0.5 m clear of the wall, near-miss margin 1.0 → WARN, still PASS (rc 0).
    rc = _run(_odom([(10, 11.85, 3)]), _WORLD_SDF, "--drone-radius", "0.35",
              "--near-miss-margin", "1.0")
    assert rc == 0, f"near-miss must WARN not FAIL, got {rc}"


def test_empty_log_expect_nonempty_fails_closed() -> None:
    rc = _run("", _WORLD_SDF, "--expect-nonempty")
    assert rc == 3, f"empty pose log + --expect-nonempty must fail-closed (3), got {rc}"


def test_empty_log_without_flag_passes() -> None:
    rc = _run("", _WORLD_SDF)
    assert rc == 0, f"empty pose log without flag is legacy PASS, got {rc}"


def _main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} proximity-gate tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_main())
