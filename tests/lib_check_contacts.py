#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary
# Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md.

"""
tests/lib_check_contacts.py — Scenario flight-quality gate: contact-sensor

Epic #740 Layer 3 (cold-start hardening); related root-cause investigation
in #727.  Observability layer that detects when a Gazebo scenario run
produces drone-vs-obstacle physical contact, even when the runner's
existing log-pattern checks would have reported PASS.

Background:
    Scenario 26 (and sometimes 18) in the existing `tests/run_scenario_
    gazebo.sh` suite report PASS while the drone is visibly hitting objects
    in the Gazebo GUI (see #727 reproduction matrix; scenario 25 also
    exhibits the symptom but is Tier-1 / AirSim and runs under
    `tests/run_scenario.sh` — out of scope for this Gazebo-only gate).
    Pass criteria today validate log content + FSM transitions, not
    physical flight quality.  This gate closes that observability gap by
    parsing live `gz topic -e -t /world/<name>/contacts` capture and
    asserting no drone-vs-obstacle contact occurred during the run.

What is detected:
    Any contact between the drone model (default: `x500_companion`) and a
    non-allowlisted collider.  Default allowlist:
        - `ground_plane` (Gazebo's default ground — would fire on every
          takeoff / landing legitimately)
        - configurable per-scenario via `flight_quality_gates.contact_
          allowlist` (e.g. landing pads).

What is NOT detected (out of scope for this MVP):
    - Drone-vs-ground crashes during takeoff (rotor-asymmetry failure
      mode) — requires altitude / attitude correlation and is the
      responsibility of `max_attitude_error_during_arming` gate (#740
      Layer 3 follow-up).
    - Drone-vs-ground at LAND (legitimate touchdown).

Exit codes:
    0  — no drone-vs-obstacle contacts detected (PASS)
    1  — at least one drone-vs-obstacle contact detected (FAIL)
    2  — input file missing / unreadable (treated as FAIL by the runner)
    3  — log empty but --expect-nonempty set: contact topic not publishing
         (Contact plugin / sensors missing or broken) — fail-closed (Issue #791)

Usage:
    python3 tests/lib_check_contacts.py <gz_contacts_log> \\
        [--drone-pattern x500_companion] \\
        [--allowlist ground_plane,landing_pad] \\
        [--max-events 5]

    Returns exit code 0 (PASS) or 1 (FAIL).  On FAIL, prints one line
    per detected drone-vs-obstacle contact pair (deduplicated) to stdout.

Design notes:
    `gz topic -e -t ...` emits protobuf text format with each message
    delimited by `---` and each contact block as `contact { ... }`.  We
    don't need a full protobuf parser — a small state machine that
    extracts the `name:` lines from each `collision1`/`collision2` sub-
    block is sufficient and avoids adding a gz-msgs Python dependency.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


# Regex to match `name: "..."` lines in the gz-topic text-format output.
# Matches: 4 leading spaces is typical indentation but be tolerant.
_NAME_RE = re.compile(r'\s*name:\s*"([^"]+)"\s*$')

# Regex to match the start of a `collision1 {` or `collision2 {` block.
_COLLISION_OPEN_RE = re.compile(r'\s*(collision1|collision2)\s*\{\s*$')

# Regex to match the start of a `contact {` block.
_CONTACT_OPEN_RE = re.compile(r'\s*contact\s*\{\s*$')


def is_allowlisted(name: str, allowlist: list[str]) -> bool:
    """Return True iff `name` contains any allowlist substring.

    Substring match (not regex) — operators typically know model names
    by short prefix (e.g. ``ground_plane``, ``landing_pad``), not by
    full collision-path like ``ground_plane::link::collision``.
    """
    return any(token in name for token in allowlist)


def parse_contacts(
    log_path: Path, drone_pattern: str, allowlist: list[str]
) -> list[tuple[str, str]]:
    """Parse a gz-topic text-format contacts log.

    Returns the deduplicated list of (drone_collision_name, obstacle_
    collision_name) pairs where one collider matches `drone_pattern` and
    the other does NOT match any allowlist token.

    The parser is line-oriented and uses a small state machine.  We don't
    care about exact protobuf nesting (the format is regular enough that
    a regex-based approach over collision1/collision2 blocks is
    sufficient and avoids a heavy dependency).
    """
    events: set[tuple[str, str]] = set()

    in_contact = False
    current_collision_block: str | None = None  # "collision1" or "collision2" or None
    name1: str | None = None
    name2: str | None = None

    with log_path.open("r", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.rstrip("\n")

            # gz-topic text format separates protobuf messages with `---`.
            # Reset state at message boundaries — defensive against a
            # truncated `contact { ... }` block (e.g. when `gz topic` is
            # SIGKILLed mid-write) that would otherwise carry stale
            # collision names into the next message and produce phantom
            # contact events.
            if line.strip() == "---":
                in_contact = False
                current_collision_block = None
                name1 = None
                name2 = None
                continue

            if _CONTACT_OPEN_RE.match(line):
                in_contact = True
                name1 = None
                name2 = None
                current_collision_block = None
                continue

            if not in_contact:
                continue

            m = _COLLISION_OPEN_RE.match(line)
            if m:
                current_collision_block = m.group(1)
                continue

            # Closing brace at any depth — when we see a top-level `}` that
            # closes the contact block, evaluate the pair.  We can't track
            # exact depth from text format alone, but the structure is
            # regular enough that a plain `}` after we have both names is
            # the contact-block close.
            if line.strip() == "}":
                if current_collision_block:
                    current_collision_block = None
                    continue
                if name1 and name2:
                    drone_name, other_name = _classify_pair(
                        name1, name2, drone_pattern, allowlist
                    )
                    if drone_name and other_name:
                        events.add((drone_name, other_name))
                in_contact = False
                continue

            mname = _NAME_RE.match(line)
            if mname and current_collision_block:
                name_value = mname.group(1)
                if current_collision_block == "collision1":
                    name1 = name_value
                elif current_collision_block == "collision2":
                    name2 = name_value
                continue

            # Other fields inside contact (position, normal, etc.) — ignore.

    return sorted(events)


def _classify_pair(
    a: str, b: str, drone_pattern: str, allowlist: list[str]
) -> tuple[str | None, str | None]:
    """Return (drone_collision_name, obstacle_collision_name) iff exactly
    one of `a`/`b` matches the drone pattern AND the other is not allow-
    listed.  Otherwise return ``(None, None)``."""
    a_drone = drone_pattern in a
    b_drone = drone_pattern in b
    if a_drone == b_drone:
        # Either both are drone (self-collision — ignore) or neither is
        # the drone (world-vs-world contact — not our concern).
        return (None, None)
    drone_name = a if a_drone else b
    other_name = b if a_drone else a
    if is_allowlisted(other_name, allowlist):
        return (None, None)
    return (drone_name, other_name)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Scenario flight-quality gate: detect drone-vs-obstacle "
            "contacts in a captured `gz topic -e -t /world/<name>/contacts` "
            "log (#740 Layer 3, Gate 1)."
        )
    )
    parser.add_argument(
        "log_path",
        type=Path,
        help="Path to the gz-topic-text-format contacts log captured during the scenario run.",
    )
    parser.add_argument(
        "--drone-pattern",
        default="x500_companion",
        help="Substring identifying the drone model in collision names (default: %(default)s).",
    )
    parser.add_argument(
        "--allowlist",
        default="ground_plane",
        help=(
            "Comma-separated substrings identifying acceptable colliders "
            "(default: %(default)s).  Drone-vs-allowlisted contacts are "
            "ignored.  Typical additions: 'landing_pad,helipad'."
        ),
    )
    parser.add_argument(
        "--max-events",
        type=int,
        default=5,
        help=(
            "Maximum unique drone-vs-obstacle pairs to print to stdout on "
            "FAIL (default: %(default)d).  Prevents the report flooding "
            "when the drone is sustained-in-contact with one obstacle.  "
            "Overridden by --verbose."
        ),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every detected drone-vs-obstacle pair (overrides --max-events).",
    )
    parser.add_argument(
        "--expect-nonempty",
        action="store_true",
        help=(
            "Fail-closed (Issue #791): treat an EMPTY contacts log as a failure "
            "(exit 3) instead of PASS.  Use when the world has contact sensors "
            "(test_world.sdf) so a real flight ALWAYS logs ground contacts at "
            "takeoff/landing — an empty log then means the Contact plugin / "
            "sensors aren't publishing and the gate cannot certify no-collision."
        ),
    )
    args = parser.parse_args()
    if args.verbose:
        args.max_events = sys.maxsize

    if not args.log_path.exists():
        print(
            f"[contact-sensor] FAIL: contacts log not found at {args.log_path} — "
            f"`gz topic -e` capture did not start or was killed before producing output.",
            file=sys.stderr,
        )
        return 2

    if args.log_path.stat().st_size == 0:
        # Empty log — gz topic ran but received no messages.
        # Issue #791: with contact sensors on the world's obstacles AND
        # ground_plane (test_world.sdf), a real flight ALWAYS produces ground
        # contacts at takeoff/landing.  So under --expect-nonempty an empty log
        # is NOT "a clean run" — it means the Contact system plugin / sensors
        # aren't publishing, and the gate cannot certify no-collision.  Fail
        # closed (exit 3) rather than the historical fail-open PASS that let a
        # structurally-blind detector green-light real strikes.
        if args.expect_nonempty:
            print(
                f"[contact-sensor] FAIL: contacts log is EMPTY but contact sensors are "
                f"expected ({args.log_path}).  The /world/<name>/contacts topic published "
                f"nothing — Contact system plugin or per-model contact sensors are missing "
                f"or not firing.  Cannot certify no-collision (fail-closed, Issue #791).",
                file=sys.stderr,
            )
            return 3
        # Legacy/opt-out behaviour: no sensors expected → empty == clean.
        print(
            f"[contact-sensor] PASS: no contact events captured during the run "
            f"({args.log_path} is empty)."
        )
        return 0

    allowlist = [tok.strip() for tok in args.allowlist.split(",") if tok.strip()]

    events = parse_contacts(args.log_path, args.drone_pattern, allowlist)

    if not events:
        print(
            f"[contact-sensor] PASS: no drone-vs-obstacle contacts detected "
            f"(scanned {args.log_path}, drone='{args.drone_pattern}', "
            f"allowlist={allowlist})."
        )
        return 0

    print(
        f"[contact-sensor] FAIL: {len(events)} unique drone-vs-obstacle contact pair(s) "
        f"detected during the run.  Drone model '{args.drone_pattern}' collided with:"
    )
    for drone_name, other_name in events[: args.max_events]:
        print(f"  - {drone_name}  ↔  {other_name}")
    if len(events) > args.max_events:
        print(f"  ... and {len(events) - args.max_events} more (truncated).")
    print(
        "  Source: `gz topic -e -t /world/<name>/contacts` capture.  Re-run with "
        "--verbose for the full event list."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
