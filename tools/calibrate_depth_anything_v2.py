#!/usr/bin/env python3
"""
calibrate_depth_anything_v2.py — extract anchored-scale references from a
Depth Anything V2 scenario run (Issue #616).

Reads the `perception.log` from a scenario run and scans for DA V2 raw-range
diagnostic lines (emitted every 30th frame by default):

    [DepthAnythingV2] 518x518 depth map in NNms (inv_depth range: [X.XXX, Y.YYY])

Aggregates the per-frame (min, max) pairs across the run and prints the
recommended `raw_min_ref` / `raw_max_ref` values — plus a ready-to-paste
`config_overrides` block — for pinning DA V2's anchored-scale mode.

Usage:
    python3 tools/calibrate_depth_anything_v2.py <path_to_perception.log>

Optional:
    --percentile 99   Use the N-th percentile instead of absolute min/max
                      (robustness against one-frame outliers).  Default 100
                      (absolute min/max).
    --emit-json PATH  Also write the coefficients as a JSON file for
                      scripted consumption.

This is a first-pass calibration — anchors the *scale* only.  The linear
fit (`calibration_coef_a` / `calibration_coef_b`) defaults to identity
(1.0 / 0.0).  For a true metric fit you need matched
`(raw_inverse_depth, ground_truth_metres)` pixel pairs from a Cosys
`DepthCam` stream — tracked as a follow-up issue; not in #616 scope.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# Matches the diagnostic line emitted by DepthAnythingV2Estimator::estimate():
#   "[DepthAnythingV2] 518x518 depth map in 42ms (inv_depth range: [0.123, 0.987])"
RANGE_RE = re.compile(
    r"\[DepthAnythingV2\].*inv_depth range:\s*\[([-+]?\d+(?:\.\d+)?),\s*([-+]?\d+(?:\.\d+)?)\]"
)


def parse_log(path: Path) -> list[tuple[float, float]]:
    """Return a list of (raw_min, raw_max) pairs from each matching log line."""
    pairs: list[tuple[float, float]] = []
    with path.open() as f:
        for line in f:
            m = RANGE_RE.search(line)
            if not m:
                continue
            try:
                lo, hi = float(m.group(1)), float(m.group(2))
            except ValueError:
                continue
            if lo >= hi:
                # Malformed or flat-scene frame; ignore.
                continue
            pairs.append((lo, hi))
    return pairs


def percentile(values: list[float], p: float) -> float:
    """Linear-interpolation percentile; `p` in [0, 100]."""
    if not values:
        raise ValueError("percentile() on empty list")
    s = sorted(values)
    if p <= 0:
        return s[0]
    if p >= 100:
        return s[-1]
    idx = (p / 100.0) * (len(s) - 1)
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    frac = idx - lo
    return s[lo] + (s[hi] - s[lo]) * frac


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("log_path", type=Path, help="Path to perception.log from a scenario run")
    ap.add_argument(
        "--percentile",
        type=float,
        default=100.0,
        help="Use the N-th percentile of per-frame min/max (default 100 = absolute). "
             "Applied symmetrically: raw_max_ref uses the N-th percentile of the per-frame "
             "maxes, raw_min_ref uses the (100-N)-th percentile of the per-frame mins. "
             "Try 99 or 95 to reject one-frame outliers on startup.",
    )
    ap.add_argument(
        "--emit-json",
        type=Path,
        default=None,
        help="Also write the coefficients as JSON for scripted consumption.",
    )
    args = ap.parse_args()

    if not args.log_path.exists():
        print(f"ERROR: log file not found: {args.log_path}", file=sys.stderr)
        return 2

    pairs = parse_log(args.log_path)
    if not pairs:
        print(f"ERROR: no DA V2 inv_depth range lines found in {args.log_path}", file=sys.stderr)
        print(
            "       Expected format: [DepthAnythingV2] WxH depth map in NNms "
            "(inv_depth range: [X, Y])",
            file=sys.stderr,
        )
        return 1

    # Sample-size sanity check.  The diagnostic is emitted every 30th frame,
    # so a single match means the run barely exited bootstrap.  Pinning
    # calibration refs to one sample makes the linear fit fragile —
    # any startup transient becomes the global anchor.
    if len(pairs) < 2:
        print(
            f"WARNING: only {len(pairs)} DA V2 range sample(s) parsed — calibration "
            "refs derived from such a small sample are unreliable.  Re-run the "
            "scenario for at least ~10 seconds (≥ 60 sampled frames) for a "
            "trustworthy fit.",
            file=sys.stderr,
        )

    mins = [p[0] for p in pairs]
    maxs = [p[1] for p in pairs]

    if args.percentile >= 100.0:
        raw_min_ref = min(mins)
        raw_max_ref = max(maxs)
        tag = "absolute min/max"
    else:
        # Symmetric — lower percentile of mins, upper percentile of maxs.
        raw_min_ref = percentile(mins, 100.0 - args.percentile)
        raw_max_ref = percentile(maxs, args.percentile)
        tag = f"{args.percentile:.0f}th percentile"

    # Report
    print(f"[calibrate_dav2] Parsed {len(pairs)} frames from {args.log_path}")
    print(f"[calibrate_dav2] Per-frame min range: [{min(mins):.3f}, {max(mins):.3f}]")
    print(f"[calibrate_dav2] Per-frame max range: [{min(maxs):.3f}, {max(maxs):.3f}]")
    print(f"[calibrate_dav2] Using {tag}: raw_min_ref={raw_min_ref:.6f}, "
          f"raw_max_ref={raw_max_ref:.6f}")
    print()
    print("[calibrate_dav2] Paste into your scenario's config_overrides:")
    print()
    print("  \"perception\": {")
    print("    \"depth_estimator\": {")
    print("      \"dav2\": {")
    print("        \"calibration_enabled\": true,")
    print(f"        \"raw_min_ref\": {raw_min_ref:.6f},")
    print(f"        \"raw_max_ref\": {raw_max_ref:.6f},")
    print("        \"calibration_coef_a\": 1.0,")
    print("        \"calibration_coef_b\": 0.0")
    print("      }")
    print("    }")
    print("  }")
    print()
    print("[calibrate_dav2] NOTE: coef_a / coef_b are identity passthrough.  For a")
    print("[calibrate_dav2] true metric fit you need matched (raw, GT) pixel pairs")
    print("[calibrate_dav2] from a Cosys DepthCam stream — tracked as a follow-up.")

    if args.emit_json:
        out = {
            "calibration_enabled": True,
            "raw_min_ref": raw_min_ref,
            "raw_max_ref": raw_max_ref,
            "calibration_coef_a": 1.0,
            "calibration_coef_b": 0.0,
            "_meta": {
                "source_log": str(args.log_path),
                "frames_parsed": len(pairs),
                "percentile": args.percentile,
            },
        }
        args.emit_json.parent.mkdir(parents=True, exist_ok=True)
        args.emit_json.write_text(json.dumps(out, indent=2) + "\n")
        print(f"[calibrate_dav2] Wrote JSON to {args.emit_json}")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
