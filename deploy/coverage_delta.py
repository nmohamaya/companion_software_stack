#!/usr/bin/env python3
"""deploy/coverage_delta.py — changed-line coverage delta (ADR-016 Tier-0, advisory).

Reports the line-coverage of the lines THIS change adds/modifies, using an lcov
tracefile (coverage.info) + the git diff vs a base ref. Turns the
review-test-quality question "do the new tests actually exercise the new code?"
into a number instead of an agent's reading of the diff.

Usage:
  deploy/coverage_delta.py --coverage coverage.info [--base origin/main] [--min PCT]

Exit: 0 by default (advisory). With --min PCT, exits 1 when changed-line
coverage is below PCT (opt-in gating once a sensible floor is agreed).
"""
import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict


def changed_lines(base):
    """Return {relpath: set(added/modified line numbers)} for C/C++ files."""
    out = subprocess.run(
        ["git", "diff", "--unified=0", f"{base}...HEAD", "--",
         "*.cpp", "*.h", "*.hpp", "*.cc"],
        capture_output=True, text=True,
    ).stdout
    res = defaultdict(set)
    cur = None
    for line in out.splitlines():
        if line.startswith("+++ b/"):
            cur = line[6:]
        elif line.startswith("@@") and cur:
            m = re.search(r"\+(\d+)(?:,(\d+))?", line)
            if m:
                start = int(m.group(1))
                count = int(m.group(2) or "1")
                res[cur].update(range(start, start + count))
    return res


def parse_lcov(path):
    """Return {sf_path: {line: hits}} from an lcov tracefile."""
    cov = defaultdict(dict)
    cur = None
    with open(path) as f:
        for line in f:
            if line.startswith("SF:"):
                cur = line[3:].strip()
            elif line.startswith("DA:") and cur:
                parts = line[3:].strip().split(",")
                cov[cur][int(parts[0])] = int(parts[1])
            elif line.startswith("end_of_record"):
                cur = None
    return cov


def match_cov(relpath, cov):
    """lcov SF paths may be absolute; match a repo-relative path by suffix."""
    for sf, lines in cov.items():
        if sf == relpath or sf.endswith("/" + relpath):
            return lines
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--coverage", required=True)
    ap.add_argument("--base", default="origin/main")
    ap.add_argument("--min", type=float, default=None)
    a = ap.parse_args()

    if not os.path.exists(a.coverage):
        print(f"coverage file not found: {a.coverage} — skipping (advisory)")
        return 0

    changed = changed_lines(a.base)
    cov = parse_lcov(a.coverage)

    total = covered = 0
    per_file = []
    for relpath, lines in sorted(changed.items()):
        cmap = match_cov(relpath, cov)
        if cmap is None:
            continue  # not instrumented (header-only, excluded, or not compiled)
        ft = fc = 0
        for ln in lines:
            if ln in cmap:
                ft += 1
                fc += 1 if cmap[ln] > 0 else 0
        if ft:
            per_file.append((relpath, fc, ft))
            total += ft
            covered += fc

    print(f"== Changed-line coverage (vs {a.base}) ==")
    for relpath, fc, ft in per_file:
        print(f"  {fc:>4}/{ft:<4} {100 * fc / ft:5.1f}%  {relpath}")
    if total == 0:
        print("  (no instrumented changed lines — nothing to measure)")
        return 0
    pct = 100 * covered / total
    print("  ----")
    print(f"  TOTAL changed-line coverage: {covered}/{total} = {pct:.1f}%")

    if a.min is not None and pct < a.min:
        print(f"::error::changed-line coverage {pct:.1f}% is below the {a.min}% floor")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
