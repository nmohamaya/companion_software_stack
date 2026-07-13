#!/bin/bash
# capture_planner_bad_run.sh
#
# Runs the perception-avoidance Gazebo scenario in a loop until it catches a
# "bad" run — one where the D*Lite planner hit a no-path/hover (Issue #821) —
# then auto-dumps the [PlannerDiag] counters and the per-tick "No path"
# provenance so the deciding data is on screen without eyeballing every run.
#
# WHY: the #821 return-leg seal is intermittent (~1 in 5 recent runs). The
# instrumented no-path line (PR #822) carries `shadow_astar_pts` (did a full A*
# find a path? → false-negative D*Lite could recover) and the `static`/`dynamic`
# occupied-cell split (real promoted wall vs stale incremental state). Those two
# fields deterministically pick the fix:
#   - shadow_astar_pts > 0 on most no-path ticks  → A*-fallback-on-Exit-A fixes it
#   - shadow_astar_pts = 0 with static high        → real over-promotion (upstream)
#
# Usage:
#   ./tools/capture_planner_bad_run.sh [max_runs]      # default 10
#   MAX_RUNS=6 ./tools/capture_planner_bad_run.sh
#
# Stops on the FIRST bad run (exit 0) or after max_runs all-healthy (exit 3).
# Ctrl-C between runs is safe — the scenario runner cleans up its own processes.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RUNNER="${PROJECT_DIR}/tests/run_scenario_gazebo.sh"
SCENARIO="${PROJECT_DIR}/config/scenarios/18_perception_avoidance.json"
RESULTS_DIR="${PROJECT_DIR}/drone_logs/scenarios_gazebo/perception_avoidance"

MAX_RUNS="${1:-${MAX_RUNS:-10}}"

if [[ ! -x "$RUNNER" ]]; then
    echo "ERROR: scenario runner not found/executable: $RUNNER" >&2
    exit 2
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  #821 bad-run capture — scenario 18, up to ${MAX_RUNS} runs"
echo "  Stops on the first run with a D*Lite no-path (hover) event."
echo "═══════════════════════════════════════════════════════════════"

for ((i = 1; i <= MAX_RUNS; i++)); do
    echo ""
    echo "───── run ${i}/${MAX_RUNS} — launching scenario 18 ─────"
    # The runner manages its own PX4/Gazebo/companion lifecycle + cleanup.
    "$RUNNER" "$SCENARIO" >/dev/null 2>&1
    run_rc=$?

    # Newest completed run dir (PASS or FAIL — a bad run can still PASS if the
    # drone eventually gets through the seal).
    latest="$(ls -dt "${RESULTS_DIR}"/2026-*_PASS "${RESULTS_DIR}"/2026-*_FAIL 2>/dev/null | head -1)"
    mp="${latest}/mission_planner.log"
    if [[ ! -f "$mp" ]]; then
        echo "  run ${i}: no mission_planner.log found (rc=${run_rc}) — skipping"
        continue
    fi

    # Final [PlannerDiag] line carries the end-of-flight totals.
    diag="$(grep -hE '\[PlannerDiag\]' "$mp" | tail -1)"
    no_path="$(sed -nE 's/.*no_path=([0-9]+).*/\1/p' <<<"$diag")"
    no_path="${no_path:-0}"

    if [[ "$no_path" -gt 0 ]]; then
        echo ""
        echo "╔═════════════════════════════════════════════════════════════╗"
        echo "║  BAD RUN CAPTURED on attempt ${i} — no_path=${no_path}"
        echo "║  ${latest}"
        echo "╚═════════════════════════════════════════════════════════════╝"
        echo ""
        echo "── [PlannerDiag] end-of-flight totals ──"
        echo "  ${diag#*] }"
        echo ""
        echo "── per-tick 'No path' provenance (static / dynamic / shadow_astar_pts) ──"
        grep -hE '\[D\*Lite\] No path:' "$mp" | sed -E 's/^.*\[D\*Lite\] /  /' | head -40

        # Discriminator summary: how many no-path ticks were A*-recoverable.
        total="$(grep -hcE '\[D\*Lite\] No path:' "$mp")"
        recov="$(grep -hoE 'shadow_astar_pts=[0-9]+' "$mp" | grep -vE '=0$' | wc -l)"
        echo ""
        echo "── DISCRIMINATOR ──"
        echo "  no-path ticks logged            : ${total}"
        echo "  ...with shadow_astar_pts > 0    : ${recov}   (A* WOULD have found a path)"
        if [[ "${total:-0}" -gt 0 && "${recov:-0}" -gt 0 ]]; then
            pct=$(( recov * 100 / total ))
            echo "  A*-recoverable share            : ${pct}%"
            if [[ "$pct" -ge 50 ]]; then
                echo "  ⇒ VERDICT: incremental staleness — Exit-A A* fallback fixes it."
            else
                echo "  ⇒ VERDICT: mixed — inspect static= on the shadow_astar_pts=0 ticks (real wall?)."
            fi
        else
            echo "  ⇒ VERDICT: shadow_astar_pts=0 on all ticks — A* also fails."
            echo "     Inspect static= vs dynamic= above: high static ⇒ real over-promotion (upstream fix)."
        fi
        echo ""
        echo "Paste the block above back to converge Phase 2."
        exit 0
    fi

    echo "  run ${i}: healthy (no_path=0). ${diag:+[$diag]}"
done

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  ${MAX_RUNS} runs, all healthy (no_path=0) — the pathology is rarer"
echo "  than ~1-in-5. Re-run with a higher max, e.g. MAX_RUNS=20."
echo "═══════════════════════════════════════════════════════════════"
exit 3
