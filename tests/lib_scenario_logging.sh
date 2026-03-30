#!/usr/bin/env bash
# tests/lib_scenario_logging.sh
# ═══════════════════════════════════════════════════════════════
# Shared library for scenario test logging, reporting, and history.
#
# Provides:
#   create_run_dir()        — Timestamped run directory (never overwrites)
#   finalize_run_dir()      — Rename _RUNNING → _PASS / _FAIL
#   update_latest_symlink() — Atomic 'latest' symlink
#   write_run_metadata()    — Machine-readable run_metadata.json
#   generate_run_report()   — Human-readable run_report.txt with observations
#   append_to_index()       — Append to runs.jsonl history
#
# Usage: source "${SCRIPT_DIR}/lib_scenario_logging.sh"
# ═══════════════════════════════════════════════════════════════

# ── Timestamped run directory ────────────────────────────────
# Creates: <log_dir>/<scenario_name>/YYYY-MM-DD_HHMMSS[_<pid>]_RUNNING/
# Returns the path on stdout.
create_run_dir() {
    local log_dir="$1"
    local scenario_name="$2"
    local timestamp
    timestamp=$(date +"%Y-%m-%d_%H%M%S")
    local run_dir="${log_dir}/${scenario_name}/${timestamp}_RUNNING"
    # Handle same-second collision by inserting PID before _RUNNING
    if [[ -d "$run_dir" ]]; then
        run_dir="${log_dir}/${scenario_name}/${timestamp}_$$_RUNNING"
    fi
    mkdir -p "$run_dir"
    echo "$run_dir"
}

# ── Finalize run directory ───────────────────────────────────
# Renames _RUNNING suffix to _PASS or _FAIL.  Returns new path.
finalize_run_dir() {
    local run_dir="$1"
    local pass_count="$2"
    local fail_count="$3"
    local result
    if [[ "$fail_count" -gt 0 ]]; then
        result="FAIL"
    else
        result="PASS"
    fi
    local final_dir="${run_dir%_RUNNING}_${result}"
    if [[ "$run_dir" == *_RUNNING* ]]; then
        mv "$run_dir" "$final_dir" 2>/dev/null || final_dir="$run_dir"
    else
        final_dir="$run_dir"
    fi
    echo "$final_dir"
}

# ── Abort handler for cleanup traps ──────────────────────────
# Call from cleanup trap if run_dir still ends in _RUNNING.
abort_run_dir() {
    local run_dir="$1"
    if [[ "$run_dir" == *_RUNNING ]]; then
        local aborted_dir="${run_dir%_RUNNING}_ABORTED"
        mv "$run_dir" "$aborted_dir" 2>/dev/null || true
        echo "$aborted_dir"
    else
        echo "$run_dir"
    fi
}

# ── Update 'latest' symlink ─────────────────────────────────
# Atomic via ln -sfn (creates temp + rename).
update_latest_symlink() {
    local scenario_dir="$1"   # e.g. .../scenarios_gazebo/perception_avoidance
    local run_basename="$2"   # e.g. 2026-03-29_183556_PASS
    ln -sfn "$run_basename" "${scenario_dir}/latest"
}

# ── Write run metadata JSON ──────────────────────────────────
write_run_metadata() {
    local output_file="$1"
    local scenario_name="$2"
    local runner_type="$3"        # "gazebo" or "companion"
    local scenario_file="$4"
    local start_epoch="$5"
    local end_epoch="$6"
    local pass_count="$7"
    local fail_count="$8"
    local total_count="$9"
    local project_dir="${10}"

    local duration_s=$(( end_epoch - start_epoch ))
    local result="PASS"
    [[ "$fail_count" -gt 0 ]] && result="FAIL"

    local git_commit git_branch git_dirty hostname_str
    git_commit=$(git -C "$project_dir" rev-parse --short HEAD 2>/dev/null || echo "unknown")
    git_branch=$(git -C "$project_dir" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    git_dirty=$(git -C "$project_dir" diff --quiet 2>/dev/null && echo "false" || echo "true")
    hostname_str=$(hostname 2>/dev/null || echo "unknown")

    local config_hash
    config_hash=$(sha256sum "$scenario_file" 2>/dev/null | cut -c1-12 || echo "unknown")

    python3 - "$output_file" "$scenario_name" "$runner_type" "$result" \
              "$duration_s" "$pass_count" "$fail_count" "$total_count" \
              "$git_commit" "$git_branch" "$git_dirty" "$config_hash" \
              "$hostname_str" "$start_epoch" "$end_epoch" <<'PYEOF'
import json, sys, datetime
args = sys.argv[1:]
start_dt = datetime.datetime.fromtimestamp(int(args[13]), tz=datetime.timezone.utc)
end_dt = datetime.datetime.fromtimestamp(int(args[14]), tz=datetime.timezone.utc)
meta = {
    "timestamp": start_dt.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "scenario": args[1],
    "runner": args[2],
    "result": args[3],
    "duration_s": int(args[4]),
    "pass": int(args[5]),
    "fail": int(args[6]),
    "total": int(args[7]),
    "git_commit": args[8],
    "git_branch": args[9],
    "git_dirty": args[10] == "true",
    "config_hash": args[11],
    "hostname": args[12],
    "start_time": start_dt.isoformat(),
    "end_time": end_dt.isoformat(),
}
with open(args[0], 'w') as f:
    json.dump(meta, f, indent=2)
PYEOF
}

# ── Append to index ──────────────────────────────────────────
# Appends one JSON line to runs.jsonl (the run_dir relative path is added).
append_to_index() {
    local index_file="$1"
    local metadata_file="$2"
    local run_dir_relative="$3"   # e.g. perception_avoidance/2026-03-29_183556_PASS
    python3 - "$metadata_file" "$run_dir_relative" "$index_file" <<'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    meta = json.load(f)
meta["run_dir"] = sys.argv[2]
with open(sys.argv[3], 'a') as f:
    f.write(json.dumps(meta) + "\n")
PYEOF
}

# ═══════════════════════════════════════════════════════════════
# Report generation
# ═══════════════════════════════════════════════════════════════

generate_run_report() {
    local run_dir="$1"
    local scenario_name="$2"
    local runner_type="$3"        # "gazebo" or "companion"
    local pass_count="$4"
    local fail_count="$5"
    local total_count="$6"
    local scenario_file="$7"
    local project_dir="$8"

    local report="${run_dir}/run_report.txt"
    local mp_log="${run_dir}/mission_planner.log"
    local perc_log="${run_dir}/perception.log"
    local combined_log="${run_dir}/combined.log"

    # If individual logs don't exist, fall back to combined or launcher
    if [[ ! -f "$mp_log" ]]; then
        mp_log="${run_dir}/launcher.log"
    fi
    if [[ ! -f "$perc_log" ]]; then
        perc_log="${run_dir}/launcher.log"
    fi
    if [[ ! -f "$combined_log" ]]; then
        combined_log="${run_dir}/launcher.log"
    fi

    local result="PASS"
    [[ "$fail_count" -gt 0 ]] && result="FAIL"

    local git_commit git_branch hostname_str
    git_commit=$(git -C "$project_dir" rev-parse --short HEAD 2>/dev/null || echo "unknown")
    git_branch=$(git -C "$project_dir" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    hostname_str=$(hostname 2>/dev/null || echo "unknown")

    local run_timestamp
    run_timestamp=$(basename "$run_dir" | sed 's/_[A-Z]*$//' | sed 's/_/ /' | sed 's/\([0-9]\{2\}\)\([0-9]\{2\}\)\([0-9]\{2\}\)$/\1:\2:\3/')

    {
        echo "══════════════════════════════════════════════════════════"
        echo "  SCENARIO RUN REPORT"
        echo "══════════════════════════════════════════════════════════"
        echo ""

        # ── Run Metadata ──
        echo "Run Metadata"
        echo "  Scenario  : ${scenario_name}"
        echo "  Runner    : ${runner_type}"
        echo "  Timestamp : ${run_timestamp}"
        echo "  Result    : ${result} (${pass_count}/${total_count} checks)"
        echo "  Git       : ${git_commit} (${git_branch})"
        echo "  Hostname  : ${hostname_str}"
        echo ""

        # ── Mission FSM Transitions ──
        _report_fsm_transitions "$mp_log"

        # ── Survey Quality ──
        _report_survey_quality "$mp_log"

        # ── Waypoint Progress ──
        _report_waypoint_progress "$mp_log"

        # ── Obstacle Proximity ──
        _report_obstacle_proximity "$mp_log"

        # ── Occupancy Grid Peaks ──
        _report_grid_peaks "$mp_log"

        # ── Perception ──
        _report_perception "$perc_log"

        # ── Fault Events ──
        _report_fault_events "$mp_log"

        # ── Avoider Activity ──
        _report_avoider_activity "$mp_log"

        # ── D* Lite Planner ──
        _report_planner_stats "$mp_log"

        # ── Verification Checks ──
        _report_verification_checks "$scenario_file" "$combined_log"

        # ── Overall Assessment ──
        _report_overall_assessment "$mp_log" "$perc_log" "$result" "$pass_count" "$total_count"

        echo "══════════════════════════════════════════════════════════"
    } > "$report"

    echo "$report"
}

# ═══════════════════════════════════════════════════════════════
# Report section helpers
# ═══════════════════════════════════════════════════════════════

_report_fsm_transitions() {
    local log="$1"
    echo "Mission FSM Transitions"

    local prev_ts=""
    local transitions
    transitions=$(grep -a "\[FSM\].*→\|Advanced to waypoint" "$log" 2>/dev/null | sort -t']' -k1,1 -u || true)

    if [[ -z "$transitions" ]]; then
        echo "  (no FSM transitions found in log)"
        echo ""
        return
    fi

    local total_wps=0 reached_wps=0 mission_complete=false loiter_seen=false rtl_seen=false
    total_wps=$(grep -ac "waypoints loaded\|navigating.*waypoints" "$log" 2>/dev/null | head -1)
    if [[ -z "$total_wps" || "$total_wps" == "0" ]]; then
        # Try to extract from "navigating N waypoints" or "N waypoints loaded"
        total_wps=$(grep -aoP '\d+ waypoints' "$log" 2>/dev/null | head -1 | grep -oP '\d+' || echo "?")
    fi

    while IFS= read -r line; do
        local ts
        ts=$(echo "$line" | grep -oP '\d{2}:\d{2}:\d{2}\.\d{3}' | head -1)
        local short_ts="${ts%.*}"  # HH:MM:SS without millis

        local delta=""
        if [[ -n "$prev_ts" && -n "$ts" ]]; then
            # Compute delta in seconds
            local prev_s cur_s
            prev_s=$(echo "$prev_ts" | awk -F'[:.]' '{print $1*3600+$2*60+$3+$4/1000}')
            cur_s=$(echo "$ts" | awk -F'[:.]' '{print $1*3600+$2*60+$3+$4/1000}')
            local diff
            diff=$(awk "BEGIN{printf \"%.0f\", $cur_s - $prev_s}")
            delta="(+${diff}s)"
        fi

        local event=""
        if echo "$line" | grep -q "→"; then
            event=$(echo "$line" | grep -oP '\[FSM\] .*' | sed 's/\[FSM\] //')
        elif echo "$line" | grep -q "Advanced to waypoint"; then
            event=$(echo "$line" | grep -oP 'Advanced to waypoint \d+/\d+')
            reached_wps=$((reached_wps + 1))
        fi

        printf "  %-10s %-40s %s\n" "$short_ts" "$event" "$delta"
        prev_ts="$ts"

        echo "$line" | grep -q "LOITER" && loiter_seen=true
        echo "$line" | grep -q "→ RTL" && rtl_seen=true
    done <<< "$transitions"

    # Check for mission complete
    grep -qa "Mission complete" "$log" 2>/dev/null && mission_complete=true

    echo ""
    echo "  Observations:"

    # Takeoff duration
    local takeoff_start takeoff_end
    takeoff_start=$(grep -a "PREFLIGHT.*TAKEOFF\|TAKEOFF" "$log" 2>/dev/null | head -1 | grep -oP '\d{2}:\d{2}:\d{2}\.\d{3}' | head -1)
    takeoff_end=$(grep -a "TAKEOFF.*SURVEY\|Takeoff complete" "$log" 2>/dev/null | head -1 | grep -oP '\d{2}:\d{2}:\d{2}\.\d{3}' | head -1)
    if [[ -n "$takeoff_start" && -n "$takeoff_end" ]]; then
        local ts_s te_s takeoff_dur
        ts_s=$(echo "$takeoff_start" | awk -F'[:.]' '{print $1*3600+$2*60+$3}')
        te_s=$(echo "$takeoff_end" | awk -F'[:.]' '{print $1*3600+$2*60+$3}')
        takeoff_dur=$((te_s - ts_s))
        if (( takeoff_dur < 8 )); then
            echo "  - Takeoff took ${takeoff_dur}s (fast, normal: 8-15s)"
        elif (( takeoff_dur <= 15 )); then
            echo "  - Takeoff took ${takeoff_dur}s (normal: 8-15s)"
        else
            echo "  - Takeoff took ${takeoff_dur}s (SLOW — normal: 8-15s, check PX4 boot)"
        fi
    fi

    if [[ "$mission_complete" == "true" ]]; then
        echo "  - Mission COMPLETED all waypoints successfully"
    else
        echo "  - Mission did NOT complete all waypoints — reached ${reached_wps} WPs"
        if [[ "$loiter_seen" == "true" ]]; then
            echo "  - LOITER triggered — likely fault escalation, not planning issue"
        fi
        if [[ "$rtl_seen" == "true" ]]; then
            echo "  - RTL activated — drone returned to home before finishing mission"
        fi
    fi
    echo ""
}

_report_survey_quality() {
    local log="$1"
    echo "Survey Quality"

    local survey_line
    survey_line=$(grep -a "Survey complete" "$log" 2>/dev/null | tail -1)

    if [[ -z "$survey_line" ]]; then
        echo "  (no survey data found)"
        echo ""
        return
    fi

    local static_cells promoted_cells survey_dur
    static_cells=$(echo "$survey_line" | grep -oP '\d+ static' | grep -oP '\d+')
    promoted_cells=$(echo "$survey_line" | grep -oP '\d+ promoted' | grep -oP '\d+')
    survey_dur=$(echo "$survey_line" | grep -oP '\(\d+s\)' | grep -oP '\d+')

    echo "  Duration       : ${survey_dur:-?}s"
    echo "  Static cells   : ${static_cells:-0}"
    echo "  Promoted cells : ${promoted_cells:-0}"
    echo ""
    echo "  Observations:"

    local sc="${static_cells:-0}"
    if (( sc < 200 )); then
        echo "  - ${sc} static cells is LOW (target: >400) — high risk of missing obstacles"
    elif (( sc < 400 )); then
        echo "  - ${sc} static cells is MODERATE (target: >400) — some obstacles may lack grid coverage"
    else
        echo "  - ${sc} static cells is GOOD — sufficient grid coverage for obstacle avoidance"
    fi

    if [[ -n "$promoted_cells" && -n "$static_cells" && "$static_cells" -gt 0 ]]; then
        if [[ "$promoted_cells" == "$static_cells" ]]; then
            echo "  - All static cells promoted (${promoted_cells}/${static_cells}) — no promotion bottleneck"
        else
            local ratio
            ratio=$(awk "BEGIN{printf \"%.0f\", 100*${promoted_cells}/${static_cells}}")
            echo "  - Promotion ratio: ${promoted_cells}/${static_cells} (${ratio}%) — check promotion_hits threshold"
        fi
    fi
    echo ""
}

_report_waypoint_progress() {
    local log="$1"
    echo "Waypoint Progress"

    local nav_start
    nav_start=$(grep -a "SURVEY.*NAVIGATE\|FSM.*NAVIGATE" "$log" 2>/dev/null | head -1 | grep -oP '\d{2}:\d{2}:\d{2}\.\d{3}' | head -1)

    local wp_lines
    wp_lines=$(grep -a "Advanced to waypoint" "$log" 2>/dev/null || true)

    if [[ -z "$wp_lines" ]]; then
        echo "  (no waypoint progress found)"
        echo ""
        return
    fi

    local prev_epoch nav_epoch=0 count=0 total_elapsed=0
    if [[ -n "$nav_start" ]]; then
        nav_epoch=$(echo "$nav_start" | awk -F'[:.]' '{print $1*3600+$2*60+$3+$4/1000}')
        prev_epoch=$nav_epoch
    fi

    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        count=$((count + 1))
        local ts wp_info
        ts=$(echo "$line" | grep -oP '\d{2}:\d{2}:\d{2}\.\d{3}' | head -1)
        wp_info=$(echo "$line" | grep -oP 'waypoint \d+/\d+')
        local short_ts="${ts%.*}"

        if [[ -n "$ts" && -n "$prev_epoch" ]]; then
            local cur_epoch
            cur_epoch=$(echo "$ts" | awk -F'[:.]' '{print $1*3600+$2*60+$3+$4/1000}')
            local leg_time cumul
            leg_time=$(awk "BEGIN{printf \"%.0f\", $cur_epoch - $prev_epoch}")
            cumul=$(awk "BEGIN{printf \"%.0f\", $cur_epoch - $nav_epoch}")
            total_elapsed=$cumul
            printf "  WP %-6s reached %-10s (+%ss, cumulative %ss)\n" "$wp_info" "$short_ts" "$leg_time" "$cumul"
            prev_epoch=$cur_epoch
        else
            printf "  WP %-6s reached %-10s\n" "$wp_info" "$short_ts"
        fi
    done <<< "$wp_lines"

    # Check if mission completed
    if ! grep -qa "Mission complete" "$log" 2>/dev/null; then
        echo "  (mission did not complete — remaining WPs not reached)"
    fi

    echo ""
    echo "  Observations:"
    if [[ $count -gt 0 && $total_elapsed -gt 0 ]]; then
        local avg_leg
        avg_leg=$(awk "BEGIN{printf \"%.1f\", $total_elapsed / $count}")
        echo "  - Average leg time: ${avg_leg}s across ${count} waypoint(s)"
    fi

    if grep -qa "Mission complete" "$log" 2>/dev/null; then
        echo "  - All waypoints reached — mission completed successfully"
    else
        echo "  - Mission incomplete — check fault events for cause"
    fi
    echo ""
}

_report_obstacle_proximity() {
    local log="$1"
    echo "Obstacle Proximity (from DIAG telemetry)"

    local diag_lines
    diag_lines=$(grep -a "\[DIAG\]" "$log" 2>/dev/null || true)

    if [[ -z "$diag_lines" ]]; then
        echo "  (no DIAG telemetry found — obstacle proximity not available)"
        echo ""
        return
    fi

    # Extract closest approach using awk
    # Known obstacle positions for perception_avoidance scenario:
    # RED (3,15), GREEN (10,10), BLUE (18,5), ORANGE (5,3)
    local proximity_data
    proximity_data=$(echo "$diag_lines" | grep -oP 'pos=\([^)]+\)' | awk -F'[(),]' '
    BEGIN {
        mr=999; mg=999; mb=999; mo=999
        names[0]="RED"; ox[0]=3; oy[0]=15
        names[1]="GREEN"; ox[1]=10; oy[1]=10
        names[2]="BLUE"; ox[2]=18; oy[2]=5
        names[3]="ORANGE"; ox[3]=5; oy[3]=3
    }
    {
        x=$2; y=$3
        dr=sqrt((x-3)^2+(y-15)^2); if(dr<mr){mr=dr; mrp=x","y}
        dg=sqrt((x-10)^2+(y-10)^2); if(dg<mg){mg=dg; mgp=x","y}
        db=sqrt((x-18)^2+(y-5)^2); if(db<mb){mb=db; mbp=x","y}
        d4=sqrt((x-5)^2+(y-3)^2); if(d4<mo){mo=d4; mop=x","y}
    }
    END {
        printf "RED|3,15|%.1f|%s\n", mr, mrp
        printf "GREEN|10,10|%.1f|%s\n", mg, mgp
        printf "BLUE|18,5|%.1f|%s\n", mb, mbp
        printf "ORANGE|5,3|%.1f|%s\n", mo, mop
    }')

    if [[ -z "$proximity_data" ]]; then
        echo "  (could not parse obstacle proximity from DIAG data)"
        echo ""
        return
    fi

    local min_dist=999 closest_name=""
    while IFS='|' read -r name coords dist pos; do
        [[ -z "$name" ]] && continue
        dist=$(echo "$dist" | tr -d '[:space:]')
        printf "  %-8s (%s) : closest %sm at pos=(%s)\n" "$name" "$coords" "$dist" "$pos"
        if awk "BEGIN{exit ($dist < $min_dist) ? 0 : 1}" 2>/dev/null; then
            min_dist="$dist"
            closest_name="$name"
        fi
    done <<< "$proximity_data"

    echo ""
    echo "  Observations:"

    # Assess each obstacle
    local collision_risk=false
    while IFS='|' read -r name coords dist pos; do
        [[ -z "$name" ]] && continue
        dist=$(echo "$dist" | tr -d '[:space:]')
        if awk "BEGIN{exit ($dist < 1.5) ? 0 : 1}" 2>/dev/null; then
            echo "  - ${name} at ${dist}m — COLLISION RISK (< 1.5m safety radius)"
            collision_risk=true
        elif awk "BEGIN{exit ($dist < 3.0) ? 0 : 1}" 2>/dev/null; then
            echo "  - ${name} at ${dist}m — CLOSE approach (< 3.0m, within inflation buffer)"
        fi
    done <<< "$proximity_data"

    if [[ "$collision_risk" == "false" ]]; then
        echo "  - All obstacles cleared by >1.5m — no collision risk"
    fi
    echo "  - Closest approach: ${closest_name} at ${min_dist}m"
    echo ""
}

_report_grid_peaks() {
    local log="$1"
    echo "Occupancy Grid Peaks"

    local peaks
    peaks=$(grep -a "\[DIAG\]" "$log" 2>/dev/null | awk -F'[= ]' '
    BEGIN { max_occ=0; max_static=0; max_promoted=0 }
    {
        for(i=1;i<=NF;i++) {
            if($i=="occ" && $(i+1)+0>max_occ) max_occ=$(i+1)+0
            if($i=="static" && $(i+1)+0>max_static) max_static=$(i+1)+0
            if($i=="promoted" && $(i+1)+0>max_promoted) max_promoted=$(i+1)+0
        }
    }
    END { printf "%d|%d|%d\n", max_occ, max_static, max_promoted }
    ' 2>/dev/null || echo "0|0|0")

    local max_occ max_static max_promoted
    IFS='|' read -r max_occ max_static max_promoted <<< "$peaks"

    echo "  Max occupied  : ${max_occ}"
    echo "  Max static    : ${max_static}"
    echo "  Max promoted  : ${max_promoted}"
    echo ""
    echo "  Observations:"

    if (( max_occ > 800 )); then
        echo "  - Occupied peaked at ${max_occ} — dense detection burst (may cause flight dynamics issues)"
    elif (( max_occ > 400 )); then
        echo "  - Occupied peaked at ${max_occ} — moderate detection density"
    else
        echo "  - Occupied peaked at ${max_occ} — light detection load"
    fi

    if (( max_static > 600 )); then
        echo "  - ${max_static} static cells — possible over-promotion (grid bloat)"
    elif (( max_static > 300 )); then
        echo "  - ${max_static} static cells — healthy range for obstacle avoidance"
    else
        echo "  - ${max_static} static cells — sparse, may have coverage gaps"
    fi

    # Check growth from survey to end
    local survey_static
    survey_static=$(grep -a "Survey complete" "$log" 2>/dev/null | tail -1 | grep -oP '\d+ static' | grep -oP '\d+' || echo "0")
    if [[ "$survey_static" -gt 0 && "$max_static" -gt "$survey_static" ]]; then
        local growth_pct
        growth_pct=$(awk "BEGIN{printf \"%.0f\", 100*($max_static - $survey_static)/$survey_static}")
        echo "  - Static grew from ${survey_static} (survey) → ${max_static} (nav) (+${growth_pct}%) — in-flight promotion active"
    fi
    echo ""
}

_report_perception() {
    local log="$1"
    echo "Perception (Radar-Primary)"

    local radar_only_count radar_reids camera_reids adoption_count
    radar_only_count=$(grep -a "New obstacle (radar-only)" "$log" 2>/dev/null | wc -l)
    radar_reids=$(grep -a "Re-ID (radar-only)" "$log" 2>/dev/null | wc -l)
    camera_reids=$(grep -a "Re-ID:" "$log" 2>/dev/null | grep -v "radar-only" | wc -l)
    adoption_count=$(grep -a "Adopted\|adopted\|adopt_camera" "$log" 2>/dev/null | wc -l)

    echo "  Radar-only tracks created : ${radar_only_count}"
    echo "  Radar-only dormant re-IDs : ${radar_reids}"
    echo "  Camera dormant re-IDs     : ${camera_reids}"
    echo "  Camera adoptions          : ${adoption_count}"
    echo ""
    echo "  Observations:"

    if (( radar_only_count > 0 )); then
        echo "  - Radar independently created ${radar_only_count} tracks — radar-primary architecture active"
    else
        echo "  - No radar-only tracks — radar may not be enabled or no unmatched detections"
    fi

    if (( radar_reids > 100 )); then
        echo "  - High re-ID count (${radar_reids}) — radar tracks are short-lived but consistently"
        echo "    re-detected at same world positions (dormant pool working as intended)"
    elif (( radar_reids > 0 )); then
        echo "  - ${radar_reids} radar re-IDs — dormant pool providing continuity for radar tracks"
    fi

    if (( adoption_count > 0 )); then
        echo "  - ${adoption_count} camera adoption(s) — camera refined radar-only tracks with bearing/class"
    else
        echo "  - No camera adoptions — radar tracks are in regions camera doesn't cover, or timing mismatch"
    fi
    echo ""
}

_report_fault_events() {
    local log="$1"
    echo "Fault Events"

    local faults
    faults=$(grep -a "Escalation" "$log" 2>/dev/null | sort -t']' -k1,1 -u || true)

    if [[ -z "$faults" ]]; then
        echo "  (no fault escalations during this run)"
        echo ""
        return
    fi

    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        local ts event
        ts=$(echo "$line" | grep -oP '\d{2}:\d{2}:\d{2}' | head -1)
        event=$(echo "$line" | grep -oP 'Escalation:.*' | sed 's/active_faults=.*//')
        echo "  ${ts}  ${event}"
    done <<< "$faults"

    echo ""
    echo "  Observations:"

    if echo "$faults" | grep -q "pose.*stale\|POSE_STALE"; then
        echo "  - Pose staleness detected — VIO pipeline issue (common in Gazebo SITL)"
        echo "  - This is typically a SIMULATION ARTIFACT, not a code bug"
    fi
    if echo "$faults" | grep -q "LOITER.*RTL\|loiter.*timeout"; then
        echo "  - LOITER→RTL escalation — loiter timeout triggered (expected fault recovery behavior)"
    fi
    if echo "$faults" | grep -q "EMERGENCY"; then
        echo "  - EMERGENCY escalation — CRITICAL: investigate root cause immediately"
    fi
    if echo "$faults" | grep -q "battery\|BATT"; then
        echo "  - Battery fault — check simulated battery drain rate"
    fi
    echo ""
}

_report_avoider_activity() {
    local log="$1"
    echo "Avoider Activity"

    local avoider_data
    avoider_data=$(grep -a "\[DIAG\]" "$log" 2>/dev/null | grep -oP '\|delta\|=[0-9.]+' | sed 's/|delta|=//' || true)

    if [[ -z "$avoider_data" ]]; then
        echo "  (no avoider data in DIAG telemetry)"
        echo ""
        return
    fi

    local stats
    stats=$(echo "$avoider_data" | awk '
    BEGIN { max=0; sum=0; n=0 }
    { n++; sum+=$1; if($1>max) max=$1 }
    END { printf "%.2f|%.2f|%d\n", max, (n>0?sum/n:0), n }
    ')

    local max_delta avg_delta count
    IFS='|' read -r max_delta avg_delta count <<< "$stats"

    local fallback_count
    fallback_count=$(grep -a "fallback=true" "$log" 2>/dev/null | wc -l)

    echo "  Max |delta|   : ${max_delta}"
    echo "  Avg |delta|   : ${avg_delta}"
    echo "  Fallback used : ${fallback_count} time(s)"
    echo ""
    echo "  Observations:"

    max_delta=$(echo "$max_delta" | tr -d '[:space:]')
    avg_delta=$(echo "$avg_delta" | tr -d '[:space:]')

    if awk "BEGIN{exit ($max_delta > 1.0) ? 0 : 1}" 2>/dev/null; then
        echo "  - Max avoider correction ${max_delta} m/s — HIGH (near saturation at 1.5 m/s max)"
    elif awk "BEGIN{exit ($max_delta > 0.3) ? 0 : 1}" 2>/dev/null; then
        echo "  - Max avoider correction ${max_delta} m/s — moderate, within normal range"
    else
        echo "  - Max avoider correction ${max_delta} m/s — minimal, paths already clear"
    fi

    if awk "BEGIN{exit ($avg_delta < 0.1) ? 0 : 1}" 2>/dev/null; then
        echo "  - Average delta ${avg_delta} — avoider mostly dormant (D* Lite paths are obstacle-free)"
    fi

    if (( fallback_count > 0 )); then
        echo "  - WARNING: Fallback triggered ${fallback_count} time(s) — D* Lite failed to find paths"
    else
        echo "  - No fallback triggers — D* Lite always found valid paths"
    fi
    echo ""
}

_report_planner_stats() {
    local log="$1"
    echo "D* Lite Planner"

    local replan_count max_search
    replan_count=$(grep -a "\[PlanBase\] Replan" "$log" 2>/dev/null | wc -l)
    max_search=$(grep -a "\[PlanBase\] Replan\|\[D\*Lite\]" "$log" 2>/dev/null \
        | grep -oP 'search=\d+ms' | grep -oP '\d+' \
        | sort -n | tail -1 || echo "0")
    max_search="${max_search:-0}"

    local path_failures
    path_failures=$(grep -a "Path FAILED\|path failed\|No path found" "$log" 2>/dev/null | wc -l)

    echo "  Total replans    : ${replan_count}"
    echo "  Max search time  : ${max_search}ms"
    echo "  Path failures    : ${path_failures}"
    echo ""
    echo "  Observations:"

    if (( max_search > 100 )); then
        echo "  - Max search time ${max_search}ms — ELEVATED (budget: 1500ms), grid may be dense"
    elif (( max_search > 10 )); then
        echo "  - Max search time ${max_search}ms — within normal range"
    else
        echo "  - Max search time ${max_search}ms — fast, no planning bottleneck"
    fi

    if (( path_failures > 0 )); then
        echo "  - WARNING: ${path_failures} path failure(s) — grid coverage may have gaps or obstacles block all routes"
    else
        echo "  - Zero path failures — sufficient grid coverage for all legs"
    fi
    echo ""
}

_report_verification_checks() {
    local scenario_file="$1"
    local combined_log="$2"
    echo "Verification Checks"

    # Requires json_get_array() to be defined (provided by the runner scripts)
    if ! type json_get_array &>/dev/null; then
        echo "  (json_get_array not available — skipping verification replay)"
        echo ""
        return
    fi

    # log_contains
    while read -r pattern; do
        [[ -z "$pattern" ]] && continue
        if grep -qai "$pattern" "$combined_log" 2>/dev/null; then
            echo "  [PASS] Log contains: ${pattern}"
        else
            echo "  [FAIL] Log contains: ${pattern}"
        fi
    done < <(json_get_array "$scenario_file" "pass_criteria.log_contains" 2>/dev/null)

    # log_must_not_contain
    while read -r pattern; do
        [[ -z "$pattern" ]] && continue
        if grep -qai "$pattern" "$combined_log" 2>/dev/null; then
            echo "  [FAIL] Log must NOT contain: ${pattern}"
        else
            echo "  [PASS] Log does NOT contain: ${pattern}"
        fi
    done < <(json_get_array "$scenario_file" "pass_criteria.log_must_not_contain" 2>/dev/null)

    echo ""
}

_report_overall_assessment() {
    local mp_log="$1"
    local perc_log="$2"
    local result="$3"
    local pass_count="$4"
    local total_count="$5"

    echo "Overall Assessment"

    local mission_complete=false collision=false
    grep -qa "Mission complete" "$mp_log" 2>/dev/null && mission_complete=true
    grep -qa "COLLISION\|collision" "$mp_log" 2>/dev/null && collision=true

    local fault_cause=""
    if grep -qa "pose.*stale\|POSE_STALE" "$mp_log" 2>/dev/null; then
        fault_cause="VIO pose staleness"
    elif grep -qa "battery\|BATT" "$mp_log" 2>/dev/null; then
        fault_cause="battery fault"
    elif grep -qa "fc_lost\|FC_LOST" "$mp_log" 2>/dev/null; then
        fault_cause="FC link loss"
    fi

    local radar_only_count
    radar_only_count=$(grep -a "New obstacle (radar-only)" "$perc_log" 2>/dev/null | wc -l)

    if [[ "$collision" == "true" ]]; then
        echo "  COLLISION DETECTED — investigate obstacle proximity and grid coverage."
    elif [[ "$mission_complete" == "true" && "$result" == "PASS" ]]; then
        echo "  Mission completed successfully with all checks passing."
    elif [[ "$mission_complete" == "true" && "$result" == "FAIL" ]]; then
        echo "  Mission completed but ${pass_count}/${total_count} checks passed — review failed checks above."
    elif [[ "$result" == "FAIL" && -n "$fault_cause" ]]; then
        echo "  Mission did not complete — ${fault_cause} triggered fault escalation."
        if [[ "$fault_cause" == "VIO pose staleness" ]]; then
            echo "  This is typically a Gazebo SITL timing artifact, not a code defect."
        fi
        echo "  Recommend re-running to confirm consistent behavior."
    else
        echo "  Mission did not complete — ${pass_count}/${total_count} checks passed."
        echo "  Review fault events and FSM transitions above for root cause."
    fi

    if (( radar_only_count > 0 )); then
        echo "  Radar-primary architecture active: ${radar_only_count} independent radar tracks created."
    fi
    echo ""
}
