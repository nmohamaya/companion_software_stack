# Debug Logging & Assessment for Issue #237 — Smoother Flight Path Tuning

Diagnostic logging, Gazebo assessment methodology, code changes, and root cause analyses
from Epic #237 (perception-driven obstacle avoidance with D* Lite in Scenario 18).

---

## Assessment Methodology

### Gazebo Scenario 18 Test Protocol

Each run follows this sequence:
1. `deploy/clean_build_and_run.sh` or `cmake --build build` → build from integration branch
2. `tests/run_scenario_gazebo.sh config/scenarios/18_perception_avoidance.json --gui`
3. Logs saved to `drone_logs/scenarios_gazebo/perception_avoidance/runN/`
4. Post-run analysis of `perception.log` and `mission_planner.log`

### Log Analysis Commands

```bash
# Check mission outcome
grep -a "COLLISION\|EMERGENCY\|Mission complete" mission_planner.log

# Survey quality — static cell count at survey end
grep -a "Survey complete" mission_planner.log

# Full flight DIAG trace — position, velocity, grid state
grep -a "DIAG" mission_planner.log

# Dormant obstacle tracking
grep -a "New obstacle\|Re-ID" perception.log

# Radar initialization events
grep -a "Radar-init" perception.log

# Altitude drops below threshold
grep -a "DIAG" mission_planner.log | awk -F',' '{print $3}' | grep -v "4\."

# Closest approach to each obstacle (manual: check DIAG pos near known coords)
# RED (3,15), GREEN (10,10), BLUE (18,5), ORANGE (5,3)
```

---

## Diagnostic Logging Points

### `mission_state_tick.h` — Full navigate diagnostic

| Location | Tag | What it logs |
|----------|-----|--------------|
| `tick_navigate()` ~line 295 | `[DIAG]` | Every tick (~1s): drone world position, current WP target + distance, planner velocity, post-avoider velocity, avoider delta magnitude, grid counts (occupied/static/promoted), fallback flag |

**Format:**
```
[DIAG] pos=(x,y,z) wp2/5=(12,18,4) dist=14.2m plan_v=(0.5,1.2,0.0)
       avoid_v=(0.3,1.1,0.0) |delta|=0.22 grid: occ=5 static=12 promoted=12 fallback=false
```

### `ukf_fusion_engine.cpp` — Perception diagnostics

| Location | Tag | What it logs |
|----------|-----|--------------|
| Constructor | `[UKF]` | radar_enabled, ground_filter, altitude_gate, gate_threshold, dormant config |
| Radar-init block | `[UKF] Radar-init` | Track ID, camera depth estimate vs radar range override |
| Dormant creation | `[UKF] New obstacle` | Track ID → dormant # with world position |
| Dormant re-ID | `[UKF] Re-ID` | Track re-identified to existing dormant, position, observation count, distance |
| Late radar entry | `[UKF] New obstacle (late radar)` | Track gained radar after creation, now enters dormant pool |
| Survey complete | `[Planner]` | Static cell count and promoted count at survey end |

### `mission_fsm.h` — Mission state transitions

| Location | Tag | What it logs |
|----------|-----|--------------|
| State transitions | `[FSM]` | TAKEOFF→SURVEY→NAVIGATE→RTL transitions |
| WP advance | `[FSM]` | Waypoint advancement with index |
| WP snap | `[Planner]` | Occupied waypoint snapped to nearest free cell |

---

## Code Changes Made (Epic #237)

### Phase A: Survey-Phase Obstacle Mapping

#### A1: Bearing-Only Camera Init + Radar Range Snap
**File:** `process2_perception/src/ukf_fusion_engine.cpp`

- Camera-only new tracks get `P(0,0) = 100.0` (high depth uncertainty)
- Radar-init bearing gate (0.15 rad ≈ 8.6°) matches radar to camera tracks
- `set_radar_confirmed_depth(radar_range)` rescales state and sets `P(0,0) = 0.5`
- `radar_init_used` flag tracks whether track was radar-confirmed at creation

#### A2: Size Estimation from Camera Bbox + Radar Range
**Files:** `perception/types.h`, `ukf_fusion_engine.cpp`, `occupancy_grid_3d.h`, `ipc_types.h`, `main.cpp`

- `estimated_radius_m = bbox_w * range / (2 * fx)`
- `estimated_height_m = bbox_h * range / fy`
- Only computed when range > 1m and bbox > 5px
- Grid uses per-object radius for inflation instead of fixed `inflation_cells_`

#### A3: Covariance-Aware Dormant Merge
**Files:** `ukf_fusion_engine.h`, `ukf_fusion_engine.cpp`

- `find_nearest_dormant()` now accepts position covariance matrix
- Dynamic merge radius = `2σ_max` clamped to [1.0, 5.0]m
- Radar-confirmed tracks (P(0,0)=0.5) → tight merge radius ~1.4m
- Camera-only tracks (P(0,0)=100) → large radius, prevents premature merge

### Phase B: In-Flight Obstacle Handling

#### B1: Radar-Confirmed Static Promotion
**File:** `occupancy_grid_3d.h`

- Objects with `radar_update_count >= 3` get immediate static grid promotion
- Bypasses hit-count accumulation — radar-confirmed = real obstacle
- `radar_update_count` propagated through `FusedObject → DetectedObject → IPC`

### Phase C: Planner & Avoider Fixes

#### C1: Z-Leak Fix (pre-existing on branch)
**File:** `obstacle_avoider_3d.h`
- 2D path-aware stripping when `vertical_gain=0` prevents Z injection from XY repulsion

#### C2: Backward Path Rejection (pre-existing on branch)
**File:** `grid_planner_base.h`
- D* Lite paths with `dot < 0` vs goal direction are rejected, keeping cached forward path

#### C3: Config Tuning
**File:** `config/scenarios/18_perception_avoidance.json`
- `smoothing_alpha`: 0.35 → 0.5 (faster reaction)
- `replan_interval_s`: 1.0 → 0.5 (more frequent replans)

### Dormant Pool Pollution Fix
**File:** `ukf_fusion_engine.cpp`

**Problem discovered:** Camera-only tracks with 40m default depth created phantom dormant entries at 25-60m from origin, filling the 32-entry pool before real obstacles got proper entries.

**Fix (3 changes):**
1. New track dormant creation gated on `radar_init_used` — camera-only tracks excluded
2. Existing track dormant updates gated on `radar_update_count > 0`
3. Added "late radar entry" — when existing track gets first radar update, it can now create/re-ID a dormant entry

---

## Gazebo Run Results

### Run 1 (2026-03-29) — PASS
- All 4 obstacles avoided
- ~499 static cells at survey end
- Smooth flight, no altitude issues

### Run 2 (2026-03-29) — PASS
- All 4 obstacles avoided
- ~456 static cells at survey end
- Clean flight path

### Run 3 (2026-03-29) — FAIL (RED collision)
- Drone flew through RED (3,15) at 0.86m from center (inside 1.0m radius)
- Only 316 static cells at survey end (low coverage)
- Altitude crash on WP4 leg: Z dropped from 4.0 to 0.1m over 3 seconds
- Root cause: dormant pool pollution (32 phantoms, no entry near RED)
- Dormant pool fix applied after this run

---

## Root Cause Analysis Summary

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| GREEN collision (pre-Run 1) | Camera couldn't see GREEN during approach; no radar-only track creation | Identified but not yet fixed — needs radar-primary architecture |
| RED collision (Run 3) | Dormant pool filled with 32 camera-only phantoms at wrong positions | Gated dormant entry on radar confirmation |
| Altitude crash (Run 3) | PX4 flight dynamics — rapid lateral corrections from dense obstacle cloud (occ=539) caused altitude loss; planning code NOT at fault | Flight dynamics issue, not code |
| Dormant position drift | Averaging dormant positions with camera-only depth estimates pulled positions away from truth | Gated dormant updates on radar_update_count > 0 |
| Low static cells (316 vs 499) | Fewer radar-init events = fewer radar-confirmed tracks = less grid coverage | Stochastic — depends on survey yaw timing vs radar scan timing |

---

## Remaining Architecture Gap

**Camera-primary limitation:** The current pipeline requires camera detection to create tracks. Radar detections that don't match a camera track are discarded. This means:
- Obstacles outside camera FOV are invisible
- Obstacles with poor visual contrast are missed
- Radar's accurate range measurements are wasted when camera is unavailable

**Planned fix:** Radar-primary architecture where both sensors create tracks independently, with sensor-strength-driven fusion (radar for range, camera for bearing/class).

---

## What to Look For in Logs

1. **`pos` Z dropping below 3.5:** Altitude loss — check occ count for massive spike
2. **`promoted` count growing beyond ~600:** Over-promotion — too many phantom detections
3. **`static` count < 300 at survey end:** Insufficient coverage — may miss obstacles
4. **`fallback=true`:** Planner failed, flying direct — likely into obstacles
5. **`|delta|` > 1.0:** Avoider heavily overriding — drone very close to obstacle
6. **`|delta|` near 0.0 when near obstacle:** Grid doesn't have obstacle at correct position
7. **`occ` spiking > 500:** Dense detection burst — may cause flight dynamics issues
8. **Dormant positions >20m from any obstacle:** Pool pollution with phantoms

---

## Cleanup Checklist

When removing debug logging after Epic #237 closes:

- [ ] `mission_state_tick.h`: Replace `[DIAG]` block with lighter-weight logging or remove
- [ ] Consider keeping trimmed version (every 30 ticks) as permanent operational telemetry
- [ ] Keep `[UKF] Radar-init` and `[UKF] New obstacle` logs — useful for operational monitoring
- [ ] Remove this file or archive to `docs/archive/`
