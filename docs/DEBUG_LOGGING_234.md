# Debug Logging for Issue #234 — D* Lite Planner Drift

Temporary diagnostic logging added during Issue #234 to trace D* Lite
path planning issues in Scenario 18 (perception avoidance).

Root causes identified and fixed (Bugs #50, #51, #52 in `docs/BUG_FIXES.md`).
Logging retained for ongoing tuning — **remove all entries below when Issue #234 closes.**

---

## 1. `dstar_lite_planner.h` — Search result + path diagnostics

| Location | Tag | What it logs |
|----------|-----|--------------|
| `do_search()` ~line 96-100 | `[D*Lite]` | Search wall-clock duration (`search_ms`) |
| `do_search()` ~line 102-108 | `[D*Lite] No path:` | Failed search: start/goal coords, g(start), queue size, occupied count |
| `do_search()` ~line 113-120 | `[D*Lite] Path OK:` | Successful search: path length, first/last waypoint world coords, g(start), occupied count |
| `do_search()` ~line 122-124 | `[D*Lite] Path extraction FAILED:` | Extract failed despite g(start) < Inf |
| `compute_shortest_path()` ~line 357 | `[D*Lite] search:` | Iteration count, queue size, g(start) (debug level) |

**Purpose:** Determine if searches are succeeding, how long they take, and whether the path direction makes sense.

---

## 2. `grid_planner_base.h` — Goal snap + velocity + path-following diagnostics

| Location | Tag | What it logs |
|----------|-----|--------------|
| `plan()` ~line 127-130 | `[PlanBase] Goal snapped:` | When goal cell is moved from original to snapped position |
| `plan()` ~line 141-153 | `[PlanBase] Replan:` | On each replan: path first-step direction vs goal direction, dot product (1.0=aligned, -1.0=opposite), path size, search time |
| `plan()` ~line 161-162 | `[PlanBase] Search failed` | Direct fallback with search duration |
| `plan()` ~line 209-216 | `[PlanBase] pos=... vel=... raw=...` | Every 50 ticks: drone position, EMA-smoothed velocity, raw velocity, path index/size, fallback flag, current goal XY |

**Member added:** `uint64_t diag_tick_ = 0;` (line 354)

**Purpose:** See what velocity the planner is outputting, whether it's following cached path or direct fallback, and where the path-follow goal point is.

---

## 3. `mission_state_tick.h` — Planner vs Avoider velocity comparison

| Location | Tag | What it logs |
|----------|-----|--------------|
| `tick_navigate()` ~line 209 | `[DEBUG] plan_vel=... avoid_vel=...` | Every 30 ticks: planner velocity, post-avoider velocity, delta vector, delta magnitude, current WP index |

**Member added:** `uint64_t debug_tick_ = 0;` (line 92)

**Purpose:** Detect if the obstacle avoider is overriding or deflecting the planned velocity, causing the drone to drift off-course.

---

## What to look for in logs

1. **Search takes too long (>100ms):** Blocks the planning tick; drone flies on stale velocity.
2. **Path first waypoint points south:** Search is routing around obstacles in the wrong direction.
3. **Avoider delta is large (>0.5):** Avoider is significantly modifying the planned velocity.
4. **`fallback=1`:** Planner failed, flying direct line (may be into obstacles).
5. **`raw` velocity points south but goal is north:** Path-following logic bug.
6. **Frequent `[D*Lite] Init` lines:** km_ > 10 reinit means drone moved far between replans.

---

## Cleanup checklist

When removing debug logging:

- [ ] `dstar_lite_planner.h`: Remove extra `spdlog::info` calls in `do_search()` (keep debug-level ones)
- [ ] `grid_planner_base.h`: Remove `diag_tick_` member and the periodic `spdlog::info` block in `plan()`
- [ ] `mission_state_tick.h`: Remove `debug_tick_` member and the `[DEBUG]` log block in `tick_navigate()`
- [ ] Delete this file (`docs/DEBUG_LOGGING_234.md`)
