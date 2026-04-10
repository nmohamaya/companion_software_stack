# Agent Report -- Issue #394: Snap offset exceeding acceptance_radius

## Summary

Fixed a bug where D* Lite's `snap_goal()` could relocate an occupied waypoint further than the `acceptance_radius`, making the waypoint unreachable. The drone would fly to the snapped position but `waypoint_reached()` checked distance to the original waypoint, causing an infinite loop.

The fix passes the snapped world position through to `waypoint_reached()` so the acceptance check uses the actual navigation target.

## Changes

| File | Change |
|------|--------|
| `process4_mission_planner/include/planner/mission_fsm.h` | Added optional `snapped_xyz` parameter to `waypoint_reached()` -- when non-null, acceptance check uses snapped position instead of original waypoint |
| `process4_mission_planner/include/planner/grid_planner_base.h` | Added `has_snapped_goal()` and `snapped_goal_xyz()` public accessors; added `snapped_xyz_[3]` member array populated during snap |
| `process4_mission_planner/include/planner/mission_state_tick.h` | Updated `tick_navigate()` to pass snapped position to `waypoint_reached()` via `GridPlannerBase` accessors |
| `tests/test_mission_fsm.cpp` | Added 4 regression tests for snap offset acceptance behavior |

## Design Decisions

1. **Optional parameter over overload**: Added `const float* snapped_xyz = nullptr` default parameter to `waypoint_reached()` rather than a second overload. This preserves backward compatibility -- all existing callers (including the overshoot check) continue to work unchanged.

2. **Raw float[3] pointer over struct**: Used `const float*` pointing to a 3-element array rather than introducing a new struct. This matches the existing codebase pattern where world positions are represented as float arrays (`std::array<float,3>`). The pointer is stable between `plan()` calls.

3. **dynamic_cast to GridPlannerBase**: Used `dynamic_cast<GridPlannerBase*>(grid_planner)` in `tick_navigate()` to access snap state. This is consistent with existing patterns in `tick_survey()` (line 253) and the diagnostic block (line 359). The cast is safe because `grid_planner` is always either null or a `GridPlannerBase` subclass.

4. **Snapped position stored in separate array**: Added `snapped_xyz_[3]` alongside existing `snapped_world_{x,y,z}_` to provide a contiguous float array for the public API. The existing members are used internally by `snap_goal()` and path following; the new array is the public-facing accessor.

## Test Coverage

4 new regression tests added to `tests/test_mission_fsm.cpp`:

| Test | Verifies |
|------|----------|
| `WaypointReachedAtSnappedPosition` | Drone at snapped position (6.32m from original) is "reached" when snap override provided; NOT reached without override |
| `WaypointNotReachedFarFromBothOriginalAndSnapped` | Drone far from both positions is not reached even with snap override |
| `WaypointReachedNoSnapNormalBehavior` | Null snap pointer preserves original behavior |
| `WaypointReachedWithinRadiusOfSnap` | Radius threshold: verifies acceptance at and beyond the snap radius boundary |

## Build Verification

- Build: zero warnings (`-Werror -Wall -Wextra`)
- Format: clang-format-18 clean
- Mission tests: 60/60 passed
- Test count: 1347 (unchanged -- new tests are in existing binary, ctest counts binary-level tests)

## Remaining Work

None for the core fix. Optional follow-ups:

1. **Add snap offset to telemetry logging**: When a snap occurs with offset > acceptance_radius, log a warning so operators can see the issue in flight logs.
2. **Overshoot check with snap**: `waypoint_overshot()` still checks against original waypoint positions. This is likely fine since overshoot is a convenience optimization (advance early), not a correctness requirement, but could be revisited if it causes issues.
