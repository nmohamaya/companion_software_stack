<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

# Domain Knowledge

Non-obvious project knowledge collected from agent experience. Read this at session start to avoid known pitfalls.

## Build and CI

### Stale object files from different build types
Mixing Release and Coverage builds in the same `build/` directory causes `__gcov_init` linker errors. Always `rm -rf build/*` when switching build types. Consider separate directories (`build-release/`, `build-coverage/`).

### Always build with exact CI flags before pushing
Before every push, do a clean build with `-DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra"` to match CI. A `(void)` cast does NOT suppress `warn_unused_result` -- you must actually branch on the return value.

### Don't trust test counts -- verify after build
After any build, compare test count to the known baseline (currently 1286 -- see `tests/TESTS.md`). A passing run at the wrong count is not "all tests pass".

### CI fails but local passes
Check for Anaconda `LD_LIBRARY_PATH` masking system libraries. This is a recurring issue.

## Zenoh IPC

### Session exhaustion in tests
Zenoh sessions exhaust at ~8 parallel instances in ctest. Always add `RESOURCE_LOCK "zenoh_session"` to any test that creates a Zenoh session. Without this, tests get SIGABRT under `ctest -j`.

### ALLOW_INSECURE_ZENOH=ON required for dev
Dev machines without TLS certificates must set `-DALLOW_INSECURE_ZENOH=ON` in the cmake configure step.

## Struct and API Gotchas

### DetectedObjectList has no `count` field
Use `objects.size()` instead. There is no `.count` member.

### FCState is too large for std::atomic
`FCState` is ~100 bytes. It cannot be used with `std::atomic` for lock-free access. Use a mutex for consistent snapshots.

### Result<T,E> -- no `err_value()` method
The `err_value()` method does not exist on `Result<T,E>`. Use `.error()` instead.

### Always check actual struct field names before writing tests
Field names are not always what you expect. For example, the actual fields are `cpu_usage_percent` (not `cpu_usage_pct`), `memory_usage_percent` (not `memory_usage_pct`), `cpu_temp_c` (not `cpu_temp_celsius`). Always read the struct definition before writing test code.

### Always check method signatures before writing tests
Do not assume parameter types or return conventions. Read the actual interface header. For example, `IProcessMonitor::collect()` returns by value, not via out-parameter.

### Account for `-Werror` when initializing structs
When aggregate-initializing structs under `-Werror`, always include ALL fields or use designated initializers. Missing fields trigger `-Wmissing-field-initializers`.

## Radar and Sensor Fusion

### Radar returns are noisy below 5m
Always filter ground returns before fusion. The HAL ground filter handles this, but if you bypass it or write new radar processing code, you must account for ground clutter.

### Camera provides bearing, radar provides range
The fusion architecture is covariance-driven: camera gives bearing (azimuth/elevation), radar gives range. Neither sensor alone gives a complete track. The UKF fuses them based on per-measurement covariance.

## Perception

### Color contour detector and false promotions
The `color_contour` detector can misidentify ground texture as obstacles. Combined with depth clamping (40m mission cam, 8m stereo), this creates ghost cells in the occupancy grid that get promoted permanently. This is a known issue (see project false-promotion bug notes).

## Logging

### spdlog::info is too noisy for per-tick logging
Use `spdlog::debug` for anything that fires every tick. Gate with `spdlog::should_log()` to avoid string formatting overhead when the level is disabled.

## Config and Validation

### config_validator.h non-object intermediate nodes
`validate()` in `config_validator.h` was recently fixed to handle non-object intermediate nodes, but always wrap validation calls in try/catch as defense-in-depth.

## Testing

### TripleBufferTest.HighContentionStress — fixed (was flaky)
Previously flaky on CI due to producer finishing before consumer was scheduled. Fixed with start barrier + minimum 50ms runtime. If it still fails, the issue is a real TripleBuffer bug, not scheduling.

### Fully qualify namespaces in test files
Interfaces live in sub-namespaces (`drone::slam`, `drone::planner`, `drone::monitor`). A bare `using namespace drone` is not sufficient. Either add `using namespace` for each sub-namespace you reference, or use full qualification consistently.

### Missing standard library includes
Every standard library type needs its header explicitly included. Do not rely on transitive includes. Common misses: `<random>`, `<cmath>`, `<algorithm>`.

## Eigen

### Always initialize Eigen types
Uninitialized Eigen vectors/matrices cause warnings under `-Wall`. Always use `= Eigen::Vector3f::Zero()` or equivalent at declaration.

## Concurrency Design Rationale

### When to use mutex vs atomic vs lock-free
| Scenario | Mechanism | Example |
|----------|-----------|---------|
| Single small value (bool, int, ptr) | `std::atomic` with acquire/release | `offboard_active_` in Gazebo backends |
| Multi-field struct, consistent snapshot | Mutex (`std::lock_guard`) | `MavlinkFCLink::cached_state_` (~100 bytes) |
| Hot path (>10k ops/sec) | Lock-free (atomic, triple buffer) | `ThreadHeartbeat` |
| Cold path (<100 ops/sec) | Mutex — simpler, correct, negligible cost | Config access |

`FCState` is ~100 bytes and **cannot** be `std::atomic` (hardware atomics max 8-16 bytes). Use mutex. `memory_order_relaxed` is banned without justification — use `acquire`/`release`. Performance difference is zero on x86, ~1-2ns on ARM/Jetson.

## Sensor Fusion Architecture

### Camera provides bearing, radar provides range
The UKF fuses camera (sub-degree bearing from pinhole model, color/class, bbox) with radar (±0.3m range, radial velocity). Neither alone gives a complete track. Covariance-driven: camera-only P(0,0)=100 (uncertain depth), radar-initialized P(0,0)=0.5 (accurate range).

### UKF state and coordinate conventions
State: `[x=depth(forward), y=lateral(right), z=vertical(down), vx, vy, vz]` in body frame (FRD). Radar azimuth from Gazebo is FLU (positive=left) — negate for UKF FRD (positive=right).

### ML depth conversion formulas — verify full range utilization
When implementing or modifying depth conversion formulas (inverse depth → metric depth, disparity → depth, etc.), always verify that the output spans the full `[min_depth, max_depth]` range on non-uniform input. A common bug: `depth = max / (normalized + eps)` looks correct but collapses almost all output to `max_depth` because any `normalized < ~1` produces a value above the clamp ceiling. Test with: (1) gradient input asserting `max_d - min_d > 1.0m`, and (2) half-black/half-white "known scene" asserting left/right mean depth differs by >2m. These catch formula bugs at the unit test level without needing Gazebo. See Fix #51 in `docs/tracking/BUG_FIXES.md` for the full investigation.

### Depth estimation 4-tier strategy
1. Horizon-truncated bbox → ground-plane depth from bbox bottom
2. Apparent-size: `depth = assumed_height * fy / bbox_h`
3. Ground-plane fallback: `depth = camera_height / ray_down`
4. Near-horizon conservative 8m estimate

Depth clamping at 40m (mission cam) and 8m (stereo/apparent size) creates ghost cells when depth estimation fails. Always check if a depth value equals these clamp values before trusting it.

## False Cell Promotion (Known Bug — Issues #339/#340)

### Ground features get promoted to permanent obstacles
The `color_contour` detector misidentifies ground textures/shadows/landing pads as obstacles. Depth estimation fails → clamps to 40m or 8m. After `promotion_hits` observations, ghost cells become permanent static cells that never decay. Evidence: 616 promoted cells with only 4 real obstacles in scenario 02.

### Workarounds in place
- Scenario 02 (HD-map): `promotion_hits=0` disables promotion
- Scenario 18 (perception): `max_static_cells=800` caps promoted cells

## Pipeline Monitoring (tmux + ntfy.sh)

### ntfy.sh topics are public by default
Pipeline notifications sent via ntfy.sh include issue numbers, PR URLs, branch names, and checkpoint summaries. The default `ntfy.sh` public server means anyone who guesses the topic name can read these. For production use: self-host ntfy with `NTFY_TOKEN` authentication, or use `--notify-minimal` mode. Set `NTFY_SERVER` to your self-hosted instance and `NTFY_TOKEN` for auth.

### tmux session attach uses execvp — no return on success
`TmuxSession.exec_attach()` calls `os.execvp()` which replaces the current process. If the session dies between the exists() check and the exec call (TOCTOU race), an `OSError` is raised. The code handles this, but callers should be aware that `exec_attach()` never returns on success.

## Production Readiness

### DIAG logging must use spdlog::debug
All per-tick and high-frequency diagnostic logging must use `spdlog::debug`, never `spdlog::info`. Gate with `spdlog::should_log()` to avoid string formatting overhead. PR #355 fixed this for `mission_state_tick.h`.

### D* Lite z-band collapse under grid congestion (color_contour artifact)
When `color_contour` produces noisy detections near an obstacle (occupied oscillates 5→696→1 in seconds), D* can fail to find paths at the flight altitude z-plane and drop to z=0 (ground level). At z=0 the grid has fewer occupied cells — creating a false passage through the obstacle's physical location. Observed in scenario 18 (perception_avoidance) Run 3 on 2026-04-11: drone was commanded through the green object at ground level. This is a **simulation-only issue** that resolves with a proper perception pipeline (YOLOv8, real radar, or depth-based detection) because the occupied cell oscillation disappears with stable detections. Do NOT add `z_band_cells` constraints or altitude floors to work around this — fix the perception pipeline instead.

### Don't fix code bugs by changing config
When a test fails or behavior is wrong, investigate the root cause in code first. Never adjust config thresholds to mask a code bug — it will resurface in production with real sensors.

### Integration branches for multi-issue features
Multi-issue features (epics) use an integration branch between worktrees and main. All PRs target the integration branch. Merge to main only after all sub-issues pass + full regression. This keeps main demo-ready.
