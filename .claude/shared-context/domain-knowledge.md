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

### TripleBufferTest.HighContentionStress is flaky
This is a pre-existing flaky test. If it fails intermittently, it is not caused by your changes.

### Fully qualify namespaces in test files
Interfaces live in sub-namespaces (`drone::slam`, `drone::planner`, `drone::monitor`). A bare `using namespace drone` is not sufficient. Either add `using namespace` for each sub-namespace you reference, or use full qualification consistently.

### Missing standard library includes
Every standard library type needs its header explicitly included. Do not rely on transitive includes. Common misses: `<random>`, `<cmath>`, `<algorithm>`.

## Eigen

### Always initialize Eigen types
Uninitialized Eigen vectors/matrices cause warnings under `-Wall`. Always use `= Eigen::Vector3f::Zero()` or equivalent at declaration.
