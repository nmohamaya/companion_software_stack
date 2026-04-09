# Agent Report — Issue #287: Config Key Registry

## Summary

Created a centralized config key registry (`config_keys.h`) with `constexpr const char*` constants for every runtime config key, replacing 120+ scattered magic string literals across 17 files. Misspelled keys are now caught at compile time instead of silently falling back to defaults.

## Changes

### Created
- **`common/util/include/util/config_keys.h`** — Centralized constexpr config key registry
  - 120+ keys organized by process/section namespace
  - Follows the `ipc_types.h` topic names pattern (`drone::cfg_key::` namespace)
  - Sections: top-level, zenoh, video_capture, perception, slam, mission_planner, comms, payload_manager, system_monitor, watchdog, recorder, hal

### Modified (17 files)
| File | Changes |
|------|---------|
| `process1_video_capture/src/main.cpp` | 6 key replacements (mission_cam, stereo_cam) |
| `process2_perception/src/main.cpp` | 10 key replacements (detector, tracker, fusion, radar) |
| `process2_perception/include/perception/color_contour_detector.h` | 10 key replacements |
| `process2_perception/src/opencv_yolo_detector.cpp` | 4 key replacements |
| `process3_slam_vio_nav/src/main.cpp` | 17 key replacements (ipc_backend, imu, vio, stereo) |
| `process4_mission_planner/src/main.cpp` | 50+ key replacements (path_planner, occupancy_grid, geofence, collision_recovery, etc.) |
| `process4_mission_planner/include/planner/obstacle_avoider_3d.h` | 8 key replacements |
| `process4_mission_planner/include/planner/static_obstacle_layer.h` | 1 key replacement |
| `process5_comms/src/main.cpp` | 7 key replacements (mavlink, gcs) |
| `process6_payload_manager/src/main.cpp` | 4 key replacements (gimbal, auto_track) |
| `process7_system_monitor/src/main.cpp` | 10 key replacements (thresholds, backend) |
| `common/hal/include/hal/hal_factory.h` | `.backend` and `.gz_topic` → `hal::BACKEND`, `hal::GZ_TOPIC` |
| `common/hal/include/hal/gazebo_radar.h` | 11 key replacements (radar params, noise) |
| `common/hal/include/hal/simulated_radar.h` | 9 key replacements |
| `common/ipc/include/ipc/message_bus_factory.h` | 3 key replacements (ipc_backend, zenoh) |
| `common/recorder/include/recorder/flight_recorder.h` | 2 key replacements |
| `tests/test_config.cpp` | Added 9 config key registry tests |

## Design Decisions

1. **Namespace structure** mirrors `config/default.json` hierarchy: `drone::cfg_key::mission_planner::path_planner::BACKEND`
2. **HAL sub-keys** use a `.prefix` pattern (e.g., `hal::BACKEND = ".backend"`) for dynamic section composition — matches existing HAL factory pattern
3. **Test-specific config keys** in test files were NOT replaced since they test the Config class itself with synthetic keys, not production config
4. **Constexpr** enables compile-time detection of typos

## Verification

- [x] Build succeeds with `-Werror -Wall -Wextra` (zero warnings)
- [x] All 120+ production config key string literals replaced
- [x] Zero remaining raw config strings in process/common code (verified via grep)
- [x] 9 new tests added for key value correctness and constexpr-ness
- [ ] Test suite execution (persistent permission issues prevented running ctest/test binaries in this session — build confirms compilation correctness)

## Test Additions

Added to `tests/test_config.cpp`:
- `ConfigKeyRegistryTest.TopLevelKeysMatchExpected`
- `ConfigKeyRegistryTest.VideoCaptureSectionKeys`
- `ConfigKeyRegistryTest.PerceptionDetectorKeys`
- `ConfigKeyRegistryTest.SlamKeys`
- `ConfigKeyRegistryTest.MissionPlannerKeys`
- `ConfigKeyRegistryTest.CommsKeys`
- `ConfigKeyRegistryTest.SystemMonitorKeys`
- `ConfigKeyRegistryTest.HalSubKeys`
- `ConfigTest.ConfigKeysWorkWithDefaultJson`
- `ConfigKeyRegistryTest.KeysAreConstexpr`

## Notes for Reviewer

- This is a pure refactor — no behavioral changes. All string constants resolve to the exact same values as the literals they replace.
- The HAL radar files (`gazebo_radar.h`, `simulated_radar.h`) are technically HAL implementation scope, but were updated since they contain config key string literals that benefit from centralization.
- clang-format was not explicitly run due to permission issues, but the code follows the project's 100-char column, 4-space indent style.
