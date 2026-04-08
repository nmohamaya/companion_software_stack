# Agent Report — Issue #285: ILogger Interface + DRONE_LOG Macros

## Summary

Implemented the ILogger abstraction layer (Epic #284, Sub-Epic A, Wave 1) that decouples all logging from spdlog. Created the interface, three implementations (SpdlogLogger, NullLogger, CapturingLogger), global thread-safe accessor, and DRONE_LOG_* convenience macros. Migrated all 69 existing files from direct `spdlog::*` calls to `DRONE_LOG_*` macros.

## Changes

### New Files

| File | Purpose |
|------|---------|
| `common/util/include/util/ilogger.h` | ILogger interface, SpdlogLogger, global accessor, DRONE_LOG_* macros |
| `common/util/include/util/null_logger.h` | No-op logger for benchmarking / disabled logging |
| `common/util/include/util/capturing_logger.h` | Test logger that captures messages for assertions |
| `common/util/include/util/spdlog_logger.h` | Convenience include header |
| `tests/test_ilogger.cpp` | Unit tests for ILogger, all implementations, macros, global accessor |

### Modified Files (69 files)

Mechanical replacement of `spdlog::info/warn/error/debug/critical(...)` with `DRONE_LOG_INFO/WARN/ERROR/DEBUG/CRITICAL(...)` across all processes and common libraries:

- **common/hal/** — 11 files (gazebo backends, simulated backends, HAL factory)
- **common/ipc/** — 10 files (Zenoh session, publisher, subscriber, service client/server, liveliness, network config, message bus factory)
- **common/util/** — 8 files (config, diagnostic, latency tracker, process graph, rate clamp, realtime, sd_notify, thread watchdog)
- **process1_video_capture/** — 1 file (main.cpp)
- **process2_perception/** — 6 files (main, detector factory, fusion engine, UKF, color contour, YOLO)
- **process3_slam_vio_nav/** — 3 files (main, VIO backend, IMU preintegrator)
- **process4_mission_planner/** — 12 files (main, FSM, fault manager, geofence, planners, obstacle avoider, occupancy grid, GCS handler, state tick)
- **process5_comms/** — 3 files (main, GCS link, MAVLink sim)
- **process6_payload_manager/** — 2 files (main, gimbal controller)
- **process7_system_monitor/** — 3 files (main, process manager, sys info)
- **tools/** — 1 file (flight replay)
- **tests/CMakeLists.txt** — added test_ilogger

## Design Decisions

1. **Single `log(Level, string)` method** — simpler interface than per-level virtuals; macros handle the level dispatch
2. **Atomic global accessor** — `logger()` uses `std::atomic<ILogger*>` with acquire/release for thread-safe hot-path reads; `set_logger()` holds a mutex to protect the owning `unique_ptr` (cold-path only)
3. **Level-gated macros** — `DRONE_LOG_*` checks `should_log()` before calling `fmt::format()`, giving zero overhead on disabled levels
4. **SpdlogLogger as default** — no code changes needed at startup; `LogConfig::init()` works unchanged
5. **Phase 1 scope** — interface + implementations + migration only; no changes to LogConfig internals or JSON log sink

## Test Plan

- [ ] `mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" -DALLOW_INSECURE_ZENOH=ON && make -j$(nproc)` — zero warnings
- [ ] `ctest --test-dir build --output-on-failure -j$(nproc)` — all pass, count >= 1259
- [ ] `test_ilogger` tests pass (SpdlogLogger, NullLogger, CapturingLogger, global accessor, macros, level filtering)
- [ ] clang-format clean

## Risks / Review Attention

- **Global mutable state** — `set_logger()` / `reset_logger()` mutate a process-wide singleton. Thread-safe via atomic + mutex, but callers should only call at startup or in test fixtures (never mid-flight).
- **Large mechanical diff** — 69 files changed with `spdlog::*` → `DRONE_LOG_*` replacement. Low risk per file, but review should verify no accidental format string changes.
- **spdlog still linked** — SpdlogLogger delegates to spdlog; the dependency isn't removed, just abstracted. Full removal is a future phase.
