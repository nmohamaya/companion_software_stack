# Agent Report — Issue #286: IClock Interface + MockClock

**Epic:** #284 (Platform Modularity & Adaptability)
**Sub-Epic:** A — Foundational Abstractions
**Gap:** G2 — std::chrono::steady_clock hardcoded in 15+ files
**Branch:** `feature/issue-286-feat284-a2-iclock-interface--mockclock`
**Agent:** feature-infra-core

## Summary

Created the `IClock` abstract interface with `SteadyClock` (production) and `MockClock` (test) implementations, plus a global accessor `drone::util::get_clock()` with `set_clock()` for dependency injection. Migrated all `steady_clock::now()` calls in `common/util/`, `common/ipc/`, and `common/recorder/` to use the new IClock interface. Added 26 unit tests.

## Changes

### New Files
| File | Description |
|------|-------------|
| `common/util/include/util/iclock.h` | IClock interface, SteadyClock impl, global `get_clock()`/`set_clock()` |
| `common/util/include/util/mock_clock.h` | MockClock (manually advanceable), ScopedMockClock (RAII guard) |
| `tests/test_iclock.cpp` | 26 unit tests covering all IClock functionality |

### Modified Files (11 production, 1 build)
| File | Change |
|------|--------|
| `common/util/include/util/thread_heartbeat.h` | `touch()` / `touch_with_grace()` → `get_clock().now_ns()` |
| `common/util/include/util/thread_watchdog.h` | `scan_loop()` / `scan_once()` → `get_clock().now()`/`now_ns()` |
| `common/util/include/util/thread_health_publisher.h` | timestamp → `get_clock().now_ns()` |
| `common/util/include/util/scoped_timer.h` | start/elapsed/dtor → `get_clock().now_ns()` (no more time_point) |
| `common/util/include/util/diagnostic.h` | `ScopedDiagTimer` → `get_clock().now_ns()` |
| `common/util/include/util/latency_tracker.h` | `now_ns()` static → delegates to `get_clock().now_ns()` |
| `common/ipc/include/ipc/wire_format.h` | `wire_serialize()` auto-timestamp → `get_clock().now_ns()` |
| `common/ipc/include/ipc/iservice_channel.h` | `await_response()` deadline/timeout → IClock |
| `common/ipc/include/ipc/zenoh_subscriber.h` | receive timestamp → `get_clock().now_ns()` |
| `common/ipc/include/ipc/zenoh_service_server.h` | envelope/response timestamps → `get_clock().now_ns()` |
| `common/recorder/include/recorder/flight_recorder.h` | record timestamp → `get_clock().now_ns()` |
| `tests/CMakeLists.txt` | Register `test_iclock` test binary |

## Design Decisions

1. **`get_clock()` not `clock()`** — POSIX `clock()` from `<time.h>` causes ambiguity errors in translation units that include both. Named `get_clock()` to avoid conflict.

2. **Raw pointer for global accessor** — The global clock uses `IClock*` (not `shared_ptr`) because:
   - Caller owns the clock lifetime (typically stack-allocated `MockClock` in tests)
   - No ownership transfer — just a reference swap
   - Zero overhead in the hot path (no ref counting)

3. **MockClock starts at 1 second** — Default initial time `1'000'000'000 ns` avoids edge cases where code checks `timestamp == 0` to mean "uninitialized".

4. **`sleep_for_ms` on MockClock advances time** — Instead of real blocking, MockClock's `sleep_for_ms()` advances simulated time, enabling instant test execution.

5. **`ScopedMockClock` RAII guard** — Automatically installs MockClock in constructor and restores SteadyClock in destructor, preventing test leakage.

6. **Not migrating HAL or process files** — HAL implementations (`common/hal/`) and process directories are outside this agent's scope. Those should be migrated by their respective feature agents in follow-up issues.

## Files NOT Migrated (Out of Scope)

The following files still use `steady_clock::now()` directly and should be migrated in follow-up work by their respective agents:

- `common/hal/` — 12 occurrences across simulated/gazebo/mavlink backends
- `process[1-7]_*/` — ~40 occurrences across all process directories
- `tools/` — fault_injector and flight_replay (~4 occurrences)
- `tests/` — test files using steady_clock for test timing (~20 occurrences, mostly appropriate)

## Test Coverage

26 new tests in `test_iclock.cpp`:
- **SteadyClockTest** (4): now_ns accuracy, now() time_point, now_seconds, monotonicity
- **MockClockTest** (13): default init, custom init, advance_ns/ms/s, cumulative, set_ns, reset, sleep_for_ms, now() time_point, now_seconds, thread safety
- **GlobalClockTest** (3): default is SteadyClock, set_clock to mock, null restores default
- **ScopedMockClockTest** (3): install/restore, custom initial, sleep advances
- **IClockPolymorphismTest** (3): virtual dispatch, now() convenience, now_seconds convenience

## Build Verification

- Build succeeded with `-Werror -Wall -Wextra` (zero warnings)
- 57 test binaries produced including `test_iclock`

## Remaining Work for Follow-Up

1. **Migrate HAL files** → Assign to feature-perception, feature-nav, feature-integration agents
2. **Migrate process files** → Assign to respective feature agents
3. **Migrate tools/** → fault_injector, flight_replay
4. **Documentation updates** — TESTS.md, PROGRESS.md, ROADMAP.md (blocked by test count verification)
