# ADR-008: P4 Mission Planner — Header-Only Sub-Component Extraction

| Field | Value |
|-------|-------|
| **Status** | Accepted — fully implemented (PR #157, Improvement #41) |
| **Date** | 2026-03-14 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | Issue #154, PR #157, ADR-003 (C++17), ADR-007 (error handling) |

---

## 1. Context

`process4_mission_planner/src/main.cpp` grew to **809 lines** as the mission
planner accumulated sub-systems: fault response escalation, GCS command
dispatch, per-tick FSM transitions, and HD-map static obstacle management.
All four sub-systems were inlined inside a single planning loop function:

- Hard to unit-test in isolation — each sub-system required a fully-wired
  Zenoh bus, real config, and live HAL mocks
- The FSM transition logic was particularly complex and error-prone to change
  without regressions
- Code reviews on `main.cpp` required understanding the full 800-line context
  to assess a localised change

The monolithic structure violated the single-responsibility principle and
made isolated regression testing impractical.

---

## 2. Decision Drivers

- Each sub-system must be independently unit-testable with injected mocks
  and no Zenoh/hardware dependency
- Extraction must not change observable runtime behaviour (zero behavioural
  delta — validated by existing 94 passing tests before extraction)
- Must be compatible with the existing `drone::Config` / `Result<T,E>` /
  `[[nodiscard]]` patterns (ADR-003, ADR-007)
- No new build dependencies; header-only preferred to avoid linker complexity
- `main.cpp` must stay as the sole orchestration entry point — no new
  process-level executables

---

## 3. Decision

Extract four logically cohesive sub-systems from `main.cpp` into
**header-only classes** under `process4_mission_planner/include/planner/`:

| Class | Header | Responsibility |
|-------|--------|---------------|
| `FaultResponseExecutor` | `fault_response_executor.h` | Translate `FaultAction` → trajectory stop + FC command; enforce escalation-only policy |
| `GcsCommandHandler` | `gcs_command_handler.h` | Dispatch RTL/LAND/MISSION_UPLOAD; deduplicate by command timestamp |
| `MissionStateTick` | `mission_state_tick.h` | All per-tick FSM transitions (PREFLIGHT/TAKEOFF/NAVIGATE/RTL/LAND/IDLE) |
| `StaticObstacleLayer` | `static_obstacle_layer.h` | Load HD-map entries from config; camera cross-validation; collision proximity check |

Each class:

1. Takes its dependencies via constructor injection (bus references, config,
   HAL interface pointers) — no global state, no singleton lookups
2. Exposes a single primary method (`execute`, `handle`, `tick`, `check_collision`)
   that can be called directly from unit tests with mock inputs
3. Is header-only — no separate `.cpp` compilation unit, no CMakeLists change
4. Is `[[nodiscard]]`-annotated on all methods that return state

`main.cpp` was reduced from **809 → 366 lines**. It now only:
- Parses config and instantiates components
- Subscribes to IPC channels
- Runs the planning loop (delegating each sub-task to the extracted class)

---

## 4. Alternatives Considered

### 4.1 Keep everything in `main.cpp`

**Rejected.** Unit tests for FSM transitions required linking the full
process binary and injecting messages via a live Zenoh bus. This was slow,
flaky (Zenoh session limits under `ctest -j`), and gave no isolation.

### 4.2 Separate `.cpp` compilation units (traditional split)

**Not chosen.** Header-only avoids adding new CMake targets and keeps the
extraction reversible. The classes are not large enough to justify the
compile-time overhead of separate translation units.

### 4.3 Sub-process or plugin architecture

**Out of scope.** The sub-systems share the same thread, same Zenoh bus
instance, and same in-process state. A plugin/subprocess split would be
over-engineering for what are conceptually large functions.

---

## 5. Consequences

### Positive

- **35 new unit tests** across 4 new test files — each class is tested
  completely in isolation (no Zenoh, no real hardware)
- `main.cpp` is now understandable in a single review pass (~366 lines)
- New sub-system changes (e.g. adding a new FSM state) are confined to a
  single header and have a clear test target
- Pattern is **replicable** — other process `main.cpp` files exceeding
  ~400 lines of logic should follow the same extraction approach

### Negative / Trade-offs

- Header-only classes increase `main.cpp` compile time marginally (all
  class code is compiled into the single translation unit)
- Dependency injection via constructor requires callers to manage object
  lifetime ordering (mitigated by RAII and the fact that `main.cpp` owns
  all objects on the stack)

---

## 6. Implementation Notes

- All four classes follow the same dependency-injection pattern:
  ```cpp
  FaultResponseExecutor executor{bus, cfg};
  // later in loop:
  executor.execute(fault_action, current_state);
  ```
- Unit tests use Google Test with lightweight struct mocks — no Zenoh
  `RESOURCE_LOCK` needed
- Integration validated by all 8 Gazebo SITL scenarios passing post-merge

---

## 7. Review Status

Accepted — implemented in PR #157 (Improvement #41). All 880 tests pass.
