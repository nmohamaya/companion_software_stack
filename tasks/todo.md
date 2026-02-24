# Issue #4 — API-Driven Development: IMessageBus, ITopic, IServiceChannel + Internal Process Interfaces

## Plan

### Phase 1: Setup
- [x] Add Messaging Patterns Architecture section to README roadmap
- [x] Create GitHub issue #4
- [x] Create feature branch `feature/issue-4-api-interfaces`

### Phase 2: IPC Interfaces
- [x] Create `IPublisher<T>` abstract interface (`common/ipc/include/ipc/ipublisher.h`)
- [x] Create `ISubscriber<T>` abstract interface (`common/ipc/include/ipc/isubscriber.h`)
- [x] Create `ShmPublisher<T>` — SHM-backed publisher (`common/ipc/include/ipc/shm_publisher.h`)
- [x] Verify `ShmSubscriber<T>` — SHM-backed subscriber (already existed)
- [x] Create `ShmMessageBus` — factory for SHM pub/sub (`common/ipc/include/ipc/shm_message_bus.h`)
- [x] Create `IServiceClient/IServiceServer` — request-response (`common/ipc/include/ipc/iservice_channel.h`)
- [x] Create `ShmServiceClient/ShmServiceServer` — SHM-backed service channel (`common/ipc/include/ipc/shm_service_channel.h`)

### Phase 3: Internal Process Interfaces
- [x] `IVisualFrontend` + `SimulatedVisualFrontend` + factory (`process3_slam_vio_nav/include/slam/ivisual_frontend.h`)
- [x] `IPathPlanner` + `PotentialFieldPlanner` + factory (`process4_mission_planner/include/planner/ipath_planner.h`)
- [x] `IObstacleAvoider` + `PotentialFieldAvoider` + factory (`process4_mission_planner/include/planner/iobstacle_avoider.h`)
- [x] `IProcessMonitor` + `LinuxProcessMonitor` + factory (`process7_system_monitor/include/monitor/iprocess_monitor.h`)

### Phase 4: Wire Processes Through Interfaces
- [x] P1 (video_capture): ShmWriter → ShmMessageBus → IPublisher
- [x] P2 (perception): ShmReader/ShmWriter → ISubscriber/IPublisher
- [x] P3 (slam_vio_nav): + IVisualFrontend strategy
- [x] P4 (mission_planner): + IPathPlanner + IObstacleAvoider strategies
- [x] P5 (comms): ShmReader/ShmWriter → ISubscriber/IPublisher
- [x] P6 (payload_manager): ShmReader/ShmWriter → ISubscriber/IPublisher
- [x] P7 (system_monitor): + IProcessMonitor strategy
- [x] Build passes clean (all targets)
- [x] All 121 existing tests pass

### Phase 5: New Tests
- [x] `test_message_bus.cpp` — IPublisher, ISubscriber, ShmMessageBus, ShmServiceChannel (23 tests)
- [x] `test_process_interfaces.cpp` — IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor (19 tests)
- [x] Register both in `tests/CMakeLists.txt`
- [x] Fix build errors in test files (namespace + field name mismatches)
- [x] Build passes clean with new tests
- [x] All 163 tests pass (121 existing + 42 new)

### Phase 6: Documentation & Delivery
- [x] Create `interface_api.md` documenting all interfaces
- [x] Commit all changes (26 files, +2184/-280)
- [x] Push to `feature/issue-4-api-interfaces`
- [x] Create PR #5 referencing issue #4
- [x] Update this file with review section

## Review

### Delivered
- **12 new header files**: 7 IPC interfaces/implementations, 4 process strategy interfaces, 1 API doc
- **7 process main.cpp files refactored**: all use abstract interfaces via ShmMessageBus
- **2 new test files**: 42 new tests (163 total, 100% pass, 0.62s)
- **PR #5** created on branch `feature/issue-4-api-interfaces` closing issue #4

### Metrics
| Metric | Before | After |
|--------|--------|-------|
| Tests | 121 | 163 |
| Pass rate | 100% | 100% |
| New files | — | 14 |
| Modified files | — | 10 |
| Lines added | — | ~2184 |

### Corrections & Lessons
1. Wrote tests using wrong field names (e.g. `cpu_usage_pct` vs `cpu_usage_percent`) — lesson: always read struct definitions first
2. Wrote tests with wrong method signatures (e.g. `collect(health)` vs `collect()`) — lesson: read interface headers before writing tests
3. Bootstrap ShmWriter in service channel unlinked SHM on destruction — fixed with raw `ensure_shm_exists()` helper
4. Missing `<random>`, `<cmath>` includes in ivisual_frontend.h — caught by `-Werror`
5. Forgot to create `tasks/todo.md` and `tasks/lessons.md` at session start — fixed mid-session after re-reading prompt_instructions.md

All lessons documented in `tasks/lessons.md`.
