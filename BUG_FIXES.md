# Bug Fixes Log

Tracking all bug fixes applied to the Drone Companion Software Stack.

---

## Fix #1 — P3 SLAM Use-After-Free Race Condition

**Date:** 2026-02-23  
**Severity:** Critical  
**File:** `process3_slam_vio_nav/src/main.cpp`

**Bug:** The visual frontend thread allocated a `Pose` on the heap with `new` and published it via `std::atomic<Pose*>::exchange()`. The pose publisher thread read the pointer with `latest_pose.load()`. Between the load and the read of the pointed-to data, the frontend could call `exchange()` and `delete` the old pointer — causing a use-after-free in the publisher.

**Root Cause:** Raw pointer ownership shared across threads without lifetime protection.

**Fix:** Replaced the raw `new Pose*` + `atomic<Pose*>::exchange()` pattern with a `PoseDoubleBuffer` class — two stack-allocated `Pose` buffers with an atomic index swap. The writer writes to the inactive buffer then flips the index; the reader copies from the active buffer. Zero heap allocations, no lifetime issues.

---

## Fix #2 — ShmWriter Broken Move Constructor

**Date:** 2026-02-23  
**Severity:** High  
**File:** `common/ipc/include/ipc/shm_writer.h`

**Bug:** `ShmWriter(ShmWriter&&) = default;` left the moved-from object with valid `fd_` and `ptr_` members. When the moved-from object's destructor ran, it called `shm_unlink()` + `munmap()`, destroying the shared memory segment that the moved-to object was still using.

**Root Cause:** Compiler-generated move constructor does not know about resource ownership semantics for raw file descriptors and mmap pointers.

**Fix:** Implemented custom move constructor and move assignment operator that transfer `fd_`, `ptr_`, and `name_`, then null out the source (`fd_ = -1`, `ptr_ = nullptr`). The move assignment also cleans up any existing resources before taking ownership.

---

## Fix #3 — ShmReader Broken Move Constructor

**Date:** 2026-02-23  
**Severity:** High  
**File:** `common/ipc/include/ipc/shm_reader.h`

**Bug:** Same issue as Fix #2 — `ShmReader(ShmReader&&) = default;` left the moved-from object able to `munmap()` + `close()` the shared memory mapping still in use by the moved-to object.

**Root Cause:** Same as Fix #2 — default move semantics don't handle POSIX resource cleanup.

**Fix:** Custom move constructor and move assignment operator with proper source invalidation.

---

## Fix #4 — Uninitialized `total_cost` in HungarianSolver::Result

**Date:** 2026-02-23  
**Severity:** Medium  
**File:** `process2_perception/include/perception/kalman_tracker.h`

**Bug:** `HungarianSolver::Result::total_cost` was declared without an initializer. The `solve()` method used `+=` on it without ever setting it to zero, resulting in undefined behavior (garbage value accumulated into the cost).

**Root Cause:** Missing default member initializer on a POD field.

**Fix:** Added `= 0.0` default initializer to the `total_cost` member declaration.

---

## Fix #5 — MultiObjectTracker Never Created Tracks From Empty State

**Date:** 2026-02-23  
**Severity:** High  
**File:** `process2_perception/src/kalman_tracker.cpp`

**Bug:** When `tracks_` was empty (e.g., at startup), `compute_cost_matrix()` returned a 0-row matrix. `HungarianSolver::solve()` then computed `cols = rows > 0 ? cost[0].size() : 0`, resulting in `cols = 0`. This meant no columns were reported as unmatched, so `unmatched_cols` was empty, and no new tracks were ever created. The tracker was permanently stuck with zero tracks.

**Root Cause:** The solver assumed at least one row to determine the column count. With zero rows, the column dimension was lost.

**Fix:** Added an explicit check in `MultiObjectTracker::update()`: when `tracks_` is empty, all incoming detections are directly created as new tracks, bypassing the solver entirely.

**Found by:** Unit test `MultiObjectTrackerTest.SingleDetectionBecomesTrack`.

---

## Fix #6 — Uninitialized Eigen Members in LiDARCluster

**Date:** 2026-02-23  
**Severity:** Medium  
**File:** `process2_perception/include/perception/types.h`

**Bug:** `LiDARCluster::bbox_min`, `bbox_max`, `centroid`, and `distance` had no default initializers. When a `LiDARCluster` was default-constructed and then copied (e.g., passed in a `std::vector` to `FusionEngine::fuse()`), the uninitialized `Eigen::Vector3f` members triggered `-Wuninitialized` warnings. In a non-debug build, this could also cause unpredictable fusion results from garbage bounding box data.

**Root Cause:** Struct members with non-trivial types (`Eigen::Vector3f`) not given default values.

**Fix:** Added `= Eigen::Vector3f::Zero()` default initializers to `centroid`, `bbox_min`, and `bbox_max`, and `= 0.0f` to `distance`.

---

## Fix #7 — zenohc Rust 1.85 atexit() Panic on Session Teardown

**Date:** 2026-02-28  
**Severity:** Critical  
**File:** `common/ipc/include/ipc/zenoh_session.h`

**Bug:** All 12 Zenoh tests passed internally (`[  PASSED  ]`) but the test process aborted during C++ static destructor teardown. The `ZenohSession` singleton used a Meyers singleton pattern (`static ZenohSession inst;`), which is destroyed during `atexit()`. The zenohc library (Rust FFI) panics when its `Session::close()` runs during `atexit()` because thread-local storage is unavailable — a known Rust 1.85 issue ([rust-lang/rust#138696](https://github.com/rust-lang/rust/issues/138696)).

**Root Cause:** Static local variable in `instance()` triggers destructor during `atexit()`, but zenohc's Rust runtime requires thread-local storage that is already torn down at that point.

**Fix:** Changed the singleton from stack-allocated (`static ZenohSession inst;`) to heap-allocated and intentionally leaked (`static ZenohSession* inst = new ZenohSession();`). The session is never destroyed during process teardown; the OS reclaims all resources on exit. This is a standard pattern for singletons wrapping FFI libraries with complex teardown requirements.

**Found by:** CTest reporting "Subprocess aborted" on all 12 Zenoh tests despite `[  PASSED  ]` output.

---

## Fix #8 — Stack Overflow in LargeVideoFrameRoundTrip Test

**Date:** 2026-02-28  
**Severity:** High  
**File:** `tests/test_zenoh_ipc.cpp`

**Bug:** The `ZenohPubSub.LargeVideoFrameRoundTrip` test segfaulted. The test stack-allocated two `ShmVideoFrame` structs (~6.2 MB each: `1920 × 1080 × 3` bytes of `pixel_data`). Combined with the `ZenohSubscriber<ShmVideoFrame>` internal `latest_msg_` cache (another ~6.2 MB) and the `std::vector<uint8_t>` copy inside `publish()`, total stack usage exceeded the default 8 MB stack limit.

**Root Cause:** `ShmVideoFrame` is ~6,220,800 bytes. Stack-allocating two of them plus test framework overhead exceeds the default Linux stack size (typically 8 MB).

**Fix:** Changed the test to heap-allocate both `sent` and `received` frames using `std::make_unique<ShmVideoFrame>()`.

**Found by:** CTest reporting SEGFAULT on `ZenohPubSub.LargeVideoFrameRoundTrip`.
