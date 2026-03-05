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

---

## Review Fixes (PR #52) — 9 Issues Addressed

**Date:** 2026-02-28
**Context:** Copilot pull-request reviewer flagged 9 issues on PR #52 (Zenoh Phase A). All addressed in commit `8099843`.

### R1 — `subscribe_lazy()` Returned nullptr (Potential Null Dereference)

**File:** `zenoh_message_bus.h`
**Severity:** High
**Bug:** `subscribe_lazy()` logged a warning and returned `nullptr`. Any caller switching from `ShmMessageBus` (where `subscribe_lazy()` returns a valid object) to `ZenohMessageBus` would dereference null.
**Fix:** `subscribe_lazy()` now accepts a `topic` parameter, maps it via `to_key_expr()`, and returns a functional `ZenohSubscriber<T>`. For Zenoh, lazy and eager subscription are identical since connections are always asynchronous.

### R2 — `to_key_expr()` Crashed on Empty String

**File:** `zenoh_message_bus.h`
**Severity:** Medium
**Bug:** `to_key_expr()` called `name.substr(1)` without checking if the string was empty. An empty string would throw `std::out_of_range`.
**Fix:** Added empty input guard — returns `""` with a `spdlog::warn()` instead of crashing.

### R3 — Misleading `message_bus_factory.h` Header Comment

**File:** `message_bus_factory.h`
**Severity:** Low (documentation)
**Bug:** Header comment described the API as taking `drone::Config` and returning a pair (`auto [shm_bus, zenoh_bus]`). The actual API takes a `std::string` and returns a `MessageBusVariant`.
**Fix:** Rewrote the header comment to accurately describe the `create_message_bus(string)` → `MessageBusVariant` API with correct usage examples.

### R4 — Missing `#include <unistd.h>` in Tests

**File:** `test_zenoh_ipc.cpp`
**Severity:** Low (portability)
**Bug:** `getpid()` was used without `#include <unistd.h>`, relying on a transitive include that could break on different compilers/platforms.
**Fix:** Added explicit `#include <unistd.h>`.

### R5 — Inaccurate Comment About Test Compilation Guards

**File:** `test_zenoh_ipc.cpp`
**Severity:** Low (documentation)
**Bug:** Header comment said "Topic mapping + factory (always compiled)" but topic mapping tests are inside `#ifdef HAVE_ZENOH`, so they're not always compiled.
**Fix:** Updated comment to accurately categorize the three test groups: (1) factory tests — always compiled, (2) topic mapping — HAVE_ZENOH, (3) pub/sub round-trips — HAVE_ZENOH.

### R6 — Flaky Fixed-Delay Tests (CI Risk)

**File:** `test_zenoh_ipc.cpp`
**Severity:** High (CI reliability)
**Bug:** All round-trip tests used fixed `sleep_for(100ms)` / `sleep_for(200ms)` delays for Zenoh discovery and message delivery. Under CI load, these could be insufficient, causing false failures.
**Fix:** Replaced all fixed delays with two polling helpers:
- `poll_receive()` — polls subscriber with 5 ms intervals, bounded by a 5 s timeout
- `publish_until_received()` — retransmits every 50 ms until the subscriber receives, bounded by a 5 s timeout (handles discovery window during which messages are dropped)

### R7 — Missing `#include <memory>` in `zenoh_publisher.h` (Not Applicable)

**File:** `zenoh_publisher.h`
**Severity:** N/A
**Review suggestion:** Add `#include <memory>` for `std::unique_ptr` / `std::make_unique`.
**Resolution:** Verified not needed — the file uses `std::optional` (not `unique_ptr`). The related `zenoh_message_bus.h` already includes `<memory>`.

### R8 — Missing `#include <chrono>` in `zenoh_subscriber.h`

**File:** `zenoh_subscriber.h`
**Severity:** Low (portability)
**Bug:** `std::chrono::steady_clock` and `std::chrono::duration_cast` were used in `on_sample()` without `#include <chrono>`, relying on transitive include.
**Fix:** Added explicit `#include <chrono>`.

### R9 — Unauthenticated Zenoh Sessions (Security Risk)

**File:** `CMakeLists.txt`
**Severity:** Critical (security)
**Bug:** `ENABLE_ZENOH` created a Zenoh session with `Config::create_default()` — no TLS, no authentication. `fc_commands` and `gcs_commands` would be exposed unencrypted over the network.
**Fix:** Added two CMake options:
- `ZENOH_CONFIG_PATH` — path to a secure Zenoh configuration file (TLS + mutual auth)
- `ALLOW_INSECURE_ZENOH` — explicitly opt in to insecure builds (dev/test only)

Builds fail with `FATAL_ERROR` unless one of these is provided. CI uses `-DALLOW_INSECURE_ZENOH=ON`.

When `ZENOH_CONFIG_PATH` is set, it's compiled in as `ZENOH_CONFIG_PATH="/path/to/config"` — `ZenohSession` can use it to load the config at runtime.

---

## PR #53 Review Fixes — Zenoh Phase B (Issue #47)

**Date:** 2026-02-28
**PR:** [#53](https://github.com/nmohamaya/companion_software_stack/pull/53)

### R1 — Backend-Specific Error Message in Perception

**File:** `process2_perception/src/main.cpp`
**Severity:** Low (UX)
**Bug:** Error message said "Cannot connect to video SHM" — misleading when using the Zenoh backend.
**Fix:** Changed to backend-agnostic wording: "Cannot connect to video channel".

### R2 — GCS Subscribe Ordering Causes SHM Regression

**File:** `process4_mission_planner/src/main.cpp`
**Severity:** Medium (SHM regression)
**Bug:** `bus_subscribe_optional` for GCS was placed before the blocking `bus_subscribe` for FC state. On the SHM backend, the blocking subscribe's retry window (~10 s) gives `comms` time to create all segments. Doing the optional GCS subscribe first meant it was attempted before `comms` had started, always failing to connect.
**Fix:** Reordered: FC-state blocking subscribe first (provides the wait window), then GCS optional subscribe (benefits from the delay).

### R3 — Misleading `bus_subscribe_optional` Comment

**File:** `common/ipc/include/ipc/message_bus_factory.h`
**Severity:** Low (documentation)
**Bug:** Doc comment said "For Zenoh: always connects" — misleading. It implies the function actively does something special for Zenoh.
**Fix:** Expanded to explain per-backend `is_connected()` semantics: SHM returns false if segment doesn't exist yet; Zenoh always returns true because subscriptions are asynchronous.

### R4 — HighRate_Pose Test Infinite Loop

**File:** `tests/test_zenoh_ipc.cpp`
**Severity:** High (test correctness)
**Bug:** The assertion used `while (sub.receive(last))` to drain buffered messages. But `ZenohSubscriber::receive()` is non-consuming — it returns the latest value without clearing it, so `has_data_` stays true. This created an infinite loop.
**Fix:** Changed to poll once and track the highest timestamp seen across all received messages. Asserts that the final published message (ts=400) was delivered.

### R5 — Cross-Test Coupling in FactorySubscribeOptional

**File:** `tests/test_zenoh_ipc.cpp`
**Severity:** Medium (test isolation)
**Bug:** Test used `shm_names::GCS_COMMANDS` as the key expression. If an earlier test in the same process published on that topic, the subscriber could receive stale data, making the test order-dependent.
**Fix:** Use a PID-unique key expression (`/factory_opt_test_<pid>`) to guarantee isolation.

### R6 — Missing `#include <algorithm>`

**File:** `tests/test_zenoh_ipc.cpp`
**Severity:** Low (portability)
**Bug:** `std::min` was used without `#include <algorithm>`, relying on a transitive include.
**Fix:** Added explicit `#include <algorithm>`.

### R7 — Invalid Duplicate JSON Key in Documentation

**File:** `docs/ipc-key-expressions.md`
**Severity:** Low (documentation)
**Bug:** JSON example defined `ipc_backend` twice — invalid JSON syntax.
**Fix:** Collapsed to a single key with a comment showing both options.

---

## Fix #9 — ZenohSubscriber::is_connected() Semantic Mismatch (E2E Crash)

**Date:** 2026-03-01  
**Severity:** Critical  
**Files:** `common/ipc/include/ipc/zenoh_subscriber.h`, `common/ipc/include/ipc/zenoh_message_bus.h`, `tests/test_zenoh_ipc.cpp`

**Bug:** When running all 7 processes with `ipc_backend: "zenoh"`, processes 2 (perception), 4 (mission_planner), and 6 (payload_manager) crashed immediately on startup. Each process's `main()` calls `is_connected()` on mandatory subscriptions right after `subscribe()` and exits with `return 1` if it returns false. Under the Zenoh backend, `is_connected()` always returned false at that point because no data had arrived yet.

**Root Cause:** `ZenohSubscriber::is_connected()` was implemented as `return has_data_.load()`, which only becomes true after the first message is received via the asynchronous callback. This matched SHM semantics (where `is_connected()` checks whether the shared memory segment exists, i.e., the publisher has started), but not Zenoh semantics. In Zenoh, a subscription is immediately valid upon declaration — data arrives asynchronously once a publisher is discovered via the Zenoh discovery protocol. There is no equivalent of "segment doesn't exist yet".

**Failed Fix Attempt (Retry Polling):** Initially tried adding a retry loop in `ZenohMessageBus::subscribe()` that polled for data arrival (up to 10 seconds, 50 ms intervals). This caused a **circular deadlock**: `comms` subscribes to `trajectory` (published by `mission_planner`) and waits, while `mission_planner` subscribes to `fc_state` (published by `comms`) and waits. Neither process could start publishing because both were blocked in their subscribe calls.

**Fix:** Changed `is_connected()` to return `subscriber_.has_value()` — i.e., true if the Zenoh subscriber object was successfully declared, regardless of whether any data has arrived. This correctly models Zenoh's always-connected subscription semantics. Updated two unit tests (`ZenohSubscriber.Constructs` and `ZenohPubSub.NoData`) to expect `is_connected() == true` after construction.

**Key Insight:** SHM and Zenoh have fundamentally different connection models:
- **SHM:** Publisher creates a named shared memory segment. Subscriber opens it. `is_connected()` = "does the segment exist?" (depends on publisher being alive).
- **Zenoh:** Subscriber declares interest in a key expression. Publisher declares a matching key expression. Zenoh's discovery protocol handles matching asynchronously. `is_connected()` = "was the subscription declared?" (always true after `subscribe()`).

**Found by:** End-to-end Zenoh smoke test (`tests/test_zenoh_e2e.sh`) — Phase 3 (process liveness) detected that 3 of 7 processes exited within the first second.

---

## Fix #10 — Intermittent Zenoh Test Failures Under Parallel CTest

**Date:** 2026-03-02  
**Severity:** High  
**File:** `tests/CMakeLists.txt`

**Bug:** Running `ctest -j$(nproc)` intermittently caused 1–2 Zenoh tests to fail with "Subprocess aborted" (SIGABRT). The specific failing tests varied between runs — e.g., `ZenohMessageBus.SubscribeLazyCreatesSubscriber`, `ZenohShmPublish.FactoryVideoRoundTrip`, `ShmIPCTest.ReadBeforeWrite`, etc. Every failing test passed when run individually.

**Root Cause:** Three Zenoh test binaries (`test_zenoh_ipc`, `test_zenoh_network`, `test_zenoh_liveliness`) each open their own Zenoh sessions and allocate 32 MB SHM pools. When `ctest -j$(nproc)` ran them all simultaneously, the combined resource usage (multiple concurrent Zenoh sessions, SHM pool allocations, Zenoh router discovery) exceeded system limits, triggering SIGABRT in the zenohc runtime. The `test_shm_ipc` binary also shares `/drone_test_shm` POSIX SHM segments that could collide under parallel execution.

**Reproduction:** 3 consecutive `ctest -j$(nproc)` runs: Run 1 failed (2 tests), Runs 2–3 passed 377/377 — confirming the failure was intermittent and load-dependent.

**Failed Fix Attempt:** Initially added `set_tests_properties()` with `RESOURCE_LOCK` listing individual test names (e.g., `ZenohRoundTrip.PoseRoundTrip`). This failed because `gtest_discover_tests()` generates test names at **build time**, not **configure time** — the names used in `set_tests_properties()` didn't match the actual discovered test names, so the locks were silently ignored.

**Fix:** Extended the `add_drone_test()` CMake function to accept an optional `PROPERTIES` keyword argument, forwarded directly to `gtest_discover_tests()`:

```cmake
function(add_drone_test TEST_NAME)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "PROPERTIES")
    # ... target setup ...
    if(ARG_PROPERTIES)
        gtest_discover_tests(${TEST_NAME} PROPERTIES ${ARG_PROPERTIES})
    else()
        gtest_discover_tests(${TEST_NAME})
    endif()
endfunction()
```

Applied `RESOURCE_LOCK` to 4 test targets:

| Target | Lock Name | Reason |
|--------|-----------|--------|
| `test_shm_ipc` | `posix_shm` | Shares `/drone_test_shm` POSIX SHM segment |
| `test_zenoh_ipc` | `zenoh_session` | Opens Zenoh session + 32 MB SHM pool |
| `test_zenoh_network` | `zenoh_session` | Opens Zenoh session for network tests |
| `test_zenoh_liveliness` | `zenoh_session` | Opens Zenoh session for liveliness tests |

All three Zenoh binaries share the same `"zenoh_session"` lock, so CTest never runs them simultaneously. All other tests (306 of 377) continue to run fully in parallel.

**Verification:** 3 consecutive `ctest -j$(nproc)` runs after the fix — 377/377 passed on all 3 runs.

**Key Insight:** `gtest_discover_tests()` creates test entries at build time (via a POST_BUILD step that runs the binary with `--gtest_list_tests`). Any `set_tests_properties()` call at configure time with test names will silently fail if the names don't match exactly. The correct approach is to pass `PROPERTIES` directly to `gtest_discover_tests()`, which applies them to all discovered tests from that binary.

**Found by:** `deploy/clean_build_and_run_zenoh.sh` aborting during the test phase (`set -e` + `ctest -j$(nproc)` failure).

---

## Fix #11 — ThreadSanitizer Crashes on Linux Kernel 6.x (ASLR Entropy)

**Date:** 2026-03-03  
**Severity:** Medium (CI / tooling)  
**Affects:** TSan CI leg, local TSan builds on kernel 6.x+

**Bug:** Building with `-fsanitize=thread` (TSan) and running any test binary immediately crashed with:

```
FATAL: ThreadSanitizer: unexpected memory mapping 0x61a3f0509000-0x61a3f0513000
```

The crash occurred at gtest discovery time (`gtest_discover_tests` runs the binary with `--gtest_list_tests` during build), so the entire CMake build failed — not just test execution.

**Root Cause:** Linux kernel 6.x increased the default Address Space Layout Randomization (ASLR) entropy (`vm.mmap_rnd_bits`) beyond what TSan's shadow memory layout can handle. TSan maps a fixed shadow region at compile time and expects userspace memory to fall within a predictable range. Higher ASLR entropy pushes `mmap` allocations outside that range, causing an immediate fatal error.

- Kernel 5.x default: `vm.mmap_rnd_bits = 28` → works with TSan
- Kernel 6.x default: `vm.mmap_rnd_bits = 32` (or higher) → breaks TSan

This is a [known upstream issue](https://github.com/google/sanitizers/issues/1716) affecting GCC and Clang sanitizers on newer kernels.

**Workaround:** Reduce ASLR entropy before building/running TSan-instrumented binaries:

```bash
sudo sysctl vm.mmap_rnd_bits=28
```

This is applied automatically in the CI pipeline via a dedicated step:

```yaml
- name: Fix ASLR for TSan
  if: matrix.sanitizer == 'tsan'
  run: sudo sysctl vm.mmap_rnd_bits=28
```

**Additional Note:** Even after fixing the ASLR issue, TSan produces false positives in tests that exercise **external libraries** (Zenoh, MAVSDK, OpenCV YOLO) because their internal threading is not compiled with TSan instrumentation. The CI TSan leg therefore excludes these tests:

```yaml
ctest --output-on-failure -j$(nproc) -E "Zenoh|Mavlink|Yolo|Liveliness"
```

This exclusion is safe — the TSan leg still runs all 277 core tests (SHM IPC, SPSC ring, FaultManager, MissionFSM, config, HAL, wire format, etc.), which is where TSan provides the most value for detecting data races in **our** code.

**Found by:** `cmake --build build` failing during gtest discovery on kernel 6.17 with `ENABLE_TSAN=ON`.

---

## Fix #12 — ProcessManager `restart_count` Never Incremented

**Date:** 2026-03-04  
**Severity:** High (correctness — supervisor never stops restarting a failing process)  
**Affects:** `process7_system_monitor/include/monitor/process_manager.h`  
**PR:** #101 (Phase 3 — Issue #91)

**Bug:** The `RestartPolicy.max_restarts` limit was never enforced. A crashing child process would be restarted indefinitely instead of being marked `FAILED` after N attempts.

**Root Cause:** In `launch_one()`, the process state was set to `RUNNING` *before* the condition that checked `if (proc.state == ProcessState::RESTARTING)` to increment `restart_count`. Since the state was already `RUNNING` by the time the check ran, the condition was always false and the counter stayed at 0.

```cpp
// BEFORE (broken):
proc.state = ProcessState::RUNNING;       // ← state set first
if (proc.state == ProcessState::RESTARTING) {  // ← always false!
    proc.restart_count++;
}

// AFTER (fixed):
if (proc.state == ProcessState::RESTARTING) {  // ← check while still RESTARTING
    proc.restart_count++;
}
proc.state = ProcessState::RUNNING;       // ← state set after increment
```

**Impact:** Without this fix, `max_restarts=5` would be silently ignored. A flapping process (crash loop) would restart forever, consuming system resources and masking the real failure.

**Found by:** `ProcessManagerTest.MaxRestartsExhausted` — test waited for state `FAILED` but it never arrived. Process stayed in a perpetual RESTARTING→RUNNING→crash→RESTARTING cycle.

---

## Fix #13 — uint64_t Underflow in ProcessManager Cooldown Calculation

**Date:** 2026-03-04  
**Severity:** Medium (correctness — restart counter resets prematurely)  
**Affects:** `process7_system_monitor/include/monitor/process_manager.h`  
**PR:** #101 (Phase 3 — Issue #91)

**Bug:** After a restart, the `restart_count` was immediately reset to 0 (as if the process had been stable for 60 seconds), defeating the max-restart protection on fast crash loops.

**Root Cause:** Two interacting issues in `tick()`:

1. **Stale `now_ns`:** A single `now_ns` was captured at the top of `tick()`, but `launch_one()` (called during the RESTARTING→RUNNING transition) set `started_at_ns` to a *fresh* `steady_clock::now()`. This meant `now_ns < started_at_ns`, and the subtraction `now_ns - started_at_ns` on `uint64_t` wrapped to `~18 quintillion nanoseconds` — far exceeding the 60-second cooldown window.

2. **Missing `continue`:** After the RESTARTING block launched the process, execution fell through to the cooldown check on the same iteration, using the stale `now_ns`.

```cpp
// BEFORE (broken):
uint64_t now_ns = current_time();   // captured ONCE at top of tick
for (auto& proc : processes_) {
    if (proc.state == RESTARTING) {
        launch_one(proc);           // sets started_at_ns = NOW (> now_ns)
        // falls through to cooldown check below! ↓
    }
    uint64_t stable = now_ns - proc.started_at_ns;  // underflow!
    if (stable > cooldown_ns) proc.restart_count = 0; // premature reset!
}

// AFTER (fixed):
for (auto& proc : processes_) {
    if (proc.state == RESTARTING) {
        launch_one(proc);
        continue;  // ← skip cooldown check this iteration
    }
    uint64_t fresh_now = current_time();  // ← fresh per-process
    if (fresh_now > proc.started_at_ns) { // ← guard against underflow
        uint64_t stable = fresh_now - proc.started_at_ns;
        if (stable > cooldown_ns) proc.restart_count = 0;
    }
}
```

**Impact:** Combined with Fix #12, this would have allowed a crash-looping process to restart forever — the counter incremented correctly (Fix #12) but was reset to 0 on the very next tick (this bug).

**Found by:** `ProcessManagerTest.MaxRestartsExhausted` — process reached `restart_count=2` (correct after Fix #12) but then reset to 0 and continued restarting instead of reaching `FAILED`.

---

## Fix #14 — DetectCrash Test Flaky Due to SIGSEGV + Core Dump Latency

**Date:** 2026-03-04  
**Severity:** Low (test reliability, not production code)  
**Affects:** `tests/test_process_manager.cpp`  
**PR:** #101 (Phase 3 — Issue #91)

**Bug:** `ProcessManagerTest.DetectCrash` intermittently failed — the test expected the death callback to have fired after a fixed 300ms sleep, but SIGSEGV delivery + core dump I/O sometimes took 500ms–1.5s.

**Root Cause:** When a child process crashes via `SIGSEGV`, the kernel may write a core dump before the zombie becomes reapable via `waitpid`. Core dump I/O (writing the process memory image to disk) adds significant latency beyond signal delivery itself. On a loaded system or with large `ulimit -c`, this routinely exceeds 300ms. A clean `exit(1)` completes in <50ms, but a signal-killed process is much slower.

```cpp
// BEFORE (flaky):
mgr.launch("crasher");
std::this_thread::sleep_for(std::chrono::milliseconds{300});  // not enough!
mgr.tick();
EXPECT_TRUE(signaled);  // sometimes false

// AFTER (robust):
mgr.launch("crasher");
for (int i = 0; i < 20 && !signaled; ++i) {       // poll up to 2s
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    mgr.tick();
}
EXPECT_TRUE(signaled);  // reliably true (exits early, typically ~200ms–1.3s)
```

**Impact:** No production impact — the supervisor's `tick()` is called in a loop anyway. This was purely a test timing issue. The polling pattern is both more robust (2s max) and faster in the common case (exits as soon as `waitpid` succeeds).

**Found by:** `ProcessManagerTest.DetectCrash` failing on first test run. Subsequent analysis showed the death callback fired at ~1.3s, well past the 300ms window.
