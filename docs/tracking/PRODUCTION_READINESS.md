# Production Readiness — Items to Address Before Real Hardware Deployment

> Track items that are acceptable in simulation but must be resolved before
> deploying to real drone hardware. Each item has a severity, current state,
> and what needs to change.

---

## Concurrency: Priority Inversion Risk

**Severity:** Medium (latency spikes, not data corruption)
**Current state:** Mutex used for cold-path state transitions in HAL backends (open/close/init/shutdown). Hot-path frame handoff in CosysCameraBackend uses lock-free TripleBuffer. Radar/IMU backends still use mutex for poll data reads at 20-200 Hz.

**What to address:**
- On real-time Linux (Jetson Orin with PREEMPT_RT), a low-priority thread holding a mutex can block a high-priority thread (priority inversion). This can cause latency spikes in the perception pipeline if the sensor read thread is preempted while holding the data mutex.
- **Radar (20 Hz):** Consider migrating to TripleBuffer for `RadarDetectionList` handoff. Requires making `RadarDetectionList` movable (it's a struct with a vector — already movable).
- **IMU (200 Hz):** `ImuReading` is ~56 bytes (6 floats + timestamp). Could use `std::atomic` if padded to 64 bytes (one cache line), or TripleBuffer. At 200 Hz the mutex overhead is negligible (~20ns vs 5ms period), but priority inversion is the concern, not overhead.
- **Camera:** Already uses lock-free TripleBuffer — no action needed.
- **Depth:** On-demand (not polled) — mutex is fine.

**Mitigation options for production:**
1. `pthread_mutexattr_setprotocol(PTHREAD_PRIO_INHERIT)` — priority inheritance prevents unbounded inversion. Wrap `std::mutex` with a custom mutex class that sets this attribute.
2. Migrate radar/IMU to TripleBuffer (eliminates the mutex entirely on the hot path).
3. Use `SCHED_FIFO` with appropriate priorities for sensor threads (requires RT kernel).

**When to address:** Before Tier B (real hardware) deployment. Not needed for Tier 1-3 simulation.

---

## TSan: Zenoh False Positives

**Severity:** Low (false positives, not real races)
**Current state:** 6 test patterns excluded from TSan: `Zenoh|Mavlink|Yolo|Liveliness|MessageBus|TopicResolver`. All are caused by Zenoh's internal threading in `libzenohc.so`, not our code.

**What to address:**
- Build zenohc from source with TSan annotations (issue #387) to distinguish real races from library internals.
- Or obtain a TSan suppression file from Eclipse Zenoh project.
- Our code is correctly synchronized (verified by code review — all Zenoh calls are under mutex or atomic guards).

**When to address:** Before production deployment. Not blocking simulation work.

---

## DIAG Logging: Gate Before Production

**Severity:** Low (performance, not correctness)
**Current state:** Some per-tick diagnostic logging uses `spdlog::info` instead of `spdlog::debug`. See `project_production_debug_cleanup` memory for full list.

**What to address:** Audit all `DRONE_LOG_INFO` in hot paths, gate behind `spdlog::debug` or `spdlog::should_log()`.

**When to address:** Before production deployment.
