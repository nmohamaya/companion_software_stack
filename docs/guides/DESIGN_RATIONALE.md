# Design Rationale

Gray-area decisions where both sides are defensible. Each entry captures the question, competing arguments, our decision, and when to revisit. These are not formal ADRs — they're lightweight records of judgment calls that future contributors (human or AI) will encounter.

---

## DR-001: Interface Tests Should Verify Contracts, Not Concrete Types

**Question:** When testing an abstract interface (ILogger, IClock, etc.), should we `dynamic_cast` to assert the concrete implementation type?

**Arguments for asserting type:**
- Catches accidental changes to the default implementation
- Documents which concrete class is wired up in production
- Fails loudly if someone swaps the default without updating tests

**Arguments against:**
- Couples the test to an implementation detail — the whole point of the interface is that the concrete backend can change
- A test that breaks on a legitimate refactor (e.g., swapping `SpdlogLogger` for `BufferedLogger`) is a false negative
- The interface contract is "logging works" not "logging is spdlog"

**Our decision:** Test the contract (calling `DRONE_LOG_INFO` doesn't crash, output is produced), not the wiring. If we need to verify specific backend behavior, write a dedicated backend-specific integration test.

**When to revisit:** If a default-swap bug slips through and causes a production incident, consider adding a separate "wiring test" that asserts the default type — but keep it outside the interface test suite.

**Date:** 2026-04-09

---

## DR-002: Confidence Tiers vs. Continuous Covariance for Depth Estimation

**Question:** Should the UKF fusion engine assign discrete depth_confidence tiers (0.01, 0.3, 0.6, 0.7, 1.0) or compute continuous covariance from the measurement model?

**Arguments for discrete tiers:**
- Simple to reason about and debug — "Tier 2 = 0.7" is immediately understandable in logs
- Easy to configure thresholds in the occupancy grid (e.g., `min_promotion_depth_confidence: 0.8`)
- Works well as a gate: camera-only (max 0.7) vs. radar-confirmed (1.0)
- Lower implementation risk in safety-critical code

**Arguments for continuous covariance:**
- Physically correct — depth variance from apparent-size grows as `(H * fy / h^2)^2 * sigma_h^2`, quadratically with range
- Eliminates arbitrary tier boundaries (why is 0.6 "horizon-truncated" but 0.7 is "apparent-size"?)
- UKF already uses covariance internally — the tiers are a lossy projection of information the filter already has
- Better fusion: the Kalman gain naturally weights radar more at long range, camera more at short range

**Our decision:** Keep discrete tiers for now. They provide clear, auditable gates for grid promotion that are easy to tune per-scenario. The occupancy grid's promotion logic (`depth_confidence >= threshold`) is simpler with discrete values.

**When to revisit:** When we implement Phase 1 of Issue #393 (monocular depth improvements). Covariance-weighted fusion (method 4.2 in that analysis) is the natural successor — it replaces tiers with continuous sigma, which the UKF can consume directly. The tiers were the right starting point; continuous covariance is the right destination.

**Date:** 2026-04-09

---

## DR-003: prediction_enabled Defaults to False Despite Being Useful

**Question:** The occupancy grid's velocity-based prediction (extrapolating detected objects forward by `prediction_dt_s`) is conceptually valuable — why default it to off?

**Arguments for enabling by default:**
- Pre-fills grid cells where moving obstacles will be, giving D* Lite early awareness
- Correct behavior for genuine moving targets (vehicles, people)
- The feature was built for a reason (Issue #256)

**Arguments for disabling by default:**
- Camera-only velocity estimates are dominated by depth-estimation noise on static objects
- In scenario 02, produced 3753 phantom prediction cells from just 3 stationary obstacles
- Radar gives true radial velocity, but camera frame-to-frame depth differencing is ~50% noise
- The prediction amplifies upstream errors: noisy depth → noisy velocity → massive phantom cell spray
- Safer to require opt-in after verifying velocity source quality

**Our decision:** Default to `false` in scenario configs. Enable only when radar velocity is confirmed reliable and the environment actually has moving obstacles. The feature is correct in theory but the input quality (camera-only velocity) isn't there yet.

**When to revisit:** After implementing multi-frame depth smoothing (Issue #393, method 5.1) and/or MDE networks that provide stable frame-to-frame depth. Once velocity estimation noise drops below ~20%, predictions become net-positive.

**Date:** 2026-04-09

---

## DR-004: UKF Over EKF for Multi-Sensor Fusion

**Question:** Why use an Unscented Kalman Filter instead of an Extended Kalman Filter for camera+radar fusion?

**Arguments for EKF:**
- Simpler implementation (Jacobian matrices instead of sigma points)
- Less compute per update (no sigma point propagation)
- Well-understood, decades of flight heritage
- Easier to debug (linear approximation is explicit)

**Arguments for UKF:**
- Camera measurement model (pinhole unprojection + bearing-to-depth) is highly nonlinear — EKF's first-order Taylor linearization introduces systematic bias
- Radar measurement model (spherical → Cartesian) has trigonometric nonlinearities that UKF handles better
- No Jacobian derivation needed — sigma point propagation handles arbitrary nonlinearities
- Better covariance estimation for non-Gaussian distributions (depth uncertainty is not symmetric)
- 6D state `[x, y, z, vx, vy, vz]` with mixed sensor modalities is a textbook UKF use case

**Our decision:** UKF. The measurement models are nonlinear enough that EKF linearization error would degrade fusion quality, especially at the bearing-to-depth conversion where small angular errors produce large position errors at range. The compute overhead (~2x EKF) is negligible on Jetson Orin relative to the perception pipeline.

**When to revisit:** If we move to a factor graph approach (Issue #393, method 4.3 / GTSAM), the UKF gets replaced entirely. Otherwise, UKF remains the right choice for per-object tracking.

**Date:** 2026-04-09

---

## DR-005: Mutex vs. Atomic vs. Lock-Free — Concurrency Tiering

**Question:** When should we use `std::mutex` vs `std::atomic` vs lock-free data structures?

**Arguments overlap — the question is where to draw the lines:**

**Lock-free (`std::atomic` with explicit memory ordering):**
- **Use for:** Single flags, counters, or timestamps on hot paths (>1kHz access)
- **Examples:** `ThreadHeartbeat` touch (lock-free, ~1ns), inter-thread shutdown flags, `clock_ptr` in IClock global accessor
- **Why:** Zero contention cost; no priority inversion risk; suitable for real-time threads
- **Danger:** Easy to get wrong. Always specify `memory_order_acquire/release` explicitly — never use `relaxed` unless you can prove no synchronization is needed

**Mutex (`std::lock_guard<std::mutex>`):**
- **Use for:** Multi-field state updates, config access, ring buffer operations, any shared state touched <100Hz
- **Examples:** Config reload, occupancy grid bulk updates, IPC session management, `set_clock()` ownership transfer
- **Why:** Correct by construction; protects multi-field invariants that atomics can't; RAII with `lock_guard` prevents deadlocks
- **Danger:** Priority inversion on real-time threads; never hold across blocking I/O

**Avoid entirely:**
- Recursive mutexes (restructure code instead — they mask design problems)
- `memory_order_relaxed` without written justification
- Bare `lock()`/`unlock()` calls (always use `lock_guard`/`unique_lock`)
- `std::shared_mutex` for read-heavy patterns (the overhead rarely pays off in our access patterns)

**Our decision:** Use the simplest mechanism that provides correct synchronization. Default to mutex. Only reach for atomics on measured hot paths. Document why when using non-default memory ordering.

**When to revisit:** If profiling shows mutex contention on a specific path (unlikely given our thread count), consider lock-free alternatives for that specific case.

**Date:** 2026-04-09

---

## DR-006: HD-Map Proximity Suppression vs. Exact Cell Match for Radar Promotion

**Question:** When radar confirms an obstacle near a pre-mapped HD-map position, should we suppress promotion by exact cell match or by proximity (Chebyshev neighborhood)?

**Arguments for exact cell match:**
- Simpler logic — `if (hd_map_cells_.count(cell)) skip`
- No false suppression of genuinely new obstacles near HD-map entries
- Deterministic and easy to test

**Arguments for proximity (Chebyshev-1 neighborhood, 3x3 grid):**
- Radar range has noise (~0.3m std) and camera bearing has noise (~1.5 deg)
- A real obstacle at HD-map position (10, 10) might be detected at grid cell (11, 10) due to sensor noise
- Without proximity, the noisy detection bypasses suppression and creates a duplicate promoted cell adjacent to the HD-map cell
- Chebyshev-1 (one cell in any direction) matches the expected sensor noise at typical grid resolution (2m)

**Our decision:** Chebyshev-1 proximity (`near_hd_map_cell_()` checks 3x3 neighborhood). The sensor noise argument is decisive — exact match would fail to suppress ~30-40% of legitimate re-detections of known obstacles, causing duplicate static cells.

**When to revisit:** If grid resolution decreases (e.g., 1m cells), Chebyshev-1 might be too tight. If resolution increases (e.g., 4m cells), it might be too loose. The neighborhood radius should scale with `ceil(sensor_noise_m / resolution_m)`. Filed as part of Issue #390 (HD-map invalidation architecture).

**Date:** 2026-04-09

---

## DR-007: Waypoint Acceptance Against Original vs. Snapped Position

**Question:** When `snap_goal()` relocates an occupied waypoint, should the FSM check if the drone reached the original waypoint or the snapped position?

**Arguments for checking original position:**
- The mission plan specifies exact coordinates — the pilot/operator intended the drone to go there
- Snapped position is a planner implementation detail, not a mission objective
- If the obstacle clears (dynamic cells expire), the drone should still try the original position

**Arguments for checking snapped position:**
- If the original position is occupied, the drone physically cannot reach it
- When snap offset > acceptance_radius, the waypoint becomes unreachable by definition (Bug #394)
- The drone arrives at the snapped position and sits there forever — mission failure
- The overshoot detector sometimes saves it by geometry luck, but this is fragile and non-deterministic
- In the real world, a GPS coordinate on top of a building should not cause an infinite hover

**Our decision:** This is an open bug (#394). The current code checks the original position, which causes mission failure when snap offset exceeds acceptance_radius. The recommended fix (Option A in #394) is to check the snapped position — the drone should be considered "at the waypoint" when it reaches where we actually sent it.

**When to revisit:** When implementing #394. Consider a hybrid: accept the snapped position, but log a warning if the snap offset exceeds a configurable threshold so the operator knows the mission deviated from plan.

**Date:** 2026-04-09

---

## DR-008: Conservative depth_scale (0.7) — Placing Obstacles Closer Than Estimated

**Question:** The depth formula multiplies by `depth_scale = 0.7`, placing detected obstacles 30% closer than the geometric estimate. Is this safety margin justified?

**Arguments for conservative scaling (< 1.0):**
- Monocular depth is inherently unreliable (~30% error at 10m) — systematic underestimation is safer than overestimation
- An obstacle placed too far away might not trigger avoidance in time
- An obstacle placed too close causes unnecessary avoidance but no collision
- In safety-critical systems, false positives (avoid phantom obstacle) are acceptable; false negatives (miss real obstacle) are not

**Arguments against:**
- 30% bias distorts the world model — downstream planning makes decisions on incorrect positions
- Velocity estimation from frame-to-frame depth differencing is amplified by the bias (contributing to the prediction flooding problem)
- Radar-confirmed objects don't need the safety margin (they have authoritative range)
- The occupancy grid's inflation radius already provides a safety buffer — stacking a depth bias on top is double-counting

**Our decision:** Keep `depth_scale = 0.7` for camera-only estimates. The asymmetric cost of errors (collision vs. detour) justifies the bias. However, radar-confirmed tracks should use `depth_scale = 1.0` since radar range is authoritative. Currently, depth_scale applies uniformly — separating it per-source is part of Issue #393 Phase 1.

**When to revisit:** When MDE networks (Depth Anything V2, RTS-Mono) replace geometric depth estimation. Neural depth with ~5-8% error no longer needs a 30% safety margin — reduce to 0.9 or 1.0 with the inflation radius providing the remaining buffer.

**Date:** 2026-04-09
