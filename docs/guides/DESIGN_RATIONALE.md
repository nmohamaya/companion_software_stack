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

---

## DR-009: PluginRegistry dlclose in Static Destructor vs. Intentional Leak

**Question:** `PluginRegistry` is a Meyer's singleton that calls `dlclose()` on all stored handles in its destructor (static destruction). Copilot suggested intentionally leaking the handles instead, since static destruction order across translation units is unspecified — a plugin instance in another static could outlive the registry, causing vtable UAF.

**Arguments for dlclose in static destructor (current approach):**

- Clean resource release — valgrind/sanitizers don't report leaks
- LIFO ordering (reverse of load order) is the standard pattern for dlclose
- Plugin instances are documented to be held in `main()` locals or objects owned by `main()`, never in static/global variables — this is an enforced contract, not a hope
- The codebase already follows this pattern: all HAL factory results go into process `main()` locals
- If a plugin instance violates the contract (held in a static), the bug is in the violating code, not the registry

**Arguments for intentional leak:**

- Eliminates the static destruction order fiasco entirely — zero risk regardless of how callers hold instances
- `dlclose` at process exit is largely pointless — the OS reclaims everything anyway
- Some frameworks (e.g., LLVM plugin system) intentionally leak for this reason
- Defensive programming: protect against future callers who don't read the contract

**Our decision:** Keep dlclose in the destructor with LIFO ordering. The contract (plugin instances must not outlive the registry) is documented in the header, enforced by code review, and consistent with the codebase's RAII philosophy. Intentional leaks normalize resource mismanagement and hide bugs that should be caught early.

**When to revisit:** If a use-after-free is traced to static destruction ordering with plugin handles. At that point, the leak approach becomes justified as a pragmatic defense. Also revisit if we add a plugin reload mechanism (unload + reload .so at runtime), where dlclose becomes functionally necessary.

**Date:** 2026-04-13

---

## DR-010: SHM Threshold Based on sizeof(T) vs. serialized_size()

**Question:** `ZenohPublisher` decides whether to use the SHM zero-copy path based on `sizeof(T) > kShmPublishThreshold` (a compile-time check). Copilot noted this will be incorrect for variable-length serializers (e.g., protobuf) where wire size differs from `sizeof(T)`.

**Arguments for sizeof(T) (current approach):**

- Compile-time decision — the `if constexpr` branch is resolved at compile time, so the unused path is eliminated entirely (no dead code, no branch misprediction)
- With `RawSerializer<T>` (the only serializer today), `sizeof(T) == serialized_size(msg)` is an invariant — there's no divergence
- Switching to runtime `serialized_size()` would require a runtime branch on every publish, adding overhead to the hot path for a capability we don't use yet
- The SHM path is specifically optimized for large fixed-size structs (VideoFrame ~6.2 MB) — variable-length messages may not benefit from SHM the same way

**Arguments for serialized_size() (runtime check):**

- Correct for future variable-length serializers (protobuf, flatbuffers)
- A protobuf message with `sizeof(T)` = 64 bytes but wire size = 2 KB would never hit the SHM path despite benefiting from it
- Conversely, a sparse protobuf with `sizeof(T)` = 6 MB but wire size = 100 bytes would wastefully allocate SHM

**Our decision:** Keep `sizeof(T)` with `if constexpr`. The compile-time optimization is valuable on the publish hot path, and the only serializer in use (`RawSerializer`) guarantees `sizeof(T) == serialized_size()`. When a variable-length serializer is added, this must be revisited — the SHM decision should then use `serialized_size()` with a runtime branch (the cost is justified when you're already doing non-trivial serialization).

**When to revisit:** When implementing `ProtobufSerializer<T>` or any serializer where wire size != `sizeof(T)`. File as part of that issue's acceptance criteria.

**Date:** 2026-04-13

---

## DR-011: Partial Mutation on Deserialize Failure (Non-Validate Branch)

**Question:** In `ZenohSubscriber::on_sample()`, the non-validate branch deserializes directly into `latest_msg_` under lock. If the serializer fails mid-write, `latest_msg_` may be partially mutated while `has_data_` remains true. Copilot suggested deserializing into a heap temporary (as the validate branch does) and only committing on success.

**Arguments for direct deserialization (current approach):**

- `RawSerializer::deserialize()` checks `size != sizeof(T)` **before** any copy — on size mismatch, zero bytes are written, so partial mutation cannot happen with the current serializer
- The validate branch uses `make_unique<T>()` because it needs the temporary for `validate()` anyway — the heap allocation is not purely defensive
- Adding a heap allocation to the non-validate path penalizes **every** message for every type, including small high-frequency types (Pose, FCState at 50-100 Hz) where the cost is measurable
- For large types (VideoFrame ~6.2 MB), the validate branch already handles them correctly
- The "partial mutation" scenario requires a serializer that writes some bytes before discovering failure — `RawSerializer` doesn't do this, and any future serializer can be designed to validate-before-write

**Arguments for heap temporary (defensive):**

- Future serializers (protobuf) may fail mid-deserialize, leaving `latest_msg_` in a torn state
- A subscriber reading `latest_msg_` immediately after a failed deserialize would see corrupted data
- The heap allocation cost (~100ns for typical IPC structs) is negligible compared to the Zenoh transport overhead (~10-50µs)
- Defense in depth: don't rely on serializer implementation details for correctness

**Our decision:** Keep direct deserialization for the non-validate branch. The current serializer guarantees atomic-or-nothing behavior (size check before copy). Added a code comment cross-referencing the validate branch's approach and explaining the design trade-off. When a variable-length serializer is added, this decision should be revisited — protobuf deserialize can fail mid-stream, making the heap temporary necessary.

**When to revisit:** When adding `ProtobufSerializer<T>` — at that point, the non-validate branch should adopt the heap-temporary pattern (or better, all types should go through a validate-like path since protobuf has built-in validation).

**Date:** 2026-04-13

---

## DR-013: -Werror in Docker Builds Despite CUDA System Header Warnings

**Question:** Copilot review (twice) suggested removing `-Werror` from the Docker CMake flags because CUDA devel base images may produce warnings from system headers that we don't control, breaking the build.

**Arguments for removing -Werror:**
- CUDA/cuDNN system headers may emit warnings (deprecated declarations, unused parameters) under `-Wall -Wextra`
- Docker builds should be maximally reliable — a new CUDA SDK release shouldn't break our build
- CI already enforces `-Werror` on the native build; Docker doesn't need to duplicate the gate

**Arguments for keeping -Werror (our approach):**
- The Docker image is a deployment artifact — it must meet the same quality bar as CI
- Silent warnings in Docker but not CI means the two build environments can diverge without anyone noticing
- If CUDA headers produce warnings, that's valuable signal — we should fix it (e.g., suppress specific warnings with `-Wno-*` for system headers) rather than ignore all warnings
- `-isystem` already applies to most system headers; any remaining warnings are likely in our code's use of CUDA APIs

**Our decision:** Keep `-Werror -Wall -Wextra` in the Docker build. If a CUDA SDK update introduces system header warnings, address them with targeted `-Wno-*` flags or `-isystem` adjustments rather than dropping `-Werror` entirely. The CI/Docker parity is more valuable than build convenience.

**When to revisit:** If a CUDA SDK update causes widespread system header warnings that cannot be suppressed with targeted flags, consider `-Werror` with an explicit allowlist of suppressed system-header warnings (e.g., `-Wno-deprecated-declarations` for CUDA headers only).

**Date:** 2026-04-14

---

## DR-012: CMAKE_DL_LIBS Propagation for ENABLE_PLUGINS

**Question:** When `ENABLE_PLUGINS=ON`, code using `dlopen/dlsym/dlclose` (via `plugin_loader.h`) needs to link against `libdl` on platforms where it's separate from libc. The current CMake only defines `HAVE_PLUGINS` without propagating `${CMAKE_DL_LIBS}`. Copilot suggested adding `link_libraries(${CMAKE_DL_LIBS})`.

**Arguments for deferring (current approach):**

- `ENABLE_PLUGINS` defaults to OFF — no current target enables it
- On glibc >= 2.34 (Ubuntu 22.04+, our primary platform), `dlopen` is in libc — no separate `-ldl` needed
- Adding `link_libraries()` at top-level scope is a blunt instrument — it links libdl into every target, including those that don't use plugins
- The proper fix (when needed) is to add `${CMAKE_DL_LIBS}` to specific targets (`drone_util` or `drone_hal`) via `target_link_libraries(... INTERFACE ${CMAKE_DL_LIBS})`, not a global `link_libraries()`

**Arguments for fixing now:**

- Cross-compilation targets (older ARM toolchains, musl-based systems) may have separate libdl
- If someone enables plugins and gets a linker error, the root cause is non-obvious
- One line of CMake now prevents a confusing debugging session later

**Our decision:** Defer. The feature is disabled by default, and our target platforms don't need it. When `ENABLE_PLUGINS` is first used in a real deployment (likely Jetson Orin with a custom detector backend), add `${CMAKE_DL_LIBS}` to the specific target at that time — not as a global link.

**When to revisit:** When any real target enables `ENABLE_PLUGINS=ON` and targets a platform with separate libdl (cross-compilation, musl, older glibc).

**Date:** 2026-04-13

---

## DR-014: ONNX Graph Surgery for Depth Anything V2 + OpenCV DNN Compatibility

**Question:** How should we handle the incompatibility between the Depth Anything V2 ONNX export and OpenCV DNN's Resize layer?

**Background:** DA V2's DINOv2 backbone uses bicubic interpolation and produces ONNX Resize nodes with 4 inputs (X, roi, scales, sizes) per the ONNX spec. OpenCV DNN (tested on 4.6.0 and 4.10.0) fails to load the 4-input `Resize(X, roi, scales, sizes)` form. Our workaround rewrites these to the 3-input `Resize(X, roi, scales)` form with precomputed constant scales, which OpenCV DNN accepts. It also does not support bicubic interpolation mode. This is a fundamental limitation — not a version gap we can upgrade past easily.

**Arguments for using ONNX Runtime instead of OpenCV DNN:**
- ORT supports the full ONNX spec natively — no graph surgery needed
- Better long-term compatibility as models evolve
- GPU acceleration via CUDA/TensorRT providers

**Arguments for keeping OpenCV DNN (our decision):**
- OpenCV is already a project dependency (used for detection, image processing) — no new dependency
- ONNX Runtime adds ~200MB to deployment image, plus CUDA runtime for GPU
- For our target (Jetson Orin), TensorRT is the production inference path — ORT would be a temporary stopgap anyway
- The graph surgery is deterministic and well-understood: replace 4-input Resize(X, roi, scales, sizes) with 3-input Resize(X, roi, scales) using precomputed constant scale tensors for a known input size (518x518)
- CPU inference at ~1s/frame on i7 laptop (single-threaded) is acceptable — the depth thread runs independently and the fusion engine consumes whatever rate is available. On Jetson Orin with GPU, TensorRT will be the production path

**The fix:** Three-step ONNX post-processing in `models/download_depth_anything_v2.sh`:
1. Export with `torch.onnx.export(dynamo=False, opset_version=14)` + monkey-patch `F.interpolate` to replace bicubic→bilinear
2. Run `onnxsim` to simplify the graph (folds constants, removes Shape/Gather/Cast nodes)
3. ONNX graph surgery: trace Resize input/output shapes via ORT, compute scale factors, replace 4-input Resize(X, roi, scales, sizes) with 3-input Resize(X, roi, scales) using constant scale tensors

**Limitations:**
- Scales are baked in for 518x518 input. Changing `input_size` in config requires re-running the export script.
- Bilinear interpolation in the patch embedding (instead of bicubic) may slightly affect depth accuracy — not measured, but the model still produces valid relative depth maps.

**When to revisit:** When migrating to TensorRT for Jetson Orin deployment (TensorRT supports full ONNX natively). Also revisit if OpenCV 5.x adds full Resize op support.

**Date:** 2026-04-14

---

## DR-015: OpenCV INTERFACE Link in drone_hal CMake

**Question:** Should we split OpenCV-dependent HAL backends into a separate `drone_hal_opencv` static library to prevent `${OpenCV_LIBS}` from leaking via INTERFACE link to all `drone_hal` consumers?

**Background:** PR #456 review flagged that `target_link_libraries(drone_hal INTERFACE ${OpenCV_LIBS})` makes OpenCV a transitive dependency for every target that links `drone_hal` — including tests and processes that don't use OpenCV.

**Arguments for splitting now:**
- Cleaner dependency graph — only P2 and OpenCV-specific tests link OpenCV
- Smaller link lines for non-perception targets
- Matches the pattern suggested for Cosys-AirSim (see `common/hal/CMakeLists.txt` comment)

**Arguments for deferring (our decision):**
- Currently only two OpenCV HAL backends exist (YOLO detector, DA V2 depth). The link leak is benign — OpenCV is a shared library, so unused symbols aren't linked in
- Splitting requires refactoring all consumers to explicitly link `drone_hal_opencv` — touches 5+ CMakeLists.txt files for zero functional benefit today
- The Cosys-AirSim block already documents this same pattern with the same deferral rationale ("migrate to STATIC when real SDK code is added")
- YOLO detector sources are already compiled per-target (not via INTERFACE), so the actual compile-time leak is minimal

**When to revisit:** When a third OpenCV HAL backend is added, or when the Cosys-AirSim migration to STATIC happens — do both splits together.

**Date:** 2026-04-14

---

## DR-016: rpclib Downloaded Into Submodule Working Tree

**Question:** The `FindAirSim.cmake` module downloads rpclib into `third_party/cosys-airsim/external/rpclib/`, which dirties the submodule working tree. Should we download into `${CMAKE_BINARY_DIR}` instead?

**For moving to CMAKE_BINARY_DIR:**
- Clean submodule (`git status` shows no untracked files)
- Standard CMake practice (FetchContent, ExternalProject all use binary dir)
- Less confusing for developers who check submodule status

**For keeping in submodule tree:**
- AirSim's own `cmake/CMakeLists.txt` hardcodes `find_path(... PATHS ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1)` — the build expects rpclib at that exact relative path
- Moving it would require patching AirSim's CMakeLists, defeating the purpose of vendoring an unmodified upstream
- The `external/rpclib/` directory is already in `.gitignore` (or should be) since AirSim's own `setup.sh` downloads it there too
- Only happens once (cached after first configure)

**Decision:** Keep rpclib in the submodule tree. The cost of a "dirty" submodule is low (add to `.gitignore`), while patching AirSim's build creates maintenance burden on every upstream update.

**Revisit when:** AirSim's CMake is refactored to accept external rpclib paths, or we fork the repo and can control the build system.

---

## DR-017: Perception Metrics — Inline vs Split `compute_detection_metrics` / `compute_tracking_metrics`

**Question:** The review-code-quality agent (PR #590) flagged that `compute_detection_metrics` (~80 lines) and `compute_tracking_metrics` (~80 lines) exceed the 40-line function-split heuristic and suggested splitting each into helpers. Should we split them?

**For splitting:**
- Shorter functions read faster in isolation
- Aligns with the general function-length guideline
- Future additions (e.g., class-agnostic AP variant) could reuse the per-frame-accumulate helper without duplicating the post-processing pass

**For keeping inline:**
- Both functions follow the same two-phase shape: a per-frame accumulate loop (mutating `out` / `tm`) followed by a short post-processing pass (per-class AP; AP=0 injection). Splitting them forces callers to pass the large output struct, several per-class maps, and the ground-truth inventory into a helper — so the helper signatures become long and the split buys line count, not clarity.
- The state each phase mutates is visible at a glance in-line (confusion matrix, per-class maps). Splitting would obscure the fact that `per_class_preds`/`per_class_gts` are scratch-local and only live across the same function.
- No current caller wants to skip the per-class AP pass or the AP=0 injection, so the phases aren't independently reusable.
- The test file exercises the full shape end-to-end via `compute_detection_metrics`; no test would gain clarity from the split.
- The tracking function has similar per-GT state (`gt_state` map) whose lifetime is the whole call — splitting would require passing it through a helper parameter for no readability gain.

**Decision:** Keep both functions inline. Readability of a two-phase flow is easier when both phases share locals by visibility, not by parameter threading. Accept the 80-line length here — the logic reads linearly.

**When to revisit:**
- If a new caller wants to consume only the per-frame phase (e.g. streaming / online evaluation), split the per-frame loop into a helper at that point.
- If a third metric function lands with the same two-phase shape — three repetitions would justify a shared helper.

**Date:** 2026-04-20 (during PR #590 review fix round)

---

## DR-018: Perception Metrics — `ScoredPrediction` / `ScoredGroundTruth` as Separate Types

**Question:** The review-code-quality agent (PR #590) noted that `ScoredGroundTruth` is a strict subset of `ScoredPrediction` (no confidence field) and suggested merging or making the relationship explicit. Should we unify them?

**For merging:**
- Drops one struct definition; one type to learn instead of two.
- A field addition would only need to land once.

**For keeping separate:**
- Type safety at the boundary: `compute_ap` takes predictions and ground truth as distinct parameters. Using separate types means the compiler catches an accidental swap (passing GTs where preds are expected); a unified type would let the swap compile and only fail at runtime when confidence turned out to be zero-valued GT.
- `confidence` has no meaning for a ground-truth box; carrying it on the unified type invites callers to pass uninitialised or sentinel values.
- Size is tiny (24 vs 28 bytes); there's no measurable memory cost to keeping them distinct.
- The narrowing is the *point* — `ScoredGroundTruth` being a subset-shape of `ScoredPrediction` mirrors the real-world relationship: GT is what a prediction would look like if it were guaranteed correct.

**Decision:** Keep as separate types. A header comment now documents the intentional narrowing so readers don't mistake it for accidental duplication.

**When to revisit:** If we add many fields that belong on both (e.g. frame-level metadata, timing), and the manual field duplication becomes burdensome, factor a common base struct rather than merging the leaf types.

**Date:** 2026-04-20 (during PR #590 review fix round)

---

## DR-019: Perception Metrics — `std::unordered_map` Instead of Dense Vector for `gt_by_frame`

**Question:** The review-performance agent (PR #590) noted that `compute_ap` uses `std::unordered_map<uint64_t, vector<size_t>> gt_by_frame`, but frame indices are densely packed 0..N-1 from the calling frame loop. A `std::vector<vector<size_t>>` indexed by frame would be O(1) access with no hashing. Should we switch?

**For switching to vector:**
- O(1) lookup vs unordered_map's hash + possible chain walk.
- Better cache locality.
- No bucket heap allocations.

**For keeping unordered_map:**
- `compute_ap` is a public entry point in the header — callers can pass `ScoredPrediction` / `ScoredGroundTruth` with arbitrary `frame_index` values, not necessarily dense 0-based. The caller that happens to use dense indices today (`compute_detection_metrics`) isn't the only supported caller.
- Switching to vector requires the caller to either (a) promise dense indices (silently breaks if violated), or (b) compute max frame index first and size the outer vector accordingly — which is itself O(N).
- Performance at the current AC (N=1000) is already 5 ms against a 100 ms budget. The hash cost is not on the critical path.

**Decision:** Keep `unordered_map`. The public-API flexibility is worth the small hash cost. If a future caller demands dense-index performance, expose a second `compute_ap` overload that takes the pre-grouped vector directly.

**When to revisit:** If the benchmark harness graduates into streaming / online evaluation where `compute_ap` runs 1000× per second, or if profiling shows the hash cost dominates.

**Date:** 2026-04-20 (during PR #590 review fix round)
