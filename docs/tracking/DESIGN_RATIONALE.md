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

---

## DR-020: LatencyProfiler — `std::string` in `LatencyTrace` vs Fixed `char[N]` Array

**Question:** The review-performance agent (PR #591) flagged that `LatencyTrace::stage` is a `std::string`, which makes `LatencyTrace` non-trivially-copyable and forces a (potentially heap-allocating) string assignment on every `record()` call into the trace ring. They suggested replacing with `char stage[32]{}` so the struct becomes trivially copyable and the trace-ring write is a `memcpy`.

**For switching to `char[N]`:**

- `LatencyTrace` becomes trivially copyable — bulk ring snapshots via `memcpy` instead of a per-element copy constructor.
- Eliminates the SSO dependency on typical stage-name lengths (currently safe because names like `"detector"` fit in ~15-char SSO buffers, but not guaranteed).
- Tighter bounded memory footprint — no silent heap growth if a stage name happens to be long.

**For keeping `std::string`:**

- Unbounded stage names stay supported without silent truncation. A `char[32]` truncates; the resulting JSON output then has two different stages with the same truncated key colliding in `stages`.
- The dominant cost on the `record()` path isn't the string copy — it's the mutex, `now_ns()`, and the map find. Stage-name assignment is < 100 ns per call when SSO applies.
- The `collect_traces_locked()` snapshot already copies `LatencyTrace` by value into a `std::vector<LatencyTrace>` — switching to `char[N]` makes that bulk copy faster, but it's not on the hot path.
- A fixed char array forces every caller to audit stage-name lengths across the codebase; adding a debug assert or documented contract to cap stage names is the work we'd need to do regardless.

**Decision:** Keep `std::string`. The complaint is structurally valid but practically marginal — the SSO optimisation covers typical usage, the mutex dominates the record-path cost, and the bounded-footprint argument doesn't hold up against silent-truncation collisions. The heterogeneous-comparator fix landed on the map key (avoiding the per-record `std::string` allocation for the map lookup) captures the biggest concrete win the review identified.

**When to revisit:**

- If profiling shows the trace-ring write (not the map lookup) as the dominant cost — e.g. if we start firing `record()` from tight loops where stage names are pre-computed longer strings.
- If we add a hard real-time pipeline stage (> 1 kHz) that uses the profiler directly instead of the intended `LatencyTracker` fast-path.
- If bounded allocation guarantees become a regulatory/certification constraint.

**Date:** 2026-04-20 (during PR #591 review fix round)

---

## DR-021: LatencyProfiler — `summaries()` Sorts Under the Profiler Mutex

**Question:** The review-performance and review-concurrency agents (PR #591) flagged that `LatencyProfiler::summaries()` calls `LatencyTracker::summary()` for each stage while holding the profiler mutex, and each `summary()` call sorts its sample ring (O(N log N)). With 10 stages × 1024 samples that's roughly 100 000 compare ops — ~100-200 µs of mutex hold time on an Orin Nano — during which concurrent `record()` callers block. They suggested exposing a raw-snapshot method on `LatencyTracker` and sorting outside the profiler lock.

**For offloading the sort outside the lock:**

- Shorter critical section → lower worst-case block time for concurrent recorders.
- Predictable: under-lock cost becomes O(N) memcpy per stage rather than O(N log N) sort.
- Avoids the scenario where `summaries()` can spike the very latency it's measuring.

**For keeping the sort under the lock:**

- `summaries()` is explicitly a periodic-dump path, not per-tick. The benchmark harness calls it at window boundaries (scenario-end or checkpoint), not 30 Hz. A 200 µs pause once every few seconds is invisible in the measured pipeline.
- Splitting introduces a new coupling: `LatencyTracker` would need a public `snapshot()` returning the raw sample vector, which leaks the ring-buffer implementation detail into callers. `LatencyTracker`'s current API is tight (`record`, `summary`, `reset`) — opening it up for the profiler's benefit is backwards.
- The concurrent-read-while-write test added in this PR catches any correctness regression; it doesn't catch a 200 µs pause, but that's not a correctness issue.
- The alternative (copy raw samples under lock, sort outside) still holds the lock for O(N × stages) memcpy — the absolute win is smaller than the refactor cost.

**Decision:** Keep the sort under the profiler mutex. The problem is real in theory but not triggered by the benchmark-harness usage pattern. If we ever call `summaries()` from a hot path, the right fix is to move the call off the hot path, not to re-architect the lock discipline.

**When to revisit:**

- If `summaries()` moves onto the per-tick path (e.g. a live dashboard reading current state at 30 Hz).
- If TSan or profiling shows recorder threads blocking on `mtx_` for measurable stretches during dumps.
- If `LatencyTracker` grows a snapshot API for other reasons — then this optimisation becomes free to land.

**Date:** 2026-04-20 (during PR #591 review fix round)

---

## DR-022: LatencyProfiler (Mutex-Protected) on Flight-Critical Threads — Opt-in Wiring in P2/P4

**Question:** The observability-on-flight-critical-threads rule in CLAUDE.md (and `deploy/safety_audit.sh` Rule 31) says mutex-protected observability primitives SHOULD NOT be called from flight-critical threads without documented justification. The profiler-wiring PR adds `ScopedLatency` guards in P2's detector/tracker/fusion threads (~30 Hz) and P4's planner loop (10 Hz) — each of which is a flight-critical path. Is this a violation or a justified exception?

**The rule targets two failure modes:**

1. **Priority inversion** — a lower-priority thread holding the profiler's mutex blocks a higher-priority control-loop thread.
2. **Observation affecting measurement** — the mutex cost contaminates the latency being measured.

**Analysis for this specific wiring:**

- **Priority isolation.** All recorders are peer pipeline threads in the same process — no real-time priority boundaries are crossed. P2's inference / tracker / fusion threads run at similar priority; P4's planner loop is single-threaded. No higher-priority thread calls `record()`, so classic priority inversion cannot occur. (The OS scheduler may still preempt mid-record, but that's true of any code; the profiler adds no new inversion path.)
- **Bounded mutex-hold-time dominated by measured work.** `LatencyProfiler::record()` takes a mutex, does a `std::map` find (heterogeneous `string_view` lookup — no alloc on hit), a `LatencyTracker::record()` (O(1) ring write), and a single string-assign into the trace ring. Measured cost is ~500 ns per call. Compared to detector work (tens of ms), tracker (~1–5 ms), fusion (~200 µs), planner-loop (<1 ms): the mutex hold is three-to-five orders of magnitude below the measured work. Measurement contamination is negligible.
- **Opt-in via config.** The guards live behind `benchmark.profiler.enabled` (default false in `config/default.json`). Production builds and every default scenario pay zero overhead — not even the optional-construction cost. Only scenarios that explicitly set the flag (currently only the upcoming #573 baseline-capture runs) activate the profiler.
- **No cross-thread data escape.** The `ScopedLatency` instances are stack-local to each stage's lambda; the profiler is a stack-local `std::optional<LatencyProfiler>` in `main()` whose lifetime covers all worker threads.

**Decision:** Accepted — safe exception to the rule. The three criteria the rule demands (priority isolation, bounded hold-time, explicit gating) are all met.

**Concrete implementation choices flowing from this analysis:**

- Pointer-optional pattern (`std::optional<ScopedLatency> bench; if (profiler) bench.emplace(...);`) rather than a compile-time switch, because we want the same binary to run both with and without profiling — switching a scenario's config is all it should take.
- No fallback to a lock-free per-thread buffer. Would be the correct answer for >1 kHz paths, but at 10–30 Hz the mutex is cheaper than maintaining per-thread ring buffers + a merge path. Keeping the single-profiler model simpler.
- JSON dump happens after all worker threads have joined (P2) or after the main loop exits (P4) so the mutex is uncontended during the dump.

**Addendum (2026-04-20, PR #593 review-fix round):**

- **AC denominator clarification.** The "< 2 % of tick time" claim refers to the **full pipeline tick**, not individual sub-stages. For P4's short sub-stages (`GeofenceCheck`, `FaultEval` at ~10 µs in the IDLE/PREFLIGHT fast-path early-return), the ~200 ns profiler overhead approaches 2 % of that sub-stage's own duration but stays well under 2 % of the 1–5 ms full `PlannerLoop` tick. This is acceptable — the AC is about per-tick pipeline budget, not per-sub-stage. If we later move the AC numerator to "per call-site" this sub-stage fast-path needs a second look.
- **Fast-path caveat for P4.** `GeofenceCheck` and `FaultEval` are profiled even when they early-return at IDLE/PREFLIGHT/TAKEOFF (short atomic-set path). The recorded p50/p95 will be noisier at these states because the profiler cost is comparable to the measured work. This is intentional — skipping profiling at IDLE would hide a real regression if the fast-path regressed. Baselines should segment by FSM state if this noise becomes a problem.

**When to revisit:**

- If profiling is ever used to instrument an IPC callback handler, Zenoh read-path thread, or a future >1 kHz stage — those DO have priority hazards the current analysis does not cover.
- If the profiler's `record()` internal cost grows past ~5 µs — re-evaluate contamination ratio.
- If the config flag is ever flipped to `true` by default — the "opt-in" leg of the justification disappears and the analysis must be redone.

**Date:** 2026-04-20 (during perception-v2 profiler-wiring PR)

---

## DR-023: LatencyProfiler — Raw `LatencyProfiler*` Aliasing `std::optional<LatencyProfiler>` for Thread Pass-Through

**Question:** The review-code-quality agent (PR #593) flagged that P2/P4 main.cpp construct a `std::optional<LatencyProfiler> benchmark_profiler` and then derive a raw pointer `LatencyProfiler* profiler_ptr = benchmark_profiler ? &*benchmark_profiler : nullptr;` to pass into worker thread functions. Two names alias the same underlying object — is this pattern justified?

**For an alternative (construct the profiler inside each thread, gate via `enabled` flag internally):**

- Eliminates the second name.
- No cross-scope pointer aliasing.
- Slightly simpler call-site.

**For keeping the current pattern:**

- `LatencyProfiler` is non-copyable, non-movable (holds a `std::mutex`). It cannot be passed by value into a `std::thread` functor, and it cannot be stored inside the optional *and* passed by reference into a thread's argument list, because `std::thread` decays its arguments to stored values. The raw pointer is the idiomatic workaround — C++ standard library patterns (`std::ref`, pointer args) exist for exactly this.
- The alternative "internal enable flag" would duplicate the gating logic into every `record()` call (plus every `ScopedLatency` dtor) for a hot-path branch — a cost that's absent when the pointer is null. Keeping gating outside the profiler also keeps the profiler's public API honest: if you have a profiler, it records. Period.
- The pointer is written exactly once in `main()` immediately after the optional's construction, and the `std::thread` ctor that follows provides a happens-before fence — the worker threads see a fully-constructed profiler via a stable pointer for their entire lifetime. No mutation, no race.
- The optional's storage is a local in `main()`; the main thread outlives every worker (it joins them before returning). Lifetime is trivially safe.
- Two names (`benchmark_profiler`, `profiler_ptr`) add a small cognitive cost but are unambiguous — `grep` finds both, code comments at the declaration site explain the relationship.

**Decision:** Keep the current pattern. The aliasing reads unusual at first glance but is load-bearing given `LatencyProfiler`'s non-movability and the C++ thread-argument contract. Internal gating would move cost into the hot path without improving clarity.

**When to revisit:**

- If `LatencyProfiler` becomes movable (breaking change to its mutex ownership) — then the optional could be moved directly into a thread's capture.
- If we add more call sites and the pattern duplication becomes a readability tax — factor a `ThreadedProfilerHandle` wrapper that hides the aliasing.

**Date:** 2026-04-20 (during PR #593 review fix round)

---

## DR-024: LatencyProfiler — Path Validation Allow-List (CWD + `/var/log/drone` + `/tmp`)

**Question:** The review-security agent (PR #593) flagged that `benchmark.profiler.output_dir` is taken from config without validation, enabling path-traversal writes on a tampered config. `LatencyProfiler::dump_to_file()` now canonicalises the path and checks membership in a fixed allow-list. What should the allow-list contain?

**For a minimal allow-list (production only — `/var/log/drone`):**

- Single documented safe root matches the hardened systemd `ReadWritePaths`.
- Dev workflows break: running a process from a shell dir expects the default `drone_logs/benchmark` (relative to CWD) to work.
- Would force every dev-machine run to set `output_dir` explicitly.

**For a permissive allow-list (CWD + `/var/log/drone` + `/tmp`):**

- **CWD** — lets dev runs use the default `drone_logs/benchmark` (relative path canonicalises under the launch directory). This is how every developer's local environment works today and matches the `.gitignore` assumption that `drone_logs/` is a per-machine artifact.
- **`/var/log/drone`** — the production `ReadWritePaths` entry. When the hardened systemd units eventually deploy, `benchmark.profiler.output_dir` can be set to an absolute path here.
- **`/tmp`** — tmpfs, safe for CI / smoke-test dumps that shouldn't persist. The test suite uses this.

**Against any absolute path outside the three:**

- Path traversal via `../../etc/systemd/system` is rejected after canonicalisation.
- Absolute paths to `/etc`, `/root`, `/boot`, `/usr` are refused — a tampered config cannot cause writes to arbitrary locations.
- The profiler is diagnostic-only; the feature is opt-in and off by default. A site with a legitimate unusual output path can add it to the allow-list in a small code change rather than a config-only change.

**Decision:** Adopt the permissive allow-list. Dev + production + CI all work without config gymnastics, while the security goal (no arbitrary writes from tampered config) is met. Failures produce `DumpStatus::PathRejected` and log at ERROR (since the user explicitly enabled profiling).

**When to revisit:**

- If a second observability sink needs disk output, factor the validation into a shared helper rather than duplicating the allow-list.
- If an operator needs to write benchmark data to a custom location, the allow-list is intentionally a compile-time list — add it with code review rather than config. If this friction becomes a real burden, we can add a CLI-only override flag that bypasses validation.

**Date:** 2026-04-20 (during PR #593 review fix round)

---

## DR-025: GtClassMap — Alphabetical Pattern Ordering Documented, Not Fixed

**Question:** PR #595 review flagged that `GtClassMap::lookup` docstring says "first match wins" but the implementation of `GtClassMap::load` iterates a `nlohmann::json` object, which is backed by `std::map` (alphabetical), not insertion order. Should we switch to `nlohmann::ordered_json`?

**For switching to ordered_json:**
- Matches user intuition: "I listed specific patterns first, they should win."
- Removes the trap where `"SM_Car*"` written before `"SM_*"` in config is silently overridden.

**For documenting and keeping std::map:**
- `ordered_json` is a breaking change project-wide — every `drone::Config` consumer would be affected.
- Alphabetical order is stable, deterministic, and greppable.
- Users can work around it by naming patterns so desired winners sort first (e.g. `zzz_fallback*`).
- Direct construction via `PatternEntry` vector still preserves caller order — the only case where order matters is `load()`.

**Decision:** Document clearly (see `GtClassMap::lookup` docstring in `tests/benchmark/gt_emitter.h`) and add `LoadFromConfigUsesAlphabeticalOrder` test to lock the semantic. Revisit if we hit a real scenario where alphabetical-naming workaround is insufficient.

**Revisit when:** A scenario config needs insertion-order semantics that alphabetical naming can't express, OR we move `drone::Config` to `ordered_json` for other reasons.

**Date:** 2026-04-20 (during PR #595 review fix round)

---

## DR-026: GT Class Map — `class_id = 0` as Valid (COCO "person") Not as Sentinel

**Question:** PR #595 review flagged that `GtClassMap::Entry::class_id` defaults to `0`, but COCO class 0 is "person" — a real class. Should we use `std::optional<Entry>` semantics throughout, or add a validity sentinel?

**For optional/sentinel:**
- Defends against bugs where a default-constructed Entry is silently returned instead of `nullopt`.
- Makes the "not mapped" state explicit.

**For keeping class_id = 0 as valid:**
- `lookup()` already returns `std::optional<Entry>` — the "not mapped" case IS explicit at the API boundary.
- A default-constructed `Entry` inside a `GtClassMap` only exists if the user writes one — tests don't show that happening.
- Changing to a sentinel (e.g. `UINT32_MAX`) creates a separate class of bug where the sentinel is accidentally serialised into JSONL output.
- Using `std::optional<Entry>` everywhere makes the API noisier for no real safety gain, since the lookup already returns an optional.

**Decision:** Keep `class_id = 0` as a valid class. The default value of `Entry{}` is only reached if code inside the class intentionally returns a default — which it doesn't. The `lookup()` optional return is the defensible boundary.

**Revisit when:** A new caller pattern emerges that passes `Entry{}` around without going through `lookup()`.

**Date:** 2026-04-20 (during PR #595 review fix round)

---

## DR-027: GtClassMap — Silent Skip of Entries Missing `class_id` / `class_name`

**Question:** PR #595 review flagged that `GtClassMap::load` silently drops entries that are malformed (missing `class_id` or `class_name`, or whose value is not an object). Should we WARN on each skip or throw?

**For warn/throw:**
- Catches config typos at startup rather than silently producing a smaller map.
- `LoadFromConfigSkipsMalformedEntries` test exists but nobody reads test output.

**For silent skip:**
- `GtClassMap` is used at scenario-run time, not at production runtime. A scenario author running the scenario will see GT miss and investigate.
- The outer WARN ("gt_class_map key is present but not a JSON object") already catches the whole-section typo case — added in this PR.
- Per-entry warn adds log noise for a tooling-only harness.
- If we want strict validation, that belongs in a separate `validate_scenario_config.py` pre-run tool, not in the emitter.

**Decision:** Keep silent per-entry skip. Outer WARN covers the likely-to-happen whole-section typo. Scenario-config validation is a separate concern.

**Revisit when:** A scenario ships with silently-dropped GT entries and nobody notices until a baseline regression.

**Date:** 2026-04-20 (during PR #595 review fix round)

---

## DR-028: `register_filters` Kept Public on CosysGtEmitter (anonymous namespace class)

**Question:** PR #595 review flagged that `CosysGtEmitter::register_filters` is public but the class lives in an anonymous namespace (no external callers possible). Should it be private and called from the constructor?

**For private + constructor-call:**
- Expresses the invariant that filters are always registered before any `emit()` call.
- Removes the chance of a future refactor forgetting to call it.

**For keeping as-is:**
- The class is in an anonymous namespace — there ARE no external callers, so the exposure risk is nil.
- Moving RPC calls into the constructor means construction can fail (via RPC error) but constructors can't return null. We'd have to throw — adding exception-handling paths to the factory.
- The factory currently calls `register_filters` explicitly and logs a WARN on failure without aborting emitter creation. This "partial filter set is still useful" behaviour would be lost.

**Decision:** Keep `register_filters` public and called from the factory. The anonymous-namespace scope already provides the isolation the reviewer wanted, and the explicit-call + WARN pattern preserves graceful-degradation semantics.

**Revisit when:** We move `CosysGtEmitter` out of the anonymous namespace (e.g. for direct testing), at which point re-evaluate.

**Date:** 2026-04-20 (during PR #595 review fix round)

---

## DR-029: GtClassMap — Linear Scan Not Unordered Map

**Question:** PR #595 review flagged that `GtClassMap::lookup` does a linear scan through `patterns_`. Should it be an `unordered_map` with exact matches resolved first and wildcards as a fallback list?

**For unordered_map:**
- O(1) exact-match lookup vs O(N) linear scan.

**For keeping linear scan:**
- Typical class maps are <20 entries. Linear scan of 20 entries is a handful of ns — not a real hot path.
- `lookup()` is called per-detection, ~30 times per frame at 30 Hz — ~900 calls/sec max. Even at 100 entries, ~100 µs total per second — 0.01% of tick budget.
- Wildcards MUST be scanned linearly regardless; mixing exact-hash and wildcard-scan adds code complexity for no measurable gain.
- Simple code is easier to reason about for a benchmark-harness-only component.

**Decision:** Keep linear scan. Revisit if a real scenario emerges with >100 patterns, measured impact > 1% of tick budget.

**Date:** 2026-04-20 (during PR #595 review fix round)

---

## DR-030: YoloSegInferenceBackend — Perception-Level Factory, Not HAL Factory

**Question:** `YoloSegInferenceBackend` implements `IInferenceBackend` (a HAL interface). Should it be registered in `hal_factory.h::create_inference_backend()` like `SimulatedSAMBackend`?

**For HAL factory registration:**

- Consistent with how other backends (simulated, SAM) are created — single factory, single config key.
- Callers don't need to know which factory to use.

**For perception-level factory (Option A):**

- `YoloSegInferenceBackend` depends on `perception/detector_class_maps.h` → `perception/types.h`, which live in `process2_perception/`. The HAL factory is in `common/hal/` and can't include process-level headers without creating an upward dependency.
- The .cpp lives in `process2_perception/src/` — only compiled for P2. Other processes including `hal_factory.h` would get linker errors.
- `OpenCvYoloDetector` (the existing YOLO detector) follows the same pattern: it lives in P2, not in the HAL factory. The HAL factory creates generic/simulated backends; perception-specific implementations are instantiated at the process level.
- Moving `DetectorDataset` and class-mapping functions to `common/` would couple the HAL layer to perception-domain concepts (COCO class indices, VisDrone dataset specifics) — violating the abstraction boundary.

**Decision:** Keep `YoloSegInferenceBackend` in `process2_perception/`. P2's main (or a thin perception-level factory) selects between `YoloSegInferenceBackend` and the generic HAL backends based on config. This matches the existing `OpenCvYoloDetector` pattern and preserves the HAL/perception layering.

**Revisit when:** If multiple processes need to create YOLOv8-seg backends, or if `detector_class_maps.h` is refactored into `common/`.

**Date:** 2026-04-22 (Epic #520, E5.2 implementation)

## DR-031: MaskDepthProjector::project() Marked `const` Despite Non-Const Projector Reference

**Question:** PR #604 review (review-api-contract agent, P1): `project()` is declared `const` but calls `ISemanticProjector::project()` which is a non-const virtual method. The `const` doesn't propagate through the reference member, so callers may incorrectly assume concurrent calls are safe.

**For removing `const`:**

- Honest about the mutation potential — if `ISemanticProjector` implementations become stateful, the `const` is misleading.
- Prevents false safety assumptions about thread safety.

**For keeping `const` (our decision):**

- Standard C++ pattern — `const` on a method documents that *this object's* state doesn't change, not that every transitively reachable object is immutable. The same pattern is used throughout this codebase (any class holding a `Logger&`, `IPC&`, or `Config&` reference).
- `CpuSemanticProjector::project()` is logically const (reads intrinsics, writes to output only). The non-const `override` is inherited from the interface, which is non-const to accommodate future GPU backends that may need to update internal state.
- Removing `const` would be viral — callers holding `const MaskDepthProjector&` couldn't call `project()`, breaking composition patterns.
- Thread safety is a separate concern from `const` correctness and should be documented independently if needed.

**Decision:** Keep `const` on `project()`. The method correctly documents that `MaskDepthProjector` itself is immutable. Thread safety of the underlying `ISemanticProjector` is the caller's responsibility.

**Revisit when:** If a stateful `ISemanticProjector` backend (e.g., GPU with internal batch state) is added and concurrent access becomes a real concern.

**Date:** 2026-04-22 (PR #604 review, E5.4 MaskDepthProjector)

## DR-032: GEOMETRIC_OBSTACLE Guarantee Delegated to MaskClassAssigner, Not Enforced in MaskDepthProjector

**Question:** PR #604 review (review-api-contract agent, P1 downgraded to P3): `MaskDepthProjector` docstring claims "unmatched masks receive GEOMETRIC_OBSTACLE" but the guarantee lives in `MaskAssignment::assigned_class` default value, not enforced locally. If `MaskClassAssigner` changes its default, the claim silently breaks.

**For local enforcement (assert after assign):**

- Defense-in-depth: MaskDepthProjector would verify the invariant it advertises, independent of the assigner's implementation.
- Catches future regressions if `MaskAssignment` default changes.

**For delegation (our decision):**

- `MaskClassAssigner` is the single authority for class assignment. Its struct `MaskAssignment` defaults `assigned_class` to `GEOMETRIC_OBSTACLE` — this is the documented contract of the assigner, not a coincidence.
- Adding an assertion in `MaskDepthProjector` would duplicate the assigner's responsibility and create a coupling where changes to the assigner's class policy require updating two places.
- The test suite (`ProjectSingleGeometricMask`, `ProjectMaskBelowIoUThreshold`) already verifies the end-to-end invariant through the full chain.
- Updated docstring to clarify delegation: "via MaskClassAssigner default".

**Decision:** Keep the guarantee in `MaskClassAssigner` where it belongs. `MaskDepthProjector` documents the delegation explicitly.

**Revisit when:** If a second class-assignment strategy is introduced (e.g., embedding-based) that might use a different default.

**Date:** 2026-04-22 (PR #604 review, E5.4 MaskDepthProjector)

## DR-033: `PathATrace` JSONL Writer on Flight-Critical `mask_projection_thread`

**Question:** PR #614 review (review-concurrency, P1): `PathATrace::record_batch()` performs `std::ofstream` I/O directly on `mask_projection_thread`, which is one of P2's flight-critical perception threads. Glibc's `std::ofstream` takes a streambuf sentry / write lock internally; this is the same class of hazard the "observability on flight-critical threads" rule in `docs/reference/CPP_PATTERNS_GUIDE.md` and CLAUDE.md is built to prevent. Should the diagnostic writer be reworked to buffer through an SPSC ring + dedicated IO thread before it ships?

**For the SPSC-ring approach:**

- Removes the (small) streambuf lock from the hot path entirely — the ring is lock-free.
- Aligns with `LatencyTracker` / `TripleBuffer` patterns already used for hot-path telemetry.
- Makes the production-always-safe story easier to state: "PATH A tracing is measurable-cost-free even when enabled."

**For keeping the direct `ofstream` call (our decision):**

- The CLAUDE.md rule reads *"SHOULD NOT be called from flight-critical threads without documented justification"*. This DR is the justification.
- `PathATrace` is opt-in via config key `perception.path_a.diag.trace_voxels`, **default `false`** — production configs never flip it on. Scenario 33 turns it on because it's a diagnostic scenario; no other scenario enables it.
- The `enabled_` short-circuit is a single `bool` load at the top of `record_batch()`. When disabled, cost is one load + predicted branch — zero I/O, zero allocation.
- When the user enables the trace, they are *already opting out of the production latency envelope* — the whole point is to get visibility during diagnosis, not to measure production latency. Adding a second thread + SPSC ring for a tool that only runs under a diagnostic config is over-engineering for the use case it exists to serve.
- Same rationale recently codified for `LatencyProfiler` wiring in DR-022 — opt-in observability on control threads is acceptable when the opt-in gate is a default-false config and the hot-path cost when disabled is zero.
- The companion `cosys_telemetry_poller` follows the same pattern (10 Hz direct `ofstream` on a dedicated polling thread that only exists in diagnostic runs).

**Decision:** Keep the direct `ofstream` call in `record_batch()`. Document the constraint in the header:

- file-level comment names the rule and the DR.
- method-level comment on `record_batch()` states single-writer / not-thread-safe.
- code-level: the `enabled_` short-circuit is the first statement so the hot-path cost is one predicted branch.

CI hygiene (follow-up): add a lint that greps non-diag scenarios' `config_overrides` for `trace_voxels: true` and fails if found.

**Revisit when:** If a production config ever wants `trace_voxels=true` (e.g., black-box flight recorder for certification evidence), promote to the SPSC-ring design before shipping.

**Date:** 2026-04-23 (PR #614 review, review-concurrency P1)

## DR-034: Four `BUG_FIXES` Entries Removed — Attribution Belonged to Earlier PRs

**Question:** PR #614 review (review-api-contract, P1): An earlier draft of `BUG_FIXES.md` added entries Fix #56, #57, #58, #60 for PATH A fixes (mask-convention auto-detect, SAM factory routing, FastSAM `onnxsim` post-pass, `OccupancyGrid3D::insert_voxels` ground-plane filter / position clamp / inflation reduction). The reviewer pointed out these don't appear in PR #614's diff. Should they be relabelled, removed, or kept with a pointer to the real authoring PR?

**Audit result (confirmed by git diff against the integration-branch tip `992b888`):**

- `common/hal/include/hal/cpu_semantic_projector.h` — unchanged by PR #614; the 1.5× mask-convention auto-detect and 20 m depth clamp were in place as of PR #611.
- `common/hal/include/hal/fastsam_inference_backend.h` — PR #614 only bumps `kMaxProposals` 16384 → 65536 (Fix #59). The wider FastSAM wiring and `onnxsim` post-pass predate this branch.
- `process4_mission_planner/include/planner/occupancy_grid_3d.h` — unchanged by PR #614. `insert_voxels()` including ground filter, position clamp, and 1-cell inflation were in place at the integration-branch tip.
- `models/download_fastsam.sh` — unchanged by PR #614 (repo stores it as a symlink into the parent project).

**For relabel-and-point-to-real-PR:** keeps a searchable record of *what* the fix was; the PR pointer tells readers where to dig if they want the commit.

**For removal (our decision):** `BUG_FIXES.md` is a changelog, not a retrospective catalogue. Every other entry in the file is a thing the commit/PR it references actually did. Inserting re-descriptions of earlier work violates that convention and would let future readers wrongly cite this PR as the authoring commit. Attribution notes at the end of a single adjacent entry (Fix #55) are enough to point curious readers at where the earlier work really happened.

**Decision:** Remove Fix #56, #57, #58, #60. Keep Fix #55 (camera extrinsic), #59 (kMaxProposals), #61 (yaw_towards_velocity), #62 (poller include order) — these genuinely land in PR #614. Numbering gap is consistent with pre-existing gaps elsewhere in the file.

**Revisit when:** If `BUG_FIXES.md` ever grows a "Changes inherited from integration branch" section, the removed entries could be moved there with their real PR references.

**Date:** 2026-04-23 (PR #614 review, review-api-contract P1)

---

## DR-035: PR #632 review-memory-safety P1 "Missing struct fields" — Stale review, fields exist

**Question:** PR #632 review-memory-safety flagged a P1 marked "Blocks Merge" claiming `config_.yaw_towards_velocity` and `config_.yaw_velocity_threshold_mps` referenced in the new `GridPlannerBase` accessors *do not exist* in `GridPlannerConfig`, that the diff does not add them, and that the build would fail. The reviewer also flagged the same fields being set in `tests/test_mission_state_tick.cpp` as the same compile error.

**Audit result:** Both fields *do* exist in `GridPlannerConfig` and have lived there since the integration-branch tip. `process4_mission_planner/include/planner/grid_planner_base.h` line 68:

```cpp
bool yaw_towards_velocity = false;
```

…and line 74:

```cpp
float yaw_velocity_threshold_mps = 0.4f;
```

The reviewer's "grep across the entire codebase returns zero matches" claim is incorrect — both are also referenced in `plan()` at lines 438 and 440 of the same file, and the surrounding `yaw_towards_velocity` comment block at lines 62–67 documents the feature.

**Likely cause:** The reviewer's grep was scoped to the PR diff only (which doesn't *change* those struct fields, only adds the accessors that read them), or hit a `git show` view rather than the full file at the integration-branch tip. The "first 500 lines" diff truncation note in the review's review-security section corroborates this — the GridPlannerConfig struct begins at line 33 in the file but the relevant fields are at lines 60–74, which would land near the truncation boundary.

**For accepting the comment:** zero — it's wrong on the facts. The build is green at PR #632.

**For declining (our decision):** the comment is a stale review artefact, not a real bug. Adding the fields again would either (a) shadow existing fields with the same name (compile error) or (b) be a no-op if the reviewer wanted us to verify the existing ones. No code change is correct here.

**Decision:** Do not modify `GridPlannerConfig`. Note in the PR's review-fixes table that the P1 has been audited and dismissed as a false positive against the actual file contents. Apply the *adjacent* P2 (`noexcept` on the new pure virtuals) which IS a real hardening improvement.

**Revisit when:** never — this is a one-off review artefact. Future stale-review claims of the same shape (`X does not exist`, `code will not compile`) get the same treatment: read the file at HEAD, verify, decline if the claim is wrong, fix if it's right.

**Date:** 2026-04-27 (PR #632 review-memory-safety P1)

---

## DR-036: PR #630 review-security P4 "Nonstandard `_comment_*` key" — Established convention, not deviation

**Question:** PR #630 review-security P4 flagged `"_comment_sample_grid_size"` in `config/scenarios/33_non_coco_obstacles.json` as a "nonstandard comment key name" deviating from the established `"_comment"` convention, recommending a rename to `"_comment"` (or `"_comment_path_a_sampling"` to avoid collision).

**Audit result:** `grep -rh '"_comment_' config/scenarios/` returns 23 hits across multiple scenario files: `_comment_base_config`, `_comment_detector`, `_comment_sample_grid_size`, `_comment_use_cuda`, `_comment_static_obstacles`, `_comment_temp`, `_comment_waypoints`, `_comment_model`, `_comment_enabled`. The `_comment_<key>` form is the established pattern for *inline annotations on a specific sibling key*, complementing `_comment` for *block-level annotations*. Both forms coexist in the same file (e.g. scenario 33 has both `_comment` at the top of `path_a` and `_comment_use_cuda` next to `path_a.sam.use_cuda`).

**For accepting the comment:** consistency with a single comment-key naming convention; smaller surface for "is this a parseable key or a comment" confusion.

**For declining (our decision):** the convention IS `_comment_<keyname>` for sibling annotations and `_comment` for block annotations. Renaming this one instance would BREAK consistency with 22 other live `_comment_*` keys. The reviewer was generalising from `_comment` without grepping for the broader pattern; the actual convention permits both.

**Decision:** Keep `_comment_sample_grid_size` as-is. If a future linter is added, it should accept both `^_comment$` and `^_comment_[a-z_]+$` patterns.

**Revisit when:** the codebase grows a JSON schema validator that explicitly enumerates allowed keys; at that point all `_comment*` keys should be enumerated together.

**Date:** 2026-04-27 (PR #630 review-security P4)


---

## DR-037: PR #639 review-code-quality P2 — Union-Find without union-by-rank

**Question:** PR #639 review-code-quality P2 flagged that `uf_union` in `voxel_clusterer.h` always attaches `rx` under `ry` without rank tracking, so the worst-case Union-Find depth is O(N) before path-halving in `uf_find` flattens it. The header docstring claims "O(N · α(N))", which is only true with union-by-rank. The reviewer asked us to either implement rank or correct the docstring.

**For implementing rank:** restores the documented complexity guarantee. ~10 lines.

**For declining (our decision):** at scenario-33 N (~5 000 voxels/frame) the worst-case difference between O(N · α(N)) and O(N · log N) is ~12-13 extra `find()` iterations per voxel — invisible in practice. Path-halving alone gives effectively-flat trees on realistic spatial-cluster geometry (clusters are mostly wide and shallow, not degenerate chains). Adding rank would mean tracking a second `vector<int>` and an extra branch in `uf_union`; the maintenance + cache cost is a fair trade against the asymptotic guarantee only when N grows or when a profiler shows the union pass dominating.

**Decision:** Keep the rank-free implementation. **Tighten the docstring to "O(N · log N) worst case, O(N) typical with path-halving on cluster-shaped input"** rather than claiming the rank-bound. Add `IMPROVEMENTS.md` entry for "implement union-by-rank when profiling shows union pass dominating."

**Revisit when:** N exceeds ~50 000 voxels/frame, OR a profile shows >5 % of `mask_projection_thread` time in `uf_find` / `uf_union`, OR scenario inputs become topologically degenerate (long chains rather than clusters).

**Date:** 2026-04-27 (PR #639 review-code-quality P2)

---

## DR-038: PR #640 review-performance P3 — Per-frame allocations in `VoxelInstanceTracker::update()`

**Question:** PR #640 review-performance flagged that `update()` allocates four local collections per call (`candidates`, `frame_local_to_stable`, `track_matched`, `tracks_view`). At 10 Hz × ~30 tracks this is ~40 small allocations per second, suggested fix is to mirror the `VoxelClusterScratch` pattern with persistent member fields.

**For mirroring the pattern:** consistent with Phase 1, eliminates the allocator pressure entirely.

**For declining (our decision):** Phase 1's `VoxelClusterScratch` exists because the clusterer's allocations are sized by N=5000 voxels/frame — large enough that re-rehashing matters. Phase 2's tracker collections are sized by ≤30 tracks/frame; per-call cost is ~5 μs total on Jetson Orin's allocator. Adding `VoxelTrackerScratch` mirror requires either (a) breaking encapsulation by exposing the scratch through the constructor, or (b) making the four collections `mutable` members violating the const-correctness of `tracks()`. Neither is worth the ~50 μs/s saved.

**Decision:** Keep per-call locals. **Land if a profiler shows tracker `update()` >100 μs/frame** in scenario testing. The header already documents the N≤30 assumption in the "Algorithm choice" section.

**Revisit when:** Phase 4 scenario runs show tracker `update()` cost above 100 μs/frame, OR if track count grows past ~100 (e.g. complex urban scenarios with many simultaneously-tracked obstacles).

**Date:** 2026-04-27 (PR #640 review-performance P3)

---

## DR-039: PR #641 review-memory-safety P2 — `OccupancyGrid3D` ctor with 14 positional parameters

**Question:** PR #641 added the 14th positional parameter to `OccupancyGrid3D`'s constructor. The reviewer correctly flagged this as a maintainability risk: silent argument-swap bugs become invisible at the call site. Suggested fix: refactor to a `GridConfig` struct mirror.

**For refactoring:** named-arg semantics, robust to insertion at any position, mirrors the `GridPlannerConfig` pattern already used at the planner layer.

**For declining for this PR (our decision):** the refactor has wide blast radius — `OccupancyGrid3D` is constructed in 8 unit-test files plus `GridPlannerBase` plus `add_static_obstacle` test fixtures. Each call site uses positional `/*name=*/` comments; mechanically converting them to a struct is straightforward but adds ~50 lines of net diff to a PR whose scope is already ~150 lines of behaviour change. Better to land the behaviour change first and refactor as a follow-up that touches only signature + call sites.

**Decision:** Keep the 14-parameter ctor for #641. **File a dedicated refactor PR** that introduces `OccupancyGrid3DConfig` struct and updates all call sites in one mechanical pass. Pre-condition: #639/#640/#641/#642 all merged so the integration branch is the stable base for the refactor.

**Revisit when:** parameter 15 is needed (the trigger forcing the refactor anyway), OR the next P4-side review of the constructor surface area.

**Date:** 2026-04-27 (PR #641 review-memory-safety P2-D)

---

## DR-040: PR #642 review-test-coverage P2 — No CI test for scenario JSON loading

**Question:** PR #642 review noted that no CI test parses `config/scenarios/*.json` and verifies the keys reach their consumers. A typo or stale key would only surface at scenario-run time (Tier 3 Cosys-AirSim), not in unit-test CI.

**For adding the parametric test:** catches typos at unit-test time, ~20 lines of `tests/test_config_validator.cpp` extension.

**For declining for this PR (our decision):** the suggested test is a *good idea* but is not Phase 4-specific — every config key in the codebase has the same vulnerability. Adding it as a one-off for #642 would create a half-coverage situation (tests scenarios but not `default.json`, `cosys_airsim_dev.json`, etc.). The right scope is a dedicated test that walks every JSON file in `config/` and validates structure against a schema or a key allowlist; that's its own ~100-line PR with cross-cutting impact.

**Decision:** Don't add the parametric test in #642. **Open a separate issue** for "config-validator test for all `config/*.json` files" with sub-tasks for each config category. Phase 4 ships with the PR-body explicit binary-version dependency note (already addressed).

**Revisit when:** the next config-key typo causes a debugging session, OR when `IMPROVEMENTS.md` accumulates 3+ scenario-config-related issues.

**Date:** 2026-04-27 (PR #642 review-test-coverage P2)

---

## DR-041: ObstacleAvoider3D drone-inside-AABB recovery — defense-in-depth for a "shouldn't happen" state

> ⚠️ **TOMBSTONED 2026-05-11 (PR #712 / DR-043).**  The entire AABB-aware
> distance code path discussed in this DR — both the math bug and the
> drone-inside-AABB recovery branch that fixed it — was empirically
> validated as dead weight in Cosys scenario 33 and **removed from the
> codebase**.  The avoider now uses centroid-distance throughout (no
> AABB clamp, no inside-AABB recovery, no exit-face logic).  This DR
> is retained for historical context only; the design considerations
> here no longer apply to the production code.  If a future PR re-
> introduces AABB-aware geometry, refer back to this DR before
> stripping the recovery branch.

**Question:** PR #657 (avoider AABB-aware distance) shipped a math bug — when the drone is geometrically inside an obstacle's Axis-Aligned Bounding Box, `clamp(drone, ax_min, ax_max)` returns the drone's own coordinate, so `dx=dy=dz=0` and `dist=0`.  The downstream gate `dist > kMinDistGateM` then SILENCES the obstacle entirely.  PR #685 added an explicit "find nearest exit face, push out via that face with maximum repulsion" branch.

A reviewer (and reasonably the user) might ask: **"the drone shouldn't ever be inside an obstacle's bounding box anyway — why bother with the recovery?"**

**For not bothering ("shouldn't happen, ignore"):** the avoider is the last reactive layer.  In a working stack, the planner + voxel grid + close-regime hysteresis all cooperate to keep the drone outside obstacle boundaries; if those upstream guards fail there's likely a bigger problem than the avoider can paper over.  Code complexity has cost — the exit-face lookup is ~30 lines of branching logic that only fires in a should-be-impossible state.

**For the defense-in-depth fix (our decision):** "shouldn't" is the design intent; "could" is operational reality.  Real causes that put the drone inside an obstacle's AABB:

1. **First-frame race.** PATH A voxels can flicker in/out — SAM mask jitter means an obstacle could be silently absent for one frame and present-with-drone-inside on the next.  We saw this exact pattern in scenario 33 cube collisions before #658/#659/#660 tightened the upstream filters.
2. **Depth blowup.** DA V2 reports closer than reality.  Obstacle physically at 5m could be reported at 1m; AABB centred at 1m encloses drone at 0m.  Same failure class that drove #654 (`position_clamp_m: 200→30`) and #660 (`min_confidence: 0.3→0.7`).
3. **Inflated radius estimate.** `estimated_radius_m` from the detector can over-estimate.  A 0.5m cube reported with `estimated_radius_m=1.5m` produces an AABB the drone enters before reaching real geometry.
4. **VIO drift / pose lag.** Drone pose estimate lags 50-200ms behind reality.  Pose-says-(0,0,0) but actually-at-(5,5,5) places the drone inside any AABB centered nearby.
5. **Late frame delivery.** Vision pipeline at 10Hz, drone at 5 m/s = 0.5m gap between frames.  AABB radius < gap means the drone can enter between frames.
6. **Detector trust during stereo migration.** When we move to stereo (Issue #663), depth gets bounded but obstacle classification might lag, briefly producing AABBs sized for the wrong object class.

The previous behaviour (silent failure) is the worst possible outcome — it gives FALSE CONFIDENCE that nothing's wrong while the drone is in fact penetrating an obstacle.  The recovery branch produces a degraded-but-observable output: maximum repulsion outward, close-regime engaged, brake fires.  Operator sees the close-regime-active flag and the WARN log; the drone fights its way out instead of riding through.

**Decision:** Land the exit-face recovery (PR #685) and the regression test (`AabbAwareDistance_DroneInsideAabbReceivesExitRepulsion`).  Treat the avoider as the last reactive line that must produce safe output for ANY input geometry, not just the design-intended subset.  This DR captures the principle so a future reviewer doesn't strip the recovery branch on "shouldn't be reachable" grounds.

**Revisit when:** all six upstream causes above have dedicated regression locks AND are enforced by config-validator gates; the exit-face recovery becomes belt-and-braces that can be downgraded from `WARN` to `DEBUG` on engagement (the bug is then truly elsewhere, not the avoider).

**Date:** 2026-05-02 (PR #685 / PR #657 review-fault-recovery P1)

---

## DR-042: PR #704 — `DRONE_LOG_INFO` on Cosys HAL polling threads (1 Hz cadence) without a lock-free buffer

**Question:** CLAUDE.md "Observability on flight-critical threads" rule states that mutex-protected observability primitives (loggers, profilers) MUST NOT be called from flight-critical or real-time threads without documented justification.  Both `CosysEchoBackend::poll_loop()` and `CosysGroundTruthRadarBackend::poll_loop()` emit `DRONE_LOG_INFO` directly (spdlog default sink — internally mutex-protected), at a rate of 1 Hz (every 20th scan at 20 Hz).  Should these be moved to a lock-free buffer drained by a dedicated IO thread, like the LatencyTracker pattern P3 uses?

**For moving to a lock-free buffer:** The rule exists for a reason — even at 1 Hz, mutex contention can produce priority inversion if a higher-priority thread happens to want spdlog's mutex while the poll thread is mid-`format`.  And the cost is a real spdlog format + sink-write under lock per emission.  In a deployed real-hardware system this could matter.

**For keeping spdlog on the poll thread (our decision):**

1. **Polling threads are not flight-critical.**  P2's hot path (detector, tracker) and P4's planner-tick are flight-critical.  These Cosys HAL threads are sim-only ground-truth feeders running at 20 Hz — they don't touch the FC, the pose estimator, or any real-time control loop.  An RT-priority inversion against them does not flunk the safety analysis.
2. **20 Hz is "soft real-time" at most.**  Even on a worst-case spdlog hold of 5 ms, that's a 10 % budget at 20 Hz — observable in latency profiling but well outside any flight-critical envelope.
3. **The rule's primary hazard is priority inversion against mutex-protected logging from a control thread.**  Here the writer is the polling thread itself; there is no higher-priority consumer of the lock at the same level (drainer thread, etc.).
4. **Throttled to 1 Hz.**  The first-non-zero-emission and per-20-scan summaries are explicitly capped — this is not a blast-radius concern.
5. **Sim-only code path.**  These backends are guarded by `HAVE_COSYS_AIRSIM`; production drone deployments don't compile them in.  Real-hardware perception is on PATH C / Echo / camera-from-V4L2, not Cosys.

**Risks accepted:**

- Future RT-priority work that gives the Cosys polling thread a real-time priority would expose the inversion hazard — this DR must be revisited.  Currently the threads run at default OS priority.
- Future spdlog backend changes (async, file-rotate-on-write) could change the mutex hold-time profile.  spdlog 1.13's default sink is well-understood; document this DR as predicated on the current sink class.

**Decision:** Keep `DRONE_LOG_INFO` on the polling thread.  Document this DR.  The IMPROVEMENTS.md backlog item (P2) is the long-term fix once we either (a) deploy Cosys backends to a real-hardware path, or (b) measure latency-budget impact > 0.5 ms.

**Revisit when:** Cosys backends become a flight path on real hardware OR `LatencyProfiler` data shows >0.5 ms tail latency on the Cosys polling threads OR the spdlog sink is replaced with a different mutex profile.

**Date:** 2026-05-05 (PR #704 review-performance P2 + review-concurrency observability rule)

---

## DR-043: Remove three scenario-33 avoider safety nets after empirical sweep showed they are dead weight

**Question:** PR #712 (commits `42e7af9` / `aa69d53` / `e6bf478` / `b721481` on `feature/issue-710-empirical-cleanup`) removes three avoider/perception code paths originally added as scenario-33 safety nets:

- **PR #646** — `close_regime_final_clamp`: post-correction velocity clamp that zeroed the toward-obstacle component AFTER all repulsion + brake math.  Designed as a "hard safety floor" so the final commanded velocity could never have a positive toward-obstacle component in close regime.
- **PR #657** — `aabb_aware_distance`: replaced centroid-distance with AABB-face-distance + perpendicular repulsion direction.  Designed to fix cube/wall tangential-contact collisions where centroid-distance under-estimates the safety margin.
- **PR #647** — `avoider_surface_enabled`: appended synthetic `GEOMETRIC_OBSTACLE` entries from the PATH A voxel-instance tracker into `DetectedObjectList`.  Designed so the avoider could repel from non-COCO geometry (cubes/pillars/walls) that the YOLO detector doesn't classify.

Removing safety code is by definition a defensible-on-both-sides decision.  The argument FOR keeping them is real: they were each motivated by a specific observed scenario-33 failure mode and protect against the same class of failure if it recurs.  The argument FOR removing them is empirical: with the current Cosys ground-truth perception pipeline the failure modes don't occur, and dead safety code carries cost (complexity, surprise factor for future contributors, ongoing maintenance burden, test surface).

**Empirical evidence justifying removal:**

The #710 bisect (full report at [`docs/tracking/BISECT_REPORT_710_AABB_AWARE_REGRESSION.md`](BISECT_REPORT_710_AABB_AWARE_REGRESSION.md)) was followed by a controlled empirical sweep on Cosys scenario 33:

| Sweep | Config | Result | UNSTUCK | Hover-fb | Notes |
|---|---|---|---|---|---|
| Baseline | all 3 ON | PASS 26/26 | 2 | 0 | reference |
| A | only #646 OFF | PASS 26/26 | **0** | 0 | cleaner than baseline |
| B | only #657 OFF | **FAIL 22/26** | 3 | 81 → LOITER | drove the original #710 verdict |
| C | only #647 OFF | PASS 26/26 | 0 | 0 | clean |
| **D run 1** | **all 3 OFF** | **PASS 26/26** | 0 | 21 | unexpected clean PASS |
| **D run 2** | **all 3 OFF (validation)** | **PASS 26/26** | 0 | 20 | reproducible |

The critical signal from Sweep D: PR #657 only appeared load-bearing in Sweep B (only #657 off) because of an **interaction effect** with #646 and #647.  With all three off together, scenario 33 passes cleanly.

The critical signal from Sweep A: `final_clamp_count` was 0 in the baseline run — PR #646's mechanism was already not firing in production even when enabled.  It was solving a problem that doesn't manifest in the current pipeline.

**Why the failure modes are no longer present:** PR #657 (and its companions) were validated on the OLD scenario-33 pipeline (FastSAM masks + Depth Anything V2 → noisy extents, geometric-mismatch hazards).  The current pipeline uses Cosys ground-truth segmentation (`sam.backend = cosys_airsim`, `depth_estimator.backend = cosys_airsim`) which produces pixel-perfect masks and depth, so the fused `estimated_radius_m` / `estimated_height_m` are accurate without any of the geometric safety nets.

**What remains as safety mechanisms after removal:**

- `brake_in_close_regime` (Issue #513) — proximity-based brake arbitration cancels the toward-obstacle component of planned velocity before repulsion runs.  Default `true`.
- `path_aware` (Issue #229) — strips repulsion opposing planned direction.  Default `true`.
- `close_regime_active_` atomic flag with hysteresis — prevents chatter at boundary.
- `OccupancyGrid3D` cell-inflation layer — uses `estimated_radius_m` to inflate the planner's view of each obstacle.  Untouched by #712.

**Caveat — Gazebo scenario 18 (sensor-driven, no HD-map):** Pre-#712 the AABB-aware avoider was masking two unrelated regressions: (a) PX4 SITL ↔ Gazebo bridge timing race on takeoff, (b) runtime-perception over-publishing of obstacles from camera+radar UKF fusion.  Both are filed as separate follow-up issues (#713 + IMPROVEMENTS.md 2026-05-11) and were already present before #712.  Scenario 33 (Cosys) is the regression-pin scenario this PR was validated against.

**For keeping the code (argument we considered but rejected):**

- "The empirical sweep only covers Cosys-33.  Other scenarios or real-hardware paths might still need the safety nets."  Counter: empirical evidence > theoretical worst-case.  If a real-world failure re-emerges, the revert is one PR away and we have the original commits to reference (`760bcc1`, etc.).
- "Two validation runs is a thin statistical basis for removing safety code."  Counter: combined with `final_clamp_count=0` (a stronger zero-firing signal than pass/fail) and the consistent passes in Sweep A + Sweep C + Sweep D (4 PASS runs of all-off-equivalent configs), the evidence is strong.

**For removing (our decision):** Dead safety code is worse than absent safety code — it creates false confidence and adds complexity that obscures the real safety mechanisms.  The empirical sweep is reproducible and documents the conditions under which removal is safe.

**Decision:** Remove all three code paths (PR #712, four commits totalling -1192/+207 net).  Retain the bisect report and this DR as the audit trail.  Treat this DR as the formal record that the removal was deliberate, empirically grounded, and reversible.

**Revisit when:**

1. A new perception backend is introduced that differs materially from Cosys-AirSim ground-truth (e.g. real-hardware stereo + segmentation, or a new SAM-class real-time mask model).  Each new backend's `estimated_radius_m` / `estimated_height_m` confidence properties must be re-validated; if they're noisy in the way FastSAM+DA-V2 was, the AABB-aware safety net may need to come back.
2. Issue #713 (Gazebo SITL rotor-spin-up regression) is resolved — Gazebo scenarios become a routine validation surface again; re-test scenarios 02/17/18/26 to confirm the centroid-distance avoider is sufficient for runtime perception.
3. A real-hardware deployment is approached.  Pre-flight checklist: re-run the Cosys-33 sweep on the production HW + perception pipeline before committing to the leaner avoider.

**Date:** 2026-05-11 (PR #712 review-fault-recovery + review-api-contract — DR-NNN audit trail)

## DR-044: Project frame glossary — NEU is canonical, not ENU

**Question:** Multiple files across the perception + planner stack use inconsistent labels for the same internal frame convention.  Pass 2 of the [#723 themed review](https://github.com/nmohamaya/companion_software_stack/issues/723) flagged at least three different names appearing in source comments and identifiers:

- "NEU" (`tests/test_world_transform.cpp`, `process2_perception/src/main.cpp:646`, `process3_slam_vio_nav/include/slam/ivio_backend.h:358-360`)
- "ENU" (`process2_perception/include/perception/mask_depth_projector.h:28`, `tools/cosys_telemetry_poller/main.cpp:243-268`)
- "aerospace-ENU" (assorted comments)

All three names refer to the **same** internal axis convention used throughout the companion stack: `X = North`, `Y = East`, `Z = Up`.  The variation creates confusion when a new contributor cross-references files, and it was a small but real contributor to the [#727 cold-start race investigation](https://github.com/nmohamaya/companion_software_stack/issues/727) where the initial hypothesis was "ENU↔NED frame conversion bug" (later ruled out by reading both Cosys and Gazebo backend code and confirming the *internal* frame is uniformly NEU).

**Canonical names (project glossary):**

| Source frame | Convention | Used in |
|--------------|------------|---------|
| **NED** (North-East-Down) | `X=N, Y=E, Z=Down` | MAVLink / PX4 wire format; AirSim ground truth; MavlinkFCLink trajectory commands |
| **ENU** (East-North-Up) | `X=E, Y=N, Z=Up` | Gazebo world frame; some gz-msgs Odometry topics |
| **NEU** (North-East-Up) | `X=N, Y=E, Z=Up` | **Project-internal**.  All `drone::ipc::Pose` published on `drone/slam/pose`.  All P4 planner state, including waypoints in `config/scenarios/*.json`.  Body-frame velocity commands inside the companion stack. |
| **FRD** (Forward-Right-Down) | `X=fwd, Y=right, Z=down` | MAVSDK Telemetry body-frame quaternion source |
| **FLU** (Forward-Left-Up) | `X=fwd, Y=left, Z=up` | ROS / some Eigen rotation conventions |

**Decision:** **NEU** is the canonical internal-frame label.  All future code and comments referring to the internal `drone::ipc::Pose` frame should use "NEU" verbatim.  "ENU" and "aerospace-ENU" should not appear in source comments describing internal frame; reserve "ENU" for the *source* frame (Gazebo world) before conversion.

**For not enforcing globally now (rejected option):** "It's just a comment, doesn't change behaviour".  Counter: the #727 investigation lost an hour to chasing an ENU/NEU frame-conversion hypothesis that the comments suggested but the code disproved.  Comment hygiene matters when the code path is safety-critical.

**Action items:**

1. As part of [#734](https://github.com/nmohamaya/companion_software_stack/issues/734) (tracking-doc cleanup), grep for `ENU` in `process2_perception/`, `process3_slam_vio_nav/`, `tools/cosys_telemetry_poller/`, and `common/hal/` — for each hit, decide whether it refers to (a) the Gazebo world source frame (keep), (b) a transient pre-conversion variable (keep with explicit "source: ENU" comment), or (c) the project-internal NEU frame (rename to NEU).
2. Add a one-paragraph glossary section near the top of `docs/design/API.md` cross-referencing this DR.
3. Future PRs touching pose/frame code: PR review should fail on any new "ENU" comment that refers to internal frame.

**Revisit when:** A real VIO backend (MSCKF, ORB-SLAM3, etc.) lands and introduces its own naming.  At that point reconsider whether NEU should be replaced by a more standard ROS or aerospace convention.

**Date:** 2026-05-13 (#723 rollup Phase 3 Pass 2 P3 finding — DR-NNN audit trail)

## DR-045: HAL factories throw `std::runtime_error` instead of returning `Result<unique_ptr<I>, Error>`

**Question:** The project's documented error-handling convention is `Result<T, E>` monadic types — see CLAUDE.md "Error Handling Pattern".  All public APIs in `common/util`, `common/ipc`, and most `process*/` business logic return `[[nodiscard]] Result<...>`.  But every HAL factory in `common/hal/include/hal/hal_factory.h` (`create_camera`, `create_inference_backend`, `create_volumetric_map`, `create_event_camera`, `create_semantic_projector`, `create_fc_link`, `create_radar`, etc.) **throws `std::runtime_error`** on unknown backend strings or configuration failures.  Pass 2 of the [#723 rollup review](https://github.com/nmohamaya/companion_software_stack/issues/723) flagged this as a P3 recurring pattern across the rollup's new HAL types (PR #602, #603, #604).

**Why the divergence exists:** The HAL factory contract was established pre-`Result<T, E>` (back when the project still used exceptions in some places).  Converting all factory call-sites to `Result<unique_ptr<I>, HalError>` is mechanical but touches every `process_main()` that constructs HAL backends — at least 7 files, 30+ call sites.

**For converting now (rejected for this rollup):** Consistent with the project-wide rule; eliminates the exception-handling burden at every `main.cpp`.

**For keeping for now (our decision):**

1. The throw is at startup-time only — never inside the flight-critical hot path.  `process_main()` already wraps backend construction in try/catch and exits cleanly on failure, so the exception is fully observable.
2. The new factories added in this rollup (`create_inference_backend`, `create_volumetric_map`, `create_event_camera`, `create_semantic_projector`) faithfully follow the existing convention — adding a new factory in a non-`Result<>` style would mean refactoring all the others first, which is out of scope for the rollup.
3. Test coverage is via `EXPECT_THROW` already.  Conversion would require rewriting test expectations.

**Decision:** Keep the throw pattern for HAL factories specifically.  Document the deliberate divergence here so future contributors don't propose a piecemeal conversion that leaves the codebase half-and-half.

**Action items:**

1. New HAL factories added in future PRs **may** continue to throw — explicitly *not* required to convert to `Result<>`.  Update CLAUDE.md "Error Handling Pattern" to call out HAL factories as the documented exception (no pun intended).
2. If conversion is ever decided, do it as a single dedicated PR touching all factories + all call sites in one atomic change.

**Revisit when:** Real-hardware deployment is approached and exception-safety guarantees of the startup path need re-auditing for systemd/init scenarios.

**Date:** 2026-05-13 (#723 rollup Phase 3 Pass 2 P3 finding — DR-NNN audit trail)

## DR-046: `std::optional<ScopedLatency>` per-site emplace pattern for opt-in profiler wiring

**Question:** Throughout `process2_perception/src/main.cpp` (lines 79, 151, 289) and `process4_mission_planner/src/main.cpp` (lines 411, 462, 488), every flight-critical thread that emits per-stage latency telemetry wraps each measurement site in the same boilerplate:

```cpp
std::optional<drone::util::ScopedLatency> bench_<stage>;
if (profiler_ptr) {
    bench_<stage>.emplace(*profiler_ptr, "<stage>");
}
// ... work to be measured ...
// bench_<stage> destructs at scope end → records elapsed
```

The pattern was chosen in PR #593 (Improvement #79, see DR-022) to ensure the profiler instrumentation pays **zero cost** when disabled in production (the optional stays empty, no construction, no destructor work).  Pass 1 of the [#723 rollup review](https://github.com/nmohamaya/companion_software_stack/issues/723) flagged this as a P3 recurring pattern — 6 sites today, will grow with every new measurement point.

**Considered alternative — `ScopedBenchTimer` helper that no-ops when null:**

```cpp
drone::util::ScopedBenchTimer bench("<stage>", profiler_ptr);  // no-op constructor if profiler_ptr is null
// ... work ...
```

Half the line count, eliminates the per-site comment drift.  But: the no-op path still pays a function-call cost (even if inlined to nothing) and the constructor takes `profiler_ptr` by raw pointer which the rest of the codebase has been moving away from.

**For the helper (rejected for now):** Cleaner.  Easier to read.  Mechanical refactor.

**For keeping the explicit optional-emplace (our decision):**

1. The `if (profiler_ptr)` branch is loop-invariant and the optimiser hoists / eliminates it cleanly when `profiler_ptr == nullptr`.  Empirically verified in DR-022.
2. The explicit pattern makes the conditional cost visible to every code reviewer — a future contributor adding instrumentation can't accidentally hide a cost behind a helper.
3. The 6 current sites are tolerable.  If the count grows to 15+ sites, revisit.

**Decision:** Keep `std::optional<ScopedLatency>` + `emplace()` pattern for now.  Document here as the deliberate choice so a future contributor doesn't propose a helper-class refactor.

**Action items:**

1. None immediately.  If a 7th or 8th measurement site lands, the threshold for revisit is "if adding more would push us past 10 sites total, design the helper and refactor as one PR".

**Revisit when:**

- New measurement sites push the per-site count past ~10 across the codebase.
- A future profiler backend requires multiple constructor arguments (the helper-class refactor becomes more attractive when the per-site boilerplate is longer).

**Date:** 2026-05-13 (#723 rollup Phase 3 Pass 1 P3 finding — DR-NNN audit trail)

---

## DR-047: Don't extract a shared `FirstObservationGate` helper for the two pre-existing first-observation timestamp patterns (DRY advisory from PR #741 review code-quality P2)

**Question:** PR #741's review surfaced a DRY advisory (code-quality agent, P2): the new ARM-gate debounce in `process4_mission_planner/include/planner/mission_state_tick.h` (member `armable_first_seen_ns_`) and the pre-existing pose-staleness filter in `process4_mission_planner/src/main.cpp:93–95` (local `planner_birth_ns`) share a similar **shape** — `uint64_t` ns captured from `drone::util::get_clock().now_ns()` at a first-observation event, compared against a slack/window. Should we extract a shared helper into `common/util/` to eliminate the duplication?

**The two patterns:**

```cpp
// Pattern A — process-lifetime stale-message filter (#720 / PR #721)
//   process4_mission_planner/src/main.cpp:93
const uint64_t     planner_birth_ns  = drone::util::get_clock().now_ns();
constexpr uint64_t kPoseBirthSlackNs = 100'000'000ULL;  // 100 ms

// Used at the pose-subscribe site:
if (pose.timestamp_ns > 0 && pose.timestamp_ns + kPoseBirthSlackNs < planner_birth_ns) {
    // drop as historical-cache stale
}
```

```cpp
// Pattern B — FSM-state stability debounce (#740 / PR #741)
//   process4_mission_planner/include/planner/mission_state_tick.h:214
uint64_t armable_first_seen_ns_ = 0;  // sentinel "not observing"

// Used in tick_preflight:
if (armable_first_seen_ns_ == 0 || now_ns < armable_first_seen_ns_) {
    armable_first_seen_ns_ = now_ns;     // start window
} else if ((now_ns - armable_first_seen_ns_) < window_ns) {
    return;  // still within window
}
```

**Considered alternative — extract `common/util/StabilityWindow.h`:**

```cpp
class StabilityWindow {
public:
    explicit StabilityWindow(uint64_t window_ns) : window_ns_(window_ns) {}
    bool elapsed(uint64_t now_ns);  // returns true once the window has elapsed
    void reset();                   // re-arm: next call to elapsed() starts a fresh window
private:
    uint64_t window_ns_;
    uint64_t first_seen_ns_ = 0;
};
```

Pattern B becomes a one-liner; Pattern A doesn't fit cleanly because its comparison shape is *different* (`pose.ts + slack < planner_birth_ns` rather than `now - first_seen >= window`). Pattern A is **really an age-guard**, not a stability-window — semantically distinct, just sharing the data type.

**For extraction (rejected for now):**

- Cleaner Pattern B usage site
- Sets up #718 (PREFLIGHT-timeout escalation) to use the helper from day one
- Eliminates the bug-fix-asymmetry risk: a future hardening of one site (e.g. tighter NaN handling) automatically applies to the other

**For keeping the inline patterns (our decision):**

1. **The two patterns are only superficially similar.** Pattern A is `birth_ns + slack` (process-lifetime age guard). Pattern B is `now - first_seen` (resettable elapsed-time window). Forcing them into one helper requires either two different methods (`age_exceeded()` vs `elapsed()`) or a template that parameterises both, and the resulting abstraction is harder to read than the two inline patterns.
2. **YAGNI with 2 data points.** A helper designed for 2 callers tends to over-fit the current shape; a helper designed for 3 informed by a real third use case has a much better chance of being right.
3. **The bug-fix-asymmetry risk is real but bounded** — both sites are in the same `process4_mission_planner` directory and review pipelines for either will see the other in the diff context for nearby PRs.

**Decision:** Keep the two inline patterns. Track #742 as the follow-up that will extract `common/util/StabilityWindow.h` (or a similarly named abstraction) when **a third similar pattern lands** — most likely from #718.

**Specific asymmetries the future extractor must handle:**

- **Comparison direction** — Pattern A asks "is the publisher timestamp older than my birth?" (compares two unrelated timestamps). Pattern B asks "has enough time elapsed since I first observed X?" (compares one timestamp to a "now" delta). A shared helper must support both shapes — likely as two methods, not one.
- **Underflow guard** — Pattern B has `if (now_ns < first_seen_ns_)` to handle mock-clock rewind. Pattern A doesn't need this (the timestamps it compares are independent). If extracted, the underflow guard must be on whichever method does `now - first_seen`.
- **Reset semantics** — Pattern A never resets (`planner_birth_ns` is `const` for process lifetime). Pattern B resets on every `armable=false` tick. The helper must support both lifetimes (e.g. one-shot via `const` instance, resettable via a `reset()` method).
- **Sentinel encoding** — Pattern B uses `first_seen_ns_ == 0` as "not yet observed". The helper should use the same sentinel, OR use `std::optional<uint64_t>` for clarity. Mixed sentinel conventions across the two existing patterns would be a stealth bug.

**Action items:**

1. **None immediately** — this is the audit trail for "considered, declined now".
2. **Filed [#742](https://github.com/nmohamaya/companion_software_stack/issues/742)** as the follow-up — extraction work scheduled to happen alongside #718 (PREFLIGHT-timeout escalation), which will add the third pattern that informs the right abstraction.
3. **In any future PR touching either of the two existing sites**, the reviewer MUST cross-reference the other site to check for asymmetric maintenance. Reference this DR-047 entry in the PR body if such a change lands.

**Revisit when:**

- #718 (PREFLIGHT-timeout escalation) lands, adding a third similar timing pattern in `tick_preflight()` — that's the natural trigger to extract.
- Any other PR introduces a fourth "first-observation timestamp" pattern in `process[1-7]_*` — if we get to 4 without a helper, we've over-deferred.
- A real bug is found that's caused by asymmetric maintenance between Patterns A and B — that would invalidate the "tolerable for now" claim and force immediate extraction.

**Backend-independence note (#740 follow-up):** the duplication does NOT introduce a backend-specific failure mode. Both patterns are agnostic to the active HAL backend (Gazebo MAVSDK, Cosys SimpleFlight, Cosys + PX4 HIL, real hardware) because they consume IPC types (Pose, FCState) produced by the HAL layer. What varies per-backend is the *probability* of each pattern firing and the *appropriate tuning value* (slack, window), both of which are already config-driven via `drone::Config`. Deferring extraction doesn't expose us to sim-vs-hardware divergence — it exposes us to maintenance drift only.

**Date:** 2026-05-13 (PR #741 review code-quality P2 advisory — DR-NNN audit trail)

---

## DR-048: `FAULT_PLANNER_STALL` is FSM-state-agnostic — same fault bit drives DISARM on the ground and LOITER in the air

**Context:** PR #775 (Issue #718 + partial #765) added `PlannerStallHandler` and `FAULT_PLANNER_STALL` (`1 << 14`) — a single fault bit raised whenever the `ThreadWatchdog` declares the planner-tick thread stuck. P4's response to that bit is asymmetric by FSM state:

- **Pre-flight (PREFLIGHT/READY_TO_ARM)** — stuck planner means the drone is on the ground but its mission loop is dead. Safe action: **DISARM**.
- **In-flight (TAKING_OFF / NAVIGATING / RTL / LANDING / any airborne state)** — disarming kills the motors and drops the airframe. Safe action: **LOITER** (autopilot maintains altitude/position via the FC failsafe, allowing operator intervention).

The fault bit itself does NOT carry the FSM context. The handler raises the bit; `MissionStateTick`'s next pass reads the FSM state and the bit, then chooses DISARM vs LOITER.

**Considered alternative — split into two bits:**

```cpp
FAULT_PLANNER_STALL_GROUND = (1 << 14),   // → DISARM
FAULT_PLANNER_STALL_AIR    = (1 << 15),   // → LOITER
```

PR #775 rollup review (review-api-contract P3) flagged this as a "dual-purpose semantic" — one bit, two distinct operator-visible behaviours, fault telemetry can't tell from the bit alone which response fired.

**For splitting (rejected):**

- GCS / telemetry analyst can derive response from the fault bit without the FSM snapshot.
- Audit logs / black-box recorder have unambiguous "what happened" rather than "stuck planner — response depended on state at the time".
- Future fault bits driving asymmetric responses (e.g. `FAULT_FC_PREFLIGHT_TIMEOUT`, `FAULT_BATTERY_CRITICAL`) might benefit from the same split, creating a consistent convention.

**For one bit (our decision):**

1. **The fault is the *cause*, not the *response*.** The planner-tick thread being stuck is a single fact about the system. Splitting it into two bits encodes the FSM-state-snapshot into the fault model, which couples the fault layer to the FSM layer. Today FSM has 9 states; tomorrow if NAVIGATING splits into NAVIGATING_TO_WAYPOINT and NAVIGATING_TO_RTL, do we need 3 bits? 4?
2. **The response *is* recoverable from the existing data.** Every fault flag co-occurs with an FSM state in the IPC snapshot (FaultSnapshot includes the active FSM state). GCS analyst sees `fault_flags=PLANNER_STALL` AND `fsm_state=NAVIGATING` → "this triggered LOITER, not DISARM". No information loss.
3. **The same pattern already exists for `FAULT_VIO_LOST` and `FAULT_FC_LINK_LOST`** — both raise a single bit and let FSM decide the response (LOITER in-flight, DISARM on the ground for VIO_LOST; nothing on the ground but RTL in-flight for FC_LINK_LOST). Splitting only PLANNER_STALL would create an inconsistency with the rest of the fault catalogue.
4. **Splitting requires the *raiser* to know the FSM state.** Today `PlannerStallHandler::on_thread_stuck()` runs in the watchdog scan thread and doesn't see the FSM. Splitting would force the handler to either subscribe to FSM updates (new IPC dependency, race window) or defer the raise until the next tick (defeats the purpose — the tick thread is stuck). The current pattern decouples raiser from interpreter, which is the right boundary.

**Decision:** Keep `FAULT_PLANNER_STALL` as a single FSM-state-agnostic bit. The response asymmetry (DISARM pre-flight, LOITER in-flight) lives in `MissionStateTick::evaluate_fault_response()`, which is the right place for state-aware response selection. Document this convention in `common/ipc/include/ipc/ipc_types.h` (the FAULT_PLANNER_STALL doc comment already notes the dual response in PR #775).

**Audit-trail recovery:** GCS / black-box analyst can answer "what response fired?" by joining `FaultSnapshot.flags & FAULT_PLANNER_STALL` with `MissionStateSnapshot.fsm_state` at the same timestamp. Both are already published. No new telemetry needed.

**Revisit when:**

- A real GCS feature lands that needs response-selection at fault-bit granularity (e.g. operator filter UI). At that point either add a derived `last_fault_response` field to `FaultSnapshot` (cheaper than splitting bits) or revisit this DR.
- A new FSM state has a *third* response to planner-stall (not DISARM, not LOITER — something genuinely new). At three responses, the single-bit-multi-response argument weakens; revisit then.
- The same single-bit-with-state-dependent-response pattern accumulates a fourth instance (today: VIO_LOST, FC_LINK_LOST, PLANNER_STALL). At four, codify the convention explicitly in `docs/design/API.md` rather than relying on individual DR entries.

**Date:** 2026-05-16 (PR #776 rollup review-api-contract P3 — DR-NNN audit trail)

---

## DR-049: Running an async-signal-safe stack-trace handler ON a flight-critical thread (Issue #765)

**Context:** PR 2 of Issue #765 wires `drone::util::StackTraceCapture` into
`PlannerStallHandler`. When `ThreadWatchdog` detects the `planning_loop`
thread stuck, the watchdog (observer) thread calls `capture_and_log()`,
which delivers `SIGUSR1` via `tgkill` to the stuck thread; the signal
handler **executes in the stuck thread's own context** and writes a
`backtrace()` into a static buffer.

`planning_loop` is a flight-critical thread (P4 mission planner tick).
CLAUDE.md §"Observability on flight-critical threads" says mutex-protected
observability MUST NOT run on flight-critical threads without a documented
DR. Running a *signal handler* on that thread is the same class of concern
(it borrows the thread's stack + execution), so this DR records why it is
acceptable.

**The concern:** A handler running on a control-loop thread could (a)
introduce priority inversion (if it took a lock a lower-priority thread
holds), or (b) perturb the control loop's timing/state.

**Why it is acceptable here:**

1. **Lock-free + allocation-free handler.** The handler touches only
   lock-free `std::atomic` (asserted `is_always_lock_free`) + `backtrace()`
   into a pre-allocated static buffer (glibc unwinder pre-warmed at
   `install()` time, outside signal context). No mutex → no priority-
   inversion channel. No malloc → no allocator-lock contention.
2. **It only fires AFTER the thread is already stuck** ≥ the watchdog
   threshold (`thread_stuck_threshold_ms`, default 5 s). By definition the
   thread is doing no useful control work at that point — there is no live
   control state to contaminate. The capture is a post-mortem of an
   already-failed loop, not an interruption of a healthy one.
3. **`SA_RESTART` prevents syscall-state perturbation.** If the stuck
   thread is blocked in a syscall, the handler runs and the syscall
   resumes transparently — we do NOT inject an `EINTR` into an unaudited
   call site. (See stack_trace_capture.h for the full SA_RESTART rationale;
   it is a deliberate deviation from `SignalHandler`'s `sa_flags=0`.)
4. **Config-gated + rate-limited.** Off via `watchdog.stack_trace.enabled`;
   at most one capture per `min_interval_s` (default 30 s) per thread, so
   even a persistently-stuck thread is signalled at most once per window.
5. **The mutex-using parts run on the OBSERVER thread.** Symbolisation
   (`backtrace_symbols`, mallocs) and logging happen on the watchdog scan
   thread in `capture_and_log()`, never in the signal handler. The watchdog
   is not a control loop; this matches the existing DR-022 precedent (the
   LatencyProfiler mutex dump is also taken on the watchdog side).

**Decision:** Run the capture. The combination of lock-free/alloc-free
handler + fires-only-post-stall + SA_RESTART + config-gate + rate-limit
makes it strictly safer than the failure it diagnoses (a silent 31 s stall
ending in PX4 disarm — the #765 motivating incident). The alternative —
no userspace trace — leaves every recurrence a shrug.

**Revisit when:**

- A real-time scheduling policy (SCHED_FIFO/RR) is adopted for
  `planning_loop` on hardware — re-examine whether signal delivery to a
  high-priority thread has any latency cost worth gating more tightly.
- The mutex-snapshot follow-up (still deferred — "which locks does the
  stuck thread hold") is designed: that WILL need lock introspection and
  must not reintroduce a priority-inversion path; it gets its own DR.

**Date:** 2026-06-14 (Issue #765 PR 2 — flight-critical-thread signal analysis)

---

## DR-050: Camera dynamic-occupancy gates default OFF (opt-in), not voxel-parity ON (Issue #764)

**Question:** When adding ground-reject + N-hit confirmation gates to the camera `update_from_objects()` path (to stop `color_contour` ground ghosts flooding the dynamic planning grid), should the gates default ON at the voxel path's value (ground floor 0.3 m), or default OFF (opt-in per scenario)?

**Context:** This is a perception-*suppression* change — a gate that DROPS obstacle detections before the planner. The asymmetric-cost calculus inverts: dropping a *ghost* = brief over-caution (cheap); dropping a *real* obstacle = collision = lost vehicle (catastrophic).

**Arguments for default-ON (voxel parity, 0.3 m floor):**
- Consistency: the voxel path already rejects sub-0.3 m world-z; matching it makes the two ingestion paths behave the same.
- Catches the ghost class everywhere by default, not only where a scenario remembers to opt in.

**Arguments for default-OFF (opt-in):**
- A non-zero default ground floor is a behaviour change for *every* scenario — it broke 7 existing tests that place obstacles below 0.3 m, and could silently drop a low real obstacle in a scenario we haven't validated.
- The safe direction for a suppression gate is false-accept (keep the obstacle). Default-OFF preserves prior behaviour; a scenario opts in only after its N=10 validation proves the gate doesn't drop real obstacles.
- Matches the codebase idiom (`max_static_cells=0`, `promotion_hits=0`, `static_cell_ttl_s=0` all mean "disabled").

**Decision:** **Default OFF** — `min_obstacle_altitude_m=0.0` (no floor; gate active only when `> 0`) and `dynamic_confirmation_hits=1` (no confirmation). Scenario 02 opts into `0.5` / `3`. The reactive `ObstacleAvoider3D` backstops the confirmation window; thresholds are config-clamped so a bad value can't suppress obstacles entirely or indefinitely. Generalised into the CLAUDE.md rule "Perception-suppression gates must fail safe" + the `review-data-plumbing` (P1) and `review-fault-recovery` checklists.

**Revisit when:** the N=10 scenario-02 sweep proves the opt-in values are safe and effective → consider promoting the ground floor to a low default (voxel parity) per-scenario once each is validated; or when a less ghost-prone detector replaces `color_contour`.

**Date:** 2026-06-15 (Issue #764)

---

## DR-052: Takeoff promotion gate keys on drone altitude (opt-in), not FSM phase (Issue #789)

> DR-051 is reserved for the #764 D*Lite A*-fallback PR (#788) — both PRs branch off #787, so #789 uses DR-052 to avoid a merge-time number collision.

**Question:** To stop the occupancy grid promoting the ground/landing-pad it sees during takeoff (saturating the static cap → planner STUCK → collision, Issue #789), should promotion be suppressed by (a) the drone's *altitude* being below a floor, or (b) the *FSM phase* (a `set_takeoff_pause` mirroring the existing `set_landing_pause`)?

**Context:** Perception-*suppression* change — the asymmetric-cost calculus inverts (dropping a real obstacle = collision). The existing `landing_pause_` (Issue #340) is FSM/phase-driven and wired from the tick loop; the natural instinct is to mirror it for takeoff.

**Arguments for FSM-phase pause (mirror `landing_pause_`):**
- Symmetric with the landing case; one obvious pattern.
- The FSM already knows it is in `TAKEOFF`.

**Arguments for altitude gate:**
- Self-contained in `OccupancyGrid3D` (it already receives the pose) → unit-testable in microseconds without standing up the FSM. The phase pause needs an integration test.
- Decoupled from phase *names*: it also protects any low-altitude window, and aligns with the existing cold-start data-hygiene rule ("don't trust first/low observations").
- The actual hazard is physical ("the drone is low, so it sees the ground"), which is precisely altitude — not a phase label.

**Decision:** **Altitude gate** — `min_promotion_altitude_m` (default `0.0` = off; gate active only when `> 0`), folded into the existing `can_promote` decision in `update_from_objects()`. Suppresses **static promotion only**; the dynamic TTL layer (`occupied_[c]` stamping precedes the promotion block) and the reactive `ObstacleAvoider3D` stay active, so a real obstacle is still seen reactively and promotion resumes once the drone clears the floor — well before it translates toward obstacles. Threshold config-clamped to `[0,5]` m so a typo can't suppress promotion for the whole flight. Scenario 18 opts into `2.0` m. Real obstacles cannot be permanently dropped: the drone climbs vertically off the pad (no obstacles directly above), the reactive layer covers the takeoff window, and the gate is opt-in + clamped. Follows the CLAUDE.md "Perception-suppression gates must fail safe" rule.

**Revisit when:** a scenario needs promotion *during* a low-altitude cruise (e.g. terrain-following below the floor) — then the gate would need an AGL reference instead of raw pose-z, or a phase qualifier; or once a less ground-prone detector + selective radar make the takeoff flood impossible at the source.

**Date:** 2026-06-16 (Issue #789)

---

## DR-053: Camera dynamic-add gates are NOT exempted for radar-confirmed tracks (Issue #764 / Copilot PR #787)

> DR-051 (#788 A*-fallback) and DR-052 (#789 takeoff gate) are taken by sibling PRs branched off #787; this review-fix uses DR-053 to avoid a merge-time collision.

**Question:** Copilot (PR #787) flagged that the `update_from_objects` ground-altitude reject and the N-hit dynamic-confirmation gate apply to *all* objects, including radar-confirmed tracks — a radar-confirmed obstacle below the floor is dropped, and a radar-confirmed cell is delayed N frames. Should radar-confirmed detections **bypass** these suppression gates?

**Context:** Perception-*suppression* change; the asymmetric-cost calculus inverts (dropping a real obstacle = collision). The instinct is "radar is accurate → never suppress it."

**Arguments for exempting radar-confirmed:**
- Radar gives accurate range from the first hit; a radar-confirmed obstacle is the high-confidence subset and shouldn't be treated as a ground ghost.
- Matches the spirit of `require_radar_for_promotion` (radar as the trusted modality).

**Arguments against exempting (keep gates uniform):**
- **Radar selectivity is backend-dependent.** In Gazebo SITL the "radar" is `gpu_lidar`-emulated and returns ground hits *everywhere* — "radar-confirmed" then loses meaning, and a blanket exemption re-opens the exact ground flood these gates exist to stop (Issue #789, scenario 18). The grid cannot tell selective radar from non-selective lidar.
- **The reactive avoider already backstops it.** `ObstacleAvoider3D` consumes the raw `DetectedObjectList` directly — *not* this grid — so a real low / not-yet-confirmed obstacle dropped from the **planning** grid is still avoided in **real time**. The gates only delay/skip *strategic* grid promotion, never real-time avoidance.
- The gates are opt-in (default 0 = off) and config-clamped; only scenarios whose obstacle geometry has been validated against the thresholds enable them.

**Decision:** **Do not exempt radar-confirmed tracks.** Keep the gates uniform; rely on the reactive `ObstacleAvoider3D` (raw-`DetectedObjectList` consumer) as the real-time backstop, per the CLAUDE.md "Perception-suppression gates must fail safe" rule. The NaN guard stays universal (a non-finite position is garbage regardless of modality). Clarified the in-code comment to state the gates apply to all objects and why. (Copilot's companion suggestion — a test asserting radar bypasses the gates — is therefore moot: there is no bypass.)

**Revisit when:** a backend exposes a *selective* radar with a trustworthy per-detection confidence that can distinguish real returns from ground clutter → then a radar-confidence-qualified exemption (not a blanket `has_radar` one) could let high-confidence radar obstacles promote immediately while still rejecting lidar ground returns.

**Date:** 2026-06-16 (Issue #764 / Copilot PR #787)

---

## DR-051: A* fallback vs fixing D*Lite g-consistency (Issue #764 mode b)

**Question:** D*Lite's greedy `extract_path()` stalls on lazy-g-field holes — extraction fails at `occupied=0` with a finite `g(start)`. Fix by (A) adding a guaranteed-correct A* fallback on extraction failure, or (B) making `compute_shortest_path` produce a fully consistent g-field so greedy extraction never stalls?

**Arguments for (B) fix g-consistency:**
- Addresses the "true" root (the g-field) rather than adding a second search.
- One algorithm to maintain, not two.

**Arguments for (A) A* fallback:**
- D*Lite's incremental-consistency invariant (key/`km` updates across a moving start + changing goal) is subtle and easy to get *subtly* wrong; a latent mistake in a safety-critical planner is high-cost and hard to catch.
- A* is simple, well-understood, and **guaranteed correct** (finds a path iff one exists) using the *same* `cost()` — it cannot bypass an obstacle.
- Invoked only on the rare stall path, so the common case keeps D*Lite's fast incremental walk; at this grid scale A* is sub-millisecond.
- Matches the "make the planning algorithm robust" intent — defense-in-depth over a fragile invariant.

**Decision:** **(A)** — A* fallback on greedy-extraction failure. `astar_fallback_count()` / `astar_fallback_recovered()` telemetry surfaces how often the g-field leaves holes.

**Revisit when:** the fallback fires on the majority of replans (then D*Lite's incremental value is illusory at this grid scale → switch the backend to plain A*), or the g-consistency root cause is independently fixed.

**Date:** 2026-06-15 (Issue #764 mode b)

## DR-054: Promoted static-cell decay ON by default (30 s), not permanent (Issue #799 Phase B)

**Question:** Should promoted (static-layer) occupancy cells stay permanent by default (`static_cell_ttl_s = 0`), or decay after N seconds without re-observation (default 30 s)?

**Arguments for permanent (0 — the previous default):**
- Perfect memory for a genuinely static world — an obstacle confirmed once is never forgotten, even if the drone loses sight of it.
- No risk of a real obstacle decaying while briefly out of the sensor FOV.

**Arguments for decaying (30 s — the new default):**
- "Remember forever" + imperfect perception = unbounded accumulation. In scenario 18, radar ghosts + the drone's own wake grew the static layer **25→650 cells over one flight**, sealing every navigable corridor → D*Lite found no path → mission incomplete (#799). Permanent memory is only safe if perception is perfect; it isn't.
- Decay clears the drone's *un-re-observed wake* — cells it flew past, no longer relevant to the path ahead — reopening return corridors.
- Made fail-safe: **(a)** re-observed cells REFRESH their timestamp — both the voxel path and, new in Phase B, the object/radar path (`inflate_disk_at_cell_`), so a real obstacle *still in view* never decays; **(b)** HD-map cells (`add_static_obstacle`) are timestamp-free and exempt; **(c)** the reactive `ObstacleAvoider3D` backstops any cell that decays while the drone still approaches it.
- 30 s ≫ a typical inter-waypoint leg (~19 s in scenario 18), so an obstacle ahead is re-observed (refreshed) long before it could decay.

**Decision:** **Decaying, default 30 s.** Set in `config/default.json` (runtime SSOT) + the struct/ctor defaults (code fallback). An explicit `0.0` remains the documented "disabled → permanent" sentinel for any scenario that genuinely needs long-term static memory. The object-path re-observation refresh was added as part of this change — a fail-safe gap the decay unit tests surfaced (a continuously-radar-observed obstacle would otherwise have decayed after 30 s).

**Revisit when:** a scenario needs long-term static memory (map-then-revisit), OR 30 s proves too short (a real obstacle decays during a legitimately long out-of-FOV window → raise it) or too long (ghosts persist long enough to matter → lower it). Already per-scenario tunable via config.

**Date:** 2026-06-30 (Issue #799 Phase B)
