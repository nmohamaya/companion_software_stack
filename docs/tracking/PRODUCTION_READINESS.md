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
**Current state:** Some per-tick diagnostic logging uses `spdlog::info` instead of `spdlog::debug`. See `project_production_debug_cleanup` notes for full list.

**What to address:** Audit all `DRONE_LOG_INFO` in hot paths, gate behind `spdlog::debug` or `spdlog::should_log()`.

**When to address:** Before production deployment.

---

## Performance: Camera Frame Allocation Churn

**Severity:** Low-Medium (allocator pressure, not correctness)
**Current state:** CosysCameraBackend creates a new `FrameData` with `vector::assign()` on every frame, allocating and copying RGB data each time. At 30 FPS with 1280x720 RGB this is ~2.7 MB/frame of allocation.

**What to address:**
- Pre-allocate `FrameData` buffers in the TripleBuffer slots to match expected frame size
- On retrieval, resize (no-op if same size) and copy directly into the pre-allocated buffer
- Eliminates per-frame heap allocation — only copies remain (unavoidable with RPC response)

**When to address:** When profiling shows allocator contention on constrained hardware (Jetson Orin). Not a concern for desktop/cloud simulation.

---

## Performance: Radar Math Extraction

**Severity:** Low (testability, not performance)
**Current state:** Cartesian-to-spherical conversion, ground filter, and SNR model are inline in `CosysRadarBackend::poll_loop()` inside `#ifdef HAVE_COSYS_AIRSIM`. Pure arithmetic but untestable without the SDK.

**What to address:**
- Extract to free functions: `cartesian_to_spherical(x, y, z)`, `compute_snr_db(range)`, `is_ground_return(elevation)`
- Place outside the `#ifdef` guard so they can be unit-tested independently
- Guards against `std::asin` domain errors on corrupt radar points

**When to address:** Before hardware radar integration. Good first issue for a new contributor.

---

## Sim-to-Real Transition Map — What Changes on Real Hardware

**Severity:** Not a bug, but a reference. Answers "what do we actually flip when we move from Cosys / Gazebo to a Jetson + real camera?"

**Architectural anchor:** The HAL config-key pattern (`config/*.json → backend: "..."`) means **most changes are one-line swaps** in a hardware profile. Code doesn't know the backend changed. The hard work is on the hardware side: calibration, sensor wiring, power/thermal validation.

### What changes — perception & depth

| Sub-system | Sim (Cosys dev profile) | Real hardware (Jetson Orin + Holybro X500 v2) |
|---|---|---|
| `video_capture.mission_cam.backend` | `cosys_airsim` (UE5 RPC render) | `v4l2` (USB/CSI) or `libargus` (NVIDIA CSI) — Issue #32 |
| `DepthCam` | Simulator's ground-truth depth, available as an oracle | **Does not exist.** No equivalent sensor on a production drone. |
| `perception.depth_estimator.backend` | `depth_anything_v2` (or `cosys_airsim` GT in cloud profile) | `depth_anything_v2` monocular, **or** stereo SGBM if airframe has stereo, **or** time-of-flight / lidar if present |
| DA V2 calibration coefficients | Fitted against UE5 Blocks via `DepthCam` | Must be **re-fitted against real scenes** before first flight. Blocks-trained `(a, b)` will not generalise. |
| Texture gate | Active (config-gated) | Active — unchanged. Gradient-based, sensor-agnostic. |
| Max-depth clamp (20 m) / ground-plane filter (z<0.3) | Defensive bandages for DA V2 drift | **More necessary** — real cameras add noise; sky/horizon pixels are still garbage |

### What changes — SLAM / VIO / comms

| Sub-system | Sim | Real hardware |
|---|---|---|
| `slam.vio.backend` | `cosys_airsim` (GT pose republisher) or `gazebo` (same pattern) | Real stereo VIO (Epic #497 SWVIO) or ORB-SLAM3 / VINS-Fusion (Issue #37) |
| `slam.imu.backend` | Cosys simulated IMU + Gaussian noise | Real MPU6050 / BMI088 via I²C or flight-controller IMU stream |
| `comms.mavlink.backend` | `cosys_rpc` (SimpleFlight in UE5) | `mavsdk` — real PX4 over UART / USB |
| Collision detection | `simGetCollisionInfo` RPC (physics ground truth) | IMU jerk spike + MAVLink events (Issue #505) |
| Camera intrinsics | Near-perfect UE5 pinhole | Real intrinsics + distortion model (Brown-Conrady / KB4) — Issue #601 |
| Camera-body extrinsic | Declared in `cosys_settings.json` | Measured physically per airframe build. The `R_body_from_cam` rotation from Fix #55 generalises; the translational offset + fine rotation are per-unit |

### The three categories of "hidden" work

**1. Calibration drift between environments.** A DA V2 coefficient fit from a UE5 Blocks map won't match a real forest, warehouse, or proving ground. This requires one of:

- **Pre-flight calibration procedure** — park the drone in the real environment, run a 1–2 min data-collection flight, re-fit coefficients, save per-mission config overlay.
- **Robust-scale architecture** — use DA V2 only for relative depth; let a physical sensor (stereo / ToF) anchor absolute scale.
- **Domain-specific fine-tune** — full retrain of DA V2 weights on real-environment data. Largest investment; highest quality.

**2. Ground truth evaporates.** `DepthCam` in sim is the grading oracle — on real hardware, no pixel has a "correct answer". Validation becomes:

- **Instrumented ground runs** — tape measure + marked objects at known distances.
- **Stereo-as-reference** — if the airframe has stereo, use stereo matching as the reference-of-record and DA V2 as backup / densifier.
- **Fleet telemetry** — collision events (per #505) become the only "grades" once flying. Painful feedback loop.

**3. Environmental factors are no longer pristine.** Motion blur, vibration, lighting variation, rolling shutter, thermal noise, rain, fog — all stress-test the perception + VIO pipeline in ways UE5 does not exercise by default.

### What stays the same (architectural)

- The 7-process architecture + every IPC wire type.
- Every HAL interface (`IDepthEstimator`, `ICamera`, `IObstacleAvoider`, `IVIOBackend`, etc.) — only the backends change.
- Everything in `common/` (avoider, planner, tracker, watchdog, fault manager, grid).
- The scenario / config / pass-criteria framework (hardware scenarios can be added the same way).
- The DA V2 ONNX model file itself. Same weights, same code path.
- Texture gate + max-depth clamp + ground-plane filter — defensive tuning that only gets *more* useful on real cameras.

### Why this matters for currently-sim-scoped work (e.g. #616 DA V2 calibration)

Work that looks like "sim polish" is often **shipping the machinery the real deploy needs**. Example: #616 adds config-driven DA V2 calibration coefficients + texture gate. On real hardware the coefficients *have to* come from config — you can't bake them into the binary because they change per-environment. The `DepthCam`-based oracle is how we grade the machinery during development; the machinery itself is what flies.

### Implementation roadmap (tracked separately, listed here for context)

- **#25** — Real Drone Deployment epic (phases 2 + 3: V4L2, stereo calibration, real VIO)
- **#32** — V4L2 Camera HAL backend
- **#37** — Real VIO backend (ORB-SLAM3 / VINS-Fusion)
- **#38** — Stereo camera calibration pipeline
- **#393** — Mono depth improvement survey (parent of #616)
- **#491** — Holybro X500 v2 + Jetson Orin Nano validation tier
- **#497** — Custom SWVIO epic
- **#505** — Real-hardware collision detection via IMU jerk
- **#601** — CameraIntrinsics + lens distortion model (Brown-Conrady / KB4)
- **#251** — Production deployment: native build & run on Jetson Orin Nano

**When to address:** The roadmap above is the "when" — sim-first work (PATH A, #513, #616) builds algorithms + validation infra; hardware epics (#25, #491, etc.) handle sensor wiring + calibration + deploy. Anything deferred with "real hardware" rationale should cross-reference this section and the relevant epic.
