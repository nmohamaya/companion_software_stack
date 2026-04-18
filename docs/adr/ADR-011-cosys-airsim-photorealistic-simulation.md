# ADR-011: Cosys-AirSim Photorealistic Simulation Layer

| Field | Value |
|-------|-------|
| **Status** | Accepted |
| **Date** | 2026-04-14 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | Epic #431, ADR-009 (Tier 1/Tier 2 simulation), ADR-006 (HAL strategy), Issue #430 (ML depth estimation) |

---

## 1. Context

### The ML Perception Gap

We're adding ML-based perception to the stack: YOLOv8 object detection (existing) and Depth Anything V2 monocular depth estimation (Issue #430). These models are trained on real-world imagery (COCO, mixed-data depth datasets) and fundamentally cannot be validated in Gazebo:

- **YOLO detects nothing useful in Gazebo.** Gazebo renders flat-shaded geometric primitives (colored cylinders, boxes). YOLO is trained on COCO's 80 real-world classes — it cannot detect a red cylinder as a "person" or a green box as a "vehicle." Our existing `color_contour` detector works in Gazebo precisely because it doesn't use learned features, but it's not the production detector.

- **ML depth models have severe domain gap on Gazebo renders.** Depth Anything V2 learns monocular depth cues from real-world textures, lighting, shadows, and perspective. Gazebo's flat-shaded Ogre2 rendering lacks these cues. In testing, DA V2 produces near-uniform depth maps on Gazebo scenes — the output is not meaningful for validation.

- **Gazebo has no native radar sensor.** We currently use `gpu_lidar` configured as a wide-beam approximation. This misses radar-specific behaviours (Doppler velocity, RCS-based detection probability, multipath) that the UKF fusion engine (merged in PR #241) depends on for radar-camera trust weighting.

**Bottom line:** Gazebo validates flight dynamics, IPC wiring, FSM logic, fault handling, and nav/planning — it does this well and continues to. But it **cannot validate the ML perception pipeline**, which is becoming the core of obstacle avoidance. We need a photorealistic simulation layer.

### The Hardware Constraint

The development machine runs an NVIDIA T1000 with 4GB VRAM — insufficient for UE5 rendering (~8GB minimum). Any photorealistic simulator must run on cloud GPU infrastructure. This shapes the architecture: cloud-only Tier 3, with local Gazebo (Tier 2) and simulated (Tier 1) unchanged.

### The Aerial Domain Gap

Even with photorealistic rendering, COCO-trained models perform poorly on aerial perspectives:
- YOLO trained on COCO scores ~50% mAP on street-level imagery but only ~15-20% mAP on aerial drone footage (a ~60% performance drop)
- Community-trained YOLOv8 checkpoints on VisDrone (aerial dataset, 265K images) recover to ~30-38% mAP — a 2x improvement over COCO baselines
- No aerial-specific depth model exists; DA V2 is the best generalist, and radar compensates for metric depth errors

This means photorealistic simulation is necessary but not sufficient — future ML training on aerial data (Epic #437) is also needed. This ADR covers the simulation infrastructure; the training pipeline is a separate concern.

---

## 2. Decision

### Add Tier 3: Photorealistic SITL via Cosys-AirSim

Extend the existing two-tier simulation model (ADR-009) with a third tier:

| | Tier 1 — Software Integration | Tier 2 — Physics SITL | **Tier 3 — Photorealistic SITL** |
|---|---|---|---|
| **Simulator** | None (simulated backends) | Gazebo Harmonic | Cosys-AirSim (UE5) |
| **Runner** | `run_scenario.sh` | `run_scenario_gazebo.sh` | `run_scenario_cosys.sh` |
| **Base config** | `default.json` | `gazebo_sitl.json` | `cosys_airsim.json` |
| **Requires** | Linux | GPU + Gazebo + PX4 | Cloud GPU (A10G 24GB) |
| **CI cadence** | Every PR | Nightly / manual | On-demand (perception PRs) |
| **Validates** | IPC, FSM, config, faults | Nav, planning, physics | ML perception, depth, radar fusion |
| **Rendering** | None | Ogre2 (functional) | UE5 (photorealistic) |
| **Radar** | Simulated | gpu_lidar hack | Native radar model |
| **PX4** | SimulatedFCLink | MAVSDK (real PX4 SITL) | MAVSDK (real PX4 SITL) |
| **ML models** | Not exercised | Not meaningful (domain gap) | Full pipeline (YOLO + DA V2) |

**Key principle: each tier validates what the tier below cannot.** Tier 1 validates software integration without physics. Tier 2 validates physics without photorealism. Tier 3 validates ML perception with photorealism.

### Why Cosys-AirSim

We evaluated 5 simulators against our requirements:

| Criterion | Gazebo (keep) | **Cosys-AirSim** | Flightmare | FlightForge | Isaac Sim |
|-----------|--------------|-------------------|------------|-------------|-----------|
| Rendering | Ogre2 (functional) | **UE5 photorealistic** | Unity (moderate) | UE5 photorealistic | RTX ray-traced |
| PX4 SITL | Native | **Native** | No | No | ROS2 bridge |
| Radar | gpu_lidar hack | **Native radar model** | None | None | None |
| Docker/Headless | gz-server headless | **Documented UE5 headless + Docker** | No Docker | Early stage | Omniverse stack |
| C++ API | gz-transport | **RPC client (msgpack)** | Python only | Python only | Python/C++ |
| Maintenance | OSRF, very active | **Univ. Antwerp, active** | ~Abandoned (2021) | Very new, unstable | NVIDIA, heavy |
| Min GPU | Intel iGPU | RTX 2070+ / A10G | GTX 1070+ | RTX 3070+ | RTX 3090+ |
| License | Apache 2.0 | **MIT** | MIT | MIT | Proprietary |

**Cosys-AirSim wins on three decisive criteria:**

1. **Native PX4 SITL** — Only option besides Gazebo. Our `MavlinkFCLink` works unchanged (TCP URI instead of UDP). Flightmare and FlightForge lack PX4 support entirely, requiring a custom flight controller bridge.

2. **Native radar sensor** — Eliminates our `gpu_lidar` approximation. Critical for validating the UKF fusion engine's radar-camera trust weighting. No other candidate offers radar.

3. **Active C++ API with Docker support** — Maps cleanly to our HAL interfaces. Flightmare is Python-only and abandoned since 2021. FlightForge is too early-stage for production use. Isaac Sim is proprietary and requires the full Omniverse stack.

**Why not Isaac Sim despite superior rendering:** Isaac Sim's RTX ray tracing produces the best visual fidelity, but the overhead doesn't justify it for our use case. The minimum GPU requirement (RTX 3090+) doubles cloud costs. The proprietary license creates vendor lock-in. The Omniverse stack adds ~15GB of dependencies to every Docker image. And critically, our ML models (YOLO + DA V2) are trained on real-world data with baked lighting — the marginal quality gain from ray tracing over UE5 rasterization is minimal for model validation purposes.

### Cloud Architecture

Two Docker containers on a single AWS instance, sharing GPU via NVIDIA Container Toolkit:

```
Container 1: cosys-airsim-ue5          [--gpus all]
  ├── UE5 headless (rendering + physics)   ~8GB VRAM
  ├── PX4 SITL (MAVLink on tcp://cosys:14540)
  └── AirSim RPC server (port 41451)

Container 2: companion-stack            [--gpus all]
  ├── 7 processes (P1-P7)
  ├── ML models: YOLOv8m + DA V2 ViT-B    ~8GB VRAM
  ├── HAL backends: "cosys_airsim" via config
  └── Zenoh IPC (SHM within container)
```

**Why two containers, not one:** This mirrors production topology. On real hardware (Jetson Orin), the companion stack runs on the compute module while sensors are external. The two-container layout validates the same process boundaries, network communication patterns, and HAL abstraction layer that production uses. It also allows independent versioning — updating UE5/Cosys-AirSim doesn't require rebuilding the companion stack.

**GPU sharing:** Both containers run with `--gpus all`. CUDA time-shares the GPU automatically — standard practice for multi-container GPU workloads. No MPS or MIG configuration needed. Total VRAM: UE5 (~8GB) + YOLOv8m (~4GB) + DA V2 ViT-B (~4GB) + overhead (~0.5GB) = ~16.5GB on a 24GB A10G.

### HAL Integration

New HAL backends follow the existing pattern (ADR-006):

| Backend | Interface | Notes |
|---------|-----------|-------|
| `CosysCameraBackend` | `ICamera` | AirSim RPC `simGetImage()` → BGR frame |
| `CosysRadarBackend` | `IRadar` | AirSim RPC `getRadarData()` → radar returns |
| `CosysIMUBackend` | `IIMUSource` | AirSim RPC `getImuData()` → accel + gyro |
| `CosysDepthBackend` | `IDepthEstimator` | AirSim ground-truth depth (for validation baselines) |

Compile guard: `HAVE_COSYS_AIRSIM` (same pattern as `HAVE_GAZEBO`). Config overlay: `config/cosys_airsim.json`. Factory selection via `hal_factory.h` — zero changes to process code.

VIO: Reuse PX4 EKF2 pose via existing `MavlinkFCLink` — PX4 SITL already runs inside Cosys-AirSim and publishes odometry. No new VIO backend needed.

### ML Model Tier Strategy

Cloud GPU (24GB) enables larger, more accurate models than edge hardware:

| Target | YOLO | Depth | VRAM Budget | CMake Preset |
|--------|------|-------|-------------|--------------|
| Edge (T1000, 4GB) | YOLOv8n (3.2M) | DA V2 ViT-S (25M) | ~3GB | `edge` |
| Orin (8-16GB shared) | YOLOv8s (11.2M) | DA V2 ViT-S (25M) | ~4GB | `orin` |
| Cloud (A10G, 24GB) | YOLOv8m (25.9M) | DA V2 ViT-B (97M) | ~8GB | `cloud` |

Selection is config-driven at runtime (`detector.model_path`, `depth_estimator.model_path`) with CMake `MODEL_PRESET` setting build-time defaults for Docker images. Runtime config always overrides build-time defaults.

**Why this matters for validation:** Cloud-tier models (YOLOv8m: 50.2 mAP vs nano's 37.3) provide a higher-fidelity perception baseline. If the fusion pipeline works correctly with medium models on photorealistic data, we have high confidence it will work on real hardware with smaller models and real sensors (where radar compensates for depth model limitations).

### Cloud Infrastructure

**AWS g5.xlarge:** A10G 24GB, 4 vCPU, 16GB RAM. On-demand ~$1.01/hr, Spot ~$0.35/hr.

**Usage model: on-demand sessions, not 24/7.**
```
Local (T1000):  Code → build → unit tests → Gazebo scenarios (daily)
Cloud (A10G):   Pull → Cosys-AirSim perception tests (1-2hr sessions)
```

Estimated monthly cost: ~80 hrs x $0.35 = ~$28/month on spot instances.

---

## 3. Alternatives Considered

### A. Gazebo with improved rendering (Ogre2 PBR materials)

**Rejected.** Gazebo supports PBR materials in theory, but the rendering quality gap is too large for ML model validation. The fundamental issue isn't texture quality — it's scene complexity. Gazebo scenes have dozens of objects; real-world (and UE5) scenes have thousands of distinct surfaces, shadows, reflections, and lighting conditions that ML models depend on for feature extraction. Improving Gazebo's rendering would require rebuilding every scenario environment from scratch and still wouldn't reach UE5 fidelity.

### B. FlightForge (CTU Prague, UE5)

**Rejected (too early).** FlightForge uses UE5 and shows promise, but was released in late 2025 and has limited documentation, no Docker support, no radar sensor, and Python-only API. The project has fewer than 50 GitHub stars and no stable release. Revisit in 12-18 months if it matures.

### C. Isaac Sim (NVIDIA)

**Rejected (overkill).** Best rendering quality (RTX ray tracing) but requires RTX 3090+ (doubles cloud costs), proprietary license (vendor lock-in), full Omniverse stack (~15GB dependencies), and Python/ROS2 API (poor fit for our C++ HAL). The rendering quality advantage doesn't translate to meaningful ML validation improvement over UE5 rasterization — our models are trained on real-world data with baked lighting, not ray-traced scenes.

### D. Flightmare (University of Zurich, Unity)

**Rejected (abandoned).** Last significant commit in 2021. Python-only API. No PX4 integration. No radar. Unity rendering is a step up from Gazebo but below UE5. The project's abandonment means no bug fixes or compatibility updates.

### E. Real-world flight data only (no photorealistic sim)

**Rejected (unsafe iteration loop).** Without photorealistic simulation, the only way to validate ML perception is real flight tests. This creates a slow, expensive, and dangerous iteration cycle — each perception bug requires a physical flight to reproduce and validate the fix. Simulation enables rapid iteration: change code → rebuild → run scenario → check metrics, all from a terminal.

### F. Cloud-only without containerization

**Rejected (non-transferable).** Running the stack directly on the cloud VM works for testing but doesn't validate the containerized deployment that production hardware uses (JetPack 6 + NVIDIA Container Toolkit on Jetson Orin). The two-container architecture ensures that what we test in the cloud is structurally identical to what runs on hardware.

---

## 4. Consequences

### Positive

- **ML perception validation path.** For the first time, YOLO and depth estimation can be tested against photorealistic scenes with correct object classes, realistic textures, and physically-based lighting.
- **Native radar validation.** UKF fusion engine's radar-camera trust weighting can be tested against a real radar model instead of the `gpu_lidar` approximation.
- **Production topology validation.** Two-container cloud architecture mirrors the production hardware layout (companion compute + external sensors).
- **Hardware transferability.** Same codebase, same config system, same HAL — only the `MODEL_PRESET` flag and HAL backend config change between cloud and hardware.
- **Future-proofing.** Cloud-tier models (YOLOv8m + ViT-B) exercise the perception pipeline at a fidelity that next-gen edge hardware (Jetson Thor, 32GB+) will natively support.
- **Cost-effective.** Spot pricing (~$0.35/hr) keeps monthly costs under $30 for typical use.
- **Tier 1 and Tier 2 unchanged.** No disruption to existing CI or Gazebo workflows.

### Negative

- **Cloud dependency for Tier 3.** Cannot run perception validation locally (T1000 4GB VRAM insufficient). Developers must provision cloud infrastructure for photorealistic testing.
- **Cosys-AirSim maintenance risk.** Maintained by a single university lab (Cosys-Lab, University of Antwerp). If they stop maintaining it, we inherit the fork. Mitigated by: MIT license allows forking, C++ API is thin (~500 lines of client code), and our HAL abstraction means swapping the simulator only requires new HAL backends.
- **UE5 headless complexity.** UE5 headless rendering is less mature than Gazebo's headless mode. Docker image size is large (~15-20GB). Startup time is ~60-90 seconds vs Gazebo's ~5 seconds.
- **Three tiers to maintain.** Scenarios may need Tier 1, Tier 2, and Tier 3 variants. Config overlays (`default.json`, `gazebo_sitl.json`, `cosys_airsim.json`) must be kept in sync.
- **AWS account and billing management.** Spot instance interruptions require checkpointing and retry logic. Cost monitoring needed to prevent runaway spend.

### Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Cosys-AirSim abandonment | Low-Medium | High | MIT license, thin API layer, HAL abstraction enables swap |
| UE5 headless instability | Medium | Medium | Pin Docker image, add retry/restart logic, health checks |
| AWS spot interruption mid-test | Medium | Low | Checkpoint artifacts to S3 periodically, auto-retry |
| VRAM budget exceeded (UE5 + ML models) | Low | Medium | Monitor with `nvidia-smi`, fall back to ViT-S if tight |
| AirSim API breaking changes | Low | Medium | Pin version in Docker, adapter pattern in HAL backends |
| Zenoh SHM in Docker container | Low | Low | `--ipc=host` or `--shm-size=256m`; fallback to network mode |

---

## 5. Implementation

See Epic #431 for the phased implementation plan:

| Phase | Issue | Scope | Dependencies |
|-------|-------|-------|-------------|
| 1 | #432 — Docker containerization | Dockerfile, compose, CI-ready | None |
| 2a | #433 — CMake model presets | `MODEL_PRESET`, config overlays | #430 |
| 2b | #434 — Cosys-AirSim HAL backends | ICamera, IRadar, IIMU, IDepth | #430 |
| 3 | #435 — Cloud deployment | AWS, docker-compose, artifacts | #432, #433, #434 |
| 4 | #436 — CI perception workflow | GitHub Actions, advisory check | #435 (future) |

Phases 1 + 2 can run in parallel. Phase 3 requires both. Phase 4 is future/optional.

---

## 6. References

- Epic #431 — Cosys-AirSim Photorealistic Simulation Layer + Cloud Deployment
- Issue #430 — ML depth estimation (Depth Anything V2)
- Epic #437 — ML Training Pipeline (aerial data, fine-tuning)
- ADR-009 — Tier 1 / Tier 2 Simulation Architecture
- ADR-006 — HAL Hardware Abstraction Strategy
- [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) — University of Antwerp fork
- [VisDrone Dataset](https://github.com/VisDrone/VisDrone-Dataset) — 265K aerial images
- `config/default.json` — Tier 1 base config
- `config/gazebo_sitl.json` — Tier 2 base config
- `config/cosys_airsim.json` — Tier 3 base config (to be created)
- `common/hal/include/hal/hal_factory.h` — HAL backend factory

---

## 7. Amendment 2026-04-17: Flight Controller Split — Gazebo for PX4, AirSim for SimpleFlight

### Context

Section 2 ("HAL Integration") of this ADR originally stated:

> **VIO:** Reuse PX4 EKF2 pose via existing `MavlinkFCLink` — PX4 SITL already runs inside Cosys-AirSim and publishes odometry. No new VIO backend needed.

After exhaustive debugging on 2026-04-17 (session memory `project_session_2026_04_17_px4_hil_blocked.md`), **PX4 SITL HIL integration with Cosys-AirSim does not work** on the desktop setup. Six configurations were attempted:

| Config | Result |
|---|---|
| LockStep=false + minimal Sensors | `ekf2 missing data` — no HIL flow |
| Explicit Sensors (imu/baro/gps/mag) | Only gyros/mags arrive, no GPS |
| LockStep=true + SteppableClock | PX4 never receives sim connection (deadlock) |
| Canonical config + OriginGeopoint + NAV_RCL_ACT | Same `ekf2 missing data` |
| PX4 main branch | Same failure |
| PX4 v1.16.1 stable | Same failure |

Matches unresolved upstream bugs: [PX4 #24033](https://github.com/PX4/PX4-Autopilot/issues/24033), [AirSim #5018](https://github.com/microsoft/AirSim/issues/5018). The 2026-04-16 smoke test memory explicitly noted PX4 SITL integration was **"Not Yet Done"** — only the direct HAL↔AirSim RPC path was verified.

### Decision

**Split flight controller by simulation tier:**

| Tier | Simulator | Flight Controller | Purpose |
|---|---|---|---|
| **Tier 2** | Gazebo | PX4 SITL (MAVLink) | Flight-critical: FC interaction, fault recovery, mission logic, MAVLink compliance |
| **Tier 3** | Cosys-AirSim | SimpleFlight (AirSim RPC) | Perception-critical: ML, photorealistic scenes, visual fidelity, synthetic data |

This matches industry practice — perception teams use photorealistic sims (CARLA, AirSim, NVIDIA Isaac); flight control teams use physics sims (Gazebo, jMAVSim). Using one simulator for both roles is an anti-pattern.

### Rationale

1. **PX4+Cosys HIL is broken and upstream fixes are unavailable.** Both [PX4 #24033](https://github.com/PX4/PX4-Autopilot/issues/24033) and [AirSim #5018](https://github.com/microsoft/AirSim/issues/5018) are marked stale with no resolution. Waiting for upstream fixes blocks Epic #431 indefinitely.
2. **SimpleFlight is the AirSim-recommended default** (per Cosys-AirSim docs: *"Unless you have at least intermediate level of experience with PX4 stack, we recommend you use simple_flight"*).
3. **Gazebo+PX4 is already validated** — 20+ Tier 2 scenarios run reliably on this path. PX4-specific code (EKF, fault escalation, MAVLink parsing) is exercised there.
4. **No test coverage loss.** Perception code paths in AirSim don't need FC realism. FC code paths in Gazebo don't need photorealism. The matrix is complementary, not overlapping.
5. **Preserves hardware transfer path.** Gazebo+PX4 is the industry-standard smooth transition to real Pixhawk hardware. AirSim+SimpleFlight is dev/test only — not intended for production.
6. **Both HAL backends coexist.** `MavlinkFCLink` is unchanged. Add a new `CosysFCLink` that uses AirSim RPC (`takeoffAsync`, `moveToPositionAsync`, `hoverAsync`, `armAsync`, `landAsync`). Config-selectable per scenario.

### HAL Addition

New IFCLink backend:

| Backend | Interface | Used by | Transport |
|---------|-----------|---------|-----------|
| `MavlinkFCLink` | `IFCLink` | Tier 2 (Gazebo+PX4) | MAVLink over UDP (14540/14580) |
| `CosysFCLink` (new) | `IFCLink` | Tier 3 (AirSim+SimpleFlight) | AirSim RPC (port 41451) |

Compile guard: `HAVE_COSYS_AIRSIM`. Config: `comms.mavlink.backend: "cosys_rpc"` (or new key). Factory selection in `hal_factory.h`.

Scenario config selects which backend to use:
- Tier 2 (Gazebo): `"backend": "mavlink"` (existing)
- Tier 3 (AirSim): `"backend": "cosys_rpc"` (new)

### Consequences (Amendment)

**Positive:**
- Unblocks Epic #431 Tier 3 scenarios immediately — no more PX4 HIL debugging
- Clean separation of concerns: flight control in Gazebo, perception in AirSim
- No new dependencies, no upstream bug waits
- SimpleFlight is zero-config — reduces onboarding friction for perception work

**Negative:**
- Two FC code paths to maintain (`MavlinkFCLink` + `CosysFCLink`)
- AirSim scenarios can't validate MAVLink-specific behaviour — must be tested in Gazebo
- SimpleFlight has no fault injection realism (motor failures, GPS loss, EKF divergence) — all fault tests stay in Gazebo
- Developers switching between tiers must know which backend applies
- Future PX4+Cosys HIL fix upstream would require revisiting this split

**Risks:**

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Divergent flight behaviour between tiers | Medium | Medium | Document explicit: "takeoff tested in Gazebo, perception tested in AirSim" — not one-vs-other |
| Fault scenarios untested against PX4 | Low | High | All fault injection stays in Tier 2 (Gazebo) where it's proven to work |
| `CosysFCLink` API drift | Low | Low | AirSim RPC is thin and stable; pin version in submodule |

### Implementation

See Issue #TBD (created as follow-up to this amendment) — `CosysFCLink` HAL backend for AirSim SimpleFlight.

### When to Revisit

- PX4 releases a stable SITL integration with Cosys-AirSim that's verified to work
- Or: we switch Tier 3 to a simulator with working PX4 HIL (e.g., Isaac Sim via Pegasus wrapper)
- Or: we add HITL tier (real Pixhawk) — in which case Tier 3 AirSim continues to use SimpleFlight for perception work only

---

## 8. Amendment 2026-04-17 (Part 2): Comprehensive Simulator Landscape Analysis

After the PX4+Cosys failure, we conducted a broader "tool evaluation phase" analysis — since we'll commit to this stack for weeks or months, we wanted to be rigorous about alternatives before locking in.

### Candidates Evaluated

| ID | Name | Verdict |
|---|---|---|
| A | Simulated HAL | Keep for T1 (CI) |
| B | Gazebo + PX4 SITL | **Keep for T2** |
| C | Cosys-AirSim + SimpleFlight | **Adopt for T3a** |
| D | Pegasus Simulator + Isaac Sim + PX4 | Park (future cloud T3b) |
| E | Webots + PX4 | Rejected — no unique value |
| F | jMAVSim | Optional CI add-on |
| — | FlightForge (CTU-MRS) | Rejected — not PX4-compatible (uses MRS protocol) |
| — | ArduPilot SITL + AirSim | Rejected — GPL v3 license concerns for commercial path |
| — | FPV/consumer sims (VelociDrone, Liftoff, TRYP, Zephyr, DJI, etc.) | Rejected — wrong category (pilot training, not dev) |
| — | Enterprise sims (rFpro, Ansys, dSpace, Cognata, Applied Intuition) | Rejected — automotive-first, expensive, no drone fit |
| — | CARLA | Rejected — autonomous driving domain, not drones |
| — | Flightmare (UZH) | Rejected — mostly inactive since 2022 |
| — | MuJoCo / Drake | Rejected — physics-only, no rendering or FC |

### Scoring Across 10 Dimensions

Each candidate was evaluated on: cost/time, exit cost, coverage matrix, hardware transfer path, onboarding/teaming, compliance, obsolescence risk, performance, data generation, multi-vehicle support.

### Key Surfaces From the Analysis

#### 1. The tier split is architecturally sound, not just pragmatic

Coverage matrix analysis shows Gazebo and Cosys-AirSim are genuinely complementary:

| Test category | Gazebo+PX4 | Cosys+SimpleFlight |
|---|---|---|
| FC arm/disarm, MAVLink compliance | ✓ | Fake (SimpleFlight is not MAVLink) |
| EKF convergence, fault injection, RC/link loss | ✓ | ✗ |
| Mission waypoint execution | ✓ | via RPC |
| YOLO object detection on photorealistic scenes | ✗ | ✓ |
| Depth Anything V2 depth estimation | ✗ | ✓ |
| Synthetic data generation for ML training | ✗ | ✓ |
| Weather/lighting variation | Limited | ✓ |
| Native radar sensor | LiDAR proxy | LiDAR proxy (base SDK has no radar) |

**No overlap is wasted.** Each tier validates what the other cannot. Consolidation into a single sim would require Isaac Sim + Pegasus (Dimension 4 below).

#### 2. Cosys-AirSim is our highest obsolescence risk

Single-lab maintained (University of Antwerp, Cosys-Lab), already a fork of the abandoned Microsoft AirSim mainline, niche community. **Mitigation is structural:** our HAL abstraction (ADR-006) means swapping to Isaac/Webots takes ~2-3 weeks of backend work, not a full rewrite. But it is the piece most likely to force a migration in 2-3 years. Accepting the risk because (a) alternatives have different problems, (b) HAL abstraction limits migration cost.

#### 3. Hardware transfer path dead-ends at SimpleFlight

SimpleFlight runs only inside AirSim — it does not exist on Pixhawk hardware. **Anything we verify in SimpleFlight must be re-verified on PX4 before hardware deployment.** This is acceptable for perception code (same camera, same depth model on hardware) but would be wrong for FC logic. This is the core reason the tier split is correct: SimpleFlight must be scoped to perception only.

| Simulator | SITL→HITL→Real drone transferability |
|---|---|
| B (Gazebo+PX4) | Clean — swap `SIMULATOR` env var → Pixhawk serial |
| C (Cosys+SimpleFlight) | **Path breaks** — SimpleFlight never runs on hardware |
| D (Isaac+PX4 via Pegasus) | Clean — same as Gazebo |

#### 4. Isaac Sim via Pegasus is the only "consolidated" option

Of all candidates, only **Pegasus Simulator + Isaac Sim + PX4** provides PX4 SITL + photorealistic rendering + full fault injection in a single simulator. But it is hardware-gated:

- Requires NVIDIA Ampere (RTX 30-series) or newer — our GTX 1080 Ti cannot run it
- Cloud A10G instance at ~$0.50-1.00/hr on spot pricing
- NVIDIA Omniverse account and familiarity curve
- MIT-licensed wrapper (Pegasus) over NVIDIA-licensed Isaac Sim

**Parked as future Tier 3b.** Revisit when we have consistent cloud budget (~$300+/mo) OR an upgraded dev machine (RTX 3080+) OR decide single-sim consolidation is worth the cost.

#### 5. Compliance does NOT demand high simulator fidelity

We are not in a regulated domain where simulator output is cert evidence. Real-world flight testing is always required regardless. **This lowers the bar for sim fidelity** — "good enough to build confidence" beats "perfect but fragile." Reinforces: don't over-invest in simulator quality.

#### 6. Exit cost analysis validates Gazebo as durable

~2 months of scenario configs, world files, and mission tests accumulated in Gazebo. Migrating away would be 4-6 weeks. **Gazebo is our most durable tool commitment.** Cosys-AirSim is by-design more replaceable — the HAL keeps it loosely coupled.

#### 7. Performance gaps validate CI strategy

Cosys and Isaac take 60-120 seconds just to start UE5. They are **interactive-only**; per-PR CI must remain Gazebo-based. **This validates our existing CI design** — Cosys perception runs as "advisory" workflow (#436), not blocking.

#### 8. Real drone hardware may be better than cloud Isaac Sim

If Cosys-AirSim Tier 3 proves the perception stack, a programmable drone (~$500-2000 one-time) provides:
- Real-world validation that no simulator can match (lighting variance, real camera noise, real GPS multipath)
- Cheaper than 12 months of cloud Isaac Sim ($6k+/yr spot)
- Forces us to solve real-hardware problems earlier rather than discovering them post-simulation

**Implication:** Isaac Sim cloud tier may be skipped entirely in favor of Cosys-AirSim (perception) → real hardware (FC/flight), bypassing the "consolidated sim" step. This is aligned with PX4's own recommended SITL→HITL→real progression.

### Rejected Alternatives — Brief Rationale

**FlightForge (CTU-MRS):** Uses proprietary MRS UAV protocol, not PX4 MAVLink. Integration would require building a custom HIL bridge — weeks of engineering with no guarantee. Despite BSD-3 license and UE5 rendering, not compatible with our PX4-centric stack.

**ArduPilot + AirSim:** Documentation suggests ArduPilot HIL works better with AirSim than PX4 HIL does. However, **GPL v3** license creates commercial friction: linking ArduPilot code into any distributed product propagates GPL. MAVLink-only communication (no linkage) is permissible but requires careful build isolation. Priority on BSD-clean commercial future argues against adoption.

**Webots + PX4:** Working combination with reasonable rendering and native radar. But middle ground solves no specific pain — rendering is not photorealistic enough to validate ML perception, yet adds a third simulator to maintain. Not worth the cost.

**FPV/consumer sims (VelociDrone, Liftoff, Uncrashed, TRYP FPV, DRL, DJI, Zephyr, Tiny Whoop):** Different category entirely — pilot training for human FPV/racing. No SDK, no PX4, no MAVLink, no programmatic API. Irrelevant to autonomous stack development.

**Enterprise sims (rFpro, Ansys AVxcelerate, dSPACE, Cognata, Applied Intuition, Siemens Simcenter):** Expensive ($$$$+), automotive-primary, closed source, no drone-first fit. Only relevant if we pivot to automotive-adjacent work or take major enterprise funding.

### Scope: One-Command Simulator Switching

The architectural target (modular enough to swap sims with minimal code change):

```bash
# Target developer experience:
bash tests/run_scenario.sh --scenario 30 --sim gazebo   # Tier 2: PX4+Gazebo
bash tests/run_scenario.sh --scenario 30 --sim airsim   # Tier 3a: SimpleFlight+Cosys
bash tests/run_scenario.sh --scenario 30 --sim isaac    # Tier 3b: PX4+Pegasus (future)
```

Already ~70% in place:
- ✓ HAL abstraction enables backend swap via config
- ✓ Separate config overlays per tier (`gazebo_sitl.json`, `cosys_airsim_dev.json`, `cosys_airsim.json`)
- ✓ Backend factory in `hal_factory.h`
- ✓ Per-sim runner scripts (`run_scenario_gazebo.sh`, `run_scenario_cosys.sh`)

Remaining work:
- `CosysFCLink` HAL backend (issue #490)
- Unified runner `run_scenario.sh` with `--sim` flag
- Future: Isaac Sim backends + `isaac_sim.json` overlay when Tier 3b is pursued

### Final Decision

**T1: Simulated HAL** — unchanged, CI/unit tests
**T2: Gazebo + PX4 SITL** — FC, nav, fault, mission (80% of test matrix)
**T3a: Cosys-AirSim + SimpleFlight** — perception, ML, synthetic data generation (this session's #490 work)
**T3b: Pegasus + Isaac Sim + PX4** — PARKED. Revisit if/when (a) cloud budget available, (b) RTX 30+ hardware, (c) real drone hardware proves insufficient for FC validation.

**Probable real path forward:** Cosys-AirSim for perception → real programmable drone (Holybro X500, Modal AI Voxl2, etc.) for FC/flight validation. This **skips the Isaac Sim tier entirely** — cheaper, more realistic, forces early hardware-level problem discovery.
