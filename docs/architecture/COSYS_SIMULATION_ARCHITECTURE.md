# Cosys-AirSim (Tier 3) Simulation Architecture

> **Scope:** runtime architecture of the **Tier 3** photorealistic simulation layer (Cosys-AirSim). Sibling document to [`SIMULATION_ARCHITECTURE.md`](SIMULATION_ARCHITECTURE.md), which covers Tier 1 (pure simulated HAL backends) and Tier 2 (Gazebo SITL).
>
> **For the why:** see [ADR-011 — Cosys-AirSim Photorealistic Simulation](../adr/ADR-011-cosys-airsim-photorealistic-simulation.md).
>
> **For the how-to-set-up:** see [`docs/how-to/COSYS_SETUP.md`](../how-to/COSYS_SETUP.md).

---

## Table of Contents

1. [Why a third tier](#1-why-a-third-tier)
2. [Runtime topology](#2-runtime-topology)
3. [HAL backends](#3-hal-backends)
4. [Scenarios](#4-scenarios)
5. [Differences from Gazebo (Tier 2)](#5-differences-from-gazebo-tier-2)
6. [Build gating and feature detection](#6-build-gating-and-feature-detection)
7. [Known limitations and gaps](#7-known-limitations-and-gaps)
8. [Related](#8-related)

---

## 1. Why a third tier

Tier 1 (pure simulated) gives instant feedback but no photorealism — the camera frames are synthetic gradients, not images a real ML model would ever see. Tier 2 (Gazebo) adds physics and renders scenes that are good enough for control-loop validation but fall apart the moment perception needs real textures, lighting, occlusion, or motion blur.

Tier 3 (Cosys-AirSim, an Unreal-Engine-5 based fork of Microsoft AirSim maintained by the University of Antwerp's Cosys-Lab) covers that gap. The drone code runs unmodified — only the HAL backend layer changes. Cameras render through the UE5 pipeline, IMU samples come from the UE5 physics tick, and the flight-controller bridge talks to SimpleFlight (Cosys's bundled FC) over Cosys's RPC API.

ADR-011 documents the trade-off in full. The headline benefit is being able to test the ML perception stack against rendered scenes that share most properties with real-world camera input, without needing physical hardware.

## 2. Runtime topology

```
┌────────────────────────────────────────────────────────────────────────┐
│                       Cosys-AirSim runtime (UE5)                       │
│                                                                        │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │ Unreal Engine 5 — scene rendering, physics, mesh segmentation   │  │
│   │ (textures, lighting, IMU integration, ground-truth semantics)   │  │
│   └────────────────────────┬────────────────────────────────────────┘  │
│                            │                                            │
│                            ▼ rpclib (msgpack/TCP, default :41451)      │
│   ┌─────────────────────────────────────────────────────────────────┐  │
│   │ AirSim plug-in — RPC server: `simGetImages`, `getImuData`,      │  │
│   │ `simGetSegmentationImage`, `getLidarData`, FC bridge, etc.      │  │
│   └────────────────────────┬────────────────────────────────────────┘  │
└────────────────────────────┼────────────────────────────────────────────┘
                             │
                             │  msgpack-RPC
                             │
┌────────────────────────────▼────────────────────────────────────────────┐
│                Companion software stack (this repo)                     │
│                                                                         │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  HAL Cosys backends (HAVE_COSYS_AIRSIM):                        │   │
│   │  - CosysCameraBackend / CosysIMU / CosysDepthBackend            │   │
│   │  - CosysGroundtruthRadar (LiDAR-as-radar adapter)               │   │
│   │  - CosysFCLink (drives SimpleFlight, not PX4)                   │   │
│   │  - CosysSegmentationBackend (sim-only GT mask producer)         │   │
│   │  - CosysEchoBackend (mmWave echo, optional)                     │   │
│   └────────────────────────┬────────────────────────────────────────┘   │
│                            │                                             │
│                       ┌────▼────┐                                        │
│                       │  HAL    │  same interfaces as Tier 1 / 2         │
│                       │ factory │                                        │
│                       └────┬────┘                                        │
│                            │                                             │
│           ┌────────────────┼───────────────┐                             │
│           ▼                ▼               ▼                             │
│      P1 video         P3 SLAM/VIO     P5 comms ↔ SimpleFlight (Cosys)    │
│      capture          P2 perception   ...                                │
│           │                │               │                             │
│           └────────────────┴───────────────┘                             │
│                            │                                             │
│                    Zenoh IPC (unchanged)                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

Key property: the **only** layer that changes between Gazebo and Cosys is the HAL backend selection. The 7-process Zenoh topology, the planner, the watchdog, and the scenario runner all behave identically. This is the whole point of the HAL.

## 3. HAL backends

All Cosys-specific headers live in `common/hal/include/hal/`. Each is wired into the HAL factories in `hal_factory.h` behind `#ifdef HAVE_COSYS_AIRSIM` (see §6).

| Header | Interface | Factory key | Role |
|---|---|---|---|
| `cosys_camera.h` *(in `icamera.h` factory)* | `ICamera` | `"cosys_airsim"` | Pulls scene-rendered RGB images via `simGetImages(ImageType::Scene)`. Same `CapturedFrame` contract as `SimulatedCamera` / `GazeboCamera`. |
| `cosys_imu.h` | `IIMUSource` | `"cosys_airsim"` | Reads `getImuData(vehicle)` — UE5-integrated angular velocity + linear acceleration. |
| `cosys_depth.h` | `IDepthEstimator` | `"cosys_airsim"` | `simGetImages(ImageType::DepthPlanar)` passthrough — ground-truth depth, no ML inference. Use for end-to-end PATH A validation without DA-V2 noise. |
| `cosys_groundtruth_radar.h` | `IRadar` | `"cosys_airsim"` | Adapter that converts Cosys LiDAR returns into the `RadarDetectionList` wire type. Lets us exercise the radar fusion path against scene geometry. |
| `cosys_echo_backend.h` | `IRadar` | (alternative) | mmWave-style echo backend when the AirSim Echo sensor plugin is present. |
| `cosys_fc_link.h` | `IFCLink` | `"cosys_airsim"` | Drives **SimpleFlight** (Cosys's bundled FC), not PX4. Maps `FCCommand` → SimpleFlight RPC calls (`armDisarm`, `takeoffAsync`, `moveByVelocityAsync`, etc.). |
| `cosys_segmentation_backend.h` | `IInferenceBackend` | `"cosys_airsim"` | Sim-only ground-truth instance segmentation via `simGetSegmentationImage` + scene-object enumeration. Pair with the depth backend above for ground-truth PATH A (no ML, no SAM mega-mask collapse). |
| `cosys_name_resolver.h` | (helper) | — | Maps friendly scene-object names to Cosys segmentation indices. Used by the segmentation backend to attach class labels to GT masks. |

**Why SimpleFlight instead of PX4 in Tier 3.** PX4 SITL adds another process layer and is sensitive to the EKF2 cold-start variance we already wrestle with in Tier 2. SimpleFlight is good enough for perception-focused testing where the FC's job is "follow the velocity command" — closing one loop fewer makes scenario sweeps more deterministic. See [`docs/how-to/COSYS_SETUP.md`](../how-to/COSYS_SETUP.md) for the rationale.

## 4. Scenarios

Cosys-specific scenario configs live alongside the Gazebo ones in `config/scenarios/` (single source of truth — see `CLAUDE.md` § Single Sources of Truth) and are executed by `tests/run_scenario_cosys.sh`.

| Scenario | Purpose |
|---|---|
| `29_cosys_perception.json` | Mixed dynamic — scene objects move during flight; exercises tracker + fusion under photorealistic input |
| `30_cosys_static.json` | Static proving ground — fixed obstacles, predictable layout, used as the baseline for "did anything regress" checks |
| `33_non_coco_obstacles.json` | PATH A end-to-end with non-COCO obstacle classes (cubes, panels) — validates the SAM → MaskClassAssigner → MaskDepthProjector → `/semantic_voxels` flow against ground-truth perception backends |

Cosys scenarios are gated separately from Gazebo scenarios in CI because the Tier 3 runtime is heavier (UE5 needs a GPU). The scenario runner script handles Cosys vs Gazebo selection via the config's `simulator` field.

## 5. Differences from Gazebo (Tier 2)

| Dimension | Tier 2 (Gazebo) | Tier 3 (Cosys-AirSim) |
|---|---|---|
| Renderer | gz-sim (OGRE) | Unreal Engine 5 (Lumen, Nanite) |
| Photorealism | Functional / low | High (real-world-comparable) |
| Flight controller | PX4 SITL via MAVLink | SimpleFlight via Cosys RPC |
| IPC bridge | gz-transport13 | rpclib (msgpack TCP) |
| GPU requirement | Optional (CPU fallback) | Strongly recommended (UE5 needs it for usable frame rates) |
| Ground-truth perception | gz-sim segmentation cameras (limited) | `simGetSegmentationImage` + scene-object enumeration (pixel-perfect) |
| Determinism | Moderate (EKF2 cold-start variance) | Higher within a single run; UE5 frame timing still varies |
| Iteration speed | Fast scenario reset (gz-sim service call) | Slower — UE5 level reload, scene-object respawn |
| What it's best at | Control-loop validation, fault injection, FSM coverage | Perception validation, ML-in-the-loop, GT comparison |

In practice: most CI scenarios run on Tier 2 because it's faster and free of GPU dependencies. Tier 3 runs are deliberately reserved for perception work — the moment a scenario needs scene textures or pixel-perfect GT masks, it migrates to Cosys.

## 6. Build gating and feature detection

Cosys integration is **optional** — the canonical build is Tier 1/2-only. Cosys backends are gated behind a CMake feature flag.

- `find_package(CosysAirSim QUIET)` in the root `CMakeLists.txt` detects the locally-built Cosys SDK
- When found, the build defines `HAVE_COSYS_AIRSIM` and links the Cosys-specific HAL `.cpp` files
- Every Cosys-specific header is wrapped in `#ifdef HAVE_COSYS_AIRSIM`, including the factory entries in `hal_factory.h`
- CI's primary build does **not** require Cosys — `HAVE_COSYS_AIRSIM` is off in `ci.yml` and the Tier 3 scenarios are run separately on a GPU-equipped runner (or skipped)

Result: anyone can clone, build, and run the stack against Gazebo or pure-simulated backends without ever touching Cosys. The Tier 3 capability is purely additive.

## 7. Known limitations and gaps

- **Ground-truth emitter design doc** — the per-frame segmentation-mask GT producer pairs with the perception pipeline and is documented in the (currently gitignored) `docs/design/perception_v2_detailed_design.md` § "Ground-truth emitter". That section will become reachable from here once the perception v2 design doc lands on the branch.
- **Echo (mmWave) backend** is functional but not yet exercised by a scenario sweep.
- **UE5 scene management** is currently manual — there is no scripted scene reset between scenarios, which is why Tier 3 runs do not match Tier 2 for iteration speed.
- **Real hardware fallback** is not symmetric with Tier 2 — there is no "production Cosys backend" since Tier 3 is by definition a simulator. The real-hardware equivalent of a Cosys camera is a V4L2 camera plus a real lens (Tier 0, not part of any simulation tier).

## 8. Related

- [ADR-011 — Cosys-AirSim Photorealistic Simulation](../adr/ADR-011-cosys-airsim-photorealistic-simulation.md) — decision rationale
- [`docs/how-to/COSYS_SETUP.md`](../how-to/COSYS_SETUP.md) — local setup
- [`SIMULATION_ARCHITECTURE.md`](SIMULATION_ARCHITECTURE.md) — Tier 1 / Tier 2 sibling
- [`docs/design/hal_design.md`](../design/hal_design.md) — HAL interface reference
- [ADR-009 — Tier 1 / Tier 2 Simulation Architecture](../adr/ADR-009-tier1-tier2-simulation-architecture.md) — original two-tier model that this doc extends
- [ADR-013 — Stereo + Radar Defence-in-Depth vs Unified Fusion](../adr/ADR-013-stereo-radar-redundancy-vs-fusion.md) — context for why we run radar-from-LiDAR adapters even in Tier 3
