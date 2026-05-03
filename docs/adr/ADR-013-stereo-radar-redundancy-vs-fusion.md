# ADR-013: Stereo + Radar — Defence-in-Depth Redundancy vs. Unified UKF Fusion

| Field           | Value                                                                                                   |
|-----------------|---------------------------------------------------------------------------------------------------------|
| **Status**      | Accepted                                                                                                |
| **Date**        | 2026-05-03                                                                                              |
| **Author**      | Naveen Mohanan + Claude Opus 4.7                                                                        |
| **Deciders**    | Project leads                                                                                           |
| **Supersedes**  | —                                                                                                       |
| **Related**     | Issue #698 (PATH A grid promotion has no radar veto), PR #699 (DA V2 saturation band — Fix #2 of #698), Issue #229 (radar fusion corrupting UKF tracks), `project_epic237_architecture` (sensor fusion architecture), `project_concurrency_rationale` (when to use which primitive), [ADR-009](ADR-009-tier1-tier2-simulation-architecture.md), [ADR-011](ADR-011-cosys-airsim-photorealistic-simulation.md), `docs/design/perception_design.md` |

---

## 1. Context

### The decision in front of us

Scenario 33 (`2026-05-02_161132_ABORTED`) failed in a way that surfaced a long-standing architectural question. The PATH A perception pipeline (camera → SAM → DA V2 depth → MaskDepthProjector → voxels) cemented 4596 ghost static cells into the planner's occupancy grid, walled off WP2, and froze the drone ~12 m short. The radar pipeline running in parallel saw the *real* obstacles correctly at sensible positions. But the radar was **never consulted** before PATH A's voxels were promoted to permanent static — `OccupancyGrid3D::insert_voxels()` has no `require_radar_for_promotion` gate, only the detector path does.

The quick fix (PR #699) caps DA V2 saturation samples at the projector boundary so they cannot reach the grid. That stops the immediate failure mode but leaves the deeper question open:

> Should PATH A and radar be **two architecturally independent paths** into the occupancy grid (defence-in-depth redundancy), or should they be **fused upstream into a single UKF** that emits one cross-checked truth?

We considered three options: (a) keep two paths, harden the boundary; (b) feed PATH A clusters into the UKF as synthetic detections so radar veto becomes a free property of the existing fusion math; (c) replace radar entirely with a denser 4D imaging unit and trust a single fused channel.

This ADR records why we chose (a), and why (b) — though architecturally clean on paper — is the wrong answer for the hardware we can actually fly.

### Why now

We are about to migrate the depth source from monocular DA V2 to a passive stereo camera. Before we refactor the perception graph, we need a binding decision on whether stereo and radar collapse into one fused state or stay as independent contributors. A wrong call here either:

1. invests effort in a unified UKF that the literature and our own incident history say will degrade map quality (the bad bearings of a low-cost mmWave unit corrupting a well-localised stereo track), or
2. invests effort in a parallel architecture that may need to be undone when 4D imaging radar becomes light enough to fly.

Either way, the decision is upstream of the stereo HAL backend, the perception thread topology, and the planner's grid-write API. Deferring it bakes in cost.

---

## 2. Decision

**We keep stereo and radar as two independent paths into the occupancy grid.** Each path can promote cells to static under its own gating logic. The track-level UKF continues to fuse camera bearing + radar range + Doppler **for moving objects only**, where Doppler genuinely adds state. Static obstacles do not pass through the UKF — they go directly to the grid via the channel that observed them, and a *late-stage cross-veto* at the grid boundary handles long-range disagreements (radar's strength) and short-range disagreements (stereo's strength).

Operationally:

1. **Stereo path** owns dense local geometry under ~10 m. Replaces the current PATH A camera+SAM+DA V2 chain. Promotes via the existing instance gate (cluster + tracker + N-frame observation count). Saturation-band filter (PR #699) stays in place as a universal "depth source said max → reject" guard.
2. **Radar path** owns range >10 m and Doppler. Promotes its own grid cells when range is confident *and* the detection has Doppler-consistent re-observation across N frames.
3. **UKF as track-level cross-checker for moving objects only.** Doppler-positive tracks get camera bearing + radar range fused. Stationary detections do not enter the UKF — the camera is more accurate at bearing than the radar we can fly, so fusing on stationary objects strictly degrades the estimate.
4. **Late-stage cross-veto at the grid boundary.** When PATH A wants to promote a cell at >10 m and radar reports nothing in that bearing/range cone, the cell stays dynamic (TTL-decay) instead of being cemented. Symmetric: when radar wants to promote a cell at <2 m where it's in the near-field blind zone and stereo says clear, the radar promotion is suppressed. Conflicts are *logged*, not silently resolved — operators see disagreements and can tune.

**We explicitly close the door on the unified-UKF refactor for the radar hardware we can fly today.** The decision can be revisited if and when the airframe can carry a 4D imaging radar (~500 g, ~18 W, sub-1° azimuth — see §3 quantitative comparison) where radar bearings become peer-quality to camera bearings.

---

## 3. Quantitative basis

The decision rests on hard numbers, not preference. Source citations are in §7.

### Range accuracy as a function of distance

| Range  | ZED 2i depth σ (B = 120 mm, σ_d = 0.1 px)         | TI AWR1843-class FMCW radar (B = 4 GHz)        |
|-------:|---------------------------------------------------|------------------------------------------------|
| 2 m    | ~0.7 cm                                           | ~3 cm                                          |
| 5 m    | ~4.6 cm                                           | ~3 cm                                          |
| 10 m   | ~19 cm                                            | ~3 cm                                          |
| 20 m   | ~74 cm                                            | ~3 cm                                          |
| 40 m   | ~3 m (effectively unusable past product max ~30 m)| ~3 cm                                          |

Stereo depth uncertainty grows as Z² (analytical: `σ_Z = Z²·σ_d / (f·B)`). Radar range resolution depends only on chirp bandwidth (`Δr = c / (2B)`) and stays approximately constant across range until SNR collapses. **Crossover is around 5–8 m for the 12 cm baseline we expect to fly.** Past 10 m, radar wins range by an order of magnitude. Under 3 m, stereo wins range by an order of magnitude.

### Cross-range (angular) accuracy

| Range  | Stereo (1 px @ 0.057°/px on ZED 2i HD1080) | TI AWR1843 (1° boresight) | 4D imaging radar (Continental ARS540, 0.5°) |
|-------:|--------------------------------------------:|--------------------------:|-------------------------------------------:|
| 2 m    | 2 mm                                       | 3.5 cm                    | 1.7 cm                                     |
| 5 m    | 5 mm                                       | 8.7 cm                    | 4.4 cm                                     |
| 10 m   | 1 cm                                       | 17 cm                     | 9 cm                                       |
| 20 m   | 2 cm                                       | 35 cm                     | 17 cm                                      |
| 40 m   | 4 cm                                       | 70 cm                     | 35 cm                                      |

**Stereo wins angular accuracy by 1–2 orders of magnitude over flyable radar.** This is the structural reason fusing radar bearing with camera bearing in a single UKF is the wrong move on this hardware: the radar bearing is the higher-variance measurement and dragging the fused estimate toward it strictly degrades the track. The 4D imaging radar column is the comparison point that would change the analysis — but it's a 575 g / 18 W unit, currently outside any sUAS payload budget.

### Statistically independent failure modes

| Hazard                                          | Stereo               | mmWave radar         |
|-------------------------------------------------|----------------------|----------------------|
| Fog / dust / rain                               | Fails at <50 m vis   | Pass-through         |
| Featureless white walls / cube faces            | Fails (no disparity) | Strong specular hit  |
| Wire mesh / thin foliage / low-RCS small drone  | Sees fine            | Often invisible      |
| Glare / low light                               | Fails                | Pass-through         |
| Indoor multi-path                               | Pass-through         | Ghost targets        |
| Near-field (<0.5 m for AWR1843, <2 m for some)  | Pass-through         | Blind zone           |
| Calibration drift (vibrating airframe)          | Slow degrade         | Insensitive          |
| Dense mid-range obstacles (5–30 m)              | OK                   | OK (range), poor (bearing) |

Failure-mode independence is the textbook precondition for **defence-in-depth redundancy** (Mobileye True Redundancy thesis). Tight upstream fusion blinds *both* channels when the fusion logic itself fails — a non-recoverable failure mode for safety-critical control loops. Independent paths can be validated independently with much lower test budget; aerospace certification standards (ARP-4754A, DO-178C) explicitly favour architectural diversity over algorithmic complexity.

### What production sUAS actually do

- **Skydio X10** — vision-only, six 32 MP nav cameras; deep-learning depth (no radar fusion).
- **DJI Mavic 3 / Inspire 3** — multiple stereo pairs, vision-only obstacle avoidance.
- **Parrot ANAFI Ai** — stereo + monocular depth-from-motion both feed the *same voxel grid independently* (parallel paths, late reconciliation).
- **Mobileye Drive (L4 robotaxi)** — two parallel perception stacks (camera-only and radar+lidar), reconciled at scene level not signal level. "True Redundancy" is the public marketing name for this architecture.
- **Waymo 6th-gen Driver** — 13 cameras + 4 lidars + 6 radars, "complementary, overlapping views", explicit modality redundancy when one sensor is degraded.

**No production sUAS publicly fuses radar and stereo at the UKF level.** Radar+camera UKF papers exist in research (Tang et al. 2024 fuse-before-track on sUAS, CEUR 2024 EKF tracker, Springer 2024 lightweight CNN) and show detection-rate gains on benchmarks — but they have not made it into deployed safety-critical systems.

### Our own incident history

`project_issue229_radar_fusion` (2026-03-29): "radar still corrupts UKF tracks" after the HAL ground filter was added. The recurring pattern: when radar bearings (14° boresight on AWR1843-class) are fused into a UKF that already has tight camera bearings, the fused estimate moves toward the radar measurement and *degrades* localisation accuracy. The eventual fix in that issue was a tighter spatial association window and more conservative covariance on radar updates — i.e. *less* fusion, not more. This is consistent with the literature's caution about cheap mmWave + UKF fusion.

---

## 4. Consequences

### Positive

1. **Defence-in-depth.** A bug in either path cannot blind the other. The grid still receives data even if PATH A or radar individually fails, because they write independently.
2. **Hardware-honest.** Treats radar as good for range and Doppler (where it wins) and stereo as good for bearing and dense local geometry (where it wins). Avoids the "fuse the worse measurement into the better one" anti-pattern documented in #229.
3. **Validation tractability.** Each path can be unit-tested, scenario-tested, and certified independently. No combinatorial explosion of fusion failure modes.
4. **Aligned with the field.** Mirrors what every certifiable autonomy stack (Mobileye, Waymo) and every flying production sUAS (Skydio, DJI, Parrot) actually does.
5. **Stereo migration unblocked.** The decision does not require any UKF refactor before the stereo HAL backend lands. Stereo replaces DA V2 in PATH A; the existing radar path continues as-is.
6. **Reversible.** If the airframe gains a 4D imaging radar later (Continental ARS-class), the analysis flips — radar bearings become peer-quality to camera, and the unified-UKF refactor becomes defensible. This ADR can be superseded then.

### Negative

1. **Two sources of truth in the grid.** Disagreements between paths must be reconciled at the grid boundary. We need explicit logic for "PATH A says obstacle here, radar says clear at >10 m" and the symmetric near-field case. This is more code than a single UKF channel and must be tested for both directions of disagreement.
2. **Doppler under-utilised on static objects.** Radar's Doppler signal is only consumed for moving-object UKF tracks. We give up the theoretical benefit of using zero-Doppler as confirming evidence on static obstacles. Acceptable: static-obstacle promotion already has multiple gates (instance count, observation persistence, TTL decay) and Doppler is one signal among several rather than load-bearing.
3. **Two thread topologies to maintain.** PATH A clusterer + tracker run independently of the UKF. We carry the operational cost of two state machines that do similar work (per-instance ID, dormant pool, re-ID logic). Documented and accepted; the literature shows the alternative is worse.
4. **Cross-veto logic is new code.** §2 item 4 introduces a new component in the grid-write path. Must be implemented carefully so that a fault in the cross-veto cannot itself blind both channels (i.e. a default-open policy: if the cross-veto fails to evaluate, write the cell rather than silently dropping it).

### Neutral / out of scope

1. **Stereo backend choice (ZED 2i, RealSense D455, OAK-D Pro, custom)** is decided separately. ADR-013 only locks in the architectural shape, not the specific sensor.
2. **Whether to also support a 4D imaging radar in the future** is left open. The HAL `IRadarBackend` interface stays generic enough to accept a higher-resolution radar if and when payload allows.
3. **The PATH A clusterer + tracker themselves** stay as today. We are not refactoring them into the UKF.

---

## 5. Alternatives considered

### Alternative B — Feed PATH A clusters into the UKF as synthetic detections

The clusterer already produces what looks like a synthetic `DetectedObject` — a 3D centroid, bounding extent, instance ID, observation count, confidence. Feeding those into the UKF would give radar cross-validation as a free property of the existing fusion math. Architecturally clean.

**Why rejected (specifically for this hardware):** the radar we can fly (TI AWR1843 class, ~14° angular resolution) is the higher-variance bearing measurement. The UKF's covariance-weighted update would pull a well-localised stereo cluster toward a wide radar bearing cone — exactly the "radar corrupts UKF tracks" failure pattern from #229. The fix in that incident was *less* fusion, not more. Rejecting this alternative is a hardware-specific call, not a permanent architectural rejection. If 4D imaging radar becomes flyable, this option becomes the right answer.

A weaker version — feeding clusters into the UKF only for moving-object tracks (where Doppler adds state) and not for static obstacles — is essentially the chosen architecture in §2 item 3. We take the good parts of B and leave the parts that would degrade stereo localisation.

### Alternative C — Replace radar with a denser 4D imaging unit and trust a single fused channel

If we could fly Continental ARS540 / ZF FRGen21 class (sub-1° azimuth, 4D imaging), the angular-resolution gap with stereo collapses to within a factor of 2-4×, and a single fused channel becomes defensible. This is what large autonomous-vehicle stacks do.

**Why rejected for now:** these units are 500–700 g / 18 W. Outside any sUAS payload budget for the platform we are designing. The trade studies for swapping our companion-computer-class radar for an imaging unit live in a separate hardware roadmap, not in this software ADR. If a future airframe revision absorbs the SWaP, supersede this ADR.

### Alternative D — Keep PATH A on its own grid, write the planner against two grids

One occupancy grid per modality, planner consumes both. Maximum architectural separation. Rejected because it forces every grid consumer (planner, avoider, telemetry, replay tooling) to handle two grids; the multiplier on downstream complexity is larger than the cross-veto logic in the chosen design. Defence-in-depth without grid duplication is achievable by §2's late-stage cross-veto at the single-grid boundary.

---

## 6. Implementation hooks

This ADR is the architectural contract. The follow-up issues that implement it:

| Issue   | Title                                                              | Status               |
|---------|--------------------------------------------------------------------|----------------------|
| #698    | PATH A voxels cement walls without radar agreement                 | Open                 |
| PR #699 | Reject DA V2 max-clamp saturation samples in PATH A projector      | Merged candidate     |
| #698 Fix #1 (TBD issue) | Long-range radar veto on PATH A grid promotion (>10 m only) | To file              |
| #696    | P3 VIO pose freezes mid-scenario                                   | Open (workaround #697) |
| TBD     | Stereo HAL backend + replace DA V2 in PATH A                       | Planned              |
| TBD     | Cross-veto logic at grid-write boundary (logged disagreements)     | Planned              |

The cross-veto logic is the only genuinely new code this ADR mandates. Everything else is either already in place (PATH A, radar paths, UKF) or has its own dedicated issue.

---

## 7. References

### Primary sources used in the trade analysis

- Stereolabs ZED 2i datasheet (Mouser): https://www.mouser.com/datasheet/2/1520/StereoLabs_ZED_2i_Datasheet-3401402.pdf
- Stereolabs ZED 2i depth-accuracy Help Center: https://support.stereolabs.com/hc/en-us/articles/1500008533841
- *Depth accuracy analysis of the ZED 2i Stereo Camera in an indoor environment*, Robotics and Autonomous Systems, 2024: https://www.sciencedirect.com/science/article/pii/S0921889024001374
- Intel RealSense D455 product brief: https://www.mouser.com/pdfDocs/D455ProductBriefv90.pdf
- Teledyne Vision Solutions, *Stereo Accuracy and Error Modeling*: https://www.teledynevisionsolutions.com/support/support-center/application-note/iis/stereo-accuracy-and-error-modeling/
- Gallup et al., *Variable Baseline/Resolution Stereo*, CVPR 2008: https://people.inf.ethz.ch/pomarc/pubs/GallupCVPR08.pdf
- TI AWR1843 datasheet: https://www.ti.com/product/AWR1843
- TI app brief, *Understanding Range and Angular Resolution in mmWave Radar* (SWRA841): https://www.ti.com/lit/pdf/swra841
- Wireless Pi, FMCW Radar series (Part 3): https://wirelesspi.com/fmcw-radar-part-3-design-guidelines/
- Mathworks, *Increasing Angular Resolution with MIMO Radars*: https://www.mathworks.com/help/phased/ug/increasing-angular-resolution-with-mimo-radars.html
- Continental ARS540 product page: https://www.continental-automotive.com/en/components/radars/long-range-radars/advanced-radar-sensor-ars540.html
- Smartmicro UMRR-11 Type 132 altimeter datasheet: https://www.smartmicro.com/fileadmin/media/Downloads/Airborne_Radar/UMRR-11_Type_132_Altimeter_Datasheet_PRELIMINARY.pdf
- Ainstein US-D1 brochure: https://ainstein.ai/wp-content/uploads/US-D1-Brochure-1.pdf
- Airsight, *RCS & FPV Stealth: Detecting Carbon-Fiber Drones*: https://www.airsight.com/blog/rcs-fpv-stealth-drone-detection-radar
- *Indoor mmWave Radar Ghost Suppression*, MDPI Sensors 25/11/3377, 2025: https://www.mdpi.com/1424-8220/25/11/3377
- *Perception and sensing for autonomous vehicles under adverse weather conditions: A survey*, ISPRS J., 2022: https://www.sciencedirect.com/science/article/pii/S0924271622003367
- *A Review of Multi-Sensor Fusion in Autonomous Driving*, MDPI Sensors 25/19/6033, 2025: https://www.mdpi.com/1424-8220/25/19/6033
- *Radar and Camera Fusion for Object Detection and Tracking: A Comprehensive Survey*, arXiv:2410.19872, 2024: https://arxiv.org/html/2410.19872v1
- Tang et al., *Radar/visual fusion with fuse-before-track strategy for low-altitude non-cooperative sense and avoid*, Aerospace Sci. Tech., 2024: https://www.sciencedirect.com/science/article/pii/S1270963824000798
- *Enhanced UAV Tracking through Multi-Sensor Fusion and Extended Kalman Filtering*, CEUR Vol-3900 Paper 19, 2024: https://ceur-ws.org/Vol-3900/Paper19.pdf
- *Fast detection and obstacle avoidance on UAVs using lightweight CNN based on the fusion of radar and camera*, Applied Intelligence, 2024: https://link.springer.com/article/10.1007/s10489-024-05768-5

### Production references

- Mobileye True Redundancy: https://www.mobileye.com/technology/true-redundancy/
- Mobileye blog, *AV Safety Demands True Redundancy*: https://www.mobileye.com/blog/av-safety-demands-true-redundancy/
- Waymo blog, *Meet the 6th-generation Waymo Driver*, 2024: https://waymo.com/blog/2024/08/meet-the-6th-generation-waymo-driver/
- Skydio X10 technical specs: https://www.skydio.com/x10/technical-specs
- Parrot ANAFI Ai obstacle avoidance documentation: https://www.parrot.com/en/support/anafi-ai/how-does-anafi-ais-obstacle-avoidance-work

### Internal references

- Issue #229 — radar fusion corrupting UKF tracks (project memory: `project_issue229_radar_fusion`)
- Epic #237 — sensor fusion architecture (project memory: `project_epic237_architecture`)
- Issue #698 — PATH A grid promotion has no radar veto
- PR #699 — Reject DA V2 max-clamp saturation samples
- PR #697 — Cosys ground-truth pose passthrough (issue #696 workaround that surfaced the perception bug)
- `docs/design/perception_design.md` — current PATH A pipeline detail
- [ADR-006](ADR-006-hal-hardware-abstraction-strategy.md) — HAL strategy (the boundary at which the stereo backend plugs in)
- [ADR-009](ADR-009-tier1-tier2-simulation-architecture.md) — simulation architecture (the test infrastructure that validates the parallel paths)

---

## 8. Revisit triggers

This ADR should be reopened if any of the following occur:

1. **Airframe payload allows a 4D imaging radar** (sub-1° azimuth, ~500 g, ~18 W class). Radar bearings become peer-quality to camera; Alternative C in §5 becomes the right answer.
2. **A field incident demonstrates the cross-veto logic itself failing** in a way that takes down both modalities simultaneously. The defence-in-depth premise is invalidated and the architecture must be reconsidered (likely toward stricter modal isolation, not toward unified fusion).
3. **Certification authority feedback** specifically requires unified state estimation rather than parallel paths. So far the literature suggests the opposite, but a binding regulatory call would override the analysis.
4. **Stereo replacement matures to a point where its long-range performance approaches radar** (e.g. learned long-baseline stereo with reliable ≥30 m depth). At that point the radar's primary value is reduced to Doppler and adverse-weather coverage, and a two-channel architecture may simplify to one with radar as a narrowly-scoped specialist.

Until one of those triggers fires, the parallel-paths-with-cross-veto architecture is the authoritative shape.
