# ADR-014: Stereo-Inertial VIO Algorithm Selection

| Field | Value |
|-------|-------|
| **Status** | Accepted (with §9 amendment 2026-05-13 — FTO design-around constraints) |
| **Date** | 2026-04-18 (original); 2026-05-13 (FTO amendment) |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | ADR-009 (Tier 1/Tier 2 simulation), ADR-011 (Cosys-AirSim), ADR-013 (stereo-radar redundancy vs fusion), Issue #191 (Gazebo stereo VIO), Issue #254 (covariance-based VIO quality), Issue #497 (Custom SWVIO epic), Issue #504 (SWVIO Phase 2) |

---

## 1. Context

### The VIO Gap

The VIO pipeline (P3, `process3_slam_vio_nav`) has complete HAL interfaces (`IVIOBackend`, `IFeatureExtractor`, `IStereoMatcher`, `ImuPreintegrator`) and a production-grade IMU pre-integrator (Forster et al. 2017), but **no real visual-inertial fusion algorithm**. Current backends:

- **SimulatedVIOBackend** — runs the full pipeline (feature extraction → stereo matching → IMU pre-integration) but generates pose independently via target-following dynamics. No actual fusion.
- **GazeboVIOBackend** — returns Gazebo ground-truth odometry. Ignores stereo frames and IMU.
- **GazeboFullVIOBackend** — runs the full pipeline on Gazebo-rendered frames but uses ground-truth for the final pose output.
- **CosysVIOBackend** (`ivio_backend.h:740+`) — polls Cosys-AirSim ground-truth kinematics from the simulator's RPC interface. Like the Gazebo backends, this is a ground-truth pass-through for sim integration; it does no actual visual-inertial fusion.

The factory comment at `ivio_backend.h:886` explicitly states: *"To add a real VIO algorithm (e.g. MSCKF), add a new class implementing IVIOBackend and register it here."*

### Requirements

1. **Accuracy** — drone waypoint navigation requires <2% position drift over a 10-minute mission
2. **Real-time** — 30 Hz frame rate on target hardware (NVIDIA Jetson Orin, 8-core ARM, 32GB, GPU)
3. **Safety-critical** — bounded memory, deterministic compute, graceful degradation on tracking loss
4. **Multi-sensor** — must integrate with existing radar, ML depth estimation, and object detection pipelines
5. **License** — MIT/BSD compatible for production deployment
6. **Dependencies** — minimize external library dependencies; Eigen3 is the only linear algebra library currently linked
7. **Long-term ceiling** — loop closure capability for missions >15 minutes

### Existing Infrastructure to Leverage

- `ImuPreintegrator` — Forster et al. 2017 on-manifold pre-integration with 9×9 covariance and 9×6 bias Jacobians
- `IFeatureExtractor` / `IStereoMatcher` — HAL interfaces with simulated implementations
- `StereoCalibration` — intrinsics + baseline for depth-from-disparity
- `VIOResult<T>` — monadic error handling pattern
- `KeyframePolicy` — parallax/ratio/time-based keyframe selection
- P3 threading: IMU reader (400 Hz), VIO pipeline (~30 Hz), pose publisher (100 Hz)

---

## 2. Decision

**Implement a custom sliding-window optimization VIO with lightweight loop closure**, built in-house using only Eigen. This is the same algorithm class as VINS-Fusion (sliding-window bundle adjustment + pose graph loop closure) but MIT-licensed, integrated with our HAL architecture, and extensible for multi-sensor fusion.

The backend is named `SlidingWindowVIOBackend` (config key: `"swvio"`).

---

## 3. Alternatives Considered

### 3.1 MSCKF (Multi-State Constraint Kalman Filter)

An EKF-based filter that maintains a sliding window of camera poses in its state vector. Features are used as constraints between poses via null-space projection, then discarded (never added to the state).

**Strengths:**
- Simplest to implement (~1700 lines)
- O(n) in features, lowest compute per frame (~2-3 ms)
- Fixed state size → bounded memory, fully deterministic
- Zero new dependencies (Eigen only)
- Well-proven for drones (OpenVINS is the reference MSCKF implementation)

**Weaknesses:**
- ~30-50% lower accuracy than optimization-based methods (single linearization point)
- No loop closure — drift accumulates unboundedly over long missions
- Lossy marginalization — information permanently lost when camera clones are removed
- Linearization inconsistency — Jacobians evaluated at potentially incorrect state estimate
- Limited to point features — cannot incorporate line features, planes, or dense depth
- Sensitive to feature tracking quality — degrades sharply with motion blur or low texture

**Verdict:** Sufficient for short missions (<10 min) but insufficient long-term ceiling. The accuracy gap and lack of loop closure are real limitations for survey and mapping missions.

### 3.2 VINS-Fusion

Sliding-window optimization VIO with loop closure, from HKUST. The gold standard for drone VIO.

**Strengths:**
- Battle-tested by thousands of researchers across hundreds of datasets
- Stereo-inertial mode with loop closure via DBoW2
- High accuracy on standard benchmarks (EuRoC, TUM-VI)
- Active community

**Weaknesses:**
- **GPL v3 license** — requires open-sourcing any code that links against it. Non-starter for production deployment.
- Ceres Solver dependency (~300K lines) — large build, ARM cross-compilation overhead
- Own threading model (4 threads) — conflicts with our P3 architecture
- Own IMU pre-integrator — duplicates our existing production-grade Forster implementation
- Own feature tracker — bypasses our `IFeatureExtractor` interface
- Monolithic design — not a library, a full system. Hard to wrap behind `IVIOBackend`

**Verdict:** Algorithm is excellent; license and architecture are incompatible.

### 3.3 ORB-SLAM3

Full visual-inertial SLAM with map reuse, loop closure, multi-map, and relocalization.

**Strengths:**
- Most complete SLAM system available
- Stereo-inertial mode, multi-map, relocalization in known environments
- Highest accuracy on benchmarks

**Weaknesses:**
- **GPL v3 license** — same production blocker as VINS-Fusion
- Unbounded memory (full map of keyframes + map points grows with environment size)
- Complex failure modes — map corruption, spurious loop closures, relocalization failures
- Dependencies: g2o, DBoW2, OpenCV
- Overkill for drone VIO — most missions don't revisit locations or need persistent maps

**Verdict:** Right algorithm for autonomous cars and robots; overkill and license-incompatible for drone VIO.

### 3.4 Kimera-VIO

Stereo-inertial VIO from MIT SPARK Lab with loop closure (Kimera-RPGO) and mesh reconstruction.

**Strengths:**
- BSD 2-Clause license — production-friendly (matches `docs/architecture/MAKE_OR_BUY.md:498` inventory)
- Loop closure via robust pose graph optimization
- Well-tested on drone platforms (MIT's aerial vehicles)
- Active development (as of 2024-2025)

**Weaknesses:**
- **GTSAM dependency** (~500K lines of C++) — massive build, ARM cross-compilation uncertain
- GTSAM uses unbounded memory (factor graph grows), free allocations in hot paths, exception-based error handling — all violate our safety-critical coding standards
- Integration complexity: Kimera has its own threading model, image pipeline, IMU handling
- Debugging: failures surface deep inside GTSAM's template metaprogramming
- Version coupling: GTSAM updates can break Kimera silently

**Verdict:** License is right; dependency profile is wrong for embedded safety-critical. GTSAM's `shared_ptr`-heavy, exception-throwing, unbounded-memory architecture conflicts with our `Result<T,E>`, RAII, bounded-memory design philosophy.

### 3.5 Basalt VIO

Sliding-window optimization VIO with stereo + IMU support, from TUM (Usenko et al.). The fastest open-source VIO with the best published accuracy on EuRoC and TUM-VI benchmarks.

**Strengths:**
- BSD 3-Clause license — production-friendly (per `docs/architecture/MAKE_OR_BUY.md:496` inventory: "Best open-source VIO accuracy, permissive license, stereo + IMU, visual-inertial bundle adjustment, actively maintained")
- Top-tier accuracy (0.012m on EuRoC per the inventory)
- High throughput (~300 Hz claimed on capable hardware)
- Stereo-inertial mode + visual-inertial bundle adjustment
- Actively maintained

**Weaknesses:**
- Research-grade code quality — limited documentation, integration effort estimated at 4-6 weeks per the inventory
- Heavy template-metaprogramming-driven implementation — debugging complexity
- Custom build system + dependency profile (TBB, Sophus, OpenCV) — non-trivial ARM cross-compilation
- Own threading + frame pipeline — conflicts with our `IVIOBackend` interface boundary and process-3 thread model
- Bundle-adjustment hot paths use unbounded heap allocation patterns inconsistent with our safety-critical bounded-memory / `Result<T,E>` discipline

**Verdict:** Strongest algorithm + license combination in the open-source landscape. The dependency footprint (TBB threading model + research-grade code organisation) makes integration into the safety-critical P3 architecture costly. The custom SWVIO path (§3.6) reaches a similar accuracy ceiling with deterministic-compute / bounded-memory guarantees aligned with our existing patterns. Basalt remains the prime fallback if the custom path overruns its scope budget.

### 3.6 Custom Sliding-Window Optimization VIO (Chosen)

Build the same algorithm as VINS-Fusion (sliding-window bundle adjustment + pose graph loop closure) using only Eigen.

**Strengths:**
- Same accuracy as VINS-Fusion — iterative Gauss-Newton re-linearization
- Loop closure via async pose graph module
- Zero new dependencies — Eigen's `SparseQR`/`SimplicialLDLT` for sparse solves
- MIT license (our code)
- Integrates with existing HAL interfaces — reuses our IMU pre-integrator, feature extractor, stereo matcher
- Multi-sensor extensibility — can add radar range, ML depth priors, dynamic object masking as additional residuals
- Safety-critical: bounded sliding window, `Result<T,E>` error handling, RAII throughout
- Bounded compute: fixed window caps optimization cost; 3-5 GN iterations at 30 Hz

**Weaknesses:**
- Custom implementation risk — we own all the bugs (mitigation: extensive testing, phased delivery)
- More scope than MSCKF (~3500 vs ~1700 lines) (mitigation: 5 independent phases)
- Iterative solver may not converge in degenerate cases (mitigation: iteration cap, LM damping, IMU fallback)
- Loop closure vocabulary quality depends on feature descriptor quality (mitigation: OpenCV ORB, geometric verification)

---

## 4. Decision Rationale

The decision comes down to three axes:

### 4.1 License eliminates two options

VINS-Fusion and ORB-SLAM3 are GPL v3. For a production drone product, this requires open-sourcing the entire linked binary. This is a hard constraint, not a preference.

### 4.2 Dependency philosophy eliminates one more

Kimera-VIO's MIT license is correct, but GTSAM's 500K-line dependency conflicts with our embedded safety-critical design:
- GTSAM uses `std::shared_ptr` pervasively — our CLAUDE.md explicitly calls this a code smell in safety code
- GTSAM throws exceptions on numerical failures — we use `Result<T,E>` throughout
- GTSAM allocates dynamically in hot paths — we require bounded memory for deterministic execution
- ARM cross-compilation is unverified — risk of blocking deployment on Jetson Orin

Wrapping a library that violates our coding standards inside our safety-critical stack creates a fragile boundary that will leak on every edge case.

### 4.3 Accuracy and long-term ceiling eliminate MSCKF

MSCKF is the simplest path to working VIO, but:
- ~30-50% accuracy gap vs. optimization-based methods is significant for survey missions
- No loop closure means unbounded drift — problematic for any mission >10 minutes
- The accuracy gap matters most in aggressive maneuvers (high angular rate) where linearization error is highest — exactly the scenarios a drone encounters

### 4.4 Multi-sensor advantage is unique to custom

Our stack has capabilities that no standalone VIO system can access:
- **Radar range fusion** — bounds drift independent of vision (fog, darkness, featureless terrain)
- **Dynamic object masking** — P2 detects moving objects; mask them before feature extraction
- **ML depth priors** — regularize stereo depth for far objects beyond baseline range
- **Health-aware degradation** — graceful failover using 3-tier watchdog, not hard crash

These are additional optimization residuals that slot naturally into a sliding-window optimizer but cannot be added to a filter (MSCKF) or an external system (Kimera) without major architectural changes.

---

## 5. Consequences

### Positive

- Full control over the VIO algorithm — can tune, extend, and debug without external dependency constraints
- Multi-sensor fusion capability unique to our stack
- Safety-critical compliance: bounded memory, deterministic compute, `Result<T,E>` error handling
- HAL interface (`IVIOBackend`) means the algorithm can be replaced or augmented without downstream changes
- Phased delivery: each phase is independently buildable and testable

### Negative

- Higher initial development effort (~3500 lines vs. ~500 for Kimera wrapper or ~1700 for MSCKF)
- Custom optimizer bugs are our responsibility — no community to fall back on
- Need to build loop closure from scratch (vocabulary, geometric verification, pose graph)
- Validation requires both Gazebo and Cosys-AirSim to be confidence-complete

### Neutral

- The `IVIOBackend` interface protects against lock-in — if a future MIT-licensed VIO library emerges that meets our constraints, we can adopt it behind the same interface
- IMU propagation and feature tracking code is shared with MSCKF — the investment isn't lost if we later downgrade to a filter for resource-constrained platforms

---

## 6. Implementation Plan

Five phases, each independently deliverable:

| Phase | Description | Scope | Depends On |
|-------|------------|-------|------------|
| 1 | IMU propagation + state augmentation | ~880 lines | None |
| 2 | Sliding-window GN optimization | ~1170 lines | Phase 1 |
| 3 | Robustness + OpenCV extractors + validation | ~700 lines | Phase 2 |
| 4 | Loop closure module (async) | ~690 lines | Phase 2 |
| 5 | Multi-sensor enhancements (radar, masking, ML depth, GPU, health) | ~800 lines | Phase 2 |

Total: ~4240 lines + ~1400 test lines across ~23 files.

See the SWVIO epic (Issue #497) for the overall arc and Issue #504 specifically for the Phase 2 (Gauss-Newton sliding-window optimizer) plan — file lists, algorithm specifications, and per-phase test strategies are captured in those issues. Phase 1 (#498) is merged via PR #500.

---

## 7. Revisit Criteria

Re-evaluate this decision when:

- **Mission duration exceeds 30 minutes** with <0.5% drift requirement — may need tighter loop closure or multi-session map
- **Map building and reuse** across missions becomes a requirement — would need persistent landmark database
- **Multi-vehicle cooperative SLAM** is needed — factor graph (GTSAM) may become justified
- **A production-grade MIT-licensed VIO library** emerges that matches our safety-critical constraints and dependency budget
- **Jetson Orin GPU utilization** analysis shows the custom GN solver is a bottleneck — consider GPU-accelerated optimization

---

## 8. References

- Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation," ICRA 2007
- Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry," T-RO 2017
- Qin, Li & Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," T-RO 2018
- Qin et al., "VINS-Fusion: A Robust and Versatile Stereo Visual-Inertial State Estimator," T-RO extension 2019
- Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM," T-RO 2021
- Rosinol et al., "Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping," ICRA 2020
- Geneva et al., "OpenVINS: A Research Platform for Visual-Inertial State Estimation," IROS 2020
- Leutenegger et al., "Keyframe-Based Visual-Inertial Odometry Using Nonlinear Optimization (OKVIS)," IJRR 2015
- Usenko et al., "Visual-Inertial Mapping with Non-Linear Factor Recovery (Basalt)," IEEE RAL 2020
- Stumberg & Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry," IEEE RAL 2022
- `docs/architecture/MAKE_OR_BUY.md` — internal make-or-buy inventory for VIO / SLAM components

---

## 9. Freedom-to-Operate Design-Around Choices (added 2026-05-13)

### 9.1 Why this section exists

Following a research-grade patentability search and Freedom-to-Operate (FTO) analysis performed 2026-05-13, three implementation choices in this ADR's selected algorithm are now documented as **deliberate FTO-motivated design-around** decisions, not merely engineering preferences. This conversion of implicit architectural choices into documented intentional design-around strengthens the FTO posture should any of the surveyed patents ever be asserted.

The full FTO analysis is preserved in the project's private business repository at `business/legal/fto-swvio-phase2.md`, alongside the patentability prior-art analysis at `business/legal/prior-art/swvio-phase2.md`. This ADR carries only the public-safe summary of the design-around choices themselves.

This section is research-grade input to a future legal-attorney FTO opinion, not a substitute for one.

### 9.2 The three deliberate design-around choices

#### 9.2.1 Indirect feature representation (not direct sparse VIO)

**Decision:** the SWVIO backend uses **indirect features** — extracted keypoints with descriptors matched across frames — rather than **direct sparse VIO** operating on raw pixel intensities.

**FTO rationale:** US20190301871A1 ("Direct Sparse VIO Using Dynamic Marginalization", Google) recites "direct" feature-tracking on pixel intensities as a claim limitation. The indirect-feature approach falls outside this limitation, eliminating a key element of the claim's coverage.

**Engineering rationale (independent of FTO):** indirect features are more robust to lighting changes and motion blur, consistent with the safety-critical drone-deployment scenarios validated in Cosys-AirSim Tier-3. The choice was made on engineering merits and is reinforced by the FTO analysis.

**Where in the code (planned, Issue #504):** the Phase 2 implementation in `process3_slam_vio_nav/src/swvio_backend.cpp` (extension of the Phase 1 file added via PR #500) and the new `swvio_optimizer.{h,cpp}` use the existing `IFeatureExtractor` interface (ORB, FAST, or equivalent keypoint extractor) followed by descriptor matching at the feature-extraction stage. Direct (pixel-intensity / photometric residual) feature handling is explicitly out of scope. As of this ADR update (2026-05-13) the Phase 2 files have not been added to the repository yet; this design-around constraint is recorded here so the Phase 2 implementation lands aligned to it.

#### 9.2.2 Static Schur marginalization (not dynamic / state-adaptive)

**Decision:** the SWVIO optimizer uses a **fixed, static Schur-complement marginalization strategy** — fixed window size, fixed feature-elimination order, fixed marginalization triggers — rather than a **dynamic / state-adaptive** marginalization that varies the strategy based on runtime observations.

**FTO rationale:** US20190301871A1's claim language emphasizes "dynamic" marginalization. Static marginalization falls outside this limitation. Additionally, the US9658070B2 family (Roumeliotis-style inverse sliding-window filters) recites adaptive linearization-point selection; the static approach further differentiates.

**Engineering rationale (independent of FTO):** static marginalization is deterministic, easier to validate under safety-critical scrutiny (ISO 26262-aligned analyses), and consistent with the project's bounded-memory / deterministic-compute requirements (Section 1, Requirement 3).

**Where in the code (planned, Issue #504):** `process3_slam_vio_nav/include/slam/swvio_optimizer.h` and `process3_slam_vio_nav/src/swvio_optimizer.cpp` will define a fixed `max_clones` window size, fixed feature-elimination order per Gauss-Newton iteration, and trigger marginalization only on window-overflow (no state-adaptive trigger). As of this ADR update (2026-05-13) these files have not been added to the repository yet.

#### 9.2.3 Optimizer architecture (not filter / Kalman family)

**Decision:** the SWVIO backend is implemented as an **iterative Gauss-Newton optimizer** over a sliding window of poses + landmarks, **not** as a filter (Extended Kalman Filter, Multi-State Constraint Kalman Filter, Square-Root Inverse Schmidt-Kalman, or similar).

**FTO rationale:** the US9658070B2 / US20160327395A1 / US20190178646A1 patent family is consistently framed around **inverse sliding-window filters** (ISWF) — the filter-vs-optimizer architectural distinction is the dispositive defence against these patents per the FTO analysis. Implementing as an optimizer (with iterative re-linearization, manifold updates via SO(3) exp-map, and converged residual minimization rather than single-pass covariance update) places the SWVIO architecture outside the filter family these patents recite.

**Engineering rationale (independent of FTO):** iterative optimization with re-linearization is the canonical approach in OKVIS, VINS-Fusion, ORB-SLAM3, Basalt, DM-VIO — all open-source VIO implementations whose academic justification is improved accuracy over EKF-style filters via reduced linearization error. This is also the headline differentiator stated in the ADR's own Section 4 Decision Rationale.

**Where in the code (planned, Issue #504):** the Phase 2 `swvio_backend.cpp` extension will invoke `SWVIOOptimizer::optimize()` (new class in `process3_slam_vio_nav/include/slam/swvio_optimizer.h`), which performs 3-5 Gauss-Newton iterations with Schur complement reduction, manifold updates on SO(3), and convergence checking on update norm. No EKF / MSCKF / Schmidt-Kalman covariance-propagation pattern is used. As of this ADR update (2026-05-13) the Phase 2 files have not been added to the repository yet; this design-around constraint is recorded here so the Phase 2 implementation lands aligned to it.

### 9.3 What this section does NOT claim

- It does not claim the algorithm is non-infringing in absolute terms — that requires a paid legal-attorney FTO opinion (budgeted at Series A per `business/legal/ip-strategy.md`).
- It does not claim novelty over prior art — the patentability search at `business/legal/prior-art/swvio-phase2.md` concluded the algorithm's individual components are well-anticipated by published literature (OKVIS 2015, VINS-Fusion 2019, Basalt 2020, DM-VIO 2022, MSCKF 2007).
- It does not claim immunity from future patents not yet identified.
- It does not constitute legal advice.

### 9.4 Empirical FTO context

No litigation has surfaced against any of the major open-source VIO projects (OKVIS, VINS-Fusion, OpenVINS, Kimera-VIO, Basalt, DM-VIO, ORB-SLAM3) in 10+ years of public availability, despite some of these being closer to the threatening claims than this ADR's planned implementation. This is the single strongest empirical FTO signal.

Jurisdictional carve-outs further mitigate risk for defence-tech-deployed builds:
- **US 28 USC § 1498** — government-use compensation rights (reaffirmed in the 2026 Federal Circuit AeroVironment / Ingenuity case)
- **UK Crown use** — Patents Act 1977 s.55-59
- **Germany § 13 PatG** — Crown use
- **EU Article 346 TFEU** — defence essential security interests carve-out

### 9.5 Revisit triggers (for §9 specifically)

This section should be re-evaluated when:

- First commercial customer contract is signed (Stage 1A → Stage 2 transition per `business/strategy/technical-funding-org-roadmap.md`)
- Any of the surveyed patents is asserted publicly against any open-source VIO project
- US entry begins (Series B per the funding roadmap) — formal FTO opinion budget (€40-80K) activates
- A new patent in the SWVIO space is published with broader claims
- The architecture deviates from any of the three design-around choices in §9.2

### 9.6 Cross-references

- `business/legal/fto-swvio-phase2.md` — full FTO analysis (private)
- `business/legal/prior-art/swvio-phase2.md` — full patentability analysis (private)
- `business/legal/ip-strategy.md` — IP strategy framework
- `business/legal/swvio-ip-package.md` — canonical SWVIO IP analysis bundle
- Issue #497 — Custom SWVIO epic
- Issue #504 — SWVIO Phase 2 (where these design-around choices manifest in code)
- ADR-013 — stereo-radar redundancy vs fusion (related sensor-fusion decision)
