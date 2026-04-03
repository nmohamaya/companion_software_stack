# License & Dependency Audit

**Audit date:** 2026-03-18
**Scope:** All source code, build dependencies, algorithms, models, and vendored code in the companion software stack.

---

## Summary

| Category | Count | Details |
|----------|-------|---------|
| Required core dependencies | 6 | spdlog, Eigen3, nlohmann-json, GTest, Threads, zenohc |
| Optional dependencies | 5 | OpenCV, MAVSDK, gz-transport13, gz-msgs10, libsystemd |
| Original algorithm implementations | 9 | SORT, ByteTrack, UKF fusion, IMU pre-integration, A*, D* Lite, PotentialField, ObstacleAvoider3D, ColorContourDetector |
| Academic citations | 3 | Forster et al. (2017), Koenig & Likhachev (2002), Zhang et al. (2022) |
| Third-party models | 1 | YOLOv8n ONNX (Ultralytics, **AGPL 3.0**) |
| Vendored/copied code | 0 | All external code linked via CMake |
| License compliance issues | 1 | YOLOv8n AGPL 3.0 — see below |

**Verdict:** All dependencies except YOLOv8n are permissive (MIT, BSD, Apache 2.0, MPL 2.0, EPL 2.0). All major algorithms are original implementations with academic paper attribution where applicable. No vendored third-party source code found.

---

## 1. Required Build Dependencies

| Library | Version | License | Purpose |
|---------|---------|---------|---------|
| **spdlog** | 1.12.0 | MIT | Structured logging (all 7 processes) |
| **Eigen3** | 3.4.0 | MPL 2.0 | Linear algebra — Kalman filters, pose math, planning |
| **nlohmann-json** | 3.11.3 | MIT | JSON config parsing (`drone::Config`) |
| **GTest** | 1.14.0 | BSD 3-Clause | Unit testing (see [tests/TESTS.md](../../tests/TESTS.md) for count) |
| **Threads** | System | N/A | POSIX threading |
| **zenohc** | 1.7.2 | EPL 2.0 | IPC transport (sole backend) |

All required dependencies are permissive and compatible with proprietary use.

### Zenoh EPL 2.0 Note

EPL 2.0 is permissive for projects that *use* Zenoh as a library. If we *modify* Zenoh source code itself, those modifications must be disclosed under EPL 2.0. We do not modify Zenoh — no action required.

---

## 2. Optional Dependencies

| Library | Version | License | Purpose | Compile Guard |
|---------|---------|---------|---------|---------------|
| **OpenCV** | 4.10.0 | Apache 2.0 | YOLOv8 ONNX DNN inference | `HAVE_OPENCV` / `HAS_OPENCV` |
| **MAVSDK** | 2.12.12 | BSD 3-Clause | MAVLink flight controller comms | `HAVE_MAVSDK` |
| **gz-transport13** | 13.x | Apache 2.0 | Gazebo SITL transport | `HAVE_GAZEBO` |
| **gz-msgs10** | 10.x | Apache 2.0 | Gazebo message definitions | `HAVE_GAZEBO` |
| **libsystemd** | System | LGPL 2.1+ | sd_notify() watchdog | `HAVE_SYSTEMD` |

All optional dependencies are permissive. libsystemd is LGPL 2.1+ but dynamically linked only — no copyleft concern.

---

## 3. Algorithm Implementations (All Original)

### 3.1 Multi-Object Tracking (Process 2)

**SORT Tracker** — `process2_perception/include/perception/kalman_tracker.h`
- Original implementation of the SORT algorithm
- Components: 8D constant-velocity Kalman filter, O(n³) Hungarian (Kuhn-Munkres) solver
- Uses Eigen3 for matrix operations
- License: Project-original

**ByteTrack Tracker** — `process2_perception/include/perception/bytetrack_tracker.h`
- Original implementation based on: Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every Detection Box", ECCV 2022
- Two-stage Hungarian IoU association (high-confidence + low-confidence recovery)
- Reuses KalmanBoxTracker and HungarianSolver from SORT (no code duplication)
- Attribution in source file header
- License: Project-original

### 3.2 Sensor Fusion (Process 2)

**UKF Fusion Engine** — `process2_perception/include/perception/ukf_fusion_engine.h`
- Original Unscented Kalman Filter implementation
- Per-object state: [x, y, z, vx, vy, vz] with camera + thermal measurement models
- Sigma point generation via Cholesky decomposition
- License: Project-original

**Camera-Only Fusion Engine** — `process2_perception/include/perception/fusion_engine.h`
- Original pinhole camera model unproject + depth-from-bbox heuristic
- License: Project-original

### 3.3 Visual-Inertial Odometry (Process 3)

**IMU Pre-integration** — `process3_slam_vio_nav/include/slam/imu_preintegrator.h`
- Original implementation following: Forster et al. (2017), "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
- First-order noise propagation with bias Jacobians
- Attribution in source file header
- License: Project-original

### 3.4 Path Planning (Process 4)

**A* Search** — `process4_mission_planner/include/planner/astar_planner.h`
- Original 3D A* on 26-connected occupancy grid
- Euclidean heuristic, obstacle inflation, caching optimization
- License: Project-original

**D* Lite** — `process4_mission_planner/include/planner/dstar_lite_planner.h`
- Original implementation based on: Koenig & Likhachev (2002), "D* Lite"
- Incremental replanning with priority queue persistence
- Attribution in source file header
- License: Project-original

### 3.5 Obstacle Avoidance (Process 4)

**PotentialFieldAvoider** — 2D inverse-square repulsive force field. Project-original.

**ObstacleAvoider3D** — `process4_mission_planner/include/planner/obstacle_avoider_3d.h`
- Velocity-Obstacle-inspired 3D repulsive field with vertical forces and velocity prediction
- License: Project-original

### 3.6 Object Detection (Process 2)

**ColorContourDetector** — `process2_perception/include/perception/color_contour_detector.h`
- Original HSV segmentation + union-find connected-component labeling
- Optimizations: single-pass RGB→HSV, spatial subsampling, FPS throttle
- License: Project-original

**OpenCV YOLOv8 Detector** — `process2_perception/include/perception/opencv_yolo_detector.h`
- Wrapper around YOLOv8n ONNX model using OpenCV DNN module
- Code is project-original; **model weights are AGPL 3.0** (see section 5)

---

## 4. Vendored / Copied Code

**None detected.** All third-party functionality is linked via CMake `find_package()`. No external source files have been copied into the repository.

---

## 5. AGPL 3.0 Concern — YOLOv8 Model

| Item | Detail |
|------|--------|
| Model | `models/yolov8n.onnx` (12.8 MB) |
| Source | Ultralytics YOLOv8n, exported via `ultralytics` Python package |
| License | **AGPL 3.0** |
| Download script | `models/download_yolov8n.sh` |
| Compile guard | `HAS_OPENCV=1` (disabled by default) |

### Impact

If YOLOv8n is included in a proprietary/closed-source product deployment, AGPL 3.0 requires:
- Full source code disclosure to end-users
- Granting reciprocal modification rights

### Mitigations

- YOLOv8 is **optional** — disabled without `HAS_OPENCV`
- Model file is `.gitignore`'d (not checked into repo)
- Production can use `ColorContourDetector` (project-original, no license constraint)
- If AGPL is incompatible with product licensing, consider:
  - Training a custom model with permissive weights
  - Using YOLO-NAS or RT-DETR (Apache 2.0 alternatives)
  - Licensing YOLOv8 commercially from Ultralytics

---

## 6. Development & Build Tools

| Tool | License | Notes |
|------|---------|-------|
| CMake 3.16+ | BSD 3-Clause | Build system |
| GCC 13.3 | GPL 3.0 | Compiler (output is not GPL-encumbered) |
| clang-format-18 | NCSA | Code formatting |
| clang-tidy-18 | NCSA | Static analysis |
| lcov | GPL 2+ | Coverage reporting (dev tool only) |
| Python 3 | PSF | GCS client, model export |
| Gazebo Harmonic | Apache 2.0 | Simulation (dev/test only) |
| PX4-Autopilot | BSD 3-Clause | SITL flight controller (dev/test only) |

GCC's GPL does not apply to compiled output. All build tools are standard and carry no deployment-time license obligations.

---

## 7. Simulation Assets

| Asset | Location | License |
|-------|----------|---------|
| x500_companion drone model | `sim/models/x500_companion/` | Project-original |
| test_world.sdf | `sim/worlds/test_world.sdf` | Project-original |

Custom simulation assets — no third-party models or meshes.

---

## 8. Recommendations

1. ~~**Add a LICENSE file** to the repository root with the project's chosen license~~ — Done (BSD 3-Clause, `LICENSE`)
2. ~~**Document the YOLOv8 AGPL constraint** in README for contributors~~ — Done (added note in `README.md` dependencies section)
3. **Evaluate AGPL-free detection alternatives** before commercial deployment with YOLOv8 enabled — See make-or-buy analysis
4. ~~**Add ACKNOWLEDGMENTS.md** listing academic papers and third-party libraries~~ — Done (`ACKNOWLEDGMENTS.md`)

---

*Last updated: 2026-03-18*
