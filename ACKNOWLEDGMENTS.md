# Acknowledgments

## Academic Papers

The following algorithms were implemented from scratch based on published academic research:

| Algorithm | Paper | Used In |
|-----------|-------|---------|
| **ByteTrack** | Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every Detection Box", ECCV 2022 | P2 Perception — `bytetrack_tracker.h` |
| **IMU Pre-integration** | Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", 2017 | P3 SLAM/VIO — `imu_preintegrator.h` |
| **D\* Lite** | Koenig & Likhachev, "D\* Lite", AAAI 2002 | P4 Mission Planner — `dstar_lite_planner.h` |
| **SORT** | Bewley et al., "Simple Online and Realtime Tracking", ICIP 2016 | P2 Perception — `kalman_tracker.h` |
| **Hungarian Algorithm** | Kuhn (1955), Munkres (1957) — O(n³) optimal bipartite assignment | P2 Perception — `kalman_tracker.h` |

All implementations are original — no source code was copied from reference implementations.

## Third-Party Libraries

| Library | License | Purpose |
|---------|---------|---------|
| [spdlog](https://github.com/gabime/spdlog) | MIT | Structured logging |
| [Eigen3](https://eigen.tuxfamily.org) | MPL 2.0 | Linear algebra |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | JSON config parsing |
| [Google Test](https://github.com/google/googletest) | BSD 3-Clause | Unit testing |
| [Eclipse Zenoh](https://zenoh.io) | EPL 2.0 | IPC transport |
| [OpenCV](https://opencv.org) | Apache 2.0 | DNN inference (optional) |
| [MAVSDK](https://mavsdk.mavlink.io) | BSD 3-Clause | MAVLink flight controller link (optional) |
| [Gazebo](https://gazebosim.org) | Apache 2.0 | SITL simulation (optional) |
| [PX4-Autopilot](https://px4.io) | BSD 3-Clause | Flight controller SITL (dev/test only) |

## ML Models

| Model | Source | License | Notes |
|-------|--------|---------|-------|
| YOLOv8n ONNX | [Ultralytics](https://github.com/ultralytics/ultralytics) | **AGPL 3.0** | Optional — disabled without `HAS_OPENCV`. See license note below. |

### YOLOv8 AGPL 3.0 License Note

The YOLOv8n model weights are licensed under AGPL 3.0 by Ultralytics. If you build with `HAS_OPENCV=ON` and use YOLOv8 detection in a deployed product:

- **AGPL 3.0 requires** source code disclosure to end-users and granting reciprocal modification rights
- **Alternatives:** Use `ColorContourDetector` (project-original, no license constraint), obtain a [commercial Ultralytics license](https://ultralytics.com/license), or substitute a permissively-licensed model
- The model file is `.gitignore`'d and downloaded on demand — it is not distributed with this source code
