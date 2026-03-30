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

### License Compatibility Notes

**Zenoh EPL 2.0**

Eclipse Zenoh is the **primary IPC backend** for the stack. EPL 2.0 (Eclipse Public License 2.0) is a permissive open-source license with weak copyleft:
- ✅ **Compatible with BSD 3-Clause** — the stack's license
- ✅ **Linking only** — this stack links Zenoh without modifying it, so copyleft does not activate
- ⚠️ **If you modify Zenoh** — modifications must be disclosed under EPL 2.0
- ✅ **Alternative available** — POSIX SHM backend (`ipc_backend: "shm"` in config) has no external dependencies and is EPL-free

**For unencumbered deployment**, you can build with `HAVE_ZENOH=OFF` and use the POSIX SHM IPC backend instead. All stack features remain available.

## ML Models

| Model | Source | License | Notes |
|-------|--------|---------|-------|
| YOLOv8n ONNX | [Ultralytics](https://github.com/ultralytics/ultralytics) | **AGPL 3.0** | Optional — disabled without `HAS_OPENCV`. See license note below. |

### YOLOv8 AGPL 3.0 License Note

The YOLOv8n model weights are licensed under **AGPL 3.0** by Ultralytics. 

**What AGPL 3.0 Means:**
- AGPL 3.0 is a **copyleft license** — if you distribute or deploy software using YOLOv8, you must provide source code access to end-users with reciprocal rights
- Unlike permissive licenses (MIT, BSD), you cannot use YOLOv8 in commercial closed-source products without disclosure or a commercial license

**Deployment Options:**

1. **Use `ColorContourDetector`** (Recommended for proprietary deployment)
   - Project-original HSV-based object detector, no license constraint
   - Pure C++, no external dependencies
   - Suitable for simple color-based detection use cases

2. **Obtain a [commercial Ultralytics license](https://ultralytics.com/license)**
   - Exempts you from AGPL 3.0 requirements
   - Recommended for commercial drone products

3. **Substitute a permissively-licensed model**
   - Use YOLO-World, Roboflow-hosted models, or train your own on your data
   - Ensure license compatibility with BSD 3-Clause

4. **Open-source deployment**
   - If your product is open-source, AGPL 3.0 is compatible (you already disclose source)

**Build & Development:**
- The model file is `.gitignore`'d and downloaded on demand — it is **not distributed** with this source code
- During development with simulated backends (`HAS_OPENCV=OFF`), YOLOv8 is completely optional
- CI tests validate both paths (with and without OpenCV/YOLOv8)

**Recommendation:** For new deployments, use `ColorContourDetector` or a permissively-licensed alternative to avoid AGPL licensing obligations.
