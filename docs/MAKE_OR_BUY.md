# Make-or-Buy Decision Table

**Date:** 2026-03-18
**Scope:** All major software components in the companion software stack
**Target hardware:** NVIDIA Jetson Orin (Nano/NX/AGX)

---

## Executive Summary

The stack is currently 100% "make" — all algorithms are original implementations with open-source library dependencies. The only license risk is YOLOv8n (AGPL 3.0, optional). Most components have no compelling reason to buy. The two areas where commercial or open-source alternatives offer meaningful value over the current implementations are **Object Detection** (license risk) and **VIO** (when moving beyond IMU-only pre-integration to full visual-inertial odometry).

| Component | Verdict | Rationale |
|-----------|---------|-----------|
| Object Detection | **Switch** | AGPL 3.0 risk — replace with Apache 2.0 model |
| Multi-Object Tracking | **Keep** | Original SORT+ByteTrack is competitive, no license risk |
| VIO / SLAM | **Evaluate** | Current IMU-only; full VIO needed for GPS-denied flight |
| Path Planning | **Keep** | A*/D* Lite well-suited for drone use, no license risk |
| FC Communication | **Keep** | MAVSDK is the right tool (BSD 3-Clause) |
| IPC Middleware | **Keep** | Zenoh outperforms alternatives, permissive license |
| Sensor Fusion | **Keep** | Original UKF is appropriate, GTSAM (BSD) if needed later |
| Simulation | **Keep** | Gazebo + PX4 SITL is the PX4 ecosystem standard |
| System Monitoring | **Keep** | Three-layer watchdog is purpose-built, no equivalent to buy |

---

## 1. Object Detection / Inference

**Current:** YOLOv8n ONNX via OpenCV DNN — **AGPL 3.0 (license risk)**

### Options

| Option | License | Annual Cost | Jetson Orin Performance | Pros | Cons |
|--------|---------|-------------|------------------------|------|------|
| **RF-DETR Nano-Large** (Roboflow) | Apache 2.0 | $0 | ~20-30 FPS (TensorRT FP16) | SOTA accuracy (60 mAP COCO on XL), ICLR 2026, permissive license, actively developed | Newer/less battle-tested than YOLO, transformer-based (higher VRAM) |
| **RT-DETR** (Baidu) | Apache 2.0 | $0 | ~50 FPS (optimized variants on Orin Nano) | Proven on aerial datasets (VisDrone), CVPR 2024, NMS-free | PaddlePaddle ecosystem less familiar |
| **Ultralytics YOLO11 Enterprise** | Commercial | ~$5,000/yr | ~60+ FPS (TensorRT INT8) | Best ecosystem, most mature, highest FPS | Ongoing cost, AGPL if license lapses |
| **NVIDIA TAO Toolkit** | Free | $0 | Excellent (native TensorRT) | 100+ pre-trained models, fine-tuning, INT8 quantization, free on Jetson | Framework, not a model — must select/train architecture |
| **Custom TensorRT pipeline** | N/A | 2-4 person-months eng | Optimal | Full control, no license risk | Upfront engineering investment |
| **RF-DETR XL/2XL** | PML 1.0 (proprietary) | Contact Roboflow | ~15-20 FPS | Highest accuracy (60.1 mAP) | Commercial license required for XL/2XL |
| **EfficientDet** (Google) | Apache 2.0 | $0 | Moderate | Proven, TFLite support | Older architecture, lower accuracy |

### Hardware-based alternatives

| Option | License | Unit Cost | Notes |
|--------|---------|-----------|-------|
| **Hailo-8 M.2** | Free SDK | ~$180/module | 26 TOPS at 2.5W — very drone-friendly, but requires rewriting detection pipeline for Hailo runtime |
| **Hailo-10H** | Free SDK | ~$130 (in HAT+ bundle) | 40+ TOPS, newer generation |
| **Luxonis OAK-D S2** | Free SDK (DepthAI) | ~$280 | Integrated camera+VPU+depth, 30 FPS detection, but less powerful than Orin GPU |

### Recommendation

**Switch to RF-DETR (Apache 2.0, Nano-Large) exported to TensorRT.** This eliminates the AGPL risk at zero cost. If RF-DETR proves too heavy for your Jetson variant, RT-DETR (Apache 2.0) is the fallback. If maximum FPS matters more than license cost, budget ~$5K/yr for Ultralytics Enterprise.

---

## 2. Multi-Object Tracking

**Current:** SORT + ByteTrack — **original C++ implementations, no license risk**

### Options

| Option | License | Annual Cost | Pros | Cons |
|--------|---------|-------------|------|------|
| **Keep current** | Project-original | $0 | Full control, C++ native, no dependency, already working and tested | No re-ID capability |
| **NVIDIA DeepStream NvDCF/NvSORT** | Free (with DeepStream) | $0 | GPU-accelerated, production-grade, multi-stream | Tied to GStreamer/DeepStream pipeline, major architecture change |
| **Norfair** (Tryolabs) | BSD 3-Clause | $0 | Lightweight, detector-agnostic | Python-only, not suitable for C++ embedded |
| **BoT-SORT** | Academic (check repo) | $0 | Higher accuracy, camera motion compensation | License unclear, Python reference implementation |
| **Ultralytics Tracking** | AGPL 3.0 / Enterprise | ~$5K/yr | Integrated with YOLO | Same AGPL problem |

### Recommendation

**Keep current.** Your original SORT+ByteTrack implementations are competitive, lightweight, C++ native, and carry zero license risk. ByteTrack remains one of the strongest tracking algorithms. No compelling reason to buy.

---

## 3. Visual-Inertial Odometry (VIO) / SLAM

**Current:** Original IMU pre-integration only — **no visual component yet**

### Open-source options (permissive license)

| Option | License | Cost | Jetson Orin | Accuracy (EuRoC) | Notes |
|--------|---------|------|-------------|-------------------|-------|
| **Basalt VIO** (TUM) | BSD 3-Clause | $0 | 300 Hz claimed | 0.012m ATE (best OSS) | Best open-source VIO for Jetson, permissive license, actively maintained |
| **Kimera-VIO** (MIT SPARK) | BSD 3-Clause | $0 | Tested on older Jetsons | Good, not top-tier | 3D mesh generation, modular, research-grade |
| **NVIDIA cuVSLAM** | NVIDIA Community License | $0 dev, production unclear | 232 FPS @ 720p (best) | <5 cm position error | GPU-accelerated, unmatched performance, but proprietary binary |

### Open-source options (GPL — incompatible with BSD 3-Clause project)

| Option | License | Cost | Notes |
|--------|---------|------|-------|
| **ORB-SLAM3** | GPL 3.0 (commercial available) | Contact Univ. Zaragoza | Gold standard accuracy, commercial license possible but opaque pricing |
| **OpenVINS** | GPL 3.0 only | $0 | Excellent monocular VIO, no commercial license option |
| **VINS-Fusion** | GPL 3.0 | $0 | Mono/stereo+IMU, GPS fusion, proven on drones |

### Commercial SDKs

| Option | License | Annual Cost | Jetson Orin | Notes |
|--------|---------|-------------|-------------|-------|
| **Spleenlab visonAIry** | Proprietary | Custom (contact sales) | Xavier NX confirmed, Orin likely | Top drone customers: Quantum Systems, DroneUp, Aerialoop. Auterion/PX4 ecosystem partner |
| **SLAMcore** | Proprietary | Per-seat + per-robot/month (undisclosed) | Orin since v23.04 | RealSense D435i/D455 out-of-box, warehouse/consumer focus |
| **Kudan SLAM** | Proprietary | GBP 1,000/app (AR); enterprise custom | Orin (NVIDIA partner) | LiDAR + visual SLAM, Terra Drone customer |
| **Spectacular AI** | Proprietary | Custom (undisclosed) | ARM with commercial license | OAK-D, RealSense, free non-commercial x86 tier |
| **ZED SDK** (Stereolabs) | Free with camera | $549-685 per camera | Full support | Built-in VIO + SLAM, but locked to ZED cameras |

### Hardware-locked options (not compatible with Jetson architecture)

| Option | Cost | Notes |
|--------|------|-------|
| **ModalAI VOXL VIO** | Bundled with $2,950+ dev kits | Qualcomm QRB5165 only — cannot run on Jetson |

### Recommendation

**Evaluate Basalt VIO (BSD 3-Clause) as the primary candidate** for your VIO HAL backend. It offers the best combination of performance (300 Hz on Orin), accuracy (0.012m ATE), and commercial-friendly licensing. cuVSLAM is the performance king but the proprietary license is a risk. If you need commercial support, Spleenlab has the strongest drone references but pricing is opaque.

---

## 4. Path Planning

**Current:** A* + D* Lite — **original implementations, no license risk**

### Options

| Option | License | Annual Cost | Pros | Cons |
|--------|---------|-------------|------|------|
| **Keep current** | Project-original | $0 | Lightweight, drone-optimized, 3D grid-based, works | No sampling-based planners |
| **OMPL** (Rice University) | BSD 3-Clause (core) | $0 | Extensive planners (RRT*, PRM, etc.), well-tested | Designed for manipulation, needs aerial adaptation |
| **Nav2** (ROS 2) | Apache 2.0 | $0 | Full navigation stack, Smac planner, behavior trees | Heavy ROS 2 dependency, ground-robot focused |
| **Auterion SDK** | Commercial | ~$1,500/unit (Skynode hardware) + Suite SaaS | Full drone autonomy, PX4-native, defense-grade | Vendor lock-in, requires Skynode hardware |
| **Skydio Autonomy** | Commercial | $1,499/drone/yr | Combat-proven autonomy | Skydio hardware only, not sold as standalone SDK |

### Recommendation

**Keep current.** A*/D* Lite are well-suited for 3D drone path planning. If you need sampling-based planners for complex environments, add **OMPL** (BSD 3-Clause core) as a library — don't take on the full ROS 2 dependency. No commercial option makes sense unless you're buying the full Auterion/Skynode platform.

---

## 5. Flight Controller Communication

**Current:** MAVSDK — **BSD 3-Clause, no license risk**

### Options

| Option | License | Annual Cost | Pros | Cons |
|--------|---------|-------------|------|------|
| **MAVSDK (keep)** | BSD 3-Clause | $0 | PX4-recommended, clean C++ API, cross-platform | Limited ArduPilot support |
| **MAVROS** | BSD 3-Clause | $0 | Full MAVLink bridge, PX4+ArduPilot | Requires ROS, heavier |
| **pymavlink / custom** | LGPL 3.0 | $0 | Maximum control | Significant engineering, LGPL concerns |
| **DJI MSDK** | Free (DJI developer program) | $0 | Deep DJI integration | DJI hardware only, NDAA concerns |

### Recommendation

**Keep MAVSDK.** It is the right tool — BSD-licensed, PX4-recommended, clean C++ API, no ROS dependency.

---

## 6. IPC / Middleware

**Current:** Zenoh — **EPL 2.0 / Apache 2.0, no license risk**

### Options

| Option | License | Annual Cost | Latency | Pros | Cons |
|--------|---------|-------------|---------|------|------|
| **Zenoh (keep)** | EPL 2.0 / Apache 2.0 | $0 (commercial support: contact ZettaScale) | ~15 us | Best latency+throughput, zero-copy, cloud-to-edge | Smaller community than ROS 2 |
| **RTI Connext DDS** | Commercial | ~EUR 1,000-8,000/dev/yr (no runtime royalty) | ~50-100 us | DO-178C/ISO 26262 certified, industry standard | Expensive, complex, overkill for prototype |
| **eProsima Fast-DDS** | Apache 2.0 (Pro: commercial) | $0 (Pro: contact sales) | ~50-100 us | ROS 2 default, permissive license | Higher latency than Zenoh |
| **Eclipse iceoryx** | Apache 2.0 | $0 | ~1 us (shared memory) | True zero-copy, ultra-low latency | Single-machine only, no network |
| **eCAL** (Continental) | Apache 2.0 | $0 | Low | Automotive-grade, good tooling | Less drone ecosystem integration |
| **ZeroMQ** | MPL 2.0 | $0 | Low | Mature, lightweight | No pub/sub discovery, manual serialization |

### Recommendation

**Keep Zenoh.** It outperforms DDS alternatives in latency and throughput, works across network topologies, and the Apache 2.0 side of the dual license is fully permissive. The only reason to switch would be if you needed safety certification (RTI Connext DDS at ~EUR 4,500/dev/yr) or ROS 2 ecosystem integration.

---

## 7. Sensor Fusion

**Current:** Original UKF — **no license risk**

### Options

| Option | License | Annual Cost | Pros | Cons |
|--------|---------|-------------|------|------|
| **Keep current** | Project-original | $0 | Lightweight, tailored state vector, real-time | No graph optimization |
| **GTSAM** (Georgia Tech) | BSD 3-Clause | $0 | Factor graph optimization, SLAM backend, well-maintained | Heavier, learning curve |
| **robot_localization** (ROS 2) | BSD 3-Clause | $0 | EKF/UKF, sensor fusion out-of-box | ROS 2 dependency |
| **Ceres Solver** (Google) | BSD 3-Clause | $0 | General nonlinear least squares | Batch optimizer, not real-time streaming |
| **SBG Systems INS** | Hardware-bundled | $2K-20K (hardware) | Calibrated, validated, turnkey | Locked to SBG IMU hardware |

### Recommendation

**Keep current UKF** for real-time IMU+vision fusion. If you later need graph-based optimization (loop closure, multi-sensor batch optimization), add **GTSAM** (BSD 3-Clause). No need to buy commercial hardware-bundled fusion.

---

## 8. Simulation

**Current:** Gazebo Harmonic + PX4 SITL — **Apache 2.0 + BSD 3-Clause, no cost**

### Options

| Option | License | Annual Cost | Pros | Cons |
|--------|---------|-------------|------|------|
| **Gazebo + PX4 SITL (keep)** | Apache 2.0 + BSD 3-Clause | $0 | PX4-native, full SITL, good sensor plugins | Graphics fidelity lower than Unreal |
| **NVIDIA Isaac Sim** | Free dev; Omniverse Enterprise $4,500/GPU/yr production | $0-$4,500/yr | Photorealistic, synthetic data gen, domain randomization | GPU-heavy, not drone-focused, ROS 2 needed |
| **Unity Pro** | $2,200/seat/yr | $2,200/yr | Good graphics, C# scripting, ML-Agents | No native PX4 integration |
| **Unreal + AirSim fork** | Free (<$1M rev) | $0 | Best graphics fidelity | AirSim archived/unmaintained since 2022 |
| **jMAVSim** | BSD | $0 | Lightweight, PX4-bundled, fast startup | Very basic, limited sensors |
| **AnyLogic** | ~$12K-19K/yr | ~$15K/yr | Fleet logistics simulation | Not a flight dynamics sim |

### Recommendation

**Keep Gazebo + PX4 SITL.** It has the tightest PX4 integration, is free, and is the ecosystem standard. Isaac Sim is worth evaluating as a **complement** (for photorealistic synthetic training data), not a replacement.

---

## 9. System Monitoring / Fleet Management

**Current:** Original ProcessManager + systemd — **no license risk**

### Single-vehicle monitoring (what you have)

| Option | License | Annual Cost | Notes |
|--------|---------|-------------|-------|
| **Keep current** | Project-original | $0 | Three-layer watchdog (thread/process/OS), purpose-built |
| **Prometheus + Grafana** | Apache 2.0 / AGPL | $0 self-hosted | Industry-standard, but integration work needed |

### Fleet management (future need)

| Option | License | Annual Cost | Notes |
|--------|---------|-------------|-------|
| **Auterion Suite** | Commercial SaaS | Free tier available; paid per-vehicle/yr (see [pricing PDF](https://auterion.com/wp-content/uploads/2024/09/Auterion-Suite-Pricing.pdf)) | Requires AuterionOS, PX4-native |
| **FlytBase Pro** | Commercial SaaS | $99/mo or $999/yr per org | Hardware-agnostic, DJI Dock + custom platforms |
| **DJI FlightHub 2** | Commercial | $999/yr (Pro) or ~$3,280/device/yr (Enterprise) | DJI hardware only |
| **Aloft** (fmr. Kittyhawk) | Commercial | Contact sales | Airspace compliance focus, acquired by Versaterm (2026) |

### Recommendation

**Keep current** for single-vehicle process supervision. When fleet management is needed, evaluate **FlytBase** ($999/yr, hardware-agnostic) or **Auterion Suite** (if using AuterionOS/Skynode).

---

## Cost Summary: Full Stack Scenarios

### Scenario A: Fully open-source (current path + fixes)

| Component | Solution | Annual Cost |
|-----------|----------|-------------|
| Detection | RF-DETR (Apache 2.0) + TensorRT | $0 |
| Tracking | Original SORT+ByteTrack | $0 |
| VIO | Basalt VIO (BSD 3-Clause) | $0 |
| Planning | Original A*/D* Lite | $0 |
| FC Comms | MAVSDK (BSD 3-Clause) | $0 |
| IPC | Zenoh (Apache 2.0) | $0 |
| Fusion | Original UKF | $0 |
| Simulation | Gazebo + PX4 SITL | $0 |
| Monitoring | Original + systemd | $0 |
| **Total** | | **$0/yr** |

### Scenario B: Best-of-breed commercial (maximum capability)

| Component | Solution | Annual Cost |
|-----------|----------|-------------|
| Detection | Ultralytics YOLO11 Enterprise | ~$5,000 |
| Tracking | NVIDIA DeepStream NvSORT | $0 |
| VIO | Spleenlab visonAIry or cuVSLAM | ~$10,000-50,000 (est.) |
| Planning | Auterion SDK + Skynode | ~$5,000-20,000 |
| FC Comms | MAVSDK | $0 |
| IPC | RTI Connext DDS (certified) | ~$4,500-8,000 |
| Fusion | GTSAM (BSD) | $0 |
| Simulation | Gazebo + Isaac Sim | ~$4,500 |
| Monitoring | Auterion Suite | ~$2,000-5,000 |
| **Total** | | **~$31,000-88,000/yr** |

### Scenario C: Pragmatic hybrid (recommended)

| Component | Solution | Annual Cost |
|-----------|----------|-------------|
| Detection | RF-DETR (Apache 2.0) + TensorRT | $0 |
| Tracking | Original SORT+ByteTrack | $0 |
| VIO | Basalt VIO (BSD) or cuVSLAM (free on Jetson dev) | $0 |
| Planning | Original A*/D* Lite + OMPL (BSD) if needed | $0 |
| FC Comms | MAVSDK (BSD) | $0 |
| IPC | Zenoh (Apache 2.0) | $0 |
| Fusion | Original UKF + GTSAM (BSD) if needed | $0 |
| Simulation | Gazebo + PX4 SITL | $0 |
| Monitoring | Original + systemd | $0 |
| Fleet mgmt | FlytBase Pro (when needed) | $999 |
| **Total** | | **$0-999/yr** |

---

## Priority Actions

1. **Immediate:** Replace YOLOv8n with RF-DETR or RT-DETR (Apache 2.0) — eliminates sole license risk
2. **Issue #169:** Evaluate Basalt VIO (BSD) as the VIO HAL backend for GPS-denied flight
3. **Future:** Add OMPL (BSD) for sampling-based planning if complex 3D environments require it
4. **Future:** Evaluate FlytBase or Auterion Suite when fleet management is needed

---

*Last updated: 2026-03-18*
