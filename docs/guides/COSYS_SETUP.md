# Cosys-AirSim Setup Guide

Comprehensive setup guide for Tier 3 photorealistic simulation using Cosys-AirSim. This enables ML perception validation (YOLO + Depth Anything V2) and native radar simulation that Gazebo (Tier 2) cannot provide.

**See also:** [ADR-011](../adr/ADR-011-cosys-airsim-photorealistic-simulation.md) for architecture decisions and rationale.

---

## Overview

### Simulation Tiers

| | Tier 1 -- Software | Tier 2 -- Physics SITL | Tier 3 -- Photorealistic SITL |
|---|---|---|---|
| **Simulator** | None (simulated backends) | Gazebo Harmonic | Cosys-AirSim (UE5) |
| **Launcher** | `deploy/launch_all.sh` | `deploy/launch_gazebo.sh` | `docker compose -f docker/docker-compose.cosys.yml up` |
| **Config** | `config/default.json` | `config/gazebo_sitl.json` | `config/cosys_airsim.json` (cloud) / `config/cosys_airsim_dev.json` (dev) |
| **GPU required** | No | Yes (any) | Yes (8+ GB VRAM) |
| **Validates** | IPC, FSM, config, faults | Nav, planning, physics | ML perception, depth, radar fusion |
| **ML models** | Not exercised | Not meaningful (domain gap) | Full pipeline (YOLO + DA V2) |

### Two Deployment Profiles

**Dev (native desktop)** -- Run Cosys-AirSim directly on a local machine with a GTX 1080 Ti or better. UE5 renders locally, the companion stack runs natively. Best for rapid iteration on perception code.

**Cloud (Docker on AWS)** -- Two Docker containers on an AWS g5.xlarge instance (A10G 24 GB VRAM). Larger ML models (YOLOv8m, DA V2 ViT-B) and full 1080p rendering. Best for high-fidelity perception validation and CI.

### Why Tier 3?

Gazebo validates flight dynamics, IPC wiring, FSM logic, and nav/planning -- it does this well and continues to. But Gazebo renders flat-shaded geometric primitives that ML models (YOLO, Depth Anything V2) cannot meaningfully process. Cosys-AirSim provides photorealistic UE5 rendering, native PX4 SITL integration, and a native radar sensor model -- the three capabilities needed to validate the ML perception pipeline.

---

## Quick Start -- Dev Machine

### Prerequisites

- **OS:** Ubuntu 22.04 or 24.04 (x86_64)
- **GPU:** NVIDIA GTX 1080 Ti or newer (8+ GB VRAM, 11+ GB recommended)
- **CUDA toolkit:** 11.8+ (for ML inference only -- UE5 uses Vulkan for rendering)
- **Disk:** ~50 GB free (UE5 assets are large)
- **RAM:** 16 GB minimum, 32 GB recommended

### Step 1: Install NVIDIA Drivers and CUDA

If not already installed:

```bash
# Verify GPU is detected
nvidia-smi

# If nvidia-smi is not found, install drivers:
sudo apt-get install -y nvidia-driver-535   # or latest stable
sudo reboot

# Install CUDA toolkit (for ML inference)
# See: https://developer.nvidia.com/cuda-downloads
```

### Step 2: Initialize the AirSim Submodule

The AirSim C++ SDK is vendored as a git submodule. CMake builds it automatically via `FindAirSim.cmake`, but the submodule must be initialized first:

```bash
cd companion_software_stack
git submodule update --init third_party/cosys-airsim
```

This downloads the Cosys-AirSim SDK (~200 MB). The CMake build will also automatically download rpclib v2.3.1 (the RPC transport library) on first configure.

### Step 3: Build with Dev Model Preset

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
         -DALLOW_INSECURE_ZENOH=ON \
         -DMODEL_PRESET=dev
make -j$(nproc)
```

The CMake summary should show:

```
  Cosys-AirSim : 5.5-v3.3 -- Cosys-AirSim HAL backends available (ExternalProject)
  Model Preset : dev
  YOLO Model   : yolov8s.onnx
  Depth Model  : depth_anything_v2_vits.onnx
```

If Cosys-AirSim shows `NOT FOUND`, verify the submodule was initialized (Step 2).

### Step 4: Install and Run Cosys-AirSim (UE5)

Follow the [Cosys-AirSim installation guide](https://github.com/Cosys-Lab/Cosys-AirSim) for your platform. The typical workflow:

1. Download a pre-built UE5 environment (e.g., `AirSimNH`, `Blocks`, or a custom environment)
2. Place `settings.json` in `~/Documents/AirSim/settings.json` (or the platform-specific location)
3. Launch the UE5 environment binary

```bash
# Example: run a pre-built environment
cd ~/AirSimNH/
./AirSimNH.sh -ResX=1280 -ResY=720 -windowed
```

The AirSim RPC server starts automatically on port 41451.

### Step 5: Launch the Companion Stack

Once UE5 is running and the RPC server is available:

```bash
cd companion_software_stack

# Verify RPC server is reachable
timeout 3 bash -c '</dev/tcp/127.0.0.1/41451' && echo "RPC OK" || echo "RPC not available"

# Launch all 7 processes with the dev config overlay
# (launch script does not exist yet -- use manual launch)
CONFIG_FILE=config/cosys_airsim_dev.json
for proc in system_monitor video_capture comms perception slam_vio_nav mission_planner payload_manager; do
    ./build/bin/${proc} --config ${CONFIG_FILE} &
done
```

### Step 6: Download ML Models

The dev profile uses YOLOv8s and Depth Anything V2 ViT-S:

```bash
# YOLOv8s (22 MB)
mkdir -p models
wget -O models/yolov8s.onnx \
    https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s.onnx

# Depth Anything V2 ViT-S (~95 MB) -- requires export script
python3 -m venv .venv
source .venv/bin/activate
pip install torch transformers onnx onnxruntime onnxsim
bash models/download_depth_anything_v2.sh
deactivate
```

See [INSTALL.md Section 4](INSTALL.md#4-opencv-410-optional--yolov8-detection) for details on model export.

---

## Quick Start -- Cloud (AWS)

### Prerequisites

- **AWS CLI** configured with appropriate credentials
- **Docker** and **docker compose** (v2) installed locally
- **SSH key** for the EC2 instance

### Step 1: Launch an AWS Instance

**Recommended instance:** `g5.xlarge` -- A10G GPU with 24 GB VRAM, 4 vCPU, 16 GB RAM.

| Pricing | Rate |
|---------|------|
| On-demand | ~$1.01/hr |
| Spot | ~$0.35/hr |

Estimated monthly cost for typical use (80 hrs): ~$28/month on spot instances.

```bash
# Launch a spot instance (example -- adjust AMI, subnet, security group)
aws ec2 run-instances \
    --image-id ami-0abcdef1234567890 \
    --instance-type g5.xlarge \
    --key-name your-key \
    --instance-market-options '{"MarketType":"spot","SpotOptions":{"MaxPrice":"0.50"}}' \
    --block-device-mappings '[{"DeviceName":"/dev/sda1","Ebs":{"VolumeSize":100,"VolumeType":"gp3"}}]' \
    --tag-specifications 'ResourceType=instance,Tags=[{Key=Name,Value=cosys-airsim}]'
```

The instance needs:
- **NVIDIA drivers** (use the Deep Learning AMI, which comes pre-configured)
- **NVIDIA Container Toolkit** (`nvidia-ctk`) for GPU passthrough to Docker
- **Docker** and **docker compose** (v2)
- **100+ GB disk** (UE5 Docker image is ~15-20 GB)

### Step 2: Clone and Configure

```bash
ssh -i your-key.pem ubuntu@<instance-ip>

git clone https://github.com/nmohamaya/companion_software_stack.git
cd companion_software_stack
git submodule update --init third_party/cosys-airsim
```

### Step 3: Launch with Docker Compose

```bash
docker compose -f docker/docker-compose.cosys.yml up --build
```

This starts two containers:

1. **cosys-airsim** -- UE5 headless rendering + PX4 SITL + AirSim RPC server
2. **companion-stack** -- 7-process drone stack with `MODEL_PRESET=cloud`

The compose file includes a health check that waits up to 120 seconds for UE5 to load assets before starting the companion stack. The companion stack container starts only after the AirSim RPC server is healthy.

### Step 4: Monitor and Collect Artifacts

```bash
# Follow logs
docker compose -f docker/docker-compose.cosys.yml logs -f

# Check GPU memory usage
docker exec -it companion-stack nvidia-smi

# Copy artifacts (logs, metrics) to local machine
docker compose -f docker/docker-compose.cosys.yml cp companion-stack:/app/drone_logs ./results/
```

### Step 5: Shut Down

```bash
docker compose -f docker/docker-compose.cosys.yml down

# Terminate the EC2 instance when done (stop spot charges)
aws ec2 terminate-instances --instance-ids <instance-id>
```

---

## VRAM Budget Tables

### Dev Profile (GTX 1080 Ti, 11 GB VRAM)

| Component | VRAM Usage | Notes |
|-----------|-----------|-------|
| UE5 rendering (Vulkan) | ~8 GB | Rasterization mode (no Nanite/Lumen on Pascal) |
| YOLOv8s (11.2M params) | ~1.5 GB | CUDA inference via OpenCV DNN |
| DA V2 ViT-S (25M params) | ~0.5 GB | CUDA inference via OpenCV DNN |
| System overhead | ~0.5 GB | Driver, display server |
| **Total** | **~10.5 GB** | Fits within 11 GB with ~0.5 GB headroom |

> **Pascal GPUs (GTX 1080 Ti):** UE5's Nanite virtualized geometry and Lumen global illumination require hardware mesh shaders (Turing+). On Pascal, UE5 falls back to traditional rasterization. Visual quality is lower but functional for ML validation. Resolution is capped at 1280x720 to stay within VRAM budget.

### Cloud Profile (A10G, 24 GB VRAM)

| Component | VRAM Usage | Notes |
|-----------|-----------|-------|
| UE5 rendering (Vulkan) | ~10 GB | Full Nanite + Lumen on Ampere |
| YOLOv8m (25.9M params) | ~3 GB | Larger model, higher mAP |
| DA V2 ViT-B (97M params) | ~1.5 GB | Larger model, better depth quality |
| System overhead | ~0.5 GB | Container runtime |
| **Total** | **~15 GB** | Fits within 24 GB with ~9 GB headroom |

---

## Profile Comparison

| Attribute | Dev (native) | Cloud (Docker) |
|-----------|-------------|----------------|
| **Config file** | `config/cosys_airsim_dev.json` | `config/cosys_airsim.json` |
| **CMake flag** | `-DMODEL_PRESET=dev` | `-DMODEL_PRESET=cloud` |
| **YOLO model** | YOLOv8s (11.2M params, ~42 mAP) | YOLOv8m (25.9M params, ~50 mAP) |
| **Depth model** | DA V2 ViT-S (25M params) | DA V2 ViT-B (97M params) |
| **Resolution** | 1280x720 | 1920x1080 |
| **UE5 rendering** | Rasterization (Pascal) | Nanite + Lumen (Ampere) |
| **Radar backend** | `cosys_airsim` | `cosys_airsim` |
| **PX4 connection** | `simulated` (local dev) | `mavlink` (tcp://cosys-airsim:14540) |
| **Deployment** | Native processes | Docker Compose (2 containers) |
| **GPU minimum** | GTX 1080 Ti (11 GB) | A10G (24 GB) |
| **Typical cost** | Electricity only | ~$0.35/hr (spot) |

---

## GPU Rendering vs CUDA Compute

A common source of confusion: UE5 and the ML inference pipeline use the GPU differently.

**UE5 rendering uses Vulkan** -- the Unreal Engine graphics pipeline runs entirely through the Vulkan graphics API. It does not require the CUDA toolkit. The GPU's shader cores handle vertex processing, rasterization, fragment shading, and (on Turing+) Nanite mesh shading. This is the same rendering path used by games.

**ML inference uses CUDA** -- YOLOv8 and Depth Anything V2 run through OpenCV's DNN module with the CUDA backend. This uses the GPU's CUDA cores for tensor operations (matrix multiplies, convolutions). CUDA requires the NVIDIA CUDA toolkit (11.8+) and cuDNN (8.6+).

**Both share VRAM but not compute pipelines.** The GPU time-shares between Vulkan rendering and CUDA compute automatically. VRAM is the binding constraint -- both workloads must fit in GPU memory simultaneously. The VRAM budget tables above account for this.

**Implication:** You can run UE5 without CUDA installed (for visual inspection), but you need CUDA for the ML inference pipeline to use GPU acceleration. Without CUDA, OpenCV DNN falls back to CPU inference, which is significantly slower (~2 FPS vs ~15 FPS for YOLOv8s).

---

## Known Limitations

### Pascal GPUs (GTX 1080 Ti, GTX 1080)

- **No Nanite or Lumen:** UE5's advanced rendering features require hardware mesh shaders (Turing architecture, RTX 20-series and newer). Pascal GPUs fall back to traditional rasterization with static global illumination. Visual quality is noticeably lower than Ampere/Turing but still far above Gazebo.
- **Resolution cap:** The dev config uses 1280x720 to stay within the VRAM budget. Higher resolutions risk OOM crashes.
- **No ray-traced shadows:** Shadow quality comes from shadow maps only. This can affect depth estimation accuracy in certain lighting conditions.

### General

- **UE5 startup time:** 60-90 seconds for asset loading (vs ~5 seconds for Gazebo). Docker health check accounts for this with a 120-second `start_period`.
- **Docker image size:** The Cosys-AirSim Docker image is ~15-20 GB. First pull takes significant time on slow connections.
- **Cosys-AirSim maintenance:** Maintained by Cosys-Lab (University of Antwerp). MIT license allows forking if needed. Our HAL abstraction layer means swapping the simulator only requires new HAL backends.
- **VIO backend:** The dev config currently uses the simulated VIO backend. PX4 EKF2 pose via `MavlinkFCLink` is used in cloud mode. A native Cosys-AirSim VIO backend is planned but not yet implemented.
- **Spot instance interruptions:** AWS spot instances can be reclaimed with 2 minutes notice. Use checkpoint/retry logic for long-running test suites.

---

## Troubleshooting

### Common Errors

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Cosys-AirSim : NOT FOUND` in CMake | AirSim submodule not initialized | `git submodule update --init third_party/cosys-airsim` |
| `rpclib download failed` during CMake | Network issue or firewall blocking GitHub | Download rpclib v2.3.1 manually from GitHub and place in `third_party/cosys-airsim/external/rpclib/rpclib-2.3.1/` |
| `Connection refused` on port 41451 | AirSim RPC server not running | Start UE5 environment first; verify with `timeout 3 bash -c '</dev/tcp/HOST/41451'` |
| `CUDA error: out of memory` | VRAM budget exceeded | Close other GPU applications; reduce resolution in config; use smaller ML models |
| UE5 crash on startup with `VK_ERROR_DEVICE_LOST` | GPU driver too old or incompatible | Update to NVIDIA driver 535+ (`sudo apt-get install nvidia-driver-535`) |
| UE5 renders black screen (headless) | Missing Vulkan ICD in container | Ensure `NVIDIA_DRIVER_CAPABILITIES=graphics,display` in Docker environment |
| `PX4 connection timeout` (cloud) | companion-stack started before PX4 ready | The compose health check handles this; if manual, wait for `ss -tlnp \| grep 14540` |
| Docker `nvidia` runtime not found | NVIDIA Container Toolkit not installed | Install: `sudo apt-get install nvidia-container-toolkit && sudo systemctl restart docker` |
| Slow ML inference (~2 FPS) | OpenCV DNN using CPU backend | Verify CUDA is available: build OpenCV with `-DWITH_CUDA=ON`. See [INSTALL.md Section 4c](INSTALL.md#option-c-build-from-source-with-cuda-gpu-accelerated-dnn) |
| `ExternalProject` build failure for AirSim | Compiler version mismatch or missing dependencies | Check `build/airsim-build/airsim_external-prefix/src/airsim_external-stamp/` for logs |
| Zenoh SHM errors in Docker | Container lacks shared memory | Add `--shm-size=256m` to `docker run` or use `--ipc=host` |

### Verifying the Full Pipeline

After setup, verify that each component works:

```bash
# 1. AirSim RPC connectivity
timeout 3 bash -c '</dev/tcp/127.0.0.1/41451' && echo "RPC: OK"

# 2. Build detected AirSim SDK
grep "Cosys-AirSim" build/CMakeCache.txt
# Should show: COSYS_AIRSIM_FOUND:INTERNAL=TRUE

# 3. ML models are present
ls -la models/yolov8s.onnx models/depth_anything_v2_vits.onnx

# 4. GPU memory (should show UE5 using ~8 GB)
nvidia-smi

# 5. Run perception unit tests (AirSim backends)
ctest --test-dir build --output-on-failure -R cosys
```

### Checking GPU VRAM Usage

Monitor VRAM to ensure you stay within budget:

```bash
# One-shot
nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv

# Continuous (every 2 seconds)
watch -n 2 nvidia-smi

# In Docker
docker exec companion-stack nvidia-smi
```

If VRAM usage approaches the GPU limit, reduce resolution in the config file or switch to smaller ML models (e.g., `yolov8n.onnx` instead of `yolov8s.onnx`).

---

## Configuration Reference

### Dev Config: `config/cosys_airsim_dev.json`

Key settings for the dev profile (GTX 1080 Ti):

| Setting | Value | Notes |
|---------|-------|-------|
| `cosys_airsim.host` | `127.0.0.1` | Local UE5 instance |
| `cosys_airsim.port` | `41451` | Default AirSim RPC port |
| `video_capture.mission_cam.backend` | `cosys_airsim` | Camera frames from UE5 |
| `video_capture.mission_cam.width` | `1280` | Reduced for VRAM budget |
| `video_capture.mission_cam.height` | `720` | Reduced for VRAM budget |
| `perception.detector.backend` | `color_contour` | Switch to `yolov8` when OpenCV DNN + models available |
| `perception.detector.model_path` | `models/yolov8s.onnx` | YOLOv8s for dev |
| `perception.radar.backend` | `cosys_airsim` | Native AirSim radar |
| `perception.depth_estimator.backend` | `depth_anything_v2` | ML depth (until ground-truth depth backend is wired) |
| `slam.imu.backend` | `cosys_airsim` | AirSim IMU data |
| `comms.mavlink.backend` | `simulated` | No PX4 in dev mode |

### Cloud Config: `config/cosys_airsim.json`

Key differences from dev:

| Setting | Value | Notes |
|---------|-------|-------|
| `cosys_airsim.host` | `cosys-airsim` | Docker service name (DNS resolved by compose network) |
| `video_capture.mission_cam.width` | `1920` | Full 1080p on A10G |
| `video_capture.mission_cam.height` | `1080` | Full 1080p on A10G |
| `perception.detector.backend` | `yolov8` | Full ML detection |
| `perception.detector.model_path` | `models/yolov8m.onnx` | YOLOv8m for cloud |
| `perception.depth_estimator.backend` | `cosys_airsim` | Ground-truth depth from simulator |
| `comms.mavlink.backend` | `mavlink` | Real PX4 SITL via TCP |
| `comms.mavlink.uri` | `tcp://cosys-airsim:14540` | PX4 inside AirSim container |

---

## Architecture Diagram

```
DEV MACHINE (GTX 1080 Ti, 11 GB)
+-------------------------------------------+
|  UE5 (Vulkan rendering)     ~8 GB VRAM    |
|    +-- AirSim RPC server (port 41451)     |
|    +-- Sensors: camera, radar, IMU, depth |
+-------------------------------------------+
         |  RPC (msgpack, localhost)
+-------------------------------------------+
|  Companion Stack (7 processes)            |
|    +-- P1: CosysCameraBackend             |
|    +-- P2: YOLOv8s + DA V2 ViT-S  ~2 GB  |
|    +-- P3-P7: planning, comms, etc.       |
|    +-- Zenoh IPC (SHM)                    |
+-------------------------------------------+


CLOUD (AWS g5.xlarge, A10G 24 GB)
+-------------------------------------------+
|  Container 1: cosys-airsim                |
|    +-- UE5 headless (Vulkan)   ~10 GB     |
|    +-- PX4 SITL (MAVLink tcp:14540)       |
|    +-- AirSim RPC (port 41451)            |
+-------------------------------------------+
         |  Docker bridge network
+-------------------------------------------+
|  Container 2: companion-stack             |
|    +-- 7 processes (P1-P7)                |
|    +-- YOLOv8m + DA V2 ViT-B   ~4.5 GB   |
|    +-- MavlinkFCLink (real PX4)           |
|    +-- Zenoh IPC (SHM within container)   |
+-------------------------------------------+
```

---

## Licensing Compliance

### Unreal Engine 5 EULA

UE5 is licensed under the [Unreal Engine End User License Agreement](https://www.unrealengine.com/eula). Our use case (internal SITL testing, not a shipped product) is well within the permissive zone:

- **Internal use is unrestricted.** Running UE5 on owned or rented hardware (including AWS EC2) for testing is allowed. The EULA does not distinguish between owned and cloud infrastructure.
- **No royalties apply.** We use UE5 as an internal development/testing tool. We do not distribute a product containing UE5 to customers. Even if we did, the $1M revenue threshold and the "non-interactive simulation" exemption would apply.
- **Docker images must stay in a private registry.** UE5 binaries are classified as "Engine Tools" under EULA Section 1A. Engine Tools may be shared with team members via **private channels only** (e.g., AWS ECR). **Never push a UE5-containing Docker image to a public registry** (Docker Hub, GitHub Container Registry public, etc.).
- **Each team member must be an Engine Licensee.** Anyone pulling or using a Docker image containing UE5 must have an [Epic Games account](https://www.epicgames.com/id/login) linked to their [GitHub account](https://www.unrealengine.com/ue-on-github). This is free — it grants access to the official UE5 base images at `ghcr.io/epicgames/unreal-engine`.

### Cosys-AirSim License

Cosys-AirSim is licensed under the **MIT License** (inherited from Microsoft AirSim, via Codex Laboratories and University of Antwerp). Fully permissive — no additional restrictions on use, modification, or distribution. See the [LICENSE file](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE).

### Compliance Checklist

Before deploying to cloud or sharing Docker images:

- [ ] All team members have linked Epic Games + GitHub accounts
- [ ] Docker images containing UE5 are stored in **private** registries only (e.g., AWS ECR)
- [ ] UE5-containing images are **never** pushed to public Docker Hub or public GHCR
- [ ] `.dockerignore` excludes UE5 source code from any publicly distributed artifacts
- [ ] Cloud instances are terminated after use (cost and security hygiene)

### Industry Precedent

This deployment pattern (UE5 in Docker on cloud GPUs for internal simulation) is well-established:

- **CARLA** (autonomous driving simulator) uses the same setup, validated by Waymo, Cruise, and others
- **AWS + Epic Games** have an official partnership with reference architectures for UE5 on EC2
- **unrealcontainers.com** provides community tooling and EULA compliance documentation

---

## Related Documentation

- [ADR-011: Cosys-AirSim Photorealistic Simulation](../adr/ADR-011-cosys-airsim-photorealistic-simulation.md) -- Architecture decision record
- [INSTALL.md](INSTALL.md) -- Dependency installation (OpenCV, CUDA, MAVSDK)
- [GETTING_STARTED.md](GETTING_STARTED.md) -- First-time setup (Tier 1 and Tier 2)
- [DEVELOPMENT_WORKFLOW.md](DEVELOPMENT_WORKFLOW.md) -- Branch conventions and CI workflow
- `cmake/FindAirSim.cmake` -- AirSim SDK build integration
- `cmake/ModelPresets.cmake` -- ML model preset definitions
- `config/cosys_airsim_dev.json` -- Dev profile config
- `config/cosys_airsim.json` -- Cloud profile config
- `docker/docker-compose.cosys.yml` -- Cloud Docker Compose definition
