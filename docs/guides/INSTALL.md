# Installation Guide

Step-by-step instructions for setting up the Drone Companion Software Stack on a fresh Ubuntu machine. The stack is designed so that **all optional dependencies are truly optional** — you can build and run the full stack with only the core libraries. Each optional dependency unlocks additional backends.

| Dependency Level | Libraries | What It Enables |
|---|---|---|
| **Core (required)** | spdlog, fmt, Eigen3, nlohmann-json, GTest, Zenoh | Full stack with simulated backends; all tests pass |
| **Optional: OpenCV** | OpenCV 4.x with DNN module | `OpenCvYoloDetector` — YOLOv8-nano object detection via ONNX |
| **Optional: MAVSDK** | MAVSDK 2.x | `MavlinkFCLink` — real MAVLink communication with PX4 flight controller |
| **Optional: Gazebo** | Gazebo Harmonic (gz-sim 8), gz-transport 13, gz-msgs 10 | `GazeboCamera`, `GazeboIMU`, `GazeboVIOBackend` — SITL simulation with PX4 |
| **Optional: Cosys-AirSim** | AirSim C++ SDK (vendored submodule) | `CosysCamera`, `CosysRadar`, `CosysIMU`, `CosysDepth` — photorealistic simulation |

> **Conda/Anaconda Warning:** If you have Conda installed, **deactivate it before building:**
> ```bash
> conda deactivate
> ```
> Conda ships its own `libfmt`, `libstdc++`, and `libprotobuf` which conflict with system libraries. Symptoms: linker errors about `libfmt.so.9` conflicting with `libfmt.so.8`, or `GLIBCXX_3.4.30 not found`. The launch scripts handle this automatically, but **you must deactivate Conda for building and CMake configuration.**

---

## 1. Prerequisites

**Tested on:** Ubuntu 22.04 LTS and Ubuntu 24.04 LTS (x86_64) with GCC 11.4+ and CMake 3.22+.

```bash
# Update package lists
sudo apt-get update && sudo apt-get upgrade -y

# Core build tools
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    unzip
```

### Ubuntu 22.04 vs 24.04 Differences

| Package | Ubuntu 22.04 | Ubuntu 24.04 | Notes |
|---|---|---|---|
| GCC | 11.4 | 13.3 | Both work; 22.04 has stricter enum/ternary warnings |
| spdlog | 1.9.2 | 1.12.0 | 22.04 version links against `libfmt.so.8` separately |
| fmt | 8.1.1 (separate) | 10.x (bundled with spdlog) | On 22.04, install `libfmt-dev` explicitly |
| OpenCV | 4.5.4 (apt) | 4.6.0 (apt) | Both work for DNN; 4.10 from source recommended |
| CMake | 3.22 | 3.28 | Both work |
| clang-format-18 | Needs LLVM apt repo | Available in default repos | See Section 7 |
| nlohmann-json | 3.10.5 | 3.11.3 | Both work |

---

## 2. Core Dependencies (Required)

These are needed for every build. The CI pipeline installs only these.

```bash
sudo apt-get install -y --no-install-recommends \
    libspdlog-dev \
    libfmt-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libgtest-dev
```

| Library | Purpose |
|---|---|
| spdlog | Structured logging (all 7 processes) |
| fmt | String formatting (spdlog dependency, explicit on 22.04) |
| Eigen3 | Linear algebra — Kalman filter, pose math, path planning |
| nlohmann-json | JSON config parsing (`Config` class) |
| GTest | Unit testing framework |

---

## 3. Zenoh IPC Backend (Required)

Zenoh is the **sole IPC backend** for the companion stack. It provides high-performance publish/subscribe with zero-copy, peer-to-peer networking, and multi-machine support. The stack will not build without it.

Zenoh is not available via apt — install from GitHub releases.

### Step 3.1: Install zenohc (C library)

```bash
ZENOH_VERSION="1.7.2"

# Download the release zip (contains .deb packages)
wget "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip"

# Extract and install
unzip libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip
sudo dpkg -i libzenohc_*.deb libzenohc-dev_*.deb
sudo ldconfig

# Clean up
rm -f libzenohc-*.zip libzenohc_*.deb libzenohc-dev_*.deb

# Verify
pkg-config --modversion zenohc
# Should print: 1.7.2
```

**For aarch64 (Jetson Orin):** Replace `x86_64-unknown-linux-gnu` with `aarch64-unknown-linux-gnu` in the URL.

### Step 3.2: Install zenoh-cpp (C++ headers)

```bash
cd /tmp
git clone --depth 1 --branch 1.7.2 https://github.com/eclipse-zenoh/zenoh-cpp.git
cd zenoh-cpp
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHCXX_ZENOHC=OFF
sudo cmake --install build
cd ~ && rm -rf /tmp/zenoh-cpp

# Verify
ls /usr/local/include/zenoh.hxx
# Should exist
```

### Minimal Build (Core + Zenoh Only)

```bash
cd companion_software_stack
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
make -j$(nproc)
ctest --output-on-failure -j$(nproc)
```

`ALLOW_INSECURE_ZENOH=ON` is required on dev machines without TLS certificates. You should see all tests pass and 7 binaries in `build/bin/`. The stack runs with simulated backends — no hardware, OpenCV, MAVSDK, or Gazebo needed.

---

## 4. OpenCV 4.10 (Optional — YOLOv8 Detection)

### Why OpenCV?

The `OpenCvYoloDetector` backend uses OpenCV's DNN module to run a YOLOv8-nano ONNX model for 80-class object detection. This gives real object detection (people, vehicles, animals, etc.) instead of the random bounding boxes from the simulated detector or the color-only detection from `ColorContourDetector`.

**What it enables:**
- `"backend": "yolov8"` in config -> `OpenCvYoloDetector`
- Loads `models/yolov8n.onnx` (12.8 MB), runs inference at ~7-13 FPS on CPU
- 80 COCO classes mapped to internal `ObjectClass` enum (PERSON, VEHICLE_CAR, etc.)

**Without OpenCV:** The stack still builds and runs. The `ColorContourDetector` (pure C++ HSV segmentation) or `SimulatedDetector` are always available.

### Option A: Build from Source (Recommended)

The Ubuntu apt package works but is older (4.5.4 on 22.04, 4.6.0 on 24.04). Building from source gets you 4.10.0 with optimized DNN support:

```bash
# Install build dependencies
sudo apt-get install -y \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev

# Clone OpenCV 4.10.0
cd /tmp
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git

# Build with DNN module enabled (no CUDA — avoids version pinning issues)
mkdir -p opencv/build && cd opencv/build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
    -DBUILD_LIST=core,imgproc,dnn,imgcodecs,highgui \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python3=OFF \
    -DWITH_PROTOBUF=ON \
    -DBUILD_PROTOBUF=ON \
    -DOPENCV_DNN_OPENCL=OFF \
    -DWITH_CUDA=OFF \
    ..

make -j$(nproc)
sudo make install
sudo ldconfig
```

**Build time:** ~10-20 minutes depending on CPU cores.

### Option B: Ubuntu Package (Simpler, Older Version)

```bash
sudo apt-get install -y libopencv-dev
```

This installs OpenCV 4.5.4 (22.04) or 4.6.0 (24.04). It works but may lack some DNN optimizations present in 4.10.0.

### Verify OpenCV Installation

```bash
pkg-config --modversion opencv4
# Should print 4.10.0 (from source) or 4.5.4/4.6.0 (from apt)
```

Then rebuild the stack — CMake will auto-detect OpenCV:

```bash
cd companion_software_stack/build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
# Look for: "OpenCV : 4.10.0 — YOLOv8 detector available"
make -j$(nproc)
```

### Downloading ML Models

The stack uses two ONNX models for perception. Download scripts are provided in `models/`:

**YOLOv8n (object detection, 12.8 MB):**
```bash
# Option A: Direct download (no Python deps needed)
mkdir -p models
wget -O models/yolov8n.onnx \
    https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.onnx

# Option B: Export via script (requires: pip install ultralytics)
bash models/download_yolov8n.sh
```

**Depth Anything V2 ViT-S (monocular depth estimation, ~95 MB):**
```bash
# Must use the export script — raw HuggingFace ONNX is not compatible with
# OpenCV DNN (Resize node format mismatch). The script applies graph surgery
# to fix this.
# Use a venv to keep system Python clean:
python3 -m venv .venv
source .venv/bin/activate
pip install torch transformers onnx onnxruntime onnxsim
bash models/download_depth_anything_v2.sh
deactivate
```

> **Note:** Both models are optional. Without them, the corresponding test suites skip (13 tests) but everything else works. The YOLOv8 model can be exported via the `ultralytics` Python package; the DA V2 model **must** be exported via the script due to OpenCV DNN compatibility requirements.
>
> **DA V2 requires OpenCV 4.10+ at runtime.** The apt version (4.5.4 on 22.04, 4.6.0 on 24.04) cannot parse the `ReduceMean` ONNX node. If DA V2 model tests fail with "parse error: Required argument keepdims not found", build OpenCV 4.10 from source (Option A above).

### Option C: Build from Source with CUDA (GPU-Accelerated DNN)

If you have an NVIDIA GPU and want GPU-accelerated inference, add CUDA flags. **Requires CUDA 11.8+ and cuDNN 8.6+** (CUDA 11.5 is not compatible with OpenCV 4.10 CUDA modules).

```bash
# Same as Option A, but with CUDA enabled
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
    -DBUILD_LIST=core,imgproc,dnn,imgcodecs,highgui,cudev \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python3=OFF \
    -DWITH_PROTOBUF=ON \
    -DBUILD_PROTOBUF=ON \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DOPENCV_DNN_CUDA=ON \
    -DCUDA_ARCH_BIN=6.1 \
    ..
```

**`CUDA_ARCH_BIN` values by GPU:**

| GPU | Architecture | `CUDA_ARCH_BIN` |
|---|---|---|
| GTX 1080 Ti | Pascal | 6.1 |
| RTX 2080 / T1000 | Turing | 7.5 |
| RTX 3090 / A10G | Ampere | 8.6 |
| Jetson Orin | Ampere | 8.7 |

**If CUDA nvcc rejects your GCC version** (e.g., CUDA 11.x doesn't support GCC 13), point CUDA at an older host compiler:
```bash
-DCUDA_HOST_COMPILER=/usr/bin/g++-11
```

**Note:** `BUILD_LIST` must include `cudev` (from opencv_contrib) when CUDA is enabled.

> **GPU rendering vs CUDA compute:** UE5 and Cosys-AirSim use the GPU directly for rendering via Vulkan — they do **not** require the CUDA toolkit. CUDA is only needed for GPU-accelerated ML inference (OpenCV DNN CUDA, TensorRT). If your CUDA toolkit is older than 11.8, build OpenCV without CUDA (`-DWITH_CUDA=OFF`) — CPU DNN inference still works.

### Known Issues — OpenCV

| Issue | Symptom | Fix |
|---|---|---|
| **Stale source build with CUDA** | CMake error: `Could NOT find CUDA: Found unsuitable version` | Old OpenCV at `/usr/local` was built against a different CUDA. Fix: `-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4` to force apt version, or remove stale build: `sudo rm -rf /usr/local/lib/cmake/opencv4 /usr/local/lib/libopencv*` |
| **pkg-config finds apt version instead of source build** | CMake picks up 4.6.0 when 4.10.0 is installed to `/usr/local` | Set `OpenCV_DIR=/usr/local/lib/cmake/opencv4` or uninstall `libopencv-dev` |
| **Missing protobuf for DNN** | `cv::dnn::readNetFromONNX()` fails at build time | Add `-DWITH_PROTOBUF=ON -DBUILD_PROTOBUF=ON` to the OpenCV CMake flags |
| **Anaconda/Conda conflicts** | Linker picks up Conda's older `libstdc++.so` | `conda deactivate` before building |
| **ONNX model not found** | Runtime error: "Failed to load model" | Download `yolov8n.onnx` from [Ultralytics](https://github.com/ultralytics/assets/releases) into `models/` |

---

## 5. MAVSDK 2.x (Optional — PX4 MAVLink Communication)

### Why MAVSDK?

MAVSDK provides a high-level C++ API for MAVLink communication with the PX4 flight controller. It enables the `MavlinkFCLink` HAL backend, which sends/receives real commands (arm, takeoff, position setpoints, telemetry) over UDP to PX4 SITL or a physical flight controller over serial UART.

**What it enables:**
- `"backend": "mavlink"` in comms config -> `MavlinkFCLink`
- Real arming, takeoff, landing, position commands via MAVLink 2
- Telemetry: battery, GPS, attitude, flight mode, armed state

**Without MAVSDK:** The stack uses `SimulatedFCLink`, which generates synthetic telemetry (battery drain, fixed GPS, mode state machine). No real flight controller communication.

### Installation (Build from Source)

MAVSDK is not available as an Ubuntu apt package. Build from source:

```bash
# Install dependencies
sudo apt-get install -y \
    libcurl4-openssl-dev \
    libjsoncpp-dev \
    libtinyxml2-dev

# Clone and build MAVSDK v2.12.x
cd /tmp
git clone --recursive https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git checkout v2.12.12  # or latest v2.x tag

cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON

cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

**Build time:** ~15-30 minutes (downloads MAVLink definitions during build).

### Verify MAVSDK Installation

```bash
ls /usr/local/lib/libmavsdk.so
# Should exist

# Rebuild the stack
cd companion_software_stack/build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
# Look for: "MAVSDK : 2.12.12 — MavlinkFCLink backend available"
make -j$(nproc)
```

### Known Issues — MAVSDK

| Issue | Symptom | Fix |
|---|---|---|
| **Missing `--recursive` during clone** | Build fails with missing submodule errors | `git submodule update --init --recursive` |
| **CMake can't find MAVSDK** | "MAVSDK : NOT FOUND" despite installation | Set `CMAKE_PREFIX_PATH=/usr/local` or check `ldconfig -p \| grep mavsdk` |
| **Version mismatch** | Linking errors with MAVSDK v1.x | Make sure you use MAVSDK v2.x (API changed significantly from v1) |
| **Connection refused to PX4** | `MavlinkFCLink` fails to connect | PX4 SITL must be running first; default UDP port is `udp://:14540` |
| **Stale TCP/UDP sockets** | "Address already in use" on restart | `ss -ulnp \| grep 14540` — kill leftover PX4 process |

---

## 6. Gazebo Harmonic + PX4 SITL (Optional — Full Simulation)

### Why Gazebo?

Gazebo Harmonic (gz-sim 8) provides a physics-based 3D simulation environment. Combined with PX4 SITL (Software-In-The-Loop), it gives you a complete simulated drone with realistic camera, IMU, and GPS data — no physical hardware needed.

**What it enables:**
- `GazeboCamera` — subscribes to gz-transport image topics for real rendered frames
- `GazeboIMU` — subscribes to gz-transport IMU data (accelerometer + gyroscope)
- `GazeboVIOBackend` — subscribes to gz-transport odometry for ground-truth pose
- Full SITL loop: PX4 flight controller + physics + companion stack in one machine

**Without Gazebo:** The stack uses simulated backends (synthetic gradient images, circular trajectory pose, random IMU noise). Everything works, just not with physics or rendered visuals.

### Step 6.1: Install Gazebo Harmonic

Follow the [official installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu):

```bash
# Add Gazebo package repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic

# Also install development headers for C++ integration
sudo apt-get install -y \
    libgz-transport13-dev \
    libgz-msgs10-dev
```

> **Note:** The CMake build system also supports Gazebo Ionic (gz-transport14 / gz-msgs11) and will auto-detect whichever version is installed.

### Step 6.2: Install PX4-Autopilot

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Install PX4 build dependencies (script installs many packages)
bash Tools/setup/ubuntu.sh

# Build SITL target
make px4_sitl_default
```

**Build time:** ~20-40 minutes on first build.

### Step 6.3: Set Up Simulation Assets

The companion stack includes custom Gazebo world and drone model files that need to be linked into PX4's resource directories:

```bash
cd companion_software_stack

# Create symlinks for custom world and model
PX4_DIR="${HOME}/PX4-Autopilot"

# World file
ln -sf "$(pwd)/sim/worlds/test_world.sdf" \
    "${PX4_DIR}/Tools/simulation/gz/worlds/test_world.sdf"

# Drone model (x500 with companion computer sensors)
ln -sf "$(pwd)/sim/models/x500_companion" \
    "${PX4_DIR}/Tools/simulation/gz/models/x500_companion"
```

### Step 6.4: Rebuild Stack with Gazebo Support

```bash
cd companion_software_stack/build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
# Should show:
#   MAVSDK       : 2.12.12 — MavlinkFCLink backend available
#   OpenCV       : 4.10.0 — YOLOv8 detector available
#   Gazebo libs  : gz-transport 13.x, gz-msgs 10.x — Gazebo backends available
make -j$(nproc)
```

### Step 6.5: Run the Full SITL Stack

```bash
# Headless (no GUI)
bash deploy/launch_gazebo.sh

# With Gazebo GUI (renders the 3D scene)
bash deploy/launch_gazebo.sh --gui
```

The launch script:
1. Starts PX4 SITL with Gazebo physics server
2. Waits for MAVLink heartbeat on UDP port 14540
3. Optionally starts the Gazebo GUI client
4. Launches all 7 companion stack processes with `config/gazebo_sitl.json`
5. Catches Ctrl+C and cleanly shuts everything down

### Known Issues — Gazebo + PX4

| Issue | Symptom | Fix |
|---|---|---|
| **Gazebo GUI won't start** | `gz sim -g` crashes or shows black screen | Often caused by Snap-installed Gazebo conflicting with apt. Remove Snap version: `sudo snap remove gz-harmonic` |
| **Anaconda `libstdc++` conflict** | Runtime crash with `GLIBCXX_3.4.30 not found` | Deactivate Conda: `conda deactivate`, or set `LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"`. The launch script does this automatically. |
| **PX4 can't find world/model** | "World file not found" or model doesn't spawn | Ensure the symlinks from Step 6.3 are correct. Check `GZ_SIM_RESOURCE_PATH` includes both `sim/models/` and PX4's model directory. |
| **GUI detaches/lags** | GUI renders but doesn't show the drone | Give PX4 ~5-8 seconds to spawn the model before launching GUI. The launch script handles this with `sleep 3`. |
| **"Address already in use" on port 14540** | Stale PX4 instance from a previous run | `pkill -f px4; sleep 2` before relaunching |
| **Protobuf version mismatch** | Link errors about `google::protobuf` when building with both Gazebo and OpenCV | Build OpenCV with `-DBUILD_PROTOBUF=ON` to use its bundled protobuf, or ensure system protobuf version matches what Gazebo was built against |
| **Camera follow doesn't work in GUI** | Drone flies but camera stays at origin | The launch script sends `gz service` commands after an 8-second delay. If PX4 takes longer to initialize, increase the sleep in the GUI follow section. |

---

## 7. Developer Tooling

### clang-format-18

Required for the format gate (CI enforces `clang-format-18 --Werror`).

**Ubuntu 24.04:**
```bash
sudo apt-get install -y clang-format-18
```

**Ubuntu 22.04** (needs LLVM apt repository):
```bash
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install -y clang-format-18
rm llvm.sh
```

### Verify

```bash
clang-format-18 --version
# Should print: clang-format version 18.x.x
```

---

## 8. Complete Installation (All Dependencies)

For the full experience with all backends enabled:

```bash
# 0. Deactivate Conda if active
conda deactivate 2>/dev/null || true

# 1. Core (required)
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git pkg-config wget curl unzip \
    libspdlog-dev libfmt-dev libeigen3-dev nlohmann-json3-dev libgtest-dev

# 2. Zenoh (required) — see Section 3 for full commands
#    Download .zip from GitHub releases, extract, dpkg -i

# 3. OpenCV 4.10 from source (see Section 4 for full commands)
#    ...or quick: sudo apt-get install -y libopencv-dev

# 4. MAVSDK 2.x from source (see Section 5)
#    No apt package available

# 5. Gazebo Harmonic (see Section 6)
sudo apt-get install -y gz-harmonic libgz-transport13-dev libgz-msgs10-dev

# 6. PX4 SITL (see Section 6.2)
cd ~ && git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot && bash Tools/setup/ubuntu.sh && make px4_sitl_default

# 7. Developer tooling (see Section 7)
sudo apt-get install -y clang-format-18  # 24.04 only; see Section 7 for 22.04

# 8. Build the companion stack
cd companion_software_stack
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
make -j$(nproc)
ctest --output-on-failure -j$(nproc)
```

Or use the automated installer:
```bash
bash deploy/install_dependencies.sh --all
```

---

## 9. Build System Reference

### CMake Configure Summary

After running `cmake`, look at the summary printed at the end:

```
════════════════════════════════════════════
  Drone Companion Stack v1.0.0
  C++ Standard : C++17
  Build Type   : Release
  MAVSDK       : 2.12.12 (or 0 = not found)
  Gazebo       : TRUE (or FALSE)
  Cosys-AirSim : TRUE (or FALSE)
  OpenCV       : TRUE (or FALSE)
  Zenoh        : REQUIRED
  Model Preset : edge (or orin / cloud / dev)
  Plugins      : ON (or OFF)
  systemd      : ON (or OFF)
════════════════════════════════════════════
```

### Compile Definitions

| Define | Set When | Effect |
|---|---|---|
| `HAS_OPENCV` | `find_package(OpenCV)` succeeds | Enables `OpenCvYoloDetector` class, `DepthAnythingV2Estimator`, and ML test suites |
| `HAVE_MAVSDK` | `find_package(MAVSDK)` succeeds | Enables `MavlinkFCLink` backend in HAL |
| `HAVE_GAZEBO` | `find_package(gz-transport13)` + `find_package(gz-msgs10)` both succeed | Enables `GazeboCamera`, `GazeboIMU`, `GazeboVIOBackend` backends |
| `HAVE_COSYS_AIRSIM` | Cosys-AirSim submodule present and built | Enables `CosysCamera`, `CosysRadar`, `CosysIMU`, `CosysDepth` backends |
| `HAVE_DEPTH_ANYTHING_V2` | OpenCV found + DA V2 source compiled | Enables ML-based monocular depth estimation |

### Graceful Degradation

The build system is designed so that missing optional dependencies never break the build:

- If OpenCV is missing: `OpenCvYoloDetector` and `DepthAnythingV2Estimator` compile as stubs that return errors at runtime. The detector factory falls back to `ColorContourDetector`.
- If MAVSDK is missing: `MavlinkFCLink` is not compiled. The comms process uses `SimulatedFCLink`.
- If Gazebo is missing: Gazebo backends are not compiled. Simulated backends are used instead.
- If Cosys-AirSim submodule is missing: AirSim backends compile as stubs (ifdef guards). Run `git submodule update --init third_party/cosys-airsim` to enable.
- If Zenoh is missing: **Build fails.** Zenoh is the sole IPC backend and is always required.

---

## 10. Troubleshooting

### General

```bash
# Clean rebuild (fixes most CMake cache issues)
rm -rf build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON ..
make -j$(nproc)
```

### Conda/Anaconda Conflicts

Conda is the **#1 source of build failures** on developer machines. It ships its own versions of `libfmt`, `libstdc++`, `libprotobuf`, and `GTest` that conflict with system libraries.

| Symptom | Cause | Fix |
|---|---|---|
| `libfmt.so.8 may conflict with libfmt.so.9` + linker errors | Conda's `libfmt.so.9` conflicts with system spdlog built against `libfmt.so.8` | `conda deactivate` before building |
| `GLIBCXX_3.4.30 not found` | Conda's older `libstdc++.so.6` | `conda deactivate` or `LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"` |
| GTest found at `/home/*/anaconda3/lib/cmake/GTest/` | CMake finds Conda's GTest instead of system | `conda deactivate` or `-DGTest_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTest` |
| Protobuf version mismatch | Conda's protobuf conflicts with Gazebo's | `conda deactivate` |

**Best practice:** Always `conda deactivate` before any build or CMake operation.

### Check What CMake Detected

```bash
cd build
grep -E "OPENCV_FOUND|MAVSDK_FOUND|GAZEBO_FOUND|COSYS_AIRSIM_FOUND|zenohc" CMakeCache.txt
```

### Verify Shared Libraries

```bash
# Check that all linked .so files are found
ldd build/bin/perception | grep "not found"
ldd build/bin/comms | grep "not found"
ldd build/bin/slam_vio_nav | grep "not found"
```

If any show "not found", run `sudo ldconfig` or check `LD_LIBRARY_PATH`.

### Do NOT Build spdlog/fmt from Source

The apt versions of `libspdlog-dev` and `libfmt-dev` work correctly with GCC 13 on Ubuntu 22.04. **Do not build spdlog or fmt from source** — a source-built spdlog installs cmake configs to `/usr/local` that conflict with the apt shared library, causing linker errors like:
```
undefined reference to `spdlog::details::log_msg::log_msg(...)'
```
If you accidentally installed source-built versions, remove them:
```bash
sudo rm -rf /usr/local/lib/cmake/spdlog /usr/local/lib/cmake/fmt \
    /usr/local/lib/libspdlog* /usr/local/lib/libfmt* \
    /usr/local/include/spdlog /usr/local/include/fmt
sudo ldconfig
```

### ROS2 Coexistence

If ROS2 is installed (e.g., Humble), its `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH` entries may interfere. The companion stack does not depend on ROS2. If you see unexpected library resolution, try unsourcing ROS2:

```bash
# Before building:
unset CMAKE_PREFIX_PATH
unset AMENT_PREFIX_PATH
# Or open a fresh terminal without sourcing /opt/ros/humble/setup.bash
```

### Run Tests

```bash
cd build
ctest --output-on-failure -j$(nproc)
```

Expected: all tests pass. See [tests/TESTS.md](../../tests/TESTS.md) for the current baseline count (varies slightly based on which optional dependencies are detected).
