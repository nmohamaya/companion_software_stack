# Desktop Development Machine Setup — Complete Log

> Step-by-step record of setting up the GTX 1080 Ti desktop (Ubuntu 22.04) as a full
> development environment. Every issue encountered is documented with the fix.
> A new developer should be able to follow this from top to bottom.

**Machine:** Intel i7-7800X, 32 GB RAM, NVIDIA GTX 1080 Ti (11 GB VRAM), Ubuntu 22.04 LTS
**Date:** 2026-04-15
**Target:** Full stack + Gazebo SITL + Cosys-AirSim (Tier 3 photorealistic simulation)

---

## Prerequisites

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
    build-essential cmake git pkg-config wget curl unzip \
    lsb-release gnupg ca-certificates
```

### Conda Warning

If Conda/Anaconda is installed, **remove it or deactivate it permanently**. Conda's `libfmt`, `libstdc++`, `libprotobuf`, and `GTest` conflict with system libraries and cause linker errors. We removed Conda entirely:

```bash
rm -rf ~/anaconda3 ~/miniconda3
sed -i '/>>> conda initialize >>>/,/<<< conda initialize <<</d' ~/.bashrc
source ~/.bashrc
```

---

## Step 1: Core Dependencies

```bash
sudo apt-get install -y --no-install-recommends \
    libspdlog-dev \
    libfmt-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libgtest-dev
```

> **Note:** `libfmt-dev` is required on Ubuntu 22.04 (separate from spdlog). On 24.04 it's bundled.

---

## Step 2: GCC 13 (match CI / Ubuntu 24.04)

Ubuntu 22.04 ships GCC 11. We install GCC 13 from PPA to match CI:

```bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y g++-13 gcc-13
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
```

> **Keep GCC 11 installed** — CUDA requires it as host compiler (`-DCUDA_HOST_COMPILER=/usr/bin/g++-11`).

---

## Step 3: clang-format-18

Not in Ubuntu 22.04 default repos — needs LLVM apt repo:

```bash
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install -y clang-format-18
rm llvm.sh
```

---

## Step 4: Zenoh IPC (Required)

Zenoh is the sole IPC backend. Not available via apt — install from GitHub releases:

```bash
ZENOH_VERSION="1.7.2"
wget "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip"
unzip libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip
sudo dpkg -i libzenohc_*.deb libzenohc-dev_*.deb
sudo ldconfig
rm -f libzenohc-*.zip libzenohc_*.deb libzenohc-dev_*.deb

# Verify
pkg-config --modversion zenohc  # Should print 1.7.2
```

### zenoh-cpp (C++ headers)

```bash
cd /tmp
git clone --depth 1 --branch 1.7.2 https://github.com/eclipse-zenoh/zenoh-cpp.git
cd zenoh-cpp
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHCXX_ZENOHC=OFF
sudo cmake --install build
cd ~ && rm -rf /tmp/zenoh-cpp
```

---

## Step 5: CUDA 12.6 Toolkit

The driver (580.x) supports CUDA 13.0, but the toolkit must be upgraded from 11.5:

```bash
# Remove ALL old CUDA toolkit packages (keep the driver)
sudo apt-get remove -y cuda-toolkit-11-5 cuda-tools-11-5 cuda-compiler-11-5 \
    nvidia-cuda-toolkit nvidia-cuda-dev nvidia-cuda-gdb nvidia-cuda-toolkit-doc 2>/dev/null
sudo apt-get autoremove -y
# Remove stale CUDA 11.5 headers (prevents version mismatch with OpenCV CUDA)
sudo rm -rf /usr/include/crt /usr/include/cuda* /usr/include/thrust \
    /usr/include/nv* /usr/include/cooperative_groups*

# Install CUDA 12.6
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-6

# Update PATH
echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

nvcc --version  # Should print 12.6
```

> **CUDA 12.6 still requires GCC 11 as host compiler for nvcc.** GCC 13 is not supported. This only affects CUDA device code compilation — the rest of the stack uses GCC 13 fine.

---

## Step 6: OpenCV 4.10 from Source (with CUDA)

Must build from source — the apt package (4.5.4) is too old for DA V2 depth models. Build with CUDA for GPU-accelerated DNN inference:

```bash
cd /tmp
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git

mkdir -p opencv/build && cd opencv/build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python3=OFF \
    -DWITH_PROTOBUF=ON -DBUILD_PROTOBUF=ON \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DOPENCV_DNN_CUDA=ON \
    -DCUDA_ARCH_BIN=6.1 \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 \
    -DBUILD_LIST=core,imgproc,dnn,imgcodecs,highgui,calib3d,video,cudev
make -j$(nproc)
sudo make install
sudo ldconfig
```

### After installing from source — remove the apt package

PX4's `ubuntu.sh` may reinstall it. If CMake shows OpenCV 4.5.4 instead of 4.10.0:

```bash
sudo apt-get remove -y libopencv-dev
sudo ldconfig
```

### Issues encountered

| Issue | Cause | Fix |
|---|---|---|
| `CUDA: OpenCV requires enabled 'cudev' module` | `BUILD_LIST` didn't include `cudev` | Add `cudev` to `BUILD_LIST` |
| `unsupported GNU version! gcc versions later than 11` | Stale CUDA 11.5 headers at `/usr/include/crt/` | Remove old headers: `sudo rm -rf /usr/include/crt /usr/include/cuda*`, set `-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6` |
| `cuda_runtime.h: No such file or directory` | Old CUDA headers removed but toolkit path not set | Add `-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6` to cmake |
| `opencv2/video/tracking.hpp: No such file` (PX4 build) | `BUILD_LIST` too restrictive | Add `video`, `calib3d` or remove `BUILD_LIST` entirely |
| CMake finds apt 4.5.4 instead of source 4.10.0 | Both installed, pkg-config prefers apt | Remove `libopencv-dev` apt package |

### CUDA_ARCH_BIN values

| GPU | Architecture | Value |
|---|---|---|
| GTX 1080 Ti | Pascal | 6.1 |
| RTX 2080 / T1000 | Turing | 7.5 |
| RTX 3090 / A10G | Ampere | 8.6 |
| Jetson Orin | Ampere | 8.7 |

---

## Step 7: MAVSDK 2.12 (PX4 MAVLink Communication)

```bash
sudo apt-get install -y libcurl4-openssl-dev libjsoncpp-dev libtinyxml2-dev

cd /tmp
git clone --recursive https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git checkout v2.12.12

CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

> **CMake 4.x issue:** MAVSDK's bundled jsoncpp has old `cmake_minimum_required`. The `CMAKE_POLICY_VERSION_MINIMUM=3.5` env var fixes it. This affects MAVSDK, PX4, and AirSim builds.

---

## Step 8: Gazebo Harmonic

```bash
sudo wget -q https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-transport13-dev libgz-msgs10-dev
```

---

## Step 9: PX4 SITL

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx

CMAKE_POLICY_VERSION_MINIMUM=3.5 make px4_sitl_default
```

> **PX4's `ubuntu.sh` may reinstall `libopencv-dev`** — remove it again after PX4 builds if CMake picks up 4.5.4.

### Simulation asset symlinks

```bash
cd ~/Projects/companion_software_stack
PX4_DIR="${HOME}/PX4-Autopilot"
ln -sf "$(pwd)/sim/worlds/test_world.sdf" "${PX4_DIR}/Tools/simulation/gz/worlds/test_world.sdf"
ln -sf "$(pwd)/sim/models/x500_companion" "${PX4_DIR}/Tools/simulation/gz/models/x500_companion"
```

---

## Step 10: Cosys-AirSim SDK + UE5

### 10.1: Initialize AirSim submodule

```bash
cd ~/Projects/companion_software_stack
git submodule add --depth 1 https://github.com/Cosys-Lab/Cosys-AirSim.git third_party/cosys-airsim
cd third_party/cosys-airsim
git fetch --depth 1 origin tag 5.5-v3.3
git checkout 5.5-v3.3
```

### 10.2: Build Cosys-AirSim plugin

```bash
cd ~/Projects/companion_software_stack/third_party/cosys-airsim
./setup.sh
CMAKE_POLICY_VERSION_MINIMUM=3.5 ./build.sh
```

### 10.3: Install Unreal Engine 5.5

> **CRITICAL: You MUST use UE5 5.5, NOT 5.7+.**
> Cosys-AirSim tag `5.5-v3.3` only supports UE5 5.5. UE5 5.7+ requires
> `VK_EXT_mesh_shader` (SM6 Vulkan profile) which Pascal GPUs (GTX 1080 Ti)
> don't support. You'll get "Vulkan Driver is required to run the engine"
> even though Vulkan works fine — it's the mesh shader extension that's missing.

#### Option A: Build from Source (Recommended)

> **⚠ CRITICAL: Use UE5 version 5.5 specifically.** Newer versions (5.7+) cause **VK_ERROR_DEVICE_LOST** (Vulkan GPU crash) on Pascal GPUs like the GTX 1080 Ti, killing the simulation mid-run. Tested 2026-04-17: UE5 5.4.4 crashes, 5.5 works.

Requires linking your Epic Games account to GitHub:
1. Create Epic account: https://www.epicgames.com/id/register
2. Link to GitHub: https://www.unrealengine.com/en-US/ue-on-github
3. Enter your GitHub username when prompted
4. **Accept the invite email** from GitHub (check inbox for EpicGames org invite)
5. Also accept at: https://github.com/orgs/EpicGames/invitation

```bash
sudo mkdir -p /opt/UnrealEngine
sudo chown $USER /opt/UnrealEngine

# Clone UE5 5.5 (private repo — requires Epic-GitHub link)
# Use the SSH alias for your linked account
git clone --depth 1 --branch 5.5 git@github-nmohamaya:EpicGames/UnrealEngine.git /opt/UnrealEngine

# Build (takes 1-2 hours)
cd /opt/UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

> **Build time:** 1-2 hours. Start this before bed and let it run overnight.

#### Option B: Pre-built Binary (if available for 5.5)

Download from: https://www.unrealengine.com/en-US/linux

Look for version 5.5.x specifically (not latest). If only 5.7+ is available,
use Option A.

```bash
sudo mkdir -p /opt/UnrealEngine
sudo chown $USER /opt/UnrealEngine
# Download 5.4.4 — must match Cosys-AirSim's EngineAssociation: "5.4"
unzip ~/Downloads/Linux_Unreal_Engine_5.4.4.zip -d /opt/UnrealEngine
chmod -R 755 /opt/UnrealEngine
```

#### Common Issues — UE5

| Issue | Cause | Fix |
|---|---|---|
| "Vulkan Driver is required to run the engine" with UE5 5.7 | UE5 5.7 requires `VK_EXT_mesh_shader` — Pascal GPUs don't support it | Use UE5 5.5 (not 5.7+) |
| "Repository not found" during git clone | Epic-GitHub link not complete | Accept the org invite email from GitHub |
| Clone works with one SSH key but not another | Multi-account SSH — use the right alias | `git@github-nmohamaya:EpicGames/...` |
| "No space left on device" during unzip | UE5 is ~50GB extracted | Free disk space, `df -h` to check |
| Vulkan not found at all | Missing Vulkan packages | `sudo apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers` |

### 10.4: Set up Blocks environment

```bash
cd ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks
./update_from_git.sh
```

### 10.5: Test UE5 + AirSim launch

```bash
/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
    ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks/Blocks.uproject
```

- If prompted "Missing Blocks Modules" → click **Yes** to rebuild (first time only)
- If prompted "Choose Vehicle" → click **Yes** for multirotor (drone) simulation
- The Blocks environment loads with a quadcopter visible
- AirSim RPC server starts on port 41451 automatically
- Verify: `ss -tlnp | grep 41451` should show UnrealEditor listening

> **UE version must match:** Cosys-AirSim 5.5-v3.3 targets UE 5.4. Using UE 5.7+ causes a full recompile and may fail on Pascal GPUs (missing Vulkan SM6 features). Always use **UE 5.4.4**.

---

## Step 11: ML Models

```bash
cd ~/Projects/companion_software_stack
python3 -m venv .venv
source .venv/bin/activate
pip install ultralytics torch transformers onnx onnxruntime onnxsim
bash models/download_yolov8n.sh
bash models/download_depth_anything_v2.sh
deactivate
```

---

## Step 12: Build and Test the Stack

```bash
cd ~/Projects/companion_software_stack
rm -rf build && mkdir -p build && cd build
CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc)
ctest --output-on-failure -j$(nproc)
```

Expected output:
```
  MAVSDK       : 2.12.12 — MavlinkFCLink backend available
  OpenCV       : 4.10.0 — YOLOv8 detector available
  Gazebo libs  : gz-transport 13.x, gz-msgs 10.x — Gazebo Harmonic backends available
  Cosys-AirSim : 5.5-v3.3 — Cosys-AirSim HAL backends available (ExternalProject)
```

---

## Step 13: SSH Multi-Account Setup

If you use multiple GitHub accounts from the same machine:

```bash
# Generate key for second account
ssh-keygen -t ed25519 -C "your-email@example.com" -f ~/.ssh/id_ed25519_secondaccount -N ""

# Add to ~/.ssh/config
cat >> ~/.ssh/config << 'EOF'
# Default account
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519

# Second account
Host github-secondaccount
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_secondaccount
EOF
chmod 600 ~/.ssh/config

# Add key to GitHub: Settings → SSH Keys
gh ssh-key add ~/.ssh/id_ed25519_secondaccount.pub --title "desktop-secondaccount"

# Update repo remote
git remote set-url origin git@github-secondaccount:owner/repo.git
```

---

## Known Issues & Workarounds

| Issue | Cause | Fix |
|---|---|---|
| CMake 4.x breaks third-party deps | `cmake_minimum_required(VERSION 2.x)` in jsoncpp, rpclib | `CMAKE_POLICY_VERSION_MINIMUM=3.5` env var on all builds. Affects MAVSDK, PX4, AirSim. |
| Stale CUDA 11.5 headers after upgrade | `/usr/include/crt/host_config.h` still says "gcc > 11 not supported" | Remove: `sudo rm -rf /usr/include/crt /usr/include/cuda* /usr/include/thrust /usr/include/nv*` |
| `cuda_runtime.h: No such file or directory` | Old CUDA headers removed but CMake doesn't know new toolkit path | Add `-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6` |
| `Found unsuitable CUDA version "11.5", required "12.6"` | `nvidia-cuda-toolkit` apt package still installed (CUDA 11.5) | `sudo apt-get remove -y nvidia-cuda-toolkit nvidia-cuda-dev` |
| PX4 reinstalls apt OpenCV | `ubuntu.sh` runs `apt install` | Remove `libopencv-dev` after PX4 build |
| OpenCV `BUILD_LIST` breaks PX4 | PX4 OpticalFlow needs `calib3d`, `video` | Add those to BUILD_LIST, or omit BUILD_LIST entirely |
| AirSim `confirmConnection()` blocks forever | No timeout in RPC client when server not running | Integration tests moved to `#if 0`, unit tests use offline-only assertions |
| Cosys-AirSim has no radar API | Radar is in ASVSim extension only (maritime vehicles) | Use `getLidarData()` as radar proxy — provides range + bearing, no velocity |
| UE5 5.7 "Vulkan Driver required" on Pascal | UE5 5.7+ needs `VK_EXT_mesh_shader` (SM6) — GTX 1080 Ti doesn't have it | **Use UE5 5.5** — matches Cosys-AirSim tag and supports Pascal |
| UE5 on Pascal (GTX 1080 Ti) | No Nanite/Lumen support | Runs in rasterization mode — functional but less visual quality |
| `libfmt.so.9` conflict | Conda's fmt conflicts with system spdlog | Remove Conda entirely |
| spdlog source build breaks linking | Source-built spdlog at `/usr/local` conflicts with apt shared lib | Don't build spdlog from source — apt version works fine with GCC 13 |
| Epic repo "not found" during clone | Need to link Epic account to GitHub AND accept org invite email | https://www.unrealengine.com/en-US/ue-on-github + accept email invite |

---

## Final Verification Checklist

```bash
# ── Versions ──
g++ --version | head -1          # GCC 13.x
nvcc --version | grep release    # CUDA 12.6
pkg-config --modversion opencv4  # 4.10.0
pkg-config --modversion zenohc   # 1.7.2
clang-format-18 --version        # 18.x
ls /usr/local/lib/libmavsdk.so   # MAVSDK present
gz sim --version                 # Gazebo Harmonic
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4  # PX4 SITL

# ── Vulkan (for UE5) ──
vulkaninfo --summary             # Should show GTX 1080 Ti

# ── Stack build ──
cd ~/Projects/companion_software_stack
rm -rf build && mkdir -p build && cd build
CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DALLOW_INSECURE_ZENOH=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
# Check output:
#   MAVSDK       : 2.12.12
#   OpenCV       : 4.10.0
#   Gazebo       : TRUE
#   Cosys-AirSim : TRUE
make -j$(nproc)    # Zero warnings
ctest -j$(nproc)   # All pass (1545+), no hangs

# ── Tier 1: Simulated (no external deps) ──
bash deploy/launch_all.sh

# ── Tier 2: Gazebo SITL ──
bash deploy/launch_gazebo.sh --gui

# ── Tier 3: Cosys-AirSim (after UE5 build) ──
# First launch UE5 with Blocks:
/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
    ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks/Blocks.uproject
# Then in another terminal:
bash deploy/launch_cosys.sh --gui
```

---

## Time Estimates (for planning)

| Step | Time | Notes |
|---|---|---|
| Core deps (apt) | 5 min | |
| GCC 13 + clang-format-18 | 5 min | PPA + LLVM repo |
| Zenoh (download + install) | 2 min | .deb from GitHub |
| CUDA 12.6 toolkit | 10 min | Large download |
| OpenCV 4.10 from source (with CUDA) | 20-30 min | 12 cores |
| MAVSDK from source | 15-30 min | Downloads MAVLink defs during build |
| Gazebo Harmonic (apt) | 5 min | |
| PX4 SITL (clone + build) | 30-40 min | Recursive submodules |
| Cosys-AirSim (setup + build) | 10 min | After submodule init |
| UE5 5.5 from source | 1-2 hours | Clone ~15GB + full build |
| ML models (YOLO + DA V2) | 5 min | Python venv + export |
| **Total (sequential)** | **~3-4 hours** | Parallelise downloads to save time |

---

## Running Tier 3 Cosys-AirSim Scenario (Post-Setup)

Once Unreal Engine and the companion stack are built, you can run the full perception scenario.

### Prerequisites

✓ UE5 5.4 built and tested  
✓ Companion stack built (`bash deploy/build.sh`)  
✓ ML models downloaded (`bash models/download_yolov8n.sh` + `bash models/download_depth_anything_v2.sh`)  
✓ RPC server port 41451 available (firewall check: `ss -tlnp | grep 41451`)

### Step 1: Launch UE5 with Cosys-AirSim

In **Terminal 1**, navigate to Blocks environment and start the simulator:

```bash
cd ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks

# Launch UE5 Blocks with AirSim plugin
/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
    ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks/Blocks.uproject

# Wait for:
# 1. UE5 window opens with 3D Blocks scene
# 2. Bottom-left log: "AirSim plugin started"
# 3. RPC server listening on 127.0.0.1:41451
```

**Verify RPC is ready (in another terminal):**
```bash
timeout 3 bash -c '</dev/tcp/127.0.0.1/41451' && echo "✓ RPC OK" || echo "✗ RPC not available"
```

### Step 2: Launch Companion Stack

In **Terminal 2**, start all 7 processes with the Cosys-AirSim dev config:

```bash
cd ~/Projects/companion_software_stack
CONFIG_FILE=config/cosys_airsim_dev.json

# Launch all processes in background
./build/bin/system_monitor --config ${CONFIG_FILE} &
./build/bin/video_capture --config ${CONFIG_FILE} &
./build/bin/perception --config ${CONFIG_FILE} &
./build/bin/slam_vio_nav --config ${CONFIG_FILE} &
./build/bin/mission_planner --config ${CONFIG_FILE} &
./build/bin/comms --config ${CONFIG_FILE} &
./build/bin/payload_manager --config ${CONFIG_FILE} &

wait  # Ctrl+C stops all processes gracefully
```

### Step 3: Monitor the Scenario

In **Terminal 3**, tail the logs and watch for key events:

```bash
cd ~/Projects/companion_software_stack

# Watch for perception pipeline
tail -f drone_logs/*.log | grep -E "HealthCheck|DepthAnything|CosysRadar|UKF|FSM"
```

**Expected log sequence (first ~30 seconds):**

```
[video_capture] [info] Published stereo pair #1 (0 drops)
[video_capture] [info] Published mission frame #1
[perception] [info] Detector initialized: yolov8s.onnx
[perception] [info] Depth estimator initialized: depth_anything_v2_vits.onnx
[slam_vio_nav] [info] VIO pose: (0.00, 0.00, 0.00) q=1 health=NOMINAL
[mission_planner] [info] FSM state: ARMED
[mission_planner] [info] FSM state: TAKING_OFF
```

**During flight (10-60 seconds):**

```
[perception] [info] YOLO: detected 3 objects
[perception] [info] Depth map: 1280x720, range 0.5-50.0m
[CosysRadar] [info] First scan: 512 detections from 'Radar'
[UKF] [info] tracks: 5 active, 2 coasted
[mission_planner] [info] Navigating to waypoint 1/5
```

**At completion (120+ seconds):**

```
[mission_planner] [info] FSM state: RETURNING_HOME
[mission_planner] [info] FSM state: LANDING
[mission_planner] [info] FSM state: LANDED
[system_monitor] [info] Stack health: NOMINAL
```

### Step 4: Verify Scenario Validation

After the stack completes (FSM reaches LANDED), check the validation results in the log dir:

```bash
# View summary
grep -E "HealthCheck|detections|FSM.*LANDED" drone_logs/perception*.log | tail -20

# Full validation checks:
echo "=== YOLO Detections ==="
grep -c "\[Detector\].*detected" drone_logs/*.log

echo "=== Depth Estimates ==="
grep -c "\[DepthAnything\].*depth map" drone_logs/*.log

echo "=== Radar Scans ==="
grep -c "\[CosysRadar\].*First scan" drone_logs/*.log

echo "=== UKF Fusion Active ==="
grep -c "\[UKF\].*tracks:" drone_logs/*.log

echo "=== Mission Success ==="
grep "FSM.*LANDED" drone_logs/mission*.log && echo "✓ Landing confirmed" || echo "✗ No landing"
```

### Scenario Configuration

Scenario file: `config/scenarios/29_cosys_perception.json`

**Key parameters:**
- **Mission**: 5 waypoints, 2.0 m/s cruise speed, 5m altitude, ~120s flight time
- **Perception**: YOLO v8s (dev) / v8m (cloud), Depth Anything V2 ViT-S (dev) / ViT-B (cloud)
- **Avoidance**: D* Lite planner + 3D potential field avoider, 2m min distance
- **Fusion**: UKF with radar (LiDAR proxy) + camera + depth
- **Radar**: CosysRadarBackend emulating from `getLidarData()` RPC calls

### Cleanup

When scenario is complete, stop all processes:

```bash
# If using `wait`, just press Ctrl+C in Terminal 2

# Or kill manually:
pkill -f "video_capture|perception|slam_vio_nav|mission_planner|comms|payload_manager|system_monitor"

# Verify:
ps aux | grep -E "video_capture|perception|mission_planner" | grep -v grep
# Should return nothing
```

### Troubleshooting

| Issue | Cause | Fix |
|---|---|---|
| "RPC not available" | UE5 not running or crashed | Restart UE5, check port 41451 |
| "Invalid frame (drop)" | Slow RPC connection | Normal on startup, check network latency |
| "Invalid IMU reading" | Sensor warmup | Expected first 30 seconds, harmless |
| "No YOLO detections" | Model not found | Verify `models/yolov8s.onnx` exists |
| "Depth map empty" | DA V2 model not ready | Check `models/depth_anything_v2_vits.onnx` |
| "Processes don't stop on Ctrl+C" | Launched with `&` without `wait` | Use `pkill` or restart terminal |
| High latencies (>1s) | CPU throttling / GPU memory pressure | Close other apps, check `nvidia-smi` |
| UE5 "Vulkan Driver required" | Using UE5 5.7+ on Pascal GPU | **Must use UE5 5.4**, not 5.7+ |

### Next Steps

- **Iterate on scenario**: Edit `config/scenarios/29_cosys_perception.json` to change waypoints, models, or planner parameters
- **Profile performance**: Use `nvidia-smi` to monitor GPU (should stay <60% for inference)
- **Test cloud deployment**: Run on AWS g5.xlarge with Docker: `docker compose -f docker/docker-compose.cosys.yml up`
- **Extend integration tests**: Add checks to `tests/test_cosys_hal_backends.cpp` for E2E validation
