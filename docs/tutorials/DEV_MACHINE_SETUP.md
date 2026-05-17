# Dev Machine Setup — Complete End-to-End Guide

> Real-world walkthrough of setting up the full dev + simulation stack on a
> fresh Ubuntu 22.04 machine. Every issue we hit is documented with the fix
> that actually worked, so a new developer can follow this top-to-bottom
> without hitting the same walls.
>
> Applies to any Linux development machine (desktop, laptop, or cloud VM).
> GPU-specific steps (CUDA, UE5) are clearly marked — skip them on CPU-only
> or low-VRAM machines.

**Target:** Full companion stack + Gazebo SITL + PX4 + Cosys-AirSim (Tier 3 photorealistic)
**Tested on:** Intel i7-7800X, 32 GB RAM, NVIDIA GTX 1080 Ti (11 GB VRAM), Ubuntu 22.04 LTS
**Driver:** NVIDIA 580.x (supports CUDA 13.0 capability)
**Estimated time:** ~4-6 hours (includes ~20GB UE5 download, ~40 min Cosys-AirSim build)

---

## Critical Warnings — Read First

### 1. Remove Conda/Anaconda

Conda ships its own `libfmt`, `libstdc++`, `libprotobuf`, and `GTest` which conflict with system libraries and cause linker errors. **Remove it entirely** — we hit `libfmt.so.8 vs libfmt.so.9` conflicts and wasted hours debugging.

```bash
rm -rf ~/anaconda3 ~/miniconda3
sed -i '/>>> conda initialize >>>/,/<<< conda initialize <<</d' ~/.bashrc
source ~/.bashrc
```

### 2. UE5 Version Matters

- **Use UE 5.4.** Linux + UE 5.5/5.6/5.7 is broken with Cosys-AirSim (confirmed by maintainer in [Cosys-AirSim Issue #57](https://github.com/Cosys-Lab/Cosys-AirSim/issues/57)).
- Use the Cosys-AirSim `5.4` **branch** (not a tag like `5.4-v3.2`).

### 3. ABI Consistency (libc++ vs libstdc++)

UE5 on Linux uses **libc++** (via clang). Cosys-AirSim must be built with clang + libc++ to match. If built with g++ (libstdc++), you get dozens of `undefined symbol: std::__cxx11::basic_string` errors during plugin link. We hit this twice — first with base cosys build.sh using gcc-13, then with clang-12 missing libc++ headers.

---

## Phase 1: System Prerequisites

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
    build-essential cmake git pkg-config wget curl unzip \
    lsb-release gnupg ca-certificates
```

### GCC 13 (match CI)

Ubuntu 22.04 ships GCC 11. CI uses GCC 13 (default on Ubuntu 24.04). Install via PPA but **keep GCC 11 too** — CUDA and some tooling need it.

```bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y g++-13 gcc-13
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
```

### Clang 18 + libc++ 18 (for Cosys-AirSim/UE5)

UE 5.4 requires clang-18. Install the full C++ toolchain AND libc++:

```bash
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install -y clang-18 clang++-18 clang-format-18 \
                        libc++-18-dev libc++abi-18-dev libunwind-18-dev lld-18
rm llvm.sh

# Verify libc++ headers exist (if this returns nothing, reinstall libc++-18-dev)
find /usr -path "*libc++*" -name "atomic" | head -1
```

---

## Phase 2: Core Stack Dependencies

```bash
sudo apt-get install -y --no-install-recommends \
    libspdlog-dev libfmt-dev libeigen3-dev \
    nlohmann-json3-dev libgtest-dev
```

> `libfmt-dev` is REQUIRED on Ubuntu 22.04 (spdlog needs it separately). The `install_dependencies.sh` used to miss this.

### Zenoh IPC (required)

Not in apt — download from GitHub releases:

```bash
ZENOH_VERSION="1.7.2"
wget "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip"
unzip libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip
sudo dpkg -i libzenohc_*.deb libzenohc-dev_*.deb
sudo ldconfig
rm -f libzenohc-*.zip libzenohc_*.deb libzenohc-dev_*.deb

# zenoh-cpp headers
cd /tmp
git clone --depth 1 --branch 1.7.2 https://github.com/eclipse-zenoh/zenoh-cpp.git
cd zenoh-cpp
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHCXX_ZENOHC=OFF
sudo cmake --install build
cd ~ && rm -rf /tmp/zenoh-cpp

# Verify
pkg-config --modversion zenohc   # 1.7.2
ls /usr/local/include/zenoh.hxx  # must exist
```

---

## Phase 3: CUDA 12.6 Toolkit

Driver 580 supports CUDA 13.0 capability, but the installed toolkit was CUDA 11.5 (too old for modern OpenCV). Upgrade to 12.6:

```bash
# Remove ALL old CUDA packages including the Ubuntu-shipped toolkit
sudo apt-get remove -y cuda-toolkit-11-5 cuda-tools-11-5 cuda-compiler-11-5 \
    nvidia-cuda-toolkit nvidia-cuda-dev nvidia-cuda-gdb nvidia-cuda-toolkit-doc 2>/dev/null
sudo apt-get autoremove -y

# Remove stale CUDA 11.5 headers (this is critical — they linger at /usr/include/)
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

nvcc --version  # must print 12.6
```

### Issue: Stale CUDA headers prevent new OpenCV CUDA build

**Symptom:** `error: #error -- unsupported GNU version! gcc versions later than 11 are not supported!`

**Why:** The old `/usr/include/crt/host_config.h` from CUDA 11.5 is still on the include path and gets picked up even though we installed CUDA 12.6 at `/usr/local/cuda-12.6/`.

**Fix:** the `sudo rm -rf` step above removes the stale headers. **Don't skip it.**

---

## Phase 4: OpenCV 4.10 from Source (with CUDA)

Must build from source — the apt OpenCV (4.5.4) can't parse the Depth Anything V2 ONNX model (missing `keepdims` attribute handling). **Do NOT use `-DBUILD_LIST=...`** — PX4 needs `calib3d`, `video` and other modules that a restrictive BUILD_LIST excludes.

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
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6
make -j$(nproc)
sudo make install
sudo ldconfig
```

### GPU architecture values

| GPU | Architecture | `CUDA_ARCH_BIN` |
|---|---|---|
| GTX 1080 Ti | Pascal | 6.1 |
| RTX 2080 / T1000 | Turing | 7.5 |
| RTX 3090 / A10G | Ampere | 8.6 |
| Jetson Orin | Ampere | 8.7 |

### Important: Remove apt OpenCV after source install

PX4's `ubuntu.sh` will reinstall `libopencv-dev` (4.5.4). After each PX4 build, verify and remove:

```bash
pkg-config --modversion opencv4   # Should be 4.10.0
# If it shows 4.5.4:
sudo apt-get remove -y libopencv-dev
sudo ldconfig
```

---

## Phase 5: MAVSDK 2.12

```bash
sudo apt-get install -y libcurl4-openssl-dev libjsoncpp-dev libtinyxml2-dev

cd /tmp
git clone --recursive https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git checkout v2.12.12

# CMAKE_POLICY_VERSION_MINIMUM=3.5 needed — bundled jsoncpp has old cmake_minimum_required
CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

---

## Phase 6: Gazebo Harmonic

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

## Phase 7: PX4 SITL

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx   # WILL reinstall libopencv-dev apt package
CMAKE_POLICY_VERSION_MINIMUM=3.5 make px4_sitl_default

# AFTER build, remove the apt OpenCV that ubuntu.sh reinstalled
sudo apt-get remove -y libopencv-dev && sudo ldconfig
pkg-config --modversion opencv4   # Should be 4.10.0 again
```

### Simulation asset symlinks

```bash
cd ~/Projects/companion_software_stack
PX4_DIR="${HOME}/PX4-Autopilot"
ln -sf "$(pwd)/sim/worlds/test_world.sdf" "${PX4_DIR}/Tools/simulation/gz/worlds/test_world.sdf"
ln -sf "$(pwd)/sim/models/x500_companion" "${PX4_DIR}/Tools/simulation/gz/models/x500_companion"
```

---

## Phase 8: Cosys-AirSim SDK (The Hard Part)

This is where most of the time went. **Use the `5.4` branch, not a tag.**

### 8.1: Initialize the submodule on the `5.4` branch

```bash
cd ~/Projects/companion_software_stack

# If submodule already exists but on wrong tag, clean it first
rm -rf third_party/cosys-airsim
git rm -f third_party/cosys-airsim 2>/dev/null; true
git submodule deinit -f third_party/cosys-airsim 2>/dev/null; true

# Add fresh submodule and check out the 5.4 branch
git submodule add --depth 1 -b 5.4 https://github.com/Cosys-Lab/Cosys-AirSim.git third_party/cosys-airsim
cd third_party/cosys-airsim
git fetch --depth 1 origin 5.4
git checkout FETCH_HEAD
```

### 8.2: Patch `build.sh` to use clang-18 + libc++

**Critical step.** Default `build.sh` uses `clang-12` which doesn't have libc++ headers on Ubuntu 22.04. Edit the `else` branch of the compiler selection:

```bash
# In third_party/cosys-airsim/build.sh, find the "if $gcc; then" block and change
# the else branch to use clang-18 with libc++:

# else
#     export CC="clang-18"
#     export CXX="clang++-18"
#     export CXXFLAGS="-stdlib=libc++"
#     export LDFLAGS="-stdlib=libc++ -lc++abi"
# fi
```

### 8.3: Run setup + build

```bash
cd ~/Projects/companion_software_stack/third_party/cosys-airsim
./setup.sh                                       # Installs toolchain, downloads rpclib, Eigen, etc.
CMAKE_POLICY_VERSION_MINIMUM=3.5 ./build.sh     # Build with clang-18 + libc++
```

### 8.4: Verify ABI is correct

```bash
# Should print NSt3__1 symbols (libc++), NOT NSt7__cxx11 (libstdc++)
nm AirLib/lib/libAirLib.a | grep "basic_string" | head -1
# Expected: starts with ...NSt3__1...
```

### Issues we hit

| Issue | Cause | Fix |
|---|---|---|
| `cmake_minimum_required(VERSION 2.x) removed` | bundled jsoncpp/rpclib use old CMake policy | `CMAKE_POLICY_VERSION_MINIMUM=3.5` env var on every build |
| `Eigen/Dense: No such file` | ExternalProject doesn't pick up system Eigen | Add `-I/usr/include/eigen3` to CXXFLAGS (already in our `FindAirSim.cmake`) |
| `AirSim radar API: RadarData not a member` | Base Cosys-AirSim has no native radar API | Use `getLidarData()` as radar proxy (ASVSim does the same) |
| `undefined symbol: std::__cxx11::basic_string` on UE5 link | libstdc++ vs libc++ ABI mismatch | Rebuild Cosys with clang-18 + libc++ (see 8.2) |
| `atomic file not found` with clang-18 | `libc++-18-dev` not installed | `sudo apt-get install -y libc++-18-dev libc++abi-18-dev` |
| UE 5.5/5.7 Linux crashes on `simGetImages` | Known bug in Cosys-AirSim + UE 5.5+ | Use UE 5.4 + Cosys `5.4` branch ([Issue #57](https://github.com/Cosys-Lab/Cosys-AirSim/issues/57)) |
| Build interrupts AirSim ExternalProject | Makefile regenerates stamp files | Just re-run `make -j$(nproc)` — it resumes |

---

## Phase 9: Unreal Engine 5.4

### 9.1: Download

Free Epic Games account required: https://www.epicgames.com/id/register

Then download the Linux binary: https://www.unrealengine.com/en-US/linux

**Get UE 5.4** (the 5.7 zip we initially downloaded actually contained 5.4.4 — verify via `cat Build.version` after extraction).

### 9.2: Install

```bash
sudo mkdir -p /opt/UnrealEngine
sudo chown $USER /opt/UnrealEngine
unzip ~/Downloads/Linux_Unreal_Engine_*.zip -d /opt/UnrealEngine
chmod -R 755 /opt/UnrealEngine

# Verify
cat /opt/UnrealEngine/Engine/Build/Build.version   # MajorVersion: 5, MinorVersion: 4
```

**Disk space:** UE5 extracts to ~90 GB. Make sure you have at least 100 GB free before unzipping, or the extraction will fail part-way and leave a corrupt partial install that you'll need to delete and retry.

### 9.3: Open Blocks environment (triggers plugin build)

```bash
# Close any stale UE5 instance
pkill -9 UnrealEditor 2>/dev/null; sleep 2

# Clean cached plugin binaries from prior builds
cd ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks
./update_from_git.sh           # Copies AirLib + MavLinkCom + rpclib into Blocks/Plugins/AirSim/
rm -rf Binaries Intermediate Plugins/AirSim/Binaries Plugins/AirSim/Intermediate

# Launch — UE5 will prompt "rebuild Blocks/AirSim?" — click YES (~5-10 min compile)
/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
    ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks/Blocks.uproject
```

### 9.4: Verify it works

1. UE5 editor opens with Blocks scene (you should see the drone)
2. Press the **Play** button (green triangle in toolbar)
3. Check that the AirSim RPC server is listening:
   ```bash
   nc -zv 127.0.0.1 41451
   # Expected: "Connection to 127.0.0.1 41451 port [tcp/*] succeeded!"
   ```

### Default AirSim settings location

AirSim reads settings from `~/Documents/AirSim/settings.json`. Default minimal settings for Blocks:

```json
{
    "SettingsVersion": 2.0,
    "SimMode": "Multirotor",
    "ViewMode": "FlyWithMe",
    "Vehicles": {
        "Drone0": {
            "VehicleType": "SimpleFlight",
            "DefaultVehicleState": "Armed",
            "AutoCreate": true,
            "Cameras": {
                "front_center": {
                    "CaptureSettings": [
                        { "ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90 },
                        { "ImageType": 2, "Width": 640, "Height": 480, "FOV_Degrees": 90 }
                    ],
                    "X": 0.5, "Y": 0, "Z": -0.1, "Pitch": 0, "Roll": 0, "Yaw": 0
                }
            },
            "Sensors": {
                "imu": { "SensorType": 2, "Enabled": true }
            }
        }
    }
}
```

For radar testing, also add a LiDAR sensor (we use LiDAR as radar proxy — base Cosys-AirSim has no native radar):
```json
"lidar": {
    "SensorType": 6,
    "Enabled": true,
    "Range": 100,
    "PointsPerSecond": 10000
}
```

---

## Phase 10: ML Models (Python venv)

```bash
cd ~/Projects/companion_software_stack
python3 -m venv .venv
source .venv/bin/activate
pip install ultralytics torch transformers onnx onnxruntime onnxsim
bash models/download_yolov8n.sh
bash models/download_depth_anything_v2.sh
deactivate
```

> Direct-download URLs for YOLOv8n ONNX **no longer exist** on Ubuntu releases — must export via `ultralytics` Python package.

---

## Phase 11: Build the Companion Stack

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

**Expected CMake output:**
```
  MAVSDK       : 2.12.12 — MavlinkFCLink backend available
  OpenCV       : 4.10.0 — YOLOv8 detector available
  Gazebo libs  : gz-transport 13.x, gz-msgs 10.x — Gazebo Harmonic backends available
  Cosys-AirSim : 5.4 — Cosys-AirSim HAL backends available (ExternalProject)
```

**Expected ctest:** `1500+ tests passed, 0 failed, 0 hanging`.

---

## Phase 11b: Proving-Ground Assets (optional, for scenarios 30+)

Epic #480 scenarios (starting with scenario 30 `cosys_static`) spawn people, furniture, and buildings into the Blocks env via `simSpawnObject` RPC. The assets must live inside the Blocks project before UE5 launches so its asset registry indexes them.

One-time import:

```bash
bash deploy/setup_blocks_assets.sh              # copies DynamicObjects + StarterContent
# re-runnable; idempotent (uses rsync --update).
```

The script copies:

- `DynamicObjects/Content/GroupedAI/` → SK_Mannequin humans (YOLO `person`).
- UE5 `StarterContent/Architecture/` → walls, pillars, doors (obstacles for radar + grid).
- UE5 `StarterContent/Props/` → chairs, couches, bushes (COCO `chair`/`couch`/`potted plant`).

…all into `third_party/cosys-airsim/Unreal/Environments/Blocks/Content/Imported/`. No UE5 rebuild is required — pure `.uasset` additions; launching Blocks once after import is enough for the registry to pick them up.

Once imported, scenario 30 will spawn objects automatically via the scenario runner:

```bash
bash tests/run_scenario_cosys.sh \
     config/scenarios/30_cosys_static.json \
     --base-config config/cosys_airsim_dev.json \
     --gui --verbose
```

---

## Phase 12: End-to-End Smoke Test

With UE5 running Blocks and Play pressed:

```bash
cd ~/Projects/companion_software_stack

# Compile smoke test with clang++-18 + libc++ (MUST match Cosys SDK ABI)
clang++-18 -std=c++17 -stdlib=libc++ -O0 -g \
    -DHAVE_COSYS_AIRSIM=1 \
    -DRPCLIB_MSGPACK=clmdep_msgpack \
    -I common/hal/include -I common/util/include \
    -I third_party/cosys-airsim/AirLib/include \
    -I third_party/cosys-airsim/AirLib/deps/eigen3 \
    -I third_party/cosys-airsim/MavLinkCom/include \
    -I third_party/cosys-airsim/external/rpclib/rpclib-2.3.1/include \
    -I /usr/include/eigen3 \
    tools/cosys_smoke_test.cpp \
    third_party/cosys-airsim/AirLib/lib/libAirLib.a \
    third_party/cosys-airsim/AirLib/deps/MavLinkCom/lib/libMavLinkCom.a \
    third_party/cosys-airsim/AirLib/deps/rpclib/lib/librpc.a \
    -lpthread -lc++ -lc++abi \
    -o /tmp/cosys_smoke_test

/tmp/cosys_smoke_test
```

**Expected output:**
```
=== Cosys-AirSim Smoke Test ===
[OK] Connected to 127.0.0.1:41451

[1/3] IMU (getImuData)
  accel: [0, 0, -9.80665] m/s^2 (magnitude 9.80665)
  [PASS] gravity magnitude within expected range

[2/3] Camera RGB (simGetImages Scene, front_center, Drone0)
  dimensions: 1280x720
  uint8 bytes: 2764800
  [PASS] 1280x720 RGB

[3/3] Depth (simGetImages DepthPerspective, front_center, Drone0)
  dimensions: 640x480
  float pixels: 307200
  depth range: [0, 0.773438] m
  [PASS] depth varies across scene

=== 3 passed, 0 failed ===
```

---

## Known Issues & Quick Reference

### ABI mismatch — most common failure

| Symptom | Cause |
|---|---|
| Link error `std::__cxx11::basic_string` (libstdc++) | Cosys-AirSim built with gcc — rebuild with clang-18 + libc++ |
| Link error `std::__1::basic_string` (libc++) | Our code built with gcc — build smoke test / stack with clang++-18 |
| Both ABIs present | Mixed build artifacts — `rm -rf build_release AirLib/lib AirLib/deps/*/lib` + rebuild |

**Debug command:** `nm libAirLib.a | grep basic_string | head -1`
- `NSt3__1...` → libc++ ✓ (what we want)
- `NSt7__cxx11...` → libstdc++ ✗

### CMake 4.x breaks third-party deps

Kitware PPA installs CMake 4.x. Many third-party deps (jsoncpp in MAVSDK, rpclib in AirSim) have `cmake_minimum_required(VERSION 2.x)` which CMake 4.x refuses.

**Fix:** Set `CMAKE_POLICY_VERSION_MINIMUM=3.5` on every cmake invocation.

### CUDA + GCC version compatibility

| CUDA | Max GCC | Workaround |
|---|---|---|
| 11.5 | GCC 10 | `-DCUDA_HOST_COMPILER=/usr/bin/g++-11` |
| 12.6 | GCC 13 | None needed |
| 13.x | GCC 14 | None needed |

### UE5 crashes on `simGetImages`

**Symptom:** Segfault in `ASimModeBase::getCamera` / `WorldSimApi::setPresetLensSettings`.
**Cause:** UE 5.5+ incompatibility — see [Cosys-AirSim Issue #57](https://github.com/Cosys-Lab/Cosys-AirSim/issues/57).
**Fix:** Use UE 5.4 + Cosys-AirSim `5.4` branch.

---

## Final Verification Checklist

```bash
# Versions
g++ --version | head -1              # GCC 13.x
clang++-18 --version | head -1       # clang 18.x
nvcc --version | grep release        # CUDA 12.6
pkg-config --modversion opencv4      # 4.10.0
pkg-config --modversion zenohc       # 1.7.2
cat /opt/UnrealEngine/Engine/Build/Build.version | grep Minor   # 4

# ABI check on Cosys SDK
nm ~/Projects/companion_software_stack/third_party/cosys-airsim/AirLib/lib/libAirLib.a \
  | grep basic_string | head -1      # Must show NSt3__1 (libc++)

# Stack build
cd ~/Projects/companion_software_stack/build
make -j$(nproc)                      # Zero warnings
ctest -j$(nproc)                     # All pass

# Gazebo SITL end-to-end
bash deploy/launch_gazebo.sh --gui

# Cosys-AirSim end-to-end (after UE5 Play pressed)
/tmp/cosys_smoke_test                # 3/3 PASS
```

---

## SSH Multi-Account Setup (Optional)

If you work with multiple GitHub accounts on this machine:

```bash
# Generate a second key
ssh-keygen -t ed25519 -C "second-email@example.com" -f ~/.ssh/id_ed25519_accountB -N ""

# Add alias to ~/.ssh/config
cat >> ~/.ssh/config << 'EOF'
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519

Host github-accountB
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_accountB
EOF
chmod 600 ~/.ssh/config

# Add to GitHub and update remote
gh ssh-key add ~/.ssh/id_ed25519_accountB.pub --title "this-machine"
git remote set-url origin git@github-accountB:owner/repo.git
```

---

## What Broke During This Setup

A real-world log of issues hit during setup, in chronological order:

1. **Conda libfmt.so.9 conflict with system libfmt.so.8** — wasted hours until we removed Conda entirely
2. **Zenoh .deb URL pattern changed** — old pattern `libzenohc_${VER}-1_amd64.deb` now distributed as zip
3. **OpenCV `BUILD_LIST` too restrictive** — broke PX4 build (missing `calib3d`, `video`)
4. **apt OpenCV reinstalled by PX4's ubuntu.sh** — needed to remove after every PX4 build
5. **CMake 4.x rejects old `cmake_minimum_required`** — needed `CMAKE_POLICY_VERSION_MINIMUM=3.5` everywhere
6. **Stale CUDA 11.5 headers at `/usr/include/`** — blocked CUDA 12.6 OpenCV build until manually removed
7. **AirSim ExternalProject didn't forward CMAKE_POLICY flag** — had to patch `FindAirSim.cmake`
8. **AirSim needs Eigen on include path** — had to add `-I/usr/include/eigen3` to CXXFLAGS
9. **Cosys-AirSim `5.5-v3.3` tag broken on Linux** — switched to `5.4` branch per Issue #57
10. **Build.sh defaulted to gcc (libstdc++) but UE5 needs clang + libc++** — ABI mismatch, hours of debugging
11. **clang-12 has no libc++ headers** — needed to install `libc++-18-dev` for clang-18
12. **UE5 segfault on `simGetImages`** — was UE 5.5+ incompatibility, fixed by downgrading to 5.4
13. **Integration tests hang without AirSim server** — `confirmConnection()` blocks indefinitely, moved to `#if 0`
14. **UE5 downloads are huge (~90GB extracted)** — partial extraction left 69GB orphan, had to clean + retry

Total resolution time: ~4-6 hours across two sessions.

---

## Time Estimates

| Step | Time | Notes |
|---|---|---|
| Core deps (apt) | 5 min | |
| GCC 13 + clang-format-18 | 5 min | PPA + LLVM repo |
| Clang 18 + libc++ 18 | 5 min | Required for AirSim ABI |
| Zenoh (download + install) | 2 min | .deb from GitHub |
| CUDA 12.6 toolkit | 10 min | Large download |
| OpenCV 4.10 from source (with CUDA) | 20-30 min | 12 cores |
| MAVSDK from source | 15-30 min | Downloads MAVLink defs during build |
| Gazebo Harmonic (apt) | 5 min | |
| PX4 SITL (clone + build) | 30-40 min | Recursive submodules |
| Cosys-AirSim (setup + build) | 10 min | After submodule init |
| UE5 5.4 download (pre-built) | 30-60 min | ~20GB zip, ~90GB extracted |
| ML models (YOLO + DA V2) | 5 min | Python venv + export |
| **Total (sequential)** | **~3-4 hours** | Parallelise downloads to save time |

---

## Running Tier 3 Cosys-AirSim Scenario (Post-Setup)

### Step 1: Launch UE5

Terminal 1:
```bash
/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
    ~/Projects/companion_software_stack/third_party/cosys-airsim/Unreal/Environments/Blocks/Blocks.uproject
```

Wait for "AirSim plugin started" in the bottom-left log. Verify:
```bash
timeout 3 bash -c '</dev/tcp/127.0.0.1/41451' && echo "RPC OK" || echo "RPC not available"
```

### Step 2: Launch Companion Stack

Terminal 2:
```bash
cd ~/Projects/companion_software_stack
CONFIG_FILE=config/cosys_airsim_dev.json
./build/bin/system_monitor --config ${CONFIG_FILE} &
./build/bin/video_capture --config ${CONFIG_FILE} &
./build/bin/perception --config ${CONFIG_FILE} &
./build/bin/slam_vio_nav --config ${CONFIG_FILE} &
./build/bin/mission_planner --config ${CONFIG_FILE} &
./build/bin/comms --config ${CONFIG_FILE} &
./build/bin/payload_manager --config ${CONFIG_FILE} &
wait
```

### Step 3: Monitor

Terminal 3:
```bash
tail -f drone_logs/*.log | grep -E "HealthCheck|DepthAnything|CosysRadar|UKF|FSM"
```

Expected sequence (~30s in):
```
[video_capture] Published mission frame #1
[perception] Detector initialized: yolov8s.onnx
[perception] Depth estimator initialized: depth_anything_v2_vits.onnx
[slam_vio_nav] VIO pose: (0.00, 0.00, 0.00) q=1 health=NOMINAL
[mission_planner] FSM state: ARMED → TAKING_OFF
```

### Cleanup

```bash
pkill -f "video_capture|perception|slam_vio_nav|mission_planner|comms|payload_manager|system_monitor"
```

### Troubleshooting

| Issue | Fix |
|---|---|
| RPC not available | UE5 not running — check port 41451 |
| No YOLO detections | Verify `models/yolov8n.onnx` exists |
| Processes don't stop on Ctrl+C | Launched with `&` — use `pkill` |
| UE5 "Vulkan Driver required" | Must use UE5 5.4, not 5.7+ |
| High latencies (>1s) | Close other apps, check `nvidia-smi` |

### Scenario Configuration

Scenario file: `config/scenarios/29_cosys_perception.json`
- 5 waypoints, 2.0 m/s cruise, 5m altitude, ~120s flight
- YOLO v8n + Depth Anything V2 ViT-S (dev profile)
- D* Lite planner + 3D potential field, 2m min distance
- UKF fuses radar (LiDAR proxy) + camera + depth
