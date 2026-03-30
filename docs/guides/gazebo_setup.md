# Gazebo + PX4 SITL — Development Environment Setup

Step-by-step guide to set up PX4 SITL with Gazebo Harmonic for testing the companion software stack's HAL backends (MavlinkFCLink, GazeboCamera, GazeboIMU).

**Target:** Ubuntu 24.04 LTS (amd64)
**Time to complete:** ~30 minutes (varies with internet speed)

---

## Prerequisites

The core companion stack must already build. See the main [README](../README.md) for base dependencies.

```bash
sudo apt-get install -y build-essential cmake libspdlog-dev libeigen3-dev \
    nlohmann-json3-dev libgtest-dev git curl
```

---

## 1. Install Gazebo Harmonic

```bash
# Add OSRF package repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt-get update
sudo apt-get install -y gz-harmonic
```

### Verify Gazebo

```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# Check transport and message libraries
pkg-config --modversion gz-transport13 gz-msgs10
# Expected: 13.x.x and 10.x.x
```

---

## 2. Install PX4-Autopilot (SITL Mode)

```bash
# Clone PX4 (shallow clone to save space)
git clone --recursive --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot

# Run PX4 environment setup
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-sim-tools
```

> **Note:** The `--no-sim-tools` flag skips Gazebo Garden and jMAVSim install since we already have Gazebo Harmonic.

### Additional dependency (required by PX4's optical flow Gazebo plugin)

```bash
sudo apt-get install -y libopencv-dev
```

### Build PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl_default    # Build firmware only (works headless)
```

### Launch PX4 + Gazebo (requires display)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

You should see:
- Gazebo window with a quadrotor on the ground
- PX4 console showing MAVLink heartbeat on `udp://127.0.0.1:14540`

### Verify MAVLink

```bash
# In the PX4 console (pxh>)
commander arm
commander takeoff
# Drone should take off in Gazebo
commander land
```

Or connect QGroundControl to `udp://:14550` and arm/takeoff/land via the GUI.

---

## 3. Verify Gazebo Sensor Topics

While PX4 SITL + Gazebo is running, open another terminal:

```bash
gz topic --list
# Expected topics include:
#   /clock
#   /world/default/model/x500/link/base_link/sensor/imu_sensor/imu
#   /world/default/model/x500/link/base_link/sensor/camera/image
#   etc.

# Echo IMU data
gz topic --echo -t /world/default/model/x500/link/base_link/sensor/imu_sensor/imu

# Echo camera images (warning: binary data)
gz topic --echo -t /world/default/model/x500/link/base_link/sensor/camera/image
```

The exact topic paths depend on the model SDF. Use `gz topic --list` to discover them.

---

## 4. Install MAVSDK C++ Library

MAVSDK is **not** available via `apt` on Ubuntu 24.04. Build from source:

```bash
# Clone the latest stable release
git clone --depth 1 --branch v2.12.12 \
    https://github.com/mavlink/MAVSDK.git ~/MAVSDK

cd ~/MAVSDK
git submodule update --init --recursive

# Configure and build
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DSUPERBUILD=ON

cmake --build build -j$(nproc)

# Install to /usr/local
sudo cmake --install build
sudo ldconfig
```

### Verify MAVSDK

```bash
pkg-config --modversion mavsdk
# Expected: 2.12.12

# Verify CMake can find it
ls /usr/local/lib/cmake/MAVSDK/
# Expected: MAVSDKConfig.cmake, MAVSDKTargets.cmake, etc.
```

---

## 5. Verify Companion Stack Builds with Optional Dependencies

```bash
cd /path/to/companion_software_stack
rm -rf build && mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release ..
```

Look for these lines in the CMake output:

```
  MAVSDK       : 2.12.12 — MavlinkFCLink backend available
  Gazebo libs  : gz-transport 13.x.x, gz-msgs 10.x.x — Gazebo backends available
```

Then build and test:

```bash
make -j$(nproc)
ctest --output-on-failure -j$(nproc)
# All existing tests must still pass
```

---

## Architecture: How Simulator Backends Connect

```
                    PX4 SITL
                   ┌─────────┐
                   │ px4     │
                   │ firmware │
                   │ (SITL)  │
                   └────┬────┘
                        │ MAVLink UDP
                        │ udp://127.0.0.1:14540
                        ▼
┌─────────────────────────────────────────────────┐
│              Companion Stack                     │
│                                                  │
│  ┌──────────────┐         ┌──────────────────┐  │
│  │ MavlinkFCLink│◄─MAVSDK─┤ P5 Comms         │  │
│  │ (IFCLink)    │         │                   │  │
│  └──────────────┘         └──────────────────┘  │
│                                                  │
│  ┌──────────────┐         ┌──────────────────┐  │
│  │ GazeboCamera │◄gz-tpt──┤ P1 Video Capture │  │
│  │ (ICamera)    │         │                   │  │
│  └──────────────┘         └──────────────────┘  │
│                                                  │
│  ┌──────────────┐         ┌──────────────────┐  │
│  │ GazeboIMU    │◄gz-tpt──┤ P3 SLAM/VIO/Nav  │  │
│  │ (IIMUSource) │         │                   │  │
│  └──────────────┘         └──────────────────┘  │
└─────────────────────────────────────────────────┘
                        │
                        │ gz-transport
                        ▼
                   ┌─────────┐
                   │ Gazebo  │
                   │ Harmonic│
                   │ (sim)   │
                   └─────────┘
```

---

## Troubleshooting

| Issue | Solution |
|---|---|
| `make px4_sitl gz_x500` fails with OpenCV error | Install: `sudo apt-get install -y libopencv-dev` |
| PX4 build fails: missing Python packages | Run: `pip3 install --user empy==3.3.4 pyros-genmsg packaging jinja2 pyyaml jsonschema` |
| Gazebo window doesn't appear | Ensure you have a display. For headless: use `HEADLESS=1 make px4_sitl gz_x500` |
| `pkg-config --modversion mavsdk` fails | Run `sudo ldconfig` after MAVSDK install. Check `/usr/local/lib/pkgconfig/` |
| CMake doesn't find MAVSDK | Set `CMAKE_PREFIX_PATH=/usr/local` or verify `/usr/local/lib/cmake/MAVSDK/` exists |
| Gazebo topics not showing up | Check world/model SDF for sensor plugins. Use `gz topic --list` to discover actual paths |
| PX4 MAVLink not connecting | Default port is `udp://127.0.0.1:14540`. Check firewall / port conflicts |

---

## Headless Operation (CI / Remote SSH)

For environments without a display:

```bash
# PX4 SITL with Gazebo in headless mode
HEADLESS=1 make px4_sitl gz_x500

# Or use Gazebo's headless rendering
GZ_SIM_RENDER_ENGINE_PATH="" gz sim --headless-rendering -r worlds/default.sdf
```

> **Note:** The companion stack's Gazebo backends (`GazeboCamera`, `GazeboIMU`) use `gz-transport` to subscribe to topics — they don't need a Gazebo GUI window. They work in headless mode as long as `gz-sim` is running.

---

## What's Next

With the environment set up, the following phases will implement the actual backends:

| Phase | Issue | Description |
|---|---|---|
| Phase 1 | [#8](https://github.com/nmohamaya/companion_software_stack/issues/8) | `MavlinkFCLink` — MAVSDK-based `IFCLink` backend |
| Phase 2 | [#9](https://github.com/nmohamaya/companion_software_stack/issues/9) | `GazeboCamera` — gz-transport `ICamera` backend |
| Phase 3 | [#10](https://github.com/nmohamaya/companion_software_stack/issues/10) | `GazeboIMU` — gz-transport `IIMUSource` backend |
| Phase 4 | [#11](https://github.com/nmohamaya/companion_software_stack/issues/11) | Closed-loop integration test |
