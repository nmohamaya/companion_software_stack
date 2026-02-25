# Gazebo SITL Simulation — Setup & Flight Guide

Complete guide to running the drone companion software stack in a fully closed-loop Gazebo simulation with PX4 SITL. Covers installation, building, launching, visualising, modifying the flight plan, and troubleshooting.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Prerequisites](#2-prerequisites)
3. [Install Gazebo Harmonic](#3-install-gazebo-harmonic)
4. [Install PX4-Autopilot (SITL)](#4-install-px4-autopilot-sitl)
5. [Install MAVSDK C++ Library](#5-install-mavsdk-c-library)
6. [Symlink Custom Model & World into PX4](#6-symlink-custom-model--world-into-px4)
7. [Build the Companion Stack](#7-build-the-companion-stack)
8. [Launch the Simulation](#8-launch-the-simulation)
9. [What Happens During a Flight](#9-what-happens-during-a-flight)
10. [Using the Gazebo GUI](#10-using-the-gazebo-gui)
11. [Modifying the Flight Plan](#11-modifying-the-flight-plan)
12. [Adding Obstacles to the World](#12-adding-obstacles-to-the-world)
13. [Viewing Logs](#13-viewing-logs)
14. [Troubleshooting](#14-troubleshooting)

---

## 1. Overview

The simulation connects three components:

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Linux Host                                  │
│                                                                     │
│   ┌──────────────┐    MAVLink UDP     ┌──────────────────────────┐  │
│   │  PX4 SITL    │◄──────────────────►│  Companion Stack         │  │
│   │  Firmware     │  udp://:14540      │  (7 processes)           │  │
│   └──────┬───────┘                    │                          │  │
│          │                            │  P1 Video Capture        │  │
│   Gazebo │ libgz-sim                  │  P2 Perception           │  │
│   Plugin │                            │  P3 SLAM/VIO/Nav         │  │
│          ▼                            │  P4 Mission Planner      │  │
│   ┌──────────────┐    gz-transport    │  P5 Comms (MavlinkFCLink)│  │
│   │  Gazebo      │◄─────────────────►│  P6 Payload Manager      │  │
│   │  Harmonic    │  /camera, /imu,    │  P7 System Monitor       │  │
│   │  (Server)    │  /stereo_left      └──────────────────────────┘  │
│   └──────┬───────┘                                                  │
│          │                                                          │
│   ┌──────▼───────┐                                                  │
│   │  Gazebo GUI  │  (optional — 3D visualisation)                   │
│   │  Client      │                                                  │
│   └──────────────┘                                                  │
└─────────────────────────────────────────────────────────────────────┘
```

**Key points:**

- PX4 launches the Gazebo physics *server* (no GUI by default).
- The companion stack talks to PX4 via MAVSDK over MAVLink UDP.
- Video Capture and SLAM read Gazebo sensor topics (`/camera`, `/stereo_left`, `/imu`) via gz-transport.
- A separate Gazebo GUI client can be launched for 3D visualisation.
- All 7 companion processes communicate via POSIX shared memory (IPC).

---

## 2. Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| **Ubuntu** | 24.04 LTS (amd64) | Other versions may work but are untested |
| **GCC** | ≥ 11 | C++17 support required |
| **CMake** | ≥ 3.16 | Build system |
| **Display server** | X11 or Wayland | Required for GUI mode only |

Install base dependencies:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git curl \
    libspdlog-dev libeigen3-dev nlohmann-json3-dev libgtest-dev
```

---

## 3. Install Gazebo Harmonic

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

### Verify

```bash
gz sim --version          # Expected: Gazebo Sim, version 8.x.x
pkg-config --modversion gz-transport13 gz-msgs10
```

---

## 4. Install PX4-Autopilot (SITL)

```bash
# Clone PX4
git clone --recursive --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot

cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-sim-tools

# Install OpenCV (required by PX4's optical flow plugin)
sudo apt-get install -y libopencv-dev

# Build PX4 SITL firmware
make px4_sitl_default
```

### Verify

```bash
# Quick smoke test (will open Gazebo + PX4 console):
cd ~/PX4-Autopilot && make px4_sitl gz_x500
# Type in the PX4 console: commander arm && commander takeoff
# The drone should lift off. Press Ctrl+C to quit.
```

---

## 5. Install MAVSDK C++ Library

MAVSDK is not in the Ubuntu 24.04 repos — build from source:

```bash
git clone --depth 1 --branch v2.12.12 \
    https://github.com/mavlink/MAVSDK.git ~/MAVSDK

cd ~/MAVSDK
git submodule update --init --recursive

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DSUPERBUILD=ON

cmake --build build -j$(nproc)

sudo cmake --install build
sudo ldconfig
```

### Verify

```bash
pkg-config --modversion mavsdk                    # Expected: 2.12.12
ls /usr/local/lib/cmake/MAVSDK/MAVSDKConfig.cmake # Should exist
```

---

## 6. Symlink Custom Model & World into PX4

PX4's startup script looks for models and worlds in specific directories. Create symlinks so PX4 can find our custom `x500_companion` model and `test_world.sdf`:

```bash
# Set paths (adjust if your project root is elsewhere)
PROJECT_DIR="$HOME/companion_software_stack"
PX4_DIR="$HOME/PX4-Autopilot"

# Symlink the custom drone model
ln -sfn "${PROJECT_DIR}/sim/models/x500_companion" \
    "${PX4_DIR}/Tools/simulation/gz/models/x500_companion"

# Symlink the test world
ln -sfn "${PROJECT_DIR}/sim/worlds/test_world.sdf" \
    "${PX4_DIR}/Tools/simulation/gz/worlds/test_world.sdf"
```

### What the custom model adds

The `x500_companion` model extends PX4's standard `x500` quadrotor with three companion-computer sensors:

| Sensor | Type | Resolution | Rate | Gazebo Topic |
|---|---|---|---|---|
| Mission Camera | RGB Camera | 640 × 480 | 30 Hz | `/camera` |
| Stereo Left | Greyscale Camera | 640 × 480 | 30 Hz | `/stereo_left` |
| Companion IMU | IMU (with noise) | — | 200 Hz | `/imu` |

All sensors are fixed-joint attached to `base_link`.

---

## 7. Build the Companion Stack

```bash
cd "$PROJECT_DIR"

# Option A: Use the build script
./deploy/build.sh           # Release build
./deploy/build.sh Debug     # Debug build (more logging, symbols)

# Option B: Manual CMake
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

CMake auto-detects MAVSDK and Gazebo libraries and enables compile guards (`HAVE_MAVSDK`, `HAVE_GAZEBO`). Look for these lines in the CMake output:

```
  MAVSDK       : 2.12.12 — MavlinkFCLink backend available
  Gazebo libs  : gz-transport 13.x.x, gz-msgs 10.x.x — Gazebo backends available
```

### Run unit tests

```bash
ctest --test-dir build --output-on-failure -j$(nproc)
# Expected: 196 tests passed
```

---

## 8. Launch the Simulation

### One-command launch (recommended)

The launch script handles everything: SHM cleanup, PX4 start, MAVLink heartbeat wait, optional GUI, and companion stack start-up in the correct order.

```bash
# Headless (no 3D window — for CI, SSH, or fast testing):
bash deploy/launch_gazebo.sh

# With Gazebo 3D visualisation:
bash deploy/launch_gazebo.sh --gui

# With debug logging:
bash deploy/launch_gazebo.sh --gui --log-level debug
```

Press **Ctrl+C** to gracefully stop everything (PX4 + Gazebo + all 7 processes).

### What the launch script does (step by step)

1. Cleans stale POSIX shared memory segments (`/dev/shm/drone_*`, etc.)
2. Creates the log directory (`drone_logs/`)
3. Starts PX4 SITL, which in turn launches Gazebo Harmonic as a server
4. Waits up to 30 s for the MAVLink UDP port (`14540`) to become available
5. **(If `--gui`)** Launches the Gazebo GUI client with a chase-cam configuration and auto-follows the drone
6. Starts the 7 companion processes in dependency order:
   - `system_monitor` → `video_capture` → `comms` (1 s settle) → `perception` → `slam_vio_nav` → `mission_planner` → `payload_manager`
7. Monitors all PIDs — if any process exits, triggers a full shutdown

### Environment variable overrides

| Variable | Default | Description |
|---|---|---|
| `PX4_DIR` | `~/PX4-Autopilot` | Path to PX4-Autopilot checkout |
| `GZ_WORLD` | `sim/worlds/test_world.sdf` | World SDF file |
| `CONFIG_FILE` | `config/gazebo.json` | JSON config for the companion stack |
| `LOG_DIR` | `<project>/drone_logs` | Where process logs are written |

Example with overrides:

```bash
PX4_DIR=/opt/px4 CONFIG_FILE=config/my_mission.json \
    bash deploy/launch_gazebo.sh --gui
```

---

## 9. What Happens During a Flight

With the default configuration (`config/gazebo.json`), the autonomous flight sequence is:

```
        IDLE ──► PREFLIGHT ──► TAKEOFF ──► NAVIGATE ──► RTL ──► LAND
         │         │              │           │           │        │
    FSM starts   ARM sent      10 m AGL    3 waypoints  Auto     Auto
    load mission  (retry 3s)   reached     flown        descend  disarm
```

### Default flight plan (~50 seconds)

The default scenario flies a clearly visible triangle pattern at 5 m altitude, completing in approximately 50 seconds (ARM → landed):

| Phase | Action | Details | ~Time |
|---|---|---|---|
| **1. ARM** | Mission planner sends ARM command | Retries every 3 s until PX4 confirms armed | ~1 s |
| **2. TAKEOFF** | Climb to 5 m AGL | PX4 internal climb rate ~1.5 m/s; transitions to NAVIGATE when altitude ≥ 4.5 m (90%) | ~13 s |
| **3. WP 1** | Fly to (15, 0, 5) | 15 m north at 5 m altitude, heading 0° | ~6 s |
| **4. WP 2** | Fly to (15, 15, 5) | 15 m east, heading 90°, **camera capture triggered** | ~6 s |
| **5. WP 3** | Fly to (0, 0, 5) | Return toward origin at 5 m altitude | ~9 s |
| **6. RTL** | Return-to-launch | PX4 flies home at 5 m (RTL altitude configured low to avoid 30 m default climb) and lands | ~14 s |
| **7. LAND** | Touch down and disarm | Automatic disarm after landing at the takeoff point | (included in RTL) |

Coordinates are in the **local frame** relative to the drone's spawn point (origin). Internal convention: X=North, Y=East, Z=Up. Cruise speed is 5.0 m/s with a 2.0 m waypoint acceptance radius. The path forms a 15 m triangle at 5 m altitude, covering ~51 m total distance.

> **Note:** The takeoff phase (~13 s) is dominated by PX4's internal MPC controller
> (`MPC_TKO_SPEED` ≈ 1.5 m/s plus settling time) and cannot be reduced from the
> companion side. The navigation phase (WP1→WP3) takes ~21 s. RTL altitude is
> set to 5 m via `action->set_return_to_launch_altitude()` so the drone returns
> home at flight altitude instead of climbing to PX4's default 30 m.

---

## 10. Using the Gazebo GUI

When launched with `--gui`, the script:

1. Opens the Gazebo 3D window with a **chase-cam** configuration (`sim/gui.config`)
2. After 8 seconds, sends a **camera follow** command so the view automatically tracks the drone

### Camera controls (while following)

| Action | Control |
|---|---|
| **Orbit** around the drone | Left-click + drag |
| **Zoom** in/out | Scroll wheel |
| **Pan** | Right-click + drag |

### If you lose sight of the drone

1. In the **Entity Tree** panel (left side), find `x500_companion_0`
2. Right-click it → **Move to**
3. The camera will snap to the drone

### Manual camera follow (if auto-follow didn't activate)

In a separate terminal:

```bash
# Set follow offset: 6 m behind, 3 m above
gz service -s /gui/follow/offset \
    --reqtype gz.msgs.Vector3d \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "x: -6, y: 0, z: 3"

# Start following the drone
gz service -s /gui/follow \
    --reqtype gz.msgs.StringMsg \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "data: 'x500_companion_0'"
```

> **Note (Conda/Snap users):** If `gz` commands fail with symbol lookup errors, prefix them with a clean environment:
> ```bash
> env -i HOME=$HOME DISPLAY=$DISPLAY PATH=/usr/bin:/usr/local/bin:/bin gz sim -g
> ```

---

## 11. Modifying the Flight Plan

The flight plan is defined in the JSON config file. The default for simulation is `config/gazebo.json`.

### Waypoint format

Open `config/gazebo.json` and find the `mission_planner` section:

```json
"mission_planner": {
    "update_rate_hz": 10,
    "takeoff_altitude_m": 5.0,
    "acceptance_radius_m": 2.0,
    "cruise_speed_mps": 5.0,
    "obstacle_avoidance": {
        "min_distance_m": 2.0,
        "influence_radius_m": 5.0,
        "repulsive_gain": 2.0
    },
    "waypoints": [
        {"x": 15, "y": 0,  "z": 5, "yaw": 0,    "speed": 5.0, "payload_trigger": false},
        {"x": 15, "y": 15, "z": 5, "yaw": 1.57,  "speed": 5.0, "payload_trigger": true},
        {"x": 0,  "y": 0,  "z": 5, "yaw": 0,     "speed": 5.0, "payload_trigger": false}
    ]
}
```

### Waypoint fields

| Field | Type | Unit | Description |
|---|---|---|---|
| `x` | float | metres | North position (NED-local frame from spawn) |
| `y` | float | metres | East position (NED-local frame from spawn) |
| `z` | float | metres | Altitude AGL (positive = up) |
| `yaw` | float | radians | Heading (0 = north, π/2 = east, π = south, -π/2 = west) |
| `speed` | float | m/s | Cruise speed for this leg (overrides `cruise_speed_mps`) |
| `payload_trigger` | bool | — | If `true`, triggers a camera capture when waypoint is reached |

### Mission parameters

| Parameter | Description | Default |
|---|---|---|
| `takeoff_altitude_m` | Target altitude for the takeoff phase | 5.0 m |
| `acceptance_radius_m` | How close the drone must be to a waypoint to consider it "reached" | 2.0 m |
| `cruise_speed_mps` | Default cruise speed (used if waypoint `speed` is omitted) | 5.0 m/s |
| `update_rate_hz` | Mission planner loop frequency | 10 Hz |

### Example: Square patrol at 20 m altitude

```json
"mission_planner": {
    "takeoff_altitude_m": 20.0,
    "acceptance_radius_m": 2.0,
    "cruise_speed_mps": 3.0,
    "obstacle_avoidance": {
        "min_distance_m": 2.0,
        "influence_radius_m": 5.0,
        "repulsive_gain": 2.0
    },
    "waypoints": [
        {"x": 30, "y": 0,  "z": 20, "yaw": 0,     "speed": 3.0, "payload_trigger": false},
        {"x": 30, "y": 30, "z": 20, "yaw": 1.57,   "speed": 3.0, "payload_trigger": true},
        {"x": 0,  "y": 30, "z": 20, "yaw": 3.14,   "speed": 3.0, "payload_trigger": true},
        {"x": 0,  "y": 0,  "z": 20, "yaw": -1.57,  "speed": 3.0, "payload_trigger": false}
    ]
}
```

This flies a 30 m × 30 m square at 20 m altitude, taking photos at the two far corners.

### Example: Single-waypoint hover test

```json
"waypoints": [
    {"x": 0, "y": 0, "z": 15, "yaw": 0, "speed": 1.0, "payload_trigger": false}
]
```

The drone takes off, flies to 15 m directly above the launch pad, considers the waypoint reached (it's at the origin), and immediately triggers RTL. Useful for testing takeoff + landing.

### Example: Long-range straight line

```json
"mission_planner": {
    "takeoff_altitude_m": 15.0,
    "cruise_speed_mps": 5.0,
    "waypoints": [
        {"x": 50,  "y": 0, "z": 15, "yaw": 0,    "speed": 5.0, "payload_trigger": false},
        {"x": 100, "y": 0, "z": 15, "yaw": 0,    "speed": 5.0, "payload_trigger": true},
        {"x": 0,   "y": 0, "z": 15, "yaw": 3.14, "speed": 5.0, "payload_trigger": false}
    ]
}
```

### Using a custom config file

You can create a separate config file for each mission profile without modifying the default:

```bash
# Copy the base config
cp config/gazebo.json config/my_patrol.json

# Edit waypoints in my_patrol.json
nano config/my_patrol.json

# Launch with the custom config
CONFIG_FILE=config/my_patrol.json bash deploy/launch_gazebo.sh --gui
```

### Coordinate system reference

```
                North (+x)
                    ▲
                    │
                    │
  West (-y) ◄──────┼──────► East (+y)
                    │
                    │
                    ▼
                South (-x)

  Altitude (+z) is up (AGL — above ground level)
  Yaw: 0 = North, π/2 = East, π = South, -π/2 = West
  Origin (0, 0, 0) = drone spawn point (landing pad)
```

### What happens after the last waypoint

After all waypoints are reached, the mission planner automatically sends an **RTL (Return-to-Launch)** command to PX4. PX4 handles the descent and landing autonomously. The drone disarms after touchdown.

---

## 12. Adding Obstacles to the World

The world file `sim/worlds/test_world.sdf` contains static obstacles. The obstacle avoider (potential field) uses detected objects to steer around them.

### Current obstacles

| Name | Shape | Position (x, y, z) | Size |
|---|---|---|---|
| `obstacle_box_1` | Box | (8, 3, 1) | 1 × 1 × 2 m |
| `obstacle_cylinder_1` | Cylinder | (10, 5, 1.5) | r=0.5 m, h=3 m |
| `obstacle_box_2` | Box | (5, 8, 0.75) | 2 × 0.5 × 1.5 m |

### Adding a new obstacle

Add a `<model>` block inside the `<world>` element in `sim/worlds/test_world.sdf`:

```xml
<!-- New obstacle: tall pillar at (15, 15) -->
<model name="obstacle_pillar_1">
  <static>true</static>
  <pose>15 15 2.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><cylinder><radius>0.3</radius><length>5</length></cylinder></geometry>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>0.3</radius><length>5</length></cylinder></geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

> **Note:** Obstacles must be within sensor detection range (~50 m) to be detected by the perception pipeline. Place them near the flight path for the avoidance algorithm to activate.

---

## 13. Viewing Logs

All process logs are written to `drone_logs/` (or the `LOG_DIR` you specify):

```bash
# Real-time log streaming
tail -f drone_logs/mission_planner.log

# All logs
ls -la drone_logs/
#   comms.log
#   gz_gui.log
#   mission_planner.log
#   payload_manager.log
#   perception.log
#   px4_sitl.log
#   slam_vio_nav.log
#   system_monitor.log
#   video_capture.log
```

### Key log messages to look for

```bash
# ARM + takeoff sequence
grep -i "arm\|takeoff\|navigate\|waypoint\|rtl\|land" drone_logs/mission_planner.log

# MAVLink connection status
grep -i "connected\|heartbeat\|armed\|altitude" drone_logs/comms.log

# PX4 boot progress
grep -i "ready\|home\|armed\|takeoff" drone_logs/px4_sitl.log
```

---

## 14. Troubleshooting

| Issue | Cause | Solution |
|---|---|---|
| `ERROR: Build directory not found` | Stack not built | Run `./deploy/build.sh` |
| `ERROR: PX4-Autopilot not found` | Wrong PX4 path | Set `PX4_DIR=/path/to/PX4-Autopilot` |
| `ERROR: PX4 SITL binary not found` | PX4 not built for SITL | Run `cd ~/PX4-Autopilot && make px4_sitl_default` |
| MAVLink port not detected after 30 s | PX4 boot failed | Check `drone_logs/px4_sitl.log` for errors |
| Gazebo GUI crashes with `libpthread` error | Conda/Snap `LD_LIBRARY_PATH` conflict | The launch script uses `env -i` to avoid this. If running `gz` manually, prefix with `env -i HOME=$HOME DISPLAY=$DISPLAY PATH=/usr/bin:/usr/local/bin:/bin` |
| Drone doesn't arm | MAVSDK heartbeat needs time | ARM retries every 3 s automatically. Wait ~10 s |
| Drone arms but doesn't take off | Takeoff command not sent | Check `mission_planner.log` for `Sending TAKEOFF` |
| Camera follow doesn't work | CameraTracking service takes time | Wait 10 s after GUI opens, or use Entity Tree → right-click drone → Move to |
| `gz service` commands time out | Gazebo GUI not running | Make sure `gz sim -g` is running |
| Tests fail with MAVSDK errors | MAVSDK not installed or wrong version | Run `pkg-config --modversion mavsdk` — must be 2.12.12 |
| Processes exit immediately | Config file not found | Verify `config/gazebo.json` exists and is valid JSON |

### Clean restart

If things get stuck, perform a full cleanup:

```bash
# Kill everything
pkill -9 -f "gz sim" ; pkill -9 -f px4 ; pkill -9 -f "ruby.*gz"
pkill -9 -f video_capture ; pkill -9 -f perception ; pkill -9 -f slam_vio_nav
pkill -9 -f mission_planner ; pkill -9 -f comms
pkill -9 -f payload_manager ; pkill -9 -f system_monitor

# Clean shared memory
rm -f /dev/shm/drone_* /dev/shm/detected_* /dev/shm/slam_* \
      /dev/shm/mission_* /dev/shm/trajectory_* /dev/shm/payload_* \
      /dev/shm/fc_* /dev/shm/gcs_* /dev/shm/system_*

# Relaunch
bash deploy/launch_gazebo.sh --gui
```

---

## File Reference

| File | Purpose |
|---|---|
| `deploy/launch_gazebo.sh` | Main launch script — starts PX4, Gazebo, GUI, and all 7 processes |
| `config/gazebo.json` | Simulation config — backends, waypoints, sensor params, comms settings |
| `sim/worlds/test_world.sdf` | Gazebo world — ground plane, landing pad, obstacles, lighting, physics |
| `sim/models/x500_companion/model.sdf` | Custom drone model — x500 + cameras + IMU |
| `sim/models/x500_companion/model.config` | Gazebo model metadata |
| `sim/gui.config` | Gazebo GUI client layout — chase-cam, camera tracking, entity tree |
| `docs/gazebo_setup.md` | Detailed environment setup guide (Gazebo, PX4, MAVSDK installation) |
