# Getting Started — Companion Software Stack

This guide takes you from a **fresh clone to a working build** in under 10 minutes. If you're new to this project, start here.

---

## Prerequisites

### System Requirements

- **OS:** Ubuntu 22.04 LTS or Ubuntu 24.04 LTS
- **Architecture:** x86-64 (development) or aarch64 (Jetson Orin)
- **Resources:** 4 GB RAM (minimum), 8+ GB disk space, 2 cores (minimum)
- **User:** Can be any user; `sudo` access required for real-time scheduling (optional)

> **Conda/Anaconda Warning:** If you have Conda installed, run `conda deactivate` before building. Conda ships its own `libfmt`, `libstdc++`, and `GTest` that conflict with system libraries and cause linker errors. See [INSTALL.md](INSTALL.md) for details.

### Check Your System

```bash
# Verify Ubuntu version
lsb_release -a       # Should show 22.04 or 24.04

# Verify architecture
uname -m             # x86_64 or aarch64

# Check available resources
free -h              # RAM
df -h /              # Disk space
nproc                # CPU cores
```

---

## 1. Clone the Repository

```bash
git clone https://github.com/nmohamaya/companion_software_stack.git
cd companion_software_stack
git log --oneline -1  # Verify clone worked
```

---

## 2. Install Dependencies

The project requires **core dependencies** (always) and **optional dependencies** (for advanced features).

### Option A: Automated (Recommended)

The project includes an interactive dependency installer:

```bash
# Interactive mode — prompts for each optional feature
bash deploy/install_dependencies.sh

# Non-interactive — install everything
bash deploy/install_dependencies.sh --all

# Core only — build without optional features (faster)
bash deploy/install_dependencies.sh --core-only
```

### Option B: Manual Install (Quick Reference)

**Core dependencies (required):**
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    unzip \
    libspdlog-dev \
    libfmt-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libgtest-dev
```

**Zenoh IPC (required):**
```bash
# Zenoh is the sole IPC backend — not available via apt
# See docs/guides/INSTALL.md Section 3 for full instructions
ZENOH_VERSION="1.7.2"
wget "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip"
unzip libzenohc-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip
sudo dpkg -i libzenohc_*.deb libzenohc-dev_*.deb && sudo ldconfig
rm -f libzenohc-*.zip libzenohc_*.deb libzenohc-dev_*.deb
# Also install zenoh-cpp headers — see INSTALL.md Section 3.2
```

**Optional (recommended):**
```bash
# OpenCV (for YOLOv8 object detection)
sudo apt-get install -y libopencv-dev

# MAVSDK (for PX4 MAVLink communication)
# Build from source — see docs/guides/INSTALL.md Section 5

# Gazebo (for SITL simulation)
sudo apt-get install -y gz-harmonic libgz-transport13-dev libgz-msgs10-dev
```

> **Note:** If you choose `--core-only`, you can still build and test everything — optional backends are auto-detected and skipped gracefully.

---

## 3. Build the Project

The simplest approach:

```bash
# From the project root:
bash deploy/build.sh
```

This runs:
1. Creates `build/` directory if needed
2. Runs CMake with Release mode + Zenoh enabled (`-DALLOW_INSECURE_ZENOH=ON`)
3. Compiles with `-Werror -Wall -Wextra` (zero warnings gate)
4. Places binaries in `build/bin/`

**Build takes ~2–5 minutes on a typical machine.**

### Alternative Build Commands

```bash
# Debug build (slower but better for troubleshooting)
bash deploy/build.sh Debug

# Clean rebuild (removes build/ first)
bash deploy/build.sh --clean

# With AddressSanitizer (memory safety checks)
bash deploy/build.sh --asan
```

---

## 4. Verify the Build

After building, verify that your build is correct:

```bash
# Expected: should print test count matching tests/TESTS.md baseline
ctest -N --test-dir build | grep "Total Tests:"

# Run all tests (takes ~1–2 minutes)
bash tests/run_tests.sh

# Expected: all tests PASS, 0 failures (see tests/TESTS.md for count)
```

If you see fewer tests than expected, clean and rebuild:
```bash
bash deploy/build.sh --clean
```

---

## 5. Run the Stack

### Quickest: Standalone Simulation (No Dependencies)

Runs all 7 processes with **simulated sensors** (synthetic data, no Gazebo needed):

```bash
# Launch all processes
bash deploy/launch_all.sh

# Expected: processes start, logs to console
# Press Ctrl+C to stop all processes gracefully
```

### With Gazebo SITL (More Realistic Simulation)

Runs a **software-in-the-loop simulation** with PX4 flight controller + Gazebo visualizer:

```bash
# Simpler: clean build + test + launch Gazebo all in one
bash deploy/clean_build_and_run.sh --gui

# Or just launch Gazebo (if already built)
bash deploy/launch_gazebo.sh --gui
```

This displays:
- **Gazebo 3D window** with simulated drone, environment, camera feeds
- **Console output** from all 7 processes
- **Autonomous flight** — mission planner executes waypoints, detects obstacles, avoids them

Expected behavior:
- Drone arms, takes off to altitude
- Navigates between 3 waypoints
- Avoids obstacles
- Returns home

### Run a Specific Scenario (Advanced Testing)

The project includes pre-defined scenario tests that exercise specific features (perception, obstacle avoidance, etc.):

```bash
# Run a particular scenario with Gazebo visualizer
bash tests/run_scenario_gazebo.sh config/scenarios/18_perception_avoidance.json --verbose --gui
```

**What this does:**
- Runs the "perception & avoidance" scenario (18_perception_avoidance.json)
- Displays Gazebo 3D window with realistic obstacles
- Shows console output with detailed logs (`--verbose`)
- Generates a timestamped run report in `drone_logs/scenarios_gazebo/`

**Available scenarios** (see `config/scenarios/` folder):
- `18_perception_avoidance.json` — Sensor-driven obstacle avoidance (no HD-map; camera+radar detect obstacles at runtime)
- Other scenarios for waypoint following, GPS loss, battery warnings, etc.

**Scenario options:**
```bash
bash tests/run_scenario_gazebo.sh <scenario_file> [OPTIONS]

Options:
  --verbose        Detailed console output
  --gui            Show Gazebo 3D visualizer
  --headless       Run without GUI (faster)
```

Each run generates a timestamped report with mission summary, timing, and any failures.

---

## 6. Next Steps

### ✅ Everything Works? Great! Now:

1. **Read the main README.md** for architecture deep-dive
2. **Check [DEVELOPMENT_WORKFLOW.md](DEVELOPMENT_WORKFLOW.md)** if you want to contribute
3. **Explore the config file** — all tunables in `config/default.json`
   ```bash
   # Modify parameters (e.g., camera resolution, detect confidence, planner speed)
   nano config/default.json
   
   # Rebuild and re-run to see effects
   bash deploy/build.sh
   bash deploy/launch_gazebo.sh --gui
   ```

4. **Run a specific test suite** (e.g., perception tests only)
   ```bash
   bash tests/run_tests.sh perception
   ```

5. **View code coverage** of tests (how much code is tested)
   ```bash
   bash deploy/view_coverage.sh  # Generates HTML report, opens in browser
   ```

6. **Real hardware?** See [docs/HARDWARE_SETUP.md](HARDWARE_SETUP.md) for Jetson Orin setup

---

## Troubleshooting

### Build Fails: `CMake not found`

```bash
sudo apt-get install -y cmake
```

### Build Fails: `Missing <spdlog/spdlog.h>`

```bash
sudo apt-get install -y libspdlog-dev libeigen3-dev nlohmann-json3-dev libgtest-dev
# Then clean rebuild:
bash deploy/build.sh --clean
```

### Build Fails: `GLIBCXX_3.4.30 not found` (Anaconda conflict)

**Cause:** Anaconda ships an older `libstdc++.so.6`

**Fix:**
```bash
conda deactivate                              # Disable Anaconda
bash deploy/launch_all.sh                     # Try again
```

Or set library path explicitly:
```bash
LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" ./build/bin/video_capture
```

### Processes Exit Immediately

**Cause:** Stale shared memory segments from a previous run

**Fix:**
```bash
sudo rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
           /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
           /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
           /dev/shm/system_health

# Re-run:
bash deploy/launch_all.sh
```

The launch scripts do this automatically, but manual cleanup is sometimes needed when switching user accounts.

### Gazebo Window Doesn't Open

**Cause:** Display server issue (common in headless/SSH environments)

**Fix:**
```bash
# Run headless (no GUI)
bash deploy/launch_gazebo.sh              # omit --gui

# Or use VNC/X11 forwarding for SSH sessions
# See docs/DEBUG.md for details
```

### Test Count Mismatch (doesn't match tests/TESTS.md baseline?)

**Cause:** Stale CMake cache from different build types (Release vs Debug)

**Fix:**
```bash
# Clean build
bash deploy/build.sh --clean

# Verify count again
ctest -N --test-dir build | grep "Total Tests:"
```

---

## File Tree for Reference

```
companion_software_stack/
  ├── README.md                    ← Read next (architecture overview)
  ├── common/
  │   ├── hal/                     ← Hardware Abstraction Layer (interfaces + backends)
  │   ├── ipc/                     ← Zenoh IPC (publisher/subscriber/MessageBus)
  │   └── util/                    ← Config, Result<T,E>, logging, watchdog
  ├── process1_video_capture/      ← P1: Camera frame acquisition
  ├── process2_perception/         ← P2: Detection + tracking + sensor fusion
  ├── process3_slam_vio_nav/       ← P3: Visual-inertial odometry + navigation
  ├── process4_mission_planner/    ← P4: FSM + path planning + obstacle avoidance
  ├── process5_comms/              ← P5: Flight controller & GCS communication
  ├── process6_payload_manager/    ← P6: Gimbal & camera control
  ├── process7_system_monitor/     ← P7: Health monitoring & process supervision
  ├── config/
  │   └── default.json             ← All tunables (camera res, detect threshold, etc.)
  ├── deploy/
  │   ├── build.sh                 ← Main build script
  │   ├── launch_all.sh            ← Run all 7 processes
  │   ├── launch_gazebo.sh         ← Run with Gazebo SITL
  │   └── install_dependencies.sh  ← Install deps
  ├── tests/
  │   ├── run_tests.sh             ← Run all tests (see TESTS.md for count)
  │   └── TESTS.md                 ← Test inventory
  └── docs/
      ├── guides/
      │   ├── GETTING_STARTED.md   ← You are here
      │   ├── INSTALL.md           ← Detailed installation guide
      │   └── DEVELOPMENT_WORKFLOW.md ← Read if you want to contribute
      └── design/
          └── API.md               ← IPC message types
```

---

## Typical First Session

```bash
# 1. Clone (1 min)
git clone https://github.com/nmohamaya/companion_software_stack.git
cd companion_software_stack

# 2. Install deps
#    Core only: ~5 min (apt + Zenoh .deb)
#    Full (+ OpenCV source + MAVSDK source + Gazebo + PX4): ~1-2 hours
bash deploy/install_dependencies.sh --all

# 3. Build (3-5 min)
bash deploy/build.sh

# 4. Test (1-2 min)
bash tests/run_tests.sh

# 5. Run simulated stack (no Gazebo needed)
bash deploy/launch_all.sh

# 6. Run with Gazebo SITL (requires Gazebo + MAVSDK + PX4)
bash deploy/launch_gazebo.sh --gui
```

**Total time:** ~10 minutes for core-only build+test, ~1-2 hours for full environment with all optional deps.

---

## Questions?

- **How do I contribute?** → [DEVELOPMENT_WORKFLOW.md](DEVELOPMENT_WORKFLOW.md) (Steps 1-9)
- **What's the architecture?** → [README.md](../../README.md#architecture)
- **Detailed installation/troubleshooting?** → [INSTALL.md](INSTALL.md)
- **Real hardware setup?** → [README.md](../../README.md#launch-on-real-hardware)
- **IPC message types?** → [API.md](../design/API.md)

---

**Ready?** Start with Section 1: Clone the Repository. ✨
