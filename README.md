# Drone Companion Computer Software Stack

Multi-process C++17 software stack for an autonomous drone companion computer. 7 independent Linux processes communicate via POSIX shared memory (SeqLock pattern).

## Architecture

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  P1 Video    │──▶│  P2 Percep-  │──▶│  P4 Mission  │
│  Capture     │   │  tion        │   │  Planner     │
│  (30 Hz)     │──▶│  (detect+    │   │  (FSM+path)  │
└──────────────┘   │  track+fuse) │   └──────┬───────┘
                   └──────────────┘          │
┌──────────────┐          ▲          ┌───────▼──────┐
│  P3 SLAM/    │──────────┘          │  P5 Comms    │
│  VIO/Nav     │────────────────────▶│  (MAVLink+   │
│  (100 Hz)    │                     │   GCS)       │
└──────────────┘                     └──────────────┘
                                            │
┌──────────────┐   ┌──────────────┐         │
│  P7 System   │   │  P6 Payload  │◀────────┘
│  Monitor     │   │  Manager     │
│  (1 Hz)      │   │  (gimbal+cam)│
└──────────────┘   └──────────────┘
```

### Processes

| # | Process | Description | Rate |
|---|---------|-------------|------|
| 1 | `video_capture` | Simulated mission + stereo camera capture | 30 Hz |
| 2 | `perception` | Object detection, Kalman tracking, sensor fusion | 15 Hz |
| 3 | `slam_vio_nav` | Visual-inertial odometry, pose estimation | 100 Hz |
| 4 | `mission_planner` | FSM waypoint navigation, obstacle avoidance | 10 Hz |
| 5 | `comms` | FC (MAVLink) + GCS (UDP) communication bridge | 10-20 Hz |
| 6 | `payload_manager` | Gimbal control + camera trigger | 50 Hz |
| 7 | `system_monitor` | CPU/mem/thermal monitoring, watchdog | 1 Hz |

### IPC via Shared Memory

All processes use SeqLock-based `ShmWriter<T>` / `ShmReader<T>` templates for zero-copy, lock-free inter-process communication. Data types are defined in `common/ipc/include/ipc/shm_types.h`.

## Prerequisites

```bash
# Ubuntu 22.04 / 24.04
sudo apt-get install -y build-essential cmake libspdlog-dev libeigen3-dev
```

## Build

```bash
# From the project root directory:
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

Or use the build script:
```bash
./deploy/build.sh          # Release build (default)
./deploy/build.sh Debug    # Debug build
```

Binaries are placed in `build/bin/`.

## Run

### Launch all processes (recommended)

The launch script handles SHM cleanup, library path fixes, and starts all 7 processes in the correct dependency order:

```bash
# As root (recommended — enables RT thread scheduling and CPU pinning):
sudo ./deploy/launch_all.sh

# As normal user (works fine, but thread affinity/RT scheduling will be skipped):
./deploy/launch_all.sh
```

Press `Ctrl+C` to gracefully stop all processes.

### Run individual processes

Start processes in dependency order — upstream producers first, then consumers:

```bash
# 1. No dependencies — start first
./build/bin/system_monitor &
./build/bin/video_capture &
./build/bin/comms &

# 2. Depends on video_capture SHM
./build/bin/perception &
./build/bin/slam_vio_nav &

# 3. Depends on slam_vio_nav + perception + comms SHM
./build/bin/mission_planner &

# 4. Depends on mission_planner SHM
./build/bin/payload_manager &
```

### Command-line options

All processes accept:
```
--log-level <level>   Set log level: trace/debug/info/warn/error (default: info)
--sim                 Simulation mode (default — all hardware is simulated)
--help                Show help
```

Example:
```bash
sudo ./deploy/launch_all.sh --log-level debug
```

### Logs

Logs are written to `/tmp/drone_logs/` with automatic rotation (5 MB per file, 3 rotated files). Console output uses colored spdlog formatting.

## Troubleshooting

### `GLIBCXX_3.4.30 not found` (Anaconda conflict)

**Symptom:**
```
./build/bin/video_capture: .../anaconda3/lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found
```

**Cause:** Anaconda ships an older `libstdc++.so.6` and its `LD_LIBRARY_PATH` takes precedence over the system library.

**Fix:** The launch script already handles this automatically by prepending the system library path. If running individual binaries manually, prefix the command:
```bash
LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" ./build/bin/video_capture
```

Or deactivate Anaconda before running:
```bash
conda deactivate
./deploy/launch_all.sh
```

### `Failed to create SHM` errors

**Symptom:**
```
[video_capture] [error] Failed to create SHM: /drone_mission_cam
```

**Cause:** Stale shared memory segments from a previous run (possibly owned by a different user or with incompatible permissions).

**Fix:** Remove leftover segments:
```bash
sudo rm -f /dev/shm/drone_* /dev/shm/detected_objects /dev/shm/slam_pose \
           /dev/shm/mission_status /dev/shm/trajectory_cmd /dev/shm/payload_commands \
           /dev/shm/fc_state /dev/shm/gcs_commands /dev/shm/payload_status \
           /dev/shm/system_health
```

The launch script does this automatically on every start. If you switch between running as root and as a normal user, you may need to clean them manually since root-owned segments can't be overwritten by a normal user.

### Thread affinity / RT scheduling warnings

**Symptom:**
```
[warn] setaffinity(0) failed: 22
[warn] setschedparam(1,50) failed: 1
```

**Cause:** Thread pinning (`pthread_setaffinity_np`) and real-time scheduling (`SCHED_FIFO`) require elevated privileges.

**Fix:** Run as root:
```bash
sudo ./deploy/launch_all.sh
```

These warnings are **harmless** — the stack runs correctly without RT scheduling, just without hard real-time guarantees. On a production Jetson, the systemd services would run with `CAP_SYS_NICE` capability.

### Processes exit immediately

**Cause:** A process can't connect to an upstream SHM segment (e.g., `mission_planner` starts before `slam_vio_nav` has created its SHM).

**Fix:** Use the launch script which starts processes in the correct order with 500ms delays. If running manually, ensure producers are started before consumers (see dependency order above).

## Project Structure

```
.
├── CMakeLists.txt                    # Super-build
├── common/
│   ├── ipc/                          # Shared memory IPC library
│   │   └── include/ipc/
│   │       ├── shm_writer.h          # SeqLock writer template
│   │       ├── shm_reader.h          # SeqLock reader template
│   │       └── shm_types.h           # All IPC data structures
│   └── util/                         # Utility library
│       └── include/util/
│           ├── signal_handler.h      # Graceful SIGINT/SIGTERM
│           ├── arg_parser.h          # CLI argument parsing
│           ├── log_config.h          # spdlog configuration
│           ├── realtime.h            # Thread naming/affinity/RT
│           ├── scoped_timer.h        # RAII timing
│           └── spsc_ring.h           # Lock-free ring buffer
├── process1_video_capture/
├── process2_perception/
├── process3_slam_vio_nav/
├── process4_mission_planner/
├── process5_comms/
├── process6_payload_manager/
├── process7_system_monitor/
└── deploy/
    ├── build.sh                      # Build script
    └── launch_all.sh                 # Launch all processes
```

## Simulation Mode

All hardware dependencies (V4L2 cameras, TensorRT, CUDA, GTSAM, MAVLink serial, etc.) are replaced with simulation stubs that generate synthetic data. This allows the full stack to compile and run on any Linux system with only spdlog and Eigen3 installed.

On real hardware (NVIDIA Jetson), replace simulation stubs with actual driver code.
