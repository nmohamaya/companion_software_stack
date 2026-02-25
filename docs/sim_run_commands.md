# Simulation Run Commands

Clean build and run the default Gazebo SITL flight scenario (~50 s total).

---

## 1. Clean & Build

```bash
cd ~/NM/Projects/companion_software_stack

# Remove old build artifacts
rm -rf build/

# Configure and build (Release mode)
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
cd ..
```

## 2. Run Tests

```bash
ctest --test-dir build --output-on-failure -j$(nproc)
```

## 3. Kill Any Existing Processes & Clean Shared Memory

```bash
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "$p" 2>/dev/null || true
done
sleep 2
rm -f /dev/shm/drone_* /dev/shm/detected_* /dev/shm/slam_* \
      /dev/shm/mission_* /dev/shm/trajectory_* /dev/shm/payload_* \
      /dev/shm/fc_* /dev/shm/gcs_* /dev/shm/system_* 2>/dev/null
```

## 4. Launch Simulation

```bash
# Headless (no GUI)
bash deploy/launch_gazebo.sh

# With Gazebo 3D GUI + chase-cam
bash deploy/launch_gazebo.sh --gui
```

## 5. Monitor Flight Progress

In a separate terminal:

```bash
# Watch the mission planner log in real time
tail -f drone_logs/mission_planner.log

# Or check key events after the flight
grep -iE "arm|takeoff|waypoint|rtl|land|disarm" drone_logs/mission_planner.log
```

## 6. Stop Everything

Press `Ctrl+C` in the launch terminal, or:

```bash
pkill -f "px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
for p in video_capture perception slam_vio_nav mission_planner comms payload_manager system_monitor; do
    pkill -f "$p" 2>/dev/null || true
done
rm -f /dev/shm/drone_* 2>/dev/null
```

---

## Expected Timeline

| Event | ~Time from launch |
|---|---|
| ARM | ~1 s |
| Takeoff complete (5 m) | ~13 s |
| WP 1 reached (15, 0, 5) | ~19 s |
| WP 2 reached (15, 15, 5) | ~25 s |
| WP 3 reached (0, 0, 5) → RTL | ~35 s |
| Landed & disarmed at origin | ~50 s |

**Flight plan:** 3 waypoints forming a 15 m triangle at 5 m AGL, 5 m/s cruise speed, 2.0 m acceptance radius. After the last waypoint the drone RTLs back to the takeoff point (RTL altitude set to 5 m to avoid the PX4 default 30 m climb).
