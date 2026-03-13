# Process 6 — Payload Manager: Design Document

> **Scope**: Detailed design of the Payload Manager process (`process6_payload_manager`).
> This document covers gimbal control, camera payload commands, and the single-threaded
> control loop.

---

## Table of Contents

1. [Overview](#overview)
2. [Thread Architecture](#thread-architecture)
3. [IPC Channels](#ipc-channels)
4. [Component: IGimbal (HAL)](#component-igimbal-hal)
5. [Gimbal Control Model](#gimbal-control-model)
6. [Payload Actions](#payload-actions)
7. [Main Loop Detail](#main-loop-detail)
8. [Configuration Reference](#configuration-reference)
9. [Testing](#testing)
10. [Known Limitations](#known-limitations)

---

## Overview

Process 6 manages the camera gimbal and camera payload actions (image capture,
video recording). It runs a single control loop at 50 Hz (configurable) that processes
gimbal pointing commands from the Mission Planner and publishes gimbal status.

---

## Thread Architecture

| Thread | Rate | Role |
|--------|------|------|
| Main loop | 50 Hz | Gimbal control + payload command processing |

P6 is the simplest process in the stack — a single thread with no concurrency.
The `ThreadWatchdog` runs its scan thread internally (1 Hz) as usual.

```
┌───────────────────────────────────────────────────────┐
│                 Payload Manager (P6)                   │
│                                                       │
│  ShmPayloadCommand ──────────────────┐                │
│  (from P4)                           │                │
│                                      ▼                │
│                              ┌──────────────┐         │
│                              │  Main Loop   │         │
│                              │  (50 Hz)     │         │
│                              │              │         │
│                              │ 1. Poll cmd  │         │
│                              │ 2. set_target│─► IGimbal│
│                              │ 3. update(dt)│   (HAL) │
│                              │ 4. publish   │         │
│                              └──────┬───────┘         │
│                                     │                 │
│                                     ▼                 │
│                            ShmPayloadStatus           │
│                            (to P4, P7)                │
└───────────────────────────────────────────────────────┘
```

---

## IPC Channels

### Subscriptions (inputs)

| Channel | SHM Segment | Type | Source |
|---------|-------------|------|--------|
| Payload commands | `/payload_commands` | `ShmPayloadCommand` | P4 (mission_planner) |

### Publications (outputs)

| Channel | SHM Segment | Type | Consumers |
|---------|-------------|------|-----------|
| Payload status | `/payload_status` | `ShmPayloadStatus` | P4, P7 |
| Thread health | `/drone_thread_health_payload_manager` | `ShmThreadHealth` | P7 |

---

## Component: IGimbal (HAL)

- **Interface:** [`igimbal.h`](../common/hal/include/hal/igimbal.h)

### Interface Methods

```cpp
class IGimbal {
    virtual bool init() = 0;
    virtual bool is_initialised() const = 0;
    virtual void set_target(float pitch_deg, float yaw_deg) = 0;
    virtual void update(float dt_s) = 0;
    virtual GimbalState state() const = 0;
    virtual uint64_t capture_image() = 0;
    virtual void start_recording() = 0;
    virtual void stop_recording() = 0;
    virtual bool is_recording() const = 0;
    virtual std::string name() const = 0;
};
```

### GimbalState Structure

```cpp
struct GimbalState {
    float pitch;        // degrees
    float yaw;          // degrees
    float roll;         // degrees (always 0 in sim)
    bool  stabilised;   // true when close to target
};
```

### Backends

| Backend | Config value | Class | Description |
|---------|-------------|-------|-------------|
| Simulated | `"simulated"` | `SimulatedGimbal` | Software gimbal model with slew rate |

---

## Gimbal Control Model

The simulated gimbal uses a first-order slew-rate-limited model:

### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max slew rate | 60°/s | Maximum angular velocity |
| Pitch range | [−90°, +30°] | Clamped on `set_target()` |
| Yaw range | [−180°, +180°] | Clamped on `set_target()` |
| Roll | 0° | Fixed (no roll control) |

### Update Step

On each `update(dt_s)` call:

```
max_step = 60.0 × dt     // degrees this tick
error    = target − current
step     = clamp(error, −max_step, +max_step)
current += step
```

At 50 Hz (`dt = 0.02s`), the maximum step per tick is 1.2°. A 90° slew takes ~75 ticks
(1.5 seconds).

### Stabilisation Flag

`stabilised = true` when `|pitch_error| < 0.1° && |yaw_error| < 0.1°`.

---

## Payload Actions

The Mission Planner sends `ShmPayloadCommand` with one of these actions:

| Action | Value | Gimbal Effect | Camera Effect |
|--------|-------|---------------|---------------|
| `NONE` | 0 | No change | No change |
| `GIMBAL_POINT` | 1 | Set target pitch/yaw | — |
| `CAMERA_CAPTURE` | 2 | — | `capture_image()` (snapshot) |
| `CAMERA_START_VIDEO` | 3 | — | `start_recording()` |
| `CAMERA_STOP_VIDEO` | 4 | — | `stop_recording()` |

### ShmPayloadCommand Structure

```cpp
struct ShmPayloadCommand {
    uint64_t      timestamp_ns;
    uint64_t      correlation_id;
    PayloadAction action;
    float         gimbal_pitch;   // target pitch (degrees)
    float         gimbal_yaw;     // target yaw (degrees)
    uint64_t      sequence_id;
    bool          valid;
};
```

### ShmPayloadStatus Structure

```cpp
struct ShmPayloadStatus {
    uint64_t timestamp_ns;
    float    gimbal_pitch;       // current pitch (degrees)
    float    gimbal_yaw;         // current yaw (degrees)
    uint32_t images_captured;    // lifetime capture count
    bool     recording_video;    // currently recording
    bool     gimbal_stabilized;  // at target within tolerance
    uint8_t  num_plugins_active; // (reserved for future use)
};
```

---

## Main Loop Detail

### Startup Sequence

1. Parse command-line args and load `config/default.json`
2. Create `MessageBus` via factory (SHM or Zenoh)
3. Create gimbal via `drone::hal::create_gimbal(cfg, "payload_manager.gimbal")`
4. Call `gimbal->init()`
5. Subscribe to `/payload_commands` channel
6. Create publisher for `/payload_status` channel
7. Create `ThreadWatchdog` (1 thread: main loop)
8. `sd_notify(READY=1)`
9. Enter main control loop

### Control Loop (50 Hz)

```
while (g_running):
    touch_heartbeat()

    if poll(ShmPayloadCommand) and cmd.valid:
        switch cmd.action:
            GIMBAL_POINT:      gimbal->set_target(cmd.gimbal_pitch, cmd.gimbal_yaw)
            CAMERA_CAPTURE:    gimbal->capture_image()
            CAMERA_START_VIDEO: gimbal->start_recording()
            CAMERA_STOP_VIDEO:  gimbal->stop_recording()

    gimbal->update(dt)

    status.gimbal_pitch     = gimbal->state().pitch
    status.gimbal_yaw       = gimbal->state().yaw
    status.gimbal_stabilized = gimbal->state().stabilised
    status.images_captured  = <capture_count>
    status.recording_video  = gimbal->is_recording()

    publish(ShmPayloadStatus)
    sleep_until(next_tick)
```

### Shutdown

- Signal handler sets `g_running = false`
- Loop exits, gimbal destroyed via RAII
- Thread health publisher stopped

---

## Configuration Reference

### `payload_manager.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `update_rate_hz` | int | 50 | Control loop frequency |

### `payload_manager.gimbal.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Gimbal HAL backend |

---

## Testing

| Test File | Tests | Coverage |
|-----------|-------|----------|
| [`test_payload_manager.cpp`](../tests/test_payload_manager.cpp) | 9 | GimbalController: init, clamping, slew, convergence, capture, recording |
| **Total** | **9** | |

### Key Test Scenarios

- **Initial state:** pitch=0, yaw=0, roll=0, stabilised=true, not recording
- **Target clamping:** pitch 100° → clamped to 30°; pitch −120° → clamped to −90°
- **Smooth motion:** One update tick moves toward target but doesn't jump to it
- **Convergence:** After 200 ticks × 0.02s = 4s, gimbal reaches target within ±0.01°
- **Capture timestamp:** `capture_image()` returns a non-zero nanosecond timestamp
- **Recording lifecycle:** start → `is_recording() == true` → stop → `false`
- **Zero dt:** `update(0.0)` does not change position

---

## Observability

P6 subscribes to SLAM pose for gimbal geo-pointing and publishes payload
status consumed by P4 (payload triggers) and P7 (health monitoring).

### Structured Logging

| JSON Field | Description |
|------------|-------------|
| `process` | `"payload_manager"` |
| `gimbal_pitch_deg` | Current gimbal pitch (degrees, positive = up) |
| `gimbal_yaw_deg` | Current gimbal yaw (degrees) |
| `gimbal_stabilised` | `true` when gimbal has converged to target |
| `is_recording` | `true` when video recording is active |

### Correlation IDs

P6 does not participate in GCS correlation (payload commands arrive via
the `/slam_pose` trigger flag, not via `/gcs_commands`).

### Latency Tracking

| Channel | Direction | Tracker call |
|---------|-----------|-------------|
| `/slam_pose` | subscriber | `reader.log_latency_if_due(50)` in control loop |

See [observability.md](observability.md) for histogram interpretation.

---

## Known Limitations

1. **No real gimbal backend:** Only simulated gimbal exists. SIYI, Gremsy, or
   PWM servo backends would be needed for real hardware.
2. **No stabilisation feedback:** The stabilisation flag is purely position-based.
   Real gimbals report IMU-based stabilisation quality.
3. **Single gimbal model:** The control model doesn't account for inertia, backlash,
   or motor saturation. The slew rate is constant regardless of load.
4. **No camera image data:** `capture_image()` returns a timestamp but no actual image
   data. The payload camera is separate from the mission/stereo cameras in P1.
5. **No plugin system:** The `num_plugins_active` field in `ShmPayloadStatus` is
   reserved but no plugin loading mechanism exists.
