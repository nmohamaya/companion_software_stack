# Process 1 — Video Capture: Design Document

> **Scope**: Detailed design of the Video Capture process (`process1_video_capture`).
> This document covers camera HAL, frame types, stereo synchronisation, and the
> two-camera threading model.

---

## Table of Contents

1. [Overview](#overview)
2. [Thread Architecture](#thread-architecture)
3. [IPC Channels](#ipc-channels)
4. [Component: ICamera (HAL)](#component-icamera-hal)
5. [Frame Types](#frame-types)
6. [Stereo Synchronisation](#stereo-synchronisation)
7. [Main Loop Detail](#main-loop-detail)
8. [Configuration Reference](#configuration-reference)
9. [Testing](#testing)
10. [Known Limitations](#known-limitations)

---

## Overview

Process 1 acquires video frames from two cameras and publishes them to the IPC bus.
It runs **3 threads**: one for each camera plus the main health/watchdog thread.

| Camera | Output | Consumer |
|--------|--------|----------|
| Mission camera | RGB frames (1920×1080 @ 30 fps) | P2 (perception) |
| Stereo camera | Grayscale pair (640×480 @ 30 fps) | P3 (SLAM/VIO) |

---

## Thread Architecture

| Thread | Rate | Role |
|--------|------|------|
| `mission_cam_thread` | 30 Hz (configurable) | Captures RGB frames, publishes `ShmVideoFrame` |
| `stereo_cam_thread` | 30 Hz (configurable) | Captures left+right grayscale, validates sync, publishes `ShmStereoFrame` |
| Main thread | 1 Hz | ThreadHealthPublisher + systemd watchdog notify |

```
┌───────────────────────────────────────────────────────────────┐
│                   Video Capture (P1)                          │
│                                                               │
│  ┌─────────────────┐                ┌─────────────────────┐   │
│  │ mission_cam     │                │ stereo_cam          │   │
│  │ thread (30 Hz)  │                │ thread (30 Hz)      │   │
│  │                 │                │                     │   │
│  │ ICamera::       │                │ ICamera:: (left)    │   │
│  │  capture()      │                │  capture()          │   │
│  │    │            │                │    │                 │   │
│  │    ▼            │                │ ICamera:: (right)   │   │
│  │ ShmVideoFrame   │                │  capture()          │   │
│  │  (publish)      │                │    │                 │   │
│  │    │            │                │ Sync check (±5ms)   │   │
│  │    ▼            │                │    │                 │   │
│  │ P2 perception   │                │    ▼                 │   │
│  │                 │                │ ShmStereoFrame       │   │
│  └─────────────────┘                │  (publish)           │   │
│                                     │    │                 │   │
│                                     │    ▼                 │   │
│                                     │ P3 SLAM/VIO         │   │
│                                     └─────────────────────┘   │
│                                                               │
│  ┌───────────────────────────────────┐                        │
│  │ Main Thread (1 Hz)                │                        │
│  │ ThreadHealthPublisher + watchdog  │                        │
│  └───────────────────────────────────┘                        │
└───────────────────────────────────────────────────────────────┘
```

---

## IPC Channels

### Publications (outputs)

| Channel | SHM Segment | Type | Consumer |
|---------|-------------|------|----------|
| Mission camera | `/drone_mission_cam` | `ShmVideoFrame` | P2 (perception) |
| Stereo camera | `/drone_stereo_cam` | `ShmStereoFrame` | P3 (SLAM/VIO) |
| Thread health | `/drone_thread_health_video_capture` | `ShmThreadHealth` | P7 |

P1 has no subscriptions — it is a pure source in the IPC graph.

---

## Component: ICamera (HAL)

- **Interface:** [`icamera.h`](../common/hal/include/hal/icamera.h)

### Interface Methods

```cpp
class ICamera {
    virtual bool open(int width, int height, int fps) = 0;
    virtual CapturedFrame capture() = 0;   // Blocking call
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual std::string name() const = 0;
};
```

### CapturedFrame Structure

```cpp
struct CapturedFrame {
    uint64_t  timestamp_ns;   // steady_clock timestamp
    uint32_t  sequence;       // Monotonically increasing
    uint32_t  width, height;
    uint32_t  channels;       // 1 = GRAY, 3 = RGB
    uint32_t  stride;         // bytes per row
    uint8_t*  data;           // Valid until next capture() call
    bool      valid;          // false if camera closed or error
};
```

### Backends

| Backend | Config value | Class | Description |
|---------|-------------|-------|-------------|
| Simulated | `"simulated"` | `SimulatedCamera` | Synthetic gradient-pattern frames |
| Gazebo | `"gazebo"` | `GazeboCamera` | gz-transport image subscription (requires `HAVE_GAZEBO`) |
| V4L2 | `"v4l2"` | `V4L2Camera` | Real USB/CSI cameras (requires hardware) |

### SimulatedCamera Behaviour

- Generates a horizontal gradient pattern (pixel value = `x % 256`)
- Channel count determined by resolution: ≤640×480 → 1 (grayscale), >640×480 → 3 (RGB)
- Frame data stored in internal buffer — pointer valid only until the next `capture()` call
- Sequence counter increments on each capture
- `set_channels(uint32_t)` available for test overrides

---

## Frame Types

### ShmVideoFrame (Mission Camera → P2)

```cpp
struct ShmVideoFrame {
    uint64_t timestamp_ns;
    uint32_t sequence_number;
    uint32_t width;             // default 1920
    uint32_t height;            // default 1080
    uint32_t channels;          // 3 (RGB)
    uint32_t stride;            // width × channels
    uint8_t  pixel_data[1920 * 1080 * 3];  // ~6 MB
};
```

### ShmStereoFrame (Stereo Camera → P3)

```cpp
struct ShmStereoFrame {
    uint64_t timestamp_ns;
    uint32_t sequence_number;
    uint32_t width;             // default 640
    uint32_t height;            // default 480
    uint8_t  left_data[640 * 480];    // ~300 KB grayscale
    uint8_t  right_data[640 * 480];   // ~300 KB grayscale
};
```

### FrameMetadata (Internal)

```cpp
struct FrameMetadata {
    uint64_t   timestamp_ns;
    uint32_t   sequence_number;
    CameraId   camera_id;       // MISSION, STEREO_L, STEREO_R
    PixelFormat format;
    uint32_t   width, height, stride;
    float      exposure_us;
    float      gain_db;
    float      temperature_c;   // sensor temperature
};
```

### Pixel Formats

| Format | Value | Description |
|--------|-------|-------------|
| `BAYER_RGGB8` | 0 | Raw Bayer (real cameras) |
| `BAYER_GRBG8` | 1 | Raw Bayer variant |
| `YUV422` | 2 | YUV 4:2:2 |
| `RGB24` | 3 | 8-bit RGB |
| `NV12` | 4 | YUV 4:2:0 semi-planar |
| `GRAY8` | 5 | 8-bit grayscale |

---

## Stereo Synchronisation

The `stereo_cam_thread` must synchronise left and right camera captures:

1. **Capture left frame** via `left_cam->capture()`
2. **Capture right frame** via `right_cam->capture()`
3. **Validate both:** If either frame has `valid == false` or `data == nullptr`, drop the pair
4. **Temporal sync check:** `|left.timestamp_ns − right.timestamp_ns| ≤ 5 ms`
   - If sync fails, log warning and drop the pair
5. **Copy pixel data** into `ShmStereoFrame.left_data[]` and `right_data[]`
6. **Publish** the `ShmStereoFrame`

The 5 ms threshold is hardcoded. Real stereo cameras typically have hardware sync
(global shutter trigger), making this check informational rather than corrective.

---

## Main Loop Detail

### Startup Sequence

1. Parse command-line args and load `config/default.json`
2. Create `MessageBus` via factory (SHM or Zenoh)
3. Create mission camera via `drone::hal::create_camera(cfg, "video_capture.mission_cam")`
4. Create left stereo camera via `drone::hal::create_camera(cfg, "video_capture.stereo_cam_left")`
5. Create right stereo camera via `drone::hal::create_camera(cfg, "video_capture.stereo_cam_right")`
6. Open all cameras with configured resolution and FPS
7. Create IPC publishers for mission and stereo channels
8. Create `ThreadWatchdog` (2 threads: mission_cam, stereo_cam)
9. Launch `mission_cam_thread` and `stereo_cam_thread`
10. `sd_notify(READY=1)` — signal systemd readiness
11. Main loop: 1 Hz health publish + watchdog check

### Shutdown

- Signal handler sets `g_running = false`
- Camera threads exit capture loops
- Cameras closed via RAII
- Threads joined

---

## Configuration Reference

### `video_capture.mission_cam.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Camera HAL backend |
| `width` | int | 1920 | Capture width (pixels) |
| `height` | int | 1080 | Capture height (pixels) |
| `fps` | int | 30 | Capture frame rate |

### `video_capture.stereo_cam_left.*` / `stereo_cam_right.*`

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backend` | string | `"simulated"` | Camera HAL backend |
| `width` | int | 640 | Capture width (pixels) |
| `height` | int | 480 | Capture height (pixels) |
| `fps` | int | 30 | Capture frame rate |

---

## Testing

| Test File | Tests | Coverage |
|-----------|-------|----------|
| [`test_hal.cpp`](../tests/test_hal.cpp) | 7 | SimulatedCamera lifecycle, capture, sequence, factory |
| [`test_gazebo_camera.cpp`](../tests/test_gazebo_camera.cpp) | 13 | Gazebo camera backend (conditional on `HAVE_GAZEBO`) |
| **Total** | **20** | |

### Key Test Scenarios

- **Open/close lifecycle:** `open()` → `is_open() == true` → `close()` → `is_open() == false`
- **Capture validity:** Returned frame has non-null data, correct dimensions, valid timestamp
- **Sequence monotonicity:** Each successive capture increments sequence number
- **Closed capture:** `capture()` on closed camera returns `valid == false`
- **Factory default:** Missing `backend` key defaults to `"simulated"`
- **Unknown backend:** Throws `std::runtime_error`

---

## Known Limitations

1. **No debayering:** Raw Bayer formats (real cameras) are published as-is. No ISP
   pipeline (debayer → white balance → denoise) exists in the capture process.
2. **Sequential stereo capture:** Left and right cameras are captured sequentially,
   not in parallel. With real cameras, this introduces a small timing offset.
3. **Large SHM frames:** `ShmVideoFrame` is ~6 MB. Publishing at 30 fps generates
   ~180 MB/s of SHM writes. The SPSC ring buffer handles this, but it limits the
   number of buffered frames.
4. **No frame drop detection:** If the consumer (P2 or P3) falls behind, old frames
   are silently overwritten in the SPSC ring. No frame-drop counter is published.
5. **Hardcoded stereo sync threshold:** The 5 ms threshold is not configurable.
