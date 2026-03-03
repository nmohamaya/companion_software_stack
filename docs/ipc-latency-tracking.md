# IPC Latency Tracking

Companion Software Stack · Observability Guide

---

## Why Latency Monitoring Matters

In a real-time drone stack every IPC hop adds delay between sensing and
acting.  If the video-capture → perception → mission-planner pipeline
exceeds the control loop budget, the aircraft can overshoot waypoints,
lose tracking lock, or miss obstacle detections entirely.

The built-in latency tracker lets you **continuously measure
publisher-to-subscriber delay** on every IPC channel — SHM and Zenoh —
with negligible overhead, so you can:

| Goal | Question answered |
|---|---|
| Detect regressions | "Did my last code change add latency?" |
| Budget allocation  | "How much of the 33 ms frame budget does IPC consume?" |
| Tail-latency hunts | "Is p99 > 5× p50?  Am I scheduling poorly?" |
| Deployment tuning  | "Is the Jetson Orin slower than my dev laptop?" |
| Compare transports | "SHM vs Zenoh latency for the same payload?" |

---

## Architecture Overview

```
┌────────────┐    write(data)     ┌────────────┐
│ ShmWriter  │ ──── SHM ────────▶ │ ShmReader  │
│            │  stamps            │            │
│            │  timestamp_ns      │            │
│            │  via now_ns()      │  read(out, &ts)
└────────────┘                    └─────┬──────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │ShmSubscriber  │
                                 │  receive()    │
                                 │  ├─ read()    │
                                 │  └─ record(   │──▶ LatencyTracker
                                 │     now-ts)   │     (ring buffer)
                                 └──────────────┘

Same pattern for ZenohPublisher → ZenohSubscriber, **with one
important caveat**: ZenohPublisher does **not** embed a send
timestamp in the wire payload.  Instead, ZenohSubscriber stamps
`steady_clock::now()` when the Zenoh callback delivers the
sample.  Therefore the Zenoh "latency" measures **callback → poll
delay** (consumer processing lag), not true publisher-to-subscriber
wire latency.  See the [Limitations](#known-limitations) section.
```

**How it works:**

1. **SHM path (true publisher → subscriber latency):**
   The publisher (ShmWriter) stamps each message with
   `steady_clock::now()` in nanoseconds (`timestamp_ns` field in the
   ShmBlock header).  The subscriber (ShmSubscriber) computes
   `now_ns() - message_timestamp_ns` on `receive()`.

2. **Zenoh path (callback → poll delay only):**
   ZenohPublisher sends raw bytes with no timestamp header.
   ZenohSubscriber records `steady_clock::now()` in its `on_sample()`
   callback.  When `receive()` is later called, it computes
   `now_ns() - callback_timestamp`.  This measures how long the message
   waited in the subscriber's latest-value cache before the main loop
   polled it — useful for tuning poll rates, but **not** a wire latency
   measurement.

3. That delta is pushed into a **LatencyTracker** — a fixed-size ring
   buffer that holds the most recent N samples.
4. At any time you can call `summary()` to get percentile statistics
   (p50/p90/p95/p99/min/max/mean), or use the convenience
   `log_latency_if_due()` for periodic log output.

> **Clock note:** Both publisher and subscriber use
> `std::chrono::steady_clock` so results are immune to NTP jumps.
> SHM channels are intra-machine, so no clock-sync issues.

---

## Quick Start

### 1. Latency tracking is ON by default

Both `ShmSubscriber` and `ZenohSubscriber` have latency tracking
**enabled by default**.  No code changes are needed to start collecting
data:

```cpp
// This already tracks latency — nothing extra required
auto sub = drone::ipc::bus_subscribe<drone::ipc::ShmVideoFrame>(
    bus, drone::ipc::shm_names::VIDEO_MISSION_CAM);
```

### 2. Log a periodic summary

In your process main loop (or health-check thread), call
`log_latency_if_due()`:

```cpp
// Main loop — every 5 s health check
while (g_running.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::seconds(5));

    video_sub->log_latency_if_due(100);   // log after ≥100 samples
    det_sub->log_latency_if_due(100);

    spdlog::info("[HealthCheck] perception alive");
}
```

Output (spdlog):
```
[info] [Latency] /drone_mission_cam — n=3247, p50=8.3µs, p90=14.1µs,
       p99=42.7µs, max=128.4µs, mean=10.2µs
```

After logging, the tracker **automatically resets** so the next report
covers a fresh window.

### 3. Disable tracking for non-critical channels

Pass `track_latency = false` to the subscriber constructor:

```cpp
// Low-priority topic — skip tracking overhead
ShmSubscriber<HeartbeatMsg> hb_sub("/drone_heartbeat", 50, 200,
                                    /*track_latency=*/false);
```

---

## API Reference

### `drone::util::LatencyTracker`

Header: `common/util/include/util/latency_tracker.h`

| Method | Complexity | Description |
|---|---|---|
| `LatencyTracker(size_t capacity = 1024)` | O(n) | Create ring buffer. Capacity is rounded up to next power-of-2. |
| `record(uint64_t latency_ns)` | **O(1)** | Push one sample. No allocation. Hot-path safe. |
| `summary() const` | O(n log n) | Return `LatencySummary` — computed over the most recent `min(total_count, capacity)` samples. |
| `reset()` | O(n) | Zero all samples and counters. |
| `total_count() const` | O(1) | Samples recorded since last reset (may exceed capacity). |
| `capacity() const` | O(1) | Ring buffer size. |
| `log_summary_if_due(topic, min_samples)` | O(n log n) | Log and reset if ≥ `min_samples` recorded. Returns `true` if logged. |
| `static now_ns()` | O(1) | `steady_clock::now()` as `uint64_t` nanoseconds. |

### `drone::util::LatencySummary`

| Field | Type | Description |
|---|---|---|
| `count` | `uint64_t` | Total samples since last reset |
| `min_ns` | `uint64_t` | Minimum latency |
| `max_ns` | `uint64_t` | Maximum latency |
| `mean_ns` | `double` | Arithmetic mean |
| `p50_ns` | `uint64_t` | Median (50th percentile) |
| `p90_ns` | `uint64_t` | 90th percentile |
| `p95_ns` | `uint64_t` | 95th percentile |
| `p99_ns` | `uint64_t` | 99th percentile |

**Conversion helpers** (static):

```cpp
LatencySummary::to_us(ns)   // → double (microseconds)
LatencySummary::to_ms(ns)   // → double (milliseconds)
```

### Subscriber Integration

Both `ShmSubscriber<T>` and `ZenohSubscriber<T>` expose:

| Method | Description |
|---|---|
| `latency_tracker()` | Direct access to the underlying `LatencyTracker&` |
| `log_latency_if_due(min_samples)` | Convenience — calls `latency_tracker().log_summary_if_due(topic_name(), min_samples)` |

---

## Usage Patterns

### Pattern 1: Periodic Console Logging

The simplest approach — add one line to your existing health-check loop:

```cpp
// In any process main loop
while (running) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    my_subscriber->log_latency_if_due(100);
}
```

### Pattern 2: Programmatic Threshold Alerts

Query the summary and raise an alert when p99 exceeds your budget:

```cpp
auto& tracker = video_sub->latency_tracker();

if (tracker.total_count() >= 500) {
    auto stats = tracker.summary();

    if (LatencySummary::to_ms(stats.p99_ns) > 10.0) {
        spdlog::warn("[ALERT] Video IPC p99 = {:.1f} ms — exceeds 10 ms budget!",
                     LatencySummary::to_ms(stats.p99_ns));

        // Optionally: publish a FaultEvent, trigger graceful degradation
        fault_manager.raise(FaultCode::IPC_LATENCY_HIGH, "video p99 > 10ms");
    }
    tracker.reset();
}
```

### Pattern 3: Multi-Channel Dashboard

Monitor all subscribers in a single reporting function:

```cpp
void report_latencies(/* all your subscribers */) {
    struct Channel {
        const char*                name;
        drone::ipc::ISubscriber*   sub;
    };

    Channel channels[] = {
        {"video",     video_sub.get()},
        {"detections", det_sub.get()},
        {"pose",       pose_sub.get()},
    };

    for (auto& ch : channels) {
        // ISubscriber doesn't expose latency_tracker() directly,
        // so downcast or store the concrete subscriber type.
        // See Pattern 4 for a generic approach.
    }
}
```

### Pattern 4: Standalone Latency Benchmark

Use `LatencyTracker` directly (without subscribers) to benchmark any
operation:

```cpp
#include "util/latency_tracker.h"

drone::util::LatencyTracker tracker(4096);

for (int i = 0; i < 10000; ++i) {
    uint64_t t0 = drone::util::LatencyTracker::now_ns();

    // ── Code under test ──
    my_function();
    // ──────────────────────

    tracker.record(drone::util::LatencyTracker::now_ns() - t0);
}

auto s = tracker.summary();
spdlog::info("p50={:.1f}µs  p99={:.1f}µs  max={:.1f}µs",
             drone::util::LatencySummary::to_us(s.p50_ns),
             drone::util::LatencySummary::to_us(s.p99_ns),
             drone::util::LatencySummary::to_us(s.max_ns));
```

### Pattern 5: Comparing SHM vs Zenoh Transport

Run the same pipeline with both backends and compare:

```bash
# Terminal 1 — SHM mode (default)
./build/bin/video_capture --config config/default.yaml &
./build/bin/perception    --config config/default.yaml

# Terminal 2 — Zenoh mode
./build/bin/video_capture --config config/zenoh.yaml &
./build/bin/perception    --config config/zenoh.yaml
```

Both will emit `[Latency]` log lines. Compare p50/p99 to quantify the
transport overhead difference.

---

## Interpreting Results

### Typical Healthy Values (Reference)

These numbers are from a development workstation (AMD Ryzen 9, 64 GB
RAM).  Embedded targets (Jetson Orin) will be higher.

| Channel | Payload Size | p50 | p90 | p99 | Notes |
|---|---|---|---|---|---|
| SHM (same machine) | 6 MB (1080p frame) | 5–15 µs | 15–30 µs | 30–100 µs | SeqLock memcpy |
| SHM (same machine) | 4 KB (pose) | 0.5–2 µs | 2–5 µs | 5–15 µs | Cache-hot path |
| Zenoh (localhost) | 4 KB | 20–50 µs | 50–150 µs | 150–500 µs | Serialisation + mutex |
| Zenoh (network) | 4 KB | 200–2000 µs | — | — | Depends on link |

### Red Flags

| Symptom | Likely Cause | Action |
|---|---|---|
| p50 > 1 ms (SHM, small msg) | CPU throttling or scheduling | Check `cpufreq` governor, set `SCHED_FIFO` |
| p99 >> 10× p50 | Kernel preemption or lock contention | Check for `RT_PREEMPT` kernel, reduce lock scope |
| p99 growing over time | Memory pressure triggering page faults | Check `vmstat`, pin SHM pages with `mlock()` |
| max >> p99 | Occasional huge spike | Often GC in another process or writeback I/O stall |
| All zeros | Subscriber not receiving | Check publisher is running, SHM name matches |
| count = 0 | `track_latency` is `false` | Enable tracking, or check `summary()` timing |

### Ring Buffer Sizing

| Capacity | Memory | Window (at 30 fps) | Percentile Accuracy |
|---|---|---|---|
| 256 | 2 KB | ~8.5 s | ±2 % |
| 1024 (default) | 8 KB | ~34 s | ±0.5 % |
| 4096 | 32 KB | ~136 s | ±0.1 % |

Larger buffers give more accurate percentiles but consume more memory
and make `summary()` slightly slower (sort cost).  The default of 1024
is a good balance for 30–100 Hz topics.

---

## Thread Safety

`LatencyTracker` is **not** internally synchronised (no atomics, no
mutex).  All three operations — `record()`, `summary()`, and `reset()`
— must be called from the **same** thread, or the caller must provide
external locking.

In practice this is safe because each subscriber owns its own tracker,
and the main loop of every process drives both `receive()` (which calls
`record()`) and `log_latency_if_due()` (which calls `summary()` +
`reset()`) **sequentially on the same thread**.

| Operation | Thread safety |
|---|---|
| `record()` | Single-thread only.  Each subscriber owns its tracker — no sharing. |
| `summary()` | Must **not** be called concurrently with `record()` or `reset()`. |
| `reset()` | Must **not** race with `record()` or `summary()`.  Called automatically by `log_summary_if_due()`. |
| Multiple subscribers | Each `ShmSubscriber` / `ZenohSubscriber` has its **own** `LatencyTracker` instance.  No sharing. |

> If a future design requires cross-thread access, wrap calls in a
> `std::mutex` or switch the ring buffer to use atomic indices.

---

## Configuration

Currently, latency tracking is controlled **per-subscriber at
construction time** via the `track_latency` boolean parameter:

```cpp
// Enabled (default)
ShmSubscriber<T> sub("/topic");

// Disabled
ShmSubscriber<T> sub("/topic", 50, 200, /*track_latency=*/false);
```

Future: A config-file option (`ipc.latency_tracking: true/false`) and
runtime toggle may be added in a follow-up issue.

---

## Testing

The latency tracker has **20 dedicated unit tests** covering:

| Test | What it validates |
|---|---|
| DefaultConstruction | 1024-sample buffer, zero initial count |
| CustomCapacityRoundedToPowerOfTwo | e.g. 1000 → 1024 |
| RecordIncrementsCount | Count increases monotonically |
| RecordWrapsRingBuffer | Old samples evicted after capacity |
| SummaryEmptyTracker | Returns zeroed `LatencySummary` |
| SummarySingleSample | Edge case: n=1 |
| SummaryUniformData | Percentiles match expected uniform distribution |
| SummaryWithOutlier | p99 catches outliers, p50 is not affected |
| PercentilesAreMonotonic | p50 ≤ p90 ≤ p95 ≤ p99 ≤ max |
| ResetClearsState | Count, summary both zero after reset |
| ResetAllowsReuse | Re-record after reset works correctly |
| ConversionToUs | 1000 ns → 1.0 µs |
| ConversionToMs | 1'000'000 ns → 1.0 ms |
| NowNsReturnsIncreasingValues | Monotonicity of `now_ns()` |
| LogSummaryNotDueWithTooFewSamples | Below threshold → no log |
| LogSummaryDueWithEnoughSamples | At threshold → logs + resets |
| LogSummaryCustomMinSamples | Custom `min_samples` parameter |
| WrappedBufferReportsMostRecentData | Evicted old data excluded |
| StressManyRecords | 100k samples, no crash, valid stats |
| IntegrationRealLatency | Real `now_ns()` round-trip timing |

Run them:

```bash
# Just the latency tests
cd build && ctest -R Latency --output-on-failure

# Or run the binary directly for verbose output
./build/bin/test_latency_tracker
```

The SHM and Zenoh integration is also covered by the existing
`ShmMessageBus*` and `MessageBusFactory.*` test suites (13 tests).

---

## Files

| File | Description |
|---|---|
| [common/util/include/util/latency_tracker.h](../common/util/include/util/latency_tracker.h) | `LatencyTracker` + `LatencySummary` — the core utility |
| [common/ipc/include/ipc/shm_subscriber.h](../common/ipc/include/ipc/shm_subscriber.h) | SHM subscriber with integrated latency tracking |
| [common/ipc/include/ipc/zenoh_subscriber.h](../common/ipc/include/ipc/zenoh_subscriber.h) | Zenoh subscriber with integrated latency tracking |
| [tests/test_latency_tracker.cpp](../tests/test_latency_tracker.cpp) | 20 unit tests |
| [docs/ipc-latency-tracking.md](ipc-latency-tracking.md) | This document |

---

## Known Limitations

1. **Zenoh latency is callback → poll delay, not wire latency.**
   `ZenohPublisher` sends raw bytes with no embedded timestamp.
   `ZenohSubscriber` stamps `steady_clock::now()` inside the Zenoh
   callback, so the recorded delta only shows how long the message sat
   in the subscriber's latest-value cache before `receive()` polled it.
   To measure true publisher → subscriber wire latency, a future change
   would need to embed a send timestamp in the Zenoh payload (Issue TBD).

2. **Not thread-safe.**  `LatencyTracker` has no internal
   synchronisation.  All calls (`record`, `summary`, `reset`) must come
   from the same thread or be externally serialised.  This is fine for
   the current single-threaded main-loop architecture.

3. **No runtime config-file toggle.**  Tracking is controlled at
   construction time via a boolean.  A config-file option may be added
   later.

---

## FAQ

**Q: Does latency tracking add overhead to the hot path?**

`record()` is **O(1)**: one array write and two increments. No heap
allocation, no system calls, no locks. On a modern CPU this is
< 10 ns — negligible compared to the `memcpy` in SHM reads.
`summary()` is O(n log n) and should only be called from the
periodic reporting path, not per-message.

**Q: What does the timestamp measure?**

It depends on the transport:

- **SHM:** The delta between `ShmWriter::write()` stamping
  `timestamp_ns` in the ShmBlock header and `ShmSubscriber::receive()`
  calling `now_ns()`.  This is **true publisher → subscriber latency**
  and includes SeqLock retry time (usually 0), `memcpy` of the payload,
  and the polling interval between `write()` and `receive()`.

- **Zenoh:** The delta between the `on_sample()` callback stamping
  `steady_clock::now()` and `receive()` calling `now_ns()`.  This
  measures **callback → poll delay** — how long the message waited in
  the subscriber's latest-value cache before the main loop polled it.
  It does **not** include wire/serialisation time because
  `ZenohPublisher` does not embed a send timestamp.

**Q: Why use `steady_clock` instead of `system_clock`?**

`system_clock` can jump backwards (NTP corrections). `steady_clock`
is monotonic and suitable for measuring elapsed time.

**Q: Can I use this to measure end-to-end pipeline latency?**

Yes — the per-hop latency gives you each segment.  Sum
video→perception + perception→mission_planner for chain latency.
Cross-process correlation IDs (Issue #80) will make this automatic.

**Q: What happens if the publisher dies?**

The subscriber stops receiving messages, so no new latency samples
are recorded. `total_count()` stops increasing.  The last
`summary()` still reflects the most recently observed latencies.
