# Observability Guide

Companion Software Stack — Logging, Latency Tracking & Correlation IDs

---

## Overview

The stack provides three complementary observability tools that work
together to help you debug, profile, and trace command flow across all
7 processes:

| Tool | What it does | Key question answered |
|------|-------------|----------------------|
| **IPC Latency Tracking** | Measures publisher → subscriber delay on every IPC channel | "Where is the pipeline slow?" |
| **Structured JSON Logging** | Emits every log line as a machine-readable JSON object | "What happened, and can I query it?" |
| **Cross-Process Correlation IDs** | Tags a GCS command with a unique ID that flows through every process | "What did this specific command trigger end-to-end?" |

```
GCS ──────────────────────────────────────────────────────────────────
  │  RTL command
  ▼
┌─────────────────────────────────────────────────────────────────────┐
│ Process 5 (Comms)                                                   │
│  gcs_rx_thread:                                                     │
│    • CorrelationContext::generate()  → corr=0x0005000100000001      │
│    • shm_cmd.correlation_id = corr                                  │
│    • publish(shm_cmd)                                               │
│    • CorrelationContext::clear()                                    │
│                                                                     │
│  fc_tx_thread:                                                      │
│    • ScopedCorrelation guard(fc_cmd.correlation_id)                 │
│    • Logs: [Comms] FC cmd: RTL corr=0x0005000100000001              │
│    • fc_link.send_command(RTL)                                      │
└─────────────┬───────────────────────────────────────────────────────┘
              │ ShmGCSCommand (via SHM / Zenoh)
              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ Process 4 (Mission Planner)                                         │
│  main loop:                                                         │
│    • active_correlation_id = gcs_cmd.correlation_id                 │
│    • ScopedCorrelation guard(gcs_cmd.correlation_id)                │
│    • send_fc_command(RTL)  → fc_cmd.correlation_id = corr           │
│    • ShmTrajectoryCmd.correlation_id = corr                         │
│    • ShmPayloadCommand.correlation_id = active_correlation_id       │
│    • ShmMissionStatus.correlation_id = active_correlation_id        │
└─────────────────────────────────────────────────────────────────────┘
```

All three tools are **always available** — no external dependencies,
no runtime agents, no configuration files.  Latency tracking is on by
default; JSON logging and correlation IDs activate via the `--json-logs`
flag and normal IPC message flow respectively.

---

## 1. IPC Latency Tracking

### Why It Matters

Every IPC hop adds delay between sensing and acting.  If the
video-capture → perception → mission-planner pipeline exceeds the
control-loop budget, the aircraft can overshoot waypoints, lose tracking
lock, or miss obstacle detections.

The built-in latency tracker lets you **continuously measure
publisher-to-subscriber delay** on every IPC channel — SHM and Zenoh —
with negligible overhead.

| Goal | Question answered |
|------|-------------------|
| Detect regressions | "Did my last code change add latency?" |
| Budget allocation  | "How much of the 33 ms frame budget does IPC consume?" |
| Tail-latency hunts | "Is p99 > 5× p50?  Am I scheduling poorly?" |
| Deployment tuning  | "Is the Jetson Orin slower than my dev laptop?" |
| Compare transports | "SHM vs Zenoh latency for the same payload?" |

### Architecture

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
```

**How it works:**

1. **SHM path (true publisher → subscriber latency):**
   The publisher stamps each message with `steady_clock::now()` in
   nanoseconds.  The subscriber computes `now_ns() - timestamp_ns` on
   `receive()`.

2. **Zenoh path (callback → poll delay only):**
   ZenohPublisher sends raw bytes with no timestamp header.
   ZenohSubscriber records `steady_clock::now()` in its callback.
   When `receive()` is later called, it computes
   `now_ns() - callback_timestamp`.  This measures how long the message
   waited in the subscriber's cache — useful for tuning poll rates, but
   **not** a wire latency measurement.

3. The delta is pushed into a **LatencyTracker** — a fixed-size ring
   buffer holding the most recent N samples.

4. Call `summary()` for percentile statistics (p50/p90/p95/p99/min/max/mean),
   or use `log_latency_if_due()` for periodic log output.

> **Clock note:** Both publisher and subscriber use
> `std::chrono::steady_clock` — immune to NTP jumps.

### Quick Start

Latency tracking is **on by default**.  No code changes needed:

```cpp
// This already tracks latency — nothing extra required
auto sub = drone::ipc::bus_subscribe<drone::ipc::ShmVideoFrame>(
    bus, drone::ipc::shm_names::VIDEO_MISSION_CAM);
```

Log a periodic summary in your health-check loop:

```cpp
while (g_running.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    video_sub->log_latency_if_due(100);   // log after ≥100 samples
    det_sub->log_latency_if_due(100);
}
```

Output:
```
[info] [Latency] /drone_mission_cam — n=3247, p50=8.3µs, p90=14.1µs,
       p99=42.7µs, max=128.4µs, mean=10.2µs
```

After logging, the tracker **automatically resets** for a fresh window.

Disable tracking for non-critical channels:

```cpp
ShmSubscriber<HeartbeatMsg> hb_sub("/drone_heartbeat", 50, 200,
                                    /*track_latency=*/false);
```

### API Reference

#### `drone::util::LatencyTracker`

Header: `common/util/include/util/latency_tracker.h`

| Method | Complexity | Description |
|--------|-----------|-------------|
| `LatencyTracker(size_t capacity = 1024)` | O(n) | Create ring buffer (rounded to next power-of-2) |
| `record(uint64_t latency_ns)` | **O(1)** | Push one sample. No allocation. Hot-path safe. |
| `summary() const` | O(n log n) | Return `LatencySummary` with percentile statistics |
| `reset()` | O(n) | Zero all samples and counters |
| `total_count() const` | O(1) | Samples recorded since last reset |
| `capacity() const` | O(1) | Ring buffer size |
| `log_summary_if_due(topic, min_samples)` | O(n log n) | Log and reset if ≥ `min_samples` recorded |
| `static now_ns()` | O(1) | `steady_clock::now()` as `uint64_t` nanoseconds |

#### `drone::util::LatencySummary`

| Field | Type | Description |
|-------|------|-------------|
| `count` | `uint64_t` | Total samples since last reset |
| `min_ns` | `uint64_t` | Minimum latency |
| `max_ns` | `uint64_t` | Maximum latency |
| `mean_ns` | `double` | Arithmetic mean |
| `p50_ns` | `uint64_t` | Median (50th percentile) |
| `p90_ns` | `uint64_t` | 90th percentile |
| `p95_ns` | `uint64_t` | 95th percentile |
| `p99_ns` | `uint64_t` | 99th percentile |

Conversion helpers:

```cpp
LatencySummary::to_us(ns)   // → double (microseconds)
LatencySummary::to_ms(ns)   // → double (milliseconds)
```

#### Subscriber Integration

Both `ShmSubscriber<T>` and `ZenohSubscriber<T>` expose:

| Method | Description |
|--------|-------------|
| `latency_tracker()` | Direct access to the underlying `LatencyTracker&` |
| `log_latency_if_due(min_samples)` | Convenience — calls tracker's `log_summary_if_due()` with the topic name |

### Usage Patterns

#### Programmatic Threshold Alerts

```cpp
auto& tracker = video_sub->latency_tracker();

if (tracker.total_count() >= 500) {
    auto stats = tracker.summary();
    if (LatencySummary::to_ms(stats.p99_ns) > 10.0) {
        spdlog::warn("[ALERT] Video IPC p99 = {:.1f} ms — exceeds 10 ms budget!",
                     LatencySummary::to_ms(stats.p99_ns));
        fault_manager.raise(FaultCode::IPC_LATENCY_HIGH, "video p99 > 10ms");
    }
    tracker.reset();
}
```

#### Standalone Latency Benchmark

Use `LatencyTracker` directly to benchmark any operation:

```cpp
#include "util/latency_tracker.h"

drone::util::LatencyTracker tracker(4096);

for (int i = 0; i < 10000; ++i) {
    uint64_t t0 = drone::util::LatencyTracker::now_ns();
    my_function();
    tracker.record(drone::util::LatencyTracker::now_ns() - t0);
}

auto s = tracker.summary();
spdlog::info("p50={:.1f}µs  p99={:.1f}µs  max={:.1f}µs",
             drone::util::LatencySummary::to_us(s.p50_ns),
             drone::util::LatencySummary::to_us(s.p99_ns),
             drone::util::LatencySummary::to_us(s.max_ns));
```

#### Comparing SHM vs Zenoh Transport

```bash
# Terminal 1 — SHM mode (default)
./build/bin/video_capture --config config/default.json &
./build/bin/perception    --config config/default.json

# Terminal 2 — Zenoh mode
./build/bin/video_capture --config config/zenoh.json &
./build/bin/perception    --config config/zenoh.json
```

Both emit `[Latency]` log lines.  Compare p50/p99 to quantify transport
overhead.

### Interpreting Results

#### Typical Healthy Values (Reference)

Development workstation (AMD Ryzen 9, 64 GB RAM).  Embedded targets
(Jetson Orin) will be higher.

| Channel | Payload Size | p50 | p90 | p99 | Notes |
|---------|-------------|-----|-----|-----|-------|
| SHM (same machine) | 6 MB (1080p frame) | 5–15 µs | 15–30 µs | 30–100 µs | SeqLock memcpy |
| SHM (same machine) | 4 KB (pose) | 0.5–2 µs | 2–5 µs | 5–15 µs | Cache-hot path |
| Zenoh (localhost) | 4 KB | 20–50 µs | 50–150 µs | 150–500 µs | Serialisation + mutex |
| Zenoh (network) | 4 KB | 200–2000 µs | — | — | Depends on link |

#### Red Flags

| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| p50 > 1 ms (SHM, small msg) | CPU throttling or scheduling | Check `cpufreq` governor, set `SCHED_FIFO` |
| p99 >> 10× p50 | Kernel preemption or lock contention | Check for `RT_PREEMPT` kernel, reduce lock scope |
| p99 growing over time | Memory pressure triggering page faults | Check `vmstat`, pin SHM pages with `mlock()` |
| max >> p99 | Occasional huge spike | Often GC in another process or writeback I/O stall |
| All zeros | Subscriber not receiving | Check publisher is running, SHM name matches |
| count = 0 | `track_latency` is `false` | Enable tracking, or check `summary()` timing |

#### Ring Buffer Sizing

| Capacity | Memory | Window (at 30 fps) | Percentile Accuracy |
|----------|--------|--------------------|--------------------|
| 256 | 2 KB | ~8.5 s | ±2 % |
| 1024 (default) | 8 KB | ~34 s | ±0.5 % |
| 4096 | 32 KB | ~136 s | ±0.1 % |

### Thread Safety

`LatencyTracker` is **not** internally synchronised.  All operations
(`record()`, `summary()`, `reset()`) must be called from the **same**
thread, or the caller must provide external locking.

This is safe in practice because each subscriber owns its own tracker,
and the main loop drives `receive()` + `log_latency_if_due()`
sequentially on the same thread.

---

## 2. Structured JSON Logging

### Why JSON Logs

Human-readable logs are great for watching in a terminal.  But when you
need to search, filter, aggregate, or feed logs into a monitoring
pipeline, structured JSON is essential:

```bash
# Find all warnings from the mission planner in the last flight
cat drone_logs/*.log | jq -c 'select(.logger=="mission_planner" and .level=="warn")'

# Count errors per process
cat drone_logs/*.log | jq -r '.logger' | sort | uniq -c | sort -rn
```

### Enabling JSON Logs

Pass `--json-logs` to any process:

```bash
# Single process
./build/bin/perception --json-logs

# All processes at once
./deploy/launch_all.sh --json-logs
```

When `--json-logs` is active:
- **stdout** receives JSON Lines (one JSON object per line)
- **Log file** (`drone_logs/<process>.log`) still gets human-readable format

This means you can pipe stdout to a log aggregator while keeping
human-readable files for local debugging.

### JSON Line Format

Each log line is a single JSON object:

```json
{
  "ts": "2026-03-03T12:34:56.789012",
  "level": "info",
  "logger": "perception",
  "thread": 12345,
  "pid": 9876,
  "msg": "Pipeline started",
  "correlation_id": "0x0005000100000001",
  "src": {
    "file": "main.cpp",
    "line": 42,
    "func": "main"
  }
}
```

#### Field Reference

| Field | Type | Always present | Description |
|-------|------|:-:|--------------|
| `ts` | string | ✓ | ISO 8601 UTC timestamp with microsecond precision |
| `level` | string | ✓ | `trace`, `debug`, `info`, `warn`, `error`, `critical` |
| `logger` | string | ✓ | Process name (e.g. `"comms"`, `"perception"`) |
| `thread` | number | ✓ | OS thread ID |
| `pid` | number | ✓ | OS process ID |
| `msg` | string | ✓ | The log message (special chars JSON-escaped) |
| `correlation_id` | string | ✗ | Hex correlation ID — **omitted when 0** (see §3) |
| `src` | object | ✗ | Source location — only present when spdlog has it |

### Querying with jq

```bash
# All log lines with a specific correlation ID
cat drone_logs/*.log | jq -c 'select(.correlation_id == "0x0005000100000001")'

# Latency-related warnings
cat drone_logs/*.log | jq -c 'select(.level == "warn" and (.msg | test("latency|p99|budget")))'

# Timeline of a GCS command across all processes (sorted by timestamp)
cat drone_logs/*.log | jq -cs '
  [.[] | select(.correlation_id == "0x0005000100000001")]
  | sort_by(.ts)
  | .[]
  | {ts, logger, msg}
'
```

### API Reference

Header: `common/util/include/util/json_log_sink.h`

| Type | Description |
|------|-------------|
| `JsonLogSink<Mutex>` | Template class — spdlog sink emitting JSON Lines to a `FILE*` |
| `JsonLogSink_mt` | Thread-safe variant (`std::mutex`) |
| `JsonLogSink_st` | Single-threaded variant (no locking, for tests) |

Constructor:
```cpp
explicit JsonLogSink(std::FILE* output = stdout);
```

The sink does **not** own the `FILE*`.

| Method | Description |
|--------|-------------|
| `last_json()` | Returns the last formatted JSON string (useful for testing) |

### How `--json-logs` Works Internally

1. `parse_args()` in `arg_parser.h` sets `ParsedArgs::json_logs = true`
2. `LogConfig::init()` checks `json_mode`:
   - **true**: creates `JsonLogSink_mt` for stdout + rotating file sink for log file
   - **false**: creates `stdout_color_sink_mt` for coloured console + rotating file sink
3. The `JsonLogSink::sink_it_()` method checks `CorrelationContext::get()` — if non-zero,
   it includes the `"correlation_id"` field automatically

---

## 3. Cross-Process Correlation IDs

### Why Correlation IDs

When a GCS operator sends an RTL command, it triggers a cascade across
multiple processes: comms receives it, mission planner processes it,
FC commands are sent, trajectories are stopped, payload may be triggered.
Without correlation IDs, matching log lines across these processes
requires careful timestamp alignment and guesswork.

With correlation IDs, you can:

```bash
# Trace a single GCS command across every process in the stack
grep "0x0005000100000001" drone_logs/*.log

# Or with JSON logs:
cat drone_logs/*.log | jq -c 'select(.correlation_id == "0x0005000100000001")'
```

### ID Generation Scheme

```
┌────────────────────────────────────────────────────┐
│              64-bit Correlation ID                  │
├─────────────────────────┬──────────────────────────┤
│  Upper 32 bits: PID     │  Lower 32 bits: Counter  │
│  (unique across procs)  │  (unique within proc)    │
└─────────────────────────┴──────────────────────────┘

Example: PID=327681 (0x00050001), Counter=1
  → 0x0005000100000001
```

- **PID** guarantees uniqueness across processes
- **Monotonic counter** (atomic) guarantees uniqueness within a process
- IDs are generated **only at the origin** (comms, when receiving a GCS command)
- Downstream processes **propagate** the received ID, they don't generate new ones

### Data Flow

```
GCS sends RTL
      │
      ▼
Process 5 (Comms) ─── gcs_rx_thread ───────────────────────
  │  generate() → 0x0005000100000001
  │  ShmGCSCommand.correlation_id = 0x0005000100000001
  │  publish → SHM/Zenoh
  │  clear()  ← prevents stale ID on next loop iteration
  │
  ▼
Process 4 (Mission Planner) ─── main loop ──────────────────
  │  active_correlation_id = gcs_cmd.correlation_id
  │  ScopedCorrelation guard(gcs_cmd.correlation_id)
  │  │
  │  ├─ send_fc_command(RTL)
  │  │    └─ ShmFCCommand.correlation_id = context.get()
  │  │
  │  ├─ ShmTrajectoryCmd.correlation_id = gcs_cmd.correlation_id
  │  │
  │  └─ (guard destructor restores previous context)
  │
  │  // Later in loop (outside ScopedCorrelation scope):
  │  ShmPayloadCommand.correlation_id = active_correlation_id
  │  ShmMissionStatus.correlation_id  = active_correlation_id
  │
  ▼
Process 5 (Comms) ─── fc_tx_thread ─────────────────────────
     ScopedCorrelation guard(fc_cmd.correlation_id)
     Log: [Comms] FC cmd: RTL corr=0x0005000100000001
     fc_link.send_command(RTL)
```

### API Reference

Header: `common/util/include/util/correlation.h`

#### `drone::util::CorrelationContext`

| Method | Description |
|--------|-------------|
| `static generate() → uint64_t` | Create new ID `(pid << 32 \| counter++)`, set on current thread, return it |
| `static set(uint64_t id)` | Set the current thread's correlation ID |
| `static get() → uint64_t` | Get the current thread's correlation ID (0 = none) |
| `static clear()` | Reset to 0 |

Storage: `thread_local uint64_t` — each thread has its own independent context.

#### `drone::util::ScopedCorrelation`

RAII guard that sets the correlation ID for a scope and restores the
previous value on destruction:

```cpp
{
    ScopedCorrelation guard(incoming_msg.correlation_id);
    // CorrelationContext::get() == incoming_msg.correlation_id
    // All log lines in this scope include the correlation ID
    outgoing_msg.correlation_id = CorrelationContext::get();
}
// CorrelationContext::get() == previous value (usually 0)
```

Non-copyable, non-movable.

#### SHM Messages with `correlation_id`

| Struct | Role |
|--------|------|
| `ShmGCSCommand` | Origin — ID generated here by comms |
| `ShmFCCommand` | Propagated from mission planner |
| `ShmTrajectoryCmd` | Propagated from mission planner |
| `ShmPayloadCommand` | Propagated from mission planner |
| `ShmMissionStatus` | Propagated from mission planner |

All fields default to 0 (= "no correlation").

#### Wire Format (Network Transport)

`WireHeader` v2 includes a `correlation_id` field at offset 24–31
(total header size: 32 bytes).  The `wire_serialize()` function accepts
an optional `corr_id` parameter:

```cpp
auto buf = wire_serialize(msg, WireMessageType::FC_COMMAND, seq, 0, corr_id);
```

**Forward compatibility:** v2 readers accept v1 (24-byte) headers by
setting `correlation_id = 0`.  Note: v1 readers will reject v2 messages
(one-way compatibility — new readers accept old writers).

### Thread Isolation

Correlation IDs use `thread_local` storage.  Each thread has its own
independent context:

- `gcs_rx_thread` generates IDs — doesn't affect `fc_tx_thread`
- `fc_tx_thread` sets context from received messages via `ScopedCorrelation`
- The mission planner's main loop uses `ScopedCorrelation` for the GCS
  command scope and `active_correlation_id` for long-lived state

---

## 4. Cookbook — Putting It All Together

### Recipe 1: Trace a GCS RTL Command End-to-End

```bash
# 1. Launch all processes with JSON logging
./deploy/launch_all.sh --json-logs 2>&1 | tee /tmp/flight.jsonl

# 2. Send an RTL command from the GCS
# ... (via your GCS software)

# 3. Find the correlation ID from the comms log
grep "GCS cmd received" /tmp/flight.jsonl
# Output: ... corr=0x0005000100000003 ...

# 4. Trace that command across all processes
grep "0x0005000100000003" /tmp/flight.jsonl

# Or with jq for structured output:
cat /tmp/flight.jsonl | jq -c '
  select(.correlation_id == "0x0005000100000003")
  | {ts, logger, msg}
'
```

### Recipe 2: Find the Slowest IPC Hop

```bash
# Look at all latency summary lines
grep "\[Latency\]" drone_logs/*.log | sort -t'=' -k3 -n -r | head -5
```

Or programmatically, set up alerts in your process:

```cpp
auto& tracker = sub->latency_tracker();
if (tracker.total_count() >= 500) {
    auto stats = tracker.summary();
    if (LatencySummary::to_ms(stats.p99_ns) > 10.0) {
        spdlog::warn("IPC p99 = {:.1f}ms — exceeds budget!", 
                     LatencySummary::to_ms(stats.p99_ns));
    }
    tracker.reset();
}
```

### Recipe 3: Monitor System Health in Real-Time with JSON

```bash
# Stream only warnings and errors, formatted as a table
./deploy/launch_all.sh --json-logs 2>&1 \
  | jq -r 'select(.level == "warn" or .level == "error")
            | [.ts[11:23], .logger, .level, .msg[:80]] | @tsv'
```

### Recipe 4: Post-Flight Latency Report

```bash
# After flight, extract latency summaries per channel
grep "\[Latency\]" drone_logs/*.log \
  | sed 's/.*\[Latency\] //' \
  | column -t -s','
```

### Recipe 5: Compare SHM vs Zenoh Latency

```bash
# Run with SHM (default config)
./deploy/launch_all.sh --config config/default.json 2>&1 | tee /tmp/shm.log
# ... wait 30 seconds for samples ...

# Run with Zenoh
./deploy/launch_all.sh --config config/zenoh.json 2>&1 | tee /tmp/zenoh.log

# Extract and compare p50/p99
grep "\[Latency\]" /tmp/shm.log /tmp/zenoh.log
```

---

## 5. Testing

| Test suite | Count | What it validates |
|-----------|-------|-------------------|
| `test_latency_tracker` | 20 | Ring buffer, percentiles, reset, edge cases |
| `test_json_log_sink` | — | JSON formatting, escaping, field presence |
| `test_correlation` | 40 | Context ops, RAII guard, thread isolation, SHM fields, WireHeader v2, backward compat, JSON sink integration |

Run them:

```bash
# All observability tests
cd build && ctest -R "Latency|Json|Correlation" --output-on-failure

# Or individually
./build/bin/test_latency_tracker
./build/bin/test_json_log_sink
./build/bin/test_correlation
```

---

## 6. Files

| File | Description |
|------|-------------|
| [common/util/include/util/latency_tracker.h](../common/util/include/util/latency_tracker.h) | `LatencyTracker` + `LatencySummary` |
| [common/util/include/util/json_log_sink.h](../common/util/include/util/json_log_sink.h) | `JsonLogSink` — structured JSON spdlog sink |
| [common/util/include/util/correlation.h](../common/util/include/util/correlation.h) | `CorrelationContext` + `ScopedCorrelation` |
| [common/util/include/util/log_config.h](../common/util/include/util/log_config.h) | `LogConfig::init()` — wires JSON/console sinks |
| [common/util/include/util/arg_parser.h](../common/util/include/util/arg_parser.h) | `--json-logs` CLI flag |
| [common/ipc/include/ipc/shm_types.h](../common/ipc/include/ipc/shm_types.h) | SHM structs with `correlation_id` field |
| [common/ipc/include/ipc/wire_format.h](../common/ipc/include/ipc/wire_format.h) | WireHeader v2 with `correlation_id` |
| [common/ipc/include/ipc/shm_subscriber.h](../common/ipc/include/ipc/shm_subscriber.h) | SHM subscriber with latency tracking |
| [common/ipc/include/ipc/zenoh_subscriber.h](../common/ipc/include/ipc/zenoh_subscriber.h) | Zenoh subscriber with latency tracking |
| [tests/test_latency_tracker.cpp](../tests/test_latency_tracker.cpp) | Latency tracker tests |
| [tests/test_json_log_sink.cpp](../tests/test_json_log_sink.cpp) | JSON sink tests |
| [tests/test_correlation.cpp](../tests/test_correlation.cpp) | Correlation ID tests |

---

## 7. Known Limitations

1. **Zenoh latency is callback → poll delay, not wire latency.**
   ZenohPublisher does not embed a send timestamp.  The recorded delta
   only shows how long the message sat in the subscriber's cache before
   `receive()` polled it.

2. **LatencyTracker is not thread-safe.**  All calls must come from the
   same thread (fine for the single-threaded main-loop architecture).

3. **No runtime config-file toggle for latency tracking.**  Controlled
   at construction time via a boolean parameter.

4. **Correlation IDs are generated only for GCS commands.**  Internally
   initiated actions (e.g., autonomous takeoff, fault-triggered RTL)
   do not currently carry correlation IDs.

5. **Wire format backward compatibility is one-way.**  v2 readers
   accept v1 messages, but v1 readers will reject v2 messages.

---

## FAQ

**Q: Does latency tracking add overhead to the hot path?**

`record()` is **O(1)**: one array write and two increments.  < 10 ns on
a modern CPU — negligible compared to the `memcpy` in SHM reads.

**Q: Does JSON logging affect performance?**

Minimally.  The `snprintf`-based formatter adds ~1–2 µs per log line
compared to spdlog's default pattern formatter.  Log lines are
typically emitted at info level (tens per second), not per-frame.

**Q: Why hex for correlation IDs?**

Hex makes the PID/counter split visually obvious:
`0x0005000100000003` → PID=0x00050001, Counter=3.

**Q: Can I use correlation IDs without JSON logging?**

Yes.  The `corr={:#x}` format in human-readable logs works fine:
```
[info] [Comms] FC cmd: RTL corr=0x0005000100000003
```
Use `grep 0x0005000100000003 drone_logs/*.log` to trace.

**Q: Why `steady_clock` instead of `system_clock` for latency?**

`system_clock` can jump backwards (NTP corrections). `steady_clock` is
monotonic and suitable for measuring elapsed time.

**Q: Can I measure end-to-end pipeline latency?**

Yes — sum per-hop latencies: video→perception + perception→planner.
Correlation IDs let you identify which messages belong to the same
command chain.
