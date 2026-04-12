# C++ Patterns & Concepts — Companion Software Stack

> A practical guide to the C++ design patterns, multithreading constructs, and
> modern language features used in this codebase.  Each concept is explained with
> the *why*, followed by code excerpts adapted from this repository (often simplified for clarity).
>
> **Audience:** C++ learners and new contributors.
> **Standard:** C++17 (`-std=c++17`)

---

## Table of Contents

1. [Design Patterns](#1-design-patterns)
   - [1.1 Strategy Pattern (Polymorphic Interfaces)](#11-strategy-pattern-polymorphic-interfaces)
   - [1.2 Factory Pattern](#12-factory-pattern)
   - [1.3 Singleton Pattern](#13-singleton-pattern)
   - [1.4 State Machine Pattern](#14-state-machine-pattern)
   - [1.5 Observer / Publish-Subscribe Pattern](#15-observer--publish-subscribe-pattern)
   - [1.6 RAII (Resource Acquisition Is Initialization)](#16-raii-resource-acquisition-is-initialization)
   - [1.7 Adapter Pattern](#17-adapter-pattern)
2. [Multithreading & Concurrency](#2-multithreading--concurrency)
   - [2.1 `std::thread` and Thread Lifecycle](#21-stdthread-and-thread-lifecycle)
   - [2.2 `std::mutex` and `std::lock_guard`](#22-stdmutex-and-stdlock_guard)
   - [2.3 `std::condition_variable`](#23-stdcondition_variable)
   - [2.4 `std::atomic` and Memory Ordering](#24-stdatomic-and-memory-ordering)
   - [2.5 SeqLock — A Custom Lock-Free Protocol](#25-seqlock--a-custom-lock-free-protocol)
   - [2.6 Lock-Free SPSC Ring Buffer](#26-lock-free-spsc-ring-buffer)
   - [2.7 Double Buffering](#27-double-buffering)
   - [2.8 Signal-Safe Shutdown](#28-signal-safe-shutdown)
   - [2.9 Thread Naming, CPU Pinning & RT Scheduling](#29-thread-naming-cpu-pinning--rt-scheduling)
   - [2.10 Compare-And-Swap (CAS) Loop](#210-compare-and-swap-cas-loop)
3. [Modern C++ Techniques](#3-modern-c-techniques)
   - [3.1 `std::variant` and `std::visit`](#31-stdvariant-and-stdvisit)
   - [3.2 `std::optional`](#32-stdoptional)
   - [3.3 `if constexpr` — Compile-Time Branching](#33-if-constexpr--compile-time-branching)
   - [3.4 `static_assert` — Compile-Time Contracts](#34-static_assert--compile-time-contracts)
   - [3.5 Smart Pointers (`unique_ptr`, `shared_ptr`)](#35-smart-pointers-unique_ptr-shared_ptr)
   - [3.6 Move Semantics](#36-move-semantics)
   - [3.7 Lambda Expressions](#37-lambda-expressions)
   - [3.8 `constexpr` — Compile-Time Constants](#38-constexpr--compile-time-constants)
   - [3.9 `std::string_view`](#39-stdstring_view)
   - [3.10 `enum class` — Scoped Enumerations](#310-enum-class--scoped-enumerations)
   - [3.11 `alignas` — Cache-Line Alignment](#311-alignas--cache-line-alignment)
   - [3.12 Structured `__attribute__((packed))`](#312-structured-__attribute__packed)
   - [3.13 `Result<T,E>` — Monadic Error Handling](#313-resultte--monadic-error-handling)
   - [3.14 `[[nodiscard]]` — Compiler-Enforced Return Checking](#314-nodiscard--compiler-enforced-return-checking)
4. [Systems Programming Concepts](#4-systems-programming-concepts)
   - [4.1 POSIX Shared Memory (`shm_open` / `mmap`)](#41-posix-shared-memory-shm_open--mmap)
   - [4.2 Binary Wire Format](#42-binary-wire-format)
   - [4.3 Conditional Compilation (`#ifdef`)](#43-conditional-compilation-ifdef)
   - [4.4 `fork` + `exec` — Process Management](#44-fork--exec--process-management)
   - [4.5 Exponential Backoff](#45-exponential-backoff)
   - [4.6 FlightRecorder Ring Buffer](#46-flightrecorder-ring-buffer)
   - [4.7 Rate Clamping](#47-rate-clamping)
   - [4.8 Covariance Health Pattern](#48-covariance-health-pattern)
5. [Safety-Critical C++ Standards](#5-safety-critical-c-standards)
   - [5.1 Mandatory Constructs](#51-mandatory-constructs)
   - [5.2 Forbidden Patterns](#52-forbidden-patterns)
   - [5.3 Code Review Gate](#53-code-review-gate)
6. [Concept Map — Where Patterns Intersect](#6-concept-map--where-patterns-intersect)
7. [Glossary](#7-glossary)

---

## 1. Design Patterns

### 1.1 Strategy Pattern (Polymorphic Interfaces)

**What:** Define a family of algorithms behind a common interface.  The caller
codes to the interface; the concrete implementation is selected at runtime.

**Why:** The drone stack needs to run with different hardware — a simulated
camera during testing vs. a Gazebo camera in SITL vs. a real V4L2 camera on
hardware.  The Strategy pattern lets every process work identically regardless
of which backend is plugged in.

**Where in the codebase:**

| Interface | File | Concrete Implementations |
|-----------|------|--------------------------|
| `ICamera` | `common/hal/include/hal/icamera.h` | `SimulatedCamera`, `GazeboCameraBackend` |
| `IFCLink` | `common/hal/include/hal/ifc_link.h` | `SimulatedFCLink`, `MavlinkFCLink` |
| `IGCSLink` | `common/hal/include/hal/igcs_link.h` | `SimulatedGCSLink` |
| `IGimbal` | `common/hal/include/hal/igimbal.h` | `SimulatedGimbal` |
| `IIMUSource` | `common/hal/include/hal/iimu_source.h` | `SimulatedIMU`, `GazeboIMUBackend` |
| `IDetector` | `process2_perception/include/perception/detector_interface.h` | `SimulatedDetector`, `OpenCvYoloDetector`, `ColorContourDetector` |
| `ITracker` | `process2_perception/include/perception/itracker.h` | `ByteTrackTracker` |
| `IPublisher<T>` | `common/ipc/include/ipc/ipublisher.h` | `ZenohPublisher<T>` |
| `ISubscriber<T>` | `common/ipc/include/ipc/isubscriber.h` | `ZenohSubscriber<T>` |
| `IPathPlanner` | `process4_mission_planner/include/planner/ipath_planner.h` | `DStarLitePlanner` |
| `IObstacleAvoider` | `process4_mission_planner/include/planner/iobstacle_avoider.h` | `ObstacleAvoider3D` |
| `IVIOBackend` | `process3_slam_vio_nav/include/slam/ivio_backend.h` | `SimulatedVIOBackend`, `GazeboVIOBackend`, `GazeboFullVIOBackend` |
| `IProcessMonitor` | `process7_system_monitor/include/monitor/iprocess_monitor.h` | `LinuxProcessMonitor` |

**Example — `ICamera` interface** (`common/hal/include/hal/icamera.h`):

```cpp
class ICamera {
public:
    virtual ~ICamera() = default;

    virtual bool open(uint32_t width, uint32_t height, int fps) = 0;
    virtual void close() = 0;
    virtual CapturedFrame capture() = 0;   // blocks until frame ready
    virtual bool is_open() const = 0;
    virtual std::string name() const = 0;
};
```

**Key C++ features used:**
- **Pure virtual functions** (`= 0`): Forces every concrete class to implement the method.
- **Virtual destructor** (`virtual ~ICamera() = default`): Ensures proper cleanup when deleting through a base pointer.
- **`= default`**: Tells the compiler to generate the default destructor body (no custom cleanup in the base).

**Usage** — the process doesn't know or care which camera it's using:

```cpp
// Caller code works with ANY ICamera implementation
std::unique_ptr<ICamera> cam = hal::create_camera(cfg, "video_capture.mission_cam");
cam->open(1280, 720, 30);
auto frame = cam->capture();  // Could be simulated, Gazebo, or V4L2
```

---

### 1.2 Factory Pattern

**What:** A function that creates and returns objects without the caller knowing
the exact concrete type.

**Why:** The correct backend implementation depends on runtime configuration
(a JSON file).  A factory centralises the `if/else` selection logic in one
place — every other file just calls the factory.

**Where:** `common/hal/include/hal/hal_factory.h`, `common/ipc/include/ipc/message_bus_factory.h`

**Example — HAL Camera Factory** (`common/hal/include/hal/hal_factory.h`):

```cpp
inline std::unique_ptr<ICamera> create_camera(
    const drone::Config& cfg, const std::string& section)
{
    auto backend = cfg.get<std::string>(section + ".backend", "simulated");

    if (backend == "simulated") {
        return std::make_unique<SimulatedCamera>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        auto gz_topic = cfg.get<std::string>(section + ".gz_topic", "/camera");
        return std::make_unique<GazeboCameraBackend>(gz_topic);
    }
#endif
    throw std::runtime_error("[HAL] Unknown camera backend: " + backend);
}
```

**Key C++ features used:**
- **`std::unique_ptr`**: The factory returns ownership via a smart pointer — no manual `delete` needed.
- **`std::make_unique<T>(args...)`**: Constructs the object directly on the heap, exception-safe.
- **`#ifdef` guards**: Only compile Gazebo code if the dependency is available at build time.

---

### 1.3 Singleton Pattern

**What:** Ensure a class has exactly one instance, accessible globally.

**Why:** All Zenoh publishers and subscribers in a process must share the same
underlying Zenoh session.  Creating multiple sessions would waste resources and
break SHM buffer sharing.

**Where:** `common/ipc/include/ipc/zenoh_session.h`

```cpp
class ZenohSession {
public:
    // Access the singleton — constructed once, on first call
    static ZenohSession& instance() {
        static ZenohSession* inst = new ZenohSession();  // intentionally leaked
        return *inst;
    }

    zenoh::Session& session() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!session_.has_value()) {
            open_session();   // lazy initialization
        }
        return session_.value();
    }

    // Non-copyable, non-movable — there can be only one
    ZenohSession(const ZenohSession&)            = delete;
    ZenohSession& operator=(const ZenohSession&) = delete;
    ZenohSession(ZenohSession&&)                 = delete;
    ZenohSession& operator=(ZenohSession&&)      = delete;

private:
    ZenohSession() = default;       // private constructor
    mutable std::mutex mutex_;
    std::optional<zenoh::Session> session_;
};
```

**Why is it leaked?**  The Zenoh C library (Rust FFI) panics if its session
destructor runs during `atexit()`.  Heap-allocating with `new` and never
`delete`-ing avoids the crash.  The OS reclaims all process memory on exit.

**Key C++ features used:**
- **`static` local variable**: C++11 guarantees thread-safe initialization of function-local statics (the "Meyers singleton").
- **Deleted copy/move**: The four `= delete` lines make it impossible to accidentally create a second instance.
- **`mutable std::mutex`**: Allows locking in `const` methods.
- **`std::optional`**: Allows lazy initialization — the session isn't created until first use.

> **Alternative considered:** `static ZenohSession inst;` (stack-based singleton).
> Rejected because Zenoh's destructor runs during `atexit()` and panics.

---

### 1.4 State Machine Pattern

**What:** Model an entity's lifecycle as a set of states with explicit
transitions.

**Why:** A drone mission has a clear lifecycle: IDLE → PREFLIGHT → TAKEOFF →
NAVIGATE → ... → LAND → IDLE.  An explicit FSM makes the legal transitions
visible and prevents invalid state combinations.

**Where:** `common/ipc/include/ipc/ipc_types.h` (the enum), `process4_mission_planner/include/planner/mission_fsm.h` (the FSM class)

```cpp
// States are a scoped enum — no implicit conversion to int
// Defined in ipc_types.h so it can be shared across processes via IPC
enum class MissionState : uint8_t {
    IDLE, PREFLIGHT, TAKEOFF, NAVIGATE, LOITER, RTL, LAND, EMERGENCY, SURVEY
};

class MissionFSM {
public:
    MissionFSM() : state_(MissionState::IDLE) {}

    MissionState state() const { return state_; }

    // Each event triggers a state transition
    void on_arm()       { transition(MissionState::PREFLIGHT); }
    void on_takeoff()   { transition(MissionState::TAKEOFF); }
    void on_navigate()  { transition(MissionState::NAVIGATE); }
    void on_rtl()       { transition(MissionState::RTL); }
    void on_landed()    { transition(MissionState::IDLE); }
    void on_emergency() { transition(MissionState::EMERGENCY); }

private:
    MissionState state_;

    void transition(MissionState new_state) {
        spdlog::info("[FSM] {} → {}", state_name(state_), state_name(new_state));
        state_ = new_state;
    }
};
```

**Usage** (in the mission planner main loop):

```cpp
switch (fsm.state()) {
    case MissionState::IDLE:      /* wait for arm command */  break;
    case MissionState::TAKEOFF:   /* command climb to 10m */  break;
    case MissionState::NAVIGATE:  /* follow waypoints */      break;
    case MissionState::RTL:       /* return to launch */      break;
    // ...
}
```

---

### 1.5 Observer / Publish-Subscribe Pattern

**What:** One component publishes data; zero or more components receive it
asynchronously, without the publisher knowing who (or how many) subscribers
exist.

**Why:** Process 1 (Video Capture) publishes camera frames.  Process 2
(Perception) and Process 3 (SLAM) both read them.  Neither publisher nor
subscribers need to know about each other — total decoupling.

**Where:** The entire `common/ipc/` layer implements this:

```
                ┌──────────────┐
                │  IPublisher  │ ◄── ZenohPublisher<T>
                └──────┬───────┘
                       │ publish(msg)
              ┌────────┴─────────┐
              ▼                  ▼
       ┌─────────────┐   ┌─────────────┐
       │ ISubscriber  │   │ ISubscriber  │  ◄── ZenohSubscriber<T>
       └─────────────┘   └─────────────┘
         Process 2           Process 3
```

**Liveliness monitoring** is another observer pattern — processes declare
"I'm alive" tokens, and the System Monitor observes births/deaths:

```cpp
// Process 1 declares: "I'm alive"
drone::ipc::LivelinessToken token("video_capture");  // RAII — auto-undeclare on exit

// Process 7 observes all processes:
drone::ipc::LivelinessMonitor monitor(
    [](const std::string& name) { spdlog::info("{} came alive", name); },
    [](const std::string& name) { spdlog::warn("{} died!", name); }
);
```

---

### 1.6 RAII (Resource Acquisition Is Initialization)

**What:** Tie a resource's lifetime to an object's lifetime.  The constructor
acquires; the destructor releases.  No manual cleanup needed.

**Why:** In systems programming, forgetting to `munmap()`, `close()`, or
`shm_unlink()` causes resource leaks.  RAII makes it impossible to forget.

**Where in the codebase:**

| Class | Resource Managed | File |
|-------|-----------------|------|
| `LivelinessToken` | Zenoh liveliness key (declare → undeclare) | `common/ipc/include/ipc/zenoh_liveliness.h` |
| `ScopedTimer` | Wall-clock measurement (start → log on destroy) | `common/util/include/util/scoped_timer.h` |
| `std::lock_guard<std::mutex>` | Mutex lock (lock → unlock) | Used throughout |

**Example — `ScopedTimer`** (`common/util/include/util/scoped_timer.h`):

```cpp
class ScopedTimer {
public:
    ScopedTimer(const char* label, double warn_ms = 0.0)
        : label_(label), warn_ms_(warn_ms),
          start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        const auto end = std::chrono::steady_clock::now();
        const double ms =
            std::chrono::duration<double, std::milli>(end - start_).count();
        if (warn_ms_ > 0.0 && ms > warn_ms_)
            spdlog::warn("{}: {:.2f} ms (limit: {:.1f} ms)", label_, ms, warn_ms_);
    }
private:
    const char* label_;
    double warn_ms_;
    std::chrono::steady_clock::time_point start_;
};

// Usage — just declare it; timing is automatic:
{
    ScopedTimer t("YOLOv8 inference", 33.0);   // warn if >33ms (30 FPS budget)
    detector->detect(frame, detections);
}   // ~ScopedTimer() logs elapsed time here
```

**Example — `ShmWriter<T>` destructor** (conceptual POSIX SHM design — not currently implemented; Zenoh is the active IPC backend):

```cpp
~ShmWriter() {
    if (ptr_) munmap(ptr_, sizeof(ShmBlock));   // unmap shared memory
    if (fd_ >= 0) {
        shm_unlink(name_.c_str());              // remove the /dev/shm entry
        close(fd_);                             // close the file descriptor
    }
}
```

No matter how the function exits — normal return, exception, early return — the
destructor runs and resources are released.

---

### 1.7 Adapter Pattern

**What:** Wrap a foreign API behind a local interface so it can be used
interchangeably with other implementations.

**Where:** `GazeboCameraBackend` wraps the gz-transport subscription API
behind `ICamera`.  `MavlinkFCLink` wraps the MAVSDK API behind `IFCLink`.
`ZenohMessageBus` maps SHM-style topic names to Zenoh key expressions.

```
   ICamera (our interface)          GazeboCameraBackend (adapter)
   ┌──────────────────┐            ┌────────────────────────────┐
   │ open()           │────────────│ subscribes to gz-transport │
   │ capture()        │────────────│ waits on condition_variable│
   │ close()          │────────────│ unsubscribes               │
   └──────────────────┘            └────────────────────────────┘
```

---

## 2. Multithreading & Concurrency

### 2.1 `std::thread` and Thread Lifecycle

**What:** `std::thread` represents a single thread of execution.  You construct
it with a callable (function pointer, lambda, functor), and it starts
immediately.

**Key rule:** You must either `.join()` (wait for completion) or `.detach()`
(fire-and-forget) before the `std::thread` object is destroyed.  Otherwise, the
program calls `std::terminate()`.

**Where:** Every process's `main.cpp` spawns dedicated threads:

```cpp
// Process 2 — Perception (process2_perception/src/main.cpp)
std::thread t_inference(inference_thread, ...);
std::thread t_tracker(tracker_thread, ...);
std::thread t_fusion(fusion_thread, ...);

// ... main loop runs ...

g_running.store(false);       // signal all threads to stop
t_inference.join();           // wait for each thread to finish
t_tracker.join();
t_fusion.join();
```

**Why not `std::jthread`?** `std::jthread` (C++20) auto-joins in its
destructor and supports cooperative cancellation via `std::stop_token`.  This
codebase targets C++17, so it uses `std::thread` + `std::atomic<bool>` for
shutdown signaling.

---

### 2.2 `std::mutex` and `std::lock_guard`

**What:** A mutex (mutual exclusion) prevents two threads from accessing
shared state simultaneously.  `std::lock_guard` is an RAII wrapper that
locks in its constructor and unlocks in its destructor.

**Where:** `ZenohSession`, `ZenohSubscriber`, `SimulatedFCLink`, `MavlinkFCLink`, and more.

```cpp
// ZenohSession — every method that touches session_ acquires the mutex
class ZenohSession {
    mutable std::mutex mutex_;
    std::optional<zenoh::Session> session_;

public:
    zenoh::Session& session() {
        std::lock_guard<std::mutex> lock(mutex_);  // locked here
        if (!session_.has_value())
            open_session();
        return session_.value();
    }   // lock released here — even if an exception is thrown
};
```

**Why `mutable`?** The mutex needs to be locked even in `const` methods
(e.g. `is_open() const`).  Without `mutable`, the compiler won't let
you lock a mutex inside a `const` function.

**Key pitfall: Deadlock.**  If you hold mutex A and then try to lock mutex B,
while another thread holds B and tries to lock A, both threads block forever.
This codebase avoids deadlock by:
1. Using `std::lock_guard` (never manually unlocking).
2. Each class has at most one mutex.
3. No nested locking across classes in a single call chain.

---

### 2.3 `std::condition_variable`

**What:** Allows a thread to sleep until another thread signals that some
condition is true.  More efficient than busy-waiting (spin loops).

**Where:** `GazeboCameraBackend` — the `capture()` method blocks until a
gz-transport callback delivers a new frame:

```cpp
CapturedFrame capture() override {
    std::unique_lock<std::mutex> lock(mtx_);
    // Wait until frame_ready_ is true OR camera was closed (timeout 100ms)
    cv_.wait_for(lock, 100ms, [this] { return frame_ready_ || !open_; });
    if (!frame_ready_) return {};
    frame_ready_ = false;
    return front_frame_;
}

// Called by gz-transport on a different thread:
void on_gz_image(const gz::msgs::Image& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    // Copy image data into back buffer
    frame_ready_ = true;
    cv_.notify_one();   // wake up capture() thread
}
```

**Why `unique_lock` not `lock_guard`?**  `condition_variable::wait()` needs to
temporarily unlock the mutex while the thread sleeps and re-lock it when waking
up.  `std::lock_guard` doesn't support unlock/re-lock — `std::unique_lock` does.

---

### 2.4 `std::atomic` and Memory Ordering

**What:** `std::atomic<T>` provides lock-free operations on primitive types
with guaranteed visibility across CPU cores.

**Why raw loads/stores aren't enough:** Modern CPUs reorder instructions and
cache memory independently per core.  Without atomics, one core's write
might not be visible to another core for an arbitrary amount of time.

**Memory ordering in this codebase:**

| Ordering | Meaning | Where Used |
|----------|---------|------------|
| `memory_order_relaxed` | No ordering guarantees — just atomicity | `g_running` shutdown flag, statistics counters |
| `memory_order_acquire` | All memory writes before the *paired release* are visible to this thread | SeqLock reader, SPSC ring consumer |
| `memory_order_release` | All memory writes before this store are visible to threads that *acquire* | SeqLock writer, SPSC ring producer |

**Example — shutdown flag** (every process):

```cpp
static std::atomic<bool> g_running{true};
SignalHandler::install(g_running);  // registers g_running with the signal handler

// Worker thread (loops until shutdown):
while (g_running.load(std::memory_order_relaxed)) {
    // ... do work ...
}
```

`relaxed` is fine here because we don't need to synchronize any *other* data
with the flag — we just need every thread to eventually see `false`.

---

### 2.5 SeqLock — A Custom Lock-Free Protocol

**What:** A SeqLock (sequence lock) is a read-write synchronization mechanism
where the writer never blocks and readers retry if they detect a concurrent
write.

**Why:** In the POSIX SHM IPC path, one process writes sensor data (e.g.
camera frames at 30 Hz) and multiple processes read it.  A mutex would make
the writer block if a reader is slow — unacceptable for real-time.  SeqLock
gives us:
- **Writer never blocks** — always O(1)
- **Readers are wait-free** (optimistic retry)
- **Zero system calls** — everything happens in user-space via atomics

**How it works:**

```
seq=0 (even → stable)
              Writer                    Reader
              ──────                    ──────
              seq → 1 (odd = writing)   reads seq=1 → odd, RETRY
              memcpy(data)
              seq → 2 (even = done)     reads seq=2 → even
                                        memcpy(data)
                                        fence(acquire)
                                        reads seq=2 → matches → SUCCESS
```

**Where:** Conceptual POSIX SHM design (not currently implemented — Zenoh is the active IPC backend). The pattern is shown below for educational purposes:

```cpp
// Writer — never blocks, always succeeds
void write(const T& data) {
    uint64_t s = ptr_->seq.load(std::memory_order_relaxed);
    ptr_->seq.store(s + 1, std::memory_order_release);   // odd = writing
    std::memcpy(&ptr_->data, &data, sizeof(T));
    ptr_->seq.store(s + 2, std::memory_order_release);   // even = done
}

// Reader — retries up to 4 times on torn reads
bool read(T& out) const {
    for (int attempt = 0; attempt < 4; ++attempt) {
        uint64_t s1 = ptr_->seq.load(std::memory_order_acquire);
        if (s1 & 1) continue;                             // writer mid-write
        std::memcpy(&out, &ptr_->data, sizeof(T));
        std::atomic_thread_fence(std::memory_order_acquire);
        uint64_t s2 = ptr_->seq.load(std::memory_order_relaxed);
        if (s1 == s2) return true;                        // consistent read
    }
    return false;   // torn read after 4 attempts
}
```

**Why the fence?** The `acquire` fence *after* the `memcpy` ensures the CPU
doesn't reorder the `memcpy` after the second `seq` load.  Without it, the
compiler or CPU could load `s2` before finishing the data copy, making the
consistency check useless.

---

### 2.6 Lock-Free SPSC Ring Buffer

**What:** A single-producer single-consumer (SPSC) queue that requires no
locks or atomic read-modify-write operations — only atomic loads and stores.

**Why:** Within Process 2 (Perception), the inference thread produces
detections and the tracker thread consumes them.  A lock would cause
priority inversion if the tracker stalls.  SPSC ring buffers give O(1)
push/pop with zero contention.

**Where:** `common/util/include/util/spsc_ring.h`

```cpp
template <typename T, size_t N>
class SPSCRing {
    static_assert((N & (N - 1)) == 0, "N must be power of 2");
public:
    bool try_push(const T& item) {
        const uint64_t w = write_idx_.load(std::memory_order_relaxed);
        const uint64_t r = read_idx_.load(std::memory_order_acquire);
        if (w - r >= N) return false;                  // full
        slots_[w & (N - 1)] = item;                    // power-of-2 mask
        write_idx_.store(w + 1, std::memory_order_release);
        return true;
    }

    std::optional<T> try_pop() {
        const uint64_t r = read_idx_.load(std::memory_order_relaxed);
        const uint64_t w = write_idx_.load(std::memory_order_acquire);
        if (r >= w) return std::nullopt;               // empty
        T item = slots_[r & (N - 1)];
        read_idx_.store(r + 1, std::memory_order_release);
        return item;
    }

private:
    alignas(64) std::atomic<uint64_t> write_idx_{0};   // producer's variable
    alignas(64) std::atomic<uint64_t> read_idx_{0};    // consumer's variable
    std::array<T, N> slots_{};
};
```

**Key design decisions:**
- **Power-of-2 size**: `w & (N-1)` is a fast modulo (bit mask, single CPU instruction) compared to `w % N` (integer division).
- **`alignas(64)`**: See [§3.11](#311-alignas--cache-line-alignment) — prevents false sharing.
- **`std::optional` return**: No exceptions, no out-parameters — the caller checks `has_value()`.

---

### 2.7 Double Buffering

**What:** Maintain two copies of data; the writer updates the inactive copy
while the reader accesses the active copy, then atomically swap.

**Where:** `process3_slam_vio_nav/src/main.cpp` — SLAM pose exchange:

```cpp
class PoseDoubleBuffer {
public:
    void write(const Pose& pose) {
        int idx = write_idx_.load(std::memory_order_relaxed) ^ 1;  // inactive slot
        buffers_[idx] = pose;                                       // write to it
        write_idx_.store(idx, std::memory_order_release);           // swap
        initialized_.store(true, std::memory_order_release);        // mark ready
    }

    bool read(Pose& out) const {
        int idx = write_idx_.load(std::memory_order_acquire);       // active slot
        if (!initialized_.load(std::memory_order_acquire)) return false;
        out = buffers_[idx];
        return true;
    }

private:
    Pose buffers_[2];
    std::atomic<int> write_idx_{0};
    std::atomic<bool> initialized_{false};
};
```

**Why not a mutex?** The SLAM pose is updated at 100+ Hz and read by the
navigation thread at the same rate.  A mutex would cause unnecessary blocking.
The atomic XOR swap is a single instruction.

---

### 2.8 Signal-Safe Shutdown

**What:** Handle Unix signals (SIGINT, SIGTERM) safely to trigger graceful
process shutdown.

**Where:** `common/util/include/util/signal_handler.h`

```cpp
class SignalHandler {
public:
    static void install(std::atomic<bool>& running_flag) {
        s_running_ = &running_flag;

        struct sigaction sa{};
        sa.sa_handler = handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;   // don't restart blocking calls

        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGINT,  &sa, nullptr);

        // Ignore SIGPIPE (broken pipe from serial/UDP)
        struct sigaction sa_ignore{};
        sa_ignore.sa_handler = SIG_IGN;
        sigaction(SIGPIPE, &sa_ignore, nullptr);
    }

private:
    static inline std::atomic<bool>* s_running_ = nullptr;

    static void handler(int sig) {
        // ONLY async-signal-safe operations allowed here!
        if (s_running_)
            s_running_->store(false, std::memory_order_relaxed);
        static constexpr char msg[] = "Signal received, shutting down...\n";
        [[maybe_unused]] auto r = write(STDOUT_FILENO, msg, sizeof(msg) - 1);
        (void)sig;
    }
};
```

**Why `write()` and not `std::cout` or `spdlog`?** Inside a signal handler,
only *async-signal-safe* functions are allowed.  `write(2)` is safe;
`printf`, `malloc`, `new`, mutex operations, and most of the C++ standard
library are NOT.  Calling them can deadlock or corrupt memory.

**Why `sa.sa_flags = 0`?** Without `SA_RESTART`, blocking calls like `read()`,
`sleep()`, and `wait()` return with `errno = EINTR` when a signal is delivered.
This ensures threads wake up promptly from blocking I/O after a signal.

---

### 2.9 Thread Naming, CPU Pinning & RT Scheduling

**What:** On Linux, you can name threads (visible in `htop`/`top`), pin them
to specific CPU cores, and set real-time scheduling priority.

**Where:** `common/util/include/util/realtime.h`

```cpp
inline void set_thread_params(const char* name, int core,
                               int policy, int priority) {
    pthread_setname_np(pthread_self(), name);      // name (max 15 chars)

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);  // pin to core

    if (policy == SCHED_FIFO || policy == SCHED_RR) {
        sched_param param{};
        param.sched_priority = priority;
        pthread_setschedparam(pthread_self(), policy, &param);  // RT priority
    }
}

// Usage in the comms process:
std::thread fc_rx([&] {
    set_thread_params("fc_rx", 2, SCHED_FIFO, 80);
    while (g_running.load()) { /* ... */ }
});
```

**Why pin threads?** In real-time systems, you want deterministic latency.
CPU migration (the OS moving a thread between cores) invalidates caches and
adds jitter.  Pinning to a specific core eliminates this.

**Why `SCHED_FIFO`?** Normal Linux scheduling (`SCHED_OTHER`) can preempt
your thread for up to ~10ms.  `SCHED_FIFO` gives the thread strict priority —
it runs until it voluntarily yields or blocks.  Critical for flight controller
communication.

---

### 2.10 Compare-And-Swap (CAS) Loop

**What:** A lock-free pattern for atomically claiming a slot in a shared array.
Instead of `fetch_add` (which can skip slots on failure), a CAS loop retries
until the compare-and-exchange succeeds, guaranteeing exactly one winner per slot.

**Where:** `common/util/include/util/thread_heartbeat.h` — `ThreadHeartbeatRegistry::register_thread()`

```cpp
// Simplified from ThreadHeartbeatRegistry::register_thread()
size_t register_thread(const char* name, bool critical) {
    size_t current = count_.load(std::memory_order_relaxed);
    while (true) {
        if (current >= kMaxThreads) return kInvalidHandle;  // full

        // Try to claim the slot at index 'current'
        if (count_.compare_exchange_weak(current, current + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
            // Won the race — 'current' is our slot
            safe_name_copy(beats_[current].name, name);
            beats_[current].is_critical = critical;
            beats_[current].initialized.store(true, std::memory_order_release);
            return current;   // handle
        }
        // Lost — 'current' updated to the new count; retry
    }
}
```

**Why `compare_exchange_weak` over `fetch_add`?**

| Approach | Guarantee | Problem |
|----------|-----------|---------|
| `fetch_add(1)` | Returns old value, always increments | If two threads call simultaneously, both increment past `kMaxThreads` — no bounds check is possible |
| `compare_exchange_weak` | Only increments if the current value matches expectation | Naturally loops back with updated value on contention; bounds check inside the loop is reliable |

**Memory ordering:**
- **`acq_rel` on success:** ensures prior writes (name, critical flag) are visible
  to snapshot readers, and subsequent writes to the slot are ordered after the claim
- **`relaxed` on failure:** we only need the updated value of `count_`; no
  ordering constraints for the retry path
- **`release` on `initialized`:** snapshot readers use `acquire` load to gate
  on this flag, ensuring they see a fully-written slot

**When to use a CAS loop:**
- Fixed-size slot arrays with bounded concurrency
- Registration / init paths (not hot loops)
- When you need bounds-checked atomic increment

---

## 3. Modern C++ Techniques

### 3.1 `std::variant` and `std::visit`

**What:** `std::variant` is a type-safe union — it holds exactly one of
several types at a time.  `std::visit` dispatches a callable to the currently
held type.

**Why:** The message bus backend is extensible — currently only `ZenohMessageBus`,
but the architecture supports adding new backends (e.g. iceoryx, DDS) by adding
one arm to the variant. `std::variant` provides type-safe runtime dispatch
without inheritance (the backends are templated factories).

**Where:** `common/ipc/include/ipc/message_bus.h`

```cpp
// The variant — holds exactly one bus type (extensible for future backends)
namespace detail {
using BusVariant = std::variant<std::unique_ptr<ZenohMessageBus>
                                // #ifdef HAVE_ICEORYX
                                // , std::unique_ptr<IceoryxMessageBus>
                                // #endif
                                >;
}

// MessageBus wraps the variant and dispatches via std::visit
class MessageBus {
public:
    template<typename T>
    std::unique_ptr<IPublisher<T>> advertise(const std::string& topic) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IPublisher<T>> {
                return b->template advertise<T>(topic);
            },
            impl_);
    }
private:
    detail::BusVariant impl_;
};
```

**How `std::visit` works:** The lambda `[&](auto& b)` is a *generic lambda* —
the compiler generates one version for each type in the variant.  At runtime,
`std::visit` calls the version matching the currently held type.

**`b->template advertise<T>(topic)`**: The `template` keyword is required
because `b` is a dependent type (its type depends on the template parameter).
Without it, the compiler parses `<T>` as a less-than operator.

---

### 3.2 `std::optional`

**What:** A wrapper that either contains a value or is empty (`std::nullopt`).
Safer than using sentinel values (`-1`, `nullptr`, `false`) to indicate "no value".

**Where used:**

```cpp
// SPSC Ring — try_pop() returns nullopt when empty
std::optional<T> try_pop() {
    if (r >= w) return std::nullopt;   // nothing to read
    return item;
}

// Zenoh session — lazily initialized
std::optional<zenoh::Session> session_;   // empty until first use

// Service channel — response might not arrive
std::optional<ServiceResponse<Resp>> call(const Req& request, uint64_t timeout_ms);
```

**Why not raw pointers?** `std::optional` is a value type — it lives on the
stack, has no heap allocation, and makes the "empty" case explicit in the type
signature.

---

### 3.3 `if constexpr` — Compile-Time Branching

**What:** `if constexpr` evaluates a condition at compile time.  The false
branch is completely discarded — it doesn't even need to be valid C++ for that
particular template instantiation.

**Where:** Zenoh publisher chooses SHM vs bytes path based on message size:

```cpp
void publish(const T& msg) override {
    if constexpr (sizeof(T) > kShmPublishThreshold) {
        publish_shm(msg);      // zero-copy path for large messages (>64KB)
    } else {
        publish_bytes(msg);    // memcpy path for small messages
    }
}
```

**Why not regular `if`?** A regular `if` would require both branches to
compile for every `T`.  But `publish_shm()` uses SHM-specific APIs that might
not make sense for small types.  `if constexpr` eliminates the unused branch
entirely.

**Another example** — bus-type-aware service channel creation:

```cpp
if constexpr (std::is_same_v<BusType, ZenohMessageBus>) {
    return b->template create_client<Req, Resp>(service, timeout_ms);
}
// else: SHM doesn't support service channels, return nullptr
```

---

### 3.4 `static_assert` — Compile-Time Contracts

**What:** Fails compilation with a custom error message if a condition is false.

**Why:** The Zenoh IPC publisher uses zero-copy transfer for IPC messages.
This only works for trivially copyable types (no virtual tables, no heap
pointers, no non-trivial constructors).  `static_assert` catches violations
at compile time:

```cpp
template <typename T>
class ZenohPublisher : public IPublisher<T> {
    static_assert(std::is_trivially_copyable_v<T>,
                  "IPC payload must be trivially copyable");
    // ...
};
```

If someone tries `ZenohPublisher<std::string>`, the compiler emits:
```
error: static assertion failed: SHM payload must be trivially copyable
```

**Other uses:**

```cpp
// SPSC ring: capacity must be power of 2
static_assert((N & (N - 1)) == 0, "N must be power of 2");

// Wire header: exact binary layout
static_assert(sizeof(WireHeader) == 24, "WireHeader must be exactly 24 bytes");
```

---

### 3.5 Smart Pointers (`unique_ptr`, `shared_ptr`)

**What:** Smart pointers automate memory management — no more `new`/`delete`.

| Type | Ownership | Overhead | Where Used |
|------|-----------|----------|------------|
| `std::unique_ptr<T>` | Exclusive (single owner) | Zero (same as raw pointer) | Factory returns, publishers, subscribers, HAL backends |
| `std::shared_ptr<T>` | Shared (reference counted) | ~16 bytes + atomic refcount | Zenoh service callbacks, MAVSDK system, loggers |

```cpp
// Factory returns unique_ptr — caller owns the object
std::unique_ptr<ICamera> cam = create_camera(cfg, "video_capture.mission_cam");

// shared_ptr — multiple threads share ownership of the callback queue
auto pending = std::make_shared<PendingQueue>();
// Lambda captures shared_ptr — the queue lives as long as any lambda exists
auto callback = [pending](const zenoh::Query& query) {
    std::lock_guard lock(pending->mutex);
    pending->queue.push_back(query);
};
```

**Rule of thumb:** Use `unique_ptr` by default.  Only use `shared_ptr` when
you genuinely need multiple owners with independent lifetimes.

---

### 3.6 Move Semantics

**What:** Move semantics allow transferring ownership of resources (heap
memory, file descriptors, SHM mappings) from one object to another without
copying.  The source is left in a valid but empty state.

**Where:** Conceptual `ShmWriter` / `ShmReader` RAII wrappers (not currently
implemented — Zenoh is the active IPC backend). The pattern illustrates how
move semantics transfer ownership of raw OS handles:

```cpp
// ShmWriter move constructor — transfers ownership of fd and mmap
ShmWriter(ShmWriter&& other) noexcept
    : fd_(other.fd_), ptr_(other.ptr_), name_(std::move(other.name_))
{
    other.fd_  = -1;       // source no longer owns the fd
    other.ptr_ = nullptr;  // source no longer owns the mapping
}

// Copy is deleted — you can't have two objects owning the same SHM segment
ShmWriter(const ShmWriter&) = delete;
ShmWriter& operator=(const ShmWriter&) = delete;
```

**`noexcept`:** Move constructors should be `noexcept` because `std::vector`
and other containers will only use move (instead of copy) during reallocation
if the move constructor is marked `noexcept`.

---

### 3.7 Lambda Expressions

**What:** Anonymous functions defined inline.  Capture variables from the
surrounding scope by value `[=]`, by reference `[&]`, or selectively `[this]`,
`[x, &y]`.

**Where used throughout the codebase:**

```cpp
// [&] capture — generic lambda for std::visit
std::visit([&](auto& b) -> std::unique_ptr<IPublisher<T>> {
    return b->template advertise<T>(topic);
}, bus);

// [this] capture — Zenoh callback
session.declare_subscriber(key,
    [this](const zenoh::Sample& sample) {
        std::lock_guard lock(data_mutex_);
        // deserialize sample into cached value
    });

// shared_ptr capture — ensure queue outlives the callback
auto responses = std::make_shared<ResponseQueue>();
session.get(key,
    [responses](const zenoh::Reply& reply) {
        std::lock_guard lock(responses->mutex);
        responses->deque.push_back(reply);
    });

// Predicate lambda — condition variable
cv_.wait_for(lock, 100ms, [this] { return frame_ready_ || !open_; });
```

**Capture pitfall:** Capturing `[this]` in a callback is dangerous if the
object is destroyed before the callback fires.  The Zenoh service classes solve
this by capturing a `shared_ptr` to the queue instead of `this`.

---

### 3.8 `constexpr` — Compile-Time Constants

**What:** `constexpr` values are computed at compile time and baked into the
binary as constants.

```cpp
// SHM segment names — evaluated at compile time, stored in read-only memory
namespace shm_names {
    constexpr const char* VIDEO_MISSION_CAM = "/drone_mission_cam";
    constexpr const char* SLAM_POSE         = "/slam_pose";
    constexpr const char* SYSTEM_HEALTH     = "/system_health";
    // ...
}

// Zenoh constants
static constexpr std::size_t kDefaultShmPoolBytes = 32 * 1024 * 1024;  // 32 MB
static constexpr std::size_t kShmPublishThreshold = 64 * 1024;         // 64 KB

// Wire format
static constexpr uint32_t kWireMagic = 0x4E4F5244;  // "DRON" in ASCII (LE)
```

**Why not `#define`?** `constexpr` is type-safe, respects scope and namespace,
and can be debugged with a debugger.  `#define` is a text substitution with
no type checking.

---

### 3.9 `std::string_view`

**What:** A non-owning reference to a string (pointer + length).  Zero-copy
alternative to `std::string` for read-only access.

**Where:**

```cpp
// Liveliness — extract process name from key expression
static std::string extract_process_name(const std::string& key) {
    const auto prefix_len = std::string_view(kLivelinessPrefix).size();
    if (key.size() >= prefix_len &&
        key.substr(0, prefix_len) == kLivelinessPrefix) {
        return key.substr(prefix_len);
    }
    return key;
}

// Zenoh sample — zero-copy access to key expression
auto key_sv = sample.get_keyexpr().as_string_view();
```

**When to use:** When you need to read a string but don't need to own or
modify it.  Avoids heap allocation.

---

### 3.10 `enum class` — Scoped Enumerations

**What:** C++11 scoped enums that don't implicitly convert to `int` and don't
pollute the enclosing namespace.

```cpp
// Mission states — can't accidentally compare with integers
enum class MissionState : uint8_t {
    IDLE, PREFLIGHT, TAKEOFF, NAVIGATE, LOITER, RTL, LAND, EMERGENCY, SURVEY
};

// Wire message types — stable numeric values for network protocol
enum class WireMessageType : uint16_t {
    UNKNOWN       = 0,
    VIDEO_FRAME   = 1,
    DETECTIONS    = 10,
    SLAM_POSE     = 20,
    // ...
};

// Service status — clear semantic error codes
enum class ServiceStatus { OK, REJECTED, TIMEOUT, ERROR };
```

**Why not plain `enum`?** Plain enums leak their names into the surrounding
scope and implicitly convert to `int`, leading to bugs like comparing a
`MissionState` with a `WireMessageType`.

---

### 3.11 `alignas` — Cache-Line Alignment

**What:** Forces a variable or struct to start on a specific memory boundary.

**Why:** Modern CPUs have 64-byte cache lines.  When two atomic variables
used by different threads are on the *same* cache line, updating one
invalidates the other's cache — called **false sharing**.  This causes massive
performance degradation.

```cpp
// SPSC Ring — producer and consumer indices on separate cache lines
alignas(64) std::atomic<uint64_t> write_idx_{0};   // producer's variable
alignas(64) std::atomic<uint64_t> read_idx_{0};    // consumer's variable
```

Without `alignas(64)`, both atomics might live within the same 64-byte line:
```
Cache line: [write_idx_ | read_idx_ | ...]
             ^^^^^^^^     ^^^^^^^^
             Core 0       Core 1      ← every write by Core 0 invalidates
                                        Core 1's cache of read_idx_!
```

With `alignas(64)`:
```
Cache line 0: [write_idx_ | padding...]    ← Core 0 only
Cache line 1: [read_idx_  | padding...]    ← Core 1 only
```

**Where else:** `ShmPose` is `alignas(64)` so the SLAM pose struct starts on
a cache-line boundary, ensuring the SeqLock atomic sits at a clean boundary.

---

### 3.12 Structured `__attribute__((packed))`

**What:** Removes all compiler-inserted padding from a struct, giving an
exact binary layout.

**Where:** The wire format header must be exactly 24 bytes for network
compatibility:

```cpp
struct __attribute__((packed)) WireHeader {
    uint32_t        magic;         // 4 bytes  [0..3]
    uint8_t         version;       // 1 byte   [4]
    uint8_t         flags;         // 1 byte   [5]
    WireMessageType msg_type;      // 2 bytes  [6..7]
    uint32_t        payload_size;  // 4 bytes  [8..11]
    uint64_t        timestamp_ns;  // 8 bytes  [12..19]
    uint32_t        sequence;      // 4 bytes  [20..23]
};                                 // Total: 24 bytes

static_assert(sizeof(WireHeader) == 24, "WireHeader must be exactly 24 bytes");
```

**Without `packed`:** A typical ABI will insert 4 bytes of padding before
`timestamp_ns` to align the `uint64_t` to an 8-byte boundary, and then add
tail padding after `sequence` so that the struct's size is a multiple of its
alignment, making it 32 bytes.  The struct received over the network would then
be misinterpreted.

**Tradeoff:** Packed structs can cause slower unaligned memory access on some
architectures (ARM).  This is acceptable for header parsing (done once per
message) but not for high-frequency data structures.

---

### 3.13 `Result<T,E>` — Monadic Error Handling

**What:** A type-safe sum type that holds either a success value (`T`) or an
error value (`E`).  Functions return `Result` instead of throwing exceptions or
returning error codes.

**Why:** In real-time drone software, exceptions are **forbidden** (stack
unwinding has unpredictable latency).  Raw error codes (like returning `bool`
or `-1`) are easy to ignore.  `Result<T,E>` gives the best of both worlds:
structured error information with zero-cost, deterministic control flow.

**Where:** `common/util/include/util/result.h`

```cpp
// Error type with code + message
enum class ErrorCode : uint8_t {
    INVALID_ARGUMENT, NOT_FOUND, IO_ERROR, TIMEOUT,
    PERMISSION_DENIED, ALREADY_EXISTS, NOT_CONNECTED,
    PARSE_ERROR, OUT_OF_RANGE, INTERNAL
};

struct Error {
    ErrorCode   code;
    std::string message;
};

// Result is backed by std::variant — zero heap allocation
template <typename T, typename E = Error>
class Result {
    std::variant<T, E> storage_;
public:
    bool ok() const;
    const T& value() const;     // throws only in debug builds
    const E& error() const;
    T value_or(T fallback) const;
};
```

**Monadic chaining** — compose operations without manual error checking:

```cpp
auto result = cfg.load_config("config/default.json")  // VoidResult
    .and_then([&]() { return cfg.require<int>("video_capture.mission_cam.width"); })
    .map([](int w) { return w * 2; });

if (!result.ok()) {
    spdlog::error("Config error: {}", result.error().message);
    return 1;
}
```

**Design rule for this codebase:** `Result` uses value semantics only —
`std::variant<T, E>` storage with move support.  No `shared_ptr` anywhere
(atomic reference counting is incompatible with real-time constraints).

---

### 3.14 `[[nodiscard]]` — Compiler-Enforced Return Checking

**What:** A C++17 attribute that causes a compiler warning if a function's
return value is silently discarded.

**Why:** In safety-critical code, silently ignoring a return value is almost
always a bug.  If `open()` returns `false` and you don't check it, the system
operates on an uninitialised resource.  `[[nodiscard]]` turns this silent bug
into a compiler warning (and with `-Werror`, a hard error).

**Where:** Every public method returning a value in this codebase is annotated.
26 headers across IPC, HAL, process logic, and Zenoh layers.

```cpp
class ICamera {
public:
    [[nodiscard]] virtual bool open(const std::string& device) = 0;
    [[nodiscard]] virtual bool capture(Frame& frame) = 0;
    [[nodiscard]] virtual bool is_open() const = 0;
    [[nodiscard]] virtual std::string name() const = 0;
    virtual ~ICamera() = default;
};
```

**Call-site patterns:**

```cpp
// ✅ Good — check the return value
if (!camera->open("/dev/video0")) {
    spdlog::error("Camera open failed");
    return 1;
}

// ✅ Good — assert in tests
ASSERT_TRUE(camera->open("/dev/video0"));

// ✅ Good — explicit discard for side-effect-only calls
(void)tracker.update(bbox);  // side-effect: updates internal state

// ❌ Bad — compiler warning (error with -Werror)
camera->open("/dev/video0");  // return value ignored!
```

**Policy:** All new public methods returning values **must** have
`[[nodiscard]]`.  When a return value is intentionally ignored (rare), use
`(void)` cast with a comment explaining why.

**Exception — fire-and-forget side-effect methods:** Some methods return a
diagnostic `bool` that is *informational*, not an error code.  If **every**
call site intentionally discards the return, `[[nodiscard]]` adds noise
(`(void)` casts at every site) without safety benefit.  In these cases, omit
`[[nodiscard]]` and document the fire-and-forget intent:

```cpp
/// Log latency summary if enough samples have been collected.
/// @return true if a summary was logged. Return value is informational
///         (fire-and-forget pattern) — not [[nodiscard]].
virtual bool log_latency_if_due(size_t min_samples = 100) const { return false; }
```

**When to apply this exception (all must be true):**

1. The return value is purely diagnostic ("did I do X?"), not an error or status
   that affects correctness.
2. Every existing and foreseeable call site discards the return.
3. Ignoring the return cannot cause data loss, resource leak, or incorrect
   program state.

If in doubt, add `[[nodiscard]]` — the `(void)` cast cost is low and makes the
intentional discard explicit.  This exception is for high-frequency patterns
(e.g., 24+ call sites in main loops) where the cast would obscure readability.

---

## 4. Systems Programming Concepts

### 4.1 POSIX Shared Memory (`shm_open` / `mmap`)

> **Note:** This section describes a conceptual POSIX SHM design that is not
> currently implemented — Zenoh is the active IPC backend.  The concepts are
> documented here for educational purposes, as the SeqLock and RAII patterns
> shown in §2.5 and §1.6 build on this foundation.

**What:** Inter-process communication by mapping a named memory region into
multiple processes' address spaces.

**How it works:**

```
Process 1 (Writer)              Process 2 (Reader)
─────────────────               ─────────────────
shm_open("/slam_pose", O_CREAT | O_RDWR)
ftruncate(fd, sizeof(Block))
mmap(PROT_READ|PROT_WRITE)      shm_open("/slam_pose", O_RDONLY)
   │                             mmap(PROT_READ)
   │                                │
   ▼ (same physical memory)        ▼
   ┌──────────────────────────────────┐
   │  seq | timestamp | T data       │  ← /dev/shm/slam_pose
   └──────────────────────────────────┘
```

**Conceptual `ShmWriter<T>::create()`:**

```cpp
bool create(const std::string& name) {
    fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);   // create segment
    ftruncate(fd_, sizeof(ShmBlock));                         // set size
    ptr_ = static_cast<ShmBlock*>(
        mmap(nullptr, sizeof(ShmBlock),
             PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));   // map into memory
    ptr_->seq.store(0, std::memory_order_relaxed);           // initialize
    return true;
}
```

**Key point:** The data is in the OS page cache, not in either process's heap.
Both processes see the same physical memory.  No serialization, no copies,
no system calls to read/write — just `memcpy` to/from the mapped pointer.

> **Security note:** The example above uses mode `0666` (world-readable and
> world-writable).  On a multi-user system this allows any local process to
> open the SHM segment by its predictable name and read or tamper with
> flight-critical data.  In production, tighten permissions (e.g., `0600`) and
> run all drone processes under a dedicated user so that only authorized
> processes can access these segments.

---

### 4.2 Binary Wire Format

**What:** A compact binary protocol for encoding messages sent over a network.

**Why not Protobuf/JSON?** Our IPC types are already fixed-size, trivially
copyable C structs.  A binary header + raw bytes is:
- **Zero-dependency** (no codegen, no protobuf runtime)
- **Zero-copy** (just `memcpy` the struct)
- **Fixed overhead** (24-byte header regardless of payload size)

**Protocol layout:**

```
┌────────────────── WireHeader (24 bytes) ──────────────────┐  ┌── Payload ──┐
│ magic(4) │ ver(1) │ flags(1) │ type(2) │ size(4) │ ts(8) │ seq(4) │  T data   │
└───────────────────────────────────────────────────────────┘  └─────────────┘
  0x4E4F5244   1       0         enum      sizeof(T)
  ("DRON" LE)
```

**Serialization** is a zero-overhead `memcpy`:

```cpp
template <typename T>
std::vector<uint8_t> wire_serialize(const T& msg, WireMessageType msg_type, ...) {
    static_assert(std::is_trivially_copyable_v<T>, "...");
    WireHeader hdr;
    hdr.msg_type     = msg_type;
    hdr.payload_size = sizeof(T);

    std::vector<uint8_t> buf(sizeof(WireHeader) + sizeof(T));
    std::memcpy(buf.data(), &hdr, sizeof(WireHeader));
    std::memcpy(buf.data() + sizeof(WireHeader), &msg, sizeof(T));
    return buf;
}
```

> **Security note:** The binary wire format uses `wire_validate` to check the
> magic bytes, version, and payload size, but provides no cryptographic
> integrity or authenticity guarantees.  On an untrusted network, an on-path
> attacker can forge valid-looking `WireHeader` values and arbitrary payloads.
> For deployments where the transport is not inherently secured (e.g., a
> public or shared network), consider adding a message authentication code
> (MAC) over the header and payload, or using a transport layer with
> per-topic authentication, and verify it before calling `wire_deserialize`.

---

### 4.3 Conditional Compilation (`#ifdef`)

**What:** Include or exclude code at compile time based on preprocessor macros.

**Why:** Not every build has the same dependencies.  A CI server might not
have MAVSDK or Gazebo installed.  The code must compile cleanly in all
configurations.

```cpp
// CMakeLists.txt sets HAVE_ZENOH=1 when Zenoh is found
if(ENABLE_ZENOH)
    add_compile_definitions(HAVE_ZENOH=1)
endif()

// In code — only compile Zenoh types when available
#ifdef HAVE_ZENOH
#include "ipc/zenoh_message_bus.h"
#endif

// In common/ipc/include/ipc/message_bus.h:
using BusVariant = std::variant<std::unique_ptr<ZenohMessageBus>
                                // #ifdef HAVE_ICEORYX
                                // , std::unique_ptr<IceoryxMessageBus>
                                // #endif
                                >;
```

**Three guards used in this codebase:**

| Macro | Set by | Controls |
|-------|--------|----------|
| `HAVE_ZENOH` | `-DENABLE_ZENOH=ON` | Zenoh IPC backend |
| `HAVE_MAVSDK` | `find_package(MAVSDK)` | MAVLink flight controller |
| `HAVE_GAZEBO` | `find_package(gz-transport13)` | Gazebo camera/IMU backends |
| `HAVE_SYSTEMD` | `-DENABLE_SYSTEMD=ON` | systemd `sd_notify` integration |

---

### 4.4 `fork` + `exec` — Process Management

**What:** The Unix pattern for spawning a child process: `fork()` creates
a copy of the current process, then `execvp()` replaces that copy with a
new program image.

**Where:** `process7_system_monitor/include/monitor/process_manager.h` — `ProcessManager::launch_one()`

```cpp
// Simplified from ProcessManager::launch_one()
bool launch_one(ManagedProcess& proc) {
    pid_t pid = fork();
    if (pid < 0) return false;   // fork failed

    if (pid == 0) {
        // ═══ CHILD PROCESS ═══
        // 1. Reset signal handlers (inherited from parent)
        signal(SIGTERM, SIG_DFL);
        signal(SIGINT,  SIG_DFL);

        // 2. Close inherited file descriptors (security + resource hygiene)
        close_fds_above(STDERR_FILENO);

        // 3. Build argv from "binary arg1 arg2"
        auto argv = split_args(proc.binary, proc.args);
        std::vector<char*> c_argv;
        for (auto& s : argv) c_argv.push_back(s.data());
        c_argv.push_back(nullptr);

        // 4. Replace this process image
        execvp(c_argv[0], c_argv.data());
        _exit(127);  // execvp failed — use _exit, not exit (no atexit handlers)
    }

    // ═══ PARENT PROCESS ═══
    proc.pid = pid;
    proc.state = ProcessState::RUNNING;
    return true;
}
```

**Key design decisions:**

| Decision | Rationale |
|----------|-----------|
| Reset signal handlers | Child inherits parent's `SIG_IGN` / custom handlers — must restore defaults |
| `close_fds_above(STDERR_FILENO)` | Prevents leaked file descriptors (SHM segments, sockets) from parent |
| `_exit(127)` on exec failure | `exit()` calls `atexit` handlers (parent's cleanup code) — `_exit()` skips them |
| `execvp` (not `execv`) | Searches `PATH` — binary doesn't need full path |
| `split_args()` helper | Builds null-terminated `char*[]` from space-separated arg string |

**Zombie reaping:** The parent calls `waitpid(WNOHANG)` periodically in `tick()`:

```cpp
void reap_children() {
    int status;
    pid_t pid;
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        // Find the ManagedProcess with this PID → set state to RESTARTING
    }
}
```

---

### 4.5 Exponential Backoff

**What:** A retry strategy where the delay between attempts doubles each
time, up to a maximum cap.  Prevents restart storms from overwhelming
the system.

**Where:** `common/util/include/util/restart_policy.h` — `RestartPolicy::backoff_ms()`

```cpp
uint32_t backoff_ms(uint32_t attempt) const {
    // initial_backoff_ms × 2^attempt, capped at max_backoff_ms
    uint32_t delay = initial_backoff_ms;
    for (uint32_t i = 0; i < attempt && delay < max_backoff_ms; ++i) {
        delay *= 2;
    }
    return std::min(delay, max_backoff_ms);
}
```

**Default sequence** (`initial=500ms`, `max=30000ms`):

| Attempt | Delay |
|---------|-------|
| 0 | 500 ms |
| 1 | 1000 ms |
| 2 | 2000 ms |
| 3 | 4000 ms |
| 4 | 8000 ms |
| 5 | 16000 ms |
| 6+ | 30000 ms (cap) |

**Combined with a cooldown window:** If a process stays alive for
`cooldown_window_s` (default 60s), the restart counter resets to zero.
This prevents one early crash from permanently consuming the retry budget.

**When to use:**
- Process restart loops (the primary use case here)
- Reconnection attempts to external services
- Any scenario where repeated rapid retries would worsen the failure

---

### 4.6 FlightRecorder Ring Buffer

**What:** A fixed-capacity ring buffer that captures IPC messages in binary
form and flushes them to a `.flog` file for post-flight replay and debugging.

**Why:** Flight-critical systems must record in-flight data without unbounded
memory growth.  A ring buffer caps memory usage at a configurable limit
(default 64 MB) and discards the oldest records when full — the most recent
data is always preserved.

**Where:** `common/recorder/include/recorder/flight_recorder.h`

**Key patterns used:**

- **RAII file management** — `std::ofstream` with binary mode, automatic close on scope exit.
- **Binary serialization** — `RecordHeader` (containing `WireHeader` + topic name length) written as raw bytes via `reinterpret_cast<const char*>`, guarded by `static_assert(std::is_trivially_copyable_v<T>)`.
- **Ring buffer** — `std::deque<RecordEntry>` for O(1) front eviction; byte-budget tracking ensures the buffer never exceeds `max_bytes`.
- **Lock-free recording flag** — `std::atomic<bool>` with `acquire`/`release` ordering for `start()`/`stop()`, so the hot-path `record()` check is lock-free.
- **Monotonic timestamps** — timestamp captured under the mutex to guarantee ordering; enforced with `std::max(last + 1, now)`.

```cpp
// Ring buffer evicts oldest entries to stay within byte budget
void push(RecordEntry entry) {
    const auto entry_size = entry.serialized_size();
    while (current_bytes_ + entry_size > max_bytes_ && !entries_.empty()) {
        current_bytes_ -= entries_.front().serialized_size();
        entries_.pop_front();   // O(1) deque pop
    }
    current_bytes_ += entry_size;
    entries_.push_back(std::move(entry));
}
```

**Flush strategy:** `flush()` moves all entries out of the deque under the
mutex (O(1) swap), then releases the lock and writes to disk — so `record()`
is not blocked during file I/O.

---

### 4.7 Rate Clamping

**What:** Clamp a config-driven loop rate to a safe range and log a warning
if the raw value was out of bounds.

**Why:** A misconfigured JSON rate (e.g. IMU at 10,000 Hz or VIO at 1 Hz)
can cause busy-loops (wasting CPU) or undersampling (missing data).  Clamping
at startup with a logged warning catches config errors early without crashing.

**Where:** `common/util/include/util/rate_clamp.h`

```cpp
inline int clamp_rate(int raw_hz, int min_hz, int max_hz, std::string_view label) {
    int clamped = std::clamp(raw_hz, min_hz, max_hz);
    if (clamped != raw_hz) {
        spdlog::warn("{} rate {} Hz out of range [{}, {}] — clamped to {} Hz",
                     label, raw_hz, min_hz, max_hz, clamped);
    }
    return clamped;
}
```

**Built-in bounds:**
- IMU rate: [100, 1000] Hz (via `clamp_imu_rate()`)
- VIO rate: [10, 1000] Hz (via `clamp_vio_rate()`)

The clamped rate is then used to compute the `sleep_until` tick period:
`auto period = std::chrono::microseconds(1'000'000 / clamped_hz)`.

---

### 4.8 Covariance Health Pattern

**What:** Using `trace(P_position)` — the sum of diagonal elements of the
3x3 position covariance block from the IMU pre-integrator — as a continuous
quality signal, thresholded into discrete health states.

**Why:** Feature count and stereo match quality are noisy proxies for VIO
health.  Covariance trace directly measures the estimator's own uncertainty
about position — it is the most principled single-number quality metric
available from the pre-integration output.

**Where:** `process3_slam_vio_nav/include/slam/ivio_backend.h`

```cpp
// Extract position covariance trace from IMU pre-integrator output
output.position_trace = pm.covariance.block<3, 3>(0, 0).trace();

// Threshold into discrete health states
if (output.position_trace <= good_trace_max_) {          // default 0.1
    new_health = VIOHealth::NOMINAL;
} else if (output.position_trace <= degraded_trace_max_) { // default 1.0
    new_health = VIOHealth::DEGRADED;
} else {
    new_health = VIOHealth::LOST;
}
```

**Threshold defaults:**
- `good_trace_max = 0.1` — trace(P_pos) <= 0.1 means NOMINAL
- `degraded_trace_max = 1.0` — trace(P_pos) <= 1.0 means DEGRADED, above means LOST

**Fallback:** When no IMU data is available (`position_trace < 0`), health
falls back to feature-count heuristics (feature count and stereo match ratio).

**Downstream effect:** `VIOHealth` maps to `Pose.quality` (0-3), which
`FaultManager` uses to trigger `FAULT_VIO_DEGRADED` (LOITER) or
`FAULT_VIO_LOST` (RTL) fault responses.

---

## 5. Safety-Critical C++ Standards

This is a **safety-critical autonomous drone stack**. C++ correctness, ownership
clarity, and memory safety are not optional — they are first-class requirements
alongside functionality.

### 5.1 Mandatory Constructs

| Requirement | Rule |
|-------------|------|
| **Ownership** | Use `std::unique_ptr` for sole ownership. `std::shared_ptr` only when shared lifetime is genuinely required and justified. **Raw owning pointers (`T*` owning heap memory) are forbidden.** |
| **Memory copies** | **`memcpy` / `memmove` / `memset` are forbidden** on safety- or functionally-relevant data. Use value semantics, structured bindings, or Zenoh's loan-based zero-copy API. |
| **Interfaces** | Pass by `const T&` or `T&&`. Never pass raw pointers across API boundaries unless interfacing with a C library — and wrap immediately in RAII. |
| **RAII** | All resources (file descriptors, threads, locks, timers) must be managed by RAII wrappers. No manual `new` / `delete`. |
| **Error handling** | Use `Result<T,E>` (see §3.13). No exceptions. No silent swallowing of errors. All `[[nodiscard]]` results must be handled (see §3.14). |
| **Concurrency** | Use `std::atomic` with explicit memory order (`acquire` / `release`). No ad-hoc `volatile` for synchronisation. Prefer lock-free ring buffers over mutexes for hot paths. |
| **Integer arithmetic** | Use fixed-width types (`uint32_t`, `int16_t`, etc.) for protocol/wire data. Avoid implicit narrowing conversions. |
| **Initialisation** | Always initialise variables at declaration. Eigen types must use `= Eigen::VectorXf::Zero()` / `= Eigen::MatrixXf::Zero()`. No uninitialised reads. |

### 5.2 Forbidden Patterns

The following are **forbidden** unless the code is explicitly non-safety and the
rationale is documented in a comment:

- `memcpy` / `memset` / `memmove` on safety-relevant structs
- Raw owning pointers (`T* p = new T(...)`)
- `std::shared_ptr` without a clear, justified shared-lifetime reason
- `reinterpret_cast` on safety-relevant data
- `static_cast` truncating values without a bounds check
- Unbounded recursion
- Global mutable state outside of explicitly documented singletons (e.g. `ZenohSession`)

### 5.3 Code Review Gate

When writing or reviewing any C++ code change, answer these four questions:

1. **Is ownership explicit and leak-free?** Every heap allocation has exactly one responsible owner; destructors are deterministic.
2. **Is there any raw memory manipulation that could be replaced with value semantics?** A `drone::ipc::Pose pose = other_pose;` copy is safer than `memcpy(&pose, &other_pose, sizeof(Pose))`.
3. **Are all error paths handled?** No silent fallback, no unchecked `Result`, no `[[nodiscard]]` return ignored.
4. **Would this code behave correctly under a use-after-free or data-race scenario?** If the answer requires thought, add a comment or restructure.

> **Design rationale:** These rules exist because silent memory corruption in a flight-control thread produces undefined behaviour with potentially catastrophic outcome. The safety overhead is near-zero — `unique_ptr` has the same runtime cost as a raw pointer; `Result<T,E>` is a small stack-allocated value type (backed by `std::variant<T,E>` — size depends on `T`/`E` but always stack-resident with no separate heap allocation). The benefit is a compiler-enforced ownership graph and exhaustive error-path coverage.

---

## 6. Concept Map — Where Patterns Intersect

The real power of these patterns is how they compose.  Here's how they
connect in a single publish operation:

```
                     ┌── Factory Pattern ──┐
config.json          │                     │
  "ipc_backend":     │  create_message_bus │
  "zenoh"  ─────────►│  returns variant    │
                     └─────────┬───────────┘
                               │ MessageBusVariant
                               ▼
                     ┌── std::variant ─────┐
                     │  std::visit(lambda)  │ ◄── std::visit + generic lambda
                     └─────────┬───────────┘
                               │ dispatches to ZenohMessageBus
                               ▼
                     ┌── Strategy Pattern ──┐
                     │  IPublisher<T>       │
                     │  → ZenohPublisher<T> │
                     └─────────┬───────────┘
                               │ publish(msg)
                               ▼
                     ┌── if constexpr ──────┐
                     │  sizeof(T) > 64KB?   │ ◄── compile-time branching
                     │  yes → publish_shm() │
                     │  no  → publish_bytes()│
                     └─────────┬───────────┘
                               │ SHM path
                               ▼
                     ┌── Singleton ─────────┐
                     │  ZenohSession::      │
                     │  instance()          │
                     │  .shm_provider()     │
                     └─────────┬───────────┘
                               │ alloc + memcpy
                               ▼
                     ┌── RAII ──────────────┐
                     │  ZShmMut buffer      │ ◄── auto-freed on scope exit
                     │  publisher_.put()    │
                     └──────────────────────┘
```

---

## 7. Glossary

| Term | Definition |
|------|-----------|
| **ABI** | Application Binary Interface — the binary-level contract between compiled code. `trivially_copyable` types have a stable ABI. |
| **Acquire/Release** | Memory ordering semantics. A `release` store makes all prior writes visible to a thread that does an `acquire` load of the same variable. |
| **Backoff** | A retry strategy where delays grow (typically doubling) between attempts. Prevents restart/reconnect storms. |
| **Cache line** | The smallest unit of data transfer between CPU and cache, typically 64 bytes on modern processors. |
| **CAS** | Compare-And-Swap — an atomic CPU instruction (`compare_exchange_weak/strong`) that sets a value only if it matches an expected value. Foundation of lock-free algorithms. |
| **Cascade** | Transitive restart propagation: when process A crashes, all processes that depend on A's state are also stopped and restarted. |
| **False sharing** | Performance degradation when two threads write to independent variables that happen to share a cache line. |
| **FIFO scheduling** | `SCHED_FIFO` — a real-time scheduling policy where the highest-priority runnable thread always executes. |
| **fork+exec** | The Unix pattern for spawning a child process: `fork()` clones the parent, `execvp()` replaces the clone with a new program. |
| **Heartbeat** | A periodic signal from a thread or process indicating liveness. Absence beyond a threshold implies the entity is stuck or dead. |
| **IPC** | Inter-Process Communication — mechanisms for separate processes to exchange data. |
| **Lock-free** | A concurrency guarantee: at least one thread makes progress, even if others are suspended. SeqLock and SPSC ring are lock-free. |
| **mmap** | Memory-map a file or shared memory object into a process's virtual address space. |
| **RAII** | Resource Acquisition Is Initialization — tying resource lifetime to object lifetime via constructor/destructor. |
| **Reap** | Calling `waitpid()` to collect a terminated child process's exit status, preventing zombie processes. |
| **sd_notify** | A systemd API for Type=notify services to signal readiness, liveness (watchdog), and status to the init system. |
| **SeqLock** | A reader-writer synchronization mechanism using an atomic sequence counter. Writers never block; readers retry on torn reads. |
| **SHM** | Shared Memory — a region of memory visible to multiple processes. |
| **SITL** | Software-In-The-Loop — running the real flight controller software in a simulator. |
| **SPSC** | Single-Producer Single-Consumer — a queue with exactly one writer and one reader thread. |
| **Trivially copyable** | A C++ type that can be safely copied with `memcpy`. No virtual functions, no non-trivial constructors/destructors, no heap pointers. |
| **Topological sort** | Ordering nodes of a DAG so every edge points forward. Used by `ProcessGraph::launch_order()` (Kahn's algorithm) to determine safe process startup sequence. |
| **Variant** | `std::variant` — a type-safe union that holds one of several specified types. |
| **Wait-free** | A stronger guarantee than lock-free: every thread completes in bounded steps. |
| **Wire format** | The binary encoding used to send messages over a network. |
| **Zero-copy** | Transferring data without intermediate buffer copies — e.g., SHM publisher writes directly to shared memory that the subscriber reads. |
