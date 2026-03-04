// common/util/include/util/thread_heartbeat.h
// Lock-free per-thread heartbeat monitoring — ADR-004 Layer 1.
//
// ThreadHeartbeatRegistry: process-global registry of thread heartbeats.
// ScopedHeartbeat: RAII convenience wrapper.
//
// Hot-path cost: one steady_clock read + one atomic_store(relaxed) per touch().
#pragma once

#include "util/safe_name_copy.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>

namespace drone::util {

/// Maximum number of threads that can register heartbeats per process.
/// Matches the system limit of 21 threads across 7 processes (max per
/// process is 6 in P2 Perception).  16 provides headroom.
static constexpr size_t kMaxThreads = 16;

/// Sentinel returned when the registry is full.
static constexpr size_t kInvalidHandle = static_cast<size_t>(-1);

/// Per-thread heartbeat entry used for tracking per-thread liveness.
/// Contains std::atomic + custom copy ctor, so NOT trivially copyable.
/// Snapshots should be taken via the explicit copy constructor/assignment.
struct ThreadHeartbeat {
    char                  name[32] = {};
    std::atomic<uint64_t> last_touch_ns{0};
    bool                  is_critical = false;

    // Non-atomic copy for snapshots
    ThreadHeartbeat() = default;
    ThreadHeartbeat(const ThreadHeartbeat& other) : is_critical(other.is_critical) {
        std::memcpy(name, other.name, sizeof(name));
        last_touch_ns.store(other.last_touch_ns.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
    }
    ThreadHeartbeat& operator=(const ThreadHeartbeat& other) {
        if (this != &other) {
            std::memcpy(name, other.name, sizeof(name));
            last_touch_ns.store(other.last_touch_ns.load(std::memory_order_relaxed),
                                std::memory_order_relaxed);
            is_critical = other.is_critical;
        }
        return *this;
    }
};

// ─── ThreadHeartbeatRegistry ────────────────────────────────────────

/// Process-global singleton that tracks per-thread heartbeat timestamps.
///
/// Writers (worker threads) call touch() on every loop iteration.
/// Readers (watchdog thread, SHM publisher) call snapshot().
///
/// Thread safety:
///   - register_thread() is safe to call from multiple threads concurrently
///     (uses atomic fetch_add on count_).
///   - touch() is safe to call from any thread (writes to own slot only).
///   - snapshot() is safe to call concurrently with touch() (reads atomics).
class ThreadHeartbeatRegistry {
public:
    /// Get the process-global singleton.
    static ThreadHeartbeatRegistry& instance() {
        static ThreadHeartbeatRegistry reg;
        return reg;
    }

    /// Register a new thread heartbeat.
    /// @param name   Human-readable thread name (truncated to 31 chars).
    /// @param critical  If true, a stuck thread triggers process self-termination.
    /// @return Handle for touch(), or kInvalidHandle if registry is full.
    size_t register_thread(const char* name, bool critical = false) {
        // CAS loop: atomically claim a slot only if below kMaxThreads.
        // This avoids the fetch_add/fetch_sub rollback race where concurrent
        // overflow registrations can corrupt count_.
        size_t idx = count_.load(std::memory_order_acquire);
        while (true) {
            if (idx >= kMaxThreads) {
                return kInvalidHandle;
            }
            if (count_.compare_exchange_weak(idx, idx + 1, std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
                break;
            }
            // idx updated with current value; retry.
        }
        // Initialize slot BEFORE it becomes visible to snapshot().
        // The CAS above publishes idx+1, but we write to beats_[idx]
        // which snapshot() only reads up to count_.  Since we just
        // incremented count_ in the CAS, a concurrent snapshot() may
        // already see this slot — so we must use the slot's own atomic
        // to signal readiness.  We set last_touch_ns = 0 first (already
        // default), then write name/critical.  snapshot() callers must
        // tolerate partially-initialized name during the brief window.
        safe_name_copy(beats_[idx].name, name);
        beats_[idx].is_critical = critical;
        // last_touch_ns stays 0 — signals "registered but not yet started"
        return idx;
    }

    /// Touch the heartbeat — called every loop iteration.
    /// Cost: one steady_clock::now() + one atomic_store(relaxed).
    void touch(size_t handle) {
        if (handle >= kMaxThreads) return;
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const auto ns  = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        beats_[handle].last_touch_ns.store(ns, std::memory_order_relaxed);
    }

    /// Touch with a grace period — bumps timestamp to now + grace.
    /// Use before a known long operation (model loading, vocabulary loading).
    /// After the operation completes, call touch() to resume normal cadence.
    void touch_with_grace(size_t handle, std::chrono::milliseconds grace) {
        if (handle >= kMaxThreads) return;
        const auto now    = std::chrono::steady_clock::now().time_since_epoch();
        const auto future = now + grace;
        const auto ns     = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(future).count());
        beats_[handle].last_touch_ns.store(ns, std::memory_order_relaxed);
    }

    /// Snapshot all registered heartbeats (deep copy of atomics).
    /// Thread-safe to call concurrently with touch().
    [[nodiscard]] std::vector<ThreadHeartbeat> snapshot() const {
        const size_t                 n = count_.load(std::memory_order_acquire);
        std::vector<ThreadHeartbeat> result;
        result.reserve(n);
        for (size_t i = 0; i < n && i < kMaxThreads; ++i) {
            result.push_back(beats_[i]);
        }
        return result;
    }

    /// Number of registered heartbeats.
    [[nodiscard]] size_t count() const { return count_.load(std::memory_order_acquire); }

    /// Reset all state — only for testing.  NOT thread-safe.
    void reset_for_testing() {
        for (size_t i = 0; i < kMaxThreads; ++i) {
            std::memset(beats_[i].name, 0, sizeof(beats_[i].name));
            beats_[i].last_touch_ns.store(0, std::memory_order_relaxed);
            beats_[i].is_critical = false;
        }
        count_.store(0, std::memory_order_relaxed);
    }

private:
    ThreadHeartbeatRegistry()                                          = default;
    ThreadHeartbeatRegistry(const ThreadHeartbeatRegistry&)            = delete;
    ThreadHeartbeatRegistry& operator=(const ThreadHeartbeatRegistry&) = delete;

    std::array<ThreadHeartbeat, kMaxThreads> beats_{};
    std::atomic<size_t>                      count_{0};
};

// ─── ScopedHeartbeat ────────────────────────────────────────────────

/// RAII convenience wrapper — registers on construction, touch() in loop.
///
/// Usage:
///   ScopedHeartbeat hb("my_thread", /*critical=*/true);
///   while (running) {
///       hb.touch();
///       // ... work ...
///   }
class ScopedHeartbeat {
public:
    /// Register this thread in the global heartbeat registry.
    /// @param name      Thread name (truncated to 31 chars).
    /// @param critical  If true, stuck → process self-terminates.
    explicit ScopedHeartbeat(const char* name, bool critical = false)
        : handle_(ThreadHeartbeatRegistry::instance().register_thread(name, critical)) {}

    /// Touch the heartbeat — call once per loop iteration.
    void touch() { ThreadHeartbeatRegistry::instance().touch(handle_); }

    /// Touch with grace period for known long operations.
    void touch_with_grace(std::chrono::milliseconds grace) {
        ThreadHeartbeatRegistry::instance().touch_with_grace(handle_, grace);
    }

    /// Check if registration succeeded.
    [[nodiscard]] bool is_valid() const { return handle_ != kInvalidHandle; }

    /// Get the underlying handle.
    [[nodiscard]] size_t handle() const { return handle_; }

private:
    size_t handle_;
};

}  // namespace drone::util
