// common/util/include/util/correlation.h
// Thread-local correlation context for cross-process tracing.
//
// A correlation ID is a uint64_t that flows through IPC messages so that
// a command originating at the GCS can be traced end-to-end across all
// 7 processes.
//
// Generation scheme: (pid << 32) | monotonic_counter
//   - Upper 32 bits = process ID → unique across processes
//   - Lower 32 bits = counter   → unique within a process
//
// Usage:
//   // At command origin (e.g. GCS command received):
//   auto cid = drone::util::CorrelationContext::generate();
//   shm_cmd.correlation_id = cid;
//
//   // When forwarding a received command:
//   ScopedCorrelation guard(incoming_msg.correlation_id);
//   outgoing_msg.correlation_id = CorrelationContext::get();
//
//   // In log messages:
//   DRONE_LOG_INFO("[cmd] corr={:#x} action=RTL", CorrelationContext::get());
#pragma once

#include <atomic>
#include <cstdint>

#include <unistd.h>

namespace drone::util {

/// Thread-local correlation context for end-to-end command tracing.
class CorrelationContext {
public:
    /// Generate a new globally-unique correlation ID.
    /// Scheme: (pid << 32) | monotonic_counter.
    [[nodiscard]] static uint64_t generate() {
        static std::atomic<uint32_t> counter{0};
        auto                         pid = static_cast<uint64_t>(static_cast<uint32_t>(getpid()));
        auto     seq = static_cast<uint64_t>(counter.fetch_add(1, std::memory_order_relaxed));
        uint64_t id  = (pid << 32) | (seq & 0xFFFFFFFF);
        set(id);
        return id;
    }

    /// Set the current thread's correlation ID (e.g. from an incoming message).
    static void set(uint64_t id) { current_id_ = id; }

    /// Get the current thread's correlation ID (0 = none).
    [[nodiscard]] static uint64_t get() { return current_id_; }

    /// Clear the current thread's correlation ID.
    static void clear() { current_id_ = 0; }

private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline thread_local uint64_t current_id_ = 0;
};

/// RAII guard that sets the correlation ID for the current scope
/// and restores the previous value on destruction.
class ScopedCorrelation {
public:
    /// Set @p id as the current correlation ID.
    explicit ScopedCorrelation(uint64_t id) : previous_(CorrelationContext::get()) {
        CorrelationContext::set(id);
    }

    ~ScopedCorrelation() { CorrelationContext::set(previous_); }

    // Non-copyable, non-movable.
    ScopedCorrelation(const ScopedCorrelation&)            = delete;
    ScopedCorrelation& operator=(const ScopedCorrelation&) = delete;
    ScopedCorrelation(ScopedCorrelation&&)                 = delete;
    ScopedCorrelation& operator=(ScopedCorrelation&&)      = delete;

private:
    uint64_t previous_;
};

}  // namespace drone::util
