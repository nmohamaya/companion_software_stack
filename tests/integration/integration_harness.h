// tests/integration/integration_harness.h
// In-process integration test harness for multi-process IPC testing.
//
// Creates a shared MessageBus with MockClock and CapturingLogger so that
// multiple "process" components can be wired together in a single test
// binary without Gazebo or real processes.
//
// Usage:
//   drone::test::IntegrationTestHarness harness;
//   auto pub = harness.bus().advertise<FCState>("/fc_state");
//   auto sub = harness.bus().subscribe<MissionStatus>("/mission_status");
//   harness.advance_time_ms(100);
//   // ... assert on received messages and captured logs ...
//
// See: Issue #292 (Epic #284 — Platform Modularity)
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/message_bus.h"
#include "ipc/message_bus_factory.h"
#include "util/capturing_logger.h"
#include "util/config.h"
#include "util/iclock.h"
#include "util/ilogger.h"
#include "util/mock_clock.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <unistd.h>

namespace drone::test {

/// In-process integration test harness.
///
/// Creates a shared MessageBus (Zenoh backend), installs a MockClock and
/// CapturingLogger, and provides helpers for deterministic time advancement
/// and log assertion.  RAII cleanup restores the previous clock and logger
/// on destruction.
///
/// Design decisions:
///   - Single shared bus: all "process" components publish/subscribe on the
///     same Zenoh session, just like the real 7-process stack but in-process.
///   - MockClock: enables deterministic timeout/escalation testing without
///     real wall-clock waits.
///   - CapturingLogger: enables assertion on log output (e.g. "fault escalated").
///   - Config loaded from default.json with test overrides via config().
class IntegrationTestHarness {
public:
    IntegrationTestHarness()
        : scoped_clock_()
        , capturing_logger_(new drone::log::CapturingLogger())
        , capturing_logger_ptr_(capturing_logger_)
        , bus_(drone::ipc::create_message_bus("zenoh")) {
        // Install capturing logger (takes ownership).
        drone::log::set_logger(std::unique_ptr<drone::log::CapturingLogger>(capturing_logger_));
        capturing_logger_ = nullptr;  // ownership transferred, use capturing_logger_ptr_

        // Load default config (best-effort — tests may override).
#ifdef PROJECT_CONFIG_DIR
        cfg_.load(std::string(PROJECT_CONFIG_DIR) + "/default.json");
#endif
    }

    ~IntegrationTestHarness() {
        // Restore default logger (ScopedMockClock restores clock automatically).
        drone::log::reset_logger();
    }

    // Non-copyable, non-movable.
    IntegrationTestHarness(const IntegrationTestHarness&)            = delete;
    IntegrationTestHarness& operator=(const IntegrationTestHarness&) = delete;
    IntegrationTestHarness(IntegrationTestHarness&&)                 = delete;
    IntegrationTestHarness& operator=(IntegrationTestHarness&&)      = delete;

    /// Get the shared message bus (all "processes" publish/subscribe here).
    [[nodiscard]] drone::ipc::MessageBus& bus() { return bus_; }

    /// Get the mock clock for deterministic time control.
    [[nodiscard]] drone::util::MockClock& clock() { return scoped_clock_.mock(); }

    /// Get the capturing logger for log assertions.
    /// Returns raw pointer — harness retains ownership via the global logger.
    [[nodiscard]] drone::log::CapturingLogger* logger() { return capturing_logger_ptr_; }

    /// Get the config (loaded from default.json, modifiable for test overrides).
    [[nodiscard]] drone::Config&       config() { return cfg_; }
    [[nodiscard]] const drone::Config& config() const { return cfg_; }

    /// Load a specific config file (replaces current config).
    [[nodiscard]] bool load_config(const std::string& path) { return cfg_.load(path); }

    /// Advance the mock clock by the given number of milliseconds.
    void advance_time_ms(uint32_t ms) { scoped_clock_.mock().advance_ms(ms); }

    /// Advance the mock clock by the given number of seconds.
    void advance_time_s(uint32_t s) { scoped_clock_.mock().advance_s(s); }

    /// Reset state between tests (clear logs, reset clock).
    void reset() {
        if (capturing_logger_ptr_) {
            capturing_logger_ptr_->clear();
        }
        scoped_clock_.mock().reset();
    }

    /// Generate a unique topic name to avoid cross-test interference.
    /// Appends PID and a counter to the base topic.
    [[nodiscard]] static std::string unique_topic(const char* base) {
        static std::atomic<uint32_t> counter{0};
        return std::string(base) + "_integ_" + std::to_string(::getpid()) + "_" +
               std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
    }

private:
    drone::util::ScopedMockClock scoped_clock_;
    drone::log::CapturingLogger* capturing_logger_     = nullptr;  // pre-transfer pointer
    drone::log::CapturingLogger* capturing_logger_ptr_ = nullptr;  // valid after transfer
    drone::ipc::MessageBus       bus_;
    drone::Config                cfg_;
};

}  // namespace drone::test
