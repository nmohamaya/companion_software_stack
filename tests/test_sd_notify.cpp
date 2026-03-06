// tests/test_sd_notify.cpp
// Tests for the sd_notify wrapper (util/sd_notify.h).
//
// When built WITHOUT -DENABLE_SYSTEMD=ON:
//   - All functions are no-ops
//   - watchdog_enabled() returns false
//   - watchdog_usec() returns 0
//
// When built WITH -DENABLE_SYSTEMD=ON:
//   - Functions call real sd_notify() — but outside a systemd unit
//     context, sd_notify() silently succeeds (no socket → ignored).
//   - watchdog_enabled() returns false (no WATCHDOG_USEC env var).

#include "util/sd_notify.h"

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// 1. Basic API — must not crash regardless of build mode
// ═══════════════════════════════════════════════════════════

TEST(SdNotifyWrapper, ReadyDoesNotCrash) {
    // Should be a no-op (no systemd socket outside a unit)
    EXPECT_NO_THROW(drone::systemd::notify_ready());
}

TEST(SdNotifyWrapper, WatchdogDoesNotCrash) {
    EXPECT_NO_THROW(drone::systemd::notify_watchdog());
}

TEST(SdNotifyWrapper, StoppingDoesNotCrash) {
    EXPECT_NO_THROW(drone::systemd::notify_stopping());
}

TEST(SdNotifyWrapper, StatusDoesNotCrash) {
    EXPECT_NO_THROW(drone::systemd::notify_status("CPU=42% MEM=60%"));
}

TEST(SdNotifyWrapper, StatusWithEmptyString) {
    EXPECT_NO_THROW(drone::systemd::notify_status(""));
}

// ═══════════════════════════════════════════════════════════
// 2. Watchdog query — outside systemd, always false/0
// ═══════════════════════════════════════════════════════════

TEST(SdNotifyWrapper, WatchdogNotEnabledOutsideSystemd) {
    // Without WATCHDOG_USEC set, watchdog should not be enabled
    ::unsetenv("WATCHDOG_USEC");
    EXPECT_FALSE(drone::systemd::watchdog_enabled());
}

TEST(SdNotifyWrapper, WatchdogUsecZeroOutsideSystemd) {
    ::unsetenv("WATCHDOG_USEC");
    EXPECT_EQ(drone::systemd::watchdog_usec(), 0u);
}

// ═══════════════════════════════════════════════════════════
// 3. Repeated calls — idempotency
// ═══════════════════════════════════════════════════════════

TEST(SdNotifyWrapper, MultipleWatchdogCallsAreSafe) {
    // Simulates a health loop calling watchdog every tick
    for (int i = 0; i < 100; ++i) {
        drone::systemd::notify_watchdog();
    }
}

TEST(SdNotifyWrapper, FullLifecycleSequence) {
    // Simulates the complete P7 lifecycle
    drone::systemd::notify_ready();
    drone::systemd::notify_status("Initializing...");

    for (int i = 0; i < 10; ++i) {
        drone::systemd::notify_watchdog();
    }

    drone::systemd::notify_status("Healthy: CPU=12% MEM=45%");
    drone::systemd::notify_stopping();
}

#ifdef HAVE_SYSTEMD
// ═══════════════════════════════════════════════════════════
// 4. systemd-specific: verify env var detection
// ═══════════════════════════════════════════════════════════

TEST(SdNotifyWrapper, WatchdogDetectsEnvVar) {
    // Set WATCHDOG_USEC like systemd would
    ::setenv("WATCHDOG_USEC", "10000000", 1);  // 10 seconds in microseconds
    ::setenv("WATCHDOG_PID", std::to_string(::getpid()).c_str(), 1);

    // sd_watchdog_enabled checks both WATCHDOG_USEC and that WATCHDOG_PID
    // matches our PID (or is unset)
    EXPECT_TRUE(drone::systemd::watchdog_enabled());
    EXPECT_EQ(drone::systemd::watchdog_usec(), 10'000'000u);

    // Clean up
    ::unsetenv("WATCHDOG_USEC");
    ::unsetenv("WATCHDOG_PID");
}

TEST(SdNotifyWrapper, WatchdogIgnoresWrongPid) {
    // If WATCHDOG_PID doesn't match our PID, watchdog should be disabled
    ::setenv("WATCHDOG_USEC", "5000000", 1);
    ::setenv("WATCHDOG_PID", "99999999", 1);  // wrong PID

    EXPECT_FALSE(drone::systemd::watchdog_enabled());
    EXPECT_EQ(drone::systemd::watchdog_usec(), 0u);

    ::unsetenv("WATCHDOG_USEC");
    ::unsetenv("WATCHDOG_PID");
}
#endif  // HAVE_SYSTEMD
