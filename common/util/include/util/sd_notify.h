// common/util/include/util/sd_notify.h
// Thin wrapper around sd_notify() for systemd watchdog integration.
//
// When built with -DENABLE_SYSTEMD=ON (defines HAVE_SYSTEMD), this
// calls the real sd_notify().  Otherwise, all functions are no-ops.
//
// Usage:
//   #include "util/sd_notify.h"
//
//   // At startup, after initialization is complete:
//   drone::systemd::notify_ready();
//
//   // In the health loop (every tick):
//   drone::systemd::notify_watchdog();
//
//   // Before shutdown:
//   drone::systemd::notify_stopping();
//
//   // Check if systemd watchdog is active:
//   if (drone::systemd::watchdog_enabled()) { ... }
//
//   // Get the watchdog interval (0 if not active):
//   auto usec = drone::systemd::watchdog_usec();
#pragma once

#include <cstdint>

#ifdef HAVE_SYSTEMD
#include "util/ilogger.h"

#include <string>

#include <systemd/sd-daemon.h>
#endif

namespace drone::systemd {

/// Notify systemd that the service is ready (Type=notify).
/// Call once after initialization is complete.
inline void notify_ready() {
#ifdef HAVE_SYSTEMD
    sd_notify(0, "READY=1");
    DRONE_LOG_INFO("[systemd] Notified READY=1");
#endif
}

/// Pet the systemd watchdog timer (WatchdogSec).
/// Call periodically from the health loop — at least every WatchdogSec/2.
inline void notify_watchdog() {
#ifdef HAVE_SYSTEMD
    sd_notify(0, "WATCHDOG=1");
#endif
}

/// Notify systemd that the service is stopping gracefully.
/// Call at the beginning of shutdown.
inline void notify_stopping() {
#ifdef HAVE_SYSTEMD
    sd_notify(0, "STOPPING=1");
    DRONE_LOG_INFO("[systemd] Notified STOPPING=1");
#endif
}

/// Publish a human-readable status string to systemd.
/// Visible in `systemctl status <unit>`.
inline void notify_status(const char* status) {
#ifdef HAVE_SYSTEMD
    std::string msg = "STATUS=";
    msg += status;
    sd_notify(0, msg.c_str());
#else
    (void)status;
#endif
}

/// Check if systemd watchdog is enabled for this process.
/// Returns true only when running under systemd with WatchdogSec > 0.
inline bool watchdog_enabled() {
#ifdef HAVE_SYSTEMD
    uint64_t usec = 0;
    int      r    = sd_watchdog_enabled(0, &usec);
    return r > 0 && usec > 0;
#else
    return false;
#endif
}

/// Get the watchdog interval in microseconds.
/// Returns 0 if watchdog is not active.
inline uint64_t watchdog_usec() {
#ifdef HAVE_SYSTEMD
    uint64_t usec = 0;
    int      r    = sd_watchdog_enabled(0, &usec);
    return (r > 0) ? usec : 0;
#else
    return 0;
#endif
}

}  // namespace drone::systemd
