// common/util/include/util/rate_clamp.h
// Utility for clamping config-driven loop rates with warning logs.
// Prevents runaway loops (too-high rate) or undersampling (too-low rate)
// from misconfigured JSON.
#pragma once

#include "util/ilogger.h"

#include <algorithm>
#include <string_view>

namespace drone::util {

// ── IMU rate bounds ────────────────────────────────────────
constexpr int kImuRateMinHz = 100;
constexpr int kImuRateMaxHz = 1000;

// ── VIO rate bounds ────────────────────────────────────────
constexpr int kVioRateMinHz = 10;
constexpr int kVioRateMaxHz = 1000;

/// Clamp a rate value to [min_hz, max_hz] and log a warning if clamping was applied.
/// Returns the clamped value.
inline int clamp_rate(int raw_hz, int min_hz, int max_hz, std::string_view label) {
    int clamped = std::clamp(raw_hz, min_hz, max_hz);
    if (clamped != raw_hz) {
        DRONE_LOG_WARN("{} rate {} Hz out of range [{}, {}] — clamped to {} Hz", label, raw_hz,
                       min_hz, max_hz, clamped);
    }
    return clamped;
}

/// Convenience: clamp IMU rate to [100, 1000] Hz.
inline int clamp_imu_rate(int raw_hz) {
    return clamp_rate(raw_hz, kImuRateMinHz, kImuRateMaxHz, "IMU");
}

/// Convenience: clamp VIO rate to [10, 1000] Hz.
inline int clamp_vio_rate(int raw_hz) {
    return clamp_rate(raw_hz, kVioRateMinHz, kVioRateMaxHz, "VIO");
}

}  // namespace drone::util
