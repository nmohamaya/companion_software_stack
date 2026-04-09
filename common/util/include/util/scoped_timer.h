// common/util/include/util/scoped_timer.h
// RAII timer for performance monitoring — logs if exceeds threshold.
#pragma once
#include "util/iclock.h"
#include "util/ilogger.h"

#include <chrono>

class ScopedTimer {
public:
    ScopedTimer(const char* label, double warn_ms = 0.0)
        : label_(label), warn_ms_(warn_ms), start_ns_(drone::util::get_clock().now_ns()) {}

    ~ScopedTimer() {
        const double ms = static_cast<double>(drone::util::get_clock().now_ns() - start_ns_) /
                          1'000'000.0;
        if (warn_ms_ > 0.0 && ms > warn_ms_) {
            DRONE_LOG_WARN("{}: {:.2f} ms (limit: {:.1f} ms)", label_, ms, warn_ms_);
        } else {
            DRONE_LOG_DEBUG("{}: {:.2f} ms", label_, ms);
        }
    }

    double elapsed_ms() const {
        return static_cast<double>(drone::util::get_clock().now_ns() - start_ns_) / 1'000'000.0;
    }

private:
    const char* label_;
    double      warn_ms_;
    uint64_t    start_ns_;
};
