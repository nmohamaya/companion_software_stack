// common/util/include/util/scoped_timer.h
// RAII timer for performance monitoring — logs if exceeds threshold.
#pragma once
#include "util/ilogger.h"

#include <chrono>

class ScopedTimer {
public:
    ScopedTimer(const char* label, double warn_ms = 0.0)
        : label_(label), warn_ms_(warn_ms), start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        auto   end = std::chrono::steady_clock::now();
        double ms  = std::chrono::duration<double, std::milli>(end - start_).count();
        if (warn_ms_ > 0.0 && ms > warn_ms_) {
            DRONE_LOG_WARN("{}: {:.2f} ms (limit: {:.1f} ms)", label_, ms, warn_ms_);
        } else {
            DRONE_LOG_DEBUG("{}: {:.2f} ms", label_, ms);
        }
    }

    double elapsed_ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start_).count();
    }

private:
    const char*                           label_;
    double                                warn_ms_;
    std::chrono::steady_clock::time_point start_;
};
