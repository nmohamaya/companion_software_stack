// common/util/include/util/null_logger.h
// No-op ILogger implementation — silently discards all messages.
//
// Use cases:
//   - Unit tests where logging output is irrelevant
//   - Benchmarks measuring pure computation without logging overhead
//   - Embedded builds with no logging support
//
// Usage:
//   drone::log::set_logger(std::make_unique<drone::log::NullLogger>());
#pragma once

#include "util/ilogger.h"

namespace drone::log {

/// Logger that discards all messages.  Zero allocation, zero I/O.
class NullLogger final : public ILogger {
public:
    void log(Level /*level*/, std::string_view /*msg*/) override {
        // Intentionally empty.
    }

    [[nodiscard]] bool should_log(Level /*level*/) const override {
        return false;  // Skip formatting overhead entirely.
    }
};

}  // namespace drone::log
