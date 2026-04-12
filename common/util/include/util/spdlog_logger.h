// common/util/include/util/spdlog_logger.h
// Production ILogger implementation that delegates to spdlog.
//
// Separated from ilogger.h (Issue #385) so that the 60+ files including
// ilogger.h do not transitively depend on spdlog headers.
//
// Include this header only where SpdlogLogger is explicitly constructed:
//   - log_config.h (LogConfig::init installs SpdlogLogger via set_logger)
//   - process_context.h (includes log_config.h)
//   - Tests that directly exercise SpdlogLogger behaviour
//
// All other code should include only ilogger.h and use the DRONE_LOG_* macros.
#pragma once

#include "util/ilogger.h"

#include <spdlog/spdlog.h>

namespace drone::log {

// ── Level conversion ───────────────────────────────────────
/// Map drone::log::Level to spdlog::level::level_enum.
inline spdlog::level::level_enum to_spdlog_level(Level level) {
    switch (level) {
        case Level::Debug: return spdlog::level::debug;
        case Level::Info: return spdlog::level::info;
        case Level::Warn: return spdlog::level::warn;
        case Level::Error: return spdlog::level::err;
        case Level::Critical: return spdlog::level::critical;
    }
    return spdlog::level::info;  // unreachable, but silences -Wreturn-type
}

// ── SpdlogLogger ───────────────────────────────────────────
/// Production logger that delegates to spdlog's default logger.
/// Installed during init_process() via LogConfig::init().
class SpdlogLogger final : public ILogger {
public:
    void log(Level level, std::string_view msg) override {
        spdlog::log(to_spdlog_level(level), "{}", msg);
    }

    [[nodiscard]] bool should_log(Level level) const override {
        return spdlog::should_log(to_spdlog_level(level));
    }
};

}  // namespace drone::log
