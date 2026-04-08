// common/util/include/util/ilogger.h
// Abstract logging interface — decouples all code from spdlog.
//
// Provides:
//   - ILogger interface with debug/info/warn/error/critical levels
//   - Global accessor: drone::log::logger() / set_logger()
//   - DRONE_LOG_DEBUG/INFO/WARN/ERROR/CRITICAL macros (fmt-style)
//
// Default implementation: SpdlogLogger (delegates to spdlog default logger).
// Test implementations: NullLogger, CapturingLogger (separate headers).
//
// Usage:
//   DRONE_LOG_INFO("Camera {} started @ {}Hz", cam.name(), fps);
//   DRONE_LOG_WARN("Frame drop #{}", count);
//   DRONE_LOG_ERROR("Failed to open device: {}", reason);
//
// Swapping logger (e.g. in tests):
//   #include "util/capturing_logger.h"
//   auto capturing = std::make_unique<drone::log::CapturingLogger>();
//   auto* cap = capturing.get();
//   drone::log::set_logger(std::move(capturing));
//   // ... run code under test ...
//   EXPECT_EQ(cap->count(), 2);
//   drone::log::reset_logger();  // restore default
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

namespace drone::log {

// ── Log severity levels ────────────────────────────────────
enum class Level : uint8_t {
    Debug    = 0,
    Info     = 1,
    Warn     = 2,
    Error    = 3,
    Critical = 4,
};

// ── ILogger interface ──────────────────────────────────────
/// Abstract logging interface.  All production and utility code
/// logs through this interface via the DRONE_LOG_* macros.
class ILogger {
public:
    virtual ~ILogger() = default;

    /// Emit a log message at the given severity level.
    virtual void log(Level level, const std::string& msg) = 0;

    /// Check if a message at this level would be emitted.
    /// Used by macros to skip fmt::format when the level is disabled.
    [[nodiscard]] virtual bool should_log(Level level) const = 0;

    // Non-copyable, non-movable (held via unique_ptr).
    ILogger()                          = default;
    ILogger(const ILogger&)            = delete;
    ILogger& operator=(const ILogger&) = delete;
    ILogger(ILogger&&)                 = delete;
    ILogger& operator=(ILogger&&)      = delete;
};

// ── SpdlogLogger (default) ─────────────────────────────────
/// Production logger that delegates to spdlog's default logger.
/// This is the default when no custom logger is installed.
class SpdlogLogger final : public ILogger {
public:
    void log(Level level, const std::string& msg) override {
        spdlog::log(to_spdlog_level(level), "{}", msg);
    }

    [[nodiscard]] bool should_log(Level level) const override {
        return spdlog::should_log(to_spdlog_level(level));
    }

private:
    static spdlog::level::level_enum to_spdlog_level(Level level) {
        switch (level) {
            case Level::Debug: return spdlog::level::debug;
            case Level::Info: return spdlog::level::info;
            case Level::Warn: return spdlog::level::warn;
            case Level::Error: return spdlog::level::err;
            case Level::Critical: return spdlog::level::critical;
        }
        return spdlog::level::info;  // unreachable, but silences -Wreturn-type
    }
};

// ── Global logger accessor ─────────────────────────────────

namespace detail {

/// Owning pointer to the user-installed logger (null = use default).
inline std::unique_ptr<ILogger>& logger_owner() {
    static std::unique_ptr<ILogger> owner;
    return owner;
}

/// Raw pointer cache (null = use default SpdlogLogger).
inline ILogger*& logger_ptr() {
    static ILogger* ptr = nullptr;
    return ptr;
}

/// Process-wide default SpdlogLogger instance.
inline ILogger& default_logger() {
    static SpdlogLogger instance;
    return instance;
}

}  // namespace detail

/// Get the active logger.  Never returns null.
/// Hot-path cost: one function call + one pointer dereference.
inline ILogger& logger() {
    auto* p = detail::logger_ptr();
    if (p) return *p;
    return detail::default_logger();
}

/// Install a custom logger (e.g. CapturingLogger for tests).
/// Pass nullptr or call reset_logger() to revert to default.
inline void set_logger(std::unique_ptr<ILogger> l) {
    detail::logger_owner() = std::move(l);
    detail::logger_ptr()   = detail::logger_owner().get();
}

/// Revert to the default SpdlogLogger.
inline void reset_logger() {
    detail::logger_owner().reset();
    detail::logger_ptr() = nullptr;
}

}  // namespace drone::log

// ── DRONE_LOG macros ───────────────────────────────────────
// fmt-style formatting, with level check to skip formatting
// when the level is disabled (zero overhead on disabled levels).

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG_DEBUG(...)                                                      \
    do {                                                                          \
        auto& _drone_lg_ = ::drone::log::logger();                                \
        if (_drone_lg_.should_log(::drone::log::Level::Debug))                    \
            _drone_lg_.log(::drone::log::Level::Debug, fmt::format(__VA_ARGS__)); \
    } while (0)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG_INFO(...)                                                      \
    do {                                                                         \
        auto& _drone_lg_ = ::drone::log::logger();                               \
        if (_drone_lg_.should_log(::drone::log::Level::Info))                    \
            _drone_lg_.log(::drone::log::Level::Info, fmt::format(__VA_ARGS__)); \
    } while (0)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG_WARN(...)                                                      \
    do {                                                                         \
        auto& _drone_lg_ = ::drone::log::logger();                               \
        if (_drone_lg_.should_log(::drone::log::Level::Warn))                    \
            _drone_lg_.log(::drone::log::Level::Warn, fmt::format(__VA_ARGS__)); \
    } while (0)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG_ERROR(...)                                                      \
    do {                                                                          \
        auto& _drone_lg_ = ::drone::log::logger();                                \
        if (_drone_lg_.should_log(::drone::log::Level::Error))                    \
            _drone_lg_.log(::drone::log::Level::Error, fmt::format(__VA_ARGS__)); \
    } while (0)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG_CRITICAL(...)                                                      \
    do {                                                                             \
        auto& _drone_lg_ = ::drone::log::logger();                                   \
        if (_drone_lg_.should_log(::drone::log::Level::Critical))                    \
            _drone_lg_.log(::drone::log::Level::Critical, fmt::format(__VA_ARGS__)); \
    } while (0)

// ── Generic level macro (for dynamic level selection) ──────
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DRONE_LOG(level, ...)                                                              \
    do {                                                                                   \
        auto& _drone_lg_ = ::drone::log::logger();                                         \
        if (_drone_lg_.should_log(level)) _drone_lg_.log(level, fmt::format(__VA_ARGS__)); \
    } while (0)
