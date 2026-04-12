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

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

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
    virtual void log(Level level, std::string_view msg) = 0;

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
    void log(Level level, std::string_view msg) override {
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
// Thread safety: logger() is hot-path (called on every DRONE_LOG_*
// invocation), so it uses an atomic load (acquire).  set_logger() and
// reset_logger() are cold-path (startup / tests) and hold a mutex to
// protect the owning unique_ptr.
//
// RCU-style retirement (Issue #384): When set_logger() or reset_logger()
// replaces the active logger, the OLD logger is moved into a retirement
// vector rather than destroyed.  This eliminates use-after-free: any
// thread that loaded the old pointer via logger() can safely finish its
// log call even if another thread concurrently swaps the logger.  The
// cost is a few leaked logger objects (2-3 per process lifetime max),
// which is negligible for a process that runs for hours/days.

namespace detail {

/// Mutex protecting the owning unique_ptr (cold-path only).
inline std::mutex& logger_mutex() {
    static std::mutex mtx;
    return mtx;
}

/// Owning pointer to the user-installed logger (null = use default).
/// Must hold logger_mutex() to read or write.
inline std::unique_ptr<ILogger>& logger_owner() {
    static std::unique_ptr<ILogger> owner;
    return owner;
}

/// Retired loggers — kept alive to prevent use-after-free.
/// Only accessed under logger_mutex().
inline std::vector<std::unique_ptr<ILogger>>& retired_loggers() {
    static std::vector<std::unique_ptr<ILogger>> vec;
    return vec;
}

/// Atomic raw pointer cache for hot-path reads (null = use default).
inline std::atomic<ILogger*>& logger_ptr() {
    static std::atomic<ILogger*> ptr{nullptr};
    return ptr;
}

/// Process-wide default SpdlogLogger instance.
inline ILogger& default_logger() {
    static SpdlogLogger instance;
    return instance;
}

}  // namespace detail

/// Get the active logger.  Never returns null.
/// Hot-path cost: one atomic load (acquire) + one branch.
inline ILogger& logger() {
    auto* p = detail::logger_ptr().load(std::memory_order_acquire);
    if (p) return *p;
    return detail::default_logger();
}

/// Install a custom logger (e.g. CapturingLogger for tests).
/// Pass nullptr or call reset_logger() to revert to default.
/// Thread-safe: serialized by mutex, atomic store visible to all readers.
/// The previous logger is retired (kept alive) to prevent use-after-free
/// if another thread is still using it.
inline void set_logger(std::unique_ptr<ILogger> l) {
    std::lock_guard<std::mutex> lock(detail::logger_mutex());
    // Retire the old logger — do NOT destroy it.
    if (detail::logger_owner()) {
        detail::retired_loggers().push_back(std::move(detail::logger_owner()));
    }
    detail::logger_owner() = std::move(l);
    detail::logger_ptr().store(detail::logger_owner().get(), std::memory_order_release);
}

/// Revert to the default SpdlogLogger.
/// The previous logger is retired (kept alive) to prevent use-after-free.
inline void reset_logger() {
    std::lock_guard<std::mutex> lock(detail::logger_mutex());
    // Retire the old logger — do NOT destroy it.
    if (detail::logger_owner()) {
        detail::retired_loggers().push_back(std::move(detail::logger_owner()));
    }
    detail::logger_ptr().store(nullptr, std::memory_order_release);
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
#define DRONE_LOG(level, ...)                                      \
    do {                                                           \
        const auto _drone_lvl_ = (level);                          \
        auto&      _drone_lg_  = ::drone::log::logger();           \
        if (_drone_lg_.should_log(_drone_lvl_))                    \
            _drone_lg_.log(_drone_lvl_, fmt::format(__VA_ARGS__)); \
    } while (0)
