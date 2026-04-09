// tests/test_ilogger.cpp
// Tests for the ILogger abstraction: interface, SpdlogLogger, NullLogger,
// CapturingLogger, DRONE_LOG macros, and global accessor.
//
// Issue #285 — ILogger interface + DRONE_LOG macros.

#include "util/capturing_logger.h"
#include "util/ilogger.h"
#include "util/null_logger.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

// ── RAII guard: restore default logger after each test ──────
class LoggerGuard {
public:
    ~LoggerGuard() { drone::log::reset_logger(); }
};

// ═══════════════════════════════════════════════════════════════
// SpdlogLogger (default)
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, DefaultLoggerIsSpdlog) {
    // The default logger should be a SpdlogLogger instance.
    auto& lg = drone::log::logger();
    // Verify it's usable (doesn't crash).
    lg.log(drone::log::Level::Info, "test message from default logger");
    EXPECT_TRUE(lg.should_log(drone::log::Level::Info));
}

TEST(ILoggerTest, SpdlogLoggerShouldLogRespectsLevel) {
    // Set spdlog level explicitly for deterministic test behavior.
    auto prev_level = spdlog::default_logger()->level();
    spdlog::set_level(spdlog::level::info);

    drone::log::SpdlogLogger logger;
    EXPECT_FALSE(logger.should_log(drone::log::Level::Debug));
    EXPECT_TRUE(logger.should_log(drone::log::Level::Info));
    EXPECT_TRUE(logger.should_log(drone::log::Level::Warn));
    EXPECT_TRUE(logger.should_log(drone::log::Level::Error));
    EXPECT_TRUE(logger.should_log(drone::log::Level::Critical));

    spdlog::set_level(prev_level);  // Restore
}

// ═══════════════════════════════════════════════════════════════
// NullLogger
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, NullLoggerDiscardsMessages) {
    drone::log::NullLogger null;
    // should_log always returns false.
    EXPECT_FALSE(null.should_log(drone::log::Level::Debug));
    EXPECT_FALSE(null.should_log(drone::log::Level::Info));
    EXPECT_FALSE(null.should_log(drone::log::Level::Error));
    // log() is a no-op — just verify it doesn't crash.
    null.log(drone::log::Level::Error, "this should be silently discarded");
}

TEST(ILoggerTest, NullLoggerAsGlobal) {
    LoggerGuard guard;
    drone::log::set_logger(std::make_unique<drone::log::NullLogger>());
    auto& lg = drone::log::logger();
    EXPECT_FALSE(lg.should_log(drone::log::Level::Info));
    // DRONE_LOG_INFO should not crash even though logger discards.
    DRONE_LOG_INFO("This message should be discarded by NullLogger");
}

// ═══════════════════════════════════════════════════════════════
// CapturingLogger
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, CapturingLoggerCapturesMessages) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("hello {}", "world");
    DRONE_LOG_WARN("warning #{}", 42);
    DRONE_LOG_ERROR("error: {}", "bad");

    EXPECT_EQ(ptr->count(), 3u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Info), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Warn), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Error), 1u);
}

TEST(ILoggerTest, CapturingLoggerContainsSubstring) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("Camera {} started @ {}Hz", "mission_cam", 30);

    EXPECT_TRUE(ptr->contains("Camera"));
    EXPECT_TRUE(ptr->contains("mission_cam"));
    EXPECT_TRUE(ptr->contains("30Hz"));
    EXPECT_FALSE(ptr->contains("nonexistent"));
}

TEST(ILoggerTest, CapturingLoggerContainsAtLevel) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("info message");
    DRONE_LOG_ERROR("error message");

    EXPECT_TRUE(ptr->contains_at(drone::log::Level::Info, "info"));
    EXPECT_TRUE(ptr->contains_at(drone::log::Level::Error, "error"));
    EXPECT_FALSE(ptr->contains_at(drone::log::Level::Info, "error"));
    EXPECT_FALSE(ptr->contains_at(drone::log::Level::Error, "info"));
}

TEST(ILoggerTest, CapturingLoggerMinLevel) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>(drone::log::Level::Warn);
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    // Debug and Info should be filtered by should_log check in macros.
    DRONE_LOG_DEBUG("debug msg");
    DRONE_LOG_INFO("info msg");
    DRONE_LOG_WARN("warn msg");
    DRONE_LOG_ERROR("error msg");

    // Only warn and error should be captured (debug/info filtered by should_log).
    EXPECT_EQ(ptr->count(), 2u);
    EXPECT_TRUE(ptr->contains("warn msg"));
    EXPECT_TRUE(ptr->contains("error msg"));
}

TEST(ILoggerTest, CapturingLoggerClear) {
    drone::log::CapturingLogger cap;
    cap.log(drone::log::Level::Info, "msg1");
    cap.log(drone::log::Level::Info, "msg2");
    EXPECT_EQ(cap.count(), 2u);

    cap.clear();
    EXPECT_EQ(cap.count(), 0u);
}

TEST(ILoggerTest, CapturingLoggerMessages) {
    drone::log::CapturingLogger cap;
    cap.log(drone::log::Level::Info, "first");
    cap.log(drone::log::Level::Error, "second");

    const auto& msgs = cap.messages();
    ASSERT_EQ(msgs.size(), 2u);
    EXPECT_EQ(msgs[0].level, drone::log::Level::Info);
    EXPECT_EQ(msgs[0].text, "first");
    EXPECT_EQ(msgs[1].level, drone::log::Level::Error);
    EXPECT_EQ(msgs[1].text, "second");
}

// ═══════════════════════════════════════════════════════════════
// Global accessor: set_logger / reset_logger
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, SetAndResetLogger) {
    LoggerGuard guard;

    // Install capturing logger.
    auto  cap = std::make_unique<drone::log::CapturingLogger>();
    auto* ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("captured");
    EXPECT_EQ(ptr->count(), 1u);

    // Reset to default.
    drone::log::reset_logger();

    // Default logger is SpdlogLogger — should not crash.
    DRONE_LOG_INFO("back to spdlog");
}

// ═══════════════════════════════════════════════════════════════
// DRONE_LOG macros
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, DroneLogMacroLevels) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_DEBUG("debug");
    DRONE_LOG_INFO("info");
    DRONE_LOG_WARN("warn");
    DRONE_LOG_ERROR("error");
    DRONE_LOG_CRITICAL("critical");

    EXPECT_EQ(ptr->count(), 5u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Debug), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Info), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Warn), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Error), 1u);
    EXPECT_EQ(ptr->count_at(drone::log::Level::Critical), 1u);
}

TEST(ILoggerTest, DroneLogGenericMacro) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG(drone::log::Level::Warn, "dynamic level: {}", 123);
    EXPECT_EQ(ptr->count(), 1u);
    EXPECT_TRUE(ptr->contains_at(drone::log::Level::Warn, "dynamic level: 123"));
}

TEST(ILoggerTest, DroneLogNoArgs) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("no format args here");
    EXPECT_TRUE(ptr->contains("no format args here"));
}

TEST(ILoggerTest, DroneLogFmtFormatting) {
    LoggerGuard guard;
    auto        cap = std::make_unique<drone::log::CapturingLogger>();
    auto*       ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    int         x = 42;
    double      y = 3.14;
    std::string s = "test";
    DRONE_LOG_INFO("int={} float={:.2f} str={}", x, y, s);
    EXPECT_TRUE(ptr->contains("int=42 float=3.14 str=test"));
}

// ═══════════════════════════════════════════════════════════════
// Level enum
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, LevelOrdering) {
    using L = drone::log::Level;
    EXPECT_LT(static_cast<uint8_t>(L::Debug), static_cast<uint8_t>(L::Info));
    EXPECT_LT(static_cast<uint8_t>(L::Info), static_cast<uint8_t>(L::Warn));
    EXPECT_LT(static_cast<uint8_t>(L::Warn), static_cast<uint8_t>(L::Error));
    EXPECT_LT(static_cast<uint8_t>(L::Error), static_cast<uint8_t>(L::Critical));
}
