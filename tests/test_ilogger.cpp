// tests/test_ilogger.cpp
// Tests for the ILogger abstraction: interface, SpdlogLogger, NullLogger,
// CapturingLogger, DRONE_LOG macros, and global accessor.
//
// Issue #285 — ILogger interface + DRONE_LOG macros.

#include "util/capturing_logger.h"
#include "util/ilogger.h"
#include "util/null_logger.h"
#include "util/spdlog_logger.h"

#include <atomic>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ── RAII guard: restore default logger after each test ──────
class LoggerGuard {
public:
    ~LoggerGuard() { drone::log::reset_logger(); }
};

// ═══════════════════════════════════════════════════════════════
// StderrFallbackLogger (default before SpdlogLogger is installed)
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, DefaultLoggerIsStderrFallback) {
    // Before init_process() installs SpdlogLogger, the default is
    // StderrFallbackLogger.  It should_log all levels (no filtering).
    auto& lg = drone::log::logger();
    lg.log(drone::log::Level::Info, "test message from default logger");

    // StderrFallbackLogger returns true for ALL levels (no filtering).
    // This distinguishes it from SpdlogLogger which filters by spdlog level.
    EXPECT_TRUE(lg.should_log(drone::log::Level::Debug));
    EXPECT_TRUE(lg.should_log(drone::log::Level::Info));
    EXPECT_TRUE(lg.should_log(drone::log::Level::Warn));
    EXPECT_TRUE(lg.should_log(drone::log::Level::Error));
    EXPECT_TRUE(lg.should_log(drone::log::Level::Critical));
}

TEST(ILoggerTest, SpdlogLoggerLogBypassesFmtReformat) {
    // Verify SpdlogLogger::log() correctly passes pre-formatted messages
    // to spdlog without double-format overhead.  The message arrives
    // already formatted by DRONE_LOG_* macros, so SpdlogLogger must NOT
    // re-parse it through fmt.  We verify by sending a message containing
    // fmt-special characters ('{', '}') — if spdlog re-parsed, it would
    // throw or corrupt the output.
    LoggerGuard guard;

    auto  cap = std::make_unique<drone::log::CapturingLogger>();
    auto* ptr = cap.get();

    // First: route through SpdlogLogger to verify it doesn't crash on
    // fmt-special characters.  SpdlogLogger sends to spdlog's default
    // logger, so we can't easily capture its output here.  But we CAN
    // verify it doesn't throw or abort.
    {
        drone::log::SpdlogLogger spdlog_logger;
        // These would throw/crash if spdlog re-parsed them as fmt strings
        spdlog_logger.log(drone::log::Level::Info, "braces: {key} {value}");
        spdlog_logger.log(drone::log::Level::Warn, "nested: {{already escaped}}");
        spdlog_logger.log(drone::log::Level::Error, "mixed: {0} and {1}");
    }

    // Second: verify the interface contract via CapturingLogger — the
    // message string_view is passed through unchanged.
    drone::log::set_logger(std::move(cap));
    DRONE_LOG_INFO("formatted: x={} y={}", 10, 20);
    ASSERT_EQ(ptr->count(), 1u);
    EXPECT_TRUE(ptr->contains("formatted: x=10 y=20"));
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

    // Reset to default (StderrFallbackLogger).
    drone::log::reset_logger();

    // Default logger is StderrFallbackLogger — should not crash.
    DRONE_LOG_INFO("back to default logger");
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

// ═══════════════════════════════════════════════════════════════
// RCU-style retirement (Issue #384)
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerRCUTest, OldLoggerSurvivesAfterSetLogger) {
    // The old logger must remain callable after set_logger() replaces it,
    // because the retirement vector keeps it alive.
    LoggerGuard guard;

    auto  cap1 = std::make_unique<drone::log::CapturingLogger>();
    auto* ptr1 = cap1.get();
    drone::log::set_logger(std::move(cap1));

    DRONE_LOG_INFO("msg1");
    EXPECT_EQ(ptr1->count(), 1u);

    // Replace with a second logger — ptr1 should still be alive (retired).
    auto  cap2 = std::make_unique<drone::log::CapturingLogger>();
    auto* ptr2 = cap2.get();
    drone::log::set_logger(std::move(cap2));

    // The old logger (ptr1) must still be callable — not destroyed.
    EXPECT_EQ(ptr1->count(), 1u);
    ptr1->log(drone::log::Level::Info, "still alive");
    EXPECT_EQ(ptr1->count(), 2u);

    // New logger works too.
    DRONE_LOG_INFO("msg2");
    EXPECT_EQ(ptr2->count(), 1u);
}

TEST(ILoggerRCUTest, OldLoggerSurvivesAfterResetLogger) {
    LoggerGuard guard;

    auto  cap = std::make_unique<drone::log::CapturingLogger>();
    auto* ptr = cap.get();
    drone::log::set_logger(std::move(cap));

    DRONE_LOG_INFO("before reset");
    EXPECT_EQ(ptr->count(), 1u);

    // Reset to default — old logger must survive (retired).
    drone::log::reset_logger();

    // The old logger (ptr) must still be callable.
    EXPECT_EQ(ptr->count(), 1u);
    ptr->log(drone::log::Level::Info, "still alive after reset");
    EXPECT_EQ(ptr->count(), 2u);
}

// ═══════════════════════════════════════════════════════════════
// StderrFallbackLogger — output format and boundary tests
// ═══════════════════════════════════════════════════════════════

TEST(ILoggerTest, StderrFallbackLoggerOutputFormat) {
    // Verify StderrFallbackLogger writes "[LEVEL] msg\n" to stderr.
    // Redirect stderr to a temp file to capture output.
    drone::log::StderrFallbackLogger fallback;

    // Save original stderr
    int orig_stderr = ::dup(STDERR_FILENO);

    std::string tmp_path = "/tmp/drone_test_stderr_" + std::to_string(::getpid()) + ".txt";
    FILE*       tmp      = std::fopen(tmp_path.c_str(), "w");
    ASSERT_NE(tmp, nullptr);
    ::dup2(::fileno(tmp), STDERR_FILENO);

    fallback.log(drone::log::Level::Info, "hello world");
    std::fflush(stderr);

    // Restore stderr
    ::dup2(orig_stderr, STDERR_FILENO);
    ::close(orig_stderr);
    std::fclose(tmp);

    // Read captured output
    std::ifstream ifs(tmp_path);
    std::string   output((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();
    std::remove(tmp_path.c_str());

    EXPECT_EQ(output, "[INFO] hello world\n");
}

TEST(ILoggerTest, StderrFallbackLoggerUnknownLevel) {
    // Verify that out-of-range Level values produce "UNKNOWN" label.
    drone::log::StderrFallbackLogger fallback;

    int orig_stderr = ::dup(STDERR_FILENO);

    std::string tmp_path = "/tmp/drone_test_stderr_unk_" + std::to_string(::getpid()) + ".txt";
    FILE*       tmp      = std::fopen(tmp_path.c_str(), "w");
    ASSERT_NE(tmp, nullptr);
    ::dup2(::fileno(tmp), STDERR_FILENO);

    fallback.log(static_cast<drone::log::Level>(99), "boundary test");
    std::fflush(stderr);

    ::dup2(orig_stderr, STDERR_FILENO);
    ::close(orig_stderr);
    std::fclose(tmp);

    std::ifstream ifs(tmp_path);
    std::string   output((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();
    std::remove(tmp_path.c_str());

    EXPECT_EQ(output, "[UNKNOWN] boundary test\n");
}

TEST(ILoggerRCUTest, ConcurrentLoggerAccessDuringSwap) {
    // Stress test: multiple threads call logger() while another thread
    // swaps loggers.  Must not crash or trigger TSAN/ASAN.
    //
    // Uses NullLogger (thread-safe, no-op) rather than CapturingLogger
    // (not thread-safe) since we are testing the swap mechanism, not
    // the logger implementation.
    LoggerGuard guard;

    constexpr int kIterations = 5000;
    constexpr int kReaders    = 4;

    std::atomic<bool> start{false};
    std::atomic<bool> stop{false};

    // Reader threads: continuously call logger() and invoke should_log().
    // NullLogger::should_log() returns false, so log() is never called.
    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    for (int i = 0; i < kReaders; ++i) {
        readers.emplace_back([&] {
            while (!start.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            while (!stop.load(std::memory_order_acquire)) {
                auto& lg = drone::log::logger();
                // Exercise the pointer dereference — the key safety property.
                auto can_log = lg.should_log(drone::log::Level::Debug);
                (void)can_log;
            }
        });
    }

    start.store(true, std::memory_order_release);

    // Writer thread: swap loggers repeatedly.
    for (int i = 0; i < kIterations; ++i) {
        drone::log::set_logger(std::make_unique<drone::log::NullLogger>());
    }

    stop.store(true, std::memory_order_release);
    for (auto& t : readers) {
        t.join();
    }

    // If we get here without crash/TSAN flag, the test passes.
    drone::log::reset_logger();
}
