// common/util/include/util/capturing_logger.h
// Test-support ILogger that captures all messages for assertion.
//
// Usage in GTest:
//   auto logger = std::make_unique<drone::log::CapturingLogger>();
//   auto* cap = logger.get();
//   drone::log::set_logger(std::move(logger));
//
//   // ... run code under test that logs ...
//
//   EXPECT_EQ(cap->count(), 2);
//   EXPECT_TRUE(cap->contains("started"));
//   EXPECT_EQ(cap->count_at(drone::log::Level::Error), 1);
//
//   drone::log::reset_logger();  // restore default
#pragma once

#include "util/ilogger.h"

#include <algorithm>
#include <string>
#include <vector>

namespace drone::log {

/// Captured log message with level and text.
struct CapturedMessage {
    Level       level;
    std::string text;
};

/// Logger that stores all messages in memory for test assertions.
/// Not thread-safe — designed for single-threaded test use.
class CapturingLogger final : public ILogger {
public:
    /// Minimum level to capture (default: Debug — capture everything).
    explicit CapturingLogger(Level min_level = Level::Debug) : min_level_(min_level) {}

    void log(Level level, const std::string& msg) override { messages_.push_back({level, msg}); }

    [[nodiscard]] bool should_log(Level level) const override { return level >= min_level_; }

    // ── Query API ──────────────────────────────────────────

    /// Total number of captured messages.
    [[nodiscard]] size_t count() const { return messages_.size(); }

    /// Number of messages at a specific level.
    [[nodiscard]] size_t count_at(Level level) const {
        return static_cast<size_t>(
            std::count_if(messages_.begin(), messages_.end(),
                          [level](const CapturedMessage& m) { return m.level == level; }));
    }

    /// Check if any message contains the given substring.
    [[nodiscard]] bool contains(const std::string& substr) const {
        return std::any_of(messages_.begin(), messages_.end(), [&substr](const CapturedMessage& m) {
            return m.text.find(substr) != std::string::npos;
        });
    }

    /// Check if any message at the given level contains the substring.
    [[nodiscard]] bool contains_at(Level level, const std::string& substr) const {
        return std::any_of(messages_.begin(), messages_.end(), [&](const CapturedMessage& m) {
            return m.level == level && m.text.find(substr) != std::string::npos;
        });
    }

    /// Get all captured messages.
    [[nodiscard]] const std::vector<CapturedMessage>& messages() const { return messages_; }

    /// Clear all captured messages.
    void clear() { messages_.clear(); }

private:
    Level                        min_level_;
    std::vector<CapturedMessage> messages_;
};

}  // namespace drone::log
