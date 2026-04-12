// tests/test_process_context.cpp
// Tests for drone::util::init_process() and ProcessContext.
// Issue #291 (Epic #284 — Platform Modularity)

#include "util/config_validator.h"
#include "util/process_context.h"

#include <atomic>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

// ── Helper: build argc/argv from a vector of strings ────────
class ArgvBuilder {
public:
    explicit ArgvBuilder(std::vector<std::string> args) : args_(std::move(args)) {
        for (auto& s : args_) {
            ptrs_.push_back(s.data());
        }
    }
    int    argc() const { return static_cast<int>(ptrs_.size()); }
    char** argv() { return ptrs_.data(); }

private:
    std::vector<std::string> args_;
    std::vector<char*>       ptrs_;
};

// ── Test: --help returns exit code 0 (not an error) ─────────
TEST(ProcessContext, HelpReturnsExitCodeZero) {
    std::atomic<bool> running{true};
    ArgvBuilder       argv_builder({"test_process", "--help"});

    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    EXPECT_FALSE(result.is_ok());
    EXPECT_EQ(result.error(), 0);
}

// ── Test: valid config produces a usable ProcessContext ──────
TEST(ProcessContext, ValidConfigProducesContext) {
    std::atomic<bool> running{true};

    // Use the real project config if available via compile-time define
#ifdef PROJECT_CONFIG_DIR
    std::string config_path = std::string(PROJECT_CONFIG_DIR) + "/default.json";
#else
    std::string config_path = "config/default.json";
#endif

    ArgvBuilder argv_builder({"test_process", "--config", config_path});

    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    // PROJECT_CONFIG_DIR is set at compile time — config should always be found.
    // If init_process fails here it's a real bug, not a skip condition.
    ASSERT_TRUE(result.is_ok()) << "init_process() failed with config at: " << config_path;

    auto& ctx = result.value();
    EXPECT_EQ(ctx.process_name, "test_proc");
    EXPECT_EQ(&ctx.running, &running);
    EXPECT_FALSE(ctx.args.help);
}

// ── Test: no args uses defaults, context still created ──────
TEST(ProcessContext, NoArgsUsesDefaults) {
    std::atomic<bool> running{true};
    ArgvBuilder       argv_builder({"test_process"});

    // With no --config, it tries config/default.json. If it doesn't exist,
    // it still proceeds with default config (load warning).
    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    // Should succeed even if config file is missing (uses defaults)
    ASSERT_TRUE(result.is_ok());
    auto& ctx = result.value();
    EXPECT_EQ(ctx.process_name, "test_proc");
}

// ── Test: ProcessContext is move-constructible ───────────────
TEST(ProcessContext, IsMoveConstructible) {
    EXPECT_TRUE(std::is_move_constructible_v<drone::util::ProcessContext>);
}

// ── Test: ProcessContext is not copy-constructible ───────────
TEST(ProcessContext, IsNotCopyConstructible) {
    EXPECT_FALSE(std::is_copy_constructible_v<drone::util::ProcessContext>);
}

// ── Test: running flag reference is correct ─────────────────
TEST(ProcessContext, RunningFlagReference) {
    std::atomic<bool> running{true};
    ArgvBuilder       argv_builder({"test_process"});

    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    ASSERT_TRUE(result.is_ok());
    auto& ctx = result.value();

    // Verify the reference points to our flag
    EXPECT_TRUE(ctx.running.load(std::memory_order_acquire));
    running.store(false, std::memory_order_release);
    EXPECT_FALSE(ctx.running.load(std::memory_order_acquire));
}

// ── Test: parsed args are preserved ─────────────────────────
TEST(ProcessContext, ParsedArgsPreserved) {
    std::atomic<bool> running{true};
    ArgvBuilder       argv_builder({"test_process", "--log-level", "debug", "--json-logs"});

    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    ASSERT_TRUE(result.is_ok());
    auto& ctx = result.value();
    EXPECT_EQ(ctx.args.log_level, "debug");
    EXPECT_TRUE(ctx.args.json_logs);
}
