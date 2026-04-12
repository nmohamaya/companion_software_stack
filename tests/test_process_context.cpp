// tests/test_process_context.cpp
// Tests for drone::util::init_process() and ProcessContext.
// Issue #291 (Epic #284 — Platform Modularity)

#include "util/config_validator.h"
#include "util/process_context.h"

#include <atomic>
#include <cstdio>
#include <fstream>
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

// ── Test: --skip-validation flag is parsed (Debug builds only) ──
// Gated behind #ifndef NDEBUG because --skip-validation is a security-sensitive
// flag intentionally disabled in Release builds. In Release, the flag is ignored
// with a stderr warning (args.skip_validation stays false), so tests that assert
// it becomes true would fail. Run with Debug/RelWithDebInfo build to test.
#ifndef NDEBUG
TEST(ProcessContext, SkipValidationFlagParsed) {
    ArgvBuilder argv_builder({"test_process", "--skip-validation"});
    auto        args = parse_args(argv_builder.argc(), argv_builder.argv(), "test");
    EXPECT_TRUE(args.skip_validation);
}
#endif  // NDEBUG

TEST(ProcessContext, SkipValidationFlagDefaultFalse) {
    ArgvBuilder argv_builder({"test_process"});
    auto        args = parse_args(argv_builder.argc(), argv_builder.argv(), "test");
    EXPECT_FALSE(args.skip_validation);
}

// ── Test: --skip-validation skips config schema enforcement (Debug only) ──
// Gated behind #ifndef NDEBUG: --skip-validation is ignored in Release builds
// (NDEBUG defined by CMake -DCMAKE_BUILD_TYPE=Release), so the "skip" path
// cannot be exercised. These tests only compile and run in Debug builds.
#ifndef NDEBUG
TEST(ProcessContext, SkipValidationAllowsInvalidConfig) {
    std::atomic<bool> running{true};

    // Create a config that's invalid: slam.vio_rate_hz = 0 (below min range of 1)
    std::string tmp_path = "/tmp/drone_test_skip_validation_" + std::to_string(::getpid()) +
                           ".json";
    {
        std::ofstream ofs(tmp_path);
        ASSERT_TRUE(ofs.is_open()) << "Failed to create temp config at: " << tmp_path;
        ofs << R"({"slam": {"vio_rate_hz": 0}})";
    }

    // A schema requiring vio_rate_hz in [1, 10000]
    auto schema = drone::util::slam_schema();

    // WITH --skip-validation: should succeed despite invalid config
    ArgvBuilder argv_skip({"test_process", "--config", tmp_path, "--skip-validation"});
    auto result_skip = drone::util::init_process(argv_skip.argc(), argv_skip.argv(), "test_proc",
                                                 running, schema);
    EXPECT_TRUE(result_skip.is_ok()) << "Expected success with --skip-validation";

    // WITHOUT --skip-validation: should fail due to out-of-range vio_rate_hz
    std::atomic<bool> running2{true};
    ArgvBuilder       argv_no_skip({"test_process", "--config", tmp_path});
    auto result_no_skip = drone::util::init_process(argv_no_skip.argc(), argv_no_skip.argv(),
                                                    "test_proc2", running2, schema);
    EXPECT_FALSE(result_no_skip.is_ok()) << "Expected failure without --skip-validation";
    EXPECT_EQ(result_no_skip.error(), 1);

    std::remove(tmp_path.c_str());
}
#endif  // NDEBUG

// ── Test: --skip-validation is preserved in ProcessContext (Debug only) ──
// Gated behind #ifndef NDEBUG: the flag is ignored in Release builds.
#ifndef NDEBUG
TEST(ProcessContext, SkipValidationPreservedInContext) {
    std::atomic<bool> running{true};
    ArgvBuilder       argv_builder({"test_process", "--skip-validation"});

    auto result = drone::util::init_process(argv_builder.argc(), argv_builder.argv(), "test_proc",
                                            running, drone::util::common_schema());

    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().args.skip_validation);
}
#endif  // NDEBUG
