// tests/test_restart_policy.cpp
// Unit tests for RestartPolicy, ProcessConfig, and StackStatus (Phase 4, #92).

#include "util/restart_policy.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using drone::util::ProcessConfig;
using drone::util::RestartPolicy;
using drone::util::StackStatus;
using drone::util::to_string;

// ═══════════════════════════════════════════════════════════
// RestartPolicy — backoff calculation
// ═══════════════════════════════════════════════════════════

TEST(RestartPolicy, BackoffDoublesEachAttempt) {
    RestartPolicy policy;
    policy.initial_backoff_ms = 500;
    policy.max_backoff_ms     = 30000;

    EXPECT_EQ(policy.backoff_ms(0), 500);    // 500
    EXPECT_EQ(policy.backoff_ms(1), 1000);   // 500 * 2
    EXPECT_EQ(policy.backoff_ms(2), 2000);   // 1000 * 2
    EXPECT_EQ(policy.backoff_ms(3), 4000);   // 2000 * 2
    EXPECT_EQ(policy.backoff_ms(4), 8000);   // 4000 * 2
    EXPECT_EQ(policy.backoff_ms(5), 16000);  // 8000 * 2
}

TEST(RestartPolicy, BackoffCapsAtMax) {
    RestartPolicy policy;
    policy.initial_backoff_ms = 500;
    policy.max_backoff_ms     = 4000;

    EXPECT_EQ(policy.backoff_ms(0), 500);    // 500
    EXPECT_EQ(policy.backoff_ms(1), 1000);   // 500 * 2
    EXPECT_EQ(policy.backoff_ms(2), 2000);   // 1000 * 2
    EXPECT_EQ(policy.backoff_ms(3), 4000);   // 2000 * 2 = 4000 = max
    EXPECT_EQ(policy.backoff_ms(4), 4000);   // capped at max
    EXPECT_EQ(policy.backoff_ms(10), 4000);  // still capped
}

TEST(RestartPolicy, BackoffWithLargeInitial) {
    RestartPolicy policy;
    policy.initial_backoff_ms = 10000;
    policy.max_backoff_ms     = 30000;

    EXPECT_EQ(policy.backoff_ms(0), 10000);  // 10000
    EXPECT_EQ(policy.backoff_ms(1), 20000);  // 10000 * 2
    EXPECT_EQ(policy.backoff_ms(2), 30000);  // capped at 30000
}

TEST(RestartPolicy, BackoffFirstAttemptEqualsInitial) {
    RestartPolicy policy;
    policy.initial_backoff_ms = 1234;
    EXPECT_EQ(policy.backoff_ms(0), 1234);
}

// ═══════════════════════════════════════════════════════════
// RestartPolicy — thermal gate
// ═══════════════════════════════════════════════════════════

TEST(RestartPolicy, ThermalGateBlocksAtThreshold) {
    RestartPolicy policy;
    policy.thermal_gate = 3;  // Block at CRITICAL (zone >= 3)

    EXPECT_FALSE(policy.is_thermal_blocked(0));  // Normal
    EXPECT_FALSE(policy.is_thermal_blocked(1));  // Warm
    EXPECT_FALSE(policy.is_thermal_blocked(2));  // Hot
    EXPECT_TRUE(policy.is_thermal_blocked(3));   // Critical - blocked
}

TEST(RestartPolicy, ThermalGateZeroAlwaysBlocks) {
    RestartPolicy policy;
    policy.thermal_gate = 0;  // Always block

    EXPECT_TRUE(policy.is_thermal_blocked(0));  // Even normal is blocked
    EXPECT_TRUE(policy.is_thermal_blocked(1));
    EXPECT_TRUE(policy.is_thermal_blocked(3));
}

TEST(RestartPolicy, ThermalGateFourNeverBlocks) {
    RestartPolicy policy;
    policy.thermal_gate = 4;  // Never block

    EXPECT_FALSE(policy.is_thermal_blocked(0));
    EXPECT_FALSE(policy.is_thermal_blocked(1));
    EXPECT_FALSE(policy.is_thermal_blocked(2));
    EXPECT_FALSE(policy.is_thermal_blocked(3));
    EXPECT_FALSE(policy.is_thermal_blocked(255));  // Even extreme
}

TEST(RestartPolicy, ThermalGateAtHot) {
    RestartPolicy policy;
    policy.thermal_gate = 2;  // Block at HOT or hotter

    EXPECT_FALSE(policy.is_thermal_blocked(0));  // Normal
    EXPECT_FALSE(policy.is_thermal_blocked(1));  // Warm
    EXPECT_TRUE(policy.is_thermal_blocked(2));   // Hot - blocked
    EXPECT_TRUE(policy.is_thermal_blocked(3));   // Critical - blocked
}

// ═══════════════════════════════════════════════════════════
// RestartPolicy — defaults
// ═══════════════════════════════════════════════════════════

TEST(RestartPolicy, DefaultValues) {
    RestartPolicy policy;
    EXPECT_EQ(policy.max_restarts, 5u);
    EXPECT_EQ(policy.cooldown_window_s, 60u);
    EXPECT_EQ(policy.initial_backoff_ms, 500u);
    EXPECT_EQ(policy.max_backoff_ms, 30000u);
    EXPECT_FALSE(policy.is_critical);
    EXPECT_EQ(policy.thermal_gate, 3);
}

// ═══════════════════════════════════════════════════════════
// StackStatus — to_string
// ═══════════════════════════════════════════════════════════

TEST(StackStatus, ToStringCoversAllStates) {
    EXPECT_STREQ(to_string(StackStatus::NOMINAL), "NOMINAL");
    EXPECT_STREQ(to_string(StackStatus::DEGRADED), "DEGRADED");
    EXPECT_STREQ(to_string(StackStatus::CRITICAL), "CRITICAL");
}

TEST(StackStatus, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(StackStatus::NOMINAL), 0);
    EXPECT_EQ(static_cast<uint8_t>(StackStatus::DEGRADED), 1);
    EXPECT_EQ(static_cast<uint8_t>(StackStatus::CRITICAL), 2);
}

// ═══════════════════════════════════════════════════════════
// ProcessConfig — JSON parsing
// ═══════════════════════════════════════════════════════════

TEST(ProcessConfig, FromJsonFullConfig) {
    nlohmann::json j = {
        {"binary", "build/bin/comms"},
        {"critical", true},
        {"max_restarts", 10},
        {"cooldown_s", 120},
        {"backoff_ms", 1000},
        {"max_backoff_ms", 60000},
        {"thermal_gate", 2},
        {"launch_after", {"video_capture", "perception"}},
        {"restart_cascade", {"mission_planner", "payload_manager"}},
    };

    auto cfg = ProcessConfig::from_json("comms", j);

    EXPECT_EQ(cfg.name, "comms");
    EXPECT_EQ(cfg.binary, "build/bin/comms");
    EXPECT_TRUE(cfg.policy.is_critical);
    EXPECT_EQ(cfg.policy.max_restarts, 10u);
    EXPECT_EQ(cfg.policy.cooldown_window_s, 120u);
    EXPECT_EQ(cfg.policy.initial_backoff_ms, 1000u);
    EXPECT_EQ(cfg.policy.max_backoff_ms, 60000u);
    EXPECT_EQ(cfg.policy.thermal_gate, 2);

    ASSERT_EQ(cfg.launch_after.size(), 2u);
    EXPECT_EQ(cfg.launch_after[0], "video_capture");
    EXPECT_EQ(cfg.launch_after[1], "perception");

    ASSERT_EQ(cfg.restart_cascade.size(), 2u);
    EXPECT_EQ(cfg.restart_cascade[0], "mission_planner");
    EXPECT_EQ(cfg.restart_cascade[1], "payload_manager");
}

TEST(ProcessConfig, FromJsonMissingFieldsUseDefaults) {
    nlohmann::json j = nlohmann::json::object();

    auto cfg = ProcessConfig::from_json("test_proc", j);

    EXPECT_EQ(cfg.name, "test_proc");
    EXPECT_TRUE(cfg.binary.empty());
    EXPECT_FALSE(cfg.policy.is_critical);
    EXPECT_EQ(cfg.policy.max_restarts, 5u);
    EXPECT_EQ(cfg.policy.cooldown_window_s, 60u);
    EXPECT_EQ(cfg.policy.initial_backoff_ms, 500u);
    EXPECT_EQ(cfg.policy.max_backoff_ms, 30000u);
    EXPECT_EQ(cfg.policy.thermal_gate, 3);
    EXPECT_TRUE(cfg.launch_after.empty());
    EXPECT_TRUE(cfg.restart_cascade.empty());
}

TEST(ProcessConfig, FromJsonPartialConfig) {
    nlohmann::json j = {
        {"critical", true},
        {"max_restarts", 3},
    };

    auto cfg = ProcessConfig::from_json("slam", j);

    EXPECT_EQ(cfg.name, "slam");
    EXPECT_TRUE(cfg.policy.is_critical);
    EXPECT_EQ(cfg.policy.max_restarts, 3u);
    // Other fields should be default
    EXPECT_EQ(cfg.policy.cooldown_window_s, 60u);
    EXPECT_EQ(cfg.policy.initial_backoff_ms, 500u);
}

TEST(ProcessConfig, FromJsonEmptyArrays) {
    nlohmann::json j = {
        {"launch_after", nlohmann::json::array()},
        {"restart_cascade", nlohmann::json::array()},
    };

    auto cfg = ProcessConfig::from_json("video", j);
    EXPECT_TRUE(cfg.launch_after.empty());
    EXPECT_TRUE(cfg.restart_cascade.empty());
}

TEST(ProcessConfig, FromJsonInvalidArrayElementsIgnored) {
    nlohmann::json j = {
        {"launch_after", {42, "valid_dep", true}},  // non-string elements
    };

    auto cfg = ProcessConfig::from_json("test", j);
    ASSERT_EQ(cfg.launch_after.size(), 1u);
    EXPECT_EQ(cfg.launch_after[0], "valid_dep");
}

TEST(ProcessConfig, FromJsonNegativeValuesClampedToZero) {
    nlohmann::json j = {
        {"max_restarts", -1},       {"cooldown_s", -10},  {"backoff_ms", -500},
        {"max_backoff_ms", -30000}, {"thermal_gate", -2},
    };

    auto cfg = ProcessConfig::from_json("neg", j);
    EXPECT_EQ(cfg.policy.max_restarts, 0u);
    EXPECT_EQ(cfg.policy.cooldown_window_s, 0u);
    EXPECT_EQ(cfg.policy.initial_backoff_ms, 0u);
    EXPECT_EQ(cfg.policy.max_backoff_ms, 0u);
    EXPECT_EQ(cfg.policy.thermal_gate, 0);
}

TEST(ProcessConfig, FromJsonThermalGateClampedToFour) {
    nlohmann::json j   = {{"thermal_gate", 99}};
    auto           cfg = ProcessConfig::from_json("hot", j);
    EXPECT_EQ(cfg.policy.thermal_gate, 4);
}

// ═══════════════════════════════════════════════════════════
// ProcessConfig — loading from full default.json
// ═══════════════════════════════════════════════════════════

TEST(ProcessConfig, LoadFromDefaultJsonFile) {
    // Load the actual default.json if available
    std::ifstream ifs("config/default.json");
    if (!ifs.is_open()) {
        // Try from build directory
        ifs.open("../config/default.json");
    }
    if (!ifs.is_open()) {
        GTEST_SKIP() << "config/default.json not found";
    }

    auto data = nlohmann::json::parse(ifs);
    ASSERT_TRUE(data.contains("watchdog"));
    ASSERT_TRUE(data["watchdog"].contains("processes"));

    auto& procs = data["watchdog"]["processes"];
    ASSERT_TRUE(procs.contains("comms"));
    ASSERT_TRUE(procs.contains("slam_vio_nav"));
    ASSERT_TRUE(procs.contains("video_capture"));

    // Verify comms is critical
    auto comms = ProcessConfig::from_json("comms", procs["comms"]);
    EXPECT_TRUE(comms.policy.is_critical);
    EXPECT_EQ(comms.restart_cascade.size(), 2u);

    // Verify slam_vio_nav is critical
    auto slam = ProcessConfig::from_json("slam_vio_nav", procs["slam_vio_nav"]);
    EXPECT_TRUE(slam.policy.is_critical);

    // Verify video_capture is NOT critical
    auto video = ProcessConfig::from_json("video_capture", procs["video_capture"]);
    EXPECT_FALSE(video.policy.is_critical);
}
