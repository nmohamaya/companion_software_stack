// tests/test_zenoh_liveliness.cpp
// Phase F — liveliness token + monitor tests.
//
// Tests the LivelinessToken, LivelinessMonitor, ProcessHealthEntry struct,
// and the extract_process_name utility.  Tests that require a real Zenoh
// session are guarded by HAVE_ZENOH.

#include "ipc/ipc_types.h"
#include "ipc/zenoh_liveliness.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Struct / constant tests (always run — no Zenoh required)
// ═══════════════════════════════════════════════════════════

TEST(LivelinessConstants, PrefixValue) {
    EXPECT_STREQ(kLivelinessPrefix, "drone/alive/");
}

TEST(LivelinessConstants, WildcardValue) {
    EXPECT_STREQ(kLivelinessWildcard, "drone/alive/**");
}

TEST(LivelinessExtract, BasicProcessName) {
    EXPECT_EQ(LivelinessToken::extract_process_name("drone/alive/video_capture"), "video_capture");
}

TEST(LivelinessExtract, SlamVioNav) {
    EXPECT_EQ(LivelinessToken::extract_process_name("drone/alive/slam_vio_nav"), "slam_vio_nav");
}

TEST(LivelinessExtract, SystemMonitor) {
    EXPECT_EQ(LivelinessToken::extract_process_name("drone/alive/system_monitor"),
              "system_monitor");
}

TEST(LivelinessExtract, NoPrefix) {
    // If the key doesn't have the prefix, return as-is
    EXPECT_EQ(LivelinessToken::extract_process_name("something/else"), "something/else");
}

TEST(LivelinessExtract, EmptyString) {
    EXPECT_EQ(LivelinessToken::extract_process_name(""), "");
}

TEST(LivelinessExtract, PrefixOnly) {
    // "drone/alive/" with nothing after it
    EXPECT_EQ(LivelinessToken::extract_process_name("drone/alive/"), "");
}

TEST(LivelinessExtract, AllProcessNames) {
    // Verify all 7 process names extract correctly
    const std::vector<std::string> names = {"video_capture",   "perception", "slam_vio_nav",
                                            "mission_planner", "comms",      "payload_manager",
                                            "system_monitor"};
    for (const auto& name : names) {
        const std::string key = std::string(kLivelinessPrefix) + name;
        EXPECT_EQ(LivelinessToken::extract_process_name(key), name) << "Failed for: " << key;
    }
}

// ═══════════════════════════════════════════════════════════
// ProcessHealthEntry struct tests
// ═══════════════════════════════════════════════════════════

TEST(ProcessHealthEntry, DefaultValues) {
    ProcessHealthEntry entry{};
    EXPECT_FALSE(entry.alive);
    EXPECT_EQ(entry.last_seen_ns, 0u);
    EXPECT_EQ(entry.name[0], '\0');
}

TEST(ProcessHealthEntry, SetName) {
    ProcessHealthEntry entry{};
    std::strncpy(entry.name, "video_capture", sizeof(entry.name) - 1);
    entry.alive        = true;
    entry.last_seen_ns = 123456789;

    EXPECT_STREQ(entry.name, "video_capture");
    EXPECT_TRUE(entry.alive);
    EXPECT_EQ(entry.last_seen_ns, 123456789u);
}

TEST(ProcessHealthEntry, NameTruncation) {
    ProcessHealthEntry entry{};
    // Name longer than 31 chars should be truncated
    const std::string long_name(40, 'x');
    std::strncpy(entry.name, long_name.c_str(), sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    EXPECT_EQ(std::strlen(entry.name), 31u);
}

TEST(ProcessHealthEntry, MaxTrackedProcesses) {
    EXPECT_GE(kMaxTrackedProcesses, 8u);
}

// ═══════════════════════════════════════════════════════════
// SystemHealth extension tests
// ═══════════════════════════════════════════════════════════

TEST(SystemHealth, HasProcessHealthFields) {
    SystemHealth health{};
    EXPECT_EQ(health.num_processes, 0u);
    EXPECT_FALSE(health.critical_failure);
}

TEST(SystemHealth, PopulateProcessEntries) {
    SystemHealth health{};

    // Simulate populating process entries
    const std::vector<std::string> names = {"video_capture",   "perception", "slam_vio_nav",
                                            "mission_planner", "comms",      "payload_manager",
                                            "system_monitor"};

    for (const auto& name : names) {
        auto& entry = health.processes[health.num_processes];
        std::strncpy(entry.name, name.c_str(), sizeof(entry.name) - 1);
        entry.alive        = true;
        entry.last_seen_ns = 100000;
        health.num_processes++;
    }

    EXPECT_EQ(health.num_processes, 7u);
    EXPECT_STREQ(health.processes[0].name, "video_capture");
    EXPECT_STREQ(health.processes[6].name, "system_monitor");
    EXPECT_TRUE(health.processes[4].alive);
}

TEST(SystemHealth, CriticalFailureFlag) {
    SystemHealth health{};
    health.critical_failure = true;
    EXPECT_TRUE(health.critical_failure);
}

TEST(SystemHealth, TriviallyCopyable) {
    EXPECT_TRUE(std::is_trivially_copyable_v<SystemHealth>);
}

TEST(SystemHealth, ProcessHealthEntryTriviallyCopyable) {
    EXPECT_TRUE(std::is_trivially_copyable_v<ProcessHealthEntry>);
}

// ═══════════════════════════════════════════════════════════
// Zenoh-dependent tests (only run when Zenoh is available)
// ═══════════════════════════════════════════════════════════


/// Poll predicate every 10ms, return true if it succeeds within timeout.
template<typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

TEST(LivelinessToken, DeclareAndUndeclare) {
    LivelinessToken token("test_process");
    EXPECT_TRUE(token.is_valid());
    EXPECT_EQ(token.key_expr(), "drone/alive/test_process");
}

TEST(LivelinessToken, KeyExprFormat) {
    LivelinessToken token("comms");
    EXPECT_EQ(token.key_expr(), "drone/alive/comms");
    EXPECT_TRUE(token.is_valid());
}

TEST(LivelinessMonitor, DetectsAliveToken) {
    // Create monitor first, then token
    std::vector<std::string> alive_names;
    std::mutex               mtx;

    LivelinessMonitor monitor(
        [&](const std::string& name) {
            std::lock_guard<std::mutex> lock(mtx);
            alive_names.push_back(name);
        },
        [](const std::string&) {});

    // Give monitor time to set up
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Declare token — monitor should receive PUT
    LivelinessToken token("test_detect_alive");

    // Poll until callback fires
    ASSERT_TRUE(wait_for([&] {
        std::lock_guard<std::mutex> lock(mtx);
        return std::find(alive_names.begin(), alive_names.end(), "test_detect_alive") !=
               alive_names.end();
    })) << "Monitor should have detected 'test_detect_alive' as alive";
}

TEST(LivelinessMonitor, DetectsTokenDeath) {
    std::vector<std::string> dead_names;
    std::mutex               mtx;

    LivelinessMonitor monitor([](const std::string&) {},
                              [&](const std::string& name) {
                                  std::lock_guard<std::mutex> lock(mtx);
                                  dead_names.push_back(name);
                              });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Create and destroy token in inner scope
    {
        LivelinessToken token("test_death");
        EXPECT_TRUE(token.is_valid());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // Token destroyed here — monitor should receive DELETE

    ASSERT_TRUE(wait_for([&] {
        std::lock_guard<std::mutex> lock(mtx);
        return std::find(dead_names.begin(), dead_names.end(), "test_death") != dead_names.end();
    })) << "Monitor should have detected 'test_death' as died";
}

TEST(LivelinessMonitor, MultipleTokens) {
    std::vector<std::string> alive_names;
    std::mutex               mtx;

    LivelinessMonitor monitor(
        [&](const std::string& name) {
            std::lock_guard<std::mutex> lock(mtx);
            alive_names.push_back(name);
        },
        [](const std::string&) {});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LivelinessToken t1("proc_a");
    LivelinessToken t2("proc_b");
    LivelinessToken t3("proc_c");

    ASSERT_TRUE(wait_for([&] {
        std::lock_guard<std::mutex> lock(mtx);
        return alive_names.size() >= 3u;
    }));

    std::lock_guard<std::mutex> lock(mtx);
    for (const auto& name : {"proc_a", "proc_b", "proc_c"}) {
        auto it = std::find(alive_names.begin(), alive_names.end(), name);
        EXPECT_NE(it, alive_names.end()) << "Missing: " << name;
    }
}

TEST(LivelinessMonitor, GetAliveProcesses) {
    LivelinessMonitor monitor([](const std::string&) {}, [](const std::string&) {});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LivelinessToken t1("get_alive_a");
    LivelinessToken t2("get_alive_b");

    ASSERT_TRUE(wait_for([&] {
        auto alive = monitor.get_alive_processes();
        return std::find(alive.begin(), alive.end(), "get_alive_a") != alive.end() &&
               std::find(alive.begin(), alive.end(), "get_alive_b") != alive.end();
    }));
}

TEST(LivelinessMonitor, IsAlive) {
    LivelinessMonitor monitor([](const std::string&) {}, [](const std::string&) {});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LivelinessToken token("is_alive_test");

    ASSERT_TRUE(wait_for([&] { return monitor.is_alive("is_alive_test"); }));
    EXPECT_FALSE(monitor.is_alive("nonexistent_process"));
}

TEST(LivelinessMonitor, TokenDropRemovesFromAlive) {
    LivelinessMonitor monitor([](const std::string&) {}, [](const std::string&) {});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        LivelinessToken token("drop_test");
        ASSERT_TRUE(wait_for([&] { return monitor.is_alive("drop_test"); }));
    }
    // Token destroyed

    ASSERT_TRUE(wait_for([&] { return !monitor.is_alive("drop_test"); }));
}


TEST(LivelinessToken, TokenIsValid) {
    LivelinessToken token("token_valid_test");
    EXPECT_TRUE(token.is_valid());
}

TEST(LivelinessMonitor, MonitorDetectsNoUnknownProcesses) {
    LivelinessMonitor monitor([](const std::string&) {}, [](const std::string&) {});
    EXPECT_FALSE(monitor.is_alive("nonexistent_process"));
}
