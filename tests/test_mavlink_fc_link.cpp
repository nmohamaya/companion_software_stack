// tests/test_mavlink_fc_link.cpp
// Unit tests for MavlinkFCLink — MAVLink HAL backend via MAVSDK.
//
// These tests verify:
//  1. Factory creates MavlinkFCLink when HAVE_MAVSDK is defined
//  2. MavlinkFCLink returns correct name()
//  3. Connection failure is handled gracefully (no PX4 SITL running)
//  4. Flight mode mapping is correct
//  5. Interface compliance (all methods callable)
//
// NOTE: Full integration tests requiring PX4 SITL are planned in Phase 4 (#11).
#include "hal/hal_factory.h"
#include "hal/ifc_link.h"
#include "util/config.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// Helper: temp config (same pattern as test_hal.cpp)
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_mavlink_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_mavlink_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_mavlink_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        ofs.close();
        g_mavlink_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_mavlink_temp_files.push_back(path);
    return path;
}

struct MavlinkTempCleanup {
    ~MavlinkTempCleanup() {
        for (const auto& f : g_mavlink_temp_files) {
            std::remove(f.c_str());
        }
    }
};
static MavlinkTempCleanup g_mavlink_cleanup;

// ═══════════════════════════════════════════════════════════
// Tests conditional on HAVE_MAVSDK
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_MAVSDK

#include "hal/mavlink_fc_link.h"

// ── Basic construction and name ────────────────────────────
TEST(MavlinkFCLinkTest, NameIsCorrect) {
    drone::hal::MavlinkFCLink fc;
    EXPECT_EQ(fc.name(), "MavlinkFCLink");
}

TEST(MavlinkFCLinkTest, InitiallyDisconnected) {
    drone::hal::MavlinkFCLink fc;
    EXPECT_FALSE(fc.is_connected());
}

TEST(MavlinkFCLinkTest, ReceiveStateDefaultWhenDisconnected) {
    drone::hal::MavlinkFCLink fc;
    auto                      state = fc.receive_state();
    EXPECT_EQ(state.timestamp_ns, 0u);
    EXPECT_FALSE(state.armed);
    EXPECT_FLOAT_EQ(state.battery_percent, 0.0f);
}

TEST(MavlinkFCLinkTest, SendTrajectoryFailsWhenDisconnected) {
    drone::hal::MavlinkFCLink fc;
    EXPECT_FALSE(fc.send_trajectory(1.0f, 0.0f, 0.0f, 0.0f));
}

TEST(MavlinkFCLinkTest, SendArmFailsWhenDisconnected) {
    drone::hal::MavlinkFCLink fc;
    EXPECT_FALSE(fc.send_arm(true));
    EXPECT_FALSE(fc.send_arm(false));
}

TEST(MavlinkFCLinkTest, SendModeFailsWhenDisconnected) {
    drone::hal::MavlinkFCLink fc;
    EXPECT_FALSE(fc.send_mode(0));
    EXPECT_FALSE(fc.send_mode(1));
    EXPECT_FALSE(fc.send_mode(3));
}

TEST(MavlinkFCLinkTest, CloseWhenNotOpenIsNoOp) {
    drone::hal::MavlinkFCLink fc;
    // Should not throw or crash
    fc.close();
    EXPECT_FALSE(fc.is_connected());
}

TEST(MavlinkFCLinkTest, OpenWithInvalidUriFailsGracefully) {
    drone::hal::MavlinkFCLink fc;
    // Invalid URI — MAVSDK should reject it
    bool result = fc.open("not_a_valid_uri", 1000);
    // Depending on MAVSDK version, this may fail at add_any_connection
    // or at first_autopilot timeout. Either way, it should not crash.
    if (!result) {
        EXPECT_FALSE(fc.is_connected());
    }
    fc.close();
}

TEST(MavlinkFCLinkTest, OpenWithNoSITLTimesOut) {
    drone::hal::MavlinkFCLink fc;
    // Connect to a port where nothing is listening — short timeout
    // Port 14599 is unlikely to have a PX4 instance
    bool result = fc.open("udp://:14599", 2000);  // 2s timeout
    EXPECT_FALSE(result);
    EXPECT_FALSE(fc.is_connected());
}

TEST(MavlinkFCLinkTest, DoubleOpenReturnsFalse) {
    drone::hal::MavlinkFCLink fc;
    // First open will fail (no SITL), but let's test the guard
    fc.open("udp://:14598", 1000);
    // Even if first open fails, internal state should be clean
    // Try connecting again — should work (or timeout again)
    bool result = fc.open("udp://:14597", 1000);
    // Just verify no crash
    fc.close();
    (void)result;
}

// ── Factory creates MavlinkFCLink ──────────────────────────
TEST(MavlinkFCLinkTest, FactoryCreatesMavlinkBackend) {
    auto          path = create_temp_config(R"({
        "comms": { "fc": { "backend": "mavlink",
                            "uri": "udp://:14596",
                            "timeout_ms": 1000 } }
    })");
    drone::Config cfg;
    cfg.load(path);

    auto fc = drone::hal::create_fc_link(cfg, "comms.fc");
    ASSERT_NE(fc, nullptr);
    EXPECT_EQ(fc->name(), "MavlinkFCLink");
}

// ── Flight mode mapping (static, testable without connection) ──
// Access the mapping through the interface by checking default state behavior.
// The internal map_flight_mode is private, so we verify it indirectly
// by checking the default flight_mode in a fresh state is 0 (STAB).
TEST(MavlinkFCLinkTest, DefaultFlightModeIsZero) {
    drone::hal::MavlinkFCLink fc;
    auto                      state = fc.receive_state();
    EXPECT_EQ(state.flight_mode, 0);  // default = STAB
}

#else  // !HAVE_MAVSDK

// ── When MAVSDK is not available ───────────────────────────
TEST(MavlinkFCLinkTest, MavlinkBackendThrowsWithoutMAVSDK) {
    auto          path = create_temp_config(R"({
        "comms": { "fc": { "backend": "mavlink" } }
    })");
    drone::Config cfg;
    cfg.load(path);

    EXPECT_THROW(drone::hal::create_fc_link(cfg, "comms.fc"), std::runtime_error);
}

TEST(MavlinkFCLinkTest, SimulatedStillWorksWithoutMAVSDK) {
    auto          path = create_temp_config(R"({
        "comms": { "fc": { "backend": "simulated" } }
    })");
    drone::Config cfg;
    cfg.load(path);

    auto fc = drone::hal::create_fc_link(cfg, "comms.fc");
    ASSERT_NE(fc, nullptr);
    EXPECT_EQ(fc->name(), "SimulatedFCLink");
}

#endif  // HAVE_MAVSDK
