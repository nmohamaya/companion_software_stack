// tests/test_cosys_fc_link.cpp
// Tests for CosysFCLink — SimpleFlight via AirSim RPC backend (Issue #490).
//
// These tests are compiled only when HAVE_COSYS_AIRSIM is defined by CMake.
// All tests must work WITHOUT a live AirSim RPC server reachable on
// 127.0.0.1:41451 — they exercise construction, pre-open queries, and
// failure paths (open/send_arm/send_mode/send_trajectory/send_takeoff all
// return false when disconnected).
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_fc_link.h"
#include "hal/cosys_rpc_client.h"
#include "util/config.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace {

/// Build a CosysFCLink wired to an unconnected CosysRpcClient so that all
/// command paths can be exercised without a live AirSim server.
std::unique_ptr<drone::hal::CosysFCLink> make_unconnected_fc_link() {
    drone::Config cfg;  // empty — defaults apply (vehicle_name_ = "Drone0")
    auto          client = std::make_shared<drone::hal::CosysRpcClient>("127.0.0.1", 41451);
    // Do NOT call client->connect() — we want the client to report is_connected()==false.
    return std::make_unique<drone::hal::CosysFCLink>(client, cfg, std::string{"comms.mavlink"});
}

}  // namespace

// ─────────────────────────────────────────────────────────────
// Construction / identity
// ─────────────────────────────────────────────────────────────

TEST(CosysFCLinkTest, Construction) {
    // Must not crash even when the RPC client has never connected.
    auto fc = make_unconnected_fc_link();
    ASSERT_NE(fc, nullptr);
}

TEST(CosysFCLinkTest, NameIsCosysFCLink) {
    auto fc = make_unconnected_fc_link();
    EXPECT_EQ(fc->name(), "CosysFCLink");
}

// ─────────────────────────────────────────────────────────────
// Pre-open state
// ─────────────────────────────────────────────────────────────

TEST(CosysFCLinkTest, InitialStateIsEmpty) {
    auto fc    = make_unconnected_fc_link();
    auto state = fc->receive_state();
    EXPECT_EQ(state.timestamp_ns, 0u);
    EXPECT_FLOAT_EQ(state.altitude_rel, 0.0f);
    EXPECT_FLOAT_EQ(state.ground_speed, 0.0f);
    EXPECT_FALSE(state.armed);
    EXPECT_EQ(state.flight_mode, 0u);
    EXPECT_EQ(state.satellites, 0u);
    EXPECT_FLOAT_EQ(state.battery_voltage, 0.0f);
}

TEST(CosysFCLinkTest, IsNotConnectedInitially) {
    auto fc = make_unconnected_fc_link();
    EXPECT_FALSE(fc->is_connected());
}

// ─────────────────────────────────────────────────────────────
// Open failure path (no live AirSim server)
// ─────────────────────────────────────────────────────────────

TEST(CosysFCLinkTest, OpenFailsGracefullyWithoutServer) {
    auto fc = make_unconnected_fc_link();
    // port / baud are ignored by this backend; open should refuse because
    // the shared RPC client has never successfully connected.
    EXPECT_FALSE(fc->open("", 0));
    EXPECT_FALSE(fc->is_connected());
}

// ─────────────────────────────────────────────────────────────
// Command-path failure when disconnected
// ─────────────────────────────────────────────────────────────

TEST(CosysFCLinkTest, SendCommandsFailWhenDisconnected) {
    auto fc = make_unconnected_fc_link();
    // None of the command methods should succeed or crash when the
    // underlying RPC client is disconnected.
    EXPECT_FALSE(fc->send_arm(true));
    EXPECT_FALSE(fc->send_arm(false));
    EXPECT_FALSE(fc->send_mode(0));  // STAB
    EXPECT_FALSE(fc->send_mode(1));  // GUIDED
    EXPECT_FALSE(fc->send_mode(2));  // AUTO/land
    EXPECT_FALSE(fc->send_mode(3));  // RTL
    EXPECT_FALSE(fc->send_takeoff(5.0f));
    EXPECT_FALSE(fc->send_trajectory(1.0f, 0.5f, 0.2f, 0.0f));
}

TEST(CosysFCLinkTest, UnknownModeRejected) {
    auto fc = make_unconnected_fc_link();
    // Modes 4–255 are not mapped by CosysFCLink::send_mode and must be
    // rejected without attempting any RPC.
    EXPECT_FALSE(fc->send_mode(4));
    EXPECT_FALSE(fc->send_mode(255));
}

// ─────────────────────────────────────────────────────────────
// Close is idempotent / safe on never-opened link
// ─────────────────────────────────────────────────────────────

TEST(CosysFCLinkTest, CloseIsSafeWhenNeverOpened) {
    auto fc = make_unconnected_fc_link();
    fc->close();  // must not crash / deadlock
    fc->close();  // second call is a no-op
    EXPECT_FALSE(fc->is_connected());
}

#else  // !HAVE_COSYS_AIRSIM

// When the AirSim SDK is not available we still register a GTEST_SKIP()
// stub so this translation unit contributes one test to the aggregate count
// and the "Total Tests" baseline stays stable across build configurations.
// Without this, a build that disables HAVE_COSYS_AIRSIM would silently drop
// 8 tests and mask regressions in the rest of the suite.
#include <gtest/gtest.h>
TEST(CosysFCLinkTest, SkippedNoCosysSDK) {
    GTEST_SKIP() << "HAVE_COSYS_AIRSIM not defined — Cosys SDK unavailable";
}

#endif  // HAVE_COSYS_AIRSIM
