// tests/test_gcs_command_handler.cpp
// Unit tests for GCSCommandHandler (Issue #154).
#include "planner/gcs_command_handler.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::planner;
using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Mock IPC types for testing
// ═══════════════════════════════════════════════════════════
namespace {

template<typename T>
class MockSubscriber : public ISubscriber<T> {
public:
    void set(const T& msg) {
        msg_       = msg;
        has_data_  = true;
        connected_ = true;
    }

    [[nodiscard]] bool receive(T& out, uint64_t* /*ts*/ = nullptr) const override {
        if (!has_data_) return false;
        out = msg_;
        return true;
    }
    [[nodiscard]] bool               is_connected() const override { return connected_; }
    [[nodiscard]] const std::string& topic_name() const override { return topic_; }

    bool connected_ = true;

private:
    T           msg_{};
    bool        has_data_ = false;
    std::string topic_    = "mock";
};

template<typename T>
class MockPublisher : public IPublisher<T> {
public:
    void                                publish(const T& msg) override { messages_.push_back(msg); }
    [[nodiscard]] bool                  is_ready() const override { return true; }
    [[nodiscard]] const std::string&    topic_name() const override { return topic_; }
    [[nodiscard]] const std::vector<T>& messages() const { return messages_; }
    void                                clear() { messages_.clear(); }

private:
    std::vector<T> messages_;
    std::string    topic_ = "mock";
};

/// Track FC commands sent via callback.
struct FCCallRecord {
    FCCommandType cmd;
    float         param;
};

}  // namespace

class GCSCommandHandlerTest : public ::testing::Test {
protected:
    MockSubscriber<GCSCommand>    gcs_sub;
    MockSubscriber<MissionUpload> upload_sub;
    MockPublisher<TrajectoryCmd>  traj_pub;
    MissionFSM                    fsm;
    SharedFlightState             flight_state;
    std::vector<FCCallRecord>     fc_calls;
    GCSCommandHandler             handler;

    FCSendFn send_fc = [this](FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };

    void SetUp() override {
        fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
        fsm.on_arm();
    }

    drone::util::FrameDiagnostics make_diag() { return drone::util::FrameDiagnostics(0); }
};

// ═══════════════════════════════════════════════════════════
// RTL dispatch
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, RTLCommandSendsFCAndTransitions) {
    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0xABCD;
    cmd.command        = GCSCommandType::RTL;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::RTL);
    EXPECT_EQ(fsm.state(), MissionState::RTL);
    EXPECT_TRUE(flight_state.nav_was_armed);
    EXPECT_EQ(handler.active_correlation_id(), 0xABCDu);
    EXPECT_GE(traj_pub.messages().size(), 1u);
    EXPECT_FALSE(traj_pub.messages().back().valid);  // stop trajectory
}

// ═══════════════════════════════════════════════════════════
// LAND dispatch
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, LANDCommandSendsFCAndTransitions) {
    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0x1234;
    cmd.command        = GCSCommandType::LAND;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::LAND);
    EXPECT_EQ(fsm.state(), MissionState::LAND);
    EXPECT_TRUE(flight_state.land_sent);
}

// ═══════════════════════════════════════════════════════════
// Deduplication — same timestamp ignored
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, DuplicateTimestampIgnored) {
    GCSCommand cmd{};
    cmd.timestamp_ns = 100;
    cmd.command      = GCSCommandType::RTL;
    cmd.valid        = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);
    fc_calls.clear();

    // Same timestamp — should be deduplicated
    auto diag2 = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag2);
    EXPECT_TRUE(fc_calls.empty());
}

// ═══════════════════════════════════════════════════════════
// Correlation ID propagation
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, CorrelationIDPersists) {
    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0xCAFE;
    cmd.command        = GCSCommandType::LAND;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    EXPECT_EQ(handler.active_correlation_id(), 0xCAFEu);
}

// ═══════════════════════════════════════════════════════════
// MISSION_UPLOAD dispatch
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, MissionUploadLoadsWaypoints) {
    // Put FSM into NAVIGATE state first
    fsm.on_takeoff();
    fsm.on_navigate();

    // Set up upload data
    MissionUpload upload{};
    upload.timestamp_ns  = 200;
    upload.num_waypoints = 2;
    upload.waypoints[0]  = {1, 2, 3, 0, 2, 3, false};
    upload.waypoints[1]  = {4, 5, 6, 0, 2, 3, true};
    upload.valid         = true;
    upload_sub.set(upload);

    GCSCommand cmd{};
    cmd.timestamp_ns = 100;
    cmd.command      = GCSCommandType::MISSION_UPLOAD;
    cmd.valid        = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    // Mission should be loaded and FSM should still be in NAVIGATE
    EXPECT_EQ(fsm.total_waypoints(), 2u);
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// MISSION_PAUSE dispatch
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, PauseCommandTransitionsToLoiter) {
    // Put FSM into NAVIGATE state
    fsm.on_takeoff();
    fsm.on_navigate();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);

    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0xBEEF;
    cmd.command        = GCSCommandType::MISSION_PAUSE;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    EXPECT_EQ(fsm.state(), MissionState::LOITER);
    EXPECT_TRUE(fc_calls.empty());  // Pause does NOT send FC command, just stops trajectory
    EXPECT_GE(traj_pub.messages().size(), 1u);
    EXPECT_FALSE(traj_pub.messages().back().valid);  // stop trajectory
}

// ═══════════════════════════════════════════════════════════
// MISSION_START resumes from LOITER
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, StartCommandResumesFromLoiter) {
    // Put FSM into LOITER state (simulating a pause)
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_loiter();
    ASSERT_EQ(fsm.state(), MissionState::LOITER);

    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0xFACE;
    cmd.command        = GCSCommandType::MISSION_START;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// MISSION_START ignored when not in LOITER
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, StartCommandIgnoredWhenNotInLoiter) {
    // FSM in NAVIGATE — MISSION_START should be a no-op
    fsm.on_takeoff();
    fsm.on_navigate();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);

    GCSCommand cmd{};
    cmd.timestamp_ns = 100;
    cmd.command      = GCSCommandType::MISSION_START;
    cmd.valid        = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// MISSION_ABORT dispatch — triggers RTL
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, AbortCommandSendsRTLAndTransitions) {
    fsm.on_takeoff();
    fsm.on_navigate();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);

    GCSCommand cmd{};
    cmd.timestamp_ns   = 100;
    cmd.correlation_id = 0xDEAD;
    cmd.command        = GCSCommandType::MISSION_ABORT;
    cmd.valid          = true;
    gcs_sub.set(cmd);

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::RTL);
    EXPECT_EQ(fsm.state(), MissionState::RTL);
    EXPECT_TRUE(flight_state.nav_was_armed);
    EXPECT_GE(traj_pub.messages().size(), 1u);
    EXPECT_FALSE(traj_pub.messages().back().valid);  // stop trajectory
}

// ═══════════════════════════════════════════════════════════
// Disconnected subscriber — no processing
// ═══════════════════════════════════════════════════════════
TEST_F(GCSCommandHandlerTest, DisconnectedSubscriberIgnored) {
    GCSCommand cmd{};
    cmd.timestamp_ns = 100;
    cmd.command      = GCSCommandType::RTL;
    cmd.valid        = true;
    gcs_sub.set(cmd);
    gcs_sub.connected_ = false;  // disconnect AFTER set() (which sets connected=true)

    auto diag = make_diag();
    handler.process(gcs_sub, upload_sub, fsm, send_fc, traj_pub, flight_state, diag);
    EXPECT_TRUE(fc_calls.empty());
}
