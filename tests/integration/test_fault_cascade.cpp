// tests/integration/test_fault_cascade.cpp
// Integration tests: fault-related IPC wire format and message flow.
//
// Tests that fault-related IPC messages can be published and received
// correctly through the shared Zenoh bus:
//   - FC disconnect messages (FCState → MissionStatus)
//   - System health messages (SystemHealth with fault flags)
//   - Battery status messages
//
// NOTE: These tests validate IPC wire format and pub/sub plumbing for fault
// messages — they do NOT instantiate real fault detection logic (FaultManager,
// MissionFSM).  A bug in fault escalation code would not be caught here.
// For actual fault logic coverage, see test_fault_manager.cpp and
// test_fault_response_executor.cpp.
//
// See: Issue #292 (Epic #284 — Platform Modularity)
#include "integration/integration_harness.h"
#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

using drone::test::IntegrationTestHarness;
namespace ipc = drone::ipc;

// ═══════════════════════════════════════════════════════════
// Fixture
// ═══════════════════════════════════════════════════════════
class FaultCascadeTest : public ::testing::Test {
protected:
    IntegrationTestHarness harness_;

    static void wait_delivery() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
};

// ─── FC disconnect → MissionStatus reflects fault ────────

TEST_F(FaultCascadeTest, FCDisconnectPropagatesThroughBus) {
    // Simulate the fault cascade:
    //   P5 publishes FCState{connected=false} on /fc_state
    //   P4 reads it, detects FC link loss, publishes MissionStatus with fault
    //   P7 reads the MissionStatus and sees the fault

    auto fc_topic      = IntegrationTestHarness::unique_topic("/fc_state");
    auto mission_topic = IntegrationTestHarness::unique_topic("/mission_status");

    auto fc_pub      = harness_.bus().advertise<ipc::FCState>(fc_topic);
    auto fc_sub      = harness_.bus().subscribe<ipc::FCState>(fc_topic);
    auto mission_pub = harness_.bus().advertise<ipc::MissionStatus>(mission_topic);
    auto mission_sub = harness_.bus().subscribe<ipc::MissionStatus>(mission_topic);

    // Step 1: P5 publishes FC disconnected state
    ipc::FCState fc_disconnected{};
    fc_disconnected.timestamp_ns      = harness_.clock().now_ns();
    fc_disconnected.battery_voltage   = 22.0f;
    fc_disconnected.battery_remaining = 80.0f;
    fc_disconnected.roll              = 0.0f;
    fc_disconnected.pitch             = 0.0f;
    fc_disconnected.yaw               = 0.0f;
    fc_disconnected.vx                = 0.0f;
    fc_disconnected.vy                = 0.0f;
    fc_disconnected.vz                = 0.0f;
    fc_disconnected.connected         = false;  // disconnected
    fc_disconnected.armed             = true;
    ASSERT_TRUE(fc_disconnected.validate());

    fc_pub->publish(fc_disconnected);
    wait_delivery();

    // Step 2: "P4 logic" reads the FC state and detects disconnection
    ipc::FCState received_fc{};
    ASSERT_TRUE(fc_sub->receive(received_fc));
    EXPECT_FALSE(received_fc.connected);

    // P4 sets fault flags and transitions to LOITER
    ipc::MissionStatus status{};
    status.timestamp_ns     = harness_.clock().now_ns();
    status.correlation_id   = 0;
    status.state            = ipc::MissionState::LOITER;
    status.current_waypoint = 3;
    status.total_waypoints  = 10;
    status.progress_percent = 30.0f;
    status.target_x         = 0.0f;
    status.target_y         = 0.0f;
    status.target_z         = 0.0f;
    status.battery_percent  = received_fc.battery_remaining;
    status.mission_active   = true;
    status.active_faults    = ipc::to_uint(ipc::FaultType::FAULT_FC_LINK_LOST);
    status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::LOITER);

    mission_pub->publish(status);
    wait_delivery();

    // Step 3: P7 reads the mission status and sees the fault
    ipc::MissionStatus received_status{};
    ASSERT_TRUE(mission_sub->receive(received_status));
    EXPECT_EQ(received_status.state, ipc::MissionState::LOITER);
    EXPECT_NE(received_status.active_faults & ipc::FaultType::FAULT_FC_LINK_LOST, 0u);
    EXPECT_EQ(received_status.fault_action, static_cast<uint8_t>(ipc::FaultAction::LOITER));
}

// ─── FC disconnect escalation: LOITER → RTL ─────────────

TEST_F(FaultCascadeTest, FCDisconnectEscalatesToRTL) {
    // After LOITER timeout, P4 escalates to RTL and issues an RTL command.

    auto mission_topic = IntegrationTestHarness::unique_topic("/mission_status");
    auto cmd_topic     = IntegrationTestHarness::unique_topic("/fc_commands");

    auto mission_pub = harness_.bus().advertise<ipc::MissionStatus>(mission_topic);
    auto mission_sub = harness_.bus().subscribe<ipc::MissionStatus>(mission_topic);
    auto cmd_pub     = harness_.bus().advertise<ipc::FCCommand>(cmd_topic);
    auto cmd_sub     = harness_.bus().subscribe<ipc::FCCommand>(cmd_topic);

    // Step 1: P4 is in LOITER due to FC link loss
    ipc::MissionStatus loiter_status{};
    loiter_status.timestamp_ns     = harness_.clock().now_ns();
    loiter_status.state            = ipc::MissionState::LOITER;
    loiter_status.active_faults    = ipc::to_uint(ipc::FaultType::FAULT_FC_LINK_LOST);
    loiter_status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::LOITER);
    loiter_status.battery_percent  = 70.0f;
    loiter_status.progress_percent = 0.0f;
    loiter_status.target_x         = 0.0f;
    loiter_status.target_y         = 0.0f;
    loiter_status.target_z         = 0.0f;
    loiter_status.mission_active   = true;

    mission_pub->publish(loiter_status);

    // Step 2: Advance mock clock past escalation timeout (e.g. 30 seconds)
    harness_.advance_time_ms(30'000);

    // Step 3: P4 escalates to RTL and publishes updated status + command
    ipc::MissionStatus rtl_status{};
    rtl_status.timestamp_ns     = harness_.clock().now_ns();
    rtl_status.state            = ipc::MissionState::RTL;
    rtl_status.active_faults    = ipc::to_uint(ipc::FaultType::FAULT_FC_LINK_LOST);
    rtl_status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::RTL);
    rtl_status.battery_percent  = 69.0f;
    rtl_status.progress_percent = 0.0f;
    rtl_status.target_x         = 0.0f;
    rtl_status.target_y         = 0.0f;
    rtl_status.target_z         = 0.0f;
    rtl_status.mission_active   = false;

    mission_pub->publish(rtl_status);

    ipc::FCCommand rtl_cmd{};
    rtl_cmd.timestamp_ns   = harness_.clock().now_ns();
    rtl_cmd.correlation_id = 300;
    rtl_cmd.command        = ipc::FCCommandType::RTL;
    rtl_cmd.param1         = 0.0f;
    rtl_cmd.sequence_id    = 1;
    rtl_cmd.valid          = true;

    cmd_pub->publish(rtl_cmd);
    wait_delivery();

    // Verify: P5 sees RTL command
    ipc::FCCommand received_cmd{};
    ASSERT_TRUE(cmd_sub->receive(received_cmd));
    EXPECT_EQ(received_cmd.command, ipc::FCCommandType::RTL);
    EXPECT_TRUE(received_cmd.valid);

    // Verify: P7 sees escalated mission status
    ipc::MissionStatus received_status{};
    ASSERT_TRUE(mission_sub->receive(received_status));
    EXPECT_EQ(received_status.state, ipc::MissionState::RTL);
    EXPECT_EQ(received_status.fault_action, static_cast<uint8_t>(ipc::FaultAction::RTL));
}

// ─── Battery low → warning → RTL escalation ─────────────

TEST_F(FaultCascadeTest, BatteryLowEscalationChain) {
    // P5 publishes FCState with low battery → P4 warns → battery drops more → RTL

    auto fc_topic      = IntegrationTestHarness::unique_topic("/fc_state");
    auto mission_topic = IntegrationTestHarness::unique_topic("/mission_status");

    auto fc_pub      = harness_.bus().advertise<ipc::FCState>(fc_topic);
    auto fc_sub      = harness_.bus().subscribe<ipc::FCState>(fc_topic);
    auto mission_pub = harness_.bus().advertise<ipc::MissionStatus>(mission_topic);
    auto mission_sub = harness_.bus().subscribe<ipc::MissionStatus>(mission_topic);

    // Step 1: Battery at warning level (25%)
    ipc::FCState fc_low{};
    fc_low.timestamp_ns      = harness_.clock().now_ns();
    fc_low.battery_voltage   = 21.0f;
    fc_low.battery_remaining = 25.0f;
    fc_low.roll              = 0.0f;
    fc_low.pitch             = 0.0f;
    fc_low.yaw               = 0.0f;
    fc_low.vx                = 0.0f;
    fc_low.vy                = 0.0f;
    fc_low.vz                = 0.0f;
    fc_low.connected         = true;
    fc_low.armed             = true;

    fc_pub->publish(fc_low);
    wait_delivery();

    ipc::FCState received_fc{};
    ASSERT_TRUE(fc_sub->receive(received_fc));
    EXPECT_FLOAT_EQ(received_fc.battery_remaining, 25.0f);

    // P4 issues WARN
    ipc::MissionStatus warn_status{};
    warn_status.timestamp_ns     = harness_.clock().now_ns();
    warn_status.state            = ipc::MissionState::NAVIGATE;
    warn_status.active_faults    = ipc::to_uint(ipc::FaultType::FAULT_BATTERY_LOW);
    warn_status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::WARN);
    warn_status.battery_percent  = 25.0f;
    warn_status.progress_percent = 50.0f;
    warn_status.target_x         = 0.0f;
    warn_status.target_y         = 0.0f;
    warn_status.target_z         = 0.0f;
    warn_status.mission_active   = true;

    mission_pub->publish(warn_status);
    wait_delivery();

    ipc::MissionStatus received_warn{};
    ASSERT_TRUE(mission_sub->receive(received_warn));
    EXPECT_EQ(received_warn.fault_action, static_cast<uint8_t>(ipc::FaultAction::WARN));

    // Step 2: Battery drops to RTL level (15%)
    harness_.advance_time_ms(60'000);

    ipc::FCState fc_critical{};
    fc_critical.timestamp_ns      = harness_.clock().now_ns();
    fc_critical.battery_voltage   = 20.0f;
    fc_critical.battery_remaining = 15.0f;
    fc_critical.roll              = 0.0f;
    fc_critical.pitch             = 0.0f;
    fc_critical.yaw               = 0.0f;
    fc_critical.vx                = 0.0f;
    fc_critical.vy                = 0.0f;
    fc_critical.vz                = 0.0f;
    fc_critical.connected         = true;
    fc_critical.armed             = true;

    fc_pub->publish(fc_critical);
    wait_delivery();

    // P4 escalates to RTL
    ipc::MissionStatus rtl_status{};
    rtl_status.timestamp_ns  = harness_.clock().now_ns();
    rtl_status.state         = ipc::MissionState::RTL;
    rtl_status.active_faults = ipc::FaultType::FAULT_BATTERY_LOW |
                               ipc::FaultType::FAULT_BATTERY_RTL;
    rtl_status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::RTL);
    rtl_status.battery_percent  = 15.0f;
    rtl_status.progress_percent = 50.0f;
    rtl_status.target_x         = 0.0f;
    rtl_status.target_y         = 0.0f;
    rtl_status.target_z         = 0.0f;
    rtl_status.mission_active   = false;

    mission_pub->publish(rtl_status);
    wait_delivery();

    ipc::MissionStatus received_rtl{};
    ASSERT_TRUE(mission_sub->receive(received_rtl));
    EXPECT_EQ(received_rtl.state, ipc::MissionState::RTL);
    EXPECT_NE(received_rtl.active_faults & ipc::FaultType::FAULT_BATTERY_RTL, 0u);
}

// ─── Perception death propagates through system health ────

TEST_F(FaultCascadeTest, PerceptionDeathPropagation) {
    // P7 detects P2 death → publishes SystemHealth with critical_failure
    // P4 reads SystemHealth and sees perception_dead fault

    auto health_topic  = IntegrationTestHarness::unique_topic("/system_health");
    auto mission_topic = IntegrationTestHarness::unique_topic("/mission_status");

    auto health_pub  = harness_.bus().advertise<ipc::SystemHealth>(health_topic);
    auto health_sub  = harness_.bus().subscribe<ipc::SystemHealth>(health_topic);
    auto mission_pub = harness_.bus().advertise<ipc::MissionStatus>(mission_topic);
    auto mission_sub = harness_.bus().subscribe<ipc::MissionStatus>(mission_topic);

    // Step 1: P7 publishes system health with perception dead
    ipc::SystemHealth health{};
    health.timestamp_ns         = harness_.clock().now_ns();
    health.cpu_usage_percent    = 30.0f;
    health.memory_usage_percent = 40.0f;
    health.disk_usage_percent   = 20.0f;
    health.max_temp_c           = 50.0f;
    health.gpu_temp_c           = 45.0f;
    health.cpu_temp_c           = 48.0f;
    health.total_healthy        = 5;
    health.total_degraded       = 0;
    health.total_dead           = 1;
    health.power_watts          = 10.0f;
    health.thermal_zone         = 0;
    health.stack_status         = 2;  // CRITICAL
    health.total_restarts       = 1;
    health.critical_failure     = true;
    health.num_processes        = 1;
    std::strncpy(health.processes[0].name, "perception", sizeof(health.processes[0].name) - 1);
    health.processes[0].alive        = false;
    health.processes[0].last_seen_ns = harness_.clock().now_ns() - 5'000'000'000ULL;
    ASSERT_TRUE(health.validate());

    health_pub->publish(health);
    wait_delivery();

    // Step 2: P4 reads system health
    ipc::SystemHealth received_health{};
    ASSERT_TRUE(health_sub->receive(received_health));
    EXPECT_TRUE(received_health.critical_failure);
    EXPECT_EQ(received_health.total_dead, 1u);

    // Step 3: P4 publishes mission status with perception dead fault
    ipc::MissionStatus status{};
    status.timestamp_ns     = harness_.clock().now_ns();
    status.state            = ipc::MissionState::LOITER;
    status.active_faults    = ipc::to_uint(ipc::FaultType::FAULT_PERCEPTION_DEAD);
    status.fault_action     = static_cast<uint8_t>(ipc::FaultAction::LOITER);
    status.battery_percent  = 80.0f;
    status.progress_percent = 0.0f;
    status.target_x         = 0.0f;
    status.target_y         = 0.0f;
    status.target_z         = 0.0f;
    status.mission_active   = true;

    mission_pub->publish(status);
    wait_delivery();

    ipc::MissionStatus received_status{};
    ASSERT_TRUE(mission_sub->receive(received_status));
    EXPECT_EQ(received_status.state, ipc::MissionState::LOITER);
    EXPECT_NE(received_status.active_faults & ipc::FaultType::FAULT_PERCEPTION_DEAD, 0u);
}

// ─── Multiple simultaneous faults ────────────────────────

TEST_F(FaultCascadeTest, MultipleFaultsAccumulate) {
    // Verify that multiple fault conditions can be combined in a single MissionStatus

    auto mission_topic = IntegrationTestHarness::unique_topic("/mission_status");
    auto mission_pub   = harness_.bus().advertise<ipc::MissionStatus>(mission_topic);
    auto mission_sub   = harness_.bus().subscribe<ipc::MissionStatus>(mission_topic);

    ipc::MissionStatus multi_fault{};
    multi_fault.timestamp_ns  = harness_.clock().now_ns();
    multi_fault.state         = ipc::MissionState::EMERGENCY;
    multi_fault.active_faults = ipc::to_uint(ipc::FaultType::FAULT_FC_LINK_LOST) |
                                ipc::to_uint(ipc::FaultType::FAULT_BATTERY_CRITICAL) |
                                ipc::to_uint(ipc::FaultType::FAULT_THERMAL_CRITICAL);
    multi_fault.fault_action     = static_cast<uint8_t>(ipc::FaultAction::EMERGENCY_LAND);
    multi_fault.battery_percent  = 5.0f;
    multi_fault.progress_percent = 0.0f;
    multi_fault.target_x         = 0.0f;
    multi_fault.target_y         = 0.0f;
    multi_fault.target_z         = 0.0f;
    multi_fault.mission_active   = false;

    mission_pub->publish(multi_fault);
    wait_delivery();

    ipc::MissionStatus received{};
    ASSERT_TRUE(mission_sub->receive(received));
    EXPECT_EQ(received.state, ipc::MissionState::EMERGENCY);
    EXPECT_EQ(received.fault_action, static_cast<uint8_t>(ipc::FaultAction::EMERGENCY_LAND));

    // Check all three fault flags are set
    EXPECT_NE(received.active_faults & ipc::FaultType::FAULT_FC_LINK_LOST, 0u);
    EXPECT_NE(received.active_faults & ipc::FaultType::FAULT_BATTERY_CRITICAL, 0u);
    EXPECT_NE(received.active_faults & ipc::FaultType::FAULT_THERMAL_CRITICAL, 0u);

    // Verify human-readable string
    auto flags_str = ipc::fault_flags_string(received.active_faults);
    EXPECT_NE(flags_str.find("FAULT_FC_LINK_LOST"), std::string::npos);
    EXPECT_NE(flags_str.find("FAULT_BATTERY_CRITICAL"), std::string::npos);
    EXPECT_NE(flags_str.find("FAULT_THERMAL_CRITICAL"), std::string::npos);
}

// ─── Harness logger captures fault-related messages ──────

TEST_F(FaultCascadeTest, FaultLogsCaptured) {
    // Verify the harness captures log messages during fault scenarios.
    DRONE_LOG_WARN("FC link lost — entering LOITER");
    DRONE_LOG_ERROR("Escalation timeout — transitioning to RTL");

    ASSERT_NE(harness_.logger(), nullptr);
    EXPECT_TRUE(harness_.logger()->contains("FC link lost"));
    EXPECT_TRUE(harness_.logger()->contains("Escalation timeout"));
    EXPECT_EQ(harness_.logger()->count_at(drone::log::Level::Warn), 1u);
    EXPECT_EQ(harness_.logger()->count_at(drone::log::Level::Error), 1u);
}
