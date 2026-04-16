// tests/test_zenoh_ipc.cpp
// Unit tests for the Zenoh IPC backend and message bus factory.
//
// Five categories:
//   1. Topic mapping tests
//   2. Zenoh pub/sub round-trip tests
//   3. SHM provider zero-copy tests
//   4. Service channel tests
//   5. MessageBusFactory tests
#include "ipc/ipc_types.h"
#include "ipc/iservice_channel.h"
#include "ipc/message_bus_factory.h"
#include "ipc/zenoh_message_bus.h"
#include "ipc/zenoh_publisher.h"
#include "ipc/zenoh_service_client.h"
#include "ipc/zenoh_service_server.h"
#include "ipc/zenoh_session.h"
#include "ipc/zenoh_subscriber.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Category 1: Topic mapping tests (always compiled)
// ═══════════════════════════════════════════════════════════


TEST(ZenohTopicMapping, VideoMissionCam) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/drone_mission_cam"), "drone/video/frame");
}

TEST(ZenohTopicMapping, VideoStereoCam) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/drone_stereo_cam"), "drone/video/stereo_frame");
}

TEST(ZenohTopicMapping, DetectedObjects) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/detected_objects"), "drone/perception/detections");
}

TEST(ZenohTopicMapping, SlamPose) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/slam_pose"), "drone/slam/pose");
}

TEST(ZenohTopicMapping, MissionStatus) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/mission_status"), "drone/mission/status");
}

TEST(ZenohTopicMapping, TrajectoryCmd) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/trajectory_cmd"), "drone/mission/trajectory");
}

TEST(ZenohTopicMapping, PayloadCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/payload_commands"), "drone/mission/payload_command");
}

TEST(ZenohTopicMapping, FCCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/fc_commands"), "drone/comms/fc_command");
}

TEST(ZenohTopicMapping, FCState) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/fc_state"), "drone/comms/fc_state");
}

TEST(ZenohTopicMapping, GCSCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/gcs_commands"), "drone/comms/gcs_command");
}

TEST(ZenohTopicMapping, PayloadStatus) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/payload_status"), "drone/payload/status");
}

TEST(ZenohTopicMapping, SystemHealth) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/system_health"), "drone/monitor/health");
}

TEST(ZenohTopicMapping, AllTwelveSegmentsMapped) {
    // Verify every shm_names constant has a mapping
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::VIDEO_MISSION_CAM), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::VIDEO_STEREO_CAM), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::DETECTED_OBJECTS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::SLAM_POSE), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::MISSION_STATUS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::TRAJECTORY_CMD), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::PAYLOAD_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::FC_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::FC_STATE), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::GCS_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::PAYLOAD_STATUS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(topics::SYSTEM_HEALTH), "");
}

TEST(ZenohTopicMapping, PassthroughZenohKeyExpr) {
    // Already a Zenoh key expression — no leading '/'
    EXPECT_EQ(ZenohMessageBus::to_key_expr("drone/test/data"), "drone/test/data");
}

TEST(ZenohTopicMapping, FallbackUnmappedShmName) {
    // Unknown SHM name — fallback: strip '/' and replace '_' with '/'
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/some_unknown_topic"), "some/unknown/topic");
}


// ═══════════════════════════════════════════════════════════
// Category 1b: MessageBusFactory tests (always compiled)
// ═══════════════════════════════════════════════════════════

TEST(MessageBusFactory, DefaultCreatesZenohBus) {
    auto bus = create_message_bus();
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(MessageBusFactory, ExplicitZenohCreatesZenohBus) {
    auto bus = create_message_bus("zenoh");
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(MessageBusFactory, ShmFallsBackToZenoh) {
    // "shm" backend removed — should fall back to Zenoh with error log
    auto bus = create_message_bus("shm");
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(MessageBusFactory, BusAdvertiseViaMessageBus) {
    auto bus = create_message_bus("zenoh");
    auto pub = bus.advertise<Pose>("/test_factory_pub_" + std::to_string(getpid()));
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
}

TEST(MessageBusFactory, BusSubscribeViaMessageBus) {
    auto bus   = create_message_bus("zenoh");
    auto topic = "/test_factory_sub_" + std::to_string(getpid());
    auto pub   = bus.advertise<Pose>(topic);
    ASSERT_NE(pub, nullptr);

    auto sub = bus.subscribe<Pose>(topic, 5, 50);
    ASSERT_NE(sub, nullptr);
    EXPECT_TRUE(sub->is_connected());
}

// ═══════════════════════════════════════════════════════════
// Category 2: Zenoh pub/sub round-trip tests (HAVE_ZENOH only)
// ═══════════════════════════════════════════════════════════


// Small trivially-copyable test payload
struct ZenohTestPayload {
    uint64_t id{0};
    float    value{0.0f};
    char     tag[16]{};
};

TEST(ZenohSession, Opens) {
    auto& session = ZenohSession::instance();
    // Accessing session() triggers lazy open
    auto& s = session.session();
    EXPECT_TRUE(session.is_open());
    (void)s;
}

TEST(ZenohPublisher, Constructs) {
    ZenohPublisher<ZenohTestPayload> pub("drone/test/construct_pub");
    EXPECT_TRUE(pub.is_ready());
    EXPECT_EQ(pub.topic_name(), "drone/test/construct_pub");
}

TEST(ZenohSubscriber, Constructs) {
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/construct_sub");
    EXPECT_EQ(sub.topic_name(), "drone/test/construct_sub");
    // Zenoh subscription is immediately valid (data arrives asynchronously)
    EXPECT_TRUE(sub.is_connected());
    // No data yet
    ZenohTestPayload msg;
    EXPECT_FALSE(sub.receive(msg));
}

// ---------------------------------------------------------------------------
// Polling helpers — replace fixed sleep_for() with bounded retry loops
// so tests are not flaky under CI load.
// ---------------------------------------------------------------------------

/// Poll subscriber until a message is received or timeout is reached.
template<typename T>
bool poll_receive(drone::ipc::ZenohSubscriber<T>& sub, T& out, uint64_t* ts = nullptr,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    constexpr auto kInterval = std::chrono::milliseconds(5);
    auto           deadline  = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (sub.receive(out, ts)) return true;
        std::this_thread::sleep_for(kInterval);
    }
    return false;
}

/// Publish (with retries) until the subscriber receives a message.
/// Handles the Zenoh discovery window during which messages are dropped
/// because the publisher and subscriber have not yet matched.
template<typename T>
bool publish_until_received(drone::ipc::ZenohPublisher<T>& pub, const T& msg,
                            drone::ipc::ZenohSubscriber<T>& sub, T& out, uint64_t* ts = nullptr,
                            std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    constexpr auto kPollInterval  = std::chrono::milliseconds(5);
    constexpr auto kRetryInterval = std::chrono::milliseconds(50);
    auto           deadline       = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        pub.publish(msg);
        auto batch_end = std::min(std::chrono::steady_clock::now() + kRetryInterval, deadline);
        while (std::chrono::steady_clock::now() < batch_end) {
            if (sub.receive(out, ts)) return true;
            std::this_thread::sleep_for(kPollInterval);
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Round-trip tests
// ---------------------------------------------------------------------------

TEST(ZenohPubSub, SmallMessageRoundTrip) {
    ZenohPublisher<ZenohTestPayload>  pub("drone/test/small_rt");
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/small_rt");

    ZenohTestPayload sent{42, 3.14f, {}};
    std::strncpy(sent.tag, "hello", sizeof(sent.tag));

    ZenohTestPayload received;
    uint64_t         ts = 0;
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received, &ts));
    EXPECT_EQ(received.id, 42u);
    EXPECT_FLOAT_EQ(received.value, 3.14f);
    EXPECT_STREQ(received.tag, "hello");
    EXPECT_GT(ts, 0u);
    EXPECT_TRUE(sub.is_connected());
}

TEST(ZenohPubSub, ShmPoseRoundTrip) {
    ZenohPublisher<Pose>  pub("drone/test/pose_rt");
    ZenohSubscriber<Pose> sub("drone/test/pose_rt");

    Pose sent{};
    sent.timestamp_ns   = 123456789;
    sent.translation[0] = 1.0;
    sent.translation[1] = 2.0;
    sent.translation[2] = 3.0;
    sent.quaternion[0]  = 1.0;  // w
    sent.quality        = 2;

    Pose received{};
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received));
    EXPECT_EQ(received.timestamp_ns, 123456789u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(received.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(received.translation[2], 3.0);
    EXPECT_EQ(received.quality, 2u);
}

TEST(ZenohPubSub, LargeVideoFrameRoundTrip) {
    ZenohPublisher<VideoFrame>  pub("drone/test/video_rt");
    ZenohSubscriber<VideoFrame> sub("drone/test/video_rt");

    // Heap-allocate — VideoFrame is ~6 MB, too large for the stack.
    auto sent                                      = std::make_unique<VideoFrame>();
    sent->timestamp_ns                             = 999;
    sent->width                                    = 1920;
    sent->height                                   = 1080;
    sent->channels                                 = 3;
    sent->sequence_number                          = 7;
    sent->pixel_data[0]                            = 0xAA;
    sent->pixel_data[1]                            = 0xBB;
    sent->pixel_data[sizeof(sent->pixel_data) - 1] = 0xFF;

    auto received = std::make_unique<VideoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->height, 1080u);
    EXPECT_EQ(received->channels, 3u);
    EXPECT_EQ(received->sequence_number, 7u);
    EXPECT_EQ(received->pixel_data[0], 0xAA);
    EXPECT_EQ(received->pixel_data[1], 0xBB);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) - 1], 0xFF);
}

TEST(ZenohPubSub, MultipleTopics) {
    ZenohPublisher<ZenohTestPayload>  pub1("drone/test/multi/a");
    ZenohPublisher<ZenohTestPayload>  pub2("drone/test/multi/b");
    ZenohPublisher<ZenohTestPayload>  pub3("drone/test/multi/c");
    ZenohSubscriber<ZenohTestPayload> sub1("drone/test/multi/a");
    ZenohSubscriber<ZenohTestPayload> sub2("drone/test/multi/b");
    ZenohSubscriber<ZenohTestPayload> sub3("drone/test/multi/c");

    ZenohTestPayload m1{1, 1.0f, {}};
    ZenohTestPayload m2{2, 2.0f, {}};
    ZenohTestPayload m3{3, 3.0f, {}};
    ZenohTestPayload r1{}, r2{}, r3{};

    // Poll each topic pair with publish retries to handle discovery latency.
    ASSERT_TRUE(publish_until_received(pub1, m1, sub1, r1));
    ASSERT_TRUE(publish_until_received(pub2, m2, sub2, r2));
    ASSERT_TRUE(publish_until_received(pub3, m3, sub3, r3));
    EXPECT_EQ(r1.id, 1u);
    EXPECT_EQ(r2.id, 2u);
    EXPECT_EQ(r3.id, 3u);
}

TEST(ZenohPubSub, NoData) {
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/no_data");
    // Zenoh subscription is immediately valid (data arrives asynchronously)
    EXPECT_TRUE(sub.is_connected());
    ZenohTestPayload msg;
    EXPECT_FALSE(sub.receive(msg));
}

TEST(ZenohPubSub, SequenceIncrementsOnPublish) {
    ZenohPublisher<ZenohTestPayload>  pub("drone/test/seq");
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/seq");

    // First message — handles discovery latency via retry loop.
    ZenohTestPayload m1{10, 0.0f, {}};
    ZenohTestPayload r1;
    uint64_t         ts1 = 0;
    ASSERT_TRUE(publish_until_received(pub, m1, sub, r1, &ts1));
    EXPECT_EQ(r1.id, 10u);

    // Second message — discovery already done, poll-receive is sufficient.
    ZenohTestPayload m2{20, 0.0f, {}};
    pub.publish(m2);
    ZenohTestPayload r2;
    uint64_t         ts2 = 0;
    ASSERT_TRUE(poll_receive(sub, r2, &ts2));
    EXPECT_EQ(r2.id, 20u);
    EXPECT_GE(ts2, ts1);
}

TEST(ZenohMessageBus, AdvertiseCreatesPublisher) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<Pose>("/slam_pose");
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
    // Should have mapped to Zenoh key expression
    EXPECT_EQ(pub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, SubscribeCreatesSubscriber) {
    ZenohMessageBus bus;
    auto            sub = bus.subscribe<Pose>("/slam_pose");
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, SubscribeLazyCreatesSubscriber) {
    ZenohMessageBus bus;
    auto            sub = bus.subscribe_lazy<Pose>("/slam_pose");
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, RoundTripViaFactory) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<ZenohTestPayload>("/detected_objects");
    auto            sub = bus.subscribe<ZenohTestPayload>("/detected_objects");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    ZenohTestPayload sent{99, 2.71f, {}};
    ZenohTestPayload received;
    // Manual publish-and-poll loop (IPublisher/ISubscriber interfaces).
    bool ok       = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(sent);
        auto batch = std::min(std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
                              deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(received)) {
                ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok);
    EXPECT_EQ(received.id, 99u);
    EXPECT_FLOAT_EQ(received.value, 2.71f);
}

// ═══════════════════════════════════════════════════════════
// Category 3: Per-channel migration round-trip tests (Phase B)
//
// Verify each of the 10 low-bandwidth channels works end-to-end
// through ZenohMessageBus with legacy SHM topic names.
// ═══════════════════════════════════════════════════════════

// ---------------------------------------------------------------------------
// Interface-level polling helper (works with IPublisher/ISubscriber)
// ---------------------------------------------------------------------------

template<typename T>
static bool publish_until_received_iface(
    IPublisher<T>& pub, const T& msg, ISubscriber<T>& sub, T& out,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        pub.publish(msg);
        auto batch = std::min(std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
                              deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub.receive(out)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    return false;
}

// --- Channel 1: Pose (P3 → P4, P5) -----------------------

TEST(ZenohMigration, Pose_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<Pose>(topics::SLAM_POSE);
    auto            sub = bus.subscribe<Pose>(topics::SLAM_POSE);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(pub->topic_name(), "drone/slam/pose");
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");

    Pose sent{};
    sent.timestamp_ns   = 100;
    sent.translation[0] = 10.0;
    sent.translation[1] = 20.0;
    sent.translation[2] = 30.0;
    sent.quaternion[0]  = 1.0;
    sent.velocity[0]    = 1.5;
    sent.quality        = 2;

    Pose received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 100u);
    EXPECT_DOUBLE_EQ(received.translation[0], 10.0);
    EXPECT_DOUBLE_EQ(received.translation[1], 20.0);
    EXPECT_DOUBLE_EQ(received.translation[2], 30.0);
    EXPECT_DOUBLE_EQ(received.quaternion[0], 1.0);
    EXPECT_DOUBLE_EQ(received.velocity[0], 1.5);
    EXPECT_EQ(received.quality, 2u);
}

// --- Channel 2: FCState (P5 → P4, P7) --------------------

TEST(ZenohMigration, FCState_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<FCState>(topics::FC_STATE);
    auto            sub = bus.subscribe<FCState>(topics::FC_STATE);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(pub->topic_name(), "drone/comms/fc_state");

    FCState sent{};
    sent.timestamp_ns       = 200;
    sent.gps_lat            = 37.7749f;
    sent.gps_lon            = -122.4194f;
    sent.battery_voltage    = 11.8f;
    sent.battery_remaining  = 72.0f;
    sent.armed              = true;
    sent.connected          = true;
    sent.satellites_visible = 12;
    sent.flight_mode        = 6;

    FCState received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 200u);
    EXPECT_FLOAT_EQ(received.gps_lat, 37.7749f);
    EXPECT_FLOAT_EQ(received.gps_lon, -122.4194f);
    EXPECT_FLOAT_EQ(received.battery_voltage, 11.8f);
    EXPECT_FLOAT_EQ(received.battery_remaining, 72.0f);
    EXPECT_TRUE(received.armed);
    EXPECT_TRUE(received.connected);
    EXPECT_EQ(received.satellites_visible, 12);
    EXPECT_EQ(received.flight_mode, 6);
}

// --- Channel 3: FCCommand (P4 → P5) ----------------------

TEST(ZenohMigration, FCCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<FCCommand>(topics::FC_COMMANDS);
    auto            sub = bus.subscribe<FCCommand>(topics::FC_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/comms/fc_command");

    FCCommand sent{};
    sent.timestamp_ns = 300;
    sent.command      = FCCommandType::TAKEOFF;
    sent.param1       = 5.0f;
    sent.sequence_id  = 42;
    sent.valid        = true;

    FCCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 300u);
    EXPECT_EQ(received.command, FCCommandType::TAKEOFF);
    EXPECT_FLOAT_EQ(received.param1, 5.0f);
    EXPECT_EQ(received.sequence_id, 42u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 4: MissionStatus (P4 → P5, P7) --------------

TEST(ZenohMigration, MissionStatus_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<MissionStatus>(topics::MISSION_STATUS);
    auto            sub = bus.subscribe<MissionStatus>(topics::MISSION_STATUS);
    EXPECT_EQ(pub->topic_name(), "drone/mission/status");

    MissionStatus sent{};
    sent.timestamp_ns     = 400;
    sent.state            = MissionState::NAVIGATE;
    sent.current_waypoint = 3;
    sent.total_waypoints  = 10;
    sent.progress_percent = 30.0f;
    sent.target_x         = 100.0f;
    sent.target_y         = 200.0f;
    sent.target_z         = 50.0f;
    sent.battery_percent  = 85.0f;
    sent.mission_active   = true;

    MissionStatus received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 400u);
    EXPECT_EQ(received.state, MissionState::NAVIGATE);
    EXPECT_EQ(received.current_waypoint, 3u);
    EXPECT_EQ(received.total_waypoints, 10u);
    EXPECT_FLOAT_EQ(received.progress_percent, 30.0f);
    EXPECT_FLOAT_EQ(received.target_x, 100.0f);
    EXPECT_TRUE(received.mission_active);
}

// --- Channel 5: TrajectoryCmd (P4 → P5) -------------------

TEST(ZenohMigration, TrajectoryCmd_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    auto            sub = bus.subscribe<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    EXPECT_EQ(pub->topic_name(), "drone/mission/trajectory");

    TrajectoryCmd sent{};
    sent.timestamp_ns     = 500;
    sent.target_x         = 10.0f;
    sent.target_y         = 20.0f;
    sent.target_z         = 5.0f;
    sent.target_yaw       = 1.57f;
    sent.velocity_x       = 2.0f;
    sent.velocity_y       = 0.0f;
    sent.velocity_z       = -0.5f;
    sent.yaw_rate         = 0.1f;
    sent.coordinate_frame = 8;
    sent.valid            = true;

    TrajectoryCmd received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 500u);
    EXPECT_FLOAT_EQ(received.target_x, 10.0f);
    EXPECT_FLOAT_EQ(received.target_yaw, 1.57f);
    EXPECT_FLOAT_EQ(received.velocity_x, 2.0f);
    EXPECT_EQ(received.coordinate_frame, 8);
    EXPECT_TRUE(received.valid);
}

// --- Channel 6: GCSCommand (P5 → P4) ---------------------

TEST(ZenohMigration, GCSCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<GCSCommand>(topics::GCS_COMMANDS);
    auto            sub = bus.subscribe<GCSCommand>(topics::GCS_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/comms/gcs_command");

    GCSCommand sent{};
    sent.timestamp_ns = 600;
    sent.command      = GCSCommandType::RTL;
    sent.param1       = 0.0f;
    sent.param2       = 0.0f;
    sent.param3       = 0.0f;
    sent.sequence_id  = 7;
    sent.valid        = true;

    GCSCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 600u);
    EXPECT_EQ(received.command, GCSCommandType::RTL);
    EXPECT_EQ(received.sequence_id, 7u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 7: DetectedObjectList (P2 → P4) -------------

TEST(ZenohMigration, DetectedObjects_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<DetectedObjectList>(topics::DETECTED_OBJECTS);
    auto            sub = bus.subscribe<DetectedObjectList>(topics::DETECTED_OBJECTS);
    EXPECT_EQ(pub->topic_name(), "drone/perception/detections");

    DetectedObjectList sent{};
    sent.timestamp_ns          = 700;
    sent.frame_sequence        = 42;
    sent.num_objects           = 2;
    sent.objects[0].track_id   = 1;
    sent.objects[0].class_id   = ObjectClass::PERSON;
    sent.objects[0].confidence = 0.95f;
    sent.objects[0].position_x = 5.0f;
    sent.objects[0].position_y = 3.0f;
    sent.objects[0].position_z = 0.0f;
    sent.objects[1].track_id   = 2;
    sent.objects[1].class_id   = ObjectClass::VEHICLE_CAR;
    sent.objects[1].confidence = 0.87f;

    DetectedObjectList received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 700u);
    EXPECT_EQ(received.frame_sequence, 42u);
    EXPECT_EQ(received.num_objects, 2u);
    EXPECT_EQ(received.objects[0].track_id, 1u);
    EXPECT_EQ(received.objects[0].class_id, ObjectClass::PERSON);
    EXPECT_FLOAT_EQ(received.objects[0].confidence, 0.95f);
    EXPECT_FLOAT_EQ(received.objects[0].position_x, 5.0f);
    EXPECT_EQ(received.objects[1].track_id, 2u);
    EXPECT_EQ(received.objects[1].class_id, ObjectClass::VEHICLE_CAR);
}

// --- Channel 8: PayloadCommand (P4 → P6) -----------------

TEST(ZenohMigration, PayloadCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<PayloadCommand>(topics::PAYLOAD_COMMANDS);
    auto            sub = bus.subscribe<PayloadCommand>(topics::PAYLOAD_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/mission/payload_command");

    PayloadCommand sent{};
    sent.timestamp_ns = 800;
    sent.action       = PayloadAction::GIMBAL_POINT;
    sent.gimbal_pitch = -30.0f;
    sent.gimbal_yaw   = 45.0f;
    sent.sequence_id  = 100;
    sent.valid        = true;

    PayloadCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 800u);
    EXPECT_EQ(received.action, PayloadAction::GIMBAL_POINT);
    EXPECT_FLOAT_EQ(received.gimbal_pitch, -30.0f);
    EXPECT_FLOAT_EQ(received.gimbal_yaw, 45.0f);
    EXPECT_EQ(received.sequence_id, 100u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 9: PayloadStatus (P6 → P4, P7) --------------

TEST(ZenohMigration, PayloadStatus_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<PayloadStatus>(topics::PAYLOAD_STATUS);
    auto            sub = bus.subscribe<PayloadStatus>(topics::PAYLOAD_STATUS);
    EXPECT_EQ(pub->topic_name(), "drone/payload/status");

    PayloadStatus sent{};
    sent.timestamp_ns       = 900;
    sent.gimbal_pitch       = -15.0f;
    sent.gimbal_yaw         = 90.0f;
    sent.images_captured    = 42;
    sent.recording_video    = true;
    sent.gimbal_stabilized  = true;
    sent.num_plugins_active = 3;

    PayloadStatus received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 900u);
    EXPECT_FLOAT_EQ(received.gimbal_pitch, -15.0f);
    EXPECT_FLOAT_EQ(received.gimbal_yaw, 90.0f);
    EXPECT_EQ(received.images_captured, 42u);
    EXPECT_TRUE(received.recording_video);
    EXPECT_TRUE(received.gimbal_stabilized);
    EXPECT_EQ(received.num_plugins_active, 3);
}

// --- Channel 10: SystemHealth (P7 → P4) -------------------

TEST(ZenohMigration, SystemHealth_RoundTrip) {
    ZenohMessageBus bus;
    auto            pub = bus.advertise<SystemHealth>(topics::SYSTEM_HEALTH);
    auto            sub = bus.subscribe<SystemHealth>(topics::SYSTEM_HEALTH);
    EXPECT_EQ(pub->topic_name(), "drone/monitor/health");

    SystemHealth sent{};
    sent.timestamp_ns         = 1000;
    sent.cpu_usage_percent    = 45.0f;
    sent.memory_usage_percent = 62.0f;
    sent.disk_usage_percent   = 38.0f;
    sent.max_temp_c           = 68.0f;
    sent.gpu_temp_c           = 55.0f;
    sent.cpu_temp_c           = 65.0f;
    sent.total_healthy        = 7;
    sent.total_degraded       = 0;
    sent.total_dead           = 0;
    sent.power_watts          = 12.5f;
    sent.thermal_zone         = 1;

    SystemHealth received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 1000u);
    EXPECT_FLOAT_EQ(received.cpu_usage_percent, 45.0f);
    EXPECT_FLOAT_EQ(received.memory_usage_percent, 62.0f);
    EXPECT_FLOAT_EQ(received.disk_usage_percent, 38.0f);
    EXPECT_FLOAT_EQ(received.max_temp_c, 68.0f);
    EXPECT_EQ(received.total_healthy, 7u);
    EXPECT_EQ(received.total_dead, 0u);
    EXPECT_FLOAT_EQ(received.power_watts, 12.5f);
    EXPECT_EQ(received.thermal_zone, 1);
}

// --- Multi-channel: 5 channels active simultaneously ----------

TEST(ZenohMigration, MultiChannel_Simultaneous) {
    ZenohMessageBus bus;

    auto pose_pub   = bus.advertise<Pose>(topics::SLAM_POSE);
    auto fc_pub     = bus.advertise<FCState>(topics::FC_STATE);
    auto traj_pub   = bus.advertise<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    auto health_pub = bus.advertise<SystemHealth>(topics::SYSTEM_HEALTH);
    auto status_pub = bus.advertise<MissionStatus>(topics::MISSION_STATUS);

    auto pose_sub   = bus.subscribe<Pose>(topics::SLAM_POSE);
    auto fc_sub     = bus.subscribe<FCState>(topics::FC_STATE);
    auto traj_sub   = bus.subscribe<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    auto health_sub = bus.subscribe<SystemHealth>(topics::SYSTEM_HEALTH);
    auto status_sub = bus.subscribe<MissionStatus>(topics::MISSION_STATUS);

    Pose pose_s{};
    pose_s.timestamp_ns = 1;
    pose_s.quality      = 2;
    FCState fc_s{};
    fc_s.timestamp_ns = 2;
    fc_s.armed        = true;
    TrajectoryCmd traj_s{};
    traj_s.timestamp_ns = 3;
    traj_s.valid        = true;
    SystemHealth hlth_s{};
    hlth_s.timestamp_ns  = 4;
    hlth_s.total_healthy = 7;
    MissionStatus ms_s{};
    ms_s.timestamp_ns   = 5;
    ms_s.mission_active = true;

    Pose          pose_r{};
    FCState       fc_r{};
    TrajectoryCmd traj_r{};
    SystemHealth  hlth_r{};
    MissionStatus ms_r{};

    ASSERT_TRUE(publish_until_received_iface(*pose_pub, pose_s, *pose_sub, pose_r));
    ASSERT_TRUE(publish_until_received_iface(*fc_pub, fc_s, *fc_sub, fc_r));
    ASSERT_TRUE(publish_until_received_iface(*traj_pub, traj_s, *traj_sub, traj_r));
    ASSERT_TRUE(publish_until_received_iface(*health_pub, hlth_s, *health_sub, hlth_r));
    ASSERT_TRUE(publish_until_received_iface(*status_pub, ms_s, *status_sub, ms_r));

    EXPECT_EQ(pose_r.quality, 2u);
    EXPECT_TRUE(fc_r.armed);
    EXPECT_TRUE(traj_r.valid);
    EXPECT_EQ(hlth_r.total_healthy, 7u);
    EXPECT_TRUE(ms_r.mission_active);
}

// --- High-rate: Pose at 200 Hz for 2 seconds -----------------

TEST(ZenohMigration, HighRate_Pose) {
    ZenohPublisher<Pose>  pub("drone/test/highrate_pose");
    ZenohSubscriber<Pose> sub("drone/test/highrate_pose");

    // Wait for discovery
    {
        Pose warm{};
        warm.timestamp_ns = 0;
        Pose warmr{};
        ASSERT_TRUE(publish_until_received(pub, warm, sub, warmr));
    }

    // Publish 400 messages at 200 Hz (5 ms interval) for 2 seconds
    constexpr int total_msgs = 400;
    for (int i = 1; i <= total_msgs; ++i) {
        Pose msg{};
        msg.timestamp_ns = static_cast<uint64_t>(i);
        msg.quality      = 2;
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Poll until the subscriber holds the final published message.
    // ZenohSubscriber keeps the latest value (non-consuming receive),
    // so we poll repeatedly until the final message (ts == total_msgs)
    // appears or a generous timeout expires.
    Pose last{};
    bool got_final     = false;
    auto poll_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    while (std::chrono::steady_clock::now() < poll_deadline) {
        if (sub.receive(last) && last.timestamp_ns == static_cast<uint64_t>(total_msgs)) {
            got_final = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(got_final) << "Timed out waiting for final message (ts=" << total_msgs
                           << "); last seen ts=" << last.timestamp_ns;
    EXPECT_EQ(last.quality, 2u);
}

// --- Factory integration: bus_subscribe_optional ---------------

TEST(ZenohMigration, FactorySubscribeOptional) {
    // Use a unique key so this test is isolated from any other test that
    // may publish on GCS_COMMANDS.
    std::string unique_key = std::string("/factory_opt_test_") + std::to_string(::getpid());
    auto        bus        = create_message_bus("zenoh");
    auto        sub        = bus.subscribe_optional<GCSCommand>(unique_key);
    ASSERT_NE(sub, nullptr);
    // No publisher exists — receive should return false but not crash
    GCSCommand cmd{};
    EXPECT_FALSE(sub->receive(cmd));
}


// ═══════════════════════════════════════════════════════════
// Category 4: Phase C — SHM provider zero-copy tests
// ═══════════════════════════════════════════════════════════

// Helper: returns true when PosixShmProvider is functional.
// Pre-built zenohc release packages omit the shared-memory cargo feature,
// so shm_provider() returns nullptr on those builds.
static bool shm_available() {
    return ZenohSession::instance().shm_provider() != nullptr;
}

// --- SHM Provider tests -----------------------------------------------
// PosixShmProvider requires zenohc built with the shared-memory cargo
// feature.  CI uses our custom .deb (see deploy/build_zenohc_deb.sh)
// which includes this feature, so these tests run unconditionally.

TEST(ZenohShmProvider, ProviderCreatesSuccessfully) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr) << "SHM provider must be available "
                                    "(zenohc must be built with shared-memory feature)";
    SUCCEED();  // provider != nullptr — creation works
}

TEST(ZenohShmProvider, AllocAndWriteBuffer) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr) << "SHM provider required";

    constexpr std::size_t buf_size = 4096;
    auto                  result   = provider->alloc_gc_defrag_blocking(buf_size);
    auto*                 buf      = std::get_if<zenoh::ZShmMut>(&result);
    ASSERT_NE(buf, nullptr);
    EXPECT_EQ(buf->len(), buf_size);

    // Write a pattern and verify
    std::memset(buf->data(), 0xCD, buf_size);
    EXPECT_EQ(buf->data()[0], 0xCD);
    EXPECT_EQ(buf->data()[buf_size - 1], 0xCD);
}

TEST(ZenohShmProvider, AllocLargeVideoFrameBuffer) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr) << "SHM provider required";

    // Allocate a buffer the size of VideoFrame (~6.2 MB)
    auto  result = provider->alloc_gc_defrag_blocking(sizeof(VideoFrame));
    auto* buf    = std::get_if<zenoh::ZShmMut>(&result);
    ASSERT_NE(buf, nullptr) << "Failed to allocate " << sizeof(VideoFrame)
                            << " bytes from SHM pool";
    EXPECT_EQ(buf->len(), sizeof(VideoFrame));
}

// --- Size-aware publish path tests ------------------------------------

TEST(ZenohShmPublish, SmallMessageUsesBytes) {
    // Pose is small (~400 bytes) — should use bytes path, not SHM
    static_assert(sizeof(Pose) <= kShmPublishThreshold, "Pose should be below SHM threshold");

    ZenohPublisher<Pose>  pub("drone/test/shm_small_path");
    ZenohSubscriber<Pose> sub("drone/test/shm_small_path");

    Pose sent{};
    sent.timestamp_ns = 42;
    sent.quality      = 2;

    Pose received{};
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received));
    EXPECT_EQ(received.timestamp_ns, 42u);
    EXPECT_EQ(received.quality, 2u);
}

TEST(ZenohShmPublish, LargeVideoFrameUsesShmPath) {
    ASSERT_TRUE(shm_available()) << "SHM provider required for SHM path assertions";

    // VideoFrame is ~6.2 MB — should use SHM provider path
    static_assert(sizeof(VideoFrame) > kShmPublishThreshold,
                  "VideoFrame should be above SHM threshold");

    ZenohPublisher<VideoFrame>  pub("drone/test/shm_video_path");
    ZenohSubscriber<VideoFrame> sub("drone/test/shm_video_path");

    // Record baseline counters before publish
    const auto shm_before   = pub.shm_publish_count();
    const auto bytes_before = pub.bytes_publish_count();

    // Heap-allocate — too large for the stack
    auto sent             = std::make_unique<VideoFrame>();
    sent->timestamp_ns    = 12345;
    sent->width           = 1920;
    sent->height          = 1080;
    sent->channels        = 3;
    sent->sequence_number = 99;
    // Write sentinel bytes at key positions
    sent->pixel_data[0]                            = 0xDE;
    sent->pixel_data[1]                            = 0xAD;
    sent->pixel_data[sizeof(sent->pixel_data) / 2] = 0xBE;
    sent->pixel_data[sizeof(sent->pixel_data) - 1] = 0xEF;

    auto received = std::make_unique<VideoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->timestamp_ns, 12345u);
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->height, 1080u);
    EXPECT_EQ(received->channels, 3u);
    EXPECT_EQ(received->sequence_number, 99u);
    EXPECT_EQ(received->pixel_data[0], 0xDE);
    EXPECT_EQ(received->pixel_data[1], 0xAD);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) / 2], 0xBE);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) - 1], 0xEF);

    // Verify SHM path was actually used (not the bytes fallback)
    EXPECT_GT(pub.shm_publish_count(), shm_before) << "Expected SHM publish path for VideoFrame";
    EXPECT_EQ(pub.bytes_publish_count(), bytes_before)
        << "Bytes path should not be used for VideoFrame";
}

TEST(ZenohShmPublish, StereoFrameUsesShmPath) {
    ASSERT_TRUE(shm_available()) << "SHM provider required for SHM path assertions";

    // StereoFrame is ~614 KB — should use SHM provider path
    static_assert(sizeof(StereoFrame) > kShmPublishThreshold,
                  "StereoFrame should be above SHM threshold");

    ZenohPublisher<StereoFrame>  pub("drone/test/shm_stereo_path");
    ZenohSubscriber<StereoFrame> sub("drone/test/shm_stereo_path");

    const auto shm_before   = pub.shm_publish_count();
    const auto bytes_before = pub.bytes_publish_count();

    auto sent                                      = std::make_unique<StereoFrame>();
    sent->timestamp_ns                             = 77777;
    sent->width                                    = 640;
    sent->height                                   = 480;
    sent->sequence_number                          = 42;
    sent->left_data[0]                             = 0xAA;
    sent->right_data[0]                            = 0xBB;
    sent->left_data[sizeof(sent->left_data) - 1]   = 0xCC;
    sent->right_data[sizeof(sent->right_data) - 1] = 0xDD;

    auto received = std::make_unique<StereoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->timestamp_ns, 77777u);
    EXPECT_EQ(received->width, 640u);
    EXPECT_EQ(received->height, 480u);
    EXPECT_EQ(received->left_data[0], 0xAA);
    EXPECT_EQ(received->right_data[0], 0xBB);
    EXPECT_EQ(received->left_data[sizeof(received->left_data) - 1], 0xCC);
    EXPECT_EQ(received->right_data[sizeof(received->right_data) - 1], 0xDD);

    // Verify SHM path was actually used
    EXPECT_GT(pub.shm_publish_count(), shm_before) << "Expected SHM publish path for StereoFrame";
    EXPECT_EQ(pub.bytes_publish_count(), bytes_before)
        << "Bytes path should not be used for StereoFrame";
}

// --- Factory integration with video channels --------------------------

TEST(ZenohShmPublish, FactoryVideoRoundTrip) {
    // End-to-end: factory → advertise VideoFrame → subscribe → roundtrip
    auto bus = create_message_bus("zenoh");
    auto pub = bus.advertise<VideoFrame>(topics::VIDEO_MISSION_CAM);
    auto sub = bus.subscribe<VideoFrame>(topics::VIDEO_MISSION_CAM);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    auto sent             = std::make_unique<VideoFrame>();
    sent->timestamp_ns    = 555;
    sent->width           = 1920;
    sent->height          = 1080;
    sent->channels        = 3;
    sent->pixel_data[100] = 0x42;

    auto received = std::make_unique<VideoFrame>();
    bool ok       = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(*sent);
        auto batch = std::min(std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
                              deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(*received)) {
                ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok) << "Factory video round-trip timed out";
    EXPECT_EQ(received->timestamp_ns, 555u);
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->pixel_data[100], 0x42);
}

TEST(ZenohShmPublish, FactoryStereoRoundTrip) {
    auto bus = create_message_bus("zenoh");
    auto pub = bus.advertise<StereoFrame>(topics::VIDEO_STEREO_CAM);
    auto sub = bus.subscribe<StereoFrame>(topics::VIDEO_STEREO_CAM);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    auto sent           = std::make_unique<StereoFrame>();
    sent->timestamp_ns  = 888;
    sent->width         = 640;
    sent->height        = 480;
    sent->left_data[42] = 0x99;

    auto received = std::make_unique<StereoFrame>();
    bool ok       = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(*sent);
        auto batch = std::min(std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
                              deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(*received)) {
                ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok) << "Factory stereo round-trip timed out";
    EXPECT_EQ(received->timestamp_ns, 888u);
    EXPECT_EQ(received->left_data[42], 0x99);
}

// --- Sustained video publish test (simulates 30 Hz camera) -----------

TEST(ZenohShmPublish, SustainedVideoPublish) {
    ASSERT_TRUE(shm_available()) << "SHM provider required for SHM path assertions";

    ZenohPublisher<VideoFrame>  pub("drone/test/shm_sustained_video");
    ZenohSubscriber<VideoFrame> sub("drone/test/shm_sustained_video");

    auto frame      = std::make_unique<VideoFrame>();
    frame->width    = 1920;
    frame->height   = 1080;
    frame->channels = 3;

    auto received = std::make_unique<VideoFrame>();

    // Warm up discovery
    {
        frame->timestamp_ns = 0;
        ASSERT_TRUE(publish_until_received(pub, *frame, sub, *received));
    }

    // Track distinct frames received via polling during publish window
    std::atomic<int> frames_seen{0};
    uint64_t         last_seen_ts = 0;

    // Publish 30 frames (simulating 1 second at 30 Hz)
    constexpr int total_frames = 30;
    for (int i = 1; i <= total_frames; ++i) {
        frame->timestamp_ns    = static_cast<uint64_t>(i);
        frame->sequence_number = static_cast<uint64_t>(i);
        frame->pixel_data[0]   = static_cast<uint8_t>(i & 0xFF);
        pub.publish(*frame);

        // Poll between publishes to count distinct frames
        if (sub.receive(*received) && received->timestamp_ns != last_seen_ts) {
            last_seen_ts = received->timestamp_ns;
            frames_seen.fetch_add(1, std::memory_order_relaxed);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // Wait for final frame to arrive
    auto deadline  = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    bool got_final = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (sub.receive(*received)) {
            if (received->timestamp_ns != last_seen_ts) {
                last_seen_ts = received->timestamp_ns;
                frames_seen.fetch_add(1, std::memory_order_relaxed);
            }
            if (received->timestamp_ns == static_cast<uint64_t>(total_frames)) {
                got_final = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(got_final) << "Did not receive final video frame (ts=" << total_frames
                           << "); last seen ts=" << received->timestamp_ns;
    EXPECT_EQ(received->sequence_number, static_cast<uint64_t>(total_frames));

    // Verify we saw a meaningful number of distinct frames (not just the last)
    // With 33ms publish interval and 10ms poll interval, we should see
    // at least half the frames. This validates sustained delivery, not just
    // final-frame arrival.
    EXPECT_GE(frames_seen.load(), total_frames / 2)
        << "Expected to observe at least " << total_frames / 2 << " distinct frames, but only saw "
        << frames_seen.load();

    // Verify SHM path was used for all publishes
    EXPECT_GE(pub.shm_publish_count(), static_cast<uint64_t>(total_frames))
        << "Expected at least " << total_frames << " SHM publishes";
}

// --- SHM pool configuration test --------------------------------------

TEST(ZenohShmProvider, PoolSizeConfiguration) {
    // Verify default pool size constant
    EXPECT_EQ(kDefaultShmPoolBytes, 32u * 1024 * 1024);
    EXPECT_EQ(kShmPublishThreshold, 64u * 1024);

    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr) << "SHM provider required";
    EXPECT_GT(ZenohSession::instance().shm_pool_bytes(), 0u);
}

// ═══════════════════════════════════════════════════════════
// Category 5: Phase D — Zenoh service channel tests
// ═══════════════════════════════════════════════════════════

// Service test payload types
struct SvcTestRequest {
    uint32_t command{0};
    float    param{0.0f};
};

struct SvcTestResponse {
    uint32_t result{0};
    bool     success{false};
};

// ---------------------------------------------------------------------------
// Polling helper — waits for the server to receive a request.
// ---------------------------------------------------------------------------
template<typename Req, typename Resp>
std::optional<ServiceEnvelope<Req>> poll_server_request(
    IServiceServer<Req, Resp>& server,
    std::chrono::milliseconds  timeout = std::chrono::milliseconds(5000)) {
    constexpr auto kInterval = std::chrono::milliseconds(5);
    auto           deadline  = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto req = server.poll_request();
        if (req.has_value()) return req;
        std::this_thread::sleep_for(kInterval);
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// 5.1 — Server constructs and declares queryable
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ServerConstructs) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_construct");
    // Should not crash; queryable declared internally
}

// ---------------------------------------------------------------------------
// 5.2 — Client constructs
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ClientConstructs) {
    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client(
        "drone/service/test_client_construct");
    // Should not crash
}

// ---------------------------------------------------------------------------
// 5.3 — Client sends request, server receives it
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ClientSendServerReceive) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_cs_recv");

    // Brief delay for queryable declaration to propagate
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client("drone/service/test_cs_recv");

    SvcTestRequest req{42, 3.14f};
    auto           cid = client.send_request(req);
    EXPECT_GE(cid, 1u);

    // Server should receive the request
    auto received = poll_server_request<SvcTestRequest, SvcTestResponse>(server);
    ASSERT_TRUE(received.has_value()) << "Server did not receive request within timeout";
    EXPECT_EQ(received->correlation_id, cid);
    EXPECT_EQ(received->payload.command, 42u);
    EXPECT_FLOAT_EQ(received->payload.param, 3.14f);
    EXPECT_TRUE(received->valid);
}

// ---------------------------------------------------------------------------
// 5.4 — Full round-trip: client → server → client
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, FullRoundTrip) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_roundtrip");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client("drone/service/test_roundtrip");

    SvcTestRequest req{1, 2.5f};
    auto           cid = client.send_request(req);

    // Server side: receive request and send response
    auto srv_req = poll_server_request<SvcTestRequest, SvcTestResponse>(server);
    ASSERT_TRUE(srv_req.has_value());

    SvcTestResponse resp{100, true};
    server.send_response(cid, ServiceStatus::OK, resp);

    // Client side: await response
    auto cli_resp = client.await_response(cid, std::chrono::milliseconds(5000));
    ASSERT_TRUE(cli_resp.has_value());
    EXPECT_EQ(cli_resp->correlation_id, cid);
    EXPECT_EQ(cli_resp->status, ServiceStatus::OK);
    EXPECT_EQ(cli_resp->payload.result, 100u);
    EXPECT_TRUE(cli_resp->payload.success);
    EXPECT_TRUE(cli_resp->valid);
}

// ---------------------------------------------------------------------------
// 5.5 — Wrong correlation ID returns nullopt
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, WrongCorrelationIdReturnsNull) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_wrong_cid");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client("drone/service/test_wrong_cid");

    auto cid = client.send_request(SvcTestRequest{1, 0.0f});

    auto srv_req = poll_server_request<SvcTestRequest, SvcTestResponse>(server);
    ASSERT_TRUE(srv_req.has_value());

    SvcTestResponse resp{1, true};
    server.send_response(cid, ServiceStatus::OK, resp);

    // Wait for the actual response to arrive
    auto correct = client.await_response(cid, std::chrono::milliseconds(5000));
    ASSERT_TRUE(correct.has_value());

    // Now polling for a different CID should return nothing
    auto wrong = client.poll_response(cid + 999);
    EXPECT_FALSE(wrong.has_value());
}

// ---------------------------------------------------------------------------
// 5.6 — Await response timeout
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, AwaitResponseTimeout) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_timeout");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client("drone/service/test_timeout",
                                                               200 /* short GET timeout */);

    auto cid = client.send_request(SvcTestRequest{1, 0.0f});
    // Don't send a server response — should timeout
    auto result = client.await_response(cid, std::chrono::milliseconds(300));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->status, ServiceStatus::TIMEOUT);
}

// ---------------------------------------------------------------------------
// 5.7 — Implements IServiceClient / IServiceServer interfaces
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ImplementsIServiceClient) {
    auto client = std::make_unique<ZenohServiceClient<SvcTestRequest, SvcTestResponse>>(
        "drone/service/test_iface_client");
    IServiceClient<SvcTestRequest, SvcTestResponse>* iface = client.get();
    (void)iface;  // Just verify the cast compiles
}

TEST(ZenohServiceChannel, ImplementsIServiceServer) {
    auto server = std::make_unique<ZenohServiceServer<SvcTestRequest, SvcTestResponse>>(
        "drone/service/test_iface_server");
    IServiceServer<SvcTestRequest, SvcTestResponse>* iface = server.get();
    (void)iface;  // Just verify the cast compiles
}

// ---------------------------------------------------------------------------
// 5.8 — Multiple sequential requests
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, MultipleRequestsSequential) {
    ZenohServiceServer<SvcTestRequest, SvcTestResponse> server("drone/service/test_multi_seq");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohServiceClient<SvcTestRequest, SvcTestResponse> client("drone/service/test_multi_seq");

    for (int i = 0; i < 5; ++i) {
        SvcTestRequest req{static_cast<uint32_t>(i), 0.0f};
        auto           cid = client.send_request(req);

        auto srv_req = poll_server_request<SvcTestRequest, SvcTestResponse>(server);
        ASSERT_TRUE(srv_req.has_value()) << "Server did not receive request " << i;
        EXPECT_EQ(srv_req->payload.command, static_cast<uint32_t>(i));

        SvcTestResponse resp{static_cast<uint32_t>(i * 10), true};
        server.send_response(cid, ServiceStatus::OK, resp);

        auto cli_resp = client.await_response(cid, std::chrono::milliseconds(5000));
        ASSERT_TRUE(cli_resp.has_value()) << "Client did not receive response for request " << i;
        EXPECT_EQ(cli_resp->payload.result, static_cast<uint32_t>(i * 10));
        EXPECT_EQ(cli_resp->status, ServiceStatus::OK);
    }
}

// ---------------------------------------------------------------------------
// 5.9 — Service via ZenohMessageBus factory
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ViaMessageBusFactory) {
    ZenohMessageBus bus;

    auto server = bus.create_server<SvcTestRequest, SvcTestResponse>("test_factory_svc");
    ASSERT_NE(server, nullptr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto client = bus.create_client<SvcTestRequest, SvcTestResponse>("test_factory_svc");
    ASSERT_NE(client, nullptr);

    auto cid     = client->send_request(SvcTestRequest{7, 1.0f});
    auto srv_req = poll_server_request<SvcTestRequest, SvcTestResponse>(*server);
    ASSERT_TRUE(srv_req.has_value());

    server->send_response(cid, ServiceStatus::OK, SvcTestResponse{77, true});

    auto resp = client->await_response(cid, std::chrono::milliseconds(5000));
    ASSERT_TRUE(resp.has_value());
    EXPECT_EQ(resp->status, ServiceStatus::OK);
    EXPECT_EQ(resp->payload.result, 77u);
}

// ---------------------------------------------------------------------------
// 5.10 — Service key expression mapping
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ServiceKeyMapping) {
    // Short name → prefixed
    EXPECT_EQ(ZenohMessageBus::to_service_key("trajectory"), "drone/service/trajectory");

    // SHM-style name → prefix stripped, converted
    EXPECT_EQ(ZenohMessageBus::to_service_key("/svc_traj_cmd"), "drone/service/traj/cmd");

    // Already a full key expression → pass-through
    EXPECT_EQ(ZenohMessageBus::to_service_key("drone/service/custom"), "drone/service/custom");

    // Empty → empty
    EXPECT_EQ(ZenohMessageBus::to_service_key(""), "");
}

// ---------------------------------------------------------------------------
// 5.11 — bus.create_client / bus.create_server via MessageBus
// ---------------------------------------------------------------------------

TEST(ZenohServiceChannel, ViaMessageBus) {
    auto bus = create_message_bus("zenoh");

    auto server =
        bus.create_server<SvcTestRequest, SvcTestResponse>("drone/service/test_variant_svc");
    ASSERT_NE(server, nullptr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto client =
        bus.create_client<SvcTestRequest, SvcTestResponse>("drone/service/test_variant_svc");
    ASSERT_NE(client, nullptr);

    auto cid     = client->send_request(SvcTestRequest{99, 0.0f});
    auto srv_req = poll_server_request<SvcTestRequest, SvcTestResponse>(*server);
    ASSERT_TRUE(srv_req.has_value());
    EXPECT_EQ(srv_req->payload.command, 99u);

    server->send_response(cid, ServiceStatus::OK, SvcTestResponse{999, true});

    auto resp = client->await_response(cid, std::chrono::milliseconds(5000));
    ASSERT_TRUE(resp.has_value());
    EXPECT_EQ(resp->payload.result, 999u);
}

// ---------------------------------------------------------------------------
// 5.12 — Service channels are available on Zenoh
// ---------------------------------------------------------------------------

TEST(MessageBusFactory, ZenohBusServiceClientReturnsNonNull) {
    auto bus    = create_message_bus("zenoh");
    auto client = bus.create_client<SvcTestRequest, SvcTestResponse>("test_svc_avail");
    EXPECT_NE(client, nullptr);
}

TEST(MessageBusFactory, ZenohBusServiceServerReturnsNonNull) {
    auto bus    = create_message_bus("zenoh");
    auto server = bus.create_server<SvcTestRequest, SvcTestResponse>("test_svc_avail");
    EXPECT_NE(server, nullptr);
}
