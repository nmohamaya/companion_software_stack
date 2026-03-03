// tests/test_message_bus.cpp
// Tests for IPublisher/ISubscriber/ShmMessageBus interfaces and
// IServiceChannel interface types (enums, structs).
//
// Note: SHM service channel tests were removed in Phase D (#49).
//       Zenoh service channel tests are in test_zenoh_ipc.cpp (Category 5).
#include "ipc/ipublisher.h"
#include "ipc/iservice_channel.h"
#include "ipc/isubscriber.h"
#include "ipc/shm_message_bus.h"
#include "ipc/shm_publisher.h"
#include "ipc/shm_subscriber.h"
#include "ipc/shm_types.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Helper: unique SHM names to avoid test collisions
// ═══════════════════════════════════════════════════════════
static std::string unique_name(const std::string& base) {
    static int counter = 0;
    return base + "_mbus_" + std::to_string(getpid()) + "_" + std::to_string(counter++);
}

// ═══════════════════════════════════════════════════════════
// Test data struct (trivially copyable)
// ═══════════════════════════════════════════════════════════
struct TestPayload {
    uint64_t id{0};
    float    value{0.0f};
    char     tag[16]{};
};

// ═══════════════════════════════════════════════════════════
// ShmPublisher tests
// ═══════════════════════════════════════════════════════════

TEST(ShmPublisherTest, CreateAndPublish) {
    auto                      name = unique_name("/test_pub");
    ShmPublisher<TestPayload> pub(name);
    ASSERT_TRUE(pub.is_ready());
    EXPECT_EQ(pub.topic_name(), name);

    TestPayload msg{42, 3.14f, "hello"};
    pub.publish(msg);  // should not crash
}

TEST(ShmPublisherTest, TopicNameStored) {
    auto                      name = unique_name("/test_pub_name");
    ShmPublisher<TestPayload> pub(name);
    EXPECT_EQ(pub.topic_name(), name);
}

TEST(ShmPublisherTest, ImplementsIPublisher) {
    auto                     name  = unique_name("/test_pub_iface");
    auto                     pub   = std::make_unique<ShmPublisher<TestPayload>>(name);
    IPublisher<TestPayload>* iface = pub.get();
    EXPECT_TRUE(iface->is_ready());
    EXPECT_EQ(iface->topic_name(), name);
}

// ═══════════════════════════════════════════════════════════
// ShmSubscriber tests
// ═══════════════════════════════════════════════════════════

TEST(ShmSubscriberTest, LazyConstructAndConnect) {
    auto name = unique_name("/test_sub_lazy");
    // Create a publisher first so the SHM topic exists
    ShmPublisher<TestPayload> pub(name);
    ASSERT_TRUE(pub.is_ready());

    ShmSubscriber<TestPayload> sub;
    EXPECT_FALSE(sub.is_connected());
    EXPECT_TRUE(sub.connect(name));
    EXPECT_TRUE(sub.is_connected());
}

TEST(ShmSubscriberTest, ReceivePublishedData) {
    auto                      name = unique_name("/test_sub_recv");
    ShmPublisher<TestPayload> pub(name);

    ShmSubscriber<TestPayload> sub;
    ASSERT_TRUE(sub.connect(name));
    ASSERT_TRUE(sub.is_connected());

    TestPayload sent{99, 2.718f, {}};
    std::strncpy(sent.tag, "test", sizeof(sent.tag));
    pub.publish(sent);

    TestPayload received{};
    ASSERT_TRUE(sub.receive(received));
    EXPECT_EQ(received.id, 99u);
    EXPECT_FLOAT_EQ(received.value, 2.718f);
    EXPECT_STREQ(received.tag, "test");
}

TEST(ShmSubscriberTest, ImplementsISubscriber) {
    auto                      name = unique_name("/test_sub_iface");
    ShmPublisher<TestPayload> pub(name);

    auto                      sub   = std::make_unique<ShmSubscriber<TestPayload>>(name, 1, 10);
    ISubscriber<TestPayload>* iface = sub.get();
    EXPECT_TRUE(iface->is_connected());
    EXPECT_EQ(iface->topic_name(), name);
}

TEST(ShmSubscriberTest, LazyConnectFails) {
    ShmSubscriber<TestPayload> sub;
    EXPECT_FALSE(sub.connect("/nonexistent_topic_12345"));
    EXPECT_FALSE(sub.is_connected());
}

// ═══════════════════════════════════════════════════════════
// ShmMessageBus tests
// ═══════════════════════════════════════════════════════════

TEST(ShmMessageBusTest, AdvertiseAndSubscribe) {
    auto          name = unique_name("/test_bus");
    ShmMessageBus bus;

    auto pub = bus.advertise<TestPayload>(name);
    ASSERT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<TestPayload>(name, 5, 10);
    ASSERT_TRUE(sub->is_connected());

    TestPayload sent{123, 1.0f, {}};
    pub->publish(sent);

    TestPayload received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.id, 123u);
}

TEST(ShmMessageBusTest, AdvReturnsIPublisher) {
    auto                                     name = unique_name("/test_bus_type");
    ShmMessageBus                            bus;
    std::unique_ptr<IPublisher<TestPayload>> pub = bus.advertise<TestPayload>(name);
    EXPECT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
}

TEST(ShmMessageBusTest, SubReturnsISubscriber) {
    auto                                      name = unique_name("/test_bus_subtype");
    ShmMessageBus                             bus;
    auto                                      pub = bus.advertise<TestPayload>(name);
    std::unique_ptr<ISubscriber<TestPayload>> sub = bus.subscribe<TestPayload>(name, 1, 10);
    EXPECT_NE(sub, nullptr);
}

TEST(ShmMessageBusTest, LazySubscriber) {
    auto          name = unique_name("/test_bus_lazy");
    ShmMessageBus bus;
    auto          pub = bus.advertise<TestPayload>(name);

    auto sub = bus.subscribe_lazy<TestPayload>();
    EXPECT_FALSE(sub->is_connected());
    EXPECT_TRUE(sub->connect(name));
    EXPECT_TRUE(sub->is_connected());
}

TEST(ShmMessageBusTest, MultiplePublishOverwrites) {
    auto          name = unique_name("/test_bus_overwrite");
    ShmMessageBus bus;
    auto          pub = bus.advertise<TestPayload>(name);
    auto          sub = bus.subscribe<TestPayload>(name, 1, 10);

    for (int i = 0; i < 10; ++i) {
        TestPayload msg{static_cast<uint64_t>(i), static_cast<float>(i), {}};
        pub->publish(msg);
    }

    TestPayload latest{};
    ASSERT_TRUE(sub->receive(latest));
    EXPECT_EQ(latest.id, 9u);  // latest-value semantics
}

TEST(ShmMessageBusTest, WithShmTypes) {
    auto          name = unique_name("/test_bus_pose");
    ShmMessageBus bus;
    auto          pub = bus.advertise<ShmPose>(name);
    ASSERT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<ShmPose>(name, 1, 10);
    ASSERT_TRUE(sub->is_connected());

    ShmPose pose{};
    pose.timestamp_ns   = 12345678;
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quality        = 2;
    pub->publish(pose);

    ShmPose received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.timestamp_ns, 12345678u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_EQ(received.quality, 2u);
}

// ═══════════════════════════════════════════════════════════
// IServiceChannel interface tests (no SHM backend needed)
// ═══════════════════════════════════════════════════════════

struct TestRequest {
    uint32_t command{0};
    float    param{0.0f};
};

struct TestResponse {
    uint32_t result{0};
    bool     success{false};
};

TEST(ServiceChannelInterface, ServiceStatusEnum) {
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::OK), 0);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::REJECTED), 1);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::TIMEOUT), 2);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::ERROR), 3);
}

TEST(ServiceChannelInterface, ServiceEnvelopeLayout) {
    ServiceEnvelope<TestRequest> env;
    env.correlation_id  = 42;
    env.timestamp_ns    = 123456;
    env.valid           = true;
    env.payload.command = 7;
    env.payload.param   = 1.5f;

    EXPECT_EQ(env.correlation_id, 42u);
    EXPECT_EQ(env.timestamp_ns, 123456u);
    EXPECT_TRUE(env.valid);
    EXPECT_EQ(env.payload.command, 7u);
    EXPECT_FLOAT_EQ(env.payload.param, 1.5f);
}

TEST(ServiceChannelInterface, ServiceResponseLayout) {
    ServiceResponse<TestResponse> resp;
    resp.correlation_id  = 99;
    resp.timestamp_ns    = 789;
    resp.status          = ServiceStatus::OK;
    resp.valid           = true;
    resp.payload.result  = 100;
    resp.payload.success = true;

    EXPECT_EQ(resp.correlation_id, 99u);
    EXPECT_EQ(resp.status, ServiceStatus::OK);
    EXPECT_TRUE(resp.valid);
    EXPECT_EQ(resp.payload.result, 100u);
    EXPECT_TRUE(resp.payload.success);
}

TEST(ServiceChannelInterface, TriviallyCopyable) {
    EXPECT_TRUE(std::is_trivially_copyable_v<ServiceEnvelope<TestRequest>>);
    EXPECT_TRUE(std::is_trivially_copyable_v<ServiceResponse<TestResponse>>);
}
