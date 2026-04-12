// tests/test_message_bus.cpp
// Tests for MessageBus interface, IPublisher/ISubscriber contracts,
// and IServiceChannel interface types (enums, structs).
//
// Zenoh pub/sub and service channel round-trip tests are in test_zenoh_ipc.cpp.
#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "ipc/iservice_channel.h"
#include "ipc/isubscriber.h"
#include "ipc/message_bus_factory.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

using namespace drone::ipc;

/// Generate a unique topic name per test to avoid Zenoh cross-test interference
/// under parallel ctest execution.
static std::string unique_topic(const char* base) {
    static std::atomic<uint32_t> counter{0};
    return std::string(base) + "_" + std::to_string(::getpid()) + "_" +
           std::to_string(counter.fetch_add(1));
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
// MessageBus factory tests
// ═══════════════════════════════════════════════════════════

TEST(MessageBusTest, FactoryCreatesZenoh) {
    auto bus = create_message_bus("zenoh");
    EXPECT_EQ(bus.backend_name(), "zenoh");
    EXPECT_TRUE(bus.has_service_channels());
}

TEST(MessageBusTest, FactoryDefaultIsZenoh) {
    auto bus = create_message_bus();
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(MessageBusTest, FactoryFallsBackFromShm) {
    // "shm" is removed — should log error and fall back to zenoh
    auto bus = create_message_bus("shm");
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(MessageBusTest, AdvertiseAndSubscribe) {
    auto topic = unique_topic("/test_mbus_advsub");
    auto bus   = create_message_bus("zenoh");
    auto pub   = bus.advertise<TestPayload>(topic);
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<TestPayload>(topic);
    ASSERT_NE(sub, nullptr);
    EXPECT_TRUE(sub->is_connected());
}

TEST(MessageBusTest, PubSubRoundTrip) {
    auto topic = unique_topic("/test_mbus_rt");
    auto bus   = create_message_bus("zenoh");
    auto pub   = bus.advertise<TestPayload>(topic);
    auto sub   = bus.subscribe<TestPayload>(topic);

    TestPayload sent{42, 3.14f, {}};
    std::strncpy(sent.tag, "test", sizeof(sent.tag));
    pub->publish(sent);

    // Allow Zenoh delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    TestPayload received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.id, 42u);
    EXPECT_FLOAT_EQ(received.value, 3.14f);
    EXPECT_STREQ(received.tag, "test");
}

TEST(MessageBusTest, WithIpcTypes) {
    auto topic = unique_topic("/test_mbus_pose");
    auto bus   = create_message_bus("zenoh");
    auto pub   = bus.advertise<Pose>(topic);
    ASSERT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<Pose>(topic);
    ASSERT_TRUE(sub->is_connected());

    Pose pose{};
    pose.timestamp_ns   = 12345678;
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quality        = 2;
    pub->publish(pose);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    Pose received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.timestamp_ns, 12345678u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_EQ(received.quality, 2u);
}

// ═══════════════════════════════════════════════════════════
// IServiceChannel interface tests (no backend needed)
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
