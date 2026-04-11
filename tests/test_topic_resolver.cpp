// tests/test_topic_resolver.cpp
// Unit tests for TopicResolver and MessageBus topic namespacing.
#include "ipc/ipc_types.h"
#include "ipc/message_bus_factory.h"
#include "ipc/topic_resolver.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// TopicResolver unit tests (no Zenoh session needed)
// ═══════════════════════════════════════════════════════════

TEST(TopicResolverTest, EmptyVehicleIdNoPrefix) {
    TopicResolver resolver;
    EXPECT_EQ(resolver.resolve("/slam_pose"), "/slam_pose");
    EXPECT_EQ(resolver.resolve("/fc_commands"), "/fc_commands");
    EXPECT_EQ(resolver.resolve("/detected_objects"), "/detected_objects");
}

TEST(TopicResolverTest, EmptyStringVehicleIdNoPrefix) {
    TopicResolver resolver("");
    EXPECT_EQ(resolver.resolve("/slam_pose"), "/slam_pose");
    EXPECT_FALSE(resolver.has_prefix());
    EXPECT_EQ(resolver.vehicle_id(), "");
}

TEST(TopicResolverTest, VehicleIdPrefixApplied) {
    TopicResolver resolver("drone42");
    EXPECT_EQ(resolver.resolve("/slam_pose"), "/drone42/slam_pose");
    EXPECT_EQ(resolver.resolve("/fc_commands"), "/drone42/fc_commands");
    EXPECT_EQ(resolver.resolve("/detected_objects"), "/drone42/detected_objects");
}

TEST(TopicResolverTest, HasPrefixTrue) {
    TopicResolver resolver("vehicle_a");
    EXPECT_TRUE(resolver.has_prefix());
    EXPECT_EQ(resolver.vehicle_id(), "vehicle_a");
}

TEST(TopicResolverTest, HasPrefixFalse) {
    TopicResolver resolver;
    EXPECT_FALSE(resolver.has_prefix());
}

TEST(TopicResolverTest, PreservesLeadingSlash) {
    TopicResolver resolver("v1");
    const auto    resolved = resolver.resolve("/topic");
    EXPECT_EQ(resolved, "/v1/topic");
    EXPECT_EQ(resolved[0], '/');
}

TEST(TopicResolverTest, TopicWithoutLeadingSlash) {
    // Without a leading slash, resolve() inserts a '/' separator automatically.
    TopicResolver resolver("drone1");
    EXPECT_EQ(resolver.resolve("topic"), "/drone1/topic");
}

TEST(TopicResolverTest, EmptyBaseTopic) {
    TopicResolver resolver("drone1");
    EXPECT_EQ(resolver.resolve(""), "/drone1");

    TopicResolver resolver_empty;
    EXPECT_EQ(resolver_empty.resolve(""), "");
}

TEST(TopicResolverTest, MoveConstruction) {
    TopicResolver a("drone42");
    TopicResolver b(std::move(a));
    EXPECT_EQ(b.resolve("/slam_pose"), "/drone42/slam_pose");
    EXPECT_TRUE(b.has_prefix());
}

// ═══════════════════════════════════════════════════════════
// Test data struct (trivially copyable)
// ═══════════════════════════════════════════════════════════
struct ResolverTestPayload {
    uint64_t id{0};
    float    value{0.0f};
};

/// Generate a unique topic name per test to avoid Zenoh cross-test interference.
static std::string unique_topic(const char* base) {
    static std::atomic<uint32_t> counter{0};
    return std::string(base) + "_" + std::to_string(::getpid()) + "_" +
           std::to_string(counter.fetch_add(1));
}

// ═══════════════════════════════════════════════════════════
// Vehicle ID validation (negative tests)
// ═══════════════════════════════════════════════════════════

TEST(TopicResolverTest, RejectsSlashInVehicleId) {
    EXPECT_THROW(TopicResolver("a/b"), std::invalid_argument);
}

TEST(TopicResolverTest, RejectsSpaceInVehicleId) {
    EXPECT_THROW(TopicResolver("drone 1"), std::invalid_argument);
}

TEST(TopicResolverTest, RejectsDotInVehicleId) {
    EXPECT_THROW(TopicResolver("../admin"), std::invalid_argument);
}

TEST(TopicResolverTest, AcceptsDashAndUnderscore) {
    TopicResolver r("drone-42_alpha");
    EXPECT_EQ(r.resolve("/topic"), "/drone-42_alpha/topic");
}

// ═══════════════════════════════════════════════════════════
// MessageBus integration: default resolver (no prefix)
// ═══════════════════════════════════════════════════════════

TEST(TopicResolverBusTest, DefaultResolverNoPrefix) {
    auto bus = create_message_bus("zenoh");
    EXPECT_FALSE(bus.topic_resolver().has_prefix());
    EXPECT_EQ(bus.topic_resolver().vehicle_id(), "");
}

TEST(TopicResolverBusTest, SetResolverPersists) {
    auto bus = create_message_bus("zenoh");
    bus.set_topic_resolver(TopicResolver("fleet7"));
    EXPECT_TRUE(bus.topic_resolver().has_prefix());
    EXPECT_EQ(bus.topic_resolver().vehicle_id(), "fleet7");
}

// ═══════════════════════════════════════════════════════════
// End-to-end: pub/sub through namespaced bus
// ═══════════════════════════════════════════════════════════

TEST(TopicResolverBusTest, NamespacedPubSubRoundTrip) {
    auto base_topic = unique_topic("/test_resolver_rt");

    auto bus = create_message_bus("zenoh");
    bus.set_topic_resolver(TopicResolver("drone99"));

    auto pub = bus.advertise<ResolverTestPayload>(base_topic);
    auto sub = bus.subscribe<ResolverTestPayload>(base_topic);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    ResolverTestPayload sent{42, 3.14f};
    pub->publish(sent);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ResolverTestPayload received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.id, 42u);
    EXPECT_FLOAT_EQ(received.value, 3.14f);
}

// ═══════════════════════════════════════════════════════════
// Multi-namespace isolation: two buses, different vehicle_ids
// ═══════════════════════════════════════════════════════════

TEST(TopicResolverBusTest, MultiNamespaceIsolation) {
    auto base_topic = unique_topic("/test_resolver_iso");

    auto bus_a = create_message_bus("zenoh");
    bus_a.set_topic_resolver(TopicResolver("alpha"));

    auto bus_b = create_message_bus("zenoh");
    bus_b.set_topic_resolver(TopicResolver("bravo"));

    // Subscribe on both namespaces
    auto sub_a = bus_a.subscribe<ResolverTestPayload>(base_topic);
    auto sub_b = bus_b.subscribe<ResolverTestPayload>(base_topic);

    // Publish only on alpha
    auto                pub_a = bus_a.advertise<ResolverTestPayload>(base_topic);
    ResolverTestPayload msg{77, 1.0f};
    pub_a->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Alpha should receive the message
    ResolverTestPayload recv_a{};
    EXPECT_TRUE(sub_a->receive(recv_a));
    EXPECT_EQ(recv_a.id, 77u);

    // Bravo should NOT receive (different namespace)
    ResolverTestPayload recv_b{};
    EXPECT_FALSE(sub_b->receive(recv_b));
}
