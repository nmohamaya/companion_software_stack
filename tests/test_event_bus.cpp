// tests/test_event_bus.cpp — EventBus<Event> unit tests (Issue #293)
#include "util/event_bus.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

// ── Test event types ────────────────────────────────────────────────

struct SimpleEvent {
    int value = 0;
};

struct StringEvent {
    std::string message;
};

// ── Basic functionality ─────────────────────────────────────────────

TEST(EventBus, SubscribeAndPublish) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                received = 0;
    auto sub = bus.subscribe([&](const SimpleEvent& e) { received = e.value; });

    bus.publish(SimpleEvent{42});
    EXPECT_EQ(received, 42);
}

TEST(EventBus, MultipleHandlers) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                sum = 0;
    auto sub1 = bus.subscribe([&](const SimpleEvent& e) { sum += e.value; });
    auto sub2 = bus.subscribe([&](const SimpleEvent& e) { sum += e.value * 10; });

    bus.publish(SimpleEvent{3});
    EXPECT_EQ(sum, 33);  // 3 + 30
}

TEST(EventBus, EmptyBusPublish) {
    drone::util::EventBus<SimpleEvent> bus;
    // Must not crash
    bus.publish(SimpleEvent{99});
    EXPECT_EQ(bus.subscriber_count(), 0u);
}

TEST(EventBus, SubscriberCount) {
    drone::util::EventBus<SimpleEvent> bus;
    EXPECT_EQ(bus.subscriber_count(), 0u);

    auto sub1 = bus.subscribe([](const SimpleEvent&) {});
    EXPECT_EQ(bus.subscriber_count(), 1u);

    auto sub2 = bus.subscribe([](const SimpleEvent&) {});
    EXPECT_EQ(bus.subscriber_count(), 2u);
}

// ── RAII unsubscribe ────────────────────────────────────────────────

TEST(EventBus, RAIIUnsubscribe) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                count = 0;
    {
        auto sub = bus.subscribe([&](const SimpleEvent&) { ++count; });
        bus.publish(SimpleEvent{});
        EXPECT_EQ(count, 1);
        EXPECT_EQ(bus.subscriber_count(), 1u);
    }
    // sub went out of scope — handler should be removed
    bus.publish(SimpleEvent{});
    EXPECT_EQ(count, 1);  // no additional call
    EXPECT_EQ(bus.subscriber_count(), 0u);
}

TEST(EventBus, ExplicitUnsubscribe) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                count = 0;
    auto                               sub   = bus.subscribe([&](const SimpleEvent&) { ++count; });

    bus.publish(SimpleEvent{});
    EXPECT_EQ(count, 1);

    sub.unsubscribe();
    EXPECT_FALSE(sub.is_active());

    bus.publish(SimpleEvent{});
    EXPECT_EQ(count, 1);  // not called after unsubscribe
}

TEST(EventBus, DoubleUnsubscribeIsSafe) {
    drone::util::EventBus<SimpleEvent> bus;
    auto                               sub = bus.subscribe([](const SimpleEvent&) {});
    sub.unsubscribe();
    sub.unsubscribe();  // Must not crash or double-free
    EXPECT_EQ(bus.subscriber_count(), 0u);
}

// ── Move semantics on Subscription ──────────────────────────────────

TEST(EventBus, MoveConstructSubscription) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                count = 0;
    auto                               sub1  = bus.subscribe([&](const SimpleEvent&) { ++count; });

    auto sub2(std::move(sub1));
    EXPECT_FALSE(sub1.is_active());  // NOLINT — testing moved-from state
    EXPECT_TRUE(sub2.is_active());

    bus.publish(SimpleEvent{});
    EXPECT_EQ(count, 1);
    EXPECT_EQ(bus.subscriber_count(), 1u);
}

TEST(EventBus, MoveAssignSubscription) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                count_a = 0;
    int                                count_b = 0;
    auto sub_a = bus.subscribe([&](const SimpleEvent&) { ++count_a; });
    auto sub_b = bus.subscribe([&](const SimpleEvent&) { ++count_b; });
    EXPECT_EQ(bus.subscriber_count(), 2u);

    // Move-assign sub_a into sub_b — sub_b's old handler should be unsubscribed
    sub_b = std::move(sub_a);
    EXPECT_EQ(bus.subscriber_count(), 1u);

    bus.publish(SimpleEvent{});
    EXPECT_EQ(count_a, 1);  // sub_a's handler still active (now owned by sub_b)
    EXPECT_EQ(count_b, 0);  // sub_b's old handler was unsubscribed
}

TEST(EventBus, DefaultConstructedSubscriptionInactive) {
    drone::util::Subscription<SimpleEvent> sub;
    EXPECT_FALSE(sub.is_active());
    // Destroying default-constructed subscription must not crash
}

// ── Re-entrancy (publish from within handler) ───────────────────────

TEST(EventBus, PublishFromWithinHandler) {
    drone::util::EventBus<SimpleEvent> bus;
    int                                outer_count = 0;
    int                                inner_count = 0;

    auto inner_sub = bus.subscribe([&](const SimpleEvent&) { ++inner_count; });

    auto outer_sub = bus.subscribe([&](const SimpleEvent& e) {
        ++outer_count;
        if (e.value == 1) {
            // Re-entrant publish — must not deadlock
            bus.publish(SimpleEvent{2});
        }
    });

    bus.publish(SimpleEvent{1});
    // First publish: both handlers called with value=1
    // Re-entrant publish(2): both handlers called with value=2
    EXPECT_EQ(outer_count, 2);
    EXPECT_EQ(inner_count, 2);
}

// ── Unsubscribe during iteration (safe due to copy-on-write) ────────

TEST(EventBus, UnsubscribeDuringIteration) {
    drone::util::EventBus<SimpleEvent>     bus;
    int                                    count = 0;
    drone::util::Subscription<SimpleEvent> sub2;
    // Handler that unsubscribes sub2 during publish iteration
    auto sub1 = bus.subscribe([&](const SimpleEvent&) {
        ++count;
        sub2.unsubscribe();  // Modifies handler list while iterating snapshot
    });
    sub2      = bus.subscribe([&](const SimpleEvent&) { ++count; });

    bus.publish(SimpleEvent{});
    // Both handlers were in the snapshot, so both should have been called
    EXPECT_EQ(count, 2);
    // But sub2 is now unsubscribed
    EXPECT_EQ(bus.subscriber_count(), 1u);
}

// ── Thread safety ───────────────────────────────────────────────────

TEST(EventBus, ConcurrentPublish) {
    drone::util::EventBus<SimpleEvent> bus;
    std::atomic<int>                   total{0};

    auto sub = bus.subscribe(
        [&](const SimpleEvent& e) { total.fetch_add(e.value, std::memory_order_relaxed); });

    constexpr int            kThreads   = 4;
    constexpr int            kPerThread = 1000;
    std::vector<std::thread> threads;
    threads.reserve(kThreads);

    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&bus]() {
            for (int i = 0; i < kPerThread; ++i) {
                bus.publish(SimpleEvent{1});
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(total.load(std::memory_order_relaxed), kThreads * kPerThread);
}

TEST(EventBus, ConcurrentSubscribeUnsubscribe) {
    drone::util::EventBus<SimpleEvent> bus;
    std::atomic<int>                   total{0};

    constexpr int            kThreads = 4;
    constexpr int            kIters   = 500;
    std::vector<std::thread> threads;
    threads.reserve(kThreads);

    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&bus, &total]() {
            for (int i = 0; i < kIters; ++i) {
                auto sub = bus.subscribe([&](const SimpleEvent& e) {
                    total.fetch_add(e.value, std::memory_order_relaxed);
                });
                bus.publish(SimpleEvent{1});
                // sub destroyed here — RAII unsubscribe
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }

    // Each iteration: subscribe, publish (at least own handler sees it), unsubscribe.
    // Total should be >= kThreads * kIters (could be more due to interleaving).
    EXPECT_GE(total.load(std::memory_order_relaxed), kThreads * kIters);
    EXPECT_EQ(bus.subscriber_count(), 0u);
}

// ── Different event types are independent ───────────────────────────

TEST(EventBus, DifferentEventTypes) {
    drone::util::EventBus<SimpleEvent> int_bus;
    drone::util::EventBus<StringEvent> str_bus;

    int         int_count = 0;
    std::string last_msg;

    auto sub1 = int_bus.subscribe([&](const SimpleEvent& e) { int_count += e.value; });
    auto sub2 = str_bus.subscribe([&](const StringEvent& e) { last_msg = e.message; });

    int_bus.publish(SimpleEvent{7});
    str_bus.publish(StringEvent{"hello"});

    EXPECT_EQ(int_count, 7);
    EXPECT_EQ(last_msg, "hello");
}

// ── Subscribe after publish ─────────────────────────────────────────

TEST(EventBus, LateSubscriberMissesEarlierEvents) {
    drone::util::EventBus<SimpleEvent> bus;
    bus.publish(SimpleEvent{1});

    int  received = 0;
    auto sub      = bus.subscribe([&](const SimpleEvent& e) { received = e.value; });

    EXPECT_EQ(received, 0);  // Late subscriber did not see earlier event

    bus.publish(SimpleEvent{2});
    EXPECT_EQ(received, 2);
}

// ── Multiple publishes ──────────────────────────────────────────────

TEST(EventBus, MultiplePublishes) {
    drone::util::EventBus<SimpleEvent> bus;
    std::vector<int>                   received;

    auto sub = bus.subscribe([&](const SimpleEvent& e) { received.push_back(e.value); });

    bus.publish(SimpleEvent{1});
    bus.publish(SimpleEvent{2});
    bus.publish(SimpleEvent{3});

    ASSERT_EQ(received.size(), 3u);
    EXPECT_EQ(received[0], 1);
    EXPECT_EQ(received[1], 2);
    EXPECT_EQ(received[2], 3);
}

// ── Lifetime: EventBus destroyed before Subscription ────────────────
// This test verifies the debug assert fires (in debug builds) or at minimum
// documents the contract.  In release builds without asserts, this would be
// UB — the test documents the expected usage constraint.

TEST(EventBus, DestroyBusBeforeSubscription_DebugAssertDocumented) {
    // Construct Subscription in outer scope, EventBus in inner scope.
    // In debug builds, ~EventBus() asserts handlers_.empty().
    // We test the CORRECT usage pattern: unsubscribe before bus destruction.
    drone::util::Subscription<SimpleEvent> sub;
    {
        drone::util::EventBus<SimpleEvent> bus;
        sub = bus.subscribe([](const SimpleEvent&) {});
        EXPECT_TRUE(sub.is_active());

        // Must unsubscribe before bus goes out of scope
        sub.unsubscribe();
        EXPECT_FALSE(sub.is_active());
        EXPECT_EQ(bus.subscriber_count(), 0u);
    }
    // Bus destroyed with no live subscriptions — safe.
    // Destroying sub again is a no-op (bus_ is nullptr after unsubscribe).
}

// ── Concurrent publish + explicit unsubscribe stress test ───────────

TEST(EventBus, ConcurrentPublishAndUnsubscribe) {
    drone::util::EventBus<SimpleEvent> bus;
    std::atomic<int>                   total{0};
    constexpr int                      kIters = 2000;

    // One thread publishes continuously
    std::atomic<bool> stop{false};
    std::thread       publisher([&bus, &stop]() {
        while (!stop.load(std::memory_order_acquire)) {
            bus.publish(SimpleEvent{1});
        }
    });

    // Main thread rapidly subscribes and unsubscribes
    for (int i = 0; i < kIters; ++i) {
        auto sub = bus.subscribe([&total](const SimpleEvent& e) {
            total.fetch_add(e.value, std::memory_order_relaxed);
        });
        // Explicitly unsubscribe while publisher may be mid-publish
        sub.unsubscribe();
    }

    stop.store(true, std::memory_order_release);
    publisher.join();

    // No crash, no TSAN violations = pass.  Total is non-deterministic.
    EXPECT_GE(total.load(std::memory_order_relaxed), 0);
    EXPECT_EQ(bus.subscriber_count(), 0u);
}

}  // namespace
