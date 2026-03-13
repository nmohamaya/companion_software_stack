// common/ipc/include/ipc/shm_message_bus.h
// ShmMessageBus — factory for SHM-backed publishers and subscribers.
//
// Usage:
//   drone::ipc::ShmMessageBus bus;
//   auto pub = bus.advertise<VideoFrame>("/drone_mission_cam");
//   auto sub = bus.subscribe<Pose>("/slam_pose");
//   pub->publish(frame);
//   Pose pose; sub->receive(pose);
//
// The bus is a lightweight factory — it holds no state beyond what the
// individual publishers/subscribers own.  Swapping the entire transport
// (e.g. to DDS) means replacing ShmMessageBus with DdsMessageBus and
// recompiling; process code depends only on IPublisher<T> / ISubscriber<T>.
#pragma once

#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"
#include "ipc/shm_publisher.h"
#include "ipc/shm_subscriber.h"

#include <memory>
#include <string>

namespace drone::ipc {

/// Factory that creates SHM-backed publishers and subscribers.
/// Template methods cannot be virtual, so this is a concrete class.
/// To support runtime transport switching, wrap in a type-erased factory
/// or use compile-time policies.
class ShmMessageBus {
public:
    /// Create a publisher for the given topic.
    /// The returned publisher owns the SHM segment and unlinks on destruction.
    template<typename T>
    std::unique_ptr<IPublisher<T>> advertise(const std::string& topic) {
        return std::make_unique<ShmPublisher<T>>(topic);
    }

    /// Create a subscriber for the given topic.
    /// Retries connecting up to max_retries times.
    template<typename T>
    std::unique_ptr<ISubscriber<T>> subscribe(const std::string& topic, int max_retries = 50,
                                              int retry_ms = 200) {
        return std::make_unique<ShmSubscriber<T>>(topic, max_retries, retry_ms);
    }

    /// Create a lazy subscriber (no immediate connection attempt).
    /// Call connect() on the returned subscriber when ready.
    template<typename T>
    std::unique_ptr<ShmSubscriber<T>> subscribe_lazy() {
        return std::make_unique<ShmSubscriber<T>>();
    }
};

}  // namespace drone::ipc
