// common/util/include/util/event_bus.h
// Lightweight per-process EventBus for intra-process declarative composition.
//
// Enables patterns like "on obstacle detected within 3m, trigger payload capture"
// without a full reactive framework.  IPC remains pull-based (correct for real-time).
//
// Thread safety:
//   - subscribe(), unsubscribe (via Subscription destructor), and publish()
//     are all safe to call concurrently from any thread.
//   - publish() uses copy-on-write: copies the handler list under lock, then
//     iterates the copy outside the lock.  This allows publish-from-within-handler
//     (re-entrancy) and unsubscribe-during-iteration without deadlock.
//
// Concurrency tier: std::mutex (non-hot-path shared state).
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

namespace drone::util {

// Forward declaration — Subscription must know EventBus<Event>.
template<typename Event>
class EventBus;

/// RAII subscription token.  Automatically unsubscribes on destruction.
/// Move-only — copying would create double-unsubscribe bugs.
template<typename Event>
class Subscription {
public:
    Subscription() = default;

    ~Subscription() { unsubscribe(); }

    // Move-only
    Subscription(Subscription&& other) noexcept : bus_(other.bus_), id_(other.id_) {
        other.bus_ = nullptr;
        other.id_  = 0;
    }

    Subscription& operator=(Subscription&& other) noexcept {
        if (this != &other) {
            unsubscribe();
            bus_       = other.bus_;
            id_        = other.id_;
            other.bus_ = nullptr;
            other.id_  = 0;
        }
        return *this;
    }

    Subscription(const Subscription&)            = delete;
    Subscription& operator=(const Subscription&) = delete;

    /// Check if this subscription is active.
    [[nodiscard]] bool is_active() const { return bus_ != nullptr; }

    /// Explicitly unsubscribe (also called by destructor).
    void unsubscribe() {
        if (bus_) {
            bus_->remove_handler(id_);
            bus_ = nullptr;
            id_  = 0;
        }
    }

private:
    friend class EventBus<Event>;

    Subscription(EventBus<Event>* bus, uint64_t id) : bus_(bus), id_(id) {}

    EventBus<Event>* bus_ = nullptr;
    uint64_t         id_  = 0;
};

/// Lightweight typed event bus for intra-process composition.
///
/// Usage:
///   struct ObstacleDetected { float distance_m; };
///   EventBus<ObstacleDetected> bus;
///   auto sub = bus.subscribe([](const ObstacleDetected& e) {
///       if (e.distance_m < 3.0f) trigger_capture();
///   });
///   bus.publish(ObstacleDetected{2.5f});
template<typename Event>
class EventBus {
public:
    EventBus()  = default;
    ~EventBus() = default;

    // Non-copyable, non-movable (subscriptions hold raw pointer to bus).
    EventBus(const EventBus&)            = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&)                 = delete;
    EventBus& operator=(EventBus&&)      = delete;

    /// Subscribe a handler.  Returns an RAII Subscription token that
    /// automatically unsubscribes when destroyed or moved-from.
    [[nodiscard]] Subscription<Event> subscribe(std::function<void(const Event&)> handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t              id = next_id_++;
        handlers_.push_back({id, std::move(handler)});
        return Subscription<Event>(this, id);
    }

    /// Publish an event to all current subscribers.
    /// Safe to call from within a handler (re-entrant) and from any thread.
    void publish(const Event& event) {
        // Copy-on-write: snapshot handler list under lock, iterate outside.
        std::vector<HandlerEntry> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot = handlers_;
        }
        for (const auto& entry : snapshot) {
            entry.handler(event);
        }
    }

    /// Number of active subscriptions (for testing/diagnostics).
    [[nodiscard]] size_t subscriber_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return handlers_.size();
    }

private:
    friend class Subscription<Event>;

    struct HandlerEntry {
        uint64_t                          id      = 0;
        std::function<void(const Event&)> handler = nullptr;
    };

    /// Remove a handler by subscription ID.  Called by Subscription destructor.
    void remove_handler(uint64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        handlers_.erase(std::remove_if(handlers_.begin(), handlers_.end(),
                                       [id](const HandlerEntry& e) { return e.id == id; }),
                        handlers_.end());
    }

    mutable std::mutex        mutex_;
    std::vector<HandlerEntry> handlers_;
    uint64_t                  next_id_ = 1;
};

}  // namespace drone::util
