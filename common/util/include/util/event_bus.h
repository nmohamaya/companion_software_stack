// common/util/include/util/event_bus.h
// Lightweight per-process EventBus for intra-process declarative composition.
//
// Enables patterns like "on obstacle detected within 3m, trigger payload capture"
// without a full reactive framework.  IPC remains pull-based (correct for real-time).
//
// Thread safety:
//   - subscribe(), publish(), and remove_handler() (via Subscription destructor)
//     are all safe to call concurrently from any thread.
//   - publish() uses snapshot-copy: copies the handler list under lock on every
//     call, then iterates the copy outside the lock.  This allows publish-from-
//     within-handler (re-entrancy) and unsubscribe-during-iteration without deadlock.
//
// Lifetime contract:
//   The EventBus MUST outlive all Subscription objects derived from it.
//   Destroying an EventBus while Subscriptions still exist is undefined behavior.
//   In debug builds, ~EventBus() asserts that all subscriptions have been released.
//
// Subscription is NOT thread-safe — a single Subscription must be accessed from
// one thread at a time (or externally synchronized).  This is consistent with
// RAII tokens being stack-local or owned by a single component.
//
// Concurrency tier: std::mutex (non-hot-path shared state).
#pragma once

#include <algorithm>
#include <cassert>
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
///
/// NOT thread-safe: a single Subscription instance must be accessed from one
/// thread at a time (typical for stack-local RAII tokens).
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
    /// NOT thread-safe — must not be called concurrently on the same Subscription.
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
/// Lifetime: The EventBus MUST outlive all Subscription tokens derived from it.
/// In debug builds, the destructor asserts that all subscriptions have been released.
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
    EventBus() = default;

    ~EventBus() {
        // All Subscriptions must be destroyed before the EventBus.
        // If this fires, a Subscription outlived its bus — fix the ownership.
        assert(handlers_.empty() && "EventBus destroyed with live subscriptions — "
                                    "all Subscription tokens must be destroyed first");
    }

    // Non-copyable, non-movable (subscriptions hold raw pointer to bus).
    EventBus(const EventBus&)            = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&)                 = delete;
    EventBus& operator=(EventBus&&)      = delete;

    /// Subscribe a handler.  Returns an RAII Subscription token that
    /// automatically unsubscribes when destroyed or moved-from.
    /// The handler must be non-empty (callable).
    [[nodiscard]] Subscription<Event> subscribe(std::function<void(const Event&)> handler) {
        assert(handler && "subscribe() called with empty handler");
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t              id = next_id_++;
        handlers_.push_back({id, std::move(handler)});
        return Subscription<Event>(this, id);
    }

    /// Publish an event to all current subscribers.
    /// Safe to call from within a handler (re-entrant) and from any thread.
    void publish(const Event& event) {
        // Snapshot-copy: copy handler list under lock, iterate outside.
        // This is O(N) per publish where N = subscriber count.  Acceptable for
        // non-hot-path usage.  For hot paths, consider shared_mutex or a
        // generation-counter optimization to skip the copy when unchanged.
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
