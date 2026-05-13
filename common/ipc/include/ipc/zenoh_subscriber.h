// common/ipc/include/ipc/zenoh_subscriber.h
// Zenoh-backed subscriber — wraps zenoh::Subscriber behind ISubscriber<T>.
//
// Receives T from raw bytes via Zenoh callback, using a pluggable
// ISerializer<T> for wire-format decoding.  The Zenoh callback runs on
// an internal Zenoh thread; receive() is called from the process main
// loop.  Thread-safety is provided by atomics + mutex.
//
// The serializer defaults to RawSerializer<T> (byte-identical to the
// previous inline reinterpret_cast + std::copy), preserving full backward
// compatibility.
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/iserializer.h"
#include "ipc/isubscriber.h"
#include "ipc/raw_serializer.h"
#include "ipc/zenoh_session.h"
#include "util/iclock.h"
#include "util/ilogger.h"
#include "util/latency_tracker.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed subscriber implementing ISubscriber<T>.
/// Maintains a latest-value cache updated by the Zenoh callback thread.
template<typename T>
class ZenohSubscriber final : public ISubscriber<T> {
public:
    /// Construct and declare a Zenoh subscriber on the given key expression.
    /// @param key_expr        Zenoh key expression (e.g. "drone/slam/pose").
    /// @param track_latency   Enable IPC latency tracking (default: true).
    /// @param serializer      Optional serializer (default: RawSerializer<T>).
    ///                        shared_ptr because MessageBus may share one
    ///                        instance across multiple subscribers for the
    ///                        same type.
    /// @param filter_pre_birth_messages  Issue #722 — when `true` (default),
    ///                                   drop incoming messages whose
    ///                                   `T::timestamp_ns` predates this
    ///                                   subscriber's construction by more
    ///                                   than `kBirthSlackNs`.  Defends
    ///                                   every timestamped IPC topic against
    ///                                   Zenoh's last-value cache delivering
    ///                                   historic messages from a previous
    ///                                   publisher session (root cause of
    ///                                   #720, originally patched per-site
    ///                                   by Issue #721).  Tests that
    ///                                   legitimately replay historic
    ///                                   messages should pass `false`.
    ///
    /// **Coverage caveat.**  The filter is implemented inside the
    /// `has_validate<T>` branch of `on_sample`, so it only fires for types
    /// with BOTH `validate()` AND `timestamp_ns`.  All currently-subscribed
    /// safety-critical IPC types satisfy both (Pose, FCState,
    /// DetectedObjectList, TrajectoryCmd, FCCommand, GCSCommand,
    /// PayloadCommand, PayloadStatus, MissionStatus, SystemHealth,
    /// SemanticVoxelBatch, RadarDetectionList).  A type with `timestamp_ns`
    /// but no `validate()` (currently `ThreadHealth`, which is publisher-
    /// side only — never subscribed via this wrapper) would pass through
    /// unfiltered.  If you add a new subscribed type with `timestamp_ns`,
    /// give it a `validate()` to opt it into the filter.
    explicit ZenohSubscriber(
        const std::string& key_expr, bool track_latency = true,
        std::shared_ptr<const ISerializer<T>> serializer = std::make_shared<RawSerializer<T>>(),
        bool                                  filter_pre_birth_messages = true)
        : key_expr_(key_expr)
        , track_latency_(track_latency)
        , serializer_(serializer ? std::move(serializer) : std::make_shared<RawSerializer<T>>())
        , filter_pre_birth_messages_(filter_pre_birth_messages)
        , subscriber_birth_ns_(drone::util::get_clock().now_ns()) {
        try {
            auto& session = ZenohSession::instance().session();
            subscriber_.emplace(session.declare_subscriber(
                zenoh::KeyExpr(key_expr), [this](zenoh::Sample& sample) { on_sample(sample); },
                []() { /* on_drop — no-op */ }));
            DRONE_LOG_INFO("[ZenohSubscriber] Subscribed to '{}' (serializer={})", key_expr,
                           serializer_->name());
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[ZenohSubscriber] Failed to subscribe to '{}': {}", key_expr,
                            e.what());
        }
    }

    /// Read the latest message.
    /// @param out          Destination for the payload.
    /// @param timestamp_ns Optional output — the time (steady_clock, ns)
    ///                     at which the Zenoh callback delivered the sample.
    ///                     NOTE: this is the *callback arrival* time, not
    ///                     the publisher's send time.  ZenohPublisher does
    ///                     not embed a send timestamp, so the latency
    ///                     recorded here reflects callback→receive() delay
    ///                     (consumer polling lag), NOT wire latency.
    /// @return true if a message was available, false otherwise.
    [[nodiscard]] bool receive(T& out, uint64_t* timestamp_ns = nullptr) const override {
        if (!has_data_.load(std::memory_order_acquire)) return false;

        // Copy under lock (protects against concurrent callback)
        uint64_t msg_ts = 0;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            out    = latest_msg_;
            msg_ts = timestamp_ns_;
        }
        if (timestamp_ns) {
            *timestamp_ns = msg_ts;
        }
        // Only record a latency sample when the message timestamp has changed
        // since the previous receive().  Without this guard, a sticky-true
        // has_data_ + an unchanged timestamp_ns_ (publisher silent) makes every
        // poll record `now - last_callback_time`, which grows linearly with
        // wall-clock and looks like an unbounded queue backlog.  Bug surfaced
        // during scenario-33 forensics on 2026-04-30 when fc_command latency
        // appeared to climb to 220s but the publisher was idle after TAKEOFF.
        //
        // PR #674 P1 review (refined by PR #691 Copilot): relaxed memory
        // ordering is correct here because `last_recorded_ts_` is only
        // ever loaded/stored from `receive()`, which is called from a
        // SINGLE consumer thread per the `ISubscriber<T>::receive()`
        // contract (documented in `isubscriber.h`).  `on_sample()` (Zenoh
        // callback thread) NEVER touches this field — only `latest_msg_`
        // and `timestamp_ns_`, both guarded by `data_mutex_`.
        //
        // Because access is single-threaded by the contract, technically
        // no atomic is needed at all (same-thread reads of one's own
        // previous writes do not constitute a data race per C++17
        // [intro.races]).  The atomic remains for two reasons: (a)
        // defence-in-depth against a future caller that violates the
        // single-consumer contract — relaxed atomics on x86 / aarch64
        // compile to plain loads/stores so there's zero hot-path cost,
        // and (b) it documents the field's threading semantics directly
        // in the type rather than only in a comment.
        if (track_latency_ && msg_ts > 0 &&
            msg_ts != last_recorded_ts_.load(std::memory_order_relaxed)) {
            last_recorded_ts_.store(msg_ts, std::memory_order_relaxed);
            uint64_t now = drone::util::LatencyTracker::now_ns();
            if (now > msg_ts) {
                latency_tracker_.record(now - msg_ts);
            }
        }
        return true;
    }

    /// Returns true if the Zenoh subscriber was successfully declared.
    /// Unlike POSIX SHM (which requires the segment to exist), Zenoh
    /// subscriptions are valid immediately — data arrives asynchronously
    /// once a publisher appears.  Use receive() to check for actual data.
    [[nodiscard]] bool is_connected() const override { return subscriber_.has_value(); }

    [[nodiscard]] const std::string& topic_name() const override { return key_expr_; }

    /// Access the latency tracker for periodic reporting.
    /// PR #674 / PR #691 Copilot review: returns a `const&` to enforce
    /// the dedup invariant in the type system.  Previously returned a
    /// non-const reference, which let callers directly invoke
    /// `latency_tracker().record(...)` and bypass the timestamp-change
    /// dedup maintained by receive().  All current callers
    /// (`log_latency_if_due`, run-report) only need read access for
    /// summary stats — `LatencyTracker`'s read-side methods (`p50()`,
    /// `p95()`, `count()`, `clear()`) work on a `const` instance.
    /// The single producer of samples is `receive()` (which holds the
    /// non-const `latency_tracker_` directly).
    [[nodiscard]] const drone::util::LatencyTracker& latency_tracker() const {
        return latency_tracker_;
    }

    /// Log latency summary if enough samples have been collected.
    bool log_latency_if_due(size_t min_samples = 100) const override {
        if (!track_latency_) return false;
        return latency_tracker_.log_summary_if_due(key_expr_, min_samples);
    }

private:
    /// Zenoh callback — runs on Zenoh internal thread.
    /// Stamps timestamp_ns_ with the arrival time so that receive()
    /// can measure *callback→poll* delay (not true wire latency).
    /// Rejects messages that fail structural validation (Issues #179, #181, #185).
    void on_sample(zenoh::Sample& sample) {
        const auto& payload = sample.get_payload();
        auto        bytes   = payload.as_vector();

        // Validate into a heap-allocated temporary before committing to
        // latest_msg_, so a failed validation never overwrites a previously
        // good value.  Heap allocation avoids stack overflow for large types
        // (e.g. VideoFrame ~6.2 MB) on Zenoh's callback thread.
        if constexpr (has_validate<T>::value) {
            auto temp = std::make_unique<T>();
            if (!serializer_->deserialize(bytes.data(), bytes.size(), *temp)) {
                DRONE_LOG_WARN("[ZenohSubscriber] Deserialization failed on '{}': "
                               "expected {} got {}",
                               key_expr_, sizeof(T), bytes.size());
                return;
            }
            if (!temp->validate()) {
                DRONE_LOG_WARN("[ZenohSubscriber] Validation failed on '{}' — "
                               "dropping message",
                               key_expr_);
                return;
            }
            // Issue #722 — wrapper-level stale-message filter.  Drops messages
            // whose publisher timestamp predates this subscriber's birth (with
            // slack) — defends every timestamped IPC topic against Zenoh's
            // last-value cache delivering historic messages from a previous
            // publisher session.  Replaces the bespoke per-site filter PR #721
            // installed in process4_mission_planner/src/main.cpp:93 for the
            // pose topic specifically.
            //
            // Only fires for types with BOTH validate() AND timestamp_ns.  All
            // safety-critical IPC types today satisfy both (Pose, FCState,
            // DetectedObjectList, TrajectoryCmd, RadarDetectionList, etc.).
            // Types like raw configuration uploads that don't have a publisher
            // timestamp pass through unfiltered.
            //
            // `timestamp_ns == 0` is the documented sentinel for "publisher
            // didn't stamp" / "default-constructed"; we let those through so
            // downstream code can apply its own staleness logic.  The pre-birth
            // check only fires for messages with a real, positive timestamp
            // that predates us — i.e. unambiguous historical-cache replays.
            //
            // PR #750 review fix: the original comparison was
            //     temp->timestamp_ns + kBirthSlackNs < subscriber_birth_ns_
            // which wraps if `temp->timestamp_ns` is near UINT64_MAX (corrupt
            // or adversarial payload).  Rewritten as an overflow-safe
            // subtraction: subtract slack from birth, then a plain `<`
            // compare.  No addition, no UB.
            if constexpr (has_timestamp_ns<T>::value) {
                if (filter_pre_birth_messages_ && temp->timestamp_ns > 0 &&
                    subscriber_birth_ns_ > kBirthSlackNs &&
                    temp->timestamp_ns < (subscriber_birth_ns_ - kBirthSlackNs)) {
                    log_stale_once(temp->timestamp_ns);
                    return;
                }
            }
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_msg_   = *temp;
            timestamp_ns_ = drone::util::get_clock().now_ns();
        } else {
            // For types without validate(), deserialize directly into latest_msg_
            // under lock.  A stack temporary is NOT safe here — large types like
            // VideoFrame (~6.2 MB) would overflow Zenoh's callback thread stack.
            // (For validating types, the if-branch above uses make_unique for the
            // same reason.)  The serializer checks size first, so partial writes
            // on failure are acceptable (the old value was already overwritten by
            // design).
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!serializer_->deserialize(bytes.data(), bytes.size(), latest_msg_)) {
                DRONE_LOG_WARN("[ZenohSubscriber] Deserialization failed on '{}': "
                               "payload {} bytes, sizeof(T) {}",
                               key_expr_, bytes.size(), sizeof(T));
                return;
            }
            timestamp_ns_ = drone::util::get_clock().now_ns();
        }

        has_data_.store(true, std::memory_order_release);
    }

    /// SFINAE helper to detect T::validate() at compile time.
    template<typename U, typename = void>
    struct has_validate : std::false_type {};

    template<typename U>
    struct has_validate<U, std::void_t<decltype(std::declval<const U&>().validate())>>
        : std::true_type {};

    /// SFINAE helper to detect T::timestamp_ns at compile time (Issue #722).
    /// Used to gate the wrapper-level stale-message filter — only types with
    /// a publisher-stamped `timestamp_ns` field can be checked against
    /// `subscriber_birth_ns_`.  Mirrors the `has_validate<T>` pattern above.
    template<typename U, typename = void>
    struct has_timestamp_ns : std::false_type {};

    template<typename U>
    struct has_timestamp_ns<U, std::void_t<decltype(std::declval<const U&>().timestamp_ns)>>
        : std::true_type {};

    /// Issue #722 — slack allowed when comparing publisher timestamps against
    /// `subscriber_birth_ns_`.  Covers the rare case where the publisher
    /// (e.g. P3) booted slightly before this subscriber's process (e.g. P4)
    /// and emitted its first valid message during our MessageBus init —
    /// without slack, that "almost-fresh" message would be misclassified as
    /// stale.  100 ms is the same value PR #721 used at the per-site filter
    /// it now replaces.
    static constexpr uint64_t kBirthSlackNs = 100'000'000ULL;

    /// Issue #722 — emit the stale-drop log line exactly once per subscriber
    /// to avoid log spam if Zenoh's cache replays many historic messages in
    /// quick succession at boot.
    void log_stale_once(uint64_t msg_ts_ns) const {
        bool expected = false;
        if (stale_logged_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            const uint64_t age_s = (subscriber_birth_ns_ > msg_ts_ns)
                                       ? (subscriber_birth_ns_ - msg_ts_ns) / 1'000'000'000ULL
                                       : 0ULL;
            DRONE_LOG_WARN("[ZenohSubscriber] Dropping pre-birth message on '{}' "
                           "(publisher timestamp ~{}s older than subscriber birth) — "
                           "Zenoh last-value cache from a previous publisher session. "
                           "Further pre-birth drops on this topic will be silent.",
                           key_expr_, age_s);
        }
    }

    std::string                            key_expr_;
    bool                                   track_latency_ = true;
    std::shared_ptr<const ISerializer<T>>  serializer_;
    std::optional<zenoh::Subscriber<void>> subscriber_;

    // Issue #722 — wrapper-level stale-message filter state.
    bool                      filter_pre_birth_messages_ = true;
    uint64_t                  subscriber_birth_ns_       = 0;
    mutable std::atomic<bool> stale_logged_{false};

    // Latest-value cache (protected by data_mutex_ + atomics)
    mutable std::mutex                  data_mutex_;
    T                                   latest_msg_{};
    uint64_t                            timestamp_ns_{0};
    std::atomic<bool>                   has_data_{false};
    mutable drone::util::LatencyTracker latency_tracker_{1024};
    // Tracks the timestamp of the last message we recorded a latency sample
    // for.  Lets receive() suppress duplicate samples on quiet topics so the
    // tracker reports real callback→poll delay rather than wall-clock drift.
    mutable std::atomic<uint64_t> last_recorded_ts_{0};
};

}  // namespace drone::ipc
