// common/recorder/include/recorder/replay_dispatch.h
// Reusable dispatch logic for flight log replay — independent of IPC transport.
//
// This module provides:
//   1. dispatch_record()          — type-safe deserialization + callback dispatch
//   2. calculate_replay_delay()   — timing calculation for speed-scaled replay
//   3. IReplayHandler (interface) — virtual callback interface for replay consumers
//
// The design is deliberately transport-agnostic: any replay application (CLI tool,
// Gazebo bridge, web player, test harness) can implement IReplayHandler or use
// the template dispatch_record() with a custom callback struct.
//
// Usage with virtual interface:
//   class MyHandler : public drone::recorder::IReplayHandler { ... };
//   MyHandler handler;
//   for (auto& entry : log.entries) {
//       auto result = drone::recorder::dispatch_record(entry, handler);
//   }
//
// Usage with template (zero-overhead, no vtable):
//   struct MyCallbacks {
//       void on_pose(const Pose&) { ... }
//       void on_detections(const DetectedObjectList&) { ... }
//       ...
//   };
//   MyCallbacks cb;
//   auto result = drone::recorder::dispatch_record(entry, cb);
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/wire_format.h"
// TODO: Extract RecordEntry/RecordHeader into a lightweight record_entry.h
//       to reduce compile dependencies for dispatch-only consumers.
#include "recorder/flight_recorder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace drone::recorder {

/// Result of attempting to dispatch a single log record.
struct DispatchResult {
    bool                        dispatched = false;
    drone::ipc::WireMessageType msg_type   = drone::ipc::WireMessageType::UNKNOWN;
};

/// Virtual interface for replay consumers.
/// Implement this to receive deserialized messages from a flight log.
/// Default implementations are no-ops so consumers can override only what they need.
class IReplayHandler {
public:
    virtual ~IReplayHandler() = default;

    virtual void on_pose(const drone::ipc::Pose& /*msg*/) {}
    virtual void on_detections(const drone::ipc::DetectedObjectList& /*msg*/) {}
    virtual void on_fc_state(const drone::ipc::FCState& /*msg*/) {}
    virtual void on_trajectory(const drone::ipc::TrajectoryCmd& /*msg*/) {}
    virtual void on_health(const drone::ipc::SystemHealth& /*msg*/) {}
    virtual void on_payload(const drone::ipc::PayloadStatus& /*msg*/) {}
};

/// Deserialize a RecordEntry payload and invoke the matching callback.
///
/// @tparam Handler  Any type with methods:
///   void on_pose(const Pose&)
///   void on_detections(const DetectedObjectList&)
///   void on_fc_state(const FCState&)
///   void on_trajectory(const TrajectoryCmd&)
///   void on_health(const SystemHealth&)
///   void on_payload(const PayloadStatus&)
///
/// Works with both IReplayHandler subclasses (virtual dispatch) and
/// plain structs (template dispatch, zero overhead).
///
/// @return DispatchResult indicating whether the record was published.
template<typename Handler>
[[nodiscard]] DispatchResult dispatch_record(const RecordEntry& entry, Handler& handler) {
    using namespace drone::ipc;

    const auto     msg_type = entry.header.wire_header.msg_type;
    DispatchResult result;
    result.msg_type = msg_type;

    // Helper: deserialize if payload size matches T exactly and header is consistent
    auto try_deserialize = [&](auto* out) -> bool {
        using T = std::remove_pointer_t<decltype(out)>;
        static_assert(std::is_trivially_copyable_v<T>);
        if (entry.header.wire_header.payload_size != sizeof(T)) return false;
        if (entry.payload.size() != entry.header.wire_header.payload_size) return false;
        std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(out));
        return true;
    };

    if (msg_type == WireMessageType::SLAM_POSE) {
        Pose msg{};
        if (try_deserialize(&msg)) {
            handler.on_pose(msg);
            result.dispatched = true;
        }
    } else if (msg_type == WireMessageType::DETECTIONS) {
        DetectedObjectList msg{};
        if (try_deserialize(&msg)) {
            handler.on_detections(msg);
            result.dispatched = true;
        }
    } else if (msg_type == WireMessageType::FC_STATE) {
        FCState msg{};
        if (try_deserialize(&msg)) {
            handler.on_fc_state(msg);
            result.dispatched = true;
        }
    } else if (msg_type == WireMessageType::TRAJECTORY_CMD) {
        TrajectoryCmd msg{};
        if (try_deserialize(&msg)) {
            handler.on_trajectory(msg);
            result.dispatched = true;
        }
    } else if (msg_type == WireMessageType::SYSTEM_HEALTH) {
        SystemHealth msg{};
        if (try_deserialize(&msg)) {
            handler.on_health(msg);
            result.dispatched = true;
        }
    } else if (msg_type == WireMessageType::PAYLOAD_STATUS) {
        PayloadStatus msg{};
        if (try_deserialize(&msg)) {
            handler.on_payload(msg);
            result.dispatched = true;
        }
    }

    return result;
}

/// Calculate the sleep duration for replay timing.
/// @param record_ts     Timestamp of the current record (nanoseconds)
/// @param first_ts      Timestamp of the first record (nanoseconds)
/// @param speed         Playback speed multiplier (0 = no delay)
/// @return Replay delay as a chrono duration. Returns 0 if speed <= 0.
[[nodiscard]] inline std::chrono::nanoseconds calculate_replay_delay(uint64_t record_ts,
                                                                     uint64_t first_ts,
                                                                     float    speed) {
    if (speed <= 0.0f || std::isnan(speed)) return std::chrono::nanoseconds{0};

    // Compute delta in uint64_t to avoid implementation-defined overflow on int64_t cast
    if (record_ts < first_ts) return std::chrono::nanoseconds{0};
    const uint64_t delta_ns = record_ts - first_ts;

    // Scale by speed, then clamp to INT64_MAX before constructing chrono duration
    const double scaled  = static_cast<double>(delta_ns) / static_cast<double>(speed);
    const auto   clamped = (scaled >= static_cast<double>(INT64_MAX)) ? INT64_MAX
                                                                      : static_cast<int64_t>(scaled);
    return std::chrono::nanoseconds(clamped);
}

}  // namespace drone::recorder
