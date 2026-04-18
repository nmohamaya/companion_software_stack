// process4_mission_planner/include/planner/gcs_command_handler.h
// Handles GCS command dispatch: RTL, LAND, MISSION_PAUSE, MISSION_START,
// MISSION_ABORT, MISSION_UPLOAD.
// Deduplicates by timestamp and propagates correlation IDs.
// Validates waypoint data (#177, #178) and rate-limits uploads (#182).
//
// Extracted from main.cpp as part of Issue #154.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"
#include "planner/mission_fsm.h"
#include "util/correlation.h"
#include "util/diagnostic.h"
#include "util/ilogger.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace drone::planner {

/// Callback type for FC command sending.
using FCSendFn = std::function<void(drone::ipc::FCCommandType, float)>;

/// Shared mutable state between GCSCommandHandler, FaultResponseExecutor,
/// and MissionStateTick.  Owned by MissionStateTick, passed by reference.
struct SharedFlightState {
    bool                                  land_sent{false};
    bool                                  nav_was_armed{false};
    std::chrono::steady_clock::time_point rtl_start_time{};
};

/// Handles GCS commands received via IPC: RTL, LAND, MISSION_PAUSE/START/ABORT/UPLOAD.
/// Deduplicates by timestamp to ignore stale values from the subscriber cache.
/// Validates waypoint data and rate-limits mission uploads.
class GCSCommandHandler {
public:
    /// Configurable limits for mission upload validation.
    struct UploadLimits {
        float    speed_min_mps{0.1f};
        float    speed_max_mps{50.0f};
        float    radius_min_m{0.1f};
        float    radius_max_m{100.0f};
        uint32_t rate_limit_ms{5000};
    };

    GCSCommandHandler() = default;
    explicit GCSCommandHandler(UploadLimits limits) : limits_(limits) {}

    /// Process one tick of GCS command handling.
    void process(drone::ipc::ISubscriber<drone::ipc::GCSCommand>&    gcs_sub,
                 drone::ipc::ISubscriber<drone::ipc::MissionUpload>& upload_sub, MissionFSM& fsm,
                 const FCSendFn&                                    send_fc,
                 drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub,
                 SharedFlightState& flight_state, drone::util::FrameDiagnostics& diag) {
        drone::ipc::GCSCommand gcs_cmd{};
        if (!gcs_sub.is_connected() || !gcs_sub.receive(gcs_cmd) || !gcs_cmd.valid ||
            gcs_cmd.timestamp_ns <= last_gcs_timestamp_)
            return;

        last_gcs_timestamp_    = gcs_cmd.timestamp_ns;
        active_correlation_id_ = gcs_cmd.correlation_id;
        drone::util::ScopedCorrelation gcs_guard(gcs_cmd.correlation_id);

        switch (gcs_cmd.command) {
            case drone::ipc::GCSCommandType::RTL:
                DRONE_LOG_INFO("[Planner] GCS command: RTL corr={:#x}", gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                flight_state.rtl_start_time = std::chrono::steady_clock::now();
                flight_state.nav_was_armed  = true;
                fsm.on_rtl();
                break;

            case drone::ipc::GCSCommandType::LAND:
                DRONE_LOG_INFO("[Planner] GCS command: LAND corr={:#x}", gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::LAND, 0.0f);
                flight_state.land_sent = true;
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                fsm.on_land();
                break;

            case drone::ipc::GCSCommandType::MISSION_PAUSE:
                DRONE_LOG_INFO("[Planner] GCS command: MISSION_PAUSE corr={:#x}",
                               gcs_cmd.correlation_id);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                fsm.on_loiter();
                break;

            case drone::ipc::GCSCommandType::MISSION_START:
                DRONE_LOG_INFO("[Planner] GCS command: MISSION_START corr={:#x}",
                               gcs_cmd.correlation_id);
                if (fsm.state() == MissionState::LOITER) {
                    fsm.on_navigate();
                }
                break;

            case drone::ipc::GCSCommandType::MISSION_ABORT:
                DRONE_LOG_INFO("[Planner] GCS command: MISSION_ABORT corr={:#x}",
                               gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                flight_state.rtl_start_time = std::chrono::steady_clock::now();
                flight_state.nav_was_armed  = true;
                fsm.on_rtl();
                break;

            case drone::ipc::GCSCommandType::MISSION_UPLOAD: {
                process_mission_upload(upload_sub, fsm, gcs_cmd.correlation_id, diag);
                break;
            }
            default: break;
        }
    }

    /// Currently active GCS correlation ID.
    [[nodiscard]] uint64_t active_correlation_id() const { return active_correlation_id_; }

    /// Number of uploads rejected for rate limiting (test observability).
    [[nodiscard]] uint32_t rate_limited_count() const { return rate_limited_count_; }

    /// Number of uploads rejected for waypoint validation (test observability).
    [[nodiscard]] uint32_t validation_rejected_count() const { return validation_rejected_count_; }

    /// Number of uploads rejected for FSM state (test observability).
    [[nodiscard]] uint32_t state_rejected_count() const { return state_rejected_count_; }

private:
    uint64_t     last_gcs_timestamp_    = 0;
    uint64_t     last_upload_timestamp_ = 0;
    uint64_t     active_correlation_id_ = 0;
    UploadLimits limits_;

    // Rate limiting (#182)
    std::chrono::steady_clock::time_point last_upload_time_{};
    uint32_t                              rate_limited_count_{0};
    uint32_t                              validation_rejected_count_{0};
    uint32_t                              state_rejected_count_{0};

    /// Process a MISSION_UPLOAD command with validation and rate limiting.
    void process_mission_upload(drone::ipc::ISubscriber<drone::ipc::MissionUpload>& upload_sub,
                                MissionFSM& fsm, uint64_t correlation_id,
                                drone::util::FrameDiagnostics& diag) {
        // FSM state check — reject during safety-critical phases (#182)
        auto state = fsm.state();
        if (state == MissionState::RTL || state == MissionState::LAND ||
            state == MissionState::EMERGENCY || state == MissionState::TAKEOFF ||
            state == MissionState::COLLISION_RECOVERY || state == MissionState::NAVIGATE_UNSTUCK) {
            DRONE_LOG_WARN("[Planner] MISSION_UPLOAD rejected — unsafe FSM state '{}' corr={:#x}",
                           static_cast<int>(state), correlation_id);
            ++state_rejected_count_;
            return;
        }

        // Rate limiting (#182)
        auto now = std::chrono::steady_clock::now();
        if (last_upload_time_ != std::chrono::steady_clock::time_point{}) {
            auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_upload_time_)
                    .count();
            if (elapsed_ms < limits_.rate_limit_ms) {
                DRONE_LOG_WARN("[Planner] MISSION_UPLOAD rate-limited — {}ms since last upload "
                               "(min {}ms) corr={:#x}",
                               elapsed_ms, limits_.rate_limit_ms, correlation_id);
                ++rate_limited_count_;
                return;
            }
        }

        drone::ipc::MissionUpload upload{};
        if (!upload_sub.is_connected() || !upload_sub.receive(upload) || !upload.valid ||
            upload.timestamp_ns <= last_upload_timestamp_ || upload.num_waypoints == 0) {
            DRONE_LOG_WARN("[Planner] MISSION_UPLOAD command but no valid upload data available");
            return;
        }

        last_upload_timestamp_ = upload.timestamp_ns;

        // Waypoint validation (#177, #178)
        std::vector<Waypoint> new_wps;
        bool                  valid = true;
        for (uint8_t i = 0; i < upload.num_waypoints && i < drone::ipc::kMaxUploadWaypoints; ++i) {
            const auto& sw = upload.waypoints[i];

            // #177: NaN/Inf coordinate check
            if (!std::isfinite(sw.x) || !std::isfinite(sw.y) || !std::isfinite(sw.z) ||
                !std::isfinite(sw.yaw)) {
                DRONE_LOG_ERROR("[Planner] MISSION_UPLOAD rejected — waypoint {} has "
                                "non-finite coordinate corr={:#x}",
                                i, correlation_id);
                valid = false;
                break;
            }

            // #178: speed and radius range validation
            if (sw.speed < limits_.speed_min_mps || sw.speed > limits_.speed_max_mps) {
                DRONE_LOG_ERROR("[Planner] MISSION_UPLOAD rejected — waypoint {} speed "
                                "{:.1f} outside [{:.1f}, {:.1f}] corr={:#x}",
                                i, sw.speed, limits_.speed_min_mps, limits_.speed_max_mps,
                                correlation_id);
                valid = false;
                break;
            }
            if (sw.radius < limits_.radius_min_m || sw.radius > limits_.radius_max_m) {
                DRONE_LOG_ERROR("[Planner] MISSION_UPLOAD rejected — waypoint {} radius "
                                "{:.1f} outside [{:.1f}, {:.1f}] corr={:#x}",
                                i, sw.radius, limits_.radius_min_m, limits_.radius_max_m,
                                correlation_id);
                valid = false;
                break;
            }

            new_wps.push_back({sw.x, sw.y, sw.z, sw.yaw, sw.radius, sw.speed, sw.trigger_payload});
        }

        if (!valid) {
            ++validation_rejected_count_;
            return;
        }

        last_upload_time_ = now;
        fsm.load_mission(new_wps);
        DRONE_LOG_INFO("[Planner] GCS MISSION_UPLOAD: {} waypoints loaded corr={:#x}",
                       new_wps.size(), correlation_id);
        diag.add_warning("MissionUpload", "Mid-flight waypoint upload: " +
                                              std::to_string(new_wps.size()) + " waypoints");
        if (fsm.state() == MissionState::NAVIGATE || fsm.state() == MissionState::LOITER) {
            fsm.on_navigate();
        }
    }

    static void publish_stop_trajectory(drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& pub,
                                        uint64_t correlation_id) {
        drone::ipc::TrajectoryCmd stop{};
        stop.valid          = true;  // P5 skips valid=false — send zero-velocity to stop
        stop.correlation_id = correlation_id;
        stop.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        pub.publish(stop);
    }
};

}  // namespace drone::planner
