// process4_mission_planner/include/planner/gcs_command_handler.h
// Handles GCS command dispatch: RTL, LAND, MISSION_PAUSE, MISSION_START,
// MISSION_ABORT, MISSION_UPLOAD.
// Deduplicates by timestamp and propagates correlation IDs.
//
// Extracted from main.cpp as part of Issue #154.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"
#include "planner/mission_fsm.h"
#include "util/correlation.h"
#include "util/diagnostic.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

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
class GCSCommandHandler {
public:
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
                spdlog::info("[Planner] GCS command: RTL corr={:#x}", gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                flight_state.rtl_start_time = std::chrono::steady_clock::now();
                flight_state.nav_was_armed  = true;
                fsm.on_rtl();
                break;

            case drone::ipc::GCSCommandType::LAND:
                spdlog::info("[Planner] GCS command: LAND corr={:#x}", gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::LAND, 0.0f);
                flight_state.land_sent = true;
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                fsm.on_land();
                break;

            case drone::ipc::GCSCommandType::MISSION_PAUSE:
                spdlog::info("[Planner] GCS command: MISSION_PAUSE corr={:#x}",
                             gcs_cmd.correlation_id);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                fsm.on_loiter();
                break;

            case drone::ipc::GCSCommandType::MISSION_START:
                spdlog::info("[Planner] GCS command: MISSION_START corr={:#x}",
                             gcs_cmd.correlation_id);
                if (fsm.state() == MissionState::LOITER) {
                    fsm.on_navigate();
                }
                break;

            case drone::ipc::GCSCommandType::MISSION_ABORT:
                spdlog::info("[Planner] GCS command: MISSION_ABORT corr={:#x}",
                             gcs_cmd.correlation_id);
                send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                publish_stop_trajectory(traj_pub, gcs_cmd.correlation_id);
                flight_state.rtl_start_time = std::chrono::steady_clock::now();
                flight_state.nav_was_armed  = true;
                fsm.on_rtl();
                break;

            case drone::ipc::GCSCommandType::MISSION_UPLOAD: {
                drone::ipc::MissionUpload upload{};
                if (upload_sub.is_connected() && upload_sub.receive(upload) && upload.valid &&
                    upload.timestamp_ns > last_upload_timestamp_ && upload.num_waypoints > 0) {
                    last_upload_timestamp_ = upload.timestamp_ns;
                    std::vector<Waypoint> new_wps;
                    for (uint8_t i = 0;
                         i < upload.num_waypoints && i < drone::ipc::kMaxUploadWaypoints; ++i) {
                        const auto& sw = upload.waypoints[i];
                        new_wps.push_back(
                            {sw.x, sw.y, sw.z, sw.yaw, sw.radius, sw.speed, sw.trigger_payload});
                    }
                    fsm.load_mission(new_wps);
                    spdlog::info("[Planner] GCS MISSION_UPLOAD: {} waypoints loaded corr={:#x}",
                                 new_wps.size(), gcs_cmd.correlation_id);
                    diag.add_warning("MissionUpload", "Mid-flight waypoint upload: " +
                                                          std::to_string(new_wps.size()) +
                                                          " waypoints");
                    if (fsm.state() == MissionState::NAVIGATE ||
                        fsm.state() == MissionState::LOITER) {
                        fsm.on_navigate();
                    }
                } else {
                    spdlog::warn("[Planner] MISSION_UPLOAD command but no valid "
                                 "upload data available");
                }
                break;
            }
            default: break;
        }
    }

    /// Currently active GCS correlation ID.
    [[nodiscard]] uint64_t active_correlation_id() const { return active_correlation_id_; }

private:
    uint64_t last_gcs_timestamp_    = 0;
    uint64_t last_upload_timestamp_ = 0;
    uint64_t active_correlation_id_ = 0;

    static void publish_stop_trajectory(drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& pub,
                                        uint64_t correlation_id) {
        drone::ipc::TrajectoryCmd stop{};
        stop.valid          = false;
        stop.correlation_id = correlation_id;
        stop.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        pub.publish(stop);
    }
};

}  // namespace drone::planner
