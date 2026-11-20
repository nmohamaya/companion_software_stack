// Backend dispatch and outcome capture for P5's RTL and LAND commands.
#pragma once

#include "hal/ifc_link.h"
#include "ipc/ipc_types.h"
#include "util/iclock.h"
#include "util/ilogger.h"

namespace drone::comms {

/// Called only for RTL and LAND. A successful backend call leaves controller
/// execution unconfirmed. Either result blocks forwarding stale trajectories.
inline ipc::FCCommandOutcome dispatch_fc_response(hal::IFCLink& fc, const ipc::FCCommand& request,
                                                  uint64_t& last_trajectory_timestamp) {
    const bool is_rtl = request.command == ipc::FCCommandType::RTL;
    DRONE_LOG_INFO("[Comms] FC cmd: {} corr={:#x}", is_rtl ? "RTL" : "LAND",
                   request.correlation_id);
    const uint8_t mode        = is_rtl ? 3 : 2;
    const bool    submitted   = fc.send_mode(mode);
    last_trajectory_timestamp = UINT64_MAX;
    return {util::get_clock().now_ns(), request.timestamp_ns, request.sequence_id, request.command,
            submitted ? ipc::FCBackendOutcome::SUBMITTED : ipc::FCBackendOutcome::FAILED};
}

}  // namespace drone::comms
