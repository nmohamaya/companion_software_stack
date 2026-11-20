// Publishes planner FC commands and correlates fault-response backend outcomes.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "util/correlation.h"
#include "util/iclock.h"
#include "util/ilogger.h"

#include <optional>

namespace drone::planner {

class FCCommandSender {
public:
    explicit FCCommandSender(ipc::IPublisher<ipc::FCCommand>& publisher) : publisher_(publisher) {}

    void send(ipc::FCCommandType command, float param) {
        publisher_.publish(make_command(command, param));
    }

    void send_fault_response(ipc::FCCommandType command, float param) {
        const auto request = make_command(command, param);
        pending_           = request;
        DRONE_LOG_INFO("[FaultResponse] requested cmd={} seq={} request_ns={} backend=unconfirmed",
                       static_cast<unsigned>(command), request.sequence_id, request.timestamp_ns);
        publisher_.publish(request);
    }

    void observe_outcome(const ipc::FCCommandOutcome& outcome) {
        if (!pending_ || !outcome.validate() || outcome.command != pending_->command ||
            outcome.request_timestamp_ns != pending_->timestamp_ns ||
            outcome.sequence_id != pending_->sequence_id) {
            return;
        }
        util::ScopedCorrelation correlation(pending_->correlation_id);
        if (outcome.backend_outcome == ipc::FCBackendOutcome::FAILED) {
            DRONE_LOG_WARN(
                "[FaultResponse] backend failed cmd={} seq={} request_ns={} result_ns={}",
                static_cast<unsigned>(outcome.command), outcome.sequence_id,
                outcome.request_timestamp_ns, outcome.timestamp_ns);
        } else {
            DRONE_LOG_INFO("[FaultResponse] backend submitted cmd={} seq={} request_ns={} "
                           "result_ns={} controller=unconfirmed",
                           static_cast<unsigned>(outcome.command), outcome.sequence_id,
                           outcome.request_timestamp_ns, outcome.timestamp_ns);
        }
        pending_.reset();
    }

    void reset_fault_response() { pending_.reset(); }

private:
    ipc::FCCommand make_command(ipc::FCCommandType command, float param) {
        ipc::FCCommand request{};
        request.timestamp_ns   = util::get_clock().now_ns();
        request.correlation_id = util::CorrelationContext::get();
        request.command        = command;
        request.param1         = param;
        request.sequence_id    = ++sequence_;
        request.valid          = true;
        return request;
    }

    ipc::IPublisher<ipc::FCCommand>& publisher_;
    uint64_t                         sequence_{0};
    std::optional<ipc::FCCommand>    pending_;
};

}  // namespace drone::planner
