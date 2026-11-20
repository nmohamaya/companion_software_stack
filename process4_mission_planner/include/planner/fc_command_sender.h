// Publishes planner commands and tracks evidence for the current fault response.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "util/correlation.h"
#include "util/iclock.h"
#include "util/ilogger.h"

#include <chrono>
#include <optional>

namespace drone::planner {

class FCCommandSender {
public:
    explicit FCCommandSender(ipc::IPublisher<ipc::FCCommand>& publisher,
                             std::chrono::milliseconds observation_timeout = std::chrono::seconds(5))
        : publisher_(publisher)
        , timeout_ns_(
              std::chrono::duration_cast<std::chrono::nanoseconds>(observation_timeout).count()) {}

    void send(ipc::FCCommandType command, float param) {
        publisher_.publish(make_command(command, param));
    }

    void send_fault_response(ipc::FCCommandType command, float param) {
        const auto request = make_command(command, param);
        pending_           = Pending{request};
        DRONE_LOG_INFO("[FaultResponse] requested cmd={} seq={} request_ns={} backend=unconfirmed",
                       static_cast<unsigned>(command), request.sequence_id, request.timestamp_ns);
        publisher_.publish(request);
    }

    void observe_outcome(const ipc::FCCommandOutcome& outcome, uint64_t now) {
        if (!pending_ || expire_pending(now) || pending_->submitted || !outcome.validate() ||
            outcome.timestamp_ns > now) {
            return;
        }
        const auto& request = pending_->request;
        if (outcome.command != request.command ||
            outcome.request_timestamp_ns != request.timestamp_ns ||
            outcome.sequence_id != request.sequence_id) {
            return;
        }
        util::ScopedCorrelation correlation(request.correlation_id);
        if (outcome.backend_outcome == ipc::FCBackendOutcome::FAILED) {
            DRONE_LOG_WARN(
                "[FaultResponse] backend failed cmd={} seq={} request_ns={} result_ns={}",
                static_cast<unsigned>(request.command), request.sequence_id, request.timestamp_ns,
                outcome.timestamp_ns);
            pending_.reset();
        } else {
            DRONE_LOG_INFO("[FaultResponse] backend submitted cmd={} seq={} request_ns={} "
                           "result_ns={} controller=unconfirmed",
                           static_cast<unsigned>(request.command), request.sequence_id,
                           request.timestamp_ns, outcome.timestamp_ns);
            pending_->submitted = true;
        }
    }

    void observe_mode(const FCModeObservation& observation, uint64_t now) {
        if (!pending_ || expire_pending(now) || !pending_->submitted || !observation.validate()) {
            return;
        }
        const auto& request  = pending_->request;
        const auto  expected = request.command == ipc::FCCommandType::RTL ? FCObservedMode::RTL
                                                                          : FCObservedMode::LAND;
        if (observation.mode != expected || observation.timestamp_ns <= request.timestamp_ns ||
            observation.timestamp_ns > now) {
            return;
        }
        util::ScopedCorrelation correlation(request.correlation_id);
        DRONE_LOG_INFO(
            "[FaultResponse] controller mode observed cmd={} seq={} request_ns={} mode_ns={}",
            static_cast<unsigned>(request.command), request.sequence_id, request.timestamp_ns,
            observation.timestamp_ns);
        pending_.reset();
    }

    void reset_fault_response() { pending_.reset(); }

private:
    struct Pending {
        ipc::FCCommand request;
        bool           submitted{false};
    };

    // A backwards clock is not usable evidence. End the observation window so
    // samples from a previous clock epoch cannot resolve the current request.
    bool expire_pending(uint64_t now) {
        const auto& request = pending_->request;
        if (now >= request.timestamp_ns && now - request.timestamp_ns < timeout_ns_) {
            return false;
        }
        util::ScopedCorrelation correlation(request.correlation_id);
        DRONE_LOG_WARN("[FaultResponse] controller unconfirmed at observation deadline "
                       "cmd={} seq={} request_ns={} backend={}",
                       static_cast<unsigned>(request.command), request.sequence_id,
                       request.timestamp_ns, pending_->submitted ? "submitted" : "unknown");
        pending_.reset();
        return true;
    }

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
    const uint64_t                   timeout_ns_;
    uint64_t                         sequence_{0};
    std::optional<Pending>           pending_;
};

}  // namespace drone::planner
