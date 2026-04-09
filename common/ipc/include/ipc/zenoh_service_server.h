// common/ipc/include/ipc/zenoh_service_server.h
// Zenoh-backed service server — wraps session.declare_queryable() behind
// IServiceServer<Req, Resp>.
//
// Request-response pattern:
//   The server declares a Zenoh queryable on a key expression.
//   Incoming queries (from session.get() calls) are enqueued with their
//   deserialized request payload and cloned Query handle.
//
//   poll_request()    → non-blocking dequeue of the next pending request.
//   send_response()   → serialises the ServiceResponse<Resp> and calls
//                       query.reply() on the stored Query handle.
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/iservice_channel.h"
#include "ipc/zenoh_session.h"
#include "util/iclock.h"
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed service server implementing IServiceServer<Req, Resp>.
///
/// Declares a Zenoh queryable.  Incoming queries carry the request payload
/// (as raw bytes) and the correlation ID in the parameters string ("cid=N").
/// The Query handle is cloned and stored so send_response() can reply later.
template<typename Req, typename Resp>
class ZenohServiceServer final : public IServiceServer<Req, Resp> {
    static_assert(std::is_trivially_copyable_v<Req>,
                  "ZenohServiceServer requires trivially copyable Req");
    static_assert(std::is_trivially_copyable_v<Resp>,
                  "ZenohServiceServer requires trivially copyable Resp");

public:
    /// @param key_expr  Zenoh key expression for the service
    ///                  (e.g. "drone/service/trajectory").
    explicit ZenohServiceServer(const std::string& key_expr) : key_expr_(key_expr) {
        auto pending = pending_;  // capture shared_ptr for callback

        auto& session = ZenohSession::instance().session();
        queryable_.emplace(session.declare_queryable(
            zenoh::KeyExpr(key_expr),
            // on_query callback — runs on Zenoh internal thread
            [key_expr, pending](zenoh::Query& query) {
                try {
                    // Parse correlation ID from parameters
                    uint64_t cid    = 0;
                    auto     params = query.get_parameters();
                    auto     pos    = params.find("cid=");
                    if (pos != std::string_view::npos) {
                        cid = std::stoull(std::string(params.substr(pos + 4)));
                    }

                    // Deserialise request payload
                    auto payload_opt = query.get_payload();
                    if (!payload_opt.has_value()) {
                        DRONE_LOG_WARN("[ZenohServiceServer] Query on '{}' "
                                       "has no payload",
                                       key_expr);
                        return;
                    }
                    auto bytes = payload_opt->get().as_vector();
                    if (bytes.size() != sizeof(Req)) {
                        DRONE_LOG_WARN("[ZenohServiceServer] Payload size "
                                       "mismatch on '{}': expected {} got {}",
                                       key_expr, sizeof(Req), bytes.size());
                        return;
                    }

                    Req   req{};
                    auto* req_dst = reinterpret_cast<uint8_t*>(&req);
                    std::copy(bytes.data(), bytes.data() + sizeof(Req), req_dst);

                    // Build envelope
                    ServiceEnvelope<Req> env;
                    env.correlation_id = cid;
                    env.timestamp_ns   = drone::util::get_clock().now_ns();
                    env.valid          = true;
                    env.payload        = req;

                    // Clone query handle and enqueue
                    std::lock_guard<std::mutex> lock(pending->mutex);
                    pending->queue.push_back({env, query.clone()});

                    DRONE_LOG_DEBUG("[ZenohServiceServer] Received query "
                                    "cid={} on '{}'",
                                    cid, key_expr);
                } catch (const std::exception& e) {
                    DRONE_LOG_ERROR("[ZenohServiceServer] Error in query "
                                    "callback: {}",
                                    e.what());
                }
            },
            // on_drop callback
            []() {}));

        DRONE_LOG_INFO("[ZenohServiceServer] Queryable declared on '{}'", key_expr);
    }

    [[nodiscard]] std::optional<ServiceEnvelope<Req>> poll_request() override {
        std::lock_guard<std::mutex> lock(pending_->mutex);
        if (pending_->queue.empty()) return std::nullopt;

        auto& front = pending_->queue.front();
        auto  env   = front.envelope;

        // Move the query to the outstanding map for later reply
        {
            std::lock_guard<std::mutex> olock(outstanding_mutex_);
            outstanding_.emplace(env.correlation_id, std::move(front.query));
        }
        pending_->queue.pop_front();
        return env;
    }

    void send_response(uint64_t correlation_id, ServiceStatus status,
                       const Resp& response) override {
        // Build response envelope
        ServiceResponse<Resp> resp;
        resp.correlation_id = correlation_id;
        resp.timestamp_ns   = drone::util::get_clock().now_ns();
        resp.status         = status;
        resp.valid          = true;
        resp.payload        = response;

        // Serialise
        const auto*          ptr = reinterpret_cast<const uint8_t*>(&resp);
        std::vector<uint8_t> buf(ptr, ptr + sizeof(ServiceResponse<Resp>));

        // Find and remove the outstanding query
        std::optional<zenoh::Query> query_handle;
        {
            std::lock_guard<std::mutex> lock(outstanding_mutex_);
            auto                        it = outstanding_.find(correlation_id);
            if (it == outstanding_.end()) {
                DRONE_LOG_WARN("[ZenohServiceServer] No outstanding query for "
                               "cid={}",
                               correlation_id);
                return;
            }
            query_handle.emplace(std::move(it->second));
            outstanding_.erase(it);
        }

        // Reply through the Zenoh query handle
        query_handle->reply(zenoh::KeyExpr(key_expr_), zenoh::Bytes(std::move(buf)));

        DRONE_LOG_DEBUG("[ZenohServiceServer] Sent response cid={} status={}", correlation_id,
                        static_cast<int>(status));
    }

private:
    /// Pending request queue entry: envelope + cloned Query handle.
    struct PendingEntry {
        ServiceEnvelope<Req> envelope;
        zenoh::Query         query;
    };

    /// Thread-safe pending queue shared with the queryable callback.
    struct PendingQueue {
        std::mutex               mutex;
        std::deque<PendingEntry> queue;
    };

    std::string                           key_expr_;
    std::optional<zenoh::Queryable<void>> queryable_;
    std::shared_ptr<PendingQueue>         pending_ = std::make_shared<PendingQueue>();

    // Outstanding queries awaiting send_response()
    std::mutex                                 outstanding_mutex_;
    std::unordered_map<uint64_t, zenoh::Query> outstanding_;
};

}  // namespace drone::ipc
