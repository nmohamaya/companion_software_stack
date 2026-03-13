// common/ipc/include/ipc/zenoh_service_client.h
// Zenoh-backed service client — wraps session.get() behind IServiceClient<Req, Resp>.
//
// Request-response pattern:
//   send_request()  → serialises Req into Bytes, calls session.get() with
//                     a callback that pushes replies into a thread-safe queue.
//   poll_response() → non-blocking dequeue matching correlation_id.
//   await_response()→ blocking spin-sleep (uses IServiceClient default impl).
//
// Key expression scheme:
//   "drone/service/<service_name>"   (e.g. "drone/service/trajectory")
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/iservice_channel.h"
#include "ipc/zenoh_session.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed service client implementing IServiceClient<Req, Resp>.
///
/// Each send_request() fires a Zenoh GET with the serialised request as
/// payload.  The correlation ID is encoded in the GET parameters string
/// ("cid=<id>") so the server can reflect it back.  Replies arrive via
/// a Zenoh callback and are enqueued for later retrieval by poll_response().
template<typename Req, typename Resp>
class ZenohServiceClient final : public IServiceClient<Req, Resp> {
    static_assert(std::is_trivially_copyable_v<Req>,
                  "ZenohServiceClient requires trivially copyable Req");
    static_assert(std::is_trivially_copyable_v<Resp>,
                  "ZenohServiceClient requires trivially copyable Resp");

public:
    /// @param key_expr  Zenoh key expression for the service
    ///                  (e.g. "drone/service/trajectory").
    /// @param timeout_ms  Timeout for the Zenoh GET operation in milliseconds.
    explicit ZenohServiceClient(const std::string& key_expr, uint64_t timeout_ms = 5000)
        : key_expr_(key_expr), timeout_ms_(timeout_ms) {
        spdlog::info("[ZenohServiceClient] Created on '{}'", key_expr);
    }

    [[nodiscard]] uint64_t send_request(const Req& request) override {
        uint64_t id = id_prefix_ | next_seq_.fetch_add(1, std::memory_order_relaxed);

        // Serialise the request into raw bytes
        const auto*          ptr = reinterpret_cast<const uint8_t*>(&request);
        std::vector<uint8_t> buf(ptr, ptr + sizeof(Req));

        // Encode correlation ID in the parameters string
        std::string params = "cid=" + std::to_string(id);

        // Capture a shared pointer to the response queue so the callback
        // can safely enqueue even if this client object is destroyed early.
        auto responses = responses_;

        auto& session = ZenohSession::instance().session();

        zenoh::Session::GetOptions opts = zenoh::Session::GetOptions::create_default();
        opts.payload                    = zenoh::Bytes(std::move(buf));
        opts.timeout_ms                 = timeout_ms_;

        session.get(
            zenoh::KeyExpr(key_expr_), params,
            // on_reply callback — runs on Zenoh internal thread
            [id, responses](zenoh::Reply& reply) {
                try {
                    if (!reply.is_ok()) {
                        spdlog::warn("[ZenohServiceClient] Got error reply "
                                     "for cid={}",
                                     id);
                        return;
                    }
                    const auto& sample = reply.get_ok();
                    auto        bytes  = sample.get_payload().as_vector();

                    if (bytes.size() != sizeof(ServiceResponse<Resp>)) {
                        spdlog::warn("[ZenohServiceClient] Reply size mismatch "
                                     "for cid={}: expected {} got {}",
                                     id, sizeof(ServiceResponse<Resp>), bytes.size());
                        return;
                    }

                    ServiceResponse<Resp> resp;
                    std::memcpy(&resp, bytes.data(), sizeof(ServiceResponse<Resp>));

                    std::lock_guard<std::mutex> lock(responses->mutex);
                    responses->queue.push_back(resp);
                } catch (const std::exception& e) {
                    spdlog::error("[ZenohServiceClient] Error in reply "
                                  "callback: {}",
                                  e.what());
                }
            },
            // on_drop callback — called when all replies received
            []() {}, std::move(opts));

        spdlog::debug("[ZenohServiceClient] Sent request cid={} on '{}'", id, key_expr_);
        return id;
    }

    [[nodiscard]] std::optional<ServiceResponse<Resp>> poll_response(
        uint64_t correlation_id) override {
        std::lock_guard<std::mutex> lock(responses_->mutex);
        for (auto it = responses_->queue.begin(); it != responses_->queue.end(); ++it) {
            if (it->correlation_id == correlation_id && it->valid) {
                auto resp = *it;
                responses_->queue.erase(it);
                return resp;
            }
        }
        return std::nullopt;
    }

private:
    /// Thread-safe response queue shared with Zenoh callbacks.
    struct ResponseQueue {
        std::mutex                        mutex;
        std::deque<ServiceResponse<Resp>> queue;
    };

    /// Generate a random 32-bit prefix so correlation IDs are unique
    /// across multiple client instances targeting the same service.
    static uint64_t make_id_prefix() {
        std::random_device rd;
        return static_cast<uint64_t>(rd()) << 32;
    }

    std::string                    key_expr_;
    uint64_t                       timeout_ms_;
    uint64_t                       id_prefix_ = make_id_prefix();
    std::atomic<uint64_t>          next_seq_{1};
    std::shared_ptr<ResponseQueue> responses_ = std::make_shared<ResponseQueue>();
};

}  // namespace drone::ipc
