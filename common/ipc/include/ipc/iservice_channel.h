// common/ipc/include/ipc/iservice_channel.h
// Abstract request-response service channel interfaces.
//
// Design:
//   IServiceClient<Req, Resp> — sends a request, polls/awaits response.
//   IServiceServer<Req, Resp> — polls for requests, sends responses.
//   Both use correlation IDs to match request-response pairs.
//
// Concrete implementations: ShmServiceChannel (SeqLock-based, latest-value).
// Production upgrade: SPSC ring in SHM, gRPC, DDS request-reply.
#pragma once

#include "util/iclock.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>
#include <type_traits>

namespace drone::ipc {

/// Envelope wrapping a service payload with correlation metadata.
/// Must be trivially copyable for SHM transport.
template<typename T>
struct ServiceEnvelope {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Service payload must be trivially copyable for SHM");
    uint64_t correlation_id{0};
    uint64_t timestamp_ns{0};
    bool     valid{false};
    T        payload{};
};

/// Response status codes for service calls.
enum class ServiceStatus : uint8_t {
    OK       = 0,
    REJECTED = 1,
    TIMEOUT  = 2,
    ERROR    = 3,
};

/// Response envelope — includes status alongside the payload.
template<typename T>
struct ServiceResponse {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Service response must be trivially copyable for SHM");
    uint64_t      correlation_id{0};
    uint64_t      timestamp_ns{0};
    ServiceStatus status{ServiceStatus::OK};
    bool          valid{false};
    T             payload{};
};

// ─────────────────────────────────────────────────────────────
// Client interface — sends requests, awaits responses.
// ─────────────────────────────────────────────────────────────

template<typename Req, typename Resp>
class IServiceClient {
public:
    virtual ~IServiceClient() = default;

    /// Send a request and return its correlation ID.
    [[nodiscard]] virtual uint64_t send_request(const Req& request) = 0;

    /// Poll for a response matching the given correlation ID.
    /// Returns the response if available, std::nullopt otherwise.
    [[nodiscard]] virtual std::optional<ServiceResponse<Resp>> poll_response(
        uint64_t correlation_id) = 0;

    /// Blocking wait for a response with timeout.
    /// Default implementation polls in a spin-sleep loop.
    [[nodiscard]] virtual std::optional<ServiceResponse<Resp>> await_response(
        uint64_t correlation_id, std::chrono::milliseconds timeout) {
        // Clamp negative timeouts to zero to prevent unsigned wraparound
        const auto timeout_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count();
        const auto deadline_ns = drone::util::get_clock().now_ns() +
                                 static_cast<uint64_t>(std::max(int64_t{0}, timeout_ns));
        while (drone::util::get_clock().now_ns() < deadline_ns) {
            if (auto resp = poll_response(correlation_id)) {
                return resp;
            }
            drone::util::get_clock().sleep_for_ms(1);
        }
        // Timed out
        ServiceResponse<Resp> timeout_resp;
        timeout_resp.correlation_id = correlation_id;
        timeout_resp.timestamp_ns   = drone::util::get_clock().now_ns();
        timeout_resp.status         = ServiceStatus::TIMEOUT;
        timeout_resp.valid          = true;
        return timeout_resp;
    }
};

// ─────────────────────────────────────────────────────────────
// Server interface — receives requests, sends responses.
// ─────────────────────────────────────────────────────────────

template<typename Req, typename Resp>
class IServiceServer {
public:
    virtual ~IServiceServer() = default;

    /// Poll for an incoming request.
    /// Returns the request envelope if a new one is available.
    [[nodiscard]] virtual std::optional<ServiceEnvelope<Req>> poll_request() = 0;

    /// Send a response for the given correlation ID.
    virtual void send_response(uint64_t correlation_id, ServiceStatus status,
                               const Resp& response) = 0;
};

}  // namespace drone::ipc
