// common/ipc/include/ipc/shm_service_channel.h
// SHM-backed request-response service channel.
//
// Uses two SHM segments (request + response) with SeqLock.
// Latest-value semantics: only one outstanding request at a time.
// For queued multi-request, upgrade to SPSC ring in SHM or gRPC.
//
// Usage:
//   // Server side:
//   ShmServiceServer<ReqType, RespType> server("/svc_traj_req", "/svc_traj_resp");
//   if (auto req = server.poll_request()) {
//       server.send_response(req->correlation_id, ServiceStatus::OK, resp);
//   }
//
//   // Client side:
//   ShmServiceClient<ReqType, RespType> client("/svc_traj_req", "/svc_traj_resp");
//   auto id = client.send_request(req);
//   auto resp = client.await_response(id, 500ms);
#pragma once

#include "ipc/iservice_channel.h"
#include "ipc/shm_writer.h"
#include "ipc/shm_reader.h"

#include <atomic>
#include <chrono>
#include <string>
#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Pre-create a SHM segment so readers can map it before the writer
/// starts.  Uses raw POSIX calls — does NOT unlink on destruction.
inline void ensure_shm_exists(const std::string& name, size_t size) {
    int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd >= 0) {
        if (ftruncate(fd, static_cast<off_t>(size)) < 0) {
            spdlog::warn("[ensure_shm_exists] ftruncate failed for '{}'", name);
        }
        ::close(fd);
    }
}

// ─────────────────────────────────────────────────────────────
// SHM Service Client
// ─────────────────────────────────────────────────────────────

template <typename Req, typename Resp>
class ShmServiceClient final : public IServiceClient<Req, Resp> {
public:
    /// @param req_topic   SHM name for requests  (client writes, server reads).
    /// @param resp_topic  SHM name for responses (server writes, client reads).
    ShmServiceClient(const std::string& req_topic,
                     const std::string& resp_topic)
        : req_topic_(req_topic), resp_topic_(resp_topic)
    {
        if (!req_writer_.create(req_topic)) {
            spdlog::error("[ShmServiceClient] Failed to create req SHM '{}'",
                          req_topic);
        }
        // Ensure resp SHM exists so we can map it before the server starts.
        // When the server's ShmWriter::create() runs later it will
        // re-open the same named segment (O_CREAT is idempotent) and
        // both sides share the same backing memory.
        ensure_shm_exists(resp_topic,
            sizeof(typename ShmWriter<ServiceResponse<Resp>>::ShmBlock));
        if (!resp_reader_.open(resp_topic)) {
            spdlog::warn("[ShmServiceClient] Still failed to open resp SHM '{}'",
                         resp_topic);
        }
    }

    uint64_t send_request(const Req& request) override {
        uint64_t id = next_id_.fetch_add(1, std::memory_order_relaxed);
        ServiceEnvelope<Req> env;
        env.correlation_id = id;
        env.timestamp_ns = now_ns();
        env.valid = true;
        env.payload = request;
        req_writer_.write(env);
        return id;
    }

    std::optional<ServiceResponse<Resp>> poll_response(
        uint64_t correlation_id) override
    {
        if (!resp_reader_.is_open()) return std::nullopt;
        ServiceResponse<Resp> resp;
        if (resp_reader_.read(resp) && resp.valid &&
            resp.correlation_id == correlation_id) {
            return resp;
        }
        return std::nullopt;
    }

private:
    ShmWriter<ServiceEnvelope<Req>> req_writer_;
    ShmReader<ServiceResponse<Resp>> resp_reader_;
    std::string req_topic_;
    std::string resp_topic_;
    std::atomic<uint64_t> next_id_{1};

    static uint64_t now_ns() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }
};

// ─────────────────────────────────────────────────────────────
// SHM Service Server
// ─────────────────────────────────────────────────────────────

template <typename Req, typename Resp>
class ShmServiceServer final : public IServiceServer<Req, Resp> {
public:
    /// @param req_topic   SHM name for requests  (client writes, server reads).
    /// @param resp_topic  SHM name for responses (server writes, client reads).
    ShmServiceServer(const std::string& req_topic,
                     const std::string& resp_topic)
        : req_topic_(req_topic), resp_topic_(resp_topic)
    {
        // Ensure req SHM exists so we can map it before the client starts.
        ensure_shm_exists(req_topic,
            sizeof(typename ShmWriter<ServiceEnvelope<Req>>::ShmBlock));
        if (!req_reader_.open(req_topic)) {
            spdlog::warn("[ShmServiceServer] Failed to open req SHM '{}'",
                         req_topic);
        }
        if (!resp_writer_.create(resp_topic)) {
            spdlog::error("[ShmServiceServer] Failed to create resp SHM '{}'",
                          resp_topic);
        }
    }

    std::optional<ServiceEnvelope<Req>> poll_request() override {
        if (!req_reader_.is_open()) return std::nullopt;
        ServiceEnvelope<Req> env;
        if (req_reader_.read(env) && env.valid &&
            env.correlation_id != last_processed_id_) {
            last_processed_id_ = env.correlation_id;
            return env;
        }
        return std::nullopt;
    }

    void send_response(uint64_t correlation_id,
                        ServiceStatus status,
                        const Resp& response) override
    {
        ServiceResponse<Resp> resp;
        resp.correlation_id = correlation_id;
        resp.timestamp_ns = now_ns();
        resp.status = status;
        resp.valid = true;
        resp.payload = response;
        resp_writer_.write(resp);
    }

private:
    ShmReader<ServiceEnvelope<Req>> req_reader_;
    ShmWriter<ServiceResponse<Resp>> resp_writer_;
    std::string req_topic_;
    std::string resp_topic_;
    uint64_t last_processed_id_ = 0;

    static uint64_t now_ns() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }
};

}  // namespace drone::ipc
