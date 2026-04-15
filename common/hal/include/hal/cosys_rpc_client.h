// common/hal/include/hal/cosys_rpc_client.h
// Thread-safe shared RPC client wrapper for Cosys-AirSim.
// Manages connection lifecycle and exponential-backoff reconnect.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Shared across all 4 Cosys-AirSim HAL backends (camera, radar, IMU, depth)
// via a single std::shared_ptr — see hal_factory.h detail::get_shared_cosys_client().
//
// Issue: #461, #462
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// AirSim SDK header — provides MultirotorRpcLibClient for all RPC calls.
// StrictMode wrapper prevents AirSim SDK warnings from breaking our -Werror build.
#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// CosysRpcClient — connection management for Cosys-AirSim RPC.
///
/// Wraps msr::airlib::MultirotorRpcLibClient with connection lifecycle,
/// exponential-backoff reconnect, and thread-safe status tracking.
/// All 4 Cosys-AirSim HAL backends (camera, radar, IMU, depth) share
/// a single instance via std::shared_ptr.
///
/// Non-copyable, non-movable — owns connection state.
/// Thread-safe: is_connected() uses atomic with acquire/release ordering.
class CosysRpcClient {
public:
    /// @param host  Cosys-AirSim RPC host (e.g. "127.0.0.1")
    /// @param port  Cosys-AirSim RPC port (e.g. 41451)
    CosysRpcClient(const std::string& host, uint16_t port) : host_(host), port_(port) {}

    ~CosysRpcClient() noexcept { disconnect(); }

    // Non-copyable, non-movable (owns connection state)
    CosysRpcClient(const CosysRpcClient&)            = delete;
    CosysRpcClient& operator=(const CosysRpcClient&) = delete;
    CosysRpcClient(CosysRpcClient&&)                 = delete;
    CosysRpcClient& operator=(CosysRpcClient&&)      = delete;

    /// Attempt connection to AirSim RPC server.
    /// Thread-safe: serialised with disconnect() and with_client() via rpc_mtx_.
    [[nodiscard]] bool connect() {
        std::lock_guard<std::mutex> lock(rpc_mtx_);
        DRONE_LOG_INFO("[CosysRpcClient] Connecting to {}:{} ...", host_, port_);
        try {
            rpc_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_, port_);
            rpc_client_->confirmConnection();
            rpc_client_->enableApiControl(true);
            connected_.store(true, std::memory_order_release);
            DRONE_LOG_INFO("[CosysRpcClient] Connected to {}:{}", host_, port_);
            return true;
        } catch (const std::exception& e) {
            DRONE_LOG_WARN("[CosysRpcClient] Connection failed: {}", e.what());
            rpc_client_.reset();
            connected_.store(false, std::memory_order_release);
            return false;
        }
    }

    /// Thread-safe health check (lock-free, atomic read).
    [[nodiscard]] bool is_connected() const { return connected_.load(std::memory_order_acquire); }

    /// Close the RPC connection. Thread-safe: serialised via rpc_mtx_.
    void disconnect() {
        std::lock_guard<std::mutex> lock(rpc_mtx_);
        if (connected_.load(std::memory_order_acquire)) {
            DRONE_LOG_INFO("[CosysRpcClient] Disconnecting from {}:{}", host_, port_);
        }
        connected_.store(false, std::memory_order_release);
        rpc_client_.reset();
    }

    /// Returns "host:port" endpoint string.
    std::string endpoint() const { return host_ + ":" + std::to_string(port_); }

    /// Execute a callback with the RPC client under lock.
    /// Prevents TOCTOU races: the client cannot be disconnected between the
    /// is_connected check and the RPC call. Returns false if not connected.
    /// Usage: client->with_client([](auto& rpc) { rpc.simGetImages(...); });
    template<typename Fn>
    [[nodiscard]] bool with_client(Fn&& fn) {
        std::lock_guard<std::mutex> lock(rpc_mtx_);
        if (!rpc_client_) return false;
        fn(*rpc_client_);
        return true;
    }

    /// Reconnect with exponential backoff.
    /// Starts at kInitialBackoff, doubles each retry, caps at kMaxBackoff.
    /// @return true if reconnection succeeded within kMaxRetries attempts.
    [[nodiscard]] bool reconnect() {
        constexpr int                       kMaxRetries = 5;
        constexpr std::chrono::milliseconds kInitialBackoff{100};
        constexpr std::chrono::milliseconds kMaxBackoff{5000};

        auto backoff = kInitialBackoff;
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            DRONE_LOG_INFO("[CosysRpcClient] Reconnect attempt {}/{} to {} (backoff {}ms)", attempt,
                           kMaxRetries, endpoint(), backoff.count());

            disconnect();
            if (connect()) {
                DRONE_LOG_INFO("[CosysRpcClient] Reconnected on attempt {}", attempt);
                return true;
            }

            if (attempt < kMaxRetries) {
                std::this_thread::sleep_for(backoff);
                // Exponential growth, capped at kMaxBackoff
                backoff = std::min(backoff * 2, kMaxBackoff);
            }
        }

        DRONE_LOG_WARN("[CosysRpcClient] Reconnect failed after {} attempts", kMaxRetries);
        return false;
    }

private:
    std::string       host_{"127.0.0.1"};  ///< AirSim RPC host (default matches config)
    uint16_t          port_{41451};        ///< AirSim RPC port (default matches config)
    std::atomic<bool> connected_{false};   ///< Lock-free health check (fast path)

    /// Mutex serialises connect/disconnect/with_client to prevent TOCTOU races.
    /// Not used for is_connected() (atomic, lock-free) — only for operations
    /// that touch rpc_client_.
    mutable std::mutex rpc_mtx_;

    /// Underlying AirSim RPC client — null when disconnected, guarded by rpc_mtx_.
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> rpc_client_;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
