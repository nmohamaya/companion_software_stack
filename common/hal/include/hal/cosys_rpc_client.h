// common/hal/include/hal/cosys_rpc_client.h
// Thread-safe shared RPC client wrapper for Cosys-AirSim.
// Manages connection lifecycle and exponential-backoff reconnect.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Shared across all 4 Cosys-AirSim HAL backends (camera, radar, IMU, depth)
// via a single std::shared_ptr — see hal_factory.h detail::get_shared_cosys_client().
//
// Issue: #461
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace drone::hal {

/// CosysRpcClient — connection management skeleton for Cosys-AirSim RPC.
///
/// This is a stub: connect() logs and returns false until the real AirSim
/// RPC calls are wired in (#462). The reconnect logic (exponential backoff)
/// is fully implemented and ready for the SDK integration.
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
    /// Stub: logs and returns false until real RPC integration (#462).
    [[nodiscard]] bool connect() {
        DRONE_LOG_INFO("[CosysRpcClient] Connecting to {}:{} ...", host_, port_);
        // TODO(#462): Real AirSim RPC connection via msr::airlib::RpcLibClientBase
        connected_.store(false, std::memory_order_release);
        DRONE_LOG_WARN("[CosysRpcClient] Stub — no real RPC connection yet");
        return false;
    }

    /// Thread-safe health check.
    [[nodiscard]] bool is_connected() const { return connected_.load(std::memory_order_acquire); }

    /// Close the RPC connection.
    void disconnect() {
        if (connected_.load(std::memory_order_acquire)) {
            DRONE_LOG_INFO("[CosysRpcClient] Disconnecting from {}:{}", host_, port_);
        }
        connected_.store(false, std::memory_order_release);
    }

    /// Returns "host:port" endpoint string.
    std::string endpoint() const { return host_ + ":" + std::to_string(port_); }

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
    std::atomic<bool> connected_{false};   ///< Thread-safe connection status
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
