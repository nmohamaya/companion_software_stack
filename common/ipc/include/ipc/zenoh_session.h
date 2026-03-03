// common/ipc/include/ipc/zenoh_session.h
// Singleton Zenoh session manager — shared by all publishers/subscribers.
//
// All ZenohPublisher/ZenohSubscriber instances share a single Zenoh session.
// The session mode depends on configuration:
//   - Local (default): peer mode, no network listeners
//   - Drone:           peer mode + TCP/UDP listener for GCS connections
//   - GCS:             client mode, connects to drone endpoint
//
// A PosixShmProvider is lazily created alongside the session for zero-copy
// high-bandwidth IPC (video frames).
//
// Guarded by HAVE_ZENOH — this file is a no-op when Zenoh is not available.
#pragma once

#ifdef HAVE_ZENOH

#include "ipc/zenoh_network_config.h"

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

namespace drone::ipc {

/// Default Zenoh SHM pool size: 32 MB.
/// Must hold at least 2× the largest message in flight (ShmVideoFrame ≈ 6 MB).
/// Configurable via configure_shm().
static constexpr std::size_t kDefaultShmPoolBytes = 32 * 1024 * 1024;

/// Messages larger than this threshold are published via the SHM provider
/// (zero-copy path).  Smaller messages use the regular Bytes path.
static constexpr std::size_t kShmPublishThreshold = 64 * 1024;  // 64 KB

/// Process-wide Zenoh session.  Thread-safe singleton.
/// Configure before first use via configure(); defaults to peer mode.
///
/// NOTE: The singleton is heap-allocated and intentionally leaked.
/// zenohc (Rust FFI) panics if its Session destructor runs during
/// atexit() — a known Rust 1.85 issue (rust-lang/rust#138696).
/// Leaking avoids the crash; the OS reclaims resources on exit.
class ZenohSession {
public:
    /// Access the singleton instance (intentionally leaked — never destroyed).
    static ZenohSession& instance() {
        static ZenohSession* inst = new ZenohSession();
        return *inst;
    }

    /// Configure the session before first use.
    /// @param config_json  Optional JSON config string (Zenoh config format).
    ///                     Empty string → default peer-mode config.
    /// Must be called BEFORE session() is first invoked.
    void configure(const std::string& config_json = "") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_.has_value()) {
            spdlog::warn("[ZenohSession] configure() called after session "
                         "already opened — ignoring");
            return;
        }
        config_json_ = config_json;
        configured_  = true;
        spdlog::info("[ZenohSession] Configured ({})", config_json.empty() ? "defaults" : "custom");
    }

    /// Configure network transport before first use.
    /// @param net_config  Network configuration (endpoints, mode, scouting).
    /// Calls configure() with the generated JSON string.
    /// Must be called BEFORE session() is first invoked.
    void configure_network(const ZenohNetworkConfig& net_config) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_.has_value()) {
            spdlog::warn("[ZenohSession] configure_network() called after "
                         "session already opened — ignoring");
            return;
        }
        config_json_     = net_config.to_json();
        network_enabled_ = true;
        configured_      = true;
        spdlog::info("[ZenohSession] Network configured: mode={}, "
                     "listen_eps={}, connect_eps={}",
                     net_config.mode, net_config.listen_endpoints.size(),
                     net_config.connect_endpoints.size());
    }

    /// Configure SHM pool size before first use.
    /// @param pool_bytes  Size of the POSIX SHM pool in bytes.
    ///                    0 = disable SHM provider (use regular Bytes path).
    void configure_shm(std::size_t pool_bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shm_provider_) {
            spdlog::warn("[ZenohSession] configure_shm() called after SHM "
                         "provider already created — ignoring");
            return;
        }
        shm_pool_bytes_ = pool_bytes;
        spdlog::info("[ZenohSession] SHM pool configured: {} bytes", pool_bytes);
    }

    /// Get the underlying Zenoh session.  Opens lazily on first call.
    zenoh::Session& session() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!session_.has_value()) {
            open_session();
        }
        return session_.value();
    }

    /// Get the POSIX SHM provider.  Created lazily on first call.
    /// Returns nullptr if SHM is disabled (pool_bytes == 0).
    zenoh::PosixShmProvider* shm_provider() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!shm_provider_ && shm_pool_bytes_ > 0) {
            create_shm_provider();
        }
        return shm_provider_.get();
    }

    /// Returns true if the session has been opened.
    bool is_open() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_.has_value();
    }

    /// Returns configured SHM pool size (bytes).  0 = disabled.
    std::size_t shm_pool_bytes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return shm_pool_bytes_;
    }

    /// Returns true if network transport is enabled.
    bool is_network_enabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return network_enabled_;
    }

    /// Set network-enabled flag (used when configure() is called directly
    /// with a network JSON config instead of via configure_network()).
    void set_network_enabled(bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        network_enabled_ = enabled;
    }

    // Non-copyable, non-movable
    ZenohSession(const ZenohSession&)            = delete;
    ZenohSession& operator=(const ZenohSession&) = delete;
    ZenohSession(ZenohSession&&)                 = delete;
    ZenohSession& operator=(ZenohSession&&)      = delete;

private:
    ZenohSession() = default;

    void open_session() {
        if (!config_json_.empty()) {
            auto config = zenoh::Config::from_str(config_json_);
            session_.emplace(zenoh::Session::open(std::move(config)));
        } else {
            auto config = zenoh::Config::create_default();
            session_.emplace(zenoh::Session::open(std::move(config)));
        }
        spdlog::info("[ZenohSession] Session opened ({})",
                     network_enabled_ ? "network-enabled" : "peer mode");
    }

    void create_shm_provider() {
        try {
            shm_provider_ = std::make_unique<zenoh::PosixShmProvider>(shm_pool_bytes_);
            spdlog::info("[ZenohSession] SHM provider created "
                         "(pool={} MB)",
                         shm_pool_bytes_ / (1024 * 1024));
        } catch (const std::exception& e) {
            spdlog::error("[ZenohSession] Failed to create SHM provider: {}", e.what());
        }
    }

    mutable std::mutex            mutex_;
    std::optional<zenoh::Session> session_;
    std::string                   config_json_;
    bool                          configured_      = false;
    bool                          network_enabled_ = false;

    // SHM provider for zero-copy large-message publishing
    std::size_t                              shm_pool_bytes_ = kDefaultShmPoolBytes;
    std::unique_ptr<zenoh::PosixShmProvider> shm_provider_;
};

}  // namespace drone::ipc

#endif  // HAVE_ZENOH
