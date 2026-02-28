// common/ipc/include/ipc/zenoh_session.h
// Singleton Zenoh session manager — shared by all publishers/subscribers.
//
// All ZenohPublisher/ZenohSubscriber instances share a single Zenoh session
// in "peer" mode (no daemon required). SHM provider is enabled by default
// for zero-copy local IPC.
//
// Guarded by HAVE_ZENOH — this file is a no-op when Zenoh is not available.
#pragma once

#ifdef HAVE_ZENOH

#include <zenoh.hxx>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace drone::ipc {

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
        configured_ = true;
        spdlog::info("[ZenohSession] Configured ({})",
                     config_json.empty() ? "defaults" : "custom");
    }

    /// Get the underlying Zenoh session.  Opens lazily on first call.
    zenoh::Session& session() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!session_.has_value()) {
            open_session();
        }
        return session_.value();
    }

    /// Returns true if the session has been opened.
    bool is_open() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_.has_value();
    }

    // Non-copyable, non-movable
    ZenohSession(const ZenohSession&) = delete;
    ZenohSession& operator=(const ZenohSession&) = delete;
    ZenohSession(ZenohSession&&) = delete;
    ZenohSession& operator=(ZenohSession&&) = delete;

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
        spdlog::info("[ZenohSession] Session opened (peer mode)");
    }

    mutable std::mutex mutex_;
    std::optional<zenoh::Session> session_;
    std::string config_json_;
    bool configured_ = false;
};

}  // namespace drone::ipc

#endif  // HAVE_ZENOH
