// common/ipc/include/ipc/zenoh_liveliness.h
// Process health monitoring via Zenoh liveliness tokens.
//
// Each process declares a LivelinessToken at startup:
//   LivelinessToken token("video_capture");
//   → declares key expression "drone/alive/video_capture"
//
// When the process exits (cleanly or crash), the token is automatically
// dropped by the Zenoh transport layer, and any LivelinessMonitor
// watching "drone/alive/**" receives an immediate death callback.
//
// This replaces custom heartbeat/polling for process health detection
// with zero network overhead and sub-second crash detection.
//
// See: Issue #51, docs/process-health-monitoring.md
//
// Zenoh is a required dependency — this file is always compiled.
#pragma once

#include "ipc/zenoh_session.h"

#include <algorithm>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

namespace drone::ipc {

/// Key expression prefix for all liveliness tokens.
static constexpr const char* kLivelinessPrefix = "drone/alive/";

/// Key expression wildcard for monitoring all process tokens.
static constexpr const char* kLivelinessWildcard = "drone/alive/**";

// ═══════════════════════════════════════════════════════════
// LivelinessToken — declare process liveness
// ═══════════════════════════════════════════════════════════

/// RAII liveliness token.  Declare one per process at startup.
///
/// Construction declares "drone/alive/{process_name}" on the Zenoh
/// session.  Destruction (or process crash) automatically signals
/// death to all LivelinessMonitor subscribers.
///
/// Usage:
///   drone::ipc::LivelinessToken token("video_capture");
///   // ... process runs ...
///   // token dropped on scope exit → monitors notified
class LivelinessToken {
public:
    /// Declare a liveliness token for the named process.
    /// @param process_name  Short name (e.g. "video_capture", "comms").
    ///                      Produces key expression "drone/alive/{process_name}".
    explicit LivelinessToken(const std::string& process_name)
        : key_expr_(std::string(kLivelinessPrefix) + process_name) {
        try {
            auto& session = ZenohSession::instance().session();
            auto  key     = zenoh::KeyExpr(key_expr_);
            token_.emplace(session.liveliness_declare_token(key));
            valid_ = true;
            spdlog::info("[Liveliness] Token declared: {}", key_expr_);
        } catch (const std::exception& e) {
            spdlog::error("[Liveliness] Failed to declare token '{}': {}", key_expr_, e.what());
            valid_ = false;
        }
    }

    ~LivelinessToken() {
        if (valid_ && token_.has_value()) {
            try {
                token_.reset();  // RAII — destructor undeclares the token
                spdlog::info("[Liveliness] Token undeclared: {}", key_expr_);
            } catch (const std::exception& e) {
                spdlog::warn("[Liveliness] Error undeclaring token: {}", e.what());
            }
        }
    }

    /// The full key expression (e.g. "drone/alive/video_capture").
    [[nodiscard]] const std::string& key_expr() const { return key_expr_; }

    /// Returns true if the token was successfully declared.
    [[nodiscard]] bool is_valid() const { return valid_; }

    /// Extract the process name from a liveliness key expression.
    /// "drone/alive/video_capture" → "video_capture"
    [[nodiscard]] static std::string extract_process_name(const std::string& key) {
        const auto prefix_len = std::string_view(kLivelinessPrefix).size();
        if (key.size() >= prefix_len && key.substr(0, prefix_len) == kLivelinessPrefix) {
            return key.substr(prefix_len);
        }
        return key;  // fallback: return as-is
    }

    // Non-copyable (token is RAII), movable
    LivelinessToken(const LivelinessToken&)            = delete;
    LivelinessToken& operator=(const LivelinessToken&) = delete;
    LivelinessToken(LivelinessToken&&)                 = default;
    LivelinessToken& operator=(LivelinessToken&&)      = default;

private:
    std::string                           key_expr_;
    std::optional<zenoh::LivelinessToken> token_;
    bool                                  valid_ = false;
};

// ═══════════════════════════════════════════════════════════
// LivelinessMonitor — watch process health
// ═══════════════════════════════════════════════════════════

/// Monitors liveliness tokens on "drone/alive/**".
///
/// Receives callbacks when processes come alive (PUT) or die (DELETE).
/// Maintains an internal set of alive process names queryable via
/// get_alive_processes().
///
/// Typically used by Process 7 (System Monitor) but can be used by
/// any process that needs to react to peer deaths.
///
/// Usage:
///   LivelinessMonitor monitor(
///       [](const std::string& p) { spdlog::info("ALIVE: {}", p); },
///       [](const std::string& p) { spdlog::error("DIED: {}", p); }
///   );
///   auto alive = monitor.get_alive_processes();
class LivelinessMonitor {
public:
    using AliveCallback = std::function<void(const std::string& process_name)>;
    using DeathCallback = std::function<void(const std::string& process_name)>;

    /// Start monitoring liveliness tokens.
    /// @param on_alive  Called when a process declares its token (or is
    ///                  discovered via history on startup).
    /// @param on_death  Called when a process token is dropped (crash/exit).
    LivelinessMonitor(AliveCallback on_alive, DeathCallback on_death)
        : on_alive_(std::move(on_alive)), on_death_(std::move(on_death)) {
        try {
            auto& session = ZenohSession::instance().session();
            auto  key     = zenoh::KeyExpr(kLivelinessWildcard);

            // Request history so we get PUT for already-declared tokens
            zenoh::Session::LivelinessSubscriberOptions opts;
            opts.history = true;

            subscriber_.emplace(session.liveliness_declare_subscriber(
                key,
                // on_sample — PUT = alive, DELETE = death
                [this](zenoh::Sample& sample) {
                    auto name = LivelinessToken::extract_process_name(
                        std::string(sample.get_keyexpr().as_string_view()));

                    if (sample.get_kind() == Z_SAMPLE_KIND_PUT) {
                        bool is_new = false;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (std::find(alive_set_.begin(), alive_set_.end(), name) ==
                                alive_set_.end()) {
                                alive_set_.push_back(name);
                                is_new = true;
                            }
                        }
                        if (is_new && on_alive_) on_alive_(name);
                    } else if (sample.get_kind() == Z_SAMPLE_KIND_DELETE) {
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            alive_set_.erase(
                                std::remove(alive_set_.begin(), alive_set_.end(), name),
                                alive_set_.end());
                        }
                        if (on_death_) on_death_(name);
                    }
                },
                // on_drop (subscriber destroyed)
                []() {}, std::move(opts)));

            spdlog::info("[Liveliness] Monitor started on '{}'", kLivelinessWildcard);
        } catch (const std::exception& e) {
            spdlog::error("[Liveliness] Failed to start monitor: {}", e.what());
        }
    }

    ~LivelinessMonitor() = default;

    /// Returns a snapshot of currently alive process names.
    [[nodiscard]] std::vector<std::string> get_alive_processes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return alive_set_;
    }

    /// Returns true if a specific process is currently alive.
    [[nodiscard]] bool is_alive(const std::string& process_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::find(alive_set_.begin(), alive_set_.end(), process_name) != alive_set_.end();
    }

    // Non-copyable, non-movable (subscriber captures `this`)
    LivelinessMonitor(const LivelinessMonitor&)            = delete;
    LivelinessMonitor& operator=(const LivelinessMonitor&) = delete;
    LivelinessMonitor(LivelinessMonitor&&)                 = delete;
    LivelinessMonitor& operator=(LivelinessMonitor&&)      = delete;

private:
    AliveCallback                          on_alive_;
    DeathCallback                          on_death_;
    mutable std::mutex                     mutex_;
    std::vector<std::string>               alive_set_;
    std::optional<zenoh::Subscriber<void>> subscriber_;
};

}  // namespace drone::ipc
