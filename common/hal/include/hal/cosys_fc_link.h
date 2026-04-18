// common/hal/include/hal/cosys_fc_link.h
// Cosys-AirSim flight controller link backend — SimpleFlight via AirSim RPC.
// Implements IFCLink — issues drone commands through MultirotorRpcLibClient
// (takeoff / land / move-by-velocity / arm / hover / go-home) and polls
// getMultirotorState() at 10 Hz for cached telemetry.
//
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// This backend is the Tier-3 counterpart to MavlinkFCLink (Tier-2).
// PX4+Cosys HIL integration is broken upstream (PX4 #24033, AirSim #5018),
// so Tier 3 uses AirSim's built-in SimpleFlight flight controller instead.
// Battery/RC/datalink fault simulation remains in Gazebo Tier 2.
//
// Coordinate convention:
//   IFCLink uses ENU-like velocity (vx=north, vy=east, vz=+up).
//   AirSim is pure NED (vz=+down). This backend negates z on commands
//   and on state read-back so callers see consistent ENU semantics.
//
// Shared RPC client:
//   The shared CosysRpcClient already calls confirmConnection() +
//   enableApiControl(true) during its own connect().  open() therefore
//   doesn't re-initialise the connection; it only starts the state
//   polling thread.  close() does NOT call disconnect() on the shared
//   client because other HAL backends (camera/imu/radar/depth) still
//   use it.
//
// Issue: #490
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/ifc_link.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorCommon.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim flight-controller link using SimpleFlight via AirSim RPC.
///
/// Thread-safety:
///   - conn_mtx_ guards connection lifecycle (open/close) and serialises
///     all command-path RPC calls issued through CosysRpcClient::with_client,
///     which itself takes an internal lock.
///   - state_mtx_ guards the cached FCState populated by the 10 Hz poll
///     thread.  receive_state() copies under this lock.
///   - thread_running_ is an atomic flag consulted by the poll loop.
class CosysFCLink : public IFCLink {
public:
    /// @param client   Shared RPC client (manages connection lifecycle)
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "comms.mavlink")
    explicit CosysFCLink(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                         const std::string& section)
        : client_(std::move(client))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0")) {
        (void)section;  // section reserved for future per-link config
        DRONE_LOG_INFO("[CosysFCLink] Created for {} vehicle='{}'", client_->endpoint(),
                       vehicle_name_);
    }

    ~CosysFCLink() override { close(); }

    // Non-copyable, non-movable (owns polling thread and RPC lifecycle)
    CosysFCLink(const CosysFCLink&)            = delete;
    CosysFCLink& operator=(const CosysFCLink&) = delete;
    CosysFCLink(CosysFCLink&&)                 = delete;
    CosysFCLink& operator=(CosysFCLink&&)      = delete;

    // ── IFCLink interface ──────────────────────────────────

    /// Open the link.
    /// @param port  Ignored (RPC endpoint comes from cosys_airsim config section).
    /// @param baud  Ignored.
    ///
    /// The shared CosysRpcClient handles connection; we only verify it is
    /// connected, re-assert confirmConnection()/enableApiControl(true)
    /// (idempotent and cheap — also protects against another backend
    /// having disabled API control), and start the 10 Hz state poll thread.
    bool open(const std::string& port, int baud) override {
        (void)port;
        (void)baud;
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (thread_running_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysFCLink] Already open — call close() first");
            return false;
        }

        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysFCLink] RPC client not connected — cannot open FC link");
            return false;
        }

        bool rpc_ok = false;
        bool ok     = client_->with_client([this, &rpc_ok](auto& rpc) {
            try {
                rpc.confirmConnection();
                rpc.enableApiControl(true, vehicle_name_);
                rpc_ok = true;
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] open(): RPC setup failed: {}", e.what());
            }
        });
        if (!ok || !rpc_ok) {
            DRONE_LOG_ERROR("[CosysFCLink] open(): RPC setup failed");
            return false;
        }

        // Set the flag before constructing the thread so poll_loop's first
        // load(acquire) observes true. If std::thread construction throws
        // (EAGAIN, OOM, pthread failure), reset the flag so a subsequent
        // open() call doesn't observe a stuck "running" state with no thread.
        thread_running_.store(true, std::memory_order_release);
        try {
            poll_thread_ = std::thread(&CosysFCLink::poll_loop, this);
        } catch (const std::system_error& e) {
            thread_running_.store(false, std::memory_order_release);
            DRONE_LOG_ERROR("[CosysFCLink] Poll thread start failed: {}", e.what());
            return false;
        }

        DRONE_LOG_INFO("[CosysFCLink] Opened on {} vehicle='{}'", client_->endpoint(),
                       vehicle_name_);
        return true;
    }

    void close() override {
        {
            std::lock_guard<std::mutex> lock(conn_mtx_);
            if (!thread_running_.load(std::memory_order_acquire)) return;
            thread_running_.store(false, std::memory_order_release);
        }

        // Join the poll thread outside the lock to avoid deadlock
        // (the thread itself takes conn_mtx_ via with_client).
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }

        // Re-acquire the lock to release API control.
        // NOTE: we intentionally do NOT call client_->disconnect() —
        // other Cosys HAL backends (camera, imu, radar, depth) share the
        // same RPC client and still need it.  Disabling API control is
        // the mirror of enabling it in open(); a single process has one
        // FC link, so this does not affect other backends' sensor polls.
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (client_->is_connected()) {
            (void)client_->with_client([this](auto& rpc) {
                try {
                    rpc.enableApiControl(false, vehicle_name_);
                } catch (const std::exception& e) {
                    DRONE_LOG_WARN("[CosysFCLink] close(): enableApiControl(false) failed: {}",
                                   e.what());
                }
            });
        }

        {
            std::lock_guard<std::mutex> st_lock(state_mtx_);
            cached_state_ = FCState{};
        }
        // Clear the send_mode override so a subsequent open() starts fresh.
        requested_mode_.store(UINT8_MAX, std::memory_order_release);

        DRONE_LOG_INFO("[CosysFCLink] Closed");
    }

    bool is_connected() const override {
        return client_->is_connected() && thread_running_.load(std::memory_order_acquire);
    }

    /// Send a velocity command in ENU-like frame.
    /// @param vx  North (forward) velocity (m/s)
    /// @param vy  East  (right)   velocity (m/s)
    /// @param vz  Up velocity     (m/s, positive = climb)
    /// @param yaw Yaw angle (radians, 0 = North, CW positive)
    ///
    /// AirSim takes duration-based velocity — we use 0.2 s so the next
    /// trajectory call refreshes before the command expires.  Non-blocking.
    bool send_trajectory(float vx, float vy, float vz, float yaw) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!thread_running_.load(std::memory_order_acquire)) return false;
        if (!client_->is_connected()) return false;

        constexpr float kVelocityDurationSec = 0.2f;
        const float     yaw_deg              = yaw * 180.0f / static_cast<float>(M_PI);
        // ENU (vz=+up) → NED (vz=+down)
        const float ned_vz = -vz;

        bool rpc_ok = false;
        bool ok     = client_->with_client([&](auto& rpc) {
            try {
                using namespace msr::airlib;
                YawMode yaw_mode(/*is_rate*/ false, yaw_deg);
                // moveByVelocityAsync returns a task pointer; we don't wait on it —
                // the next send_trajectory() call will overwrite the setpoint.
                rpc.moveByVelocityAsync(vx, vy, ned_vz, kVelocityDurationSec,
                                            DrivetrainType::MaxDegreeOfFreedom, yaw_mode,
                                            vehicle_name_);
                rpc_ok = true;
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] moveByVelocityAsync failed: {}", e.what());
            }
        });
        if (!ok || !rpc_ok) return false;

        DRONE_LOG_DEBUG("[CosysFCLink] velocity cmd N={:.2f} E={:.2f} U={:.2f} yaw={:.1f}°", vx, vy,
                        vz, yaw_deg);
        return true;
    }

    bool send_arm(bool arm) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        // Guard against RPC during close()'s join window: close() releases
        // conn_mtx_ while joining the poll thread, so a concurrent send_arm()
        // could otherwise slip through after thread_running_ was cleared.
        if (!thread_running_.load(std::memory_order_acquire)) return false;
        if (!client_->is_connected()) return false;

        bool rpc_result = false;
        bool rpc_ok     = false;
        bool ok         = client_->with_client([&](auto& rpc) {
            try {
                rpc_result = rpc.armDisarm(arm, vehicle_name_);
                rpc_ok     = true;
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] {} failed: {}", arm ? "Arm" : "Disarm", e.what());
            }
        });
        if (!ok || !rpc_ok) return false;
        if (!rpc_result) {
            DRONE_LOG_WARN("[CosysFCLink] armDisarm({}) rejected by SimpleFlight", arm);
            return false;
        }

        {
            std::lock_guard<std::mutex> st_lock(state_mtx_);
            cached_state_.armed = arm;
        }
        DRONE_LOG_INFO("[CosysFCLink] {} successful", arm ? "Armed" : "Disarmed");
        return true;
    }

    /// Map IFCLink mode codes to AirSim drone commands:
    ///   0 = STAB    → hoverAsync() (fire-and-forget; engaged by next tick)
    ///   1 = GUIDED  → no-op  (velocity commands drive SimpleFlight directly)
    ///   2 = AUTO    → landAsync() (we treat "AUTO" as auto-land for Tier 3)
    ///   3 = RTL     → goHomeAsync()
    ///
    /// All RPCs are fire-and-forget: blocking on waitOnLastTask() under
    /// conn_mtx_ would stall every other command path (send_arm / trajectory /
    /// mode) for up to the RPC's timeout. Caller is expected to poll
    /// receive_state().flight_mode to confirm mode engagement.
    bool send_mode(uint8_t mode) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!thread_running_.load(std::memory_order_acquire)) return false;
        if (!client_->is_connected()) return false;

        if (mode > 3) {
            DRONE_LOG_WARN("[CosysFCLink] Unknown mode {}", mode);
            return false;
        }

        bool rpc_ok = false;
        bool ok     = client_->with_client([&](auto& rpc) {
            try {
                switch (mode) {
                    case 0:  // STAB → hover (non-blocking)
                        rpc.hoverAsync(vehicle_name_);
                        break;
                    case 1:  // GUIDED → no-op; send_trajectory drives motion
                        break;
                    case 2:  // AUTO → land (non-blocking)
                        rpc.landAsync(/*timeout_sec*/ 60.0f, vehicle_name_);
                        break;
                    case 3:  // RTL → go home (non-blocking)
                        rpc.goHomeAsync(/*timeout_sec*/ 60.0f, vehicle_name_);
                        break;
                }
                rpc_ok = true;
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] send_mode({}) failed: {}", mode, e.what());
            }
        });
        if (!ok || !rpc_ok) return false;

        // Record the explicitly requested mode so the poll loop can report it
        // back verbatim. Using a dedicated atomic (with UINT8_MAX as the
        // "never set" sentinel) avoids the 0 == STAB == "never set" collision
        // that existed when we piggy-backed on cached_state_.flight_mode.
        requested_mode_.store(mode, std::memory_order_release);
        DRONE_LOG_INFO("[CosysFCLink] Mode changed to {}", mode);
        return true;
    }

    /// Autonomous takeoff — fire-and-forget.
    ///
    /// Fires AirSim's takeoffAsync() followed by a non-blocking moveToZAsync
    /// to the requested altitude (caller-frame +up; NED → negate z). Neither
    /// RPC is awaited here — blocking on waitOnLastTask() under conn_mtx_
    /// would stall the trajectory/arm/mode paths for up to 20 s.
    ///
    /// The caller (mission-planner FSM) is expected to poll
    /// `receive_state().altitude_rel` to detect takeoff completion.
    bool send_takeoff(float altitude_m) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!thread_running_.load(std::memory_order_acquire)) return false;
        if (!client_->is_connected()) return false;

        constexpr float kTakeoffTimeoutSec = 20.0f;
        constexpr float kClimbSpeedMps     = 2.0f;
        const float     ned_z              = -altitude_m;  // AGL (+up) → NED (+down)

        bool rpc_ok = false;
        bool ok     = client_->with_client([&](auto& rpc) {
            try {
                // Fire-and-forget: don't waitOnLastTask() under conn_mtx_.
                rpc.takeoffAsync(kTakeoffTimeoutSec, vehicle_name_);
                rpc.moveToZAsync(ned_z, kClimbSpeedMps, /*timeout_sec*/ 60.0f,
                                     msr::airlib::YawMode(), /*lookahead*/ -1.0f,
                                     /*adaptive_lookahead*/ 1.0f, vehicle_name_);
                rpc_ok = true;
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] takeoff({:.1f}m) failed: {}", altitude_m, e.what());
            }
        });
        if (!ok || !rpc_ok) return false;

        DRONE_LOG_INFO("[CosysFCLink] Takeoff to {:.1f}m initiated (non-blocking)", altitude_m);
        return true;
    }

    FCState receive_state() override {
        std::lock_guard<std::mutex> lock(state_mtx_);
        return cached_state_;
    }

    std::string name() const override { return "CosysFCLink"; }

private:
    /// State poll loop — runs at 10 Hz (kPollIntervalMs = 100 ms).
    /// Calls getMultirotorState() and translates the response into our
    /// FCState struct.  SimpleFlight does not simulate battery / GPS, so
    /// those fields are filled with constant defaults.
    void poll_loop() {
        constexpr auto kPollInterval = std::chrono::milliseconds(100);  // 10 Hz
        DRONE_LOG_INFO("[CosysFCLink] Poll thread started (interval={}ms)", kPollInterval.count());

        while (thread_running_.load(std::memory_order_acquire)) {
            try {
                // Value-initialise every poll iteration so previous-tick fields
                // cannot bleed into the next snapshot on a partial RPC failure.
                msr::airlib::MultirotorState ms{};
                bool                         got = client_->with_client(
                    [&](auto& rpc) { ms = rpc.getMultirotorState(vehicle_name_); });
                if (!got) {
                    DRONE_LOG_WARN("[CosysFCLink] RPC disconnected — skipping state poll");
                    std::this_thread::sleep_for(kPollInterval);
                    continue;
                }

                FCState    s{};
                const auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch())
                                       .count();
                s.timestamp_ns = static_cast<uint64_t>(std::max(decltype(ts_ns){0}, ts_ns));

                // Altitude above spawn: AirSim position is NED (+down), so
                // negate z to report +up (above spawn). This is not true AGL
                // — SimpleFlight does not model terrain. See FCState docstring
                // in ifc_link.h.
                // NOTE: SimpleFlight reports altitude above spawn, not true AGL.
                s.altitude_rel = static_cast<float>(-ms.kinematics_estimated.pose.position.z());

                // Ground speed from horizontal NED velocity
                const float vx = static_cast<float>(ms.kinematics_estimated.twist.linear.x());
                const float vy = static_cast<float>(ms.kinematics_estimated.twist.linear.y());
                s.ground_speed = std::sqrt(vx * vx + vy * vy);

                // Battery / GPS not simulated by SimpleFlight — fill with
                // reasonable constants so downstream health checks do not
                // flag spurious faults.  Battery fault simulation remains
                // in Gazebo Tier 2 (see ADR-011 amendment for #490).
                constexpr float   kStubBatteryVoltage     = 22.2f;  // 6S LiPo nominal
                constexpr float   kStubBatteryCurrent     = 0.0f;
                constexpr float   kStubBatteryPercent     = 100.0f;
                constexpr uint8_t kStubSatellites         = 0;
                constexpr uint8_t kSimpleFlightModeGuided = 1;  // always GUIDED
                s.battery_voltage                         = kStubBatteryVoltage;
                s.battery_current                         = kStubBatteryCurrent;
                s.battery_percent                         = kStubBatteryPercent;
                s.satellites                              = kStubSatellites;
                s.flight_mode                             = kSimpleFlightModeGuided;

                // Apply explicitly-requested mode override. requested_mode_
                // is UINT8_MAX while no send_mode() has been called; any
                // real code (0-3) takes precedence over the GUIDED default.
                // This fixes the bug where send_mode(0) (STAB) collided with
                // the "never set" sentinel when piggy-backing on flight_mode.
                const uint8_t override_mode = requested_mode_.load(std::memory_order_acquire);
                if (override_mode != UINT8_MAX) {
                    s.flight_mode = override_mode;
                }

                {
                    std::lock_guard<std::mutex> lock(state_mtx_);
                    // Preserve armed state set by send_arm() / last poll so it
                    // is not clobbered when SimpleFlight does not expose it.
                    s.armed       = cached_state_.armed;
                    cached_state_ = s;
                }
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysFCLink] RPC error in poll thread: {}", e.what());
            }

            std::this_thread::sleep_for(kPollInterval);
        }

        DRONE_LOG_INFO("[CosysFCLink] Poll thread exiting");
    }

    // ── Shared RPC client (shared_ptr: shared across 5 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ─────────────────────────────────────────────
    std::string vehicle_name_;  ///< AirSim vehicle name (matches settings.json)

    // ── State ──────────────────────────────────────────────
    mutable std::mutex conn_mtx_;               ///< Guards lifecycle + command RPCs
    std::mutex         state_mtx_;              ///< Guards cached_state_
    FCState            cached_state_{};         ///< Latest telemetry snapshot
    std::atomic<bool>  thread_running_{false};  ///< Poll thread run flag

    /// Explicitly requested flight mode (set by send_mode, read by poll_loop).
    /// UINT8_MAX = never set → poll loop leaves s.flight_mode at its default
    /// (GUIDED = 1). Any real IFCLink mode (0-3) takes precedence, so
    /// send_mode(0) (STAB) is correctly reflected — unlike the previous
    /// implementation which piggy-backed on cached_state_.flight_mode and
    /// conflated STAB with "no mode ever requested".
    std::atomic<uint8_t> requested_mode_{UINT8_MAX};

    // ── Poll thread ────────────────────────────────────────
    std::thread poll_thread_;  ///< Polls getMultirotorState() at 10 Hz
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
