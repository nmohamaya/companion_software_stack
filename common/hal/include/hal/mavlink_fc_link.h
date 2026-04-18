// common/hal/include/hal/mavlink_fc_link.h
// MAVLink flight controller link backend via MAVSDK.
// Implements IFCLink — connects to PX4/ArduPilot over UDP/serial/TCP.
// Guarded by HAVE_MAVSDK (set by CMake when MAVSDK is found).
//
// This backend repurposes the IFCLink::open() "port" parameter to accept a
// MAVSDK-style connection URI (not just a serial port name). Example URIs:
//   "udp://:14540"        — PX4 SITL default
//   "serial:///dev/ttyACM0:921600"
//   "tcp://127.0.0.1:5760"
//
// Which configuration keys map to IFCLink::open(port, baud) (e.g. "uri",
// "serial_port", "baud_rate") is decided by the calling code/process, not by
// this backend implementation.
//
// Issue: #8
#pragma once
#ifdef HAVE_MAVSDK

#include "hal/ifc_link.h"
#include "util/ilogger.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace drone::hal {

/// MAVLink flight controller link using MAVSDK.
///
/// Thread-safety: MAVSDK internally serialises plugin calls.  Cached telemetry
/// state is updated from MAVSDK subscription callbacks on their own threads and
/// protected by a mutex so that `receive_state()` is thread-safe from the
/// caller's side.
class MavlinkFCLink : public IFCLink {
public:
    MavlinkFCLink() = default;
    ~MavlinkFCLink() override { close(); }

    // Non-copyable, non-movable (owns MAVSDK resources)
    MavlinkFCLink(const MavlinkFCLink&)            = delete;
    MavlinkFCLink& operator=(const MavlinkFCLink&) = delete;
    MavlinkFCLink(MavlinkFCLink&&)                 = delete;
    MavlinkFCLink& operator=(MavlinkFCLink&&)      = delete;

    // ── IFCLink interface ──────────────────────────────────

    /// Open a MAVLink connection.
    /// @param port  Connection URI (e.g. "udp://:14540").
    ///              If empty or "auto", defaults to "udp://:14540".
    /// @param baud  Timeout in milliseconds to wait for an autopilot
    ///              discovery (0 = default 8 000 ms).
    bool open(const std::string& port, int baud) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (system_) {
            DRONE_LOG_WARN("[MavlinkFCLink] Already connected — call close() first");
            return false;
        }

        // Interpret parameters
        std::string uri       = port.empty() || port == "auto" ? "udp://:14540" : port;
        double      timeout_s = baud > 0 ? static_cast<double>(baud) / 1000.0 : 8.0;

        DRONE_LOG_INFO("[MavlinkFCLink] Connecting to '{}' (timeout {:.1f}s)…", uri, timeout_s);

        // Create MAVSDK instance.  Use GroundStation type so PX4 receives
        // GCS heartbeats and passes its "GCS connected" preflight check.
        // Without this, PX4 denies arming with "No connection to the GCS".
        mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
            mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation});

        auto result = mavsdk_->add_any_connection(uri);
        if (result != mavsdk::ConnectionResult::Success) {
            DRONE_LOG_ERROR("[MavlinkFCLink] Connection failed: {}", connection_result_str(result));
            mavsdk_.reset();
            return false;
        }

        // Wait for the first autopilot system
        auto maybe_system = mavsdk_->first_autopilot(timeout_s);
        if (!maybe_system) {
            DRONE_LOG_ERROR("[MavlinkFCLink] No autopilot found within {:.1f}s", timeout_s);
            mavsdk_.reset();
            return false;
        }
        system_ = maybe_system.value();

        // Create plugins
        action_    = std::make_unique<mavsdk::Action>(system_);
        telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
        offboard_  = std::make_unique<mavsdk::Offboard>(system_);

        // Subscribe to telemetry for cached state
        setup_subscriptions();

        // Set RTL return altitude to current flight altitude so RTL
        // doesn't climb to the PX4 default of 30 m before returning.
        constexpr float rtl_alt_m  = 5.0f;
        auto            rtl_result = action_->set_return_to_launch_altitude(rtl_alt_m);
        if (rtl_result == mavsdk::Action::Result::Success) {
            DRONE_LOG_INFO("[MavlinkFCLink] RTL return altitude set to {} m", rtl_alt_m);
        } else {
            DRONE_LOG_WARN("[MavlinkFCLink] Failed to set RTL altitude: {}",
                           action_result_str(rtl_result));
        }

        DRONE_LOG_INFO("[MavlinkFCLink] Connected to autopilot (sys_id={})",
                       system_->get_system_id());
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!system_) return;

        // Stop offboard if active
        if (offboard_ && offboard_->is_active()) {
            offboard_->stop();
        }

        // Tear down plugins before system
        offboard_.reset();
        telemetry_.reset();
        action_.reset();
        system_.reset();
        mavsdk_.reset();

        {
            std::lock_guard<std::mutex> st_lock(state_mtx_);
            cached_state_ = FCState{};
        }
        offboard_active_.store(false, std::memory_order_release);

        DRONE_LOG_INFO("[MavlinkFCLink] Closed");
    }

    bool is_connected() const override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        return system_ && system_->is_connected();
    }

    /// Send a velocity command in NED frame.
    /// Starts offboard mode automatically on the first call.
    /// @param vx  North velocity (m/s)
    /// @param vy  East velocity  (m/s)
    /// @param vz  Up velocity    (m/s, positive = climb; internally negated to NED)
    /// @param yaw Yaw angle (radians, 0 = North, CW positive — per IFCLink contract).
    ///            Converted to degrees internally before hand-off to MAVSDK
    ///            VelocityNedYaw, which expects degrees.
    bool send_trajectory(float vx, float vy, float vz, float yaw) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!offboard_ || !system_ || !system_->is_connected()) return false;

        mavsdk::Offboard::VelocityNedYaw cmd{};
        cmd.north_m_s = vx;
        cmd.east_m_s  = vy;
        cmd.down_m_s  = -vz;  // Convert from +up (ENU) to +down (NED)
        // IFCLink contract: yaw is radians. MAVSDK VelocityNedYaw expects degrees.
        cmd.yaw_deg = yaw * 180.0f / static_cast<float>(M_PI);

        // Must set an initial setpoint before starting offboard
        auto set_result = offboard_->set_velocity_ned(cmd);
        if (set_result != mavsdk::Offboard::Result::Success) {
            DRONE_LOG_WARN("[MavlinkFCLink] set_velocity_ned failed: {}",
                           offboard_result_str(set_result));
            return false;
        }

        // Start offboard mode if not already active
        if (!offboard_active_.load(std::memory_order_acquire)) {
            auto start_result = offboard_->start();
            if (start_result != mavsdk::Offboard::Result::Success) {
                DRONE_LOG_WARN("[MavlinkFCLink] Offboard start failed: {}",
                               offboard_result_str(start_result));
                return false;
            }
            offboard_active_.store(true, std::memory_order_release);
            DRONE_LOG_INFO("[MavlinkFCLink] Offboard mode started");
        }

        DRONE_LOG_DEBUG("[MavlinkFCLink] velocity cmd N={:.2f} E={:.2f} U={:.2f} yaw={:.1f}°", vx,
                        vy, vz, cmd.yaw_deg);
        return true;
    }

    bool send_arm(bool arm) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!action_ || !system_ || !system_->is_connected()) return false;

        auto result = arm ? action_->arm() : action_->disarm();
        if (result != mavsdk::Action::Result::Success) {
            DRONE_LOG_WARN("[MavlinkFCLink] {} failed: {}", arm ? "Arm" : "Disarm",
                           action_result_str(result));
            return false;
        }
        DRONE_LOG_INFO("[MavlinkFCLink] {} successful", arm ? "Armed" : "Disarmed");
        return true;
    }

    bool send_takeoff(float altitude_m) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!action_ || !system_ || !system_->is_connected()) return false;

        auto set_result = action_->set_takeoff_altitude(altitude_m);
        if (set_result != mavsdk::Action::Result::Success) {
            DRONE_LOG_WARN("[MavlinkFCLink] set_takeoff_altitude({:.1f}m) failed: {}", altitude_m,
                           action_result_str(set_result));
            return false;
        }

        auto result = action_->takeoff();
        if (result != mavsdk::Action::Result::Success) {
            DRONE_LOG_WARN("[MavlinkFCLink] Takeoff failed: {}", action_result_str(result));
            return false;
        }
        // Stop offboard if it was active — takeoff uses Auto mode
        if (offboard_active_.load(std::memory_order_acquire)) {
            offboard_->stop();
            offboard_active_.store(false, std::memory_order_release);
        }
        DRONE_LOG_INFO("[MavlinkFCLink] Takeoff to {:.1f}m initiated", altitude_m);
        return true;
    }

    /// Map IFCLink mode codes to MAVSDK commands:
    ///   0 = Stabilized  → action->hold()  (closest safe mapping)
    ///   1 = Guided       → offboard start  (PX4 Offboard)
    ///   2 = Auto/Mission → action->hold()  (mission requires a plan upload)
    ///   3 = RTL          → action->return_to_launch()
    bool send_mode(uint8_t mode) override {
        std::lock_guard<std::mutex> lock(conn_mtx_);
        if (!action_ || !system_ || !system_->is_connected()) return false;

        mavsdk::Action::Result result;
        switch (mode) {
            case 0:  // STAB → Hold (safe stabilised hover)
                result = action_->hold();
                if (offboard_active_.load(std::memory_order_acquire)) {
                    offboard_->stop();
                    offboard_active_.store(false, std::memory_order_release);
                }
                break;
            case 1:  // GUIDED → Offboard
                if (!offboard_ || offboard_active_.load(std::memory_order_acquire)) {
                    return true;  // already in offboard
                }
                // Set a zero-velocity setpoint so offboard start doesn't reject
                offboard_->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
                {
                    auto ob_result = offboard_->start();
                    if (ob_result != mavsdk::Offboard::Result::Success) {
                        DRONE_LOG_WARN("[MavlinkFCLink] Offboard start failed: {}",
                                       offboard_result_str(ob_result));
                        return false;
                    }
                }
                offboard_active_.store(true, std::memory_order_release);
                DRONE_LOG_INFO("[MavlinkFCLink] Mode → Offboard (GUIDED)");
                return true;
            case 2:  // AUTO → Land
                if (offboard_active_.load(std::memory_order_acquire)) {
                    offboard_->stop();
                    offboard_active_.store(false, std::memory_order_release);
                }
                result = action_->land();
                break;
            case 3:  // RTL
                if (offboard_active_.load(std::memory_order_acquire)) {
                    offboard_->stop();
                    offboard_active_.store(false, std::memory_order_release);
                }
                result = action_->return_to_launch();
                break;
            default: DRONE_LOG_WARN("[MavlinkFCLink] Unknown mode {}", mode); return false;
        }

        if (result != mavsdk::Action::Result::Success) {
            DRONE_LOG_WARN("[MavlinkFCLink] Mode change ({}) failed: {}", mode,
                           action_result_str(result));
            return false;
        }
        DRONE_LOG_INFO("[MavlinkFCLink] Mode changed to {}", mode);
        return true;
    }

    FCState receive_state() override {
        std::lock_guard<std::mutex> lock(state_mtx_);
        return cached_state_;
    }

    std::string name() const override { return "MavlinkFCLink"; }

private:
    // ── Subscription setup ─────────────────────────────────
    void setup_subscriptions() {
        // Position
        telemetry_->subscribe_position([this](mavsdk::Telemetry::Position pos) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.altitude_rel = pos.relative_altitude_m;
        });

        // Battery
        telemetry_->subscribe_battery([this](mavsdk::Telemetry::Battery batt) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.battery_voltage = batt.voltage_v;
            cached_state_.battery_current = batt.current_battery_a;
            cached_state_.battery_percent = batt.remaining_percent;
        });

        // Flight mode
        telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode fm) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.flight_mode = map_flight_mode(fm);
        });

        // Armed state
        telemetry_->subscribe_armed([this](bool armed) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.armed = armed;
        });

        // GPS info
        telemetry_->subscribe_gps_info([this](mavsdk::Telemetry::GpsInfo gps) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.satellites = static_cast<uint8_t>(std::clamp(gps.num_satellites, 0, 255));
        });

        // Velocity (for ground speed computation)
        telemetry_->subscribe_velocity_ned([this](mavsdk::Telemetry::VelocityNed vel) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            cached_state_.ground_speed =
                std::sqrt(vel.north_m_s * vel.north_m_s + vel.east_m_s * vel.east_m_s);
            // Update timestamp on every velocity update
            cached_state_.timestamp_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
        });
    }

    // ── Flight mode mapping ────────────────────────────────
    /// Map MAVSDK FlightMode to our IFCLink mode codes:
    ///   0 = STAB (Stabilized, Manual, Altctl, Posctl, Acro, Rattitude)
    ///   1 = GUIDED (Offboard, FollowMe)
    ///   2 = AUTO (Mission, Takeoff, Land, Hold)
    ///   3 = RTL (ReturnToLaunch)
    static uint8_t map_flight_mode(mavsdk::Telemetry::FlightMode fm) {
        switch (fm) {
            case mavsdk::Telemetry::FlightMode::Stabilized:
            case mavsdk::Telemetry::FlightMode::Manual:
            case mavsdk::Telemetry::FlightMode::Altctl:
            case mavsdk::Telemetry::FlightMode::Posctl:
            case mavsdk::Telemetry::FlightMode::Acro:
            case mavsdk::Telemetry::FlightMode::Rattitude: return 0;  // STAB
            case mavsdk::Telemetry::FlightMode::Offboard:
            case mavsdk::Telemetry::FlightMode::FollowMe: return 1;  // GUIDED
            case mavsdk::Telemetry::FlightMode::Mission:
            case mavsdk::Telemetry::FlightMode::Takeoff:
            case mavsdk::Telemetry::FlightMode::Land:
            case mavsdk::Telemetry::FlightMode::Hold: return 2;            // AUTO
            case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return 3;  // RTL
            default: return 0;                                             // Unknown → STAB
        }
    }

    // ── Result stringifiers (avoid pulling in <iostream>) ──
    static const char* connection_result_str(mavsdk::ConnectionResult r) {
        switch (r) {
            case mavsdk::ConnectionResult::Success: return "Success";
            case mavsdk::ConnectionResult::Timeout: return "Timeout";
            case mavsdk::ConnectionResult::SocketError: return "SocketError";
            case mavsdk::ConnectionResult::BindError: return "BindError";
            case mavsdk::ConnectionResult::ConnectionError: return "ConnectionError";
            case mavsdk::ConnectionResult::ConnectionUrlInvalid: return "ConnectionUrlInvalid";
            default: return "Unknown";
        }
    }

    static const char* action_result_str(mavsdk::Action::Result r) {
        switch (r) {
            case mavsdk::Action::Result::Success: return "Success";
            case mavsdk::Action::Result::NoSystem: return "NoSystem";
            case mavsdk::Action::Result::ConnectionError: return "ConnectionError";
            case mavsdk::Action::Result::Busy: return "Busy";
            case mavsdk::Action::Result::CommandDenied: return "CommandDenied";
            case mavsdk::Action::Result::Timeout: return "Timeout";
            case mavsdk::Action::Result::Unsupported: return "Unsupported";
            default: return "Unknown";
        }
    }

    static const char* offboard_result_str(mavsdk::Offboard::Result r) {
        switch (r) {
            case mavsdk::Offboard::Result::Success: return "Success";
            case mavsdk::Offboard::Result::NoSystem: return "NoSystem";
            case mavsdk::Offboard::Result::ConnectionError: return "ConnectionError";
            case mavsdk::Offboard::Result::Busy: return "Busy";
            case mavsdk::Offboard::Result::CommandDenied: return "CommandDenied";
            case mavsdk::Offboard::Result::Timeout: return "Timeout";
            default: return "Unknown";
        }
    }

    // ── Members ────────────────────────────────────────────
    mutable std::mutex conn_mtx_;        ///< Guards connection lifecycle
    std::mutex         state_mtx_;       ///< Guards cached_state_
    FCState            cached_state_{};  ///< Latest telemetry snapshot

    std::unique_ptr<mavsdk::Mavsdk>    mavsdk_;
    std::shared_ptr<mavsdk::System>    system_;
    std::unique_ptr<mavsdk::Action>    action_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Offboard>  offboard_;
    std::atomic<bool>                  offboard_active_{false};
};

}  // namespace drone::hal

#endif  // HAVE_MAVSDK
