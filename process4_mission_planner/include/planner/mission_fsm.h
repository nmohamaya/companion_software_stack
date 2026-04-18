// process4_mission_planner/include/planner/mission_fsm.h
// Finite State Machine for mission control.
#pragma once
#include "ipc/ipc_types.h"
#include "util/ilogger.h"

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace drone::planner {

using MissionState = drone::ipc::MissionState;

inline const char* state_name(MissionState s) {
    switch (s) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::PREFLIGHT: return "PREFLIGHT";
        case MissionState::TAKEOFF: return "TAKEOFF";
        case MissionState::NAVIGATE: return "NAVIGATE";
        case MissionState::LOITER: return "LOITER";
        case MissionState::RTL: return "RTL";
        case MissionState::LAND: return "LAND";
        case MissionState::EMERGENCY: return "EMERGENCY";
        case MissionState::SURVEY: return "SURVEY";
        case MissionState::COLLISION_RECOVERY: return "COLLISION_RECOVERY";
        case MissionState::NAVIGATE_UNSTUCK: return "NAVIGATE_UNSTUCK";
        default: return "UNKNOWN";
    }
}

// ── StuckDetector (Issue #503) ──────────────────────────────
// Sliding-window detector that flags the drone as stuck when:
//   - Position has not moved more than min_movement_m over window_s
//   - AND the avoider was active (non-zero correction) during the window
//   - AND the window is full (i.e. we have at least window_s of samples)
// Only NAVIGATE should feed samples in — SURVEY/LAND/RTL/etc. legitimately
// hover and must not be flagged.  Gate at the caller.
class StuckDetector {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    struct Config {
        bool  enabled            = true;
        float window_s           = 3.0f;
        float min_movement_m     = 0.5f;
        float backoff_speed_mps  = 1.0f;
        float backoff_duration_s = 2.0f;
        // Cap on STUCK→UNSTUCK→NAVIGATE oscillations within a mission.
        // When exceeded, caller escalates to LOITER with fault_triggered=true
        // so the operator sees persistent stall in health telemetry instead
        // of silent oscillation.  0 disables the cap.
        uint32_t max_stuck_count = 3;
    };

    StuckDetector() = default;
    explicit StuckDetector(const Config& cfg) : cfg_(cfg) {}

    [[nodiscard]] const Config& config() const { return cfg_; }

    /// Record a pose sample.  Samples older than window_s are dropped.
    /// Caller pushes only in states where hover is abnormal (NAVIGATE).
    void push_sample(TimePoint t, float x, float y, float z) {
        samples_.emplace_back(t, std::array<float, 3>{x, y, z});

        // Drop samples older than window_s.
        const auto window = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<float>(cfg_.window_s));
        while (!samples_.empty() && (t - samples_.front().first) > window) {
            samples_.pop_front();
        }
    }

    /// Clear history — call on state change or significant pose jump.
    void reset() { samples_.clear(); }

    /// Evaluate stuck condition using the samples collected so far.
    ///
    /// Caller is responsible for only pushing samples in states where hover
    /// is abnormal (i.e. NAVIGATE).  This function therefore does NOT gate
    /// on "avoider was active recently" — an earlier iteration of this check
    /// failed precisely when the drone was most stuck, because collision
    /// with geometry caused LiDAR-derived DetectedObjectList.num_objects to
    /// go to zero (the obstacle intersects the drone body → no returns) and
    /// the avoider became inactive.  The drone was stuck *because* perception
    /// lost the obstacle, and the detector silently decided "not stuck".
    /// Moving the liveness check out of the detector and relying on the
    /// caller's state gating is the correct fix.
    [[nodiscard]] bool is_stuck(TimePoint now) const {
        if (!cfg_.enabled) return false;
        if (samples_.size() < 2) return false;
        // Need a full window of history.
        const float span_s = std::chrono::duration<float>(now - samples_.front().first).count();
        if (span_s < cfg_.window_s) return false;
        // Compare current pose vs window-start pose.
        const auto& first = samples_.front().second;
        const auto& last  = samples_.back().second;
        const float dx    = last[0] - first[0];
        const float dy    = last[1] - first[1];
        const float dz    = last[2] - first[2];
        const float moved = std::sqrt(dx * dx + dy * dy + dz * dz);
        return moved < cfg_.min_movement_m;
    }

    /// Whether at least one sample has been recorded (for tests).
    [[nodiscard]] bool has_samples() const { return !samples_.empty(); }

private:
    Config                                                 cfg_{};
    std::deque<std::pair<TimePoint, std::array<float, 3>>> samples_;
};

/// Waypoint for mission navigation.
struct Waypoint {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // target position in world frame
    float yaw             = 0.0f;        // target heading
    float radius          = 2.0f;        // acceptance radius (m)
    float speed           = 2.0f;        // cruise speed (m/s)
    bool  trigger_payload = false;       // trigger camera/gimbal at this waypoint
};

/// Mission planner FSM — manages the mission lifecycle.
class MissionFSM {
public:
    explicit MissionFSM(float overshoot_proximity_factor = 3.0f)
        : state_(MissionState::IDLE), overshoot_proximity_factor_(overshoot_proximity_factor) {}

    [[nodiscard]] MissionState state() const { return state_; }

    /// Process an event and transition state.
    void on_arm() { transition(MissionState::PREFLIGHT); }
    void on_takeoff() { transition(MissionState::TAKEOFF); }
    void on_survey() { transition(MissionState::SURVEY); }
    void on_navigate() { transition(MissionState::NAVIGATE); }
    void on_loiter() { transition(MissionState::LOITER); }
    void on_rtl() { transition(MissionState::RTL); }
    void on_land() { transition(MissionState::LAND); }
    void on_landed() {
        transition(MissionState::IDLE);
        fault_triggered_ = false;
    }
    void on_emergency() { transition(MissionState::EMERGENCY); }
    void on_collision_recovery() { transition(MissionState::COLLISION_RECOVERY); }
    void on_recovery_complete() { transition(MissionState::NAVIGATE); }
    void on_stuck() { transition(MissionState::NAVIGATE_UNSTUCK); }
    void on_unstuck() { transition(MissionState::NAVIGATE); }

    /// Check if waypoint is reached (within acceptance radius).
    /// When a planner snaps the goal to avoid occupied cells, pass the snapped
    /// world position so the acceptance check uses the actual navigation target
    /// rather than the original (possibly unreachable) waypoint position.
    /// Checks BOTH the original and snapped positions — the waypoint is reached
    /// if the drone is within acceptance radius of either.  This handles both
    /// the snap-overshoot case (#394) and the unreachable-snap case where the
    /// drone reaches the original position but D* can't pathfind to the snap.
    /// The acceptance radius is always wp.radius regardless of snap.
    [[nodiscard]] bool waypoint_reached(float px, float py, float pz, const Waypoint& wp,
                                        const std::array<float, 3>* snapped_xyz = nullptr) const {
        const float r_sq = wp.radius * wp.radius;

        // Always check original waypoint position
        const float odx = px - wp.x, ody = py - wp.y, odz = pz - wp.z;
        if ((odx * odx + ody * ody + odz * odz) < r_sq) return true;

        // Also check snapped position if provided and finite
        if (snapped_xyz && std::isfinite((*snapped_xyz)[0]) && std::isfinite((*snapped_xyz)[1]) &&
            std::isfinite((*snapped_xyz)[2])) {
            const float sdx = px - (*snapped_xyz)[0];
            const float sdy = py - (*snapped_xyz)[1];
            const float sdz = pz - (*snapped_xyz)[2];
            if ((sdx * sdx + sdy * sdy + sdz * sdz) < r_sq) return true;
        }

        return false;
    }

    /// Load a simple mission (list of waypoints).
    void load_mission(const std::vector<Waypoint>& waypoints) {
        waypoints_  = waypoints;
        current_wp_ = 0;
        DRONE_LOG_INFO("[FSM] Mission loaded: {} waypoints", waypoints_.size());
    }

    [[nodiscard]] const Waypoint* current_waypoint() const {
        if (current_wp_ < waypoints_.size()) return &waypoints_[current_wp_];
        return nullptr;
    }

    [[nodiscard]] const Waypoint* next_waypoint() const {
        if (current_wp_ + 1 < waypoints_.size()) return &waypoints_[current_wp_ + 1];
        return nullptr;
    }

    /// Check if the drone has passed the current waypoint along the approach
    /// vector toward the next waypoint (dot-product sign check).
    /// Only triggers when the drone is within `overshoot_proximity_factor` ×
    /// acceptance_radius of the waypoint — prevents premature advancement when
    /// the drone is far away and merely "ahead" in the dot-product sense.
    /// Returns false for the last waypoint — it always requires acceptance radius.
    /// When snapped_xyz is provided, checks overshoot against BOTH the original
    /// waypoint and the snapped position (same OR-logic as waypoint_reached).
    [[nodiscard]] bool waypoint_overshot(
        float px, float py, float pz,
        std::optional<std::array<float, 3>> snapped_xyz = std::nullopt) const {
        const Waypoint* wp = current_waypoint();
        if (!wp || current_wp_ == 0) return false;               // No overshoot for first WP
        if (current_wp_ + 1 >= waypoints_.size()) return false;  // Last WP needs acceptance

        // Approach vector: previous_wp → current_wp (direction drone was traveling)
        const auto& prev = waypoints_[current_wp_ - 1];
        const float ax = wp->x - prev.x, ay = wp->y - prev.y, az = wp->z - prev.z;
        const float proximity_r = wp->radius * overshoot_proximity_factor_;
        const float prox_sq     = proximity_r * proximity_r;

        // Check overshoot relative to original waypoint position
        {
            const float dx = px - wp->x, dy = py - wp->y, dz = pz - wp->z;
            const float dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq <= prox_sq && (dx * ax + dy * ay + dz * az) > 0.0f) return true;
        }

        // Check overshoot relative to snapped position if provided and finite
        if (snapped_xyz && std::isfinite((*snapped_xyz)[0]) && std::isfinite((*snapped_xyz)[1]) &&
            std::isfinite((*snapped_xyz)[2])) {
            const float sdx     = px - (*snapped_xyz)[0];
            const float sdy     = py - (*snapped_xyz)[1];
            const float sdz     = pz - (*snapped_xyz)[2];
            const float dist_sq = sdx * sdx + sdy * sdy + sdz * sdz;
            // Use the same approach vector (prev→wp) since the drone's travel
            // direction is defined by the mission plan, not the snap offset.
            if (dist_sq <= prox_sq && (sdx * ax + sdy * ay + sdz * az) > 0.0f) return true;
        }

        return false;
    }

    [[nodiscard]] bool advance_waypoint() {
        if (current_wp_ + 1 < waypoints_.size()) {
            ++current_wp_;
            DRONE_LOG_INFO("[FSM] Advanced to waypoint {}/{}", current_wp_ + 1, waypoints_.size());
            return true;
        }
        return false;  // mission complete
    }

    [[nodiscard]] size_t current_wp_index() const { return current_wp_; }
    [[nodiscard]] size_t total_waypoints() const { return waypoints_.size(); }

    /// Returns true if the FSM is in a fault-handling state and should not
    /// be overridden by normal mission logic.  Checked states:
    /// LOITER (fault-triggered only), RTL, LAND, EMERGENCY, COLLISION_RECOVERY.
    /// NAVIGATE_UNSTUCK is deliberately NOT included — it is a recovery state
    /// driven by the stuck detector, not a FaultManager-level fault.
    [[nodiscard]] bool is_in_fault_state() const {
        return fault_triggered_ &&
               (state_ == MissionState::LOITER || state_ == MissionState::RTL ||
                state_ == MissionState::LAND || state_ == MissionState::EMERGENCY ||
                state_ == MissionState::COLLISION_RECOVERY);
    }

    /// Mark that the current state was caused by a fault (not a GCS
    /// command or normal mission completion).
    void set_fault_triggered(bool v) { fault_triggered_ = v; }

    /// Whether the current state was caused by a fault.
    [[nodiscard]] bool fault_triggered() const { return fault_triggered_; }

private:
    MissionState          state_ = MissionState::IDLE;
    float                 overshoot_proximity_factor_{3.0f};
    std::vector<Waypoint> waypoints_;
    size_t                current_wp_      = 0;
    bool                  fault_triggered_ = false;

    void transition(MissionState new_state) {
        DRONE_LOG_INFO("[FSM] {} → {}", state_name(state_), state_name(new_state));
        state_ = new_state;
    }
};

}  // namespace drone::planner
