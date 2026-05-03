// process4_mission_planner/include/planner/radar_fov_gate.h
//
// RadarFovGate — late-stage cross-modal veto helper for the OccupancyGrid3D
// promotion path.  Encodes the binding three-row promotion rule from ADR-013
// (docs/adr/ADR-013-stereo-radar-redundancy-vs-fusion.md §2 item 4):
//
//   Cell distance from drone | In radar FOV? | Promotion authority
//   ─────────────────────────────────────────────────────────────────
//   < 10 m                   | any           | Stereo alone
//   ≥ 10 m                   | yes           | Stereo + radar agreement required
//   ≥ 10 m                   | no            | Stays dynamic, never promotes
//                                              (with FOV-residency escape hatch
//                                               and dynamic-age cap; see
//                                               cross_veto_decision.h)
//
// This component owns:
//   - Radar FOV geometry (azimuth/elevation/min-range/max-range)
//   - The most recent RadarDetectionList + its receive timestamp
//   - The current drone pose (world → body transform for FOV check)
//   - Per-GridCell cumulative residency tracking (how many ns the cell has
//     been *inside* the FOV with *no* radar return), needed for the
//     "positive silence promotes as single-modality" branch in
//     decide_promotion().
//
// Threading: P4 is single-threaded today (one mission planner loop).  All
// methods are called from that thread.  No locking required.  If P4 ever
// gains additional consumer threads, a single mutex around radar_dets_ +
// pose_ + last_radar_t_ns_ would suffice.
//
// Issue: #698 (Fix #1 — long-range radar veto on PATH A grid promotion).

#pragma once

#include "ipc/ipc_types.h"
#include "planner/grid_cell.h"  // GridCell, GridCellHash

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace drone::planner {

/// FOV geometry — read once from `perception.radar.*` config at construction.
struct RadarFovConfig {
    float fov_azimuth_rad{1.047f};    // ±60° default (matches Cosys radar)
    float fov_elevation_rad{0.698f};  // ±40° default
    float min_range_m{0.5f};          // FMCW near-field blind zone
    float max_range_m{100.0f};
};

/// Cross-veto policy — read from `mission_planner.occupancy_grid.cross_veto.*`.
struct CrossVetoPolicy {
    /// Cells closer than this promote on stereo alone (§2 item 4 row 1).
    float short_range_m{10.0f};

    /// Maximum acceptable age of a radar measurement when querying.  Older
    /// than this and the gate reports `radar_stale = true`; callers treat
    /// stale-radar as "did not see" → veto promotion (research note §6).
    uint64_t radar_max_staleness_ns{100ULL * 1000ULL * 1000ULL};  // 100 ms

    /// Spatial association window for radar-vs-cell match.  At range R from
    /// the drone, a radar return is considered "near" the cell if its world-
    /// frame Euclidean distance to the cell centre is below
    ///   max(min_gate_radius_m, 0.25 * R + 0.1)
    /// This grows with range to track the radar's own cross-range resolution
    /// (~14° boresight for AWR1843 class — that is 0.25·R + bias).  See
    /// research note §3 and ADR-013 §3 quantitative basis.
    float min_gate_radius_m{1.0f};

    /// "Positive silence" promotion threshold — a cell that has been inside
    /// the radar FOV for at least this long with zero radar returns is
    /// promoted as `PromoteSingleModality` (research note §7 yaw-aware FOV
    /// residency).  Without this rule, a cell structurally outside FOV that
    /// later enters FOV after a yaw maneuver would still wait indefinitely
    /// for a *positive* radar match — which never comes for genuinely-empty
    /// space.  2 s is the research-note default.
    uint64_t fov_residency_promote_ns{2ULL * 1000ULL * 1000ULL * 1000ULL};

    /// Hard age cap on dynamic cells.  Past this, a cell that has never
    /// been radar-confirmed is force-promoted as `PromoteSingleModality`
    /// with a logged warning, to prevent dynamic-layer bloat for cells
    /// structurally outside any radar coverage (e.g. cells above the
    /// elevation FOV).  30 s default per research note §7.
    uint64_t dynamic_age_cap_ns{30ULL * 1000ULL * 1000ULL * 1000ULL};
};

/// Result of a per-cell radar query.  Used by decide_promotion() and the
/// disagreement telemetry (Phase 3) to make + audit the gating decision.
struct RadarQuery {
    bool     in_fov{false};
    bool     radar_present{false};
    bool     radar_stale{false};
    float    gate_radius_m{0.0f};
    float    range_to_drone_m{0.0f};
    uint64_t residency_ns{0};  // how long this cell has accrued FOV-with-no-return
};

class RadarFovGate {
public:
    explicit RadarFovGate(RadarFovConfig fov, CrossVetoPolicy policy)
        : fov_(fov), policy_(policy) {}

    /// Update the cached drone pose.  Called from P4 main loop on each
    /// `pose_sub->receive(pose)` success.  Cheap — pure store + matrix
    /// rebuild for the world→body transform.
    void set_pose(const drone::ipc::Pose& p) noexcept {
        // Translation (world frame).
        drone_pos_w_ = Eigen::Vector3f(static_cast<float>(p.translation[0]),
                                       static_cast<float>(p.translation[1]),
                                       static_cast<float>(p.translation[2]));
        // Quaternion is (w, x, y, z) per IPC convention — see PoseDoubleBuffer
        // in process3_slam_vio_nav/src/main.cpp for the canonical mapping.
        const Eigen::Quaternionf q(static_cast<float>(p.quaternion[0]),
                                   static_cast<float>(p.quaternion[1]),
                                   static_cast<float>(p.quaternion[2]),
                                   static_cast<float>(p.quaternion[3]));
        // Body-from-world rotation = conjugate of body→world.
        R_body_from_world_ = q.conjugate().toRotationMatrix();
        pose_valid_        = true;
    }

    /// Update the cached radar detection list.  Called from P4 main loop on
    /// each `radar_sub->receive(dets)` success.  `now_ns` is the receive
    /// timestamp (steady_clock::now in nanoseconds) — we use this rather
    /// than `dets.timestamp_ns` because the latter may live in a different
    /// epoch (publisher's steady_clock vs subscriber's), and our staleness
    /// check is "did the gate see this recently?" not "when was it sampled?"
    void set_radar_detections(const drone::ipc::RadarDetectionList& dets,
                              uint64_t                              now_ns) noexcept {
        radar_dets_         = dets;
        last_radar_t_ns_    = now_ns;
        radar_initialized_  = true;
    }

    /// Query the gate for a world-frame point (typically a grid cell centre).
    /// Returns the FOV / radar-presence / staleness / residency facts the
    /// caller needs to make a promotion decision.
    [[nodiscard]] RadarQuery query(float wx, float wy, float wz, uint64_t now_ns) const noexcept {
        RadarQuery q;
        if (!pose_valid_) {
            // No pose yet — caller treats as "unknown FOV" → veto, per the
            // default-conservative policy.  in_fov = false, radar_stale = true.
            q.in_fov      = false;
            q.radar_stale = true;
            return q;
        }

        // World → body transform.
        const Eigen::Vector3f p_world(wx, wy, wz);
        const Eigen::Vector3f p_body = R_body_from_world_ * (p_world - drone_pos_w_);

        // Spherical body-frame coords.  Body convention: +X forward, +Y left,
        // +Z up (ENU body frame, matching the pose channel's world frame).
        // Azimuth: angle in body XY plane from +X, positive toward +Y.
        // Elevation: angle from XY plane toward +Z.
        const float range = p_body.norm();
        q.range_to_drone_m = range;

        if (range < 1e-6f) {
            // Cell is essentially on the drone — treat as in-FOV by convention
            // (the avoider will handle this anyway).
            q.in_fov = true;
        } else {
            const float azimuth_rad   = std::atan2(p_body.y(), p_body.x());
            const float elevation_rad = std::asin(std::clamp(p_body.z() / range, -1.0f, 1.0f));
            q.in_fov                  = (range >= fov_.min_range_m &&
                        range <= fov_.max_range_m &&
                        std::abs(azimuth_rad) <= fov_.fov_azimuth_rad &&
                        std::abs(elevation_rad) <= fov_.fov_elevation_rad);
        }

        // Staleness: if no radar message within the policy window, treat as stale.
        if (!radar_initialized_ ||
            (now_ns >= last_radar_t_ns_ &&
             (now_ns - last_radar_t_ns_) > policy_.radar_max_staleness_ns)) {
            q.radar_stale = true;
        }

        // Spatial association window — grows with range to track radar's own
        // cross-range resolution.  See CrossVetoPolicy::min_gate_radius_m doc.
        q.gate_radius_m = std::max(policy_.min_gate_radius_m, 0.25f * range + 0.1f);

        // Radar-presence check: scan cached detections for any return whose
        // world-frame position is within gate_radius_m of (wx,wy,wz).
        if (!q.radar_stale) {
            const float gate_sq = q.gate_radius_m * q.gate_radius_m;
            for (uint32_t i = 0; i < radar_dets_.num_detections; ++i) {
                const auto& d = radar_dets_.detections[i];
                // Body-frame radar position (X forward, Y left, Z up).
                // Cosys radar convention here matches RadarDetection field
                // semantics: range_m + azimuth_rad + elevation_rad (body frame).
                const float cx = d.range_m * std::cos(d.elevation_rad) * std::cos(d.azimuth_rad);
                const float cy = d.range_m * std::cos(d.elevation_rad) * std::sin(d.azimuth_rad);
                const float cz = d.range_m * std::sin(d.elevation_rad);
                const Eigen::Vector3f det_body(cx, cy, cz);
                const float           dx = det_body.x() - p_body.x();
                const float           dy = det_body.y() - p_body.y();
                const float           dz = det_body.z() - p_body.z();
                if (dx * dx + dy * dy + dz * dz <= gate_sq) {
                    q.radar_present = true;
                    break;
                }
            }
        }

        return q;
    }

    /// Walk all currently-tracked dynamic cells once per planner tick to
    /// accrue FOV-residency time.  The caller passes the iterable of dynamic
    /// cells (from OccupancyGrid3D::occupied_) and the current `now_ns`.
    /// Cells inside the FOV without a radar return have their residency
    /// counter incremented by `dt_ns_since_last_tick`; cells outside the FOV
    /// have their counter reset (we want CUMULATIVE-WHILE-IN-FOV-WITH-NO-RETURN,
    /// not lifetime).
    ///
    /// Cost: O(N) where N = |dynamic cells|.  At scenario-33 worst case
    /// (~5 K cells × 10 Hz × ~50 ns FOV math) this is well under 1 ms.
    template<typename CellIterable>
    void tick_residency(const CellIterable& dynamic_cells, float resolution_m, uint64_t now_ns) {
        // Use a separate bool sentinel rather than `last_tick_ns_ > 0`: the
        // first tick may legitimately fire at now_ns = 0 (test fixtures, replay
        // mode where the recorded steady_clock starts at zero).
        const uint64_t dt_ns =
            (tick_initialized_ && now_ns >= last_tick_ns_) ? (now_ns - last_tick_ns_) : 0;
        last_tick_ns_     = now_ns;
        tick_initialized_ = true;
        if (!pose_valid_ || dt_ns == 0) return;

        for (const auto& c : dynamic_cells) {
            // Cell centre in world coords.
            const float wx = c.x * resolution_m;
            const float wy = c.y * resolution_m;
            const float wz = c.z * resolution_m;
            const auto  q  = query(wx, wy, wz, now_ns);
            if (q.in_fov && !q.radar_present && !q.radar_stale) {
                residency_[c] += dt_ns;
            } else if (!q.in_fov) {
                // Outside FOV → reset; the rule is "T seconds CUMULATIVE
                // while inside FOV with no return", per research note §7.
                // A drone yawing in and out of viewing the cell would
                // otherwise see its counter slowly climb on disjoint glances.
                // That's acceptable as a safety property — we want the
                // counter to reflect "we have *recently* been pointing at
                // this cell", not "we glanced at it once five minutes ago".
                residency_.erase(c);
            }
            // If radar_present: leave the counter alone.  The cell will
            // promote via the regular "Promote" path on its next
            // insert_voxels() round, no special handling needed.
        }
    }

    /// Read the residency counter for a single cell.  Returns 0 if untracked.
    [[nodiscard]] uint64_t residency_ns(const GridCell& c) const noexcept {
        const auto it = residency_.find(c);
        return (it == residency_.end()) ? 0 : it->second;
    }

    /// Drop a cell's residency entry (call when the cell is promoted or
    /// evicted from the dynamic layer).
    void clear_residency(const GridCell& c) noexcept { residency_.erase(c); }

    [[nodiscard]] const RadarFovConfig&  fov_config() const noexcept { return fov_; }
    [[nodiscard]] const CrossVetoPolicy& policy() const noexcept { return policy_; }
    [[nodiscard]] bool                   has_pose() const noexcept { return pose_valid_; }
    [[nodiscard]] uint64_t               last_radar_t_ns() const noexcept {
        return last_radar_t_ns_;
    }

    /// Diagnostic — number of cells currently in the residency tracker.
    /// Useful for end-of-run reports + scenario telemetry.
    [[nodiscard]] size_t tracked_cell_count() const noexcept { return residency_.size(); }

private:
    RadarFovConfig                                              fov_;
    CrossVetoPolicy                                             policy_;
    Eigen::Vector3f                                             drone_pos_w_{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f                                             R_body_from_world_{
        Eigen::Matrix3f::Identity()};
    bool                                                        pose_valid_{false};
    drone::ipc::RadarDetectionList                              radar_dets_{};
    uint64_t                                                    last_radar_t_ns_{0};
    bool                                                        radar_initialized_{false};
    uint64_t                                                    last_tick_ns_{0};
    bool                                                        tick_initialized_{false};
    std::unordered_map<GridCell, uint64_t, GridCellHash>        residency_;
};

}  // namespace drone::planner
