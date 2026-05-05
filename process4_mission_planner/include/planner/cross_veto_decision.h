// process4_mission_planner/include/planner/cross_veto_decision.h
//
// Pure decision function for the OccupancyGrid3D static-cell promotion path.
// Encodes the binding three-row promotion rule from ADR-013 §2 item 4 plus
// the dynamic-cell hygiene escape hatches (research note §7) the cross-veto
// requires to avoid dynamic-layer bloat.
//
// Lives in its own header so it's trivially testable without OccupancyGrid3D
// scaffolding (no grid, no Eigen pose math, no IPC types — just a struct in,
// an enum out).
//
// Issue: #698 (Fix #1 — long-range radar veto on PATH A grid promotion),
// Phase 2 of tasks/plan-scenario-33-pass.md.

#pragma once

#include "planner/radar_fov_gate.h"  // RadarQuery

#include <cstdint>

namespace drone::planner {

/// The four possible decisions for a candidate voxel cell at the promotion
/// gate.  Callers map these to grid layer transitions.
enum class PromotionDecision {
    /// Promote to permanent static.  Either:
    ///   - cell is < short_range_m (stereo-alone authority, ADR-013 row 1), or
    ///   - cell is in radar FOV with a recent radar return (rows 2 — both agree).
    Promote,
    /// Do NOT promote — keep the cell in the dynamic (TTL-decay) layer.
    /// Reasons: in FOV but radar disagrees (real veto), in FOV but radar stale
    /// (treat as "didn't see → veto"), or outside FOV with insufficient
    /// residency / age (row 3).
    DeferToDynamic,
    /// Promote with `single_modality` flag — cell accrued enough cumulative
    /// in-FOV time with no radar return to count as positive radar silence
    /// (research note §7 yaw-aware residency).  Trustworthy: radar *did*
    /// observe the area and reported nothing.  Lower-severity telemetry.
    PromoteFovSilence,
    /// Force-promote with `single_modality` flag — cell exceeded the
    /// dynamic_age_cap.  Cells structurally outside the radar cone (e.g.
    /// far above the elevation limit) accrue zero residency and would
    /// otherwise carry forever in the dynamic layer.  We promote them on a
    /// *time* basis to bound the dynamic layer, even though radar never
    /// saw the area.  Higher-severity telemetry — caller should log + count
    /// these separately so a sudden mass-eviction is visible.
    PromoteAgeCapEviction,
};

/// Pure decision function.  Returns the promotion outcome given a single
/// query result + the policy thresholds.  No I/O, no logging, no allocation.
///
/// Decision table — the three architecturally-binding rows of ADR-013 §2
/// item 4 plus the two operational escape hatches:
///
///   range < policy.short_range_m                              → Promote
///   range >= short_range_m && in_fov && radar_present         → Promote
///   range >= short_range_m && in_fov && radar_stale           → DeferToDynamic
///   range >= short_range_m && in_fov && !radar_present        → DeferToDynamic
///   range >= short_range_m && !in_fov && residency >= promote → PromoteFovSilence
///   range >= short_range_m && !in_fov && cell_age >= cap      → PromoteAgeCapEviction
///   range >= short_range_m && !in_fov                         → DeferToDynamic
///
/// Two distinct escape hatches with different semantics:
///   - `residency_ns` records cumulative time the cell spent INSIDE the FOV
///     with no radar return.  Crossing `fov_residency_promote_ns` means the
///     radar *did* observe the area and reported nothing — positive silence.
///   - `cell_age_ns` records wall time since the cell was first observed in
///     the dynamic layer.  Crossing `dynamic_age_cap_ns` rescues cells that
///     are structurally outside any radar coverage (e.g. cells above the
///     elevation cone) so they don't accumulate indefinitely.  This is a
///     pure age-based eviction independent of FOV residency.
[[nodiscard]] constexpr PromotionDecision decide_promotion(
    const RadarQuery& q, const CrossVetoPolicy& policy) noexcept {
    // Row 1 — stereo-alone authority at short range.
    if (q.range_to_drone_m < policy.short_range_m) {
        return PromotionDecision::Promote;
    }
    // Row 2 — both modalities required at long range, in FOV.
    if (q.in_fov) {
        if (q.radar_stale) return PromotionDecision::DeferToDynamic;
        return q.radar_present ? PromotionDecision::Promote
                               : PromotionDecision::DeferToDynamic;
    }
    // Row 3 — long range, outside FOV.  Default-conservative: stay dynamic.
    // Two distinct escape hatches:
    //   (a) "positive silence" via FOV residency
    //   (b) age-cap eviction via cell-lifetime
    // Order matters: prefer the lower-severity decision when both fire.
    if (q.residency_ns >= policy.fov_residency_promote_ns) {
        return PromotionDecision::PromoteFovSilence;
    }
    if (q.cell_age_ns >= policy.dynamic_age_cap_ns) {
        return PromotionDecision::PromoteAgeCapEviction;
    }
    return PromotionDecision::DeferToDynamic;
}

}  // namespace drone::planner
