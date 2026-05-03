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

/// The three possible decisions for a candidate voxel cell at the promotion
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
    /// Force-promote with a `single_modality` flag — the cell has either
    /// (a) accrued enough cumulative FOV-time with no radar return to count
    /// as positive radar silence (research note §7 yaw-aware residency), or
    /// (b) hit the dynamic_age_cap so we promote rather than carry it
    /// indefinitely in the dynamic layer.  Either way the caller logs a
    /// disagreement event for telemetry.
    PromoteSingleModality,
};

/// Pure decision function.  Returns the promotion outcome given a single
/// query result + the policy thresholds.  No I/O, no logging, no allocation.
///
/// Decision table — the three architecturally-binding rows of ADR-013 §2
/// item 4 plus the two operational escape hatches:
///
///   range < policy.short_range_m                             → Promote
///   range >= short_range_m && in_fov && radar_present        → Promote
///   range >= short_range_m && in_fov && radar_stale          → DeferToDynamic
///   range >= short_range_m && in_fov && !radar_present       → DeferToDynamic
///   range >= short_range_m && !in_fov && residency >= promote → PromoteSingleModality
///   range >= short_range_m && !in_fov && residency >= cap    → PromoteSingleModality
///   range >= short_range_m && !in_fov                        → DeferToDynamic
///
/// Note that the "fov_residency_promote_ns" branch fires even when the cell
/// is currently OUTSIDE the FOV — the residency counter records cumulative
/// time spent INSIDE FOV with no return, accrued by RadarFovGate::tick_residency().
/// A cell that was in FOV long enough to clear the threshold is promoted on
/// the next tick regardless of where it sits now.
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
    // Two escape hatches (research note §7):
    //   (a) "positive silence" — cell accrued enough FOV-time with no return,
    //       so we trust the radar's silence as a confirmation.
    //   (b) age-cap eviction — cell has been dynamic too long; promote with
    //       single-modality flag rather than carry it indefinitely.
    if (q.residency_ns >= policy.fov_residency_promote_ns ||
        q.residency_ns >= policy.dynamic_age_cap_ns) {
        return PromotionDecision::PromoteSingleModality;
    }
    return PromotionDecision::DeferToDynamic;
}

}  // namespace drone::planner
