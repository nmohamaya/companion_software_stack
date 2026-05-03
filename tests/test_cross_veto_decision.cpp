// tests/test_cross_veto_decision.cpp
//
// Pure-function tests for decide_promotion() (issue #698 Fix #1, Phase 2).
// Exercises every branch of the binding three-row promotion rule from
// ADR-013 §2 item 4 plus the two escape hatches from research note §7.

#include "planner/cross_veto_decision.h"
#include "planner/radar_fov_gate.h"

#include <gtest/gtest.h>

using drone::planner::CrossVetoPolicy;
using drone::planner::PromotionDecision;
using drone::planner::RadarQuery;
using drone::planner::decide_promotion;

namespace {

constexpr uint64_t kS(uint64_t s) { return s * 1'000'000'000ULL; }

CrossVetoPolicy default_policy() {
    return CrossVetoPolicy{};  // 10 m, 100 ms, 2 s residency, 30 s age cap
}

// Construct a RadarQuery with explicit test values.  Helper exists because
// we exercise many combinations and want to keep the test prose tight.
RadarQuery q_at(float range_m, bool in_fov, bool radar_present, bool radar_stale,
                uint64_t residency_ns = 0) {
    RadarQuery q;
    q.range_to_drone_m = range_m;
    q.in_fov           = in_fov;
    q.radar_present    = radar_present;
    q.radar_stale      = radar_stale;
    q.residency_ns     = residency_ns;
    q.gate_radius_m    = 1.0f;  // not consulted by decide_promotion()
    return q;
}

}  // namespace

// ── Row 1 — short range: stereo alone ────────────────────────

TEST(CrossVetoDecision, ShortRange_PromoteRegardlessOfRadar) {
    const auto p = default_policy();
    // Even with no radar, no FOV, stale — stereo is authoritative <10 m.
    EXPECT_EQ(decide_promotion(q_at(2.0f, false, false, true), p),
              PromotionDecision::Promote);
    EXPECT_EQ(decide_promotion(q_at(5.0f, true, true, false), p),
              PromotionDecision::Promote);
    EXPECT_EQ(decide_promotion(q_at(9.99f, false, false, false), p),
              PromotionDecision::Promote);
}

// ── Row 2 — long range, in FOV ───────────────────────────────

TEST(CrossVetoDecision, LongRangeInFov_BothAgree_Promote) {
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, true, true, false), p),
              PromotionDecision::Promote);
}

TEST(CrossVetoDecision, LongRangeInFov_RadarAbsent_Defer) {
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, true, false, false), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeInFov_RadarStale_Defer) {
    const auto p = default_policy();
    // Stale radar must veto even when a co-located return technically
    // exists — caller cannot trust an out-of-window measurement.
    EXPECT_EQ(decide_promotion(q_at(15.0f, true, true, true), p),
              PromotionDecision::DeferToDynamic);
}

// ── Row 3 — long range, outside FOV ──────────────────────────

TEST(CrossVetoDecision, LongRangeOutsideFov_NoResidency_Defer) {
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_BelowResidencyThreshold_Defer) {
    const auto p = default_policy();
    // 1 s of residency is below the 2 s promote threshold — still defer.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(1)), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_AtResidencyThreshold_PromoteSingleModality) {
    const auto p = default_policy();
    // 2 s residency exactly hits the promote threshold — positive silence.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(2)), p),
              PromotionDecision::PromoteSingleModality);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_PastAgeCap_PromoteSingleModality) {
    const auto p = default_policy();
    // 31 s residency past the 30 s age cap — force-promote.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(31)), p),
              PromotionDecision::PromoteSingleModality);
}

// ── Boundary at short_range_m ────────────────────────────────

TEST(CrossVetoDecision, AtExactShortRangeBoundary_LongRangeRules) {
    const auto p = default_policy();
    // range == 10.0 m: NOT < 10.0, falls to long-range rules.
    // Without radar / FOV / residency → defer.
    EXPECT_EQ(decide_promotion(q_at(10.0f, false, false, true), p),
              PromotionDecision::DeferToDynamic);
    // With radar present + in FOV → promote.
    EXPECT_EQ(decide_promotion(q_at(10.0f, true, true, false), p),
              PromotionDecision::Promote);
}

// ── Custom policy ────────────────────────────────────────────

TEST(CrossVetoDecision, CustomShortRange_RuleScales) {
    CrossVetoPolicy p = default_policy();
    p.short_range_m   = 5.0f;
    EXPECT_EQ(decide_promotion(q_at(4.99f, false, false, true), p),
              PromotionDecision::Promote);
    EXPECT_EQ(decide_promotion(q_at(5.0f, false, false, true), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, CustomResidencyThreshold_RuleScales) {
    CrossVetoPolicy p              = default_policy();
    p.fov_residency_promote_ns     = kS(5);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(4)), p),
              PromotionDecision::DeferToDynamic);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(5)), p),
              PromotionDecision::PromoteSingleModality);
}
