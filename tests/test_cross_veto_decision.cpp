// tests/test_cross_veto_decision.cpp
//
// Pure-function tests for decide_promotion() (issue #698 Fix #1, Phase 2).
// Exercises every branch of the binding three-row promotion rule from
// ADR-013 §2 item 4 plus the two distinct escape hatches from research
// note §7.

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
                uint64_t residency_ns = 0, uint64_t cell_age_ns = 0) {
    RadarQuery q;
    q.range_to_drone_m = range_m;
    q.in_fov           = in_fov;
    q.radar_present    = radar_present;
    q.radar_stale      = radar_stale;
    q.residency_ns     = residency_ns;
    q.cell_age_ns      = cell_age_ns;
    q.gate_radius_m    = 1.0f;  // not consulted by decide_promotion()
    return q;
}

}  // namespace

// ── Row 1 — short range: stereo alone ────────────────────────

TEST(CrossVetoDecision, ShortRange_PromoteRegardlessOfRadar) {
    const auto p = default_policy();
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
    EXPECT_EQ(decide_promotion(q_at(15.0f, true, true, true), p),
              PromotionDecision::DeferToDynamic);
}

// ── Row 3 — long range, outside FOV ──────────────────────────

TEST(CrossVetoDecision, LongRangeOutsideFov_NoResidency_NoAge_Defer) {
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_BelowResidencyThreshold_Defer) {
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(1)), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_AtResidencyThreshold_PromoteFovSilence) {
    const auto p = default_policy();
    // Exactly 2 s residency hits the threshold — positive-silence promotion.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(2)), p),
              PromotionDecision::PromoteFovSilence);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_OneNsBelowResidency_Defer) {
    const auto p = default_policy();
    // Boundary: residency exactly at threshold-1 must NOT trip the >= check.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true,
                                    p.fov_residency_promote_ns - 1, 0), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_AgeCapDistinctFromResidency) {
    auto p              = default_policy();
    p.dynamic_age_cap_ns = kS(30);  // explicit
    // Cell with no residency but past the age cap → age-cap eviction.
    // This is the case our previous bug-fix targets: cells permanently
    // outside the FOV cone will never accrue residency, only age.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true,
                                    /*residency_ns=*/0,
                                    /*cell_age_ns=*/kS(31)), p),
              PromotionDecision::PromoteAgeCapEviction);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_AtAgeCapBoundary_Promote) {
    auto p             = default_policy();
    p.dynamic_age_cap_ns = kS(30);
    // Exactly at the age cap (>=) — must promote.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, 0, kS(30)), p),
              PromotionDecision::PromoteAgeCapEviction);
    // One ns below — must defer.
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, 0, kS(30) - 1), p),
              PromotionDecision::DeferToDynamic);
}

TEST(CrossVetoDecision, LongRangeOutsideFov_BothEscapesArmed_PrefersFovSilence) {
    // When both residency >= promote AND age >= cap, the lower-severity
    // FOV-silence reason wins — radar saw the area, that's stronger
    // evidence than mere age.  This is intentional: callers that key
    // off PromoteAgeCapEviction (which logs every event because radar
    // never observed) must NOT see false alerts when radar did observe.
    const auto p = default_policy();
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(5), kS(31)), p),
              PromotionDecision::PromoteFovSilence);
}

// ── Boundary at short_range_m ────────────────────────────────

TEST(CrossVetoDecision, AtExactShortRangeBoundary_LongRangeRules) {
    const auto p = default_policy();
    // range == 10.0 m: NOT < 10.0, falls to long-range rules.
    EXPECT_EQ(decide_promotion(q_at(10.0f, false, false, true), p),
              PromotionDecision::DeferToDynamic);
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
    CrossVetoPolicy p          = default_policy();
    p.fov_residency_promote_ns = kS(5);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(4)), p),
              PromotionDecision::DeferToDynamic);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, kS(5)), p),
              PromotionDecision::PromoteFovSilence);
}

TEST(CrossVetoDecision, CustomAgeCapThreshold_RuleScales) {
    CrossVetoPolicy p    = default_policy();
    p.dynamic_age_cap_ns = kS(60);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, 0, kS(59)), p),
              PromotionDecision::DeferToDynamic);
    EXPECT_EQ(decide_promotion(q_at(15.0f, false, false, true, 0, kS(60)), p),
              PromotionDecision::PromoteAgeCapEviction);
}
