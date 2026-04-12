// tests/test_geofence.cpp
// Unit tests for the Geofence engine (Phase 3).
#include "planner/geofence.h"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::planner;

// ── Helper: 100×100m square polygon centred at origin ────────
static std::vector<GeoVertex> square_100() {
    return {{-50, -50}, {50, -50}, {50, 50}, {-50, 50}};
}

// ═════════════════════════════════════════════════════════════
// Configuration tests
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, DefaultDisabled) {
    Geofence fence;
    EXPECT_FALSE(fence.is_enabled());
    auto r = fence.check(0, 0, 50);
    EXPECT_FALSE(r.violated);
}

TEST(GeofenceTest, EnableRequiresPolygon) {
    Geofence fence;
    fence.enable(true);  // no polygon set
    EXPECT_FALSE(fence.is_enabled());
}

TEST(GeofenceTest, TooFewVerticesDisables) {
    Geofence fence;
    fence.set_polygon({{0, 0}, {1, 1}});  // only 2 vertices
    EXPECT_FALSE(fence.is_enabled());
    fence.enable(true);
    EXPECT_FALSE(fence.is_enabled());
}

TEST(GeofenceTest, EnableWithValidPolygon) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.enable(true);
    EXPECT_TRUE(fence.is_enabled());
}

TEST(GeofenceTest, DisableAfterEnable) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.enable(true);
    EXPECT_TRUE(fence.is_enabled());
    fence.enable(false);
    EXPECT_FALSE(fence.is_enabled());
}

TEST(GeofenceTest, AltitudeLimitsStored) {
    Geofence fence;
    fence.set_altitude_limits(5.0f, 100.0f);
    EXPECT_FLOAT_EQ(fence.alt_floor(), 5.0f);
    EXPECT_FLOAT_EQ(fence.alt_ceiling(), 100.0f);
}

TEST(GeofenceTest, WarningMarginClamped) {
    Geofence fence;
    fence.set_warning_margin(-5.0f);
    EXPECT_FLOAT_EQ(fence.warning_margin(), 0.0f);
    fence.set_warning_margin(10.0f);
    EXPECT_FLOAT_EQ(fence.warning_margin(), 10.0f);
}

// ═════════════════════════════════════════════════════════════
// Polygon containment tests
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, InsideSquareNotViolated) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, 50);
    EXPECT_FALSE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::NONE);
}

TEST(GeofenceTest, OutsideSquareViolated) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(60, 0, 50);  // 10m outside east edge
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::OUTSIDE_POLYGON);
    EXPECT_GT(r.margin_m, 0.0f);
    EXPECT_FALSE(r.message.empty());
}

TEST(GeofenceTest, CornerIsInside) {
    Geofence fence;
    // Slightly inside the corner
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(49, 49, 50);
    EXPECT_FALSE(r.violated);
}

TEST(GeofenceTest, FarOutside) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(1000, 1000, 50);
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::OUTSIDE_POLYGON);
}

// ═════════════════════════════════════════════════════════════
// Altitude limit tests
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, AboveCeiling) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, 130);
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::ABOVE_CEILING);
    EXPECT_NEAR(r.margin_m, 10.0f, 0.1f);
}

TEST(GeofenceTest, BelowFloor) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(5.0f, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, 3);
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::BELOW_FLOOR);
    EXPECT_NEAR(r.margin_m, 2.0f, 0.1f);
}

TEST(GeofenceTest, AtExactCeiling) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, 120);
    EXPECT_FALSE(r.violated);  // at ceiling = OK, above = violation
}

TEST(GeofenceTest, AtExactFloor) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(5, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, 5);
    EXPECT_FALSE(r.violated);  // at floor = OK, below = violation
}

TEST(GeofenceTest, SlightlyBelowFloorWithinTolerance) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    // Simulates Gazebo ground-truth reporting -0.3m before takeoff.
    // Should NOT violate — within 0.5m tolerance of the floor.
    auto r = fence.check(0, 0, -0.3f);
    EXPECT_FALSE(r.violated);
}

TEST(GeofenceTest, SlightlyAboveCeilingWithinTolerance) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    // Within 0.5m tolerance above ceiling — should NOT violate.
    auto r = fence.check(0, 0, 120.4f);
    EXPECT_FALSE(r.violated);
}

TEST(GeofenceTest, WellBelowFloorStillViolates) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    // 1m below floor — beyond tolerance, should violate.
    auto r = fence.check(0, 0, -1.0f);
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::BELOW_FLOOR);
}

// ═════════════════════════════════════════════════════════════
// Margin computation tests
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, MarginInsideNegative) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.set_warning_margin(10.0f);
    fence.enable(true);

    auto r = fence.check(0, 0, 50);  // center of square
    EXPECT_FALSE(r.violated);
    EXPECT_LT(r.margin_m, 0.0f);  // negative = safely inside
}

TEST(GeofenceTest, MarginNearEdge) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.set_warning_margin(10.0f);
    fence.enable(true);

    // 3m from east edge (x=47, edge at x=50)
    auto r = fence.check(47, 0, 50);
    EXPECT_FALSE(r.violated);
    EXPECT_NEAR(r.margin_m, -3.0f, 0.5f);  // ~3m inside, negative
}

// ═════════════════════════════════════════════════════════════
// Concave polygon test
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, ConcavePolygonL) {
    // L-shaped polygon
    Geofence fence;
    fence.set_polygon({{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}});
    fence.set_altitude_limits(0, 100);
    fence.enable(true);

    // Inside the L
    EXPECT_FALSE(fence.check(5, 5, 50).violated);
    EXPECT_FALSE(fence.check(15, 5, 50).violated);
    EXPECT_FALSE(fence.check(5, 15, 50).violated);

    // Outside the L (in the notch)
    EXPECT_TRUE(fence.check(15, 15, 50).violated);
}

// ═════════════════════════════════════════════════════════════
// NaN / Inf input tests (error handling)
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, NaNInputTreatedAsViolation) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(std::numeric_limits<float>::quiet_NaN(), 0, 50);
    EXPECT_TRUE(r.violated);
    EXPECT_EQ(r.reason, GeofenceViolation::OUTSIDE_POLYGON);
}

TEST(GeofenceTest, InfInputTreatedAsViolation) {
    Geofence fence;
    fence.set_polygon(square_100());
    fence.set_altitude_limits(0, 120);
    fence.enable(true);

    auto r = fence.check(0, 0, std::numeric_limits<float>::infinity());
    EXPECT_TRUE(r.violated);
}

// ═════════════════════════════════════════════════════════════
// violation_name() utility
// ═════════════════════════════════════════════════════════════

TEST(GeofenceTest, ViolationNameCovers) {
    EXPECT_STREQ(violation_name(GeofenceViolation::NONE), "NONE");
    EXPECT_STREQ(violation_name(GeofenceViolation::OUTSIDE_POLYGON), "OUTSIDE_POLYGON");
    EXPECT_STREQ(violation_name(GeofenceViolation::ABOVE_CEILING), "ABOVE_CEILING");
    EXPECT_STREQ(violation_name(GeofenceViolation::BELOW_FLOOR), "BELOW_FLOOR");
    EXPECT_STREQ(violation_name(static_cast<GeofenceViolation>(99)), "UNKNOWN");
}
