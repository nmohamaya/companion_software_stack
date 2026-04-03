// tests/test_rate_clamp.cpp
// Unit tests for config-driven loop rate clamping (Issue #220).
#include "util/rate_clamp.h"

#include <limits>

#include <gtest/gtest.h>

using namespace drone::util;

// ── IMU rate clamping ──────────────────────────────────────

TEST(RateClampTest, ImuNormalValue) {
    EXPECT_EQ(clamp_imu_rate(400), 400);
}

TEST(RateClampTest, ImuAtMinBound) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMinHz), kImuRateMinHz);
}

TEST(RateClampTest, ImuAtMaxBound) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMaxHz), kImuRateMaxHz);
}

TEST(RateClampTest, ImuBelowMin) {
    EXPECT_EQ(clamp_imu_rate(10), kImuRateMinHz);
}

TEST(RateClampTest, ImuAboveMax) {
    EXPECT_EQ(clamp_imu_rate(5000), kImuRateMaxHz);
}

TEST(RateClampTest, ImuZero) {
    EXPECT_EQ(clamp_imu_rate(0), kImuRateMinHz);
}

TEST(RateClampTest, ImuNegative) {
    EXPECT_EQ(clamp_imu_rate(-100), kImuRateMinHz);
}

TEST(RateClampTest, ImuJustBelowMin) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMinHz - 1), kImuRateMinHz);
}

TEST(RateClampTest, ImuJustAboveMax) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMaxHz + 1), kImuRateMaxHz);
}

TEST(RateClampTest, ImuJustAboveMin) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMinHz + 1), kImuRateMinHz + 1);
}

TEST(RateClampTest, ImuJustBelowMax) {
    EXPECT_EQ(clamp_imu_rate(kImuRateMaxHz - 1), kImuRateMaxHz - 1);
}

// ── VIO rate clamping ──────────────────────────────────────

TEST(RateClampTest, VioNormalValue) {
    EXPECT_EQ(clamp_vio_rate(100), 100);
}

TEST(RateClampTest, VioAtMinBound) {
    EXPECT_EQ(clamp_vio_rate(kVioRateMinHz), kVioRateMinHz);
}

TEST(RateClampTest, VioAtMaxBound) {
    EXPECT_EQ(clamp_vio_rate(kVioRateMaxHz), kVioRateMaxHz);
}

TEST(RateClampTest, VioBelowMin) {
    EXPECT_EQ(clamp_vio_rate(1), kVioRateMinHz);
}

TEST(RateClampTest, VioAboveMax) {
    EXPECT_EQ(clamp_vio_rate(10000), kVioRateMaxHz);
}

TEST(RateClampTest, VioZero) {
    EXPECT_EQ(clamp_vio_rate(0), kVioRateMinHz);
}

TEST(RateClampTest, VioNegative) {
    EXPECT_EQ(clamp_vio_rate(-50), kVioRateMinHz);
}

TEST(RateClampTest, VioJustBelowMin) {
    EXPECT_EQ(clamp_vio_rate(kVioRateMinHz - 1), kVioRateMinHz);
}

TEST(RateClampTest, VioJustAboveMax) {
    EXPECT_EQ(clamp_vio_rate(kVioRateMaxHz + 1), kVioRateMaxHz);
}

// ── Generic clamp_rate ─────────────────────────────────────

TEST(RateClampTest, GenericClampInRange) {
    EXPECT_EQ(clamp_rate(50, 10, 100, "Test"), 50);
}

TEST(RateClampTest, GenericClampBelowMin) {
    EXPECT_EQ(clamp_rate(5, 10, 100, "Test"), 10);
}

TEST(RateClampTest, GenericClampAboveMax) {
    EXPECT_EQ(clamp_rate(200, 10, 100, "Test"), 100);
}

TEST(RateClampTest, ExtremeNegativeValue) {
    EXPECT_EQ(clamp_imu_rate(std::numeric_limits<int>::min()), kImuRateMinHz);
}

TEST(RateClampTest, ExtremePositiveValue) {
    EXPECT_EQ(clamp_vio_rate(std::numeric_limits<int>::max()), kVioRateMaxHz);
}

// ── Verify constexpr bounds are sane ───────────────────────

TEST(RateClampTest, BoundsArePositive) {
    EXPECT_GT(kImuRateMinHz, 0);
    EXPECT_GT(kImuRateMaxHz, 0);
    EXPECT_GT(kVioRateMinHz, 0);
    EXPECT_GT(kVioRateMaxHz, 0);
}

TEST(RateClampTest, MinLessThanMax) {
    EXPECT_LT(kImuRateMinHz, kImuRateMaxHz);
    EXPECT_LT(kVioRateMinHz, kVioRateMaxHz);
}
