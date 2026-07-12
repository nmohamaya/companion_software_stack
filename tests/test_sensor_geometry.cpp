// tests/test_sensor_geometry.cpp
// Unit tests for the attitude-aware sensor↔world geometry helpers (Issue #816).
//
// These lock the math the ground-filter safety fix depends on: a radar return's
// world-frame altitude must account for the full body→world rotation, not just
// yaw.  The key regression is that roll couples into the vertical at non-zero
// azimuth (a level-flight-only formula misses it entirely).
#include "util/sensor_geometry.h"

#include <cmath>

#include <gtest/gtest.h>

using drone::util::quat_from_rpy;
using drone::util::quat_from_wxyz;
using drone::util::ray_direction_world;
using drone::util::return_world_altitude_m;

namespace {
constexpr float kDeg = 3.14159265358979323846f / 180.0f;
}

// ── Identity: level flight reduces to the legacy sensor_alt + range·sin(el) ──
TEST(SensorGeometry, IdentityMatchesFlatFormula) {
    const Eigen::Quaternionf level = Eigen::Quaternionf::Identity();
    for (float el_deg : {-20.0f, -10.0f, 0.0f, 10.0f, 20.0f}) {
        const float el    = el_deg * kDeg;
        const float range = 15.0f;
        const float flat  = 5.0f + range * std::sin(el);
        const float got   = return_world_altitude_m(5.0f, range, /*az=*/0.3f, el, level);
        EXPECT_NEAR(got, flat, 1e-4f)
            << "level flight must equal the flat formula at el=" << el_deg;
    }
}

// ── Pure pitch raises/lowers the ray by the pitch angle (az = 0) ─────────────
TEST(SensorGeometry, PurePitchShiftsElevation) {
    const float range = 20.0f, el = 0.0f, sensor_alt = 5.0f;
    // Nose-up (+pitch) points a horizontal ray upward → higher world altitude.
    const float up   = return_world_altitude_m(sensor_alt, range, 0.0f, el,
                                               quat_from_rpy(0.0f, 10.0f * kDeg, 0.0f));
    const float down = return_world_altitude_m(sensor_alt, range, 0.0f, el,
                                               quat_from_rpy(0.0f, -10.0f * kDeg, 0.0f));
    EXPECT_NEAR(up, sensor_alt + range * std::sin(10.0f * kDeg), 1e-3f);
    EXPECT_NEAR(down, sensor_alt + range * std::sin(-10.0f * kDeg), 1e-3f);
    EXPECT_GT(up, down);
}

// ── Roll does NOT couple at az = 0, but DOES at non-zero azimuth ─────────────
// This is the term the flat formula misses. Measured worst case (issue #816):
// az 30° + roll 15° → ~2.59 m vertical error at 20 m.
TEST(SensorGeometry, RollCouplesIntoVerticalOnlyOffBoresight) {
    const float range = 20.0f, el = 0.0f, alt = 5.0f;
    const auto  roll15 = quat_from_rpy(15.0f * kDeg, 0.0f, 0.0f);

    // On boresight (az = 0): roll about the forward axis leaves a forward ray flat.
    EXPECT_NEAR(return_world_altitude_m(alt, range, 0.0f, el, roll15), alt, 1e-3f)
        << "roll must not change the altitude of an on-boresight ray";

    // Off boresight (az = 30°): the ray gains a vertical component sin(az)·sin(roll).
    const float got      = return_world_altitude_m(alt, range, 30.0f * kDeg, el, roll15);
    const float expected = alt + range * std::sin(30.0f * kDeg) * std::sin(15.0f * kDeg);
    EXPECT_NEAR(got, expected, 1e-2f);
    EXPECT_NEAR(got - alt, 2.59f, 0.05f) << "the #816 worst-case coupling magnitude";
}

// ── Mount composition: a sensor tilted down −5° lowers every ray ─────────────
TEST(SensorGeometry, MountPitchComposesWithBodyAttitude) {
    // Body level, sensor mounted pitch -0.087 rad (the x500_companion lidar).
    const float mount = -0.087f;
    const auto  q     = quat_from_rpy(0.0f, mount, 0.0f);
    const float range = 20.0f, alt = 5.0f;
    // A horizontal sensor ray (el=0) actually points 5° down in the world.
    EXPECT_NEAR(return_world_altitude_m(alt, range, 0.0f, 0.0f, q), alt + range * std::sin(mount),
                1e-3f);

    // Compose mount with body nose-up +10°: net +5° → ray points slightly up.
    const auto  composed = quat_from_rpy(0.0f, mount + 10.0f * kDeg, 0.0f);
    const float net      = return_world_altitude_m(alt, range, 0.0f, 0.0f, composed);
    EXPECT_GT(net, alt) << "mount(-5°) + body(+10°) = +5° net → ray points up";
}

// ── ray_direction_world returns a unit vector ────────────────────────────────
TEST(SensorGeometry, RayDirectionIsUnitLength) {
    for (float az_deg : {-30.0f, 0.0f, 30.0f}) {
        for (float el_deg : {-20.0f, 0.0f, 20.0f}) {
            const auto d = ray_direction_world(az_deg * kDeg, el_deg * kDeg,
                                               quat_from_rpy(5.0f * kDeg, -8.0f * kDeg, 1.2f));
            EXPECT_NEAR(d.norm(), 1.0f, 1e-5f);
        }
    }
}

// ── Fail-safe: non-finite / degenerate quaternion → identity, never garbage ──
TEST(SensorGeometry, NonFiniteQuaternionFallsBackToIdentity) {
    const float nan = std::nanf("");
    EXPECT_TRUE(quat_from_wxyz(nan, 0, 0, 0).isApprox(Eigen::Quaternionf::Identity()));
    EXPECT_TRUE(quat_from_wxyz(0, 0, 0, 0).isApprox(Eigen::Quaternionf::Identity()))
        << "zero-norm quaternion must degrade to identity, not divide by zero";
    // A valid (w,x,y,z) round-trips (Pose order matches Eigen ctor (w,x,y,z)).
    const auto q = quat_from_wxyz(std::cos(0.25f), 0.0f, 0.0f, std::sin(0.25f));  // yaw 0.5 rad
    EXPECT_NEAR(q.norm(), 1.0f, 1e-5f);
    EXPECT_TRUE(q.isApprox(quat_from_rpy(0.0f, 0.0f, 0.5f), 1e-4f));
}
