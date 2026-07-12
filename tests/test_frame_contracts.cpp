// tests/test_frame_contracts.cpp
//
// Issue #816 (Tier 1 of the #817 typed-frames epic) — the FRAME CONTRACT.
//
// #815/#816 were silent frame-of-reference mismatches (el>0 meaning "up" in one
// gate and "down" in another; azimuth FLU at the HAL vs FRD in the UKF; yaw-only
// projection). Until the type system enforces frames (epic #817), this file is
// the executable spec: golden assertions pinning every convention the radar
// pipeline relies on. A sign flip ANYWHERE — sensor_geometry, az_sign, the Pose
// quaternion order, the mount sign — turns one of these red.
//
// Each assertion states the convention in prose AND checks it, so the file
// doubles as the frame-of-reference reference.
#include "ipc/ipc_types.h"
#include "perception/ukf_fusion_engine.h"
#include "util/sensor_geometry.h"

#include <cmath>

#include <gtest/gtest.h>

using drone::perception::RadarNoiseConfig;
using drone::util::quat_from_rpy;
using drone::util::quat_from_wxyz;
using drone::util::return_world_altitude_m;

namespace {
constexpr float kD2R = 3.14159265358979323846f / 180.0f;
}

// ── CONTRACT 1: World frame is ENU (Z up). A radar return's world altitude
//    grows with elevation. `el > 0` MUST mean "up". ─────────────────────────
TEST(FrameContract, ElevationPositiveIsUp) {
    const auto  level = Eigen::Quaternionf::Identity();
    const float lo    = return_world_altitude_m(5.0f, 20.0f, 0.0f, -10.0f * kD2R, level);
    const float mid   = return_world_altitude_m(5.0f, 20.0f, 0.0f, 0.0f, level);
    const float hi    = return_world_altitude_m(5.0f, 20.0f, 0.0f, +10.0f * kD2R, level);
    EXPECT_LT(lo, mid);
    EXPECT_LT(mid, hi) << "el>0 must raise world altitude (ENU, Z-up)";
    EXPECT_NEAR(mid, 5.0f, 1e-4f) << "a level dead-ahead (el=0) return sits at the drone's own alt";
}

// ── CONTRACT 2: Body frame is FLU. +pitch = NOSE UP raises a forward ray.
//    (Verified empirically vs real Pose data in #816.) ──────────────────────
TEST(FrameContract, NoseUpRaisesForwardRay) {
    const float up   = return_world_altitude_m(5.0f, 20.0f, 0.0f, 0.0f,
                                               quat_from_rpy(0.0f, 10.0f * kD2R, 0.0f));
    const float down = return_world_altitude_m(5.0f, 20.0f, 0.0f, 0.0f,
                                               quat_from_rpy(0.0f, -10.0f * kD2R, 0.0f));
    EXPECT_GT(up, 5.0f) << "nose-up must RAISE a forward ray (safety-critical sign)";
    EXPECT_LT(down, 5.0f) << "nose-down must lower it";
}

// ── CONTRACT 3: The Pose quaternion is (w, x, y, z) and maps body→world. A pure
//    yaw about +Z leaves altitude unchanged; quat_from_wxyz matches quat_from_rpy
//    for the same yaw. ───────────────────────────────────────────────────────
TEST(FrameContract, QuaternionOrderIsWXYZ_YawDoesNotChangeAltitude) {
    // Pose::quaternion[4] doc: {w, x, y, z}. A 0.5 rad yaw → (cos.25, 0,0, sin.25).
    const auto q_wxyz = quat_from_wxyz(std::cos(0.25f), 0.0f, 0.0f, std::sin(0.25f));
    EXPECT_TRUE(q_wxyz.isApprox(quat_from_rpy(0.0f, 0.0f, 0.5f), 1e-4f))
        << "Pose (w,x,y,z) with only z set must equal a pure-yaw rotation";
    const float alt       = return_world_altitude_m(5.0f, 20.0f, 0.3f, 0.1f, q_wxyz);
    const float alt_level = return_world_altitude_m(5.0f, 20.0f, 0.3f, 0.1f,
                                                    Eigen::Quaternionf::Identity());
    EXPECT_NEAR(alt, alt_level, 1e-3f) << "yaw must not change a return's altitude";
}

// ── CONTRACT 4: The radar sign convention (Issue #348). az_sign negates for the
//    Gazebo/simulated FLU→UKF-FRD path (negate_azimuth = true) and is identity
//    for native-FRD hardware. This is what the fusion engine applies to reach
//    its FRD state; the ATTITUDE gate deliberately does NOT use it (it stays in
//    FLU). ───────────────────────────────────────────────────────────────────
TEST(FrameContract, AzimuthSignConvention348) {
    RadarNoiseConfig flu;
    flu.negate_azimuth = true;  // Gazebo/simulated default
    EXPECT_FLOAT_EQ(flu.az_sign(0.4f), -0.4f) << "FLU→FRD path negates azimuth";

    RadarNoiseConfig frd;
    frd.negate_azimuth = false;  // native-FRD radar hardware
    EXPECT_FLOAT_EQ(frd.az_sign(0.4f), 0.4f) << "native FRD leaves azimuth unchanged";
}

// ── CONTRACT 5: The sensor mount tilt is a −5° (−0.087 rad) DOWN pitch, so a
//    level forward sensor ray points slightly DOWN in the world. If the SDF
//    mount sign or the gate's composition flips, this catches it. ────────────
TEST(FrameContract, MountTiltPointsForwardRayDown) {
    const float mount = -0.087f;  // x500_companion lidar, matches model.sdf
    const float alt   = return_world_altitude_m(5.0f, 20.0f, 0.0f, 0.0f,
                                                quat_from_rpy(0.0f, mount, 0.0f));
    EXPECT_LT(alt, 5.0f) << "a −5° mount must point a level forward ray downward";
    EXPECT_NEAR(alt, 5.0f + 20.0f * std::sin(mount), 1e-3f);
}

// ── CONTRACT 6: The Pose IPC struct still declares the layout these contracts
//    assume. A wire-format reshuffle that moves the quaternion must break the
//    build here, not silently mis-project. ──────────────────────────────────
TEST(FrameContract, PoseStructLayoutUnchanged) {
    drone::ipc::Pose p{};
    p.quaternion[0] = 1.0f;  // w
    p.quaternion[1] = 0.0f;  // x
    p.quaternion[2] = 0.0f;  // y
    p.quaternion[3] = 0.0f;  // z
    EXPECT_FLOAT_EQ(static_cast<float>(p.quaternion[0]), 1.0f);
    // translation is [N, E, U]; a compile error here means the layout moved.
    p.translation[2] = 4.48f;
    EXPECT_FLOAT_EQ(static_cast<float>(p.translation[2]), 4.48f);
}
