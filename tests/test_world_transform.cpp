// tests/test_world_transform.cpp
// Unit tests for camera body-frame (FRD) → world-frame (NEU) transform.
// Validates the full-quaternion rotation introduced in Issue #421,
// replacing the previous yaw-only approximation.

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

// ── Standalone helper replicating the transform from main.cpp ──
// This lets us test the math without the full fusion pipeline.
// Coordinate convention:
//   Body frame: FRD (Forward, Right, Down)
//   World frame: NEU (North, East, Up)
// Steps:
//   1. Negate Z to convert FRD → FRU
//   2. Rotate by full VIO quaternion (FRU → NEU)
//   3. Translate by drone world position
struct WorldTransformResult {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
};

/// Apply the body FRD → world NEU transform used in the perception pipeline.
/// @param q_wxyz  VIO quaternion (w, x, y, z) — body-to-world rotation
/// @param drone_pos  Drone position in world frame (North, East, Up)
/// @param body_pos   Object position in body FRD frame
/// @param body_vel   Object velocity in body FRD frame
[[nodiscard]] static WorldTransformResult body_frd_to_world_neu(const Eigen::Quaterniond& q_wxyz,
                                                                const Eigen::Vector3d&    drone_pos,
                                                                const Eigen::Vector3f&    body_pos,
                                                                const Eigen::Vector3f& body_vel) {
    const Eigen::Matrix3d R = q_wxyz.normalized().toRotationMatrix();

    // FRD → FRU: negate Z before rotation
    const Eigen::Vector3d v_fru(body_vel.x(), body_vel.y(), -static_cast<double>(body_vel.z()));
    const Eigen::Vector3d v_world = R * v_fru;

    const Eigen::Vector3d p_fru(body_pos.x(), body_pos.y(), -static_cast<double>(body_pos.z()));
    const Eigen::Vector3d p_world = R * p_fru + drone_pos;

    WorldTransformResult result;
    result.velocity = v_world.cast<float>();
    result.position = p_world.cast<float>();
    return result;
}

// ── Test: Identity quaternion matches old z-flip behavior ──────
// With no rotation, FRD→FRU just negates Z, and R=I leaves it unchanged.
// body (10, 2, 3) FRD → (10, 2, -3) FRU → world (drone_n+10, drone_e+2, drone_u-3)
TEST(WorldTransformTest, IdentityQuaternionMatchesZFlip) {
    const Eigen::Quaterniond q_identity(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d    drone_pos(100.0, 50.0, 30.0);
    const Eigen::Vector3f    body_pos(10.0f, 2.0f, 3.0f);
    const Eigen::Vector3f    body_vel(5.0f, 1.0f, 2.0f);

    const auto result = body_frd_to_world_neu(q_identity, drone_pos, body_pos, body_vel);

    // Position: drone_pos + (10, 2, -3)
    EXPECT_NEAR(result.position.x(), 110.0f, 1e-4f);  // North
    EXPECT_NEAR(result.position.y(), 52.0f, 1e-4f);   // East
    EXPECT_NEAR(result.position.z(), 27.0f, 1e-4f);   // Up = 30 - 3

    // Velocity: (5, 1, -2) — z negated
    EXPECT_NEAR(result.velocity.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(result.velocity.y(), 1.0f, 1e-4f);
    EXPECT_NEAR(result.velocity.z(), -2.0f, 1e-4f);
}

// ── Test: Pure 90-degree yaw regression ────────────────────────
// Yaw 90° CW: body forward (+X) → world East (+Y).
// Quaternion for 90° yaw (rotation about Z in FRU/NEU):
//   q = (cos(45°), 0, 0, sin(45°)) = (√2/2, 0, 0, √2/2)
// body (10, 0, 0) FRD → (10, 0, 0) FRU → rotated: (0, 10, 0) NEU
TEST(WorldTransformTest, PureYaw90DegreesRotatesForwardToEast) {
    const double             s = std::sqrt(2.0) / 2.0;
    const Eigen::Quaterniond q_yaw90(s, 0.0, 0.0, s);  // 90° about Z
    const Eigen::Vector3d    drone_pos(100.0, 50.0, 30.0);
    const Eigen::Vector3f    body_pos(10.0f, 0.0f, 0.0f);
    const Eigen::Vector3f    body_vel(10.0f, 0.0f, 0.0f);

    const auto result = body_frd_to_world_neu(q_yaw90, drone_pos, body_pos, body_vel);

    // Forward (10,0,0) rotated 90° yaw → (0, 10, 0) in world
    EXPECT_NEAR(result.position.x(), 100.0f, 1e-3f);  // drone_n + 0
    EXPECT_NEAR(result.position.y(), 60.0f, 1e-3f);   // drone_e + 10
    EXPECT_NEAR(result.position.z(), 30.0f, 1e-3f);   // drone_u + 0

    EXPECT_NEAR(result.velocity.x(), 0.0f, 1e-3f);
    EXPECT_NEAR(result.velocity.y(), 10.0f, 1e-3f);
    EXPECT_NEAR(result.velocity.z(), 0.0f, 1e-3f);
}

// ── Test: 10-degree nose-down pitch shifts vertical component ──
// In FRU (X=Forward, Y=Right, Z=Up), pitch is rotation about Y.
// Right-hand rule: positive rotation about +Y tilts +X toward -Z
// (nose down). So nose-down = positive pitch angle about Y.
//
// body (10,0,0) FRD → (10,0,0) FRU → after Ry(+10°):
//   x' = 10*cos(10°) ≈ 9.848
//   z' = -10*sin(10°) ≈ -1.736  (tilted down in world)
TEST(WorldTransformTest, Pitch10DegNoseDownShiftsVertical) {
    const double pitch_rad = 10.0 * M_PI / 180.0;  // nose-down in FRU

    // Quaternion for pitch-only (rotation about body Y axis in FRU):
    const Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));

    const Eigen::Vector3d drone_pos(0.0, 0.0, 50.0);
    const Eigen::Vector3f body_pos(10.0f, 0.0f, 0.0f);  // 10m forward in body
    const Eigen::Vector3f body_vel(0.0f, 0.0f, 0.0f);

    const auto result = body_frd_to_world_neu(q_pitch, drone_pos, body_pos, body_vel);

    // North component reduced: 10*cos(10°) ≈ 9.848
    EXPECT_NEAR(result.position.x(), 10.0f * std::cos(static_cast<float>(pitch_rad)), 1e-3f);
    EXPECT_NEAR(result.position.y(), 0.0f, 1e-3f);

    // The key assertion: world Z differs from the no-pitch case (50.0)
    const float world_z_no_pitch = 50.0f;
    EXPECT_GT(std::abs(result.position.z() - world_z_no_pitch), 1.0f)
        << "10-degree pitch should shift vertical by >1m at 10m range";

    // Verify direction: nose-down pitch moves the point below drone altitude
    EXPECT_LT(result.position.z(), world_z_no_pitch);
}

// ── Test: Combined yaw + pitch ─────────────────────────────────
// Verify that yaw and pitch compose correctly.
TEST(WorldTransformTest, CombinedYawAndPitchCompose) {
    const double yaw_rad   = M_PI / 4.0;           // 45° yaw
    const double pitch_rad = 15.0 * M_PI / 180.0;  // 15° nose-down in FRU

    // Build quaternion: yaw about Z, then pitch about Y (in body frame)
    const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond q_combined = q_yaw * q_pitch;

    const Eigen::Vector3d drone_pos(0.0, 0.0, 100.0);
    const Eigen::Vector3f body_pos(20.0f, 0.0f, 0.0f);
    const Eigen::Vector3f body_vel(0.0f, 0.0f, 0.0f);

    const auto result = body_frd_to_world_neu(q_combined, drone_pos, body_pos, body_vel);

    // With 45° yaw and 15° nose-down pitch, forward 20m should split
    // between N and E and have a vertical component from pitch.
    const float horizontal = std::sqrt(result.position.x() * result.position.x() +
                                       result.position.y() * result.position.y());
    EXPECT_NEAR(horizontal, 20.0f * std::cos(15.0f * static_cast<float>(M_PI) / 180.0f), 0.1f);

    // Nose-down pitch should place the object below drone altitude
    EXPECT_LT(result.position.z(), 100.0f)
        << "Nose-down pitch should place object below drone altitude";
}

// ── Test: Velocity transform is rotation-only (no translation) ─
TEST(WorldTransformTest, VelocityNotAffectedByDronePosition) {
    const double             s = std::sqrt(2.0) / 2.0;
    const Eigen::Quaterniond q_yaw90(s, 0.0, 0.0, s);

    const Eigen::Vector3d drone_pos_a(0.0, 0.0, 0.0);
    const Eigen::Vector3d drone_pos_b(999.0, 999.0, 999.0);
    const Eigen::Vector3f body_pos(0.0f, 0.0f, 0.0f);
    const Eigen::Vector3f body_vel(3.0f, 4.0f, 5.0f);

    const auto result_a = body_frd_to_world_neu(q_yaw90, drone_pos_a, body_pos, body_vel);
    const auto result_b = body_frd_to_world_neu(q_yaw90, drone_pos_b, body_pos, body_vel);

    // Velocity should be identical regardless of drone position
    EXPECT_NEAR(result_a.velocity.x(), result_b.velocity.x(), 1e-6f);
    EXPECT_NEAR(result_a.velocity.y(), result_b.velocity.y(), 1e-6f);
    EXPECT_NEAR(result_a.velocity.z(), result_b.velocity.z(), 1e-6f);
}
