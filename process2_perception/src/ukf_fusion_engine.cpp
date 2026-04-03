// process2_perception/src/ukf_fusion_engine.cpp
// UKF fusion engine implementation — camera + radar measurement models.
// Phase 1C (Issue #114), radar fusion (Issue #210).
#include "perception/ukf_fusion_engine.h"

#include "perception/fusion_engine.h"
#include "util/config.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <spdlog/spdlog.h>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// ObjectUKF
// ═══════════════════════════════════════════════════════════
ObjectUKF::ObjectUKF(const TrackedObject& trk, float initial_depth,
                     const RadarNoiseConfig& radar_cfg, const CameraIntrinsics& intrinsics)
    : track_id(trk.track_id), age(0) {
    // Initialize state: [x=depth, y=bearing_x*depth, z=bearing_y*depth, vx, vy, vz]
    // Bearing computed via pinhole camera model: (pixel - principal_point) / focal_length
    const float bearing_x = (trk.position_2d.x() - intrinsics.cx) / std::max(1.0f, intrinsics.fx);
    const float bearing_y = (trk.position_2d.y() - intrinsics.cy) / std::max(1.0f, intrinsics.fy);
    x_                    = StateVec::Zero();
    x_(0)                 = initial_depth;               // x (forward / depth)
    x_(1)                 = bearing_x * initial_depth;   // y (lateral, from calibrated bearing)
    x_(2)                 = bearing_y * initial_depth;   // z (vertical, from calibrated bearing)
    x_(3)                 = trk.velocity_2d.x() * 0.1f;  // vx
    x_(4)                 = trk.velocity_2d.y() * 0.1f;  // vy
    x_(5)                 = 0.0f;                        // vz

    // Initial covariance — high uncertainty
    P_       = StateMat::Identity() * 10.0f;
    P_(3, 3) = 50.0f;  // velocity highly uncertain
    P_(4, 4) = 50.0f;
    P_(5, 5) = 50.0f;

    // Process noise (constant velocity model)
    Q_       = StateMat::Identity() * 0.5f;
    Q_(0, 0) = 0.1f;
    Q_(1, 1) = 0.1f;
    Q_(2, 2) = 0.05f;
    Q_(3, 3) = 1.0f;
    Q_(4, 4) = 1.0f;
    Q_(5, 5) = 0.5f;

    // Camera measurement noise
    R_ = MeasMat::Identity() * 2.0f;

    // Radar measurement noise: diag([range², azimuth², elevation², velocity²])
    R_radar_       = RadarMeasMat::Zero();
    R_radar_(0, 0) = radar_cfg.range_std_m * radar_cfg.range_std_m;
    R_radar_(1, 1) = radar_cfg.azimuth_std_rad * radar_cfg.azimuth_std_rad;
    R_radar_(2, 2) = radar_cfg.elevation_std_rad * radar_cfg.elevation_std_rad;
    R_radar_(3, 3) = radar_cfg.velocity_std_mps * radar_cfg.velocity_std_mps;
}

// ── Radar-only constructor (Phase D) ──────────────────────
ObjectUKF::ObjectUKF(const drone::ipc::RadarDetection& det, uint32_t id,
                     const RadarNoiseConfig& radar_cfg)
    : track_id(id), age(0), radar_update_count(1), radar_only(true) {
    // Convert spherical radar (range, azimuth, elevation) to body-frame FRD.
    // Negate azimuth: Gazebo FLU → UKF FRD convention.
    const float neg_az = -det.azimuth_rad;
    const float cos_el = std::cos(det.elevation_rad);
    const float sin_el = std::sin(det.elevation_rad);

    x_    = StateVec::Zero();
    x_(0) = det.range_m * cos_el * std::cos(neg_az);  // x (forward / depth)
    x_(1) = det.range_m * cos_el * std::sin(neg_az);  // y (right / lateral)
    x_(2) = det.range_m * sin_el;                     // z (down / vertical)
    // Project radial velocity along look direction (only component observable)
    x_(3) = det.radial_velocity_mps * cos_el * std::cos(neg_az);  // vx
    x_(4) = 0.0f;                                                 // vy
    x_(5) = 0.0f;                                                 // vz

    // Covariance: range is accurate, lateral/vertical scale with range.
    P_       = StateMat::Zero();
    P_(0, 0) = 0.5f;  // radar range ±0.3m → conservative 0.5
    P_(1, 1) = det.range_m * det.range_m * radar_cfg.azimuth_std_rad *
               radar_cfg.azimuth_std_rad;  // lateral uncertainty ∝ range
    P_(2, 2) = det.range_m * det.range_m * radar_cfg.elevation_std_rad *
               radar_cfg.elevation_std_rad;  // vertical uncertainty ∝ range
    P_(3, 3) = 50.0f;                        // velocity highly uncertain
    P_(4, 4) = 50.0f;
    P_(5, 5) = 50.0f;

    // Process noise (constant velocity model, same as camera constructor)
    Q_       = StateMat::Identity() * 0.5f;
    Q_(0, 0) = 0.1f;
    Q_(1, 1) = 0.1f;
    Q_(2, 2) = 0.05f;
    Q_(3, 3) = 1.0f;
    Q_(4, 4) = 1.0f;
    Q_(5, 5) = 0.5f;

    // Camera measurement noise (not used until camera adopts, but must be initialized)
    R_ = MeasMat::Identity() * 2.0f;

    // Radar measurement noise
    R_radar_       = RadarMeasMat::Zero();
    R_radar_(0, 0) = radar_cfg.range_std_m * radar_cfg.range_std_m;
    R_radar_(1, 1) = radar_cfg.azimuth_std_rad * radar_cfg.azimuth_std_rad;
    R_radar_(2, 2) = radar_cfg.elevation_std_rad * radar_cfg.elevation_std_rad;
    R_radar_(3, 3) = radar_cfg.velocity_std_mps * radar_cfg.velocity_std_mps;
}

void ObjectUKF::adopt_camera(const TrackedObject& trk, const CameraIntrinsics& cam_intr) {
    // Camera provides accurate bearing — refine lateral/vertical from pixel position.
    const float bearing_x = (trk.position_2d.x() - cam_intr.cx) / std::max(1.0f, cam_intr.fx);
    const float bearing_y = (trk.position_2d.y() - cam_intr.cy) / std::max(1.0f, cam_intr.fy);
    const float depth     = x_(0);  // keep radar-confirmed depth

    // Overwrite lateral/vertical with camera bearing × radar depth
    x_(1) = bearing_x * depth;
    x_(2) = bearing_y * depth;

    // Tighten bearing covariance (camera is sub-degree accurate)
    P_(1, 1) = std::min(P_(1, 1), 2.0f);
    P_(2, 2) = std::min(P_(2, 2), 2.0f);

    radar_only = false;
}

void ObjectUKF::set_radar_confirmed_depth(float radar_range) {
    // Rewrite depth (x) and lateral/vertical (y, z) using the radar range
    // while preserving the bearing direction from the camera.
    const float old_depth = std::max(x_(0), 0.1f);
    const float scale     = radar_range / old_depth;
    x_(0)                 = radar_range;
    x_(1) *= scale;  // lateral scales with depth
    x_(2) *= scale;  // vertical scales with depth

    // Tighten depth covariance: radar range is ±0.3m, so P(0,0) ≈ 0.09.
    // Use 0.5 to be slightly conservative (accounts for body-frame uncertainty).
    P_(0, 0) = 0.5f;
    // Lateral/vertical covariance stays at initial values (bearing-driven).
    // Note: radar_update_count is NOT incremented here — this is a one-time
    // depth snap from radar-init, not a proper radar measurement update.
    // radar_update_count tracks actual update_radar() calls for promotion.
}

std::vector<ObjectUKF::StateVec> ObjectUKF::generate_sigma_points() const {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;
    const float     scale  = std::sqrt(n + lambda);

    std::vector<StateVec> sigma(2 * n + 1);
    sigma[0] = x_;

    // Compute matrix square root of P_ using Cholesky.
    // P_ can become numerically non-positive-definite over time,
    // so we check the decomposition and regularize if needed.
    Eigen::LLT<StateMat> llt(P_);
    StateMat             L;

    if (llt.info() == Eigen::Success) {
        L = llt.matrixL();
    } else {
        // Regularize P_ by adding small diagonal jitter
        StateMat        P_reg;
        constexpr float kBaseJitter = 1e-3f;
        bool            success     = false;

        for (int iter = 0; iter < 3 && !success; ++iter) {
            const float jitter = kBaseJitter * std::pow(10.0f, static_cast<float>(iter));
            P_reg              = P_;
            P_reg.diagonal().array() += jitter;

            llt.compute(P_reg);
            if (llt.info() == Eigen::Success) {
                L       = llt.matrixL();
                success = true;
            }
        }

        if (!success) {
            // Last resort: identity to avoid NaN propagation
            L = StateMat::Identity();
        }
    }

    for (int i = 0; i < n; ++i) {
        sigma[1 + i]     = x_ + scale * L.col(i);
        sigma[1 + n + i] = x_ - scale * L.col(i);
    }
    return sigma;
}

void ObjectUKF::predict(float dt) {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;

    auto sigma = generate_sigma_points();

    // Propagate through constant-velocity model: x_new = x + v*dt
    for (auto& sp : sigma) {
        sp(0) += sp(3) * dt;
        sp(1) += sp(4) * dt;
        sp(2) += sp(5) * dt;
    }

    // Compute weights
    float w0_mean = lambda / (n + lambda);
    float w0_cov  = w0_mean + (1.0f - kAlpha * kAlpha + kBeta);
    float wi      = 1.0f / (2.0f * (n + lambda));

    // Weighted mean
    x_ = w0_mean * sigma[0];
    for (int i = 1; i < 2 * n + 1; ++i) {
        x_ += wi * sigma[i];
    }

    // Weighted covariance
    StateVec d0 = sigma[0] - x_;
    P_          = w0_cov * (d0 * d0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        StateVec di = sigma[i] - x_;
        P_ += wi * (di * di.transpose());
    }
    P_ += Q_;

    ++age;
}

void ObjectUKF::update_camera(const TrackedObject& trk, float estimated_depth,
                              const CameraIntrinsics& intrinsics) {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;

    auto sigma = generate_sigma_points();

    // Measurement model: h(x) = [bearing_x, bearing_y, depth] ≈ [x1/x0, x2/x0, x0]
    auto h = [](const StateVec& s) -> MeasVec {
        float depth = std::max(0.1f, s(0));  // avoid divide by zero
        return MeasVec(s(1) / depth, s(2) / depth, depth);
    };

    // Propagate sigma points through measurement model
    std::vector<MeasVec> z_sigma(2 * n + 1);
    for (int i = 0; i < 2 * n + 1; ++i) {
        z_sigma[i] = h(sigma[i]);
    }

    // Weights
    float w0_mean = lambda / (n + lambda);
    float w0_cov  = w0_mean + (1.0f - kAlpha * kAlpha + kBeta);
    float wi      = 1.0f / (2.0f * (n + lambda));

    // Mean predicted measurement
    MeasVec z_mean = w0_mean * z_sigma[0];
    for (int i = 1; i < 2 * n + 1; ++i) {
        z_mean += wi * z_sigma[i];
    }

    // Innovation covariance S = Pzz + R
    MeasVec dz0 = z_sigma[0] - z_mean;
    MeasMat S   = w0_cov * (dz0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        MeasVec dzi = z_sigma[i] - z_mean;
        S += wi * (dzi * dzi.transpose());
    }
    S += R_;

    // Cross-covariance Pxz
    using CrossMat = Eigen::Matrix<float, STATE_DIM, MEAS_DIM>;
    StateVec dx0   = sigma[0] - x_;
    CrossMat Pxz   = w0_cov * (dx0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        StateVec dxi = sigma[i] - x_;
        MeasVec  dzi = z_sigma[i] - z_mean;
        Pxz += wi * (dxi * dzi.transpose());
    }

    // Kalman gain (solve without forming S.inverse() for numerical stability)
    CrossMat K = (S.ldlt().solve(Pxz.transpose())).transpose();

    // Actual measurement — pinhole camera unprojection for calibrated bearings
    const float bearing_x = (trk.position_2d.x() - intrinsics.cx) / std::max(1.0f, intrinsics.fx);
    const float bearing_y = (trk.position_2d.y() - intrinsics.cy) / std::max(1.0f, intrinsics.fy);
    MeasVec     z_actual(bearing_x, bearing_y, estimated_depth);

    // Update
    x_ += K * (z_actual - z_mean);
    P_ -= K * S * K.transpose();

    // Ensure P stays symmetric positive definite
    P_ = (P_ + P_.transpose()) * 0.5f;
}

// ═══════════════════════════════════════════════════════════
// Radar measurement model and update
// ═══════════════════════════════════════════════════════════

ObjectUKF::RadarMeasVec ObjectUKF::radar_measurement_model(const StateVec& s) {
    const float x  = s(0);
    const float y  = s(1);
    const float z  = s(2);
    const float vx = s(3);
    const float vy = s(4);
    const float vz = s(5);

    const float range_sq = x * x + y * y + z * z;
    const float range    = std::sqrt(std::max(range_sq, 1e-6f));  // avoid zero
    const float xy_dist  = std::sqrt(std::max(x * x + y * y, 1e-6f));

    RadarMeasVec z_out;
    z_out(0) = range;                               // range
    z_out(1) = std::atan2(y, x);                    // azimuth
    z_out(2) = std::atan2(z, xy_dist);              // elevation
    z_out(3) = (x * vx + y * vy + z * vz) / range;  // radial velocity
    return z_out;
}

ObjectUKF::RadarMeasVec ObjectUKF::predicted_radar_measurement() const {
    return radar_measurement_model(x_);
}

void ObjectUKF::update_radar(const drone::ipc::RadarDetection& det) {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;

    auto sigma = generate_sigma_points();

    // Propagate sigma points through radar measurement model
    std::vector<RadarMeasVec> z_sigma(2 * n + 1);
    for (int i = 0; i < 2 * n + 1; ++i) {
        z_sigma[i] = radar_measurement_model(sigma[i]);
    }

    // Weights
    float w0_mean = lambda / (n + lambda);
    float w0_cov  = w0_mean + (1.0f - kAlpha * kAlpha + kBeta);
    float wi      = 1.0f / (2.0f * (n + lambda));

    // Mean predicted measurement — use circular mean for angular components
    // (azimuth at index 1, elevation at index 2) to handle ±π wrapping.
    float        sin_az_sum = 0.0f, cos_az_sum = 0.0f;
    float        sin_el_sum = 0.0f, cos_el_sum = 0.0f;
    RadarMeasVec z_mean = RadarMeasVec::Zero();

    // First sigma point (weight w0_mean)
    z_mean(0) += w0_mean * z_sigma[0](0);  // range
    z_mean(3) += w0_mean * z_sigma[0](3);  // radial velocity
    sin_az_sum += w0_mean * std::sin(z_sigma[0](1));
    cos_az_sum += w0_mean * std::cos(z_sigma[0](1));
    sin_el_sum += w0_mean * std::sin(z_sigma[0](2));
    cos_el_sum += w0_mean * std::cos(z_sigma[0](2));

    for (int i = 1; i < 2 * n + 1; ++i) {
        z_mean(0) += wi * z_sigma[i](0);
        z_mean(3) += wi * z_sigma[i](3);
        sin_az_sum += wi * std::sin(z_sigma[i](1));
        cos_az_sum += wi * std::cos(z_sigma[i](1));
        sin_el_sum += wi * std::sin(z_sigma[i](2));
        cos_el_sum += wi * std::cos(z_sigma[i](2));
    }
    z_mean(1) = std::atan2(sin_az_sum, cos_az_sum);
    z_mean(2) = std::atan2(sin_el_sum, cos_el_sum);

    // Innovation covariance S = Pzz + R_radar (with angle-wrapped residuals)
    RadarMeasVec dz0 = z_sigma[0] - z_mean;
    dz0(1)           = wrap_angle(dz0(1));
    dz0(2)           = wrap_angle(dz0(2));
    RadarMeasMat S   = w0_cov * (dz0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        RadarMeasVec dzi = z_sigma[i] - z_mean;
        dzi(1)           = wrap_angle(dzi(1));
        dzi(2)           = wrap_angle(dzi(2));
        S += wi * (dzi * dzi.transpose());
    }
    S += R_radar_;

    // Cross-covariance Pxz (with angle-wrapped residuals)
    using RadarCrossMat = Eigen::Matrix<float, STATE_DIM, RADAR_MEAS_DIM>;
    StateVec      dx0   = sigma[0] - x_;
    RadarCrossMat Pxz   = w0_cov * (dx0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        StateVec     dxi = sigma[i] - x_;
        RadarMeasVec dzi = z_sigma[i] - z_mean;
        dzi(1)           = wrap_angle(dzi(1));
        dzi(2)           = wrap_angle(dzi(2));
        Pxz += wi * (dxi * dzi.transpose());
    }

    // Kalman gain (solve via LDLT for numerical stability)
    RadarCrossMat K = (S.ldlt().solve(Pxz.transpose())).transpose();

    // Actual radar measurement.
    // Negate azimuth: Gazebo radar uses FLU convention (positive = left),
    // but the UKF body frame is FRD (positive azimuth = right).
    RadarMeasVec z_actual;
    z_actual(0) = det.range_m;
    z_actual(1) = -det.azimuth_rad;
    z_actual(2) = det.elevation_rad;
    z_actual(3) = det.radial_velocity_mps;

    // Update with angle-wrapped innovation
    RadarMeasVec innovation = z_actual - z_mean;
    innovation(1)           = wrap_angle(innovation(1));
    innovation(2)           = wrap_angle(innovation(2));
    x_ += K * innovation;
    P_ -= K * S * K.transpose();

    // Ensure P stays symmetric positive definite
    P_ = (P_ + P_.transpose()) * 0.5f;

    ++radar_update_count;
}

Eigen::Vector3f ObjectUKF::position() const {
    return {x_(0), x_(1), x_(2)};
}

Eigen::Vector3f ObjectUKF::velocity() const {
    return {x_(3), x_(4), x_(5)};
}

Eigen::Matrix3f ObjectUKF::position_covariance() const {
    return P_.block<3, 3>(0, 0);
}

// ═══════════════════════════════════════════════════════════
// UKFFusionEngine
// ═══════════════════════════════════════════════════════════
UKFFusionEngine::UKFFusionEngine(const CalibrationData& calib, const RadarNoiseConfig& radar_cfg,
                                 bool radar_enabled, float dormant_merge_radius_m, int max_dormant)
    : calib_(calib)
    , radar_cfg_(radar_cfg)
    , radar_enabled_(radar_enabled)
    , dormant_merge_radius_m_(dormant_merge_radius_m)
    , max_dormant_(max_dormant) {
    spdlog::info("[UKF] radar_enabled={}, ground_filter_enabled={}, ground_filter_alt_m={:.2f}, "
                 "altitude_gate_m={:.1f}, gate_threshold={:.1f}, dormant_merge_radius={:.1f}m, "
                 "max_dormant={}",
                 radar_enabled_, radar_cfg_.ground_filter_enabled, radar_cfg_.min_object_altitude_m,
                 radar_cfg_.altitude_gate_m, radar_cfg_.gate_threshold, dormant_merge_radius_m_,
                 max_dormant_);
}

UKFFusionEngine::DepthEstimate UKFFusionEngine::estimate_depth(const TrackedObject& trk) const {
    const float fy = calib_.camera_intrinsics(1, 1);
    const float cy = calib_.camera_intrinsics(1, 2);

    // Four-tier depth estimation with confidence:
    //
    // 1. Horizon-truncated bbox (conf=0.6): bbox top at/above horizon,
    //    use ground-plane geometry from bbox bottom.  Geometric but
    //    extrapolating — moderate confidence.
    //
    // 2. Apparent-size (conf=0.7): depth = assumed_height * fy / bbox_h
    //    Most reliable camera-only method when full height visible.
    //
    // 3. Ground-plane fallback (conf=0.3): depth = camera_height / ray_down
    //    Weak signal from small detections — often ground features.
    //
    // 4. Near-horizon fallback (conf=0.0): conservative 8m estimate.
    //    Pure guess with no geometric basis.
    //
    // Confidence values gate downstream promotion: only detections with
    // confidence >= min_promotion_depth_confidence become permanent static
    // cells.  Low-confidence detections still create temporary dynamic cells.
    constexpr float kBboxHThreshold  = 10.0f;
    constexpr float kDepthMinM       = 1.0f;
    constexpr float kDepthMaxM       = 40.0f;
    constexpr float kRayDownMinThres = 0.01f;
    constexpr float kFallbackDepthM  = 8.0f;
    constexpr float kHorizonMarginPx = 5.0f;  // bbox top within 5px of cy = truncated

    const float ds = calib_.depth_scale;

    // Bbox top in pixel coords (centroid - half height)
    const float bbox_top = trk.position_2d.y() - trk.bbox_h * 0.5f;

    // Tier 1: Horizon-truncated — bbox top at or above optical center.
    // The obstacle extends above the frame, so bbox_h < true projected height.
    // Use the bbox BOTTOM to estimate depth via ground-plane geometry:
    //   depth = drone_altitude * fy / (bbox_bottom - cy)
    // where bbox_bottom is the pixel row of the obstacle base (assumed ground level).
    if (bbox_top <= cy + kHorizonMarginPx && trk.bbox_h > kBboxHThreshold) {
        const float bbox_bottom   = trk.position_2d.y() + trk.bbox_h * 0.5f;
        const float ray_down_base = (bbox_bottom - cy) / std::max(1.0f, fy);
        if (ray_down_base > kRayDownMinThres && has_altitude_) {
            return {std::clamp(drone_altitude_m_ * fy / (bbox_bottom - cy) * ds, kDepthMinM,
                               kDepthMaxM),
                    0.6f};
        }
        if (ray_down_base > kRayDownMinThres) {
            return {std::clamp(calib_.camera_height_m / ray_down_base * ds, kDepthMinM, kDepthMaxM),
                    0.6f};
        }
    }

    // Tier 2: Full-height apparent-size depth (bbox fully visible)
    if (trk.bbox_h > kBboxHThreshold) {
        return {std::clamp(calib_.assumed_obstacle_height_m * fy / trk.bbox_h * ds, kDepthMinM,
                           kDepthMaxM),
                0.7f};
    }

    // Tier 3: Ground-plane fallback for small detections
    const float ray_down = (trk.position_2d.y() - cy) / std::max(1.0f, fy);
    if (ray_down > kRayDownMinThres) {
        return {std::clamp(calib_.camera_height_m / ray_down * ds, kDepthMinM, kDepthMaxM), 0.3f};
    }

    // Tier 4: Near-horizon conservative estimate — no geometric basis
    return {kFallbackDepthM * ds, 0.0f};
}

void UKFFusionEngine::reset() {
    filters_.clear();
    dormant_obstacles_.clear();
    track_to_dormant_.clear();
    has_radar_data_      = false;
    next_radar_track_id_ = 0x80000000u;
}

void UKFFusionEngine::set_drone_pose(float north, float east, float up, float yaw) {
    drone_north_ = north;
    drone_east_  = east;
    drone_up_    = up;
    drone_yaw_   = yaw;
    has_pose_    = true;
}

Eigen::Vector3f UKFFusionEngine::body_to_world(const Eigen::Vector3f& body) const {
    const float cos_y = std::cos(drone_yaw_);
    const float sin_y = std::sin(drone_yaw_);
    return {drone_north_ + body.x() * cos_y - body.y() * sin_y,
            drone_east_ + body.x() * sin_y + body.y() * cos_y, drone_up_ - body.z()};
}

int UKFFusionEngine::find_nearest_dormant(const Eigen::Vector3f& world_pos,
                                          const Eigen::Matrix3f& pos_cov) const {
    // Covariance-aware merge radius (A3): use 2σ of the largest position
    // uncertainty axis, clamped to [1.0, dormant_merge_radius_m_].
    // Radar-confirmed tracks (small P) get tight radius → no cross-contamination.
    // Camera-only tracks (large P) get wider radius → correct re-identification.
    const float sigma_max      = std::sqrt(std::max(pos_cov(0, 0), pos_cov(1, 1)));
    const float dynamic_radius = std::clamp(2.0f * sigma_max, 1.0f, dormant_merge_radius_m_);

    int   best_idx  = -1;
    float best_dist = dynamic_radius;
    for (int i = 0; i < static_cast<int>(dormant_obstacles_.size()); ++i) {
        const float dist = (dormant_obstacles_[i].world_pos.head<2>() - world_pos.head<2>()).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = i;
        }
    }
    return best_idx;
}

void UKFFusionEngine::set_radar_detections(const drone::ipc::RadarDetectionList& detections) {
    if (!radar_enabled_) return;  // ignore radar data when disabled
    radar_dets_     = detections;
    has_radar_data_ = true;
}

void UKFFusionEngine::set_drone_altitude(float altitude_m) {
    drone_altitude_m_ = altitude_m;
    has_altitude_     = true;
}

bool UKFFusionEngine::try_associate_radar(ObjectUKF& ukf, std::vector<bool>& radar_matched,
                                          const Eigen::LLT<ObjectUKF::RadarMeasMat>& radar_llt,
                                          bool                                       radar_llt_ok,
                                          const ObjectUKF::RadarMeasVec&             R_diag_inv,
                                          float                                      range_3sigma) {
    if (!has_radar_data_ || radar_dets_.num_detections == 0) return false;

    auto     z_pred    = ukf.predicted_radar_measurement();
    float    best_dist = std::numeric_limits<float>::max();
    uint32_t best_idx  = 0;

    for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
        if (radar_matched[ri]) continue;
        const auto& rdet = radar_dets_.detections[ri];

        // Coarse range gate
        if (std::abs(rdet.range_m - z_pred(0)) > range_3sigma) continue;

        // Altitude gate
        {
            const float radar_z = rdet.range_m * std::sin(rdet.elevation_rad);
            const float track_z = ukf.position()(2);
            if (std::abs(radar_z - track_z) > radar_cfg_.altitude_gate_m) continue;
        }

        // Negate azimuth: Gazebo FLU → UKF FRD
        ObjectUKF::RadarMeasVec z_actual;
        z_actual(0) = rdet.range_m;
        z_actual(1) = -rdet.azimuth_rad;
        z_actual(2) = rdet.elevation_rad;
        z_actual(3) = rdet.radial_velocity_mps;

        ObjectUKF::RadarMeasVec innovation = z_actual - z_pred;
        innovation(1)                      = wrap_angle(innovation(1));
        innovation(2)                      = wrap_angle(innovation(2));

        float mahal_sq = std::numeric_limits<float>::max();
        if (radar_llt_ok) {
            ObjectUKF::RadarMeasVec y = radar_llt.solve(innovation);
            mahal_sq                  = innovation.dot(y);
        } else {
            mahal_sq = 0.0f;
            for (int d = 0; d < ObjectUKF::RADAR_MEAS_DIM; ++d) {
                mahal_sq += innovation(d) * innovation(d) * R_diag_inv(d);
            }
        }

        if (mahal_sq < radar_cfg_.gate_threshold && mahal_sq < best_dist) {
            best_dist = mahal_sq;
            best_idx  = ri;
        }
    }

    if (best_dist < radar_cfg_.gate_threshold) {
        ukf.update_radar(radar_dets_.detections[best_idx]);
        radar_matched[best_idx] = true;
        return true;
    }
    return false;
}

FusedObjectList UKFFusionEngine::fuse(const TrackedObjectList& tracked) {
    FusedObjectList output;
    output.timestamp_ns   = tracked.timestamp_ns;
    output.frame_sequence = tracked.frame_sequence;

    // Track which UKF filters are still active this frame
    std::unordered_map<uint32_t, bool> seen;

    // Track which radar detections have been matched (greedy, one-to-one)
    std::vector<bool> radar_matched;

    // Hoist Cholesky decomposition of R_radar outside the track loop.
    // R_radar is constant across all (track, detection) pairs — computing it
    // once avoids O(n_tracks × n_detections) redundant decompositions.
    Eigen::LLT<ObjectUKF::RadarMeasMat> radar_llt;
    bool                                radar_llt_ok = false;
    ObjectUKF::RadarMeasVec             R_diag_inv   = ObjectUKF::RadarMeasVec::Zero();
    const float                         range_3sigma = 3.0f * radar_cfg_.range_std_m;

    if (has_radar_data_) {
        radar_matched.resize(radar_dets_.num_detections, false);

        // Ground-plane filter: reject radar detections that resolve below a minimum
        // altitude threshold.  object_alt = drone_alt + range * sin(elevation).
        // Ground returns have negative elevation and resolve near Z=0.
        if (radar_cfg_.ground_filter_enabled && has_altitude_) {
            int filtered = 0;
            for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
                const auto& rdet       = radar_dets_.detections[ri];
                const float object_alt = drone_altitude_m_ +
                                         rdet.range_m * std::sin(rdet.elevation_rad);
                if (object_alt < radar_cfg_.min_object_altitude_m) {
                    radar_matched[ri] = true;  // skip in association loop
                    ++filtered;
                }
            }
            if (filtered > 0) {
                spdlog::debug("[UKF] Ground filter: rejected {}/{} radar detections "
                              "(alt < {:.1f}m)",
                              filtered, radar_dets_.num_detections,
                              radar_cfg_.min_object_altitude_m);
            }
        }

        // All UKFs share the same R_radar (diagonal, built from radar_cfg_).
        // Build it directly from config so it's available even on the first
        // frame when filters_ is still empty.
        ObjectUKF::RadarMeasMat R_radar = ObjectUKF::RadarMeasMat::Zero();
        R_radar(0, 0)                   = radar_cfg_.range_std_m * radar_cfg_.range_std_m;
        R_radar(1, 1)                   = radar_cfg_.azimuth_std_rad * radar_cfg_.azimuth_std_rad;
        R_radar(2, 2) = radar_cfg_.elevation_std_rad * radar_cfg_.elevation_std_rad;
        R_radar(3, 3) = radar_cfg_.velocity_std_mps * radar_cfg_.velocity_std_mps;

        radar_llt.compute(R_radar);
        radar_llt_ok = (radar_llt.info() == Eigen::Success);
        if (!radar_llt_ok) {
            // Precompute diagonal inverse fallback
            const auto& diag = R_radar.diagonal();
            for (int d = 0; d < ObjectUKF::RADAR_MEAS_DIM; ++d) {
                if (diag(d) > 0.0f) R_diag_inv(d) = 1.0f / diag(d);
            }
        }
    }

    const CameraIntrinsics cam_intr{calib_.camera_intrinsics(0, 0), calib_.camera_intrinsics(1, 1),
                                    calib_.camera_intrinsics(0, 2), calib_.camera_intrinsics(1, 2)};

    for (const auto& trk : tracked.objects) {
        auto [depth, depth_conf] = estimate_depth(trk);
        bool radar_init_used     = false;

        auto it      = filters_.find(trk.track_id);
        bool adopted = false;
        if (it == filters_.end()) {
            // Phase D5: Camera adoption — check if a radar-only track exists at
            // a compatible bearing before creating a new filter.  This prevents
            // duplicate tracks when camera first sees a radar-detected obstacle.
            if (radar_enabled_) {
                const float bearing_x = (trk.position_2d.x() - cam_intr.cx) /
                                        std::max(1.0f, cam_intr.fx);
                const float bearing_y = (trk.position_2d.y() - cam_intr.cy) /
                                        std::max(1.0f, cam_intr.fy);

                uint32_t        best_radar_tid    = 0;
                float           best_range_err    = radar_cfg_.radar_adopt_gate_m;
                constexpr float kBearingAdoptGate = 0.2f;  // ~11.5° bearing tolerance

                for (const auto& [tid, ukf] : filters_) {
                    if (tid < 0x80000000u || !ukf.radar_only) continue;
                    const auto  pos          = ukf.position();
                    const float filter_depth = pos(0);
                    if (filter_depth < 1.0f) continue;

                    const float fb_x        = pos(1) / filter_depth;  // lateral/depth
                    const float fb_y        = pos(2) / filter_depth;  // vertical/depth
                    const float bearing_err = std::sqrt((bearing_x - fb_x) * (bearing_x - fb_x) +
                                                        (bearing_y - fb_y) * (bearing_y - fb_y));
                    const float range_err   = std::abs(depth - pos.norm());

                    if (bearing_err < kBearingAdoptGate && range_err < best_range_err) {
                        best_range_err = range_err;
                        best_radar_tid = tid;
                    }
                }

                if (best_radar_tid != 0) {
                    // Adopt: move radar-only filter to camera track ID
                    auto node    = filters_.extract(best_radar_tid);
                    node.key()   = trk.track_id;
                    auto& ukf    = node.mapped();
                    ukf.track_id = trk.track_id;
                    ukf.adopt_camera(trk, cam_intr);
                    filters_.insert(std::move(node));
                    it = filters_.find(trk.track_id);

                    // Transfer dormant mapping
                    auto dit = track_to_dormant_.find(best_radar_tid);
                    if (dit != track_to_dormant_.end()) {
                        track_to_dormant_[trk.track_id] = dit->second;
                        track_to_dormant_.erase(dit);
                    }
                    adopted         = true;
                    radar_init_used = true;  // treat as radar-confirmed for dormant
                    spdlog::info("[UKF] Camera adopt: track {} ← radar-only {:#x}", trk.track_id,
                                 best_radar_tid);
                }
            }

            if (!adopted) {
                // Radar-initialized depth: for new tracks, check if an unmatched
                // radar detection exists at a similar bearing.  Radar range is far
                // more accurate than monocular depth estimation, so prefer it.
                // Match on azimuth/elevation only (bearing-only gate).
                if (has_radar_data_ && radar_enabled_) {
                    const float bearing_x = (trk.position_2d.x() - cam_intr.cx) /
                                            std::max(1.0f, cam_intr.fx);
                    const float bearing_y = (trk.position_2d.y() - cam_intr.cy) /
                                            std::max(1.0f, cam_intr.fy);
                    // Camera bearing → predicted azimuth/elevation in body frame (FRD)
                    const float cam_az = std::atan2(bearing_x, 1.0f);  // atan2(right, forward)
                    const float cam_el = std::atan2(bearing_y,
                                                    std::sqrt(1.0f + bearing_x * bearing_x));

                    constexpr float kBearingGateRad  = 0.15f;  // ~8.6° bearing tolerance
                    float           best_range       = -1.0f;
                    float           best_bearing_err = kBearingGateRad;
                    uint32_t        best_radar_idx   = 0;

                    for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
                        if (radar_matched[ri]) continue;
                        const auto& rdet = radar_dets_.detections[ri];
                        // Negate azimuth: Gazebo FLU → UKF FRD
                        const float az_err    = std::abs(wrap_angle(-rdet.azimuth_rad - cam_az));
                        const float el_err    = std::abs(wrap_angle(rdet.elevation_rad - cam_el));
                        const float total_err = az_err + el_err;
                        if (az_err < kBearingGateRad && el_err < kBearingGateRad &&
                            total_err < best_bearing_err) {
                            best_bearing_err = total_err;
                            best_range       = rdet.range_m;
                            best_radar_idx   = ri;
                        }
                    }
                    if (best_range > 0.0f) {
                        radar_matched[best_radar_idx] = true;  // reserve detection
                        spdlog::info("[UKF] Radar-init: track {} cam_depth={:.1f}m → "
                                     "radar_range={:.1f}m",
                                     trk.track_id, depth, best_range);
                        depth           = best_range;
                        radar_init_used = true;
                    }
                }

                // New track — create UKF with calibrated camera intrinsics.
                filters_.emplace(trk.track_id, ObjectUKF(trk, depth, radar_cfg_, cam_intr));
                it = filters_.find(trk.track_id);

                // Adjust depth covariance based on depth source.
                // Radar range (±0.3m) → tight covariance.
                // Monocular depth → high uncertainty, especially at long range.
                if (radar_init_used) {
                    it->second.set_radar_confirmed_depth(depth);
                    it->second.depth_confidence = 1.0f;
                } else {
                    it->second.set_depth_covariance(100.0f);
                    it->second.depth_confidence = depth_conf;
                }

                // Dormant re-identification: only radar-confirmed tracks enter the
                // dormant pool. Camera-only tracks have wildly uncertain depth
                // (P(0,0)=100) and would pollute the pool with phantom positions.
                if (has_pose_ && radar_init_used) {
                    Eigen::Vector3f world_pos = body_to_world(it->second.position());
                    int didx = find_nearest_dormant(world_pos, it->second.position_covariance());
                    if (didx >= 0) {
                        // Re-identified — merge into existing dormant obstacle
                        auto& dobs = dormant_obstacles_[didx];
                        dobs.observations++;
                        const float alpha = 1.0f / static_cast<float>(dobs.observations);
                        dobs.world_pos    = (1.0f - alpha) * dobs.world_pos + alpha * world_pos;
                        dobs.last_seen_ns = tracked.timestamp_ns;
                        track_to_dormant_[trk.track_id] = didx;
                        spdlog::info("[UKF] Re-ID: track {} → dormant #{} at ({:.1f},{:.1f}), "
                                     "obs={}, dist={:.1f}m",
                                     trk.track_id, didx, dobs.world_pos.x(), dobs.world_pos.y(),
                                     dobs.observations,
                                     (world_pos.head<2>() - dobs.world_pos.head<2>()).norm());
                    } else {
                        // New obstacle — create dormant entry
                        if (static_cast<int>(dormant_obstacles_.size()) < max_dormant_) {
                            dormant_obstacles_.push_back({world_pos, 1, tracked.timestamp_ns});
                            track_to_dormant_[trk.track_id] =
                                static_cast<int>(dormant_obstacles_.size()) - 1;
                            spdlog::info("[UKF] New obstacle: track {} → dormant #{} at "
                                         "({:.1f},{:.1f})",
                                         trk.track_id,
                                         static_cast<int>(dormant_obstacles_.size()) - 1,
                                         world_pos.x(), world_pos.y());
                        }
                    }
                }
            }  // end if (!adopted)
        } else {
            // Existing track — update its dormant entry position if mapped.
            // Only update from radar-confirmed tracks to prevent camera-only
            // depth drift from corrupting dormant world positions.
            if (has_pose_ && it->second.radar_update_count > 0) {
                auto dit = track_to_dormant_.find(trk.track_id);
                if (dit != track_to_dormant_.end() && dit->second >= 0 &&
                    dit->second < static_cast<int>(dormant_obstacles_.size())) {
                    auto&           dobs      = dormant_obstacles_[dit->second];
                    Eigen::Vector3f world_pos = body_to_world(it->second.position());
                    dobs.observations++;
                    const float alpha = 1.0f / static_cast<float>(dobs.observations);
                    dobs.world_pos    = (1.0f - alpha) * dobs.world_pos + alpha * world_pos;
                    dobs.last_seen_ns = tracked.timestamp_ns;
                }
            }
        }

        auto& ukf = it->second;
        ukf.predict();

        // Keep radar-confirmed range authoritative: once radar has confirmed a
        // track, feed the current filter depth back instead of a fresh monocular
        // estimate, which can pull the filter away from the radar range.
        const float camera_update_depth = (ukf.radar_update_count > 0) ? ukf.position()(0) : depth;
        ukf.update_camera(trk, camera_update_depth, cam_intr);

        // Update depth confidence.  For camera-only tracks, use the current
        // estimate directly so that a brief high-confidence observation cannot
        // permanently "unlock" promotion.  For radar-confirmed tracks, preserve
        // the radar confidence (1.0) until another radar update refreshes it.
        if (ukf.radar_update_count == 0) {
            ukf.depth_confidence = depth_conf;
        }

        // Radar association: find best matching radar detection within gate
        bool matched_radar = false;
        if (has_radar_data_ && radar_dets_.num_detections > 0) {
            auto z_pred = ukf.predicted_radar_measurement();

            float    best_dist = std::numeric_limits<float>::max();
            uint32_t best_idx  = 0;

            for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
                if (radar_matched[ri]) continue;

                const auto& rdet = radar_dets_.detections[ri];

                // Coarse range gate — reject ~80-90% of pairs with a single
                // float comparison before the expensive Mahalanobis computation.
                const float range_diff = std::abs(rdet.range_m - z_pred(0));
                if (range_diff > range_3sigma) continue;

                // Altitude gate: reject if radar return body-frame z differs
                // too much from the track's estimated z position.
                {
                    const float radar_z = rdet.range_m * std::sin(rdet.elevation_rad);
                    const float track_z = ukf.position()(2);
                    if (std::abs(radar_z - track_z) > radar_cfg_.altitude_gate_m) continue;
                }

                // Negate azimuth: Gazebo FLU → UKF FRD convention
                ObjectUKF::RadarMeasVec z_actual;
                z_actual(0) = rdet.range_m;
                z_actual(1) = -rdet.azimuth_rad;
                z_actual(2) = rdet.elevation_rad;
                z_actual(3) = rdet.radial_velocity_mps;

                // Innovation (residual) with angle wrapping for azimuth/elevation
                ObjectUKF::RadarMeasVec innovation = z_actual - z_pred;
                innovation(1)                      = wrap_angle(innovation(1));
                innovation(2)                      = wrap_angle(innovation(2));

                // Mahalanobis distance: d² = innovationᵀ * R_radar⁻¹ * innovation
                // Uses pre-computed Cholesky (hoisted outside track loop).
                float mahal_sq = std::numeric_limits<float>::max();
                if (radar_llt_ok) {
                    ObjectUKF::RadarMeasVec y = radar_llt.solve(innovation);
                    mahal_sq                  = innovation.dot(y);
                } else {
                    // Fallback: diagonal-only (uses pre-computed inverse)
                    mahal_sq = 0.0f;
                    for (int d = 0; d < ObjectUKF::RADAR_MEAS_DIM; ++d) {
                        mahal_sq += innovation(d) * innovation(d) * R_diag_inv(d);
                    }
                }

                if (mahal_sq < radar_cfg_.gate_threshold && mahal_sq < best_dist) {
                    best_dist = mahal_sq;
                    best_idx  = ri;
                }
            }

            if (best_dist < radar_cfg_.gate_threshold) {
                ukf.update_radar(radar_dets_.detections[best_idx]);
                radar_matched[best_idx] = true;
                matched_radar           = true;
                ukf.depth_confidence    = 1.0f;
            }
        }

        // Late dormant entry: if an existing track just got its first radar
        // update, it now has reliable depth — create/re-ID a dormant entry.
        if (has_pose_ && matched_radar && ukf.radar_update_count == 1 &&
            track_to_dormant_.find(trk.track_id) == track_to_dormant_.end()) {
            Eigen::Vector3f world_pos = body_to_world(ukf.position());
            int             didx      = find_nearest_dormant(world_pos, ukf.position_covariance());
            if (didx >= 0) {
                auto& dobs = dormant_obstacles_[didx];
                dobs.observations++;
                const float alpha = 1.0f / static_cast<float>(dobs.observations);
                dobs.world_pos    = (1.0f - alpha) * dobs.world_pos + alpha * world_pos;
                dobs.last_seen_ns = tracked.timestamp_ns;
                track_to_dormant_[trk.track_id] = didx;
                spdlog::info("[UKF] Re-ID (late radar): track {} → dormant #{} at "
                             "({:.1f},{:.1f}), obs={}, dist={:.1f}m",
                             trk.track_id, didx, dobs.world_pos.x(), dobs.world_pos.y(),
                             dobs.observations,
                             (world_pos.head<2>() - dobs.world_pos.head<2>()).norm());
            } else if (static_cast<int>(dormant_obstacles_.size()) < max_dormant_) {
                dormant_obstacles_.push_back({world_pos, 1, tracked.timestamp_ns});
                track_to_dormant_[trk.track_id] = static_cast<int>(dormant_obstacles_.size()) - 1;
                spdlog::info("[UKF] New obstacle (late radar): track {} → dormant #{} "
                             "at ({:.1f},{:.1f})",
                             trk.track_id, static_cast<int>(dormant_obstacles_.size()) - 1,
                             world_pos.x(), world_pos.y());
            }
        }

        seen[trk.track_id] = true;

        // Build output
        FusedObject fused;
        fused.track_id            = trk.track_id;
        fused.class_id            = trk.class_id;
        fused.confidence          = trk.confidence;
        fused.position_3d         = ukf.position();
        fused.velocity_3d         = ukf.velocity();
        fused.heading             = 0.0f;
        fused.has_camera          = true;
        fused.has_radar           = matched_radar || radar_init_used;
        fused.position_covariance = ukf.position_covariance();
        fused.timestamp_ns        = trk.timestamp_ns;
        fused.radar_update_count  = ukf.radar_update_count;
        fused.depth_confidence    = ukf.depth_confidence;

        // Size estimation (A2): back-project camera bbox using UKF range.
        // Only meaningful when range is reliable (radar-confirmed or close range).
        const float range = ukf.position().norm();
        if (range > 1.0f && trk.bbox_w > 5.0f && trk.bbox_h > 5.0f) {
            fused.estimated_radius_m = (trk.bbox_w * range) / (2.0f * cam_intr.fx);
            fused.estimated_height_m = (trk.bbox_h * range) / cam_intr.fy;
        }

        // If this track is re-identified with a dormant obstacle, output the
        // averaged world-frame position instead of the raw UKF body-frame estimate.
        // This produces stable, consistent positions across viewing angles.
        auto dit = track_to_dormant_.find(trk.track_id);
        if (has_pose_ && dit != track_to_dormant_.end() && dit->second >= 0 &&
            dit->second < static_cast<int>(dormant_obstacles_.size()) &&
            dormant_obstacles_[dit->second].observations > 1) {
            fused.position_3d    = dormant_obstacles_[dit->second].world_pos;
            fused.in_world_frame = true;
        }

        output.objects.push_back(fused);
    }

    // ── Phase D4: Radar-only track re-association ──────────────
    // Existing radar-only filters from previous frames need radar updates.
    // They were not iterated in the camera-track loop above.
    if (has_radar_data_ && radar_enabled_) {
        for (auto& [tid, ukf] : filters_) {
            if (tid < 0x80000000u) continue;     // skip camera-originated tracks
            if (seen.count(tid) != 0) continue;  // already processed (adopted or new orphan)

            ukf.predict();
            bool matched = try_associate_radar(ukf, radar_matched, radar_llt, radar_llt_ok,
                                               R_diag_inv, range_3sigma);
            if (matched) {
                seen[tid] = true;

                // Update dormant position
                if (has_pose_ && ukf.radar_update_count > 0) {
                    auto dit = track_to_dormant_.find(tid);
                    if (dit != track_to_dormant_.end() && dit->second >= 0 &&
                        dit->second < static_cast<int>(dormant_obstacles_.size())) {
                        auto&           dobs      = dormant_obstacles_[dit->second];
                        Eigen::Vector3f world_pos = body_to_world(ukf.position());
                        dobs.observations++;
                        const float alpha = 1.0f / static_cast<float>(dobs.observations);
                        dobs.world_pos    = (1.0f - alpha) * dobs.world_pos + alpha * world_pos;
                        dobs.last_seen_ns = output.timestamp_ns;
                    }
                }

                // Gate output: only emit after min_hits observations
                if (ukf.radar_update_count <
                    static_cast<uint32_t>(radar_cfg_.radar_orphan_min_hits))
                    continue;

                // Build output for re-associated radar-only track
                FusedObject fused;
                fused.track_id            = tid;
                fused.class_id            = ObjectClass::UNKNOWN;
                fused.confidence          = 0.8f;  // radar re-association confidence
                fused.position_3d         = ukf.position();
                fused.velocity_3d         = ukf.velocity();
                fused.heading             = 0.0f;
                fused.has_camera          = false;
                fused.has_radar           = true;
                fused.position_covariance = ukf.position_covariance();
                fused.timestamp_ns        = output.timestamp_ns;
                fused.radar_update_count  = ukf.radar_update_count;
                fused.depth_confidence    = 1.0f;  // radar-only → authoritative range
                fused.estimated_radius_m  = radar_cfg_.radar_only_default_radius_m;
                fused.estimated_height_m  = 0.0f;

                auto dit = track_to_dormant_.find(tid);
                if (has_pose_ && dit != track_to_dormant_.end() && dit->second >= 0 &&
                    dit->second < static_cast<int>(dormant_obstacles_.size()) &&
                    dormant_obstacles_[dit->second].observations > 1) {
                    fused.position_3d    = dormant_obstacles_[dit->second].world_pos;
                    fused.in_world_frame = true;
                }

                output.objects.push_back(fused);
            }
            // If not matched, the track will be removed by stale cleanup below.
        }
    }

    // ── Phase D3: Radar-orphan processing ─────────────────────
    // Create new tracks from unmatched radar detections that are far from
    // any existing filter (camera or radar-only).  This enables detection
    // of obstacles outside camera FOV.
    if (has_radar_data_ && radar_enabled_) {
        for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
            if (radar_matched[ri]) continue;
            const auto& rdet = radar_dets_.detections[ri];

            // Quality gates for orphan track creation:
            // 1. Range sanity — ignore detections beyond reliable radar range.
            //    Typical 77GHz drone radar: ≤50m. Default 40m is conservative.
            if (rdet.range_m > radar_cfg_.radar_max_orphan_range_m) continue;

            // 2. Ground-plane filter (redundant safety net) — applied even when
            //    the main ground filter is disabled, to prevent phantom ground tracks.
            //    No altitude gate: flying objects (drones, birds) may be at any altitude.
            if (has_altitude_) {
                const float object_alt = drone_altitude_m_ +
                                         rdet.range_m * std::sin(rdet.elevation_rad);
                if (object_alt < radar_cfg_.min_object_altitude_m) continue;
            }

            // Convert to body-frame for proximity check
            const float           neg_az = -rdet.azimuth_rad;
            const float           cos_el = std::cos(rdet.elevation_rad);
            const float           body_x = rdet.range_m * cos_el * std::cos(neg_az);
            const float           body_y = rdet.range_m * cos_el * std::sin(neg_az);
            const float           body_z = rdet.range_m * std::sin(rdet.elevation_rad);
            const Eigen::Vector3f body_pos(body_x, body_y, body_z);

            // Proximity check: skip if too close to any existing filter
            bool too_close = false;
            for (const auto& [tid, ukf] : filters_) {
                if ((ukf.position() - body_pos).norm() < radar_cfg_.radar_orphan_proximity_m) {
                    too_close = true;
                    break;
                }
            }
            if (too_close) continue;

            // Create new radar-only track
            const uint32_t rid = next_radar_track_id_++;
            filters_.emplace(rid, ObjectUKF(rdet, rid, radar_cfg_));
            auto& ukf = filters_.at(rid);

            // Dormant entry — radar has accurate world position
            if (has_pose_) {
                Eigen::Vector3f world_pos = body_to_world(ukf.position());
                int             didx = find_nearest_dormant(world_pos, ukf.position_covariance());
                if (didx >= 0) {
                    auto& dobs = dormant_obstacles_[didx];
                    dobs.observations++;
                    const float alpha      = 1.0f / static_cast<float>(dobs.observations);
                    dobs.world_pos         = (1.0f - alpha) * dobs.world_pos + alpha * world_pos;
                    dobs.last_seen_ns      = output.timestamp_ns;
                    track_to_dormant_[rid] = didx;
                    spdlog::info("[UKF] Re-ID (radar-only): {:#x} → dormant #{} at ({:.1f},{:.1f})",
                                 rid, didx, dobs.world_pos.x(), dobs.world_pos.y());
                } else if (static_cast<int>(dormant_obstacles_.size()) < max_dormant_) {
                    dormant_obstacles_.push_back({world_pos, 1, output.timestamp_ns});
                    track_to_dormant_[rid] = static_cast<int>(dormant_obstacles_.size()) - 1;
                    spdlog::info("[UKF] New obstacle (radar-only): {:#x} → dormant #{} at "
                                 "({:.1f},{:.1f})",
                                 rid, static_cast<int>(dormant_obstacles_.size()) - 1,
                                 world_pos.x(), world_pos.y());
                }
            }

            seen[rid] = true;

            // Gate output: only emit radar-only tracks after min_hits observations.
            // The filter still exists (accumulates future radar hits), but the grid
            // won't see it until it has enough observations to be trustworthy.
            if (ukf.radar_update_count < static_cast<uint32_t>(radar_cfg_.radar_orphan_min_hits))
                continue;

            // Build FusedObject output
            FusedObject fused;
            fused.track_id            = rid;
            fused.class_id            = ObjectClass::UNKNOWN;
            fused.confidence          = rdet.confidence;
            fused.position_3d         = ukf.position();
            fused.velocity_3d         = ukf.velocity();
            fused.heading             = 0.0f;
            fused.has_camera          = false;
            fused.has_radar           = true;
            fused.position_covariance = ukf.position_covariance();
            fused.timestamp_ns        = output.timestamp_ns;
            fused.radar_update_count  = ukf.radar_update_count;
            fused.depth_confidence    = 1.0f;  // radar-only → authoritative range
            fused.estimated_radius_m  = radar_cfg_.radar_only_default_radius_m;
            fused.estimated_height_m  = 0.0f;

            auto dit = track_to_dormant_.find(rid);
            if (has_pose_ && dit != track_to_dormant_.end() && dit->second >= 0 &&
                dit->second < static_cast<int>(dormant_obstacles_.size()) &&
                dormant_obstacles_[dit->second].observations > 1) {
                fused.position_3d    = dormant_obstacles_[dit->second].world_pos;
                fused.in_world_frame = true;
            }

            output.objects.push_back(fused);
        }
    }

    // Remove stale filters (not seen this frame).
    // Their dormant entries persist — only the active UKF filter is removed.
    for (auto it = filters_.begin(); it != filters_.end();) {
        if (seen.find(it->first) == seen.end()) {
            track_to_dormant_.erase(it->first);
            it = filters_.erase(it);
        } else {
            ++it;
        }
    }

    // Clear radar data after use
    has_radar_data_ = false;

    return output;
}

// ═══════════════════════════════════════════════════════════
// Fusion engine factory
// ═══════════════════════════════════════════════════════════
std::unique_ptr<IFusionEngine> create_fusion_engine(const std::string&     backend,
                                                    const CalibrationData& calib,
                                                    const drone::Config*   cfg) {
    if (backend == "camera_only") {
        DepthEstimationConfig depth_cfg;
        if (cfg) {
            depth_cfg.covariance_init  = cfg->get<float>("perception.fusion.depth.covariance_init",
                                                         depth_cfg.covariance_init);
            depth_cfg.bbox_h_threshold = cfg->get<float>("perception.fusion.depth.bbox_h_threshold",
                                                         depth_cfg.bbox_h_threshold);
            depth_cfg.depth_min_m      = cfg->get<float>("perception.fusion.depth.depth_min_m",
                                                         depth_cfg.depth_min_m);
            depth_cfg.depth_max_m      = cfg->get<float>("perception.fusion.depth.depth_max_m",
                                                         depth_cfg.depth_max_m);
            depth_cfg.ray_down_threshold = cfg->get<float>(
                "perception.fusion.depth.ray_down_threshold", depth_cfg.ray_down_threshold);
            depth_cfg.fallback_depth_m = cfg->get<float>("perception.fusion.depth.fallback_depth_m",
                                                         depth_cfg.fallback_depth_m);
        }
        return std::make_unique<CameraOnlyFusionEngine>(calib, depth_cfg);
    }
    if (backend == "ukf") {
        RadarNoiseConfig radar_cfg;
        bool             radar_enabled = false;
        if (cfg) {
            radar_enabled               = cfg->get<bool>("perception.fusion.radar.enabled", false);
            radar_cfg.range_std_m       = cfg->get<float>("perception.fusion.radar.range_std_m",
                                                          radar_cfg.range_std_m);
            radar_cfg.azimuth_std_rad   = cfg->get<float>("perception.fusion.radar.azimuth_std_rad",
                                                          radar_cfg.azimuth_std_rad);
            radar_cfg.elevation_std_rad = cfg->get<float>(
                "perception.fusion.radar.elevation_std_rad", radar_cfg.elevation_std_rad);
            radar_cfg.velocity_std_mps = cfg->get<float>("perception.fusion.radar.velocity_std_mps",
                                                         radar_cfg.velocity_std_mps);
            radar_cfg.gate_threshold   = cfg->get<float>("perception.fusion.radar.gate_threshold",
                                                         radar_cfg.gate_threshold);
            radar_cfg.ground_filter_enabled = cfg->get<bool>(
                "perception.fusion.radar.ground_filter_enabled", radar_cfg.ground_filter_enabled);
            radar_cfg.min_object_altitude_m = cfg->get<float>(
                "perception.fusion.radar.min_object_altitude_m", radar_cfg.min_object_altitude_m);
            radar_cfg.altitude_gate_m = cfg->get<float>("perception.fusion.radar.altitude_gate_m",
                                                        radar_cfg.altitude_gate_m);
            radar_cfg.radar_orphan_proximity_m = cfg->get<float>(
                "perception.fusion.radar.orphan_proximity_m", radar_cfg.radar_orphan_proximity_m);
            radar_cfg.radar_adopt_gate_m = cfg->get<float>("perception.fusion.radar.adopt_gate_m",
                                                           radar_cfg.radar_adopt_gate_m);
            radar_cfg.radar_only_default_radius_m = cfg->get<float>(
                "perception.fusion.radar.default_radius_m", radar_cfg.radar_only_default_radius_m);
            radar_cfg.radar_max_orphan_range_m = cfg->get<float>(
                "perception.fusion.radar.max_orphan_range_m", radar_cfg.radar_max_orphan_range_m);
            radar_cfg.radar_orphan_min_hits = cfg->get<int>(
                "perception.fusion.radar.orphan_min_hits", radar_cfg.radar_orphan_min_hits);
            radar_cfg.radar_only_promotion_hits = static_cast<uint32_t>(
                cfg->get<int>("perception.fusion.radar.promotion_hits",
                              static_cast<int>(radar_cfg.radar_only_promotion_hits)));
        }
        float dormant_merge_radius =
            cfg ? cfg->get<float>("perception.fusion.dormant_merge_radius_m", 5.0f) : 5.0f;
        int max_dormant = cfg ? cfg->get<int>("perception.fusion.max_dormant", 32) : 32;
        return std::make_unique<UKFFusionEngine>(calib, radar_cfg, radar_enabled,
                                                 dormant_merge_radius, max_dormant);
    }
    throw std::invalid_argument("Unknown fusion engine backend: " + backend);
}

}  // namespace drone::perception
