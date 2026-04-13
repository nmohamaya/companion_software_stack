// process2_perception/src/ukf_fusion_engine.cpp
// UKF fusion engine implementation — camera + radar measurement models.
// Phase 1C (Issue #114), radar fusion (Issue #210).
#include "perception/ukf_fusion_engine.h"

#include "perception/fusion_engine.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace drone::perception {

/// Shared process noise initialization for constant-velocity UKF model.
static ObjectUKF::StateMat make_process_noise() {
    ObjectUKF::StateMat Q = ObjectUKF::StateMat::Identity() * 0.5f;
    Q(0, 0)               = 0.1f;
    Q(1, 1)               = 0.1f;
    Q(2, 2)               = 0.05f;
    Q(3, 3)               = 1.0f;
    Q(4, 4)               = 1.0f;
    Q(5, 5)               = 0.5f;
    return Q;
}

// ═══════════════════════════════════════════════════════════
// ObjectUKF
// ═══════════════════════════════════════════════════════════
ObjectUKF::ObjectUKF(const TrackedObject& trk, float initial_depth,
                     const RadarNoiseConfig& radar_cfg, const CameraIntrinsics& intrinsics)
    : track_id(trk.track_id), age(0), negate_azimuth_(radar_cfg.negate_azimuth) {
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

    Q_ = make_process_noise();

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
    : track_id(id)
    , age(0)
    , radar_update_count(1)
    , radar_only(true)
    , negate_azimuth_(radar_cfg.negate_azimuth) {
    // Convert spherical radar (range, azimuth, elevation) to body-frame FRD.
    // Sign convention configurable per backend (Issue #348).
    const float az     = radar_cfg.az_sign(det.azimuth_rad);
    const float cos_el = std::cos(det.elevation_rad);
    const float sin_el = std::sin(det.elevation_rad);

    x_    = StateVec::Zero();
    x_(0) = det.range_m * cos_el * std::cos(az);  // x (forward / depth)
    x_(1) = det.range_m * cos_el * std::sin(az);  // y (right / lateral)
    x_(2) = det.range_m * sin_el;                 // z (down / vertical)
    // Project radial velocity along look direction (only component observable)
    x_(3) = det.radial_velocity_mps * cos_el * std::cos(az);  // vx
    x_(4) = 0.0f;                                             // vy
    x_(5) = 0.0f;                                             // vz

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

    Q_ = make_process_noise();

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

ObjectUKF::RadarPrediction ObjectUKF::compute_radar_prediction(
    const std::vector<StateVec>& sigma) const {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;

    // Propagate sigma points through radar measurement model
    std::vector<RadarMeasVec> z_sigma(2 * n + 1);
    for (int i = 0; i < 2 * n + 1; ++i) {
        z_sigma[i] = radar_measurement_model(sigma[i]);
    }

    // Weights
    const float w0_mean = lambda / (n + lambda);
    const float w0_cov  = w0_mean + (1.0f - kAlpha * kAlpha + kBeta);
    const float wi      = 1.0f / (2.0f * (n + lambda));

    // Mean predicted measurement (circular mean for angles)
    float        sin_az_sum = 0.0f, cos_az_sum = 0.0f;
    float        sin_el_sum = 0.0f, cos_el_sum = 0.0f;
    RadarMeasVec z_mean = RadarMeasVec::Zero();

    z_mean(0) += w0_mean * z_sigma[0](0);
    z_mean(3) += w0_mean * z_sigma[0](3);
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

    // Innovation covariance S = Pzz + R_radar
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
    return {z_mean, S, std::move(z_sigma)};
}

ObjectUKF::RadarPrediction ObjectUKF::predicted_radar_innovation_cov() const {
    auto sigma = generate_sigma_points();
    return compute_radar_prediction(sigma);
}

void ObjectUKF::update_radar(const drone::ipc::RadarDetection& det) {
    constexpr int   n      = STATE_DIM;
    constexpr float lambda = kAlpha * kAlpha * (n + kKappa) - n;

    auto sigma = generate_sigma_points();

    // Compute sigma-point radar prediction via shared helper (z_mean, S, and z_sigma).
    // z_sigma is reused below for cross-covariance — avoids redundant propagation.
    auto [z_mean, S, z_sigma] = compute_radar_prediction(sigma);

    // Weights
    const float w0_cov = lambda / (n + lambda) + (1.0f - kAlpha * kAlpha + kBeta);
    const float wi     = 1.0f / (2.0f * (n + lambda));

    // Cross-covariance Pxz (with angle-wrapped residuals)
    using RadarCrossMat = Eigen::Matrix<float, STATE_DIM, RADAR_MEAS_DIM>;
    RadarMeasVec dz0    = z_sigma[0] - z_mean;
    dz0(1)              = wrap_angle(dz0(1));
    dz0(2)              = wrap_angle(dz0(2));
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
    // Azimuth sign convention configurable per backend (Issue #348).
    RadarMeasVec z_actual;
    z_actual(0) = det.range_m;
    z_actual(1) = negate_azimuth_ ? -det.azimuth_rad : det.azimuth_rad;
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

/// EMA smoothing constants for radar-learned object height (#422, review fix #8).
/// α=0.2 for new measurements, (1-α)=0.8 retained. Prevents noisy bbox from
/// corrupting the height estimate while still adapting to the true value.
static constexpr float kHeightEmaRetain = 0.8f;
static constexpr float kHeightEmaNew    = 0.2f;

/// Maximum sane learned height [m] — rejects spoofed/distant radar (review fix #4).
static constexpr float kMaxLearnedHeightM = 25.0f;
static constexpr float kMinLearnedHeightM = 0.1f;

UKFFusionEngine::UKFFusionEngine(const CalibrationData& calib, const RadarNoiseConfig& radar_cfg,
                                 bool radar_enabled, float dormant_merge_radius_m, int max_dormant)
    : calib_(calib)
    , radar_cfg_(radar_cfg)
    , radar_enabled_(radar_enabled)
    , dormant_merge_radius_m_(dormant_merge_radius_m)
    , max_dormant_(max_dormant) {
    DRONE_LOG_INFO("[UKF] radar_enabled={}, negate_azimuth={}, ground_filter_enabled={}, "
                   "ground_filter_alt_m={:.2f}, altitude_gate_m={:.1f}, gate_threshold={:.1f}, "
                   "dormant_merge_radius={:.1f}m, max_dormant={}",
                   radar_enabled_, radar_cfg_.negate_azimuth, radar_cfg_.ground_filter_enabled,
                   radar_cfg_.min_object_altitude_m, radar_cfg_.altitude_gate_m,
                   radar_cfg_.gate_threshold, dormant_merge_radius_m_, max_dormant_);
}

/// Compute depth confidence from pinhole projection uncertainty propagation.
/// sigma_depth² = (dD/d(measurement))² × sigma_bbox²; conf = 1/(1 + sigma_depth²).
/// Returns 0.2 if raw depth exceeded kDepthMaxM (model saturated, Issue #340).
static float depth_confidence_from_covariance(float derivative, float sigma_bbox, float d_raw,
                                              float depth_max) {
    if (d_raw > depth_max) return 0.2f;
    const float sigma_depth_sq = derivative * derivative * sigma_bbox * sigma_bbox;
    return 1.0f / (1.0f + sigma_depth_sq);
}

UKFFusionEngine::DepthEstimate UKFFusionEngine::estimate_depth(const TrackedObject& trk,
                                                               float height_override) const {
    const float fy = calib_.camera_intrinsics(1, 1);
    const float cy = calib_.camera_intrinsics(1, 2);

    // Four-tier depth estimation with covariance-based confidence (#420):
    //
    // Tiers 1 & 2 use pinhole projection uncertainty propagation:
    //   sigma_depth^2 = (dD/d(bbox_h))^2 * sigma_bbox^2
    //   confidence    = 1 / (1 + sigma_depth^2)
    // Close objects (large bbox_h) → small derivative → high confidence.
    // Far objects (small bbox_h) → large derivative → low confidence.
    //
    // 1. Horizon-truncated bbox: bbox top at/above horizon,
    //    use ground-plane geometry from bbox bottom.
    //
    // 2. Apparent-size: depth = assumed_height * fy / bbox_h
    //    Most reliable camera-only method when full height visible.
    //
    // 3. Ground-plane fallback (conf=0.3): depth = camera_height / ray_down
    //    Weak signal from small detections — often ground features.
    //
    // 4. Near-horizon fallback (conf=0.01): conservative 8m estimate.
    //    Pure guess with no geometric basis.
    //
    // Confidence values gate downstream promotion: only detections with
    // confidence >= min_promotion_depth_confidence become permanent static
    // cells.  Low-confidence detections still create temporary dynamic cells.
    //
    // Tier 1/2 clamp penalty: if a Tier 1 or Tier 2 raw depth exceeds
    // kDepthMaxM (40m), the model has saturated (common for ground features
    // at shallow angles). Confidence drops to 0.2 to block promotion.
    // Issue #340.
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
            const float d_raw = drone_altitude_m_ * fy / (bbox_bottom - cy) * ds;
            const float d     = std::clamp(d_raw, kDepthMinM, kDepthMaxM);
            // Covariance-based confidence (#420): dD/d(bbox_bottom) = H*fy*ds/(bbox_bottom-cy)²
            const float denom    = (bbox_bottom - cy) * (bbox_bottom - cy);
            const float dH_dbbox = drone_altitude_m_ * fy * ds / std::max(denom, 1.0f);
            const float c = depth_confidence_from_covariance(dH_dbbox, calib_.bbox_height_noise_px,
                                                             d_raw, kDepthMaxM);
            return {d, c};
        }
        if (ray_down_base > kRayDownMinThres) {
            const float d_raw = calib_.camera_height_m / ray_down_base * ds;
            const float d     = std::clamp(d_raw, kDepthMinM, kDepthMaxM);
            // Covariance-based confidence (#420): dD/d(bbox_bottom) = cam_H*fy*ds/(bbox_bottom-cy)²
            const float denom    = (bbox_bottom - cy) * (bbox_bottom - cy);
            const float dH_dbbox = calib_.camera_height_m * fy * ds / std::max(denom, 1.0f);
            const float c = depth_confidence_from_covariance(dH_dbbox, calib_.bbox_height_noise_px,
                                                             d_raw, kDepthMaxM);
            return {d, c};
        }
    }

    // Tier 2: Full-height apparent-size depth (bbox fully visible)
    if (trk.bbox_h > kBboxHThreshold) {
        // Height precedence (#422): radar-learned > class prior > UNKNOWN fallback.
        // Radar-learned height is passed via height_override when available.
        const auto  class_idx = static_cast<uint8_t>(trk.class_id);
        const float class_h   = (class_idx < drone::perception::kNumObjectClasses)
                                    ? calib_.height_priors[class_idx]
                                    : calib_.height_priors[0];
        const float assumed_h = (height_override > 0.0f) ? height_override : class_h;
        const float d_raw     = assumed_h * fy / trk.bbox_h * ds;
        const float d         = std::clamp(d_raw, kDepthMinM, kDepthMaxM);
        // Covariance-based confidence (#420): dD/d(bbox_h) = H*fy*ds/bbox_h²
        const float dD_dbbox = assumed_h * fy * ds / (trk.bbox_h * trk.bbox_h);
        const float c = depth_confidence_from_covariance(dD_dbbox, calib_.bbox_height_noise_px,
                                                         d_raw, kDepthMaxM);
        return {d, c};
    }

    // Tier 3: Ground-plane fallback for small detections
    const float ray_down = (trk.position_2d.y() - cy) / std::max(1.0f, fy);
    if (ray_down > kRayDownMinThres) {
        return {std::clamp(calib_.camera_height_m / ray_down * ds, kDepthMinM, kDepthMaxM), 0.3f};
    }

    // Tier 4: Near-horizon conservative estimate — no geometric basis.
    // Use 0.01 (not 0.0) so the P2 camera-only fallback doesn't override
    // this with detection confidence (which would defeat the promotion gate).
    return {kFallbackDepthM * ds, 0.01f};
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

    // For radar-only tracks, use proper innovation covariance S = Pzz + R
    // that accounts for state uncertainty after predict(). Camera-originated
    // tracks have tight covariance, so R-only gating is sufficient.
    const bool use_full_S = ukf.radar_only;

    Eigen::LLT<ObjectUKF::RadarMeasMat> track_llt;
    bool                                track_llt_ok       = false;
    float                               track_range_3sigma = range_3sigma;

    ObjectUKF::RadarMeasVec track_z_mean = ObjectUKF::RadarMeasVec::Zero();

    if (use_full_S) {
        auto  pred      = ukf.predicted_radar_innovation_cov();
        auto& S         = pred.S;
        auto  z_mean_sp = pred.z_mean;
        track_z_mean    = z_mean_sp;
        track_llt.compute(S);
        track_llt_ok = (track_llt.info() == Eigen::Success);
        if (!track_llt_ok) {
            // Regularize S by adding small epsilon to diagonal and retry LLT.
            constexpr float kEpsilon = 1e-6f;
            S.diagonal().array() += kEpsilon;
            track_llt.compute(S);
            track_llt_ok = (track_llt.info() == Eigen::Success);
        }
        if (!track_llt_ok) {
            // S is not positive-definite even after regularization — treat as
            // association failure (return no-match) rather than using a permissive
            // diagonal approximation that could underestimate Mahalanobis distance.
            DRONE_LOG_WARN("[UKF] S-matrix LLT failed for radar-only track {:#x}, skipping "
                           "association",
                           ukf.track_id);
            return false;
        }
        // Widen coarse range gate to 3σ of S(0,0) (range innovation variance).
        // Guard against non-positive diagonal (numerical edge case).
        if (S(0, 0) > 0.0f) {
            track_range_3sigma = std::max(range_3sigma, 3.0f * std::sqrt(S(0, 0)));
        }
    }

    for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
        if (radar_matched[ri]) continue;
        const auto& rdet = radar_dets_.detections[ri];

        // Coarse range gate — use z_mean for radar-only tracks (consistent with S-matrix),
        // z_pred for camera-originated tracks (consistent with R-only gating).
        const float predicted_range = use_full_S ? track_z_mean(0) : z_pred(0);
        if (std::abs(rdet.range_m - predicted_range) > track_range_3sigma) continue;

        // Altitude gate (wider for radar-only tracks with uncertain z)
        {
            const float radar_z  = rdet.range_m * std::sin(rdet.elevation_rad);
            const float track_z  = ukf.position()(2);
            const float alt_gate = use_full_S
                                       ? std::max(radar_cfg_.altitude_gate_m,
                                                  3.0f * std::sqrt(ukf.position_covariance()(2, 2)))
                                       : radar_cfg_.altitude_gate_m;
            if (std::abs(radar_z - track_z) > alt_gate) continue;
        }

        // Azimuth sign convention configurable per backend (Issue #348).
        ObjectUKF::RadarMeasVec z_actual;
        z_actual(0) = rdet.range_m;
        z_actual(1) = radar_cfg_.az_sign(rdet.azimuth_rad);
        z_actual(2) = rdet.elevation_rad;
        z_actual(3) = rdet.radial_velocity_mps;

        // For radar-only tracks (use_full_S), compute innovation relative to
        // the sigma-point mean z_mean — consistent with the S matrix built from
        // z_mean residuals.  Camera-originated tracks use z_pred (h(x_)) which
        // is consistent with R-only gating.
        ObjectUKF::RadarMeasVec innovation = use_full_S ? (z_actual - track_z_mean)
                                                        : (z_actual - z_pred);
        innovation(1)                      = wrap_angle(innovation(1));
        innovation(2)                      = wrap_angle(innovation(2));

        float mahal_sq = std::numeric_limits<float>::max();
        if (use_full_S) {
            // Use proper innovation covariance S for radar-only tracks.
            // LLT is guaranteed valid here (early return above on failure).
            ObjectUKF::RadarMeasVec y = track_llt.solve(innovation);
            mahal_sq                  = innovation.dot(y);
        } else {
            // R-only gating for camera-originated tracks (backward compatible)
            if (radar_llt_ok) {
                ObjectUKF::RadarMeasVec y = radar_llt.solve(innovation);
                mahal_sq                  = innovation.dot(y);
            } else {
                mahal_sq = 0.0f;
                for (int d = 0; d < ObjectUKF::RADAR_MEAS_DIM; ++d) {
                    mahal_sq += innovation(d) * innovation(d) * R_diag_inv(d);
                }
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
                DRONE_LOG_DEBUG("[UKF] Ground filter: rejected {}/{} radar detections "
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
        // Single lookup — reused for height override and filter creation (#422, review fix #6).
        auto it = filters_.find(trk.track_id);

        // Pass radar-learned height if available for this track (#422).
        float height_override = 0.0f;
        if (it != filters_.end() && it->second.height_learned) {
            height_override = it->second.learned_height_m;
        }
        auto [depth, depth_conf] = estimate_depth(trk, height_override);
        bool radar_init_used     = false;

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
                    DRONE_LOG_INFO("[UKF] Camera adopt: track {} ← radar-only {:#x}", trk.track_id,
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
                        // Azimuth sign convention configurable per backend (Issue #348).
                        const float az_err =
                            std::abs(wrap_angle(radar_cfg_.az_sign(rdet.azimuth_rad) - cam_az));
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
                        DRONE_LOG_INFO("[UKF] Radar-init: track {} cam_depth={:.1f}m → "
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
                    // Back-calculate object height from radar-init range (#422).
                    // Compensate for depth_scale so the round-trip is consistent:
                    // forward: depth = H * fy / bbox_h * ds → back: H = depth * bbox_h / (fy * ds)
                    const float fy_val = calib_.camera_intrinsics(1, 1);
                    const float ds     = calib_.depth_scale;
                    if (trk.bbox_h > 10.0f && fy_val > 0.0f && ds > 0.0f) {
                        // Clamp to [0.1m, 25m] to reject spoofed/noisy radar (review fix #4)
                        it->second.learned_height_m = std::clamp(depth * trk.bbox_h / (fy_val * ds),
                                                                 kMinLearnedHeightM,
                                                                 kMaxLearnedHeightM);
                        it->second.height_learned   = true;
                    }
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
                        DRONE_LOG_INFO("[UKF] Re-ID: track {} → dormant #{} at ({:.1f},{:.1f}), "
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
                            DRONE_LOG_INFO("[UKF] New obstacle: track {} → dormant #{} at "
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

                // Azimuth sign convention configurable per backend (Issue #348).
                ObjectUKF::RadarMeasVec z_actual;
                z_actual(0) = rdet.range_m;
                z_actual(1) = radar_cfg_.az_sign(rdet.azimuth_rad);
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

                // Back-calculate actual object height from radar range + bbox (#422).
                // Compensate for depth_scale: H = range * bbox_h / (fy * ds).
                // EMA smoothing (α=0.2) prevents noisy bbox from corrupting the
                // estimate.  This becomes the ML depth scale anchor (§3.7 of #393).
                const float fy_val = calib_.camera_intrinsics(1, 1);
                const float ds_val = calib_.depth_scale;
                if (trk.bbox_h > 10.0f && fy_val > 0.0f && ds_val > 0.0f) {
                    // Clamp to [0.1m, 25m] to reject spoofed/noisy radar (review fix #4)
                    const float actual_h = std::clamp(radar_dets_.detections[best_idx].range_m *
                                                          trk.bbox_h / (fy_val * ds_val),
                                                      kMinLearnedHeightM, kMaxLearnedHeightM);
                    if (ukf.height_learned) {
                        ukf.learned_height_m = kHeightEmaRetain * ukf.learned_height_m +
                                               kHeightEmaNew * actual_h;
                    } else {
                        ukf.learned_height_m = actual_h;
                        ukf.height_learned   = true;
                    }
                }
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
                DRONE_LOG_INFO("[UKF] Re-ID (late radar): track {} → dormant #{} at "
                               "({:.1f},{:.1f}), obs={}, dist={:.1f}m",
                               trk.track_id, didx, dobs.world_pos.x(), dobs.world_pos.y(),
                               dobs.observations,
                               (world_pos.head<2>() - dobs.world_pos.head<2>()).norm());
            } else if (static_cast<int>(dormant_obstacles_.size()) < max_dormant_) {
                dormant_obstacles_.push_back({world_pos, 1, tracked.timestamp_ns});
                track_to_dormant_[trk.track_id] = static_cast<int>(dormant_obstacles_.size()) - 1;
                DRONE_LOG_INFO("[UKF] New obstacle (late radar): track {} → dormant #{} "
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
    if (has_radar_data_ && radar_enabled_ && radar_cfg_.radar_only_enabled) {
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

                // Gate output: only emit after promotion threshold (consecutive
                // radar observations). This delays output until the track is
                // confirmed by N frames of radar data (Issue #231).
                if (ukf.radar_update_count < radar_cfg_.radar_only_promotion_hits) continue;

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
                fused.depth_confidence    = 1.0f;  // radar range is authoritative
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
    // of obstacles outside camera FOV (Issue #231: gated by radar_only_enabled).
    if (has_radar_data_ && radar_enabled_ && radar_cfg_.radar_only_enabled) {
        for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
            if (radar_matched[ri]) continue;
            const auto& rdet = radar_dets_.detections[ri];

            // Quality gates for orphan track creation:
            // 1. Range sanity — ignore detections beyond reliable radar range.
            //    Typical 77GHz drone radar: ≤50m. Default 25m rejects distant clutter.
            if (rdet.range_m > radar_cfg_.radar_max_orphan_range_m) continue;

            // 2. Ground-plane filter (redundant safety net) — applied even when
            //    the main ground filter is disabled, to prevent phantom ground tracks.
            //    No altitude gate: flying objects (drones, birds) may be at any altitude.
            if (has_altitude_) {
                const float object_alt = drone_altitude_m_ +
                                         rdet.range_m * std::sin(rdet.elevation_rad);
                if (object_alt < radar_cfg_.min_object_altitude_m) continue;
            }

            // Convert to body-frame for proximity check (Issue #348).
            const float           az     = radar_cfg_.az_sign(rdet.azimuth_rad);
            const float           cos_el = std::cos(rdet.elevation_rad);
            const float           body_x = rdet.range_m * cos_el * std::cos(az);
            const float           body_y = rdet.range_m * cos_el * std::sin(az);
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

            // Create new radar-only track (guard overflow to stay in high-bit range)
            if (next_radar_track_id_ == 0xFFFFFFFFu) next_radar_track_id_ = 0x80000000u;
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
                    DRONE_LOG_INFO(
                        "[UKF] Re-ID (radar-only): {:#x} → dormant #{} at ({:.1f},{:.1f})", rid,
                        didx, dobs.world_pos.x(), dobs.world_pos.y());
                } else if (static_cast<int>(dormant_obstacles_.size()) < max_dormant_) {
                    dormant_obstacles_.push_back({world_pos, 1, output.timestamp_ns});
                    track_to_dormant_[rid] = static_cast<int>(dormant_obstacles_.size()) - 1;
                    DRONE_LOG_INFO("[UKF] New obstacle (radar-only): {:#x} → dormant #{} at "
                                   "({:.1f},{:.1f})",
                                   rid, static_cast<int>(dormant_obstacles_.size()) - 1,
                                   world_pos.x(), world_pos.y());
                }
            }

            seen[rid] = true;

            // Gate output: only emit radar-only tracks after promotion threshold.
            // The filter still exists (accumulates future radar hits), but the grid
            // won't see it until it has enough observations to be trustworthy.
            // Uses radar_only_promotion_hits (default 6) for degraded-visibility
            // promotion (Issue #231).
            if (ukf.radar_update_count < radar_cfg_.radar_only_promotion_hits) continue;

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
            fused.depth_confidence    = 1.0f;  // radar range is authoritative
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
            radar_cfg.radar_only_promotion_hits = static_cast<uint32_t>(
                cfg->get<int>("perception.fusion.radar.promotion_hits",
                              static_cast<int>(radar_cfg.radar_only_promotion_hits)));
            // Issue #231: configurable radar-only track toggle and promotion frames.
            // radar_orphan_promote_frames overrides promotion_hits when present.
            radar_cfg.radar_only_enabled = cfg->get<bool>("perception.fusion.radar_only_enabled",
                                                          radar_cfg.radar_only_enabled);
            // Issue #348: azimuth sign convention — true for Gazebo/Simulated (FLU→FRD),
            // false for real hardware that provides native FRD azimuth.
            radar_cfg.negate_azimuth            = cfg->get<bool>("perception.radar.negate_azimuth",
                                                                 radar_cfg.negate_azimuth);
            radar_cfg.radar_only_promotion_hits = static_cast<uint32_t>(
                cfg->get<int>("perception.fusion.radar_orphan_promote_frames",
                              static_cast<int>(radar_cfg.radar_only_promotion_hits)));

            // Validate promotion_hits: negative config values would underflow
            // uint32_t; zero means no gating which defeats the purpose.  Clamp
            // to >= 1 with a warning.
            {
                const int raw_promotion = cfg->get<int>(
                    "perception.fusion.radar_orphan_promote_frames",
                    cfg->get<int>("perception.fusion.radar.promotion_hits",
                                  static_cast<int>(radar_cfg.radar_only_promotion_hits)));
                if (raw_promotion < 1) {
                    DRONE_LOG_WARN("[UKF] radar promotion_hits={} invalid (must be >= 1), clamping "
                                   "to 1",
                                   raw_promotion);
                    radar_cfg.radar_only_promotion_hits = 1;
                }
            }
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
