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
                     const RadarNoiseConfig& radar_cfg)
    : track_id(trk.track_id), age(0) {
    // Initialize state: [x=depth, y=bearing_x*depth, z=0, vx, vy, vz]
    x_    = StateVec::Zero();
    x_(0) = initial_depth;                                 // x (forward)
    x_(1) = trk.position_2d.x() * 0.001f * initial_depth;  // y (lateral, from pixel)
    x_(2) = 0.0f;                                          // z (altitude, relative)
    x_(3) = trk.velocity_2d.x() * 0.1f;                    // vx
    x_(4) = trk.velocity_2d.y() * 0.1f;                    // vy
    x_(5) = 0.0f;                                          // vz

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

void ObjectUKF::update_camera(const TrackedObject& trk, float estimated_depth) {
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

    // Actual measurement
    float   bearing_x = trk.position_2d.x() * 0.001f;  // pixel → rough bearing
    float   bearing_y = trk.position_2d.y() * 0.001f;
    MeasVec z_actual(bearing_x, bearing_y, estimated_depth);

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

    // Mean predicted measurement
    RadarMeasVec z_mean = w0_mean * z_sigma[0];
    for (int i = 1; i < 2 * n + 1; ++i) {
        z_mean += wi * z_sigma[i];
    }

    // Innovation covariance S = Pzz + R_radar
    RadarMeasVec dz0 = z_sigma[0] - z_mean;
    RadarMeasMat S   = w0_cov * (dz0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        RadarMeasVec dzi = z_sigma[i] - z_mean;
        S += wi * (dzi * dzi.transpose());
    }
    S += R_radar_;

    // Cross-covariance Pxz
    using RadarCrossMat = Eigen::Matrix<float, STATE_DIM, RADAR_MEAS_DIM>;
    StateVec      dx0   = sigma[0] - x_;
    RadarCrossMat Pxz   = w0_cov * (dx0 * dz0.transpose());
    for (int i = 1; i < 2 * n + 1; ++i) {
        StateVec     dxi = sigma[i] - x_;
        RadarMeasVec dzi = z_sigma[i] - z_mean;
        Pxz += wi * (dxi * dzi.transpose());
    }

    // Kalman gain (solve via LDLT for numerical stability)
    RadarCrossMat K = (S.ldlt().solve(Pxz.transpose())).transpose();

    // Actual radar measurement
    RadarMeasVec z_actual;
    z_actual(0) = det.range_m;
    z_actual(1) = det.azimuth_rad;
    z_actual(2) = det.elevation_rad;
    z_actual(3) = det.radial_velocity_mps;

    // Update
    x_ += K * (z_actual - z_mean);
    P_ -= K * S * K.transpose();

    // Ensure P stays symmetric positive definite
    P_ = (P_ + P_.transpose()) * 0.5f;
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
UKFFusionEngine::UKFFusionEngine(const CalibrationData& calib, const RadarNoiseConfig& radar_cfg)
    : calib_(calib), radar_cfg_(radar_cfg) {}

float UKFFusionEngine::estimate_depth(const TrackedObject& trk) const {
    const float fy = calib_.camera_intrinsics(1, 1);
    return calib_.camera_height_m * fy / std::max(10.0f, trk.position_2d.y());
}

void UKFFusionEngine::reset() {
    filters_.clear();
    has_radar_data_ = false;
}

void UKFFusionEngine::set_radar_detections(const drone::ipc::RadarDetectionList& detections) {
    radar_dets_     = detections;
    has_radar_data_ = true;
}

FusedObjectList UKFFusionEngine::fuse(const TrackedObjectList& tracked) {
    FusedObjectList output;
    output.timestamp_ns   = tracked.timestamp_ns;
    output.frame_sequence = tracked.frame_sequence;

    // Track which UKF filters are still active this frame
    std::unordered_map<uint32_t, bool> seen;

    // Track which radar detections have been matched (greedy, one-to-one)
    std::vector<bool> radar_matched;
    if (has_radar_data_) {
        radar_matched.resize(radar_dets_.num_detections, false);
    }

    for (const auto& trk : tracked.objects) {
        float depth = estimate_depth(trk);

        auto it = filters_.find(trk.track_id);
        if (it == filters_.end()) {
            // New track — create UKF
            filters_.emplace(trk.track_id, ObjectUKF(trk, depth, radar_cfg_));
            it = filters_.find(trk.track_id);
        }

        auto& ukf = it->second;
        ukf.predict();
        ukf.update_camera(trk, depth);

        // Radar association: find best matching radar detection within gate
        bool matched_radar = false;
        if (has_radar_data_ && radar_dets_.num_detections > 0) {
            auto z_pred = ukf.predicted_radar_measurement();

            float    best_dist = std::numeric_limits<float>::max();
            uint32_t best_idx  = 0;

            for (uint32_t ri = 0; ri < radar_dets_.num_detections; ++ri) {
                if (radar_matched[ri]) continue;

                const auto&             rdet = radar_dets_.detections[ri];
                ObjectUKF::RadarMeasVec z_actual;
                z_actual(0) = rdet.range_m;
                z_actual(1) = rdet.azimuth_rad;
                z_actual(2) = rdet.elevation_rad;
                z_actual(3) = rdet.radial_velocity_mps;

                // Innovation (residual)
                ObjectUKF::RadarMeasVec innovation = z_actual - z_pred;

                // Mahalanobis distance: d² = innovation^T * R_radar^{-1} * innovation
                // Use R_radar as approximation of innovation covariance for gating
                // (full S would require sigma point propagation — R is a cheaper proxy)
                const auto& R_inv_diag = ukf.radar_noise().diagonal();
                float       mahal_sq   = 0.0f;
                for (int d = 0; d < ObjectUKF::RADAR_MEAS_DIM; ++d) {
                    mahal_sq += (innovation(d) * innovation(d)) / R_inv_diag(d);
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
        fused.has_radar           = matched_radar;
        fused.position_covariance = ukf.position_covariance();
        fused.timestamp_ns        = trk.timestamp_ns;

        output.objects.push_back(fused);
    }

    // Remove stale filters (not seen this frame)
    for (auto it = filters_.begin(); it != filters_.end();) {
        if (seen.find(it->first) == seen.end()) {
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
        if (cfg) {
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
        }
        return std::make_unique<UKFFusionEngine>(calib, radar_cfg);
    }
    throw std::invalid_argument("Unknown fusion engine backend: " + backend);
}

}  // namespace drone::perception
