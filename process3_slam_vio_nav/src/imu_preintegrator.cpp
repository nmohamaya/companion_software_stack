// process3_slam_vio_nav/src/imu_preintegrator.cpp
// IMU pre-integration implementation (Forster et al. 2017).
//
// Discrete-time integration of gyro + accelerometer:
//   For each sample i with dt_i:
//     Δγ_{i+1} = Δγ_i  ⊗ Exp((ω_m - b_g) * dt)
//     Δβ_{i+1} = Δβ_i  + Δγ_i * (a_m - b_a) * dt
//     Δα_{i+1} = Δα_i  + Δβ_i * dt + ½ * Δγ_i * (a_m - b_a) * dt²

#include "slam/imu_preintegrator.h"

#include "util/ilogger.h"

#include <cmath>

namespace drone::slam {

// ── Exp map: so(3) → SO(3) ──────────────────────────────────
// Rodrigues' formula for the exponential map.
static Eigen::Quaterniond exp_map(const Eigen::Vector3d& omega_dt) {
    double theta = omega_dt.norm();
    if (theta < 1e-10) {
        // First-order approximation for small angles
        return Eigen::Quaterniond(1.0, omega_dt.x() * 0.5, omega_dt.y() * 0.5, omega_dt.z() * 0.5)
            .normalized();
    }
    Eigen::Vector3d axis = omega_dt / theta;
    return Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
}

// ── Skew-symmetric matrix [v]× ──────────────────────────────
static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

ImuPreintegrator::ImuPreintegrator(ImuNoiseParams params) : params_(params) {}

void ImuPreintegrator::set_bias(const Eigen::Vector3d& gyro_bias,
                                const Eigen::Vector3d& accel_bias) {
    gyro_bias_  = gyro_bias;
    accel_bias_ = accel_bias;
}

void ImuPreintegrator::add_sample(const ImuSample& sample) {
    samples_.push_back(sample);
}

VIOResult<PreintegratedMeasurement> ImuPreintegrator::integrate(
    drone::util::FrameDiagnostics& diag) const {

    // ── Pre-conditions ──────────────────────────────────────
    if (samples_.empty()) {
        return VIOResult<PreintegratedMeasurement>::err(
            VIOError(VIOErrorCode::ImuNotInitialized, "ImuPreintegrator",
                     "No IMU samples to integrate", diag.frame_id()));
    }

    if (samples_.size() < 2) {
        return VIOResult<PreintegratedMeasurement>::err(
            VIOError(VIOErrorCode::ImuDataGap, "ImuPreintegrator",
                     "Need at least 2 samples for pre-integration (have " +
                         std::to_string(samples_.size()) + ")",
                     diag.frame_id()));
    }

    PreintegratedMeasurement pm;
    pm.delta_position = Eigen::Vector3d::Zero();
    pm.delta_velocity = Eigen::Vector3d::Zero();
    pm.delta_rotation = Eigen::Quaterniond::Identity();
    pm.covariance     = Eigen::Matrix<double, 9, 9>::Zero();
    pm.jacobian_bias  = Eigen::Matrix<double, 9, 6>::Zero();

    int gap_count        = 0;
    int saturation_count = 0;

    // Estimate expected dt from first two samples
    double expected_dt = samples_[1].timestamp - samples_[0].timestamp;
    if (expected_dt <= 0) expected_dt = 1.0 / 400.0;  // fallback 400 Hz

    // ── Noise covariance per sample ─────────────────────────
    // Q_d is the per-sample discrete noise covariance
    // (entries computed below using the σ²/dt convention).
    Eigen::Matrix<double, 6, 6> Q_d = Eigen::Matrix<double, 6, 6>::Zero();

    // ── Main integration loop ───────────────────────────────
    for (size_t i = 1; i < samples_.size(); ++i) {
        double dt = samples_[i].timestamp - samples_[i - 1].timestamp;

        // ── Data quality checks ─────────────────────────────
        if (dt <= 0) {
            diag.add_warning("ImuPreintegrator", "Non-positive dt at sample " + std::to_string(i) +
                                                     ": dt=" + std::to_string(dt) + "s — skipping");
            continue;
        }

        if (dt > expected_dt * kMaxDtGap_ratio) {
            ++gap_count;
            diag.add_warning("ImuPreintegrator", "IMU gap at sample " + std::to_string(i) +
                                                     ": dt=" + std::to_string(dt * 1000.0) +
                                                     "ms (expected ~" +
                                                     std::to_string(expected_dt * 1000.0) + "ms)");
        }

        // Check for sensor saturation on RAW readings (before averaging).
        // Saturation is a hardware property: if any single reading exceeds
        // the sensor's physical range the value is clipped and unreliable.
        // Check BOTH endpoints of the midpoint interval.
        for (size_t k : {i - 1, i}) {
            if (samples_[k].gyro.norm() > kMaxGyro_rad_s) {
                ++saturation_count;
                diag.add_warning("ImuPreintegrator",
                                 "Gyro saturation at sample " + std::to_string(k) +
                                     ": |ω|=" + std::to_string(samples_[k].gyro.norm()) + " rad/s");
            }
            if (samples_[k].accel.norm() > kMaxAccel_m_s2) {
                ++saturation_count;
                diag.add_warning("ImuPreintegrator",
                                 "Accel saturation at sample " + std::to_string(k) +
                                     ": |a|=" + std::to_string(samples_[k].accel.norm()) + " m/s²");
            }
        }

        // Use midpoint values for better accuracy (trapezoidal integration)
        Eigen::Vector3d gyro  = 0.5 * (samples_[i - 1].gyro + samples_[i].gyro);
        Eigen::Vector3d accel = 0.5 * (samples_[i - 1].accel + samples_[i].accel);

        // Remove bias
        Eigen::Vector3d omega_unbiased = gyro - gyro_bias_;
        Eigen::Vector3d accel_unbiased = accel - accel_bias_;

        // ── Rotation update ─────────────────────────────────
        Eigen::Quaterniond dq     = exp_map(omega_unbiased * dt);
        Eigen::Matrix3d    R_prev = pm.delta_rotation.toRotationMatrix();

        // ── Noise propagation (discrete-time) ───────────────
        // State: [δα, δβ, δφ]  (9×1)
        // F = d(state_{k+1})/d(state_k)
        // G = d(state_{k+1})/d(noise_k)
        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
        Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();

        Eigen::Matrix3d accel_skew = skew(accel_unbiased);

        // F: position row
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * R_prev * accel_skew * dt * dt;

        // F: velocity row
        F.block<3, 3>(3, 6) = -R_prev * accel_skew * dt;

        // F: rotation row (identity for error-state)
        Eigen::Matrix3d dR  = dq.toRotationMatrix();
        F.block<3, 3>(6, 6) = dR.transpose();

        // G: noise → state
        G.block<3, 3>(0, 3) = 0.5 * R_prev * dt * dt;            // accel noise → pos
        G.block<3, 3>(3, 3) = R_prev * dt;                       // accel noise → vel
        G.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity() * dt;  // gyro noise → rot

        // Per-sample noise covariance
        double sigma_g2       = params_.gyro_noise_density * params_.gyro_noise_density / dt;
        double sigma_a2       = params_.accel_noise_density * params_.accel_noise_density / dt;
        Q_d.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_g2;
        Q_d.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_a2;

        // Propagate covariance: P_{k+1} = F * P_k * F^T + G * Q_d * G^T
        pm.covariance = F * pm.covariance * F.transpose() + G * Q_d * G.transpose();

        // ── Bias Jacobian propagation ───────────────────────
        // J_{k+1} = F * J_k + G_bias
        Eigen::Matrix<double, 9, 6> G_bias = Eigen::Matrix<double, 9, 6>::Zero();
        G_bias.block<3, 3>(0, 3)           = -0.5 * R_prev * dt * dt;            // b_a → pos
        G_bias.block<3, 3>(3, 3)           = -R_prev * dt;                       // b_a → vel
        G_bias.block<3, 3>(6, 0)           = -Eigen::Matrix3d::Identity() * dt;  // b_g → rot

        pm.jacobian_bias = F * pm.jacobian_bias + G_bias;

        // ── State update (after noise propagation) ──────────
        // Position: Δα += Δβ·dt + ½·R·(a-b_a)·dt²
        pm.delta_position += pm.delta_velocity * dt + 0.5 * R_prev * accel_unbiased * dt * dt;

        // Velocity: Δβ += R·(a-b_a)·dt
        pm.delta_velocity += R_prev * accel_unbiased * dt;

        // Rotation: Δγ = Δγ ⊗ Exp(ω·dt)
        pm.delta_rotation = (pm.delta_rotation * dq).normalized();

        pm.dt += dt;
    }

    pm.num_samples = static_cast<int>(samples_.size());
    pm.valid       = true;

    // ── Post-integration diagnostics ────────────────────────
    diag.add_metric("ImuPreintegrator", "num_samples", pm.num_samples);
    diag.add_metric("ImuPreintegrator", "total_dt", pm.dt);
    diag.add_metric("ImuPreintegrator", "delta_position_norm", pm.delta_position.norm());
    diag.add_metric("ImuPreintegrator", "delta_velocity_norm", pm.delta_velocity.norm());

    if (gap_count > 0) {
        diag.add_warning("ImuPreintegrator",
                         std::to_string(gap_count) + " IMU data gap(s) detected");
    }
    if (saturation_count > 0) {
        diag.add_warning("ImuPreintegrator",
                         std::to_string(saturation_count) + " IMU saturation event(s) detected");
    }

    // Check covariance is sane (no NaN / Inf)
    if (!pm.covariance.allFinite()) {
        pm.valid = false;
        auto err = VIOError(VIOErrorCode::NumericalFailure, "ImuPreintegrator",
                            "Covariance contains NaN/Inf after integration "
                            "(" +
                                std::to_string(pm.num_samples) + " samples, " +
                                std::to_string(pm.dt) + "s)",
                            diag.frame_id());
        diag.add_error("ImuPreintegrator", err.message);
        return VIOResult<PreintegratedMeasurement>::err(std::move(err));
    }

    return VIOResult<PreintegratedMeasurement>::ok(std::move(pm));
}

void ImuPreintegrator::reset() {
    samples_.clear();
}

}  // namespace drone::slam
