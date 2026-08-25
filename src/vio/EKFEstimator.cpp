// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// EKFEstimator.cpp    Error-State EKF implementation
// Drone Swarm Sensor Fusion  |  Phase 2

#include "vio/EKFEstimator.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>

namespace drone::vio {

static constexpr double kGravity = 9.81;

std::string_view to_string(EstimatorOperationResult result) {
    switch (result) {
    case EstimatorOperationResult::Accepted:
        return "accepted";
    case EstimatorOperationResult::RejectedNotInitialized:
        return "rejected_not_initialized";
    case EstimatorOperationResult::RejectedNonFiniteInput:
        return "rejected_non_finite_input";
    case EstimatorOperationResult::RejectedInvalidTimestamp:
        return "rejected_invalid_timestamp";
    case EstimatorOperationResult::RejectedDuplicateTimestamp:
        return "rejected_duplicate_timestamp";
    case EstimatorOperationResult::RejectedBackwardTimestamp:
        return "rejected_backward_timestamp";
    case EstimatorOperationResult::RejectedTimeStepTooSmall:
        return "rejected_time_step_too_small";
    case EstimatorOperationResult::RejectedTimeStepTooLarge:
        return "rejected_time_step_too_large";
    case EstimatorOperationResult::RejectedInvalidCovariance:
        return "rejected_invalid_covariance";
    case EstimatorOperationResult::RejectedInvalidQuaternion:
        return "rejected_invalid_quaternion";
    case EstimatorOperationResult::RejectedDimensionMismatch:
        return "rejected_dimension_mismatch";
    case EstimatorOperationResult::RejectedUnsupportedMeasurement:
        return "rejected_unsupported_measurement";
    case EstimatorOperationResult::RejectedInvalidConfiguration:
        return "rejected_invalid_configuration";
    case EstimatorOperationResult::FailedFactorization:
        return "failed_factorization";
    case EstimatorOperationResult::FailedNumericalValidation:
        return "failed_numerical_validation";
    }
    return "unknown";
}

EKFEstimator::EKFEstimator(EKFConfig cfg) : cfg_(cfg) {
    logger_ = spdlog::get("EKF");
    if (!logger_)
        logger_ = spdlog::stdout_color_mt("EKF");

    // Build continuous-time IMU noise matrix Q_imu
    Q_imu_.setZero();
    Q_imu_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * cfg_.sigma_na * cfg_.sigma_na;
    Q_imu_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * cfg_.sigma_ng * cfg_.sigma_ng;
    Q_imu_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * cfg_.sigma_nba * cfg_.sigma_nba;
    Q_imu_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * cfg_.sigma_nbg * cfg_.sigma_nbg;
    validation_cfg_valid_ = validate_validation_config(validation_cfg_);
    reset_diagnostics_locked();
}

void EKFEstimator::reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                         const Eigen::Vector3d& v0) {
    std::lock_guard lock(mtx_);
    reset_diagnostics_locked();
    pos_ = p0;
    vel_ = v0;
    if (!q0.coeffs().array().isFinite().all() || q0.norm() < validation_cfg_.quaternion_min_norm) {
        q_ = Eigen::Quaterniond::Identity();
        initialized_ = false;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        return;
    }
    q_ = q0.normalized();
    ba_.setZero();
    bg_.setZero();

    // Initial covariance
    P_.setZero();
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * cfg_.init_pos_std * cfg_.init_pos_std;
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * cfg_.init_vel_std * cfg_.init_vel_std;
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * cfg_.init_att_std * cfg_.init_att_std;
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * cfg_.init_ba_std * cfg_.init_ba_std;
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * cfg_.init_bg_std * cfg_.init_bg_std;

    timestamp_ = 0.0;
    total_drift_ = 0.0;
    initialized_ = true;
    last_vision_update_ts_ = -1.0;
    last_depth_update_ts_ = -1.0;
    last_accepted_imu_timestamp_s_.reset();
    diagnostics_.initialized = true;
    diagnostics_.last_accepted_timestamp = -1.0;
    diagnostics_.last_received_timestamp = -1.0;
    set_result_locked(EstimatorOperationResult::Accepted);
    logger_->info("EKF reset. p0=[{:.3f},{:.3f},{:.3f}]", p0.x(), p0.y(), p0.z());
}

// IMU Propagation  (error-state EKF prediction step)

void EKFEstimator::propagate_imu(const Eigen::Vector3d& accel_mps2,
                                 const Eigen::Vector3d& gyro_rads, double dt) {
    std::lock_guard lock(mtx_);
    (void)propagate_imu_locked(accel_mps2, gyro_rads, dt, true);
}

EstimatorOperationResult EKFEstimator::process_imu_measurement(const Eigen::Vector3d& accel_mps2,
                                                               const Eigen::Vector3d& gyro_rads,
                                                               double timestamp_s) {
    std::lock_guard lock(mtx_);
    diagnostics_.last_received_timestamp = timestamp_s;

    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        return diagnostics_.last_operation_result;
    }
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return diagnostics_.last_operation_result;
    }
    if (!std::isfinite(timestamp_s)) {
        diagnostics_.non_finite_input_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidTimestamp);
        return diagnostics_.last_operation_result;
    }
    if (!vector_finite(accel_mps2) || !vector_finite(gyro_rads)) {
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return diagnostics_.last_operation_result;
    }
    if (!last_accepted_imu_timestamp_s_.has_value()) {
        last_accepted_imu_timestamp_s_ = timestamp_s;
        diagnostics_.last_accepted_timestamp = timestamp_s;
        set_result_locked(EstimatorOperationResult::Accepted);
        return diagnostics_.last_operation_result;
    }

    const double dt = timestamp_s - *last_accepted_imu_timestamp_s_;
    if (timestamp_s == *last_accepted_imu_timestamp_s_) {
        diagnostics_.duplicate_timestamp_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedDuplicateTimestamp);
        return diagnostics_.last_operation_result;
    }
    if (validation_cfg_.require_monotonic_timestamps &&
        timestamp_s < *last_accepted_imu_timestamp_s_) {
        diagnostics_.backward_timestamp_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedBackwardTimestamp);
        return diagnostics_.last_operation_result;
    }
    if (!std::isfinite(dt)) {
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidTimestamp);
        return diagnostics_.last_operation_result;
    }
    if (dt < validation_cfg_.min_imu_dt_s) {
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedTimeStepTooSmall);
        return diagnostics_.last_operation_result;
    }
    if (dt > validation_cfg_.max_imu_dt_s) {
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedTimeStepTooLarge);
        return diagnostics_.last_operation_result;
    }

    const auto result = propagate_imu_locked(accel_mps2, gyro_rads, dt, false);
    if (result == EstimatorOperationResult::Accepted) {
        last_accepted_imu_timestamp_s_ = timestamp_s;
        diagnostics_.last_accepted_timestamp = timestamp_s;
    }
    return result;
}

// Vision Update  (feature reprojection)

void EKFEstimator::update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                                 const std::vector<Eigen::Vector3d>& p_world,
                                 const Eigen::Matrix3d& K) {
    if (!initialized_)
        return note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized), void();
    if (z_pixels.size() != p_world.size() || z_pixels.empty())
        return note_rejection_locked(EstimatorOperationResult::RejectedDimensionMismatch), void();

    std::lock_guard lock(mtx_);
    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        return;
    }
    if (!state_valid(snapshot_locked())) {
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return;
    }

    const Eigen::Matrix3d R = q_.toRotationMatrix();
    const double fx = K(0, 0), fy = K(1, 1);
    const double cx = K(0, 2), cy = K(1, 2);
    const double sigma2 = cfg_.sigma_px * cfg_.sigma_px;

    bool accepted_update = false;
    for (size_t i = 0; i < z_pixels.size(); ++i) {
        // Project map point into current camera frame
        const Eigen::Vector3d p_c = R.transpose() * (p_world[i] - pos_);

        if (p_c.z() < 0.1)
            continue; // behind camera

        const double Xc = p_c.x(), Yc = p_c.y(), Zc = p_c.z();
        const double u_hat = fx * Xc / Zc + cx;
        const double v_hat = fy * Yc / Zc + cy;

        // Jacobian H (2Ã—15) of [u,v] w.r.t. error state
        // Only position (cols 0:2) and attitude (cols 6:8) terms are nonzero
        Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();

        // âˆ‚[u,v]/âˆ‚p_c
        Eigen::Matrix<double, 2, 3> J_proj;
        J_proj << fx / Zc, 0, -fx * Xc / (Zc * Zc), 0, fy / Zc, -fy * Yc / (Zc * Zc);

        // âˆ‚p_c/âˆ‚pos (= -R^T)
        const Eigen::Matrix3d dpc_dpos = -R.transpose();
        H.block<2, 3>(0, 0) = J_proj * dpc_dpos;

        // âˆ‚p_c/âˆ‚Î¸ (= [p_w - pos]Ã—  in body frame... skew sym)
        Eigen::Matrix3d skew_pw;
        const Eigen::Vector3d dv = p_world[i] - pos_;
        skew_pw << 0, -dv.z(), dv.y(), dv.z(), 0, -dv.x(), -dv.y(), dv.x(), 0;
        H.block<2, 3>(0, 6) = J_proj * R.transpose() * skew_pw;

        // Measurement noise
        const Eigen::Matrix2d R_meas = Eigen::Matrix2d::Identity() * sigma2;

        // Innovation
        const Eigen::Vector2d innov{z_pixels[i].x() - u_hat, z_pixels[i].y() - v_hat};

        // Mahalanobis gating
        const Eigen::Matrix2d S = H * P_ * H.transpose() + R_meas;
        const Eigen::LDLT<Eigen::Matrix2d> gate_ldlt(S);
        if (gate_ldlt.info() != Eigen::Success) {
            diagnostics_.numerical_failure_count++;
            note_rejection_locked(EstimatorOperationResult::FailedFactorization);
            continue;
        }
        const double mah_sq = innov.transpose() * gate_ldlt.solve(innov);
        if (mah_sq > cfg_.mahal_gate)
            continue; // outlier

        const auto result = apply_error_state_update_locked(
            innov, H, R_meas, [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
                candidate.pos += dx.segment<3>(0);
                candidate.vel += dx.segment<3>(3);
                candidate.ba += dx.segment<3>(9);
                candidate.bg += dx.segment<3>(12);
                const Eigen::Vector3d dtheta = dx.segment<3>(6);
                if (dtheta.norm() > 1e-8) {
                    candidate.q = candidate.q * rotvec_to_quat(dtheta);
                }
            });
        if (result == EstimatorOperationResult::Accepted) {
            accepted_update = true;
        }
    }

    if (accepted_update) {
        last_vision_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_depth(double z_depth_m, double sigma_m) {
    std::lock_guard lock(mtx_);
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return;
    }
    if (!validation_cfg_.lidar_depth_correction_enabled) {
        diagnostics_.disabled_lidar_correction_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedUnsupportedMeasurement);
        return;
    }
    if (!std::isfinite(z_depth_m) || !std::isfinite(sigma_m) || sigma_m <= 0.0) {
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return;
    }

    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;
    Eigen::Matrix<double, 1, 1> R_meas;
    R_meas(0, 0) = sigma_m * sigma_m;
    Eigen::Matrix<double, 1, 1> innov;
    innov(0, 0) = z_depth_m - pos_.z();

    const auto result = apply_error_state_update_locked(
        innov, H, R_meas, [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
        });
    if (result == EstimatorOperationResult::Accepted) {
        last_depth_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_visual_pose(const Eigen::Vector3d& observed_position,
                                      const Eigen::Vector3d& observed_velocity,
                                      double sigma_position_m, double sigma_velocity_mps) {
    std::lock_guard lock(mtx_);
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return;
    }

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> R_meas = Eigen::Matrix<double, 6, 6>::Zero();
    R_meas.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_position_m * sigma_position_m;
    R_meas.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * sigma_velocity_mps * sigma_velocity_mps;

    Eigen::Matrix<double, 6, 1> innov;
    innov.segment<3>(0) = observed_position - pos_;
    innov.segment<3>(3) = observed_velocity - vel_;

    const auto result = apply_error_state_update_locked(
        innov, H, R_meas, [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
            candidate.ba += dx.segment<3>(9);
            candidate.bg += dx.segment<3>(12);
        });
    if (result == EstimatorOperationResult::Accepted) {
        last_vision_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_zupt() {
    std::lock_guard lock(mtx_);
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return;
    }

    // Zero velocity update: z = [0,0,0], H = [0 I_3 0 ...]
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const double sigma = validation_cfg_.zupt_sigma_velocity_mps;
    if (!std::isfinite(sigma) || sigma <= 0.0) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        diagnostics_.zupt_rejected_count++;
        return;
    }

    const Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * sigma * sigma;
    const Eigen::Vector3d innov = -vel_;

    (void)apply_error_state_update_locked(
        innov, H, R_meas,
        [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
            candidate.ba += dx.segment<3>(9);
            candidate.bg += dx.segment<3>(12);
            candidate.vel.setZero();
        },
        true);
}

PoseEstimate EKFEstimator::state() const {
    std::lock_guard lock(mtx_);
    PoseEstimate est;
    est.timestamp = timestamp_;
    est.position = pos_;
    est.velocity = vel_;
    est.orientation = q_;
    est.accel_bias = ba_;
    est.gyro_bias = bg_;
    est.pos_std = P_.diagonal().head<3>().cwiseSqrt();
    est.drift_m = total_drift_;

    const double uncertainty_norm = est.pos_std.norm();
    const double vision_age = (last_vision_update_ts_ >= 0.0)
                                  ? std::max(0.0, timestamp_ - last_vision_update_ts_)
                                  : 1.0e9;
    const double depth_age =
        (last_depth_update_ts_ >= 0.0) ? std::max(0.0, timestamp_ - last_depth_update_ts_) : 1.0e9;

    double confidence = std::clamp(1.0 - (uncertainty_norm / 2.5), 0.0, 1.0);
    if (vision_age > 0.8) {
        confidence *= 0.78;
    }
    if (vision_age > 1.6) {
        confidence *= 0.62;
    }
    if (depth_age < 0.6) {
        confidence = std::min(1.0, confidence + 0.08);
    }

    est.localization_confidence = std::clamp(confidence, 0.0, 1.0);
    est.localization_degraded =
        est.localization_confidence < 0.58 || uncertainty_norm > 0.85 || vision_age > 1.2;
    est.localization_lost =
        est.localization_confidence < 0.22 || uncertainty_norm > 1.8 || vision_age > 3.5;

    if (vision_age < 0.5 && depth_age < 0.7) {
        est.localization_source = "vision-depth-fused";
    } else if (vision_age < 0.8) {
        est.localization_source = "vision-inertial";
    } else if (depth_age < 0.7) {
        est.localization_source = "lidar-aided-inertial";
    } else {
        est.localization_source = "imu-dead-reckoning";
    }
    return est;
}

void EKFEstimator::configure_validation(const EstimatorValidationConfig& cfg) {
    std::lock_guard lock(mtx_);
    validation_cfg_ = cfg;
    validation_cfg_valid_ = validate_validation_config(validation_cfg_);
    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
    }
}

EstimatorValidationConfig EKFEstimator::validation_config() const {
    std::lock_guard lock(mtx_);
    return validation_cfg_;
}

EKFDiagnostics EKFEstimator::diagnostics() const {
    std::lock_guard lock(mtx_);
    return diagnostics_;
}

CovMat EKFEstimator::covariance() const {
    std::lock_guard lock(mtx_);
    return P_;
}

// Private helpers

FMat EKFEstimator::compute_F(const Eigen::Vector3d& a_body, const Eigen::Matrix3d& R,
                             double dt) const {
    FMat F = FMat::Identity();

    // âˆ‚pos/âˆ‚vel
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // âˆ‚vel/âˆ‚Î¸  (a_body skew)
    Eigen::Matrix3d skew_a;
    skew_a << 0, -a_body.z(), a_body.y(), a_body.z(), 0, -a_body.x(), -a_body.y(), a_body.x(), 0;
    F.block<3, 3>(3, 6) = -R * skew_a * dt;

    // âˆ‚vel/âˆ‚ba
    F.block<3, 3>(3, 9) = -R * dt;

    // âˆ‚Î¸/âˆ‚bg
    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    return F;
}

GMat EKFEstimator::compute_G(const Eigen::Matrix3d& R) const {
    GMat G = GMat::Zero();
    G.block<3, 3>(3, 0) = -R;                           // accel noise  velocity
    G.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity(); // gyro noise  attitude
    G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();  // accel bias drive
    G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity(); // gyro  bias drive
    return G;
}

Eigen::Quaterniond EKFEstimator::propagate_quat(const Eigen::Quaterniond& q,
                                                const Eigen::Vector3d& omega, double dt) {
    const double angle = omega.norm() * dt;
    if (angle < 1e-10)
        return q;
    const Eigen::AngleAxisd aa(angle, omega.normalized());
    return (q * Eigen::Quaterniond(aa)).normalized();
}

Eigen::Quaterniond EKFEstimator::rotvec_to_quat(const Eigen::Vector3d& rv) {
    const double angle = rv.norm();
    if (angle < 1e-10)
        return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv / angle));
}

Eigen::Vector3d EKFEstimator::quat_to_rotvec(const Eigen::Quaterniond& q) {
    const Eigen::AngleAxisd aa(q);
    return aa.axis() * aa.angle();
}

bool EKFEstimator::validate_validation_config(const EstimatorValidationConfig& cfg) const {
    return std::isfinite(cfg.max_imu_dt_s) && std::isfinite(cfg.min_imu_dt_s) &&
           std::isfinite(cfg.covariance_symmetry_tolerance) &&
           std::isfinite(cfg.variance_negativity_tolerance) &&
           std::isfinite(cfg.quaternion_min_norm) && std::isfinite(cfg.zupt_sigma_velocity_mps) &&
           std::isfinite(cfg.stationary_detector.accel_threshold_mps2) &&
           std::isfinite(cfg.stationary_detector.gyro_threshold_rads) &&
           std::isfinite(cfg.stationary_detector.minimum_stationary_time_s) &&
           std::isfinite(cfg.stationary_detector.accel_exit_threshold_mps2) &&
           std::isfinite(cfg.stationary_detector.gyro_exit_threshold_rads) &&
           std::isfinite(cfg.zupt.velocity_noise_mps) &&
           std::isfinite(cfg.zupt.max_update_rate_hz) && cfg.max_imu_dt_s > 0.0 &&
           cfg.min_imu_dt_s >= 0.0 && cfg.max_imu_dt_s >= cfg.min_imu_dt_s &&
           cfg.covariance_symmetry_tolerance >= 0.0 && cfg.variance_negativity_tolerance >= 0.0 &&
           cfg.quaternion_min_norm > 0.0 && cfg.zupt_sigma_velocity_mps > 0.0 &&
           cfg.stationary_detector.accel_threshold_mps2 >= 0.0 &&
           cfg.stationary_detector.gyro_threshold_rads >= 0.0 &&
           cfg.stationary_detector.window_size > 0u && cfg.stationary_detector.enter_count > 0u &&
           cfg.stationary_detector.exit_count > 0u &&
           cfg.stationary_detector.enter_count <= cfg.stationary_detector.window_size &&
           cfg.stationary_detector.exit_count <= cfg.stationary_detector.window_size &&
           cfg.stationary_detector.minimum_stationary_time_s >= 0.0 &&
           cfg.stationary_detector.accel_exit_threshold_mps2 >=
               cfg.stationary_detector.accel_threshold_mps2 &&
           cfg.stationary_detector.gyro_exit_threshold_rads >=
               cfg.stationary_detector.gyro_threshold_rads &&
           cfg.zupt.velocity_noise_mps > 0.0 && cfg.zupt.max_update_rate_hz > 0.0;
}

void EKFEstimator::reset_diagnostics_locked() {
    diagnostics_ = {};
    diagnostics_.last_rejection_reason = EstimatorOperationResult::Accepted;
    diagnostics_.last_operation_result = EstimatorOperationResult::Accepted;
    diagnostics_.initialized = initialized_;
}

void EKFEstimator::set_result_locked(EstimatorOperationResult result) {
    diagnostics_.initialized = initialized_;
    diagnostics_.last_operation_result = result;
    if (result != EstimatorOperationResult::Accepted) {
        diagnostics_.last_rejection_reason = result;
    }
}

void EKFEstimator::note_rejection_locked(EstimatorOperationResult result) {
    if (result == EstimatorOperationResult::Accepted) {
        set_result_locked(result);
        return;
    }
    set_result_locked(result);
}

bool EKFEstimator::vector_finite(const Eigen::Vector3d& value) const {
    return value.array().isFinite().all();
}

bool EKFEstimator::matrix_finite(const CovMat& value) const {
    return value.array().isFinite().all();
}

bool EKFEstimator::quaternion_valid(const Eigen::Quaterniond& value) const {
    return value.coeffs().array().isFinite().all() &&
           value.norm() >= validation_cfg_.quaternion_min_norm;
}

bool EKFEstimator::covariance_valid(const CovMat& value) const {
    if (!matrix_finite(value)) {
        return false;
    }
    const CovMat sym = 0.5 * (value + value.transpose());
    if ((sym - value).cwiseAbs().maxCoeff() > validation_cfg_.covariance_symmetry_tolerance) {
        return false;
    }
    for (int i = 0; i < value.rows(); ++i) {
        if (value(i, i) < -validation_cfg_.variance_negativity_tolerance) {
            return false;
        }
    }
    return true;
}

bool EKFEstimator::state_valid(const NominalStateSnapshot& candidate) const {
    return vector_finite(candidate.pos) && vector_finite(candidate.vel) &&
           vector_finite(candidate.ba) && vector_finite(candidate.bg) &&
           quaternion_valid(candidate.q) && covariance_valid(candidate.P) &&
           std::isfinite(candidate.total_drift);
}

double EKFEstimator::compute_uncertainty_norm(const CovMat& cov) const {
    return cov.diagonal().head<3>().cwiseMax(0.0).cwiseSqrt().norm();
}

EKFEstimator::NominalStateSnapshot EKFEstimator::snapshot_locked() const {
    NominalStateSnapshot out;
    out.pos = pos_;
    out.vel = vel_;
    out.q = q_;
    out.ba = ba_;
    out.bg = bg_;
    out.P = P_;
    out.total_drift = total_drift_;
    return out;
}

void EKFEstimator::commit_locked(const NominalStateSnapshot& candidate) {
    pos_ = candidate.pos;
    vel_ = candidate.vel;
    q_ = candidate.q.normalized();
    ba_ = candidate.ba;
    bg_ = candidate.bg;
    P_ = 0.5 * (candidate.P + candidate.P.transpose());
    total_drift_ = candidate.total_drift;
}

EstimatorOperationResult EKFEstimator::propagate_imu_locked(const Eigen::Vector3d& accel_mps2,
                                                            const Eigen::Vector3d& gyro_rads,
                                                            double dt,
                                                            bool update_internal_timestamp) {
    if (!validation_cfg_valid_) {
        diagnostics_.rejected_propagation_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        return diagnostics_.last_operation_result;
    }
    if (!initialized_) {
        diagnostics_.rejected_propagation_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return diagnostics_.last_operation_result;
    }
    if (!vector_finite(accel_mps2) || !vector_finite(gyro_rads)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return diagnostics_.last_operation_result;
    }
    if (!std::isfinite(dt)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidTimestamp);
        return diagnostics_.last_operation_result;
    }
    if (dt < validation_cfg_.min_imu_dt_s) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedTimeStepTooSmall);
        return diagnostics_.last_operation_result;
    }
    if (dt > validation_cfg_.max_imu_dt_s) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.invalid_timestep_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedTimeStepTooLarge);
        return diagnostics_.last_operation_result;
    }

    NominalStateSnapshot candidate = snapshot_locked();
    if (!state_valid(candidate)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return diagnostics_.last_operation_result;
    }

    const Eigen::Vector3d a = accel_mps2 - ba_;
    const Eigen::Vector3d w = gyro_rads - bg_;
    if (!vector_finite(a) || !vector_finite(w)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return diagnostics_.last_operation_result;
    }

    const Eigen::Matrix3d R = q_.toRotationMatrix();
    const Eigen::Vector3d g_world{0.0, 0.0, -kGravity};
    const Eigen::Quaterniond q_half = propagate_quat(q_, w, dt * 0.5);
    if (!quaternion_valid(q_half)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        return diagnostics_.last_operation_result;
    }
    const Eigen::Matrix3d R_half = q_half.toRotationMatrix();
    const Eigen::Vector3d a_world = R_half * a + g_world;
    candidate.pos += candidate.vel * dt + 0.5 * a_world * dt * dt;
    candidate.vel += a_world * dt;
    candidate.q = propagate_quat(candidate.q, w, dt);
    if (!quaternion_valid(candidate.q)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        return diagnostics_.last_operation_result;
    }

    const FMat F = compute_F(a, R, dt);
    const GMat G = compute_G(R);
    const CovMat Q_d = (G * Q_imu_ * G.transpose()) * dt;
    candidate.P = F * candidate.P * F.transpose() + Q_d;
    candidate.P = 0.5 * (candidate.P + candidate.P.transpose());
    candidate.total_drift = compute_uncertainty_norm(candidate.P);

    if (!state_valid(candidate)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.covariance_failure_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return diagnostics_.last_operation_result;
    }

    commit_locked(candidate);
    if (update_internal_timestamp) {
        timestamp_ += dt;
    }
    diagnostics_.accepted_propagation_count++;
    set_result_locked(EstimatorOperationResult::Accepted);
    return diagnostics_.last_operation_result;
}

EstimatorOperationResult EKFEstimator::apply_error_state_update_locked(
    const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R_meas,
    const std::function<void(NominalStateSnapshot&, const ErrorVec&)>& apply_dx,
    bool count_as_zupt) {
    if (!validation_cfg_valid_) {
        diagnostics_.rejected_update_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    if (!initialized_) {
        diagnostics_.rejected_update_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    if (innovation.size() != H.rows() || H.cols() != kErrorDim || R_meas.rows() != H.rows() ||
        R_meas.cols() != H.rows()) {
        diagnostics_.rejected_update_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedDimensionMismatch);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    if (!innovation.array().isFinite().all() || !H.array().isFinite().all() ||
        !R_meas.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    if ((R_meas - R_meas.transpose()).cwiseAbs().maxCoeff() >
        validation_cfg_.covariance_symmetry_tolerance) {
        diagnostics_.rejected_update_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidCovariance);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    for (int i = 0; i < R_meas.rows(); ++i) {
        if (R_meas(i, i) < 0.0) {
            diagnostics_.rejected_update_count++;
            note_rejection_locked(EstimatorOperationResult::RejectedInvalidCovariance);
            if (count_as_zupt) {
                diagnostics_.zupt_rejected_count++;
            }
            return diagnostics_.last_operation_result;
        }
    }

    const Eigen::MatrixXd S = H * P_ * H.transpose() + R_meas;
    if (!S.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedFactorization);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    const Eigen::MatrixXd PHt = P_ * H.transpose();
    const Eigen::MatrixXd K_gain = ldlt.solve(PHt.transpose()).transpose();
    if (!K_gain.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    const Eigen::VectorXd dx_dyn = K_gain * innovation;
    if (dx_dyn.size() != kErrorDim || !dx_dyn.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    const ErrorVec dx = dx_dyn;

    NominalStateSnapshot candidate = snapshot_locked();
    apply_dx(candidate, dx);
    if (!quaternion_valid(candidate.q)) {
        diagnostics_.rejected_update_count++;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    candidate.q.normalize();

    const CovMat I_KH = CovMat::Identity() - (K_gain * H);
    candidate.P = I_KH * P_ * I_KH.transpose() + K_gain * R_meas * K_gain.transpose();
    candidate.P = 0.5 * (candidate.P + candidate.P.transpose());
    candidate.total_drift = compute_uncertainty_norm(candidate.P);
    if (!state_valid(candidate)) {
        diagnostics_.rejected_update_count++;
        diagnostics_.covariance_failure_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    commit_locked(candidate);
    diagnostics_.accepted_update_count++;
    if (count_as_zupt) {
        diagnostics_.zupt_accepted_count++;
    }
    set_result_locked(EstimatorOperationResult::Accepted);
    return diagnostics_.last_operation_result;
}

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
