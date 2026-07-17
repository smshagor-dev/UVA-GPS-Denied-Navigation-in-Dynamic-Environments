// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// EKFEstimator.cpp    Error-State EKF implementation
// Drone Swarm Sensor Fusion  |  Phase 2

#include "vio/EKFEstimator.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace drone::vio {

static constexpr double kGravity = 9.81;
static constexpr double kCovarianceFloor = 1.0e-12;
static constexpr double kMaxCorrectionNorm = 100.0;

namespace {

template <typename Derived>
bool matrix_is_finite(const Eigen::MatrixBase<Derived>& matrix) {
    return matrix.array().isFinite().all();
}

} // namespace

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
    diagnostics_.minimum_covariance_diagonal_seen = kCovarianceFloor;
    diagnostics_.covariance_min_diagonal = kCovarianceFloor;
    diagnostics_.health_state = EstimatorHealthState::INITIALIZING;
}

void EKFEstimator::reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                         const Eigen::Vector3d& v0) {
    std::lock_guard lock(mtx_);
    pos_ = p0;
    vel_ = v0;
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
    estimated_position_uncertainty_m_ = 0.0;
    initialized_ = true;
    last_vision_update_ts_ = -1.0;
    last_depth_update_ts_ = -1.0;
    diagnostics_ = {};
    diagnostics_.covariance_min_diagonal = P_.diagonal().minCoeff();
    diagnostics_.minimum_covariance_diagonal_seen = diagnostics_.covariance_min_diagonal;
    diagnostics_.health_state = EstimatorHealthState::INITIALIZING;
    logger_->info("EKF reset. p0=[{:.3f},{:.3f},{:.3f}]", p0.x(), p0.y(), p0.z());
}

// IMU Propagation  (error-state EKF prediction step)

void EKFEstimator::propagate_imu(const Eigen::Vector3d& accel_mps2,
                                 const Eigen::Vector3d& gyro_rads, double dt) {
    if (!initialized_)
        return;
    if ((validation_cfg_.reject_non_finite_measurements &&
         (!matrix_is_finite(accel_mps2) || !matrix_is_finite(gyro_rads) || !std::isfinite(dt)))) {
        std::lock_guard lock(mtx_);
        mark_invalid_input("non-finite imu sample rejected");
        return;
    }
    if (dt <= 0.0 ||
        (validation_cfg_.require_monotonic_timestamps && dt <= 0.0)) {
        std::lock_guard lock(mtx_);
        ++diagnostics_.timestamp_rejection_count;
        mark_measurement_rejected(EstimatorSensorType::IMU, "non-monotonic imu timestamp");
        return;
    }
    if (dt > validation_cfg_.max_imu_dt_s) {
        std::lock_guard lock(mtx_);
        ++diagnostics_.timestamp_rejection_count;
        mark_measurement_rejected(EstimatorSensorType::IMU, "imu dt exceeds configured limit");
        return;
    }
    const auto started_at = std::chrono::steady_clock::now();
    std::lock_guard lock(mtx_);

    // Remove bias estimates
    const Eigen::Vector3d a = accel_mps2 - ba_;
    const Eigen::Vector3d w = gyro_rads - bg_;

    const Eigen::Matrix3d R = q_.toRotationMatrix();

    //  Nominal state integration (mid-point / Runge-Kutta 2)
    const Eigen::Vector3d g_world{0, 0, -kGravity};

    // Half-step quaternion
    const Eigen::Quaterniond q_half = propagate_quat(q_, w, dt * 0.5);
    const Eigen::Matrix3d R_half = q_half.toRotationMatrix();

    // Position and velocity (mid-point)
    const Eigen::Vector3d a_world = R_half * a + g_world;
    pos_ += vel_ * dt + 0.5 * a_world * dt * dt;
    vel_ += a_world * dt;

    // Full-step quaternion
    q_ = propagate_quat(q_, w, dt);
    q_.normalize();

    //  Error-state covariance propagation â”€
    const FMat F = compute_F(a, R, dt);
    const GMat G = compute_G(R);

    // Discrete-time noise: Q_d = G * Q_c * G^T * dt
    const CovMat Q_d = (G * Q_imu_ * G.transpose()) * dt;

    P_ = F * P_ * F.transpose() + Q_d;

    // Symmetrize to prevent numerical drift
    finalize_covariance_locked();

    timestamp_ += dt;
    ++diagnostics_.propagation_count;
    accumulate_propagation_latency(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started_at)
            .count());
    update_health_locked();
}

// Vision Update  (feature reprojection)

void EKFEstimator::update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                                 const std::vector<Eigen::Vector3d>& p_world,
                                 const Eigen::Matrix3d& K) {
    if (!initialized_)
        return;
    if (z_pixels.size() != p_world.size() || z_pixels.empty())
        return;

    std::lock_guard lock(mtx_);

    const Eigen::Matrix3d R = q_.toRotationMatrix();
    const double fx = K(0, 0), fy = K(1, 1);
    const double cx = K(0, 2), cy = K(1, 2);
    const double sigma2 = cfg_.sigma_px * cfg_.sigma_px;

    bool accepted_update = false;
    for (size_t i = 0; i < z_pixels.size(); ++i) {
        if ((validation_cfg_.reject_non_finite_measurements &&
             (!matrix_is_finite(z_pixels[i]) || !matrix_is_finite(p_world[i]) ||
              !matrix_is_finite(K)))) {
            mark_invalid_input("non-finite vision feature input rejected");
            return;
        }
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
        if (apply_linear_update(H, innov, R_meas, EstimatorSensorType::VISION_FEATURE,
                                "vision_feature")) {
            accepted_update = true;
        }
    }

    if (accepted_update) {
        last_vision_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_depth(double z_depth_m, double sigma_m) {
    if (!initialized_)
        return;
    if (validation_cfg_.reject_non_finite_measurements &&
        (!std::isfinite(z_depth_m) || !std::isfinite(sigma_m))) {
        std::lock_guard lock(mtx_);
        mark_invalid_input("non-finite depth measurement rejected");
        return;
    }
    if (!validate_measurement_noise(sigma_m, "depth")) {
        std::lock_guard lock(mtx_);
        mark_invalid_input("invalid depth covariance rejected");
        return;
    }
    std::lock_guard lock(mtx_);

    // H = [0 0 1 | 0...] (z-position only)
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;

    const Eigen::Matrix<double, 1, 1> innovation =
        (Eigen::Matrix<double, 1, 1>() << (z_depth_m - pos_.z())).finished();
    const Eigen::Matrix<double, 1, 1> R_meas =
        (Eigen::Matrix<double, 1, 1>() << sigma_m * sigma_m).finished();
    if (apply_linear_update(H, innovation, R_meas, EstimatorSensorType::DEPTH, "depth")) {
        last_depth_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_visual_pose(const Eigen::Vector3d& observed_position,
                                      const Eigen::Vector3d& observed_velocity,
                                      double sigma_position_m, double sigma_velocity_mps) {
    if (!initialized_)
        return;
    if (validation_cfg_.reject_non_finite_measurements &&
        (!matrix_is_finite(observed_position) || !matrix_is_finite(observed_velocity) ||
         !std::isfinite(sigma_position_m) || !std::isfinite(sigma_velocity_mps))) {
        std::lock_guard lock(mtx_);
        mark_invalid_input("non-finite visual pose input rejected");
        return;
    }
    if (!validate_measurement_noise(sigma_position_m, "visual_pose_position") ||
        !validate_measurement_noise(sigma_velocity_mps, "visual_pose_velocity")) {
        std::lock_guard lock(mtx_);
        mark_invalid_input("invalid visual pose covariance rejected");
        return;
    }
    std::lock_guard lock(mtx_);

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
    if (apply_linear_update(H, innov, R_meas, EstimatorSensorType::VISUAL_POSE, "visual_pose")) {
        last_vision_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_zupt() {
    if (!initialized_)
        return;
    std::lock_guard lock(mtx_);

    // Zero velocity update: z = [0,0,0], H = [0 I_3 0 ...]
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * 1e-6;
    const Eigen::Vector3d innov = -vel_;
    apply_linear_update(H, innov, R_meas, EstimatorSensorType::ZUPT, "zupt",
                        true, true, true, false);
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
    est.pos_std = P_.diagonal().head<3>().cwiseMax(kCovarianceFloor).cwiseSqrt();
    est.drift_m = estimated_position_uncertainty_m_;

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

EstimatorDiagnostics EKFEstimator::diagnostics() const {
    std::lock_guard lock(mtx_);
    return diagnostics_;
}

void EKFEstimator::set_validation_config(EstimatorValidationConfig config) {
    std::lock_guard lock(mtx_);
    validation_cfg_ = std::move(config);
    update_health_locked();
}

void EKFEstimator::note_timestamp_violation(const std::string& reason) {
    std::lock_guard lock(mtx_);
    ++diagnostics_.timestamp_rejection_count;
    mark_measurement_rejected(EstimatorSensorType::IMU, reason);
}

void EKFEstimator::note_dropped_sensor_event(const std::string& reason) {
    std::lock_guard lock(mtx_);
    ++diagnostics_.dropped_sensor_event_count;
    diagnostics_.last_rejection_reason = reason;
    update_health_locked();
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

template <int N>
bool EKFEstimator::apply_linear_update(const Eigen::Matrix<double, N, 15>& H,
                                       const Eigen::Matrix<double, N, 1>& innovation,
                                       const Eigen::Matrix<double, N, N>& R_meas,
                                       EstimatorSensorType sensor_type, const char* sensor_name,
                                       bool apply_velocity, bool apply_biases,
                                       bool apply_attitude, bool enable_gating) {
    const auto started_at = std::chrono::steady_clock::now();
    if (!matrix_is_finite(H) || !matrix_is_finite(innovation) || !matrix_is_finite(R_meas)) {
        mark_invalid_input(std::string(sensor_name) + " update had non-finite matrices");
        return false;
    }

    const Eigen::Matrix<double, N, N> S = H * P_ * H.transpose() + R_meas;
    if (!matrix_is_finite(S)) {
        mark_measurement_rejected(sensor_type, std::string(sensor_name) + " innovation covariance invalid");
        return false;
    }
    Eigen::LDLT<Eigen::Matrix<double, N, N>> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        mark_measurement_rejected(sensor_type, std::string(sensor_name) + " innovation solve failed");
        return false;
    }

    const double innovation_magnitude = innovation.norm();
    const double mahal_sq = innovation.dot(ldlt.solve(innovation));
    if (!std::isfinite(mahal_sq)) {
        mark_measurement_rejected(sensor_type, std::string(sensor_name) + " mahalanobis invalid");
        return false;
    }
    if (enable_gating && mahal_sq > cfg_.mahal_gate) {
        mark_measurement_rejected(sensor_type, std::string(sensor_name) + " innovation gated");
        diagnostics_.last_innovation_magnitude = innovation_magnitude;
        diagnostics_.last_mahalanobis_distance = std::sqrt(std::max(0.0, mahal_sq));
        return false;
    }

    const Eigen::Matrix<double, 15, N> K_gain =
        ldlt.solve(H * P_.transpose()).transpose();
    const Eigen::Matrix<double, 15, 1> dx = K_gain * innovation;
    if (!matrix_is_finite(dx) || dx.norm() > kMaxCorrectionNorm) {
        mark_measurement_rejected(sensor_type, std::string(sensor_name) + " correction invalid");
        return false;
    }

    pos_ += dx.segment<3>(0);
    if (apply_velocity) {
        vel_ += dx.segment<3>(3);
    }
    if (apply_biases) {
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);
    }
    if (apply_attitude) {
        const Eigen::Vector3d dtheta = dx.segment<3>(6);
        if (dtheta.norm() > 1e-8) {
            q_ = q_ * rotvec_to_quat(dtheta);
            q_.normalize();
        }
    }

    const Eigen::Matrix<double, 15, 15> I_KH =
        Eigen::Matrix<double, 15, 15>::Identity() - K_gain * H;
    P_ = I_KH * P_ * I_KH.transpose() + K_gain * R_meas * K_gain.transpose();
    finalize_covariance_locked();

    mark_measurement_accepted(sensor_type, innovation_magnitude, std::sqrt(std::max(0.0, mahal_sq)));
    accumulate_measurement_latency(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started_at)
            .count());
    update_health_locked();
    return true;
}

bool EKFEstimator::validate_measurement_noise(double sigma_m, const char* sensor_name) {
    if (!std::isfinite(sigma_m) || sigma_m <= 0.0) {
        if (logger_) {
            logger_->warn("{} measurement rejected due to invalid sigma {}", sensor_name, sigma_m);
        }
        return false;
    }
    return true;
}

bool EKFEstimator::validate_state_finite() const {
    return matrix_is_finite(pos_) && matrix_is_finite(vel_) && matrix_is_finite(ba_) &&
           matrix_is_finite(bg_) && std::isfinite(q_.w()) && std::isfinite(q_.x()) &&
           std::isfinite(q_.y()) && std::isfinite(q_.z());
}

bool EKFEstimator::validate_covariance_finite() const {
    return matrix_is_finite(P_);
}

void EKFEstimator::finalize_covariance_locked() {
    P_ = 0.5 * (P_ + P_.transpose());
    for (int i = 0; i < P_.rows(); ++i) {
        if (!std::isfinite(P_(i, i)) || P_(i, i) < kCovarianceFloor) {
            P_(i, i) = kCovarianceFloor;
        }
    }
    diagnostics_.covariance_symmetry_error = (P_ - P_.transpose()).cwiseAbs().maxCoeff();
    diagnostics_.max_covariance_asymmetry =
        std::max(diagnostics_.max_covariance_asymmetry, diagnostics_.covariance_symmetry_error);
    diagnostics_.covariance_min_diagonal = P_.diagonal().minCoeff();
    diagnostics_.minimum_covariance_diagonal_seen =
        diagnostics_.propagation_count == 0 &&
                diagnostics_.accepted_updates[sensor_index(EstimatorSensorType::VISION_FEATURE)] == 0 &&
                diagnostics_.accepted_updates[sensor_index(EstimatorSensorType::VISUAL_POSE)] == 0 &&
                diagnostics_.accepted_updates[sensor_index(EstimatorSensorType::DEPTH)] == 0 &&
                diagnostics_.accepted_updates[sensor_index(EstimatorSensorType::ZUPT)] == 0
            ? diagnostics_.covariance_min_diagonal
            : std::min(diagnostics_.minimum_covariance_diagonal_seen,
                       diagnostics_.covariance_min_diagonal);
    estimated_position_uncertainty_m_ =
        P_.diagonal().head<3>().cwiseMax(kCovarianceFloor).cwiseSqrt().norm();
    diagnostics_.has_non_finite_covariance = !validate_covariance_finite();
    diagnostics_.has_non_finite_state = !validate_state_finite();
}

void EKFEstimator::mark_measurement_rejected(EstimatorSensorType sensor_type, std::string reason) {
    ++diagnostics_.rejected_updates[sensor_index(sensor_type)];
    diagnostics_.last_rejection_reason = std::move(reason);
    update_health_locked();
}

void EKFEstimator::mark_measurement_accepted(EstimatorSensorType sensor_type,
                                             double innovation_magnitude,
                                             double mahalanobis_distance) {
    ++diagnostics_.accepted_updates[sensor_index(sensor_type)];
    diagnostics_.last_innovation_magnitude = innovation_magnitude;
    diagnostics_.last_mahalanobis_distance = mahalanobis_distance;
    diagnostics_.last_rejection_reason = "none";
}

void EKFEstimator::mark_invalid_input(std::string reason) {
    ++diagnostics_.invalid_input_count;
    diagnostics_.last_rejection_reason = std::move(reason);
    update_health_locked();
}

void EKFEstimator::update_health_locked() {
    diagnostics_.has_non_finite_state = !validate_state_finite();
    diagnostics_.has_non_finite_covariance = !validate_covariance_finite();
    if (diagnostics_.has_non_finite_state || diagnostics_.has_non_finite_covariance) {
        diagnostics_.health_state = EstimatorHealthState::INVALID;
        return;
    }
    if (!initialized_) {
        diagnostics_.health_state = EstimatorHealthState::INITIALIZING;
        return;
    }
    if (diagnostics_.covariance_min_diagonal <= kCovarianceFloor ||
        diagnostics_.covariance_symmetry_error > 1.0e-6) {
        diagnostics_.health_state = EstimatorHealthState::NUMERICAL_WARNING;
        return;
    }
    const uint64_t rejected_total = std::accumulate(diagnostics_.rejected_updates.begin(),
                                                    diagnostics_.rejected_updates.end(), uint64_t{0});
    if (rejected_total > 0 && diagnostics_.propagation_count == 0) {
        diagnostics_.health_state = EstimatorHealthState::REJECTING_MEASUREMENTS;
        return;
    }
    if (estimated_position_uncertainty_m_ > 1.8 || diagnostics_.invalid_input_count > 0) {
        diagnostics_.health_state = EstimatorHealthState::DEGRADED;
        return;
    }
    diagnostics_.health_state = EstimatorHealthState::NOMINAL;
}

void EKFEstimator::accumulate_propagation_latency(double latency_us) {
    diagnostics_.max_update_latency_us = std::max(diagnostics_.max_update_latency_us, latency_us);
    const double count = static_cast<double>(std::max<uint64_t>(1, diagnostics_.propagation_count));
    diagnostics_.average_propagation_latency_us =
        ((diagnostics_.average_propagation_latency_us * (count - 1.0)) + latency_us) / count;
}

void EKFEstimator::accumulate_measurement_latency(double latency_us) {
    diagnostics_.max_update_latency_us = std::max(diagnostics_.max_update_latency_us, latency_us);
    const uint64_t accepted_total =
        std::accumulate(diagnostics_.accepted_updates.begin(), diagnostics_.accepted_updates.end(),
                        uint64_t{0});
    const double count = static_cast<double>(std::max<uint64_t>(1, accepted_total));
    diagnostics_.average_measurement_latency_us =
        ((diagnostics_.average_measurement_latency_us * (count - 1.0)) + latency_us) / count;
}

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
