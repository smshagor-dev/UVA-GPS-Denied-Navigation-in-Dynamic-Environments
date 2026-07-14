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

namespace drone::vio {

static constexpr double kGravity = 9.81;

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
    total_drift_ = 0.0;
    initialized_ = true;
    last_vision_update_ts_ = -1.0;
    last_depth_update_ts_ = -1.0;
    logger_->info("EKF reset. p0=[{:.3f},{:.3f},{:.3f}]", p0.x(), p0.y(), p0.z());
}

// IMU Propagation  (error-state EKF prediction step)

void EKFEstimator::propagate_imu(const Eigen::Vector3d& accel_mps2,
                                 const Eigen::Vector3d& gyro_rads, double dt) {
    if (!initialized_ || dt <= 0.0 || dt > 0.5)
        return;
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
    P_ = 0.5 * (P_ + P_.transpose());
    total_drift_ = P_.diagonal().head<3>().cwiseSqrt().norm();

    timestamp_ += dt;
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
        const double mah_sq = innov.transpose() * S.ldlt().solve(innov);
        if (mah_sq > cfg_.mahal_gate)
            continue; // outlier

        // Kalman gain
        const Eigen::Matrix<double, 15, 2> K_gain = P_ * H.transpose() * S.inverse();

        // Error state correction
        const Eigen::Matrix<double, 15, 1> dx = K_gain * innov;

        // Apply correction to nominal state
        pos_ += dx.segment<3>(0);
        vel_ += dx.segment<3>(3);
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);

        // Quaternion box-plus
        const Eigen::Vector3d dtheta = dx.segment<3>(6);
        if (dtheta.norm() > 1e-8) {
            q_ = q_ * rotvec_to_quat(dtheta);
            q_.normalize();
        }

        // Joseph form covariance update (numerically stable)
        const Eigen::Matrix<double, 15, 15> I_KH =
            Eigen::Matrix<double, 15, 15>::Identity() - K_gain * H;
        P_ = I_KH * P_ * I_KH.transpose() + K_gain * R_meas * K_gain.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
        total_drift_ = P_.diagonal().head<3>().cwiseSqrt().norm();
        accepted_update = true;
    }

    if (accepted_update) {
        last_vision_update_ts_ = timestamp_;
    }
}

void EKFEstimator::update_depth(double z_depth_m, double sigma_m) {
    if (!initialized_)
        return;
    std::lock_guard lock(mtx_);

    // H = [0 0 1 | 0...] (z-position only)
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;

    const double innov = z_depth_m - pos_.z();
    const double S = H * P_ * H.transpose() + sigma_m * sigma_m;
    const auto K_gain = P_ * H.transpose() / S;
    const auto dx = K_gain * innov;

    pos_ += dx.segment<3>(0);
    vel_ += dx.segment<3>(3);
    P_ -= K_gain * H * P_;
    P_ = 0.5 * (P_ + P_.transpose());
    total_drift_ = P_.diagonal().head<3>().cwiseSqrt().norm();
    last_depth_update_ts_ = timestamp_;
}

void EKFEstimator::update_visual_pose(const Eigen::Vector3d& observed_position,
                                      const Eigen::Vector3d& observed_velocity,
                                      double sigma_position_m, double sigma_velocity_mps) {
    if (!initialized_)
        return;
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

    const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_meas;
    const Eigen::Matrix<double, 15, 6> K_gain = P_ * H.transpose() * S.inverse();
    const auto dx = K_gain * innov;

    pos_ += dx.segment<3>(0);
    vel_ += dx.segment<3>(3);
    ba_ += dx.segment<3>(9);
    bg_ += dx.segment<3>(12);

    const Eigen::Matrix<double, 15, 15> I_KH =
        Eigen::Matrix<double, 15, 15>::Identity() - K_gain * H;
    P_ = I_KH * P_ * I_KH.transpose() + K_gain * R_meas * K_gain.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    total_drift_ = P_.diagonal().head<3>().cwiseSqrt().norm();
    last_vision_update_ts_ = timestamp_;
}

void EKFEstimator::update_zupt() {
    if (!initialized_)
        return;
    std::lock_guard lock(mtx_);

    // Zero velocity update: z = [0,0,0], H = [0 I_3 0 ...]
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * 1e-4;
    const Eigen::Vector3d innov = -vel_;

    const Eigen::Matrix3d S = H * P_ * H.transpose() + R_meas;
    const Eigen::Matrix<double, 15, 3> K_gain = P_ * H.transpose() * S.inverse();
    const auto dx = K_gain * innov;

    vel_ += dx.segment<3>(3);
    vel_.setZero();
    P_ -= K_gain * H * P_;
    P_ = 0.5 * (P_ + P_.transpose());
    total_drift_ = P_.diagonal().head<3>().cwiseSqrt().norm();
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

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
