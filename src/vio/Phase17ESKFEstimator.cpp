#include "vio/Phase17ESKFEstimator.hpp"

#include "vio/MeasurementEnvelope.hpp"
#include "vio/MsckfMarginalization.hpp"
#include "vio/MsckfRetirementTransaction.hpp"
#include "vio/StateEstimator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <boost/math/distributions/chi_squared.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace drone::vio {

namespace {

constexpr double kGravity = 9.81;
constexpr double kFejFeatureQuantizationScale = 1.0e6;
constexpr double kMsckfTimestampQuantizationScale = 1.0e6;
constexpr double kTriangulationEigenvalueTolerance = 1.0e-9;
constexpr double kMsckfProjectionTolerance = 1.0e-9;
constexpr double kInnovationConditionNumberLimit = 1.0e12;
constexpr double kInnovationEigenvalueTolerance = 1.0e-12;
constexpr double kNullspaceValidationTolerance = 1.0e-8;

} // namespace

Phase17ESKFEstimator::Phase17ESKFEstimator(EKFConfig cfg) : cfg_(cfg) {
    logger_ = spdlog::get("EKF");
    if (!logger_) {
        logger_ = spdlog::stdout_color_mt("EKF");
    }

    Q_imu_.setZero();
    Q_imu_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * cfg_.sigma_na * cfg_.sigma_na;
    Q_imu_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * cfg_.sigma_ng * cfg_.sigma_ng;
    Q_imu_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * cfg_.sigma_nba * cfg_.sigma_nba;
    Q_imu_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * cfg_.sigma_nbg * cfg_.sigma_nbg;
    refresh_config_validity_locked();
    reset_diagnostics_locked();
}

void Phase17ESKFEstimator::reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                                 const Eigen::Vector3d& v0) {
    std::lock_guard lock(mtx_);
    reset_diagnostics_locked();
    reset_fej_locked();
    reset_msckf_locked();
    reset_stationary_detector_locked();
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

    P_.setZero();
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * cfg_.init_pos_std * cfg_.init_pos_std;
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * cfg_.init_vel_std * cfg_.init_vel_std;
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * cfg_.init_att_std * cfg_.init_att_std;
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * cfg_.init_ba_std * cfg_.init_ba_std;
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * cfg_.init_bg_std * cfg_.init_bg_std;
    augmented_covariance_ = P_;

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
}

void Phase17ESKFEstimator::propagate_imu(const Eigen::Vector3d& accel_mps2,
                                         const Eigen::Vector3d& gyro_rads, double dt) {
    std::lock_guard lock(mtx_);
    (void)propagate_imu_locked(accel_mps2, gyro_rads, dt, true);
}

EstimatorOperationResult Phase17ESKFEstimator::process_imu_measurement(
    const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rads, double timestamp_s) {
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
        update_stationary_detector_locked(accel_mps2, gyro_rads, timestamp_s);
        maybe_apply_automatic_zupt_locked(timestamp_s);
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
        update_stationary_detector_locked(accel_mps2, gyro_rads, timestamp_s);
        maybe_apply_automatic_zupt_locked(timestamp_s);
    }
    return result;
}

void Phase17ESKFEstimator::update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                                         const std::vector<Eigen::Vector3d>& p_world,
                                         const Eigen::Matrix3d& K) {
    std::lock_guard lock(mtx_);
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return;
    }
    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        return;
    }
    if (z_pixels.size() != p_world.size() || z_pixels.empty()) {
        note_rejection_locked(EstimatorOperationResult::RejectedDimensionMismatch);
        return;
    }
    if (!state_valid(snapshot_locked())) {
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return;
    }

    const Eigen::Matrix3d R = q_.toRotationMatrix();
    const bool use_fej = fej_enabled_locked();
    const double fx = K(0, 0);
    const double fy = K(1, 1);
    const double cx = K(0, 2);
    const double cy = K(1, 2);
    const double sigma2 = cfg_.sigma_px * cfg_.sigma_px;
    bool accepted_update = false;
    const uint64_t observation_epoch = ++fej_observation_epoch_;

    for (size_t i = 0; i < z_pixels.size(); ++i) {
        const Eigen::Vector3d p_c = R.transpose() * (p_world[i] - pos_);
        if (p_c.z() < 0.1) {
            continue;
        }

        Eigen::Matrix3d jacobian_rotation = R;
        Eigen::Vector3d jacobian_position = pos_;
        if (use_fej) {
            FejFeatureSnapshot* fej_snapshot = get_or_create_fej_snapshot_locked(p_world[i]);
            if (!fej_snapshot || !fej_snapshot_valid_locked(*fej_snapshot)) {
                ++diagnostics_.fej_validation_failures;
                if (validation_cfg_.fej.validation_checks) {
                    diagnostics_.numerical_failure_count++;
                    note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
                    continue;
                }
            } else {
                fej_snapshot->last_observed_epoch = observation_epoch;
                jacobian_rotation = fej_snapshot->first_orientation.toRotationMatrix();
                jacobian_position = fej_snapshot->first_position_m;
                ++diagnostics_.fej_jacobian_evaluations;
            }
        }

        const double Xc = p_c.x();
        const double Yc = p_c.y();
        const double Zc = p_c.z();
        const double u_hat = fx * Xc / Zc + cx;
        const double v_hat = fy * Yc / Zc + cy;

        const Eigen::Vector3d p_c_jacobian =
            jacobian_rotation.transpose() * (p_world[i] - jacobian_position);
        if (!p_c_jacobian.array().isFinite().all() || p_c_jacobian.z() < 0.1) {
            if (use_fej) {
                ++diagnostics_.fej_validation_failures;
            }
            continue;
        }

        Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
        Eigen::Matrix<double, 2, 3> J_proj;
        J_proj << fx / p_c_jacobian.z(), 0.0,
            -fx * p_c_jacobian.x() / (p_c_jacobian.z() * p_c_jacobian.z()), 0.0,
            fy / p_c_jacobian.z(), -fy * p_c_jacobian.y() / (p_c_jacobian.z() * p_c_jacobian.z());
        H.block<2, 3>(0, 0) = J_proj * (-jacobian_rotation.transpose());
        H.block<2, 3>(0, 6) =
            J_proj * jacobian_rotation.transpose() * skew(p_world[i] - jacobian_position);

        const Eigen::Matrix2d R_meas = Eigen::Matrix2d::Identity() * sigma2;
        const Eigen::Vector2d innov{z_pixels[i].x() - u_hat, z_pixels[i].y() - v_hat};
        const Eigen::Matrix2d S = H * P_ * H.transpose() + R_meas;
        const Eigen::LDLT<Eigen::Matrix2d> gate_ldlt(S);
        if (gate_ldlt.info() != Eigen::Success) {
            diagnostics_.numerical_failure_count++;
            note_rejection_locked(EstimatorOperationResult::FailedFactorization);
            continue;
        }
        const double mah_sq = innov.transpose() * gate_ldlt.solve(innov);
        if (mah_sq > cfg_.mahal_gate) {
            continue;
        }

        const auto result = apply_error_state_update_locked(
            innov, H, R_meas,
            [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
                candidate.pos += dx.segment<3>(0);
                candidate.vel += dx.segment<3>(3);
                candidate.ba += dx.segment<3>(9);
                candidate.bg += dx.segment<3>(12);
            },
            true);
        accepted_update = accepted_update || (result == EstimatorOperationResult::Accepted);
    }

    if (accepted_update) {
        last_vision_update_ts_ = timestamp_;
        const uint64_t state_id = capture_or_get_msckf_camera_state_locked(
            last_accepted_imu_timestamp_s_.value_or(timestamp_));
        if (state_id != 0u) {
            record_feature_observations_locked(z_pixels, p_world, K, state_id);
            try_initialize_triangulated_landmarks_locked(K);
            try_apply_msckf_feature_updates_locked(K, state_id);
            while (msckf_state_order_.size() > msckf_cfg_.max_camera_states) {
                const auto before = msckf_state_order_.size();
                evict_msckf_state_locked();
                if (msckf_state_order_.size() >= before) {
                    break;
                }
            }
        }
    }
    if (use_fej) {
        release_inactive_fej_snapshots_locked(observation_epoch);
    }
}

void Phase17ESKFEstimator::update_depth(double z_depth_m, double sigma_m) {
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
        innov, H, R_meas,
        [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
        },
        false);
    if (result == EstimatorOperationResult::Accepted) {
        last_depth_update_ts_ = timestamp_;
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void Phase17ESKFEstimator::update_visual_pose(const Eigen::Vector3d& observed_position,
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
        innov, H, R_meas,
        [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
            candidate.ba += dx.segment<3>(9);
            candidate.bg += dx.segment<3>(12);
        },
        false);
    if (result == EstimatorOperationResult::Accepted) {
        last_vision_update_ts_ = timestamp_;
        maybe_capture_msckf_camera_state_locked(
            last_accepted_imu_timestamp_s_.value_or(timestamp_));
    }
}

void Phase17ESKFEstimator::update_zupt() {
    std::lock_guard lock(mtx_);
    if (!initialized_) {
        note_rejection_locked(EstimatorOperationResult::RejectedNotInitialized);
        return;
    }
    (void)update_zupt_locked(effective_zupt_velocity_noise_mps());
}

PoseEstimate Phase17ESKFEstimator::state() const {
    std::lock_guard lock(mtx_);
    PoseEstimate est;
    est.timestamp = timestamp_;
    est.position = pos_;
    est.velocity = vel_;
    est.orientation = q_;
    est.accel_bias = ba_;
    est.gyro_bias = bg_;
    est.pos_std = P_.diagonal().head<3>().cwiseMax(0.0).cwiseSqrt();
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

void Phase17ESKFEstimator::configure_validation(const EstimatorValidationConfig& cfg) {
    std::lock_guard lock(mtx_);
    validation_cfg_ = cfg;
    refresh_config_validity_locked();
    reset_fej_locked();
    reset_msckf_locked();
    reset_stationary_detector_locked();
    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
    }
}

void Phase17ESKFEstimator::configure_msckf(const MsckfConfig& cfg) {
    std::lock_guard lock(mtx_);
    msckf_cfg_ = cfg;
    refresh_config_validity_locked();
    clear_msckf_diagnostics_locked();
    clear_triangulation_diagnostics_locked();
    refresh_triangulation_diagnostics_locked();
    if (!validation_cfg_valid_) {
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
    }
}

EstimatorValidationConfig Phase17ESKFEstimator::validation_config() const {
    std::lock_guard lock(mtx_);
    return validation_cfg_;
}

MsckfConfig Phase17ESKFEstimator::msckf_config() const {
    std::lock_guard lock(mtx_);
    return msckf_cfg_;
}

EKFDiagnostics Phase17ESKFEstimator::diagnostics() const {
    std::lock_guard lock(mtx_);
    return diagnostics_;
}

CovMat Phase17ESKFEstimator::covariance() const {
    std::lock_guard lock(mtx_);
    return P_;
}

Eigen::Matrix3d
Phase17ESKFEstimator::attitude_reset_jacobian(const Eigen::Vector3d& injected_delta_theta) {
    return Eigen::Matrix3d::Identity() - 0.5 * skew(injected_delta_theta);
}

EstimatorOperationResult Phase17ESKFEstimator::inject_error_for_test(const ErrorVec& dx) {
    std::lock_guard lock(mtx_);
    return inject_error_state_locked(dx);
}

std::vector<uint64_t> Phase17ESKFEstimator::fej_snapshot_ids_for_test() const {
    std::lock_guard lock(mtx_);
    std::vector<uint64_t> ids;
    ids.reserve(fej_snapshots_.size());
    for (const auto& [key, snapshot] : fej_snapshots_) {
        (void)key;
        ids.push_back(snapshot.snapshot_id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<uint64_t> Phase17ESKFEstimator::msckf_state_ids_for_test() const {
    std::lock_guard lock(mtx_);
    return {msckf_state_order_.begin(), msckf_state_order_.end()};
}

std::vector<double> Phase17ESKFEstimator::msckf_state_timestamps_for_test() const {
    std::lock_guard lock(mtx_);
    std::vector<double> timestamps;
    timestamps.reserve(msckf_state_order_.size());
    for (const auto state_id : msckf_state_order_) {
        if (const auto it = msckf_camera_states_.find(state_id); it != msckf_camera_states_.end()) {
            timestamps.push_back(it->second.timestamp_s);
        }
    }
    return timestamps;
}

std::optional<double>
Phase17ESKFEstimator::msckf_state_timestamp_for_id_for_test(uint64_t state_id) const {
    std::lock_guard lock(mtx_);
    if (const auto it = msckf_camera_states_.find(state_id); it != msckf_camera_states_.end()) {
        return it->second.timestamp_s;
    }
    return std::nullopt;
}

std::optional<uint64_t>
Phase17ESKFEstimator::msckf_state_id_for_timestamp_for_test(double timestamp_s) const {
    std::lock_guard lock(mtx_);
    const auto key = make_msckf_timestamp_key(timestamp_s);
    if (const auto it = msckf_timestamp_to_state_id_.find(key);
        it != msckf_timestamp_to_state_id_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::optional<Phase17ESKFEstimator::MsckfCameraStateForTest>
Phase17ESKFEstimator::msckf_camera_state_for_test(uint64_t state_id) const {
    std::lock_guard lock(mtx_);
    if (const auto it = msckf_camera_states_.find(state_id); it != msckf_camera_states_.end()) {
        return MsckfCameraStateForTest{
            .state_id = it->second.state_id,
            .timestamp_s = it->second.timestamp_s,
            .position_m = it->second.position_m,
            .orientation = it->second.orientation,
            .fej_position_m = it->second.fej_position_m,
            .fej_orientation = it->second.fej_orientation,
        };
    }
    return std::nullopt;
}

uint64_t Phase17ESKFEstimator::feature_track_count_for_test() const {
    std::lock_guard lock(mtx_);
    return feature_tracks_.size();
}

uint64_t Phase17ESKFEstimator::active_landmark_count_for_test() const {
    std::lock_guard lock(mtx_);
    uint64_t count = 0;
    for (const auto& [key, track] : feature_tracks_) {
        (void)key;
        count += track.landmark_initialized ? 1u : 0u;
    }
    return count;
}

std::vector<uint64_t> Phase17ESKFEstimator::feature_track_ids_for_test() const {
    std::lock_guard lock(mtx_);
    std::vector<uint64_t> ids;
    ids.reserve(feature_tracks_.size());
    for (const auto& [key, track] : feature_tracks_) {
        (void)key;
        ids.push_back(track.track_id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::optional<uint64_t> Phase17ESKFEstimator::feature_track_id_for_feature_for_test(
    const Eigen::Vector3d& feature_world) const {
    std::lock_guard lock(mtx_);
    const auto key = make_fej_feature_key(feature_world);
    if (const auto it = feature_tracks_.find(key); it != feature_tracks_.end()) {
        return it->second.track_id;
    }
    return std::nullopt;
}

std::optional<size_t> Phase17ESKFEstimator::feature_track_observation_count_for_feature_for_test(
    const Eigen::Vector3d& feature_world) const {
    std::lock_guard lock(mtx_);
    const auto key = make_fej_feature_key(feature_world);
    if (const auto it = feature_tracks_.find(key); it != feature_tracks_.end()) {
        return it->second.observations.size();
    }
    return std::nullopt;
}

std::optional<Eigen::Vector3d> Phase17ESKFEstimator::triangulated_landmark_for_feature_for_test(
    const Eigen::Vector3d& feature_world) const {
    std::lock_guard lock(mtx_);
    const auto key = make_fej_feature_key(feature_world);
    if (const auto it = feature_tracks_.find(key);
        it != feature_tracks_.end() && it->second.landmark_initialized) {
        return it->second.landmark_world;
    }
    return std::nullopt;
}

std::optional<Phase17ESKFEstimator::FeatureUpdateLinearizationForTest>
Phase17ESKFEstimator::feature_update_linearization_for_test(const Eigen::Vector3d& feature_world,
                                                            const Eigen::Matrix3d& K) const {
    std::lock_guard lock(mtx_);
    const auto key = make_fej_feature_key(feature_world);
    const auto it = feature_tracks_.find(key);
    if (it == feature_tracks_.end()) {
        return std::nullopt;
    }
    FeatureUpdateBuildResult build_result;
    if (!const_cast<Phase17ESKFEstimator*>(this)->build_feature_update_linearization_locked(
            it->second, K, build_result)) {
        return std::nullopt;
    }

    FeatureUpdateLinearizationForTest out;
    out.raw_residual_norm = build_result.raw_residual_norm;
    const Eigen::Index measurement_dim =
        static_cast<Eigen::Index>(build_result.linearizations.size() * 2u);
    const auto layout = build_augmented_state_layout_locked();
    out.residual = Eigen::VectorXd::Zero(measurement_dim);
    out.H_state =
        Eigen::MatrixXd::Zero(measurement_dim, static_cast<Eigen::Index>(layout.total_dim));
    out.H_feature = Eigen::MatrixXd::Zero(measurement_dim, 3);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(build_result.linearizations.size());
         ++i) {
        const auto& linearization = build_result.linearizations[static_cast<size_t>(i)];
        out.state_ids.push_back(linearization.state_id);
        out.residual.segment<2>(i * 2) = linearization.residual;
        out.H_state.block(i * 2, 0, 2, out.H_state.cols()) = linearization.H_state;
        out.H_feature.block(i * 2, 0, 2, 3) = linearization.H_feature;
    }
    return out;
}

std::optional<Phase17ESKFEstimator::ProjectedFeatureUpdateForTest>
Phase17ESKFEstimator::projected_feature_update_for_test(const Eigen::Vector3d& feature_world,
                                                        const Eigen::Matrix3d& K) const {
    std::lock_guard lock(mtx_);
    const auto key = make_fej_feature_key(feature_world);
    const auto it = feature_tracks_.find(key);
    if (it == feature_tracks_.end()) {
        return std::nullopt;
    }
    FeatureUpdateBuildResult build_result;
    if (!const_cast<Phase17ESKFEstimator*>(this)->build_feature_update_linearization_locked(
            it->second, K, build_result)) {
        return std::nullopt;
    }
    ProjectedMeasurementSystem projected;
    uint64_t rank = 0;
    if (!const_cast<Phase17ESKFEstimator*>(this)->project_feature_nullspace_locked(
            build_result.linearizations, projected, rank)) {
        return std::nullopt;
    }

    ProjectedFeatureUpdateForTest out;
    out.residual = projected.residual;
    out.H = projected.H;
    out.R = projected.R;
    out.rank = projected.rank;
    out.annihilation_norm = projected.annihilation_norm;
    out.orthogonality_error = projected.orthogonality_error;
    return out;
}

Eigen::MatrixXd Phase17ESKFEstimator::augmented_covariance_for_test() const {
    std::lock_guard lock(mtx_);
    return augmented_covariance_;
}

uint64_t Phase17ESKFEstimator::capture_msckf_camera_state_for_test() {
    std::lock_guard lock(mtx_);
    return capture_or_get_msckf_camera_state_locked(
        last_accepted_imu_timestamp_s_.value_or(timestamp_));
}

bool Phase17ESKFEstimator::process_msckf_observations_for_test(
    uint64_t state_id, const std::vector<Eigen::Vector2d>& z_pixels,
    const std::vector<Eigen::Vector3d>& p_world, const Eigen::Matrix3d& K) {
    std::lock_guard lock(mtx_);
    if (state_id == 0u || msckf_camera_states_.find(state_id) == msckf_camera_states_.end()) {
        note_rejection_locked(EstimatorOperationResult::RejectedDimensionMismatch);
        return false;
    }
    record_feature_observations_locked(z_pixels, p_world, K, state_id);
    try_initialize_triangulated_landmarks_locked(K);
    try_apply_msckf_feature_updates_locked(K, state_id);
    return true;
}

void Phase17ESKFEstimator::corrupt_first_fej_snapshot_for_test() {
    std::lock_guard lock(mtx_);
    if (fej_snapshots_.empty()) {
        return;
    }
    auto it = fej_snapshots_.begin();
    it->second.first_orientation = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
}

void Phase17ESKFEstimator::corrupt_first_msckf_fej_clone_for_test() {
    std::lock_guard lock(mtx_);
    if (msckf_state_order_.empty()) {
        return;
    }
    const uint64_t state_id = msckf_state_order_.front();
    auto it = msckf_camera_states_.find(state_id);
    if (it == msckf_camera_states_.end()) {
        return;
    }
    it->second.fej_orientation = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
}

Eigen::Matrix3d Phase17ESKFEstimator::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d out;
    out << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return out;
}

Phase17ESKFEstimator::PropagationModel
Phase17ESKFEstimator::build_propagation_model(const Eigen::Vector3d& accel_body,
                                              const Eigen::Vector3d& omega_body,
                                              const Eigen::Quaterniond& q_mid, double dt) const {
    PropagationModel model;
    const Eigen::Matrix3d R = q_mid.toRotationMatrix();
    const Eigen::Matrix3d accel_skew = skew(accel_body);
    const Eigen::Matrix3d omega_skew = skew(omega_body);

    model.Fc.setZero();
    model.Fc.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    model.Fc.block<3, 3>(3, 6) = -R * accel_skew;
    model.Fc.block<3, 3>(3, 9) = -R;
    model.Fc.block<3, 3>(6, 6) = -omega_skew;
    model.Fc.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity();

    model.Gc.setZero();
    model.Gc.block<3, 3>(3, 0) = -R;
    model.Gc.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    model.Gc.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    model.Gc.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    model.Phi = FMat::Identity() + model.Fc * dt;
    model.Phi.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    model.Phi.block<3, 3>(0, 6) = -0.5 * R * accel_skew * dt * dt;
    model.Phi.block<3, 3>(0, 9) = -0.5 * R * dt * dt;
    model.Phi.block<3, 3>(3, 6) = -R * accel_skew * dt;
    model.Phi.block<3, 3>(3, 9) = -R * dt;
    model.Phi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - omega_skew * dt;
    model.Phi.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    const CovMat Qc = model.Gc * Q_imu_ * model.Gc.transpose();
    model.Qd = model.Phi * Qc * model.Phi.transpose() * dt;
    model.Qd.block<3, 3>(0, 0) +=
        Eigen::Matrix3d::Identity() * (0.25 * cfg_.sigma_na * cfg_.sigma_na * dt * dt * dt * dt);
    model.Qd.block<3, 3>(0, 3) +=
        Eigen::Matrix3d::Identity() * (0.5 * cfg_.sigma_na * cfg_.sigma_na * dt * dt * dt);
    model.Qd.block<3, 3>(3, 0) = model.Qd.block<3, 3>(0, 3).transpose();
    model.accel_world = R * accel_body + Eigen::Vector3d{0.0, 0.0, -kGravity};
    return model;
}

Eigen::Quaterniond Phase17ESKFEstimator::propagate_quat(const Eigen::Quaterniond& q,
                                                        const Eigen::Vector3d& omega, double dt) {
    const double angle = omega.norm() * dt;
    if (angle < 1.0e-12) {
        return q;
    }
    return (q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized()))).normalized();
}

Eigen::Quaterniond Phase17ESKFEstimator::rotvec_to_quat(const Eigen::Vector3d& rv) {
    const double angle = rv.norm();
    if (angle < 1.0e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv / angle));
}

bool Phase17ESKFEstimator::validate_validation_config(const EstimatorValidationConfig& cfg) const {
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
           cfg.zupt.velocity_noise_mps > 0.0 && cfg.zupt.max_update_rate_hz > 0.0 &&
           (!cfg.enable_fej || cfg.fej.enabled);
}

bool Phase17ESKFEstimator::validate_msckf_config(const MsckfConfig& cfg) const {
    const auto& triangulation = cfg.triangulation;
    const auto& update = cfg.update;
    const bool triangulation_valid =
        std::isfinite(triangulation.minimum_baseline) &&
        std::isfinite(triangulation.maximum_reprojection_error) &&
        std::isfinite(triangulation.minimum_depth) && std::isfinite(triangulation.maximum_depth) &&
        triangulation.minimum_observations >= 2u && triangulation.minimum_baseline >= 0.0 &&
        triangulation.maximum_reprojection_error >= 0.0 && triangulation.minimum_depth > 0.0 &&
        triangulation.maximum_depth >= triangulation.minimum_depth &&
        (!triangulation.enabled || cfg.enabled);
    const bool update_valid =
        std::isfinite(update.chi_square_probability) && std::isfinite(update.maximum_residual) &&
        update.minimum_track_length >= 2u &&
        update.maximum_track_length >= update.minimum_track_length &&
        update.chi_square_probability > 0.0 && update.chi_square_probability < 1.0 &&
        update.maximum_residual >= 0.0 &&
        (!update.enabled || (cfg.enabled && triangulation.enabled));
    return cfg.max_camera_states > 0u && cfg.eviction_policy == "oldest_first" &&
           triangulation_valid && update_valid;
}

void Phase17ESKFEstimator::refresh_config_validity_locked() {
    validation_cfg_valid_ =
        validate_validation_config(validation_cfg_) && validate_msckf_config(msckf_cfg_);
}

std::optional<size_t>
Phase17ESKFEstimator::AugmentedStateLayout::clone_index_for_state(uint64_t state_id) const {
    const auto it = std::find(clone_ids.begin(), clone_ids.end(), state_id);
    if (it == clone_ids.end()) {
        return std::nullopt;
    }
    return static_cast<size_t>(std::distance(clone_ids.begin(), it));
}

std::optional<size_t>
Phase17ESKFEstimator::AugmentedStateLayout::clone_block_offset_for_state(uint64_t state_id) const {
    const auto index = clone_index_for_state(state_id);
    if (!index.has_value()) {
        return std::nullopt;
    }
    return clone_block_offset(*index);
}

void Phase17ESKFEstimator::clear_msckf_diagnostics_locked() {
    diagnostics_.msckf_window_size = 0;
    diagnostics_.msckf_states_created = 0;
    diagnostics_.msckf_states_removed = 0;
    diagnostics_.msckf_oldest_state_age_s = 0.0;
    diagnostics_.msckf_deterministic_evictions = 0;
    diagnostics_.marginalization_attempts = 0;
    diagnostics_.marginalizations_completed = 0;
    diagnostics_.marginalization_failures = 0;
    diagnostics_.marginalization_retiring_state_id = 0;
    diagnostics_.marginalization_affected_tracks = 0;
    diagnostics_.marginalization_constraint_candidates = 0;
    diagnostics_.marginalization_covariance_dim_before = 0;
    diagnostics_.marginalization_covariance_dim_after = 0;
    diagnostics_.marginalization_stale_references = 0;
    diagnostics_.marginalization_covariance_symmetry_error = 0.0;
    diagnostics_.marginalization_covariance_min_eigenvalue = 0.0;
}

void Phase17ESKFEstimator::clear_triangulation_diagnostics_locked() {
    diagnostics_.triangulation_attempts = 0;
    diagnostics_.triangulation_successes = 0;
    diagnostics_.triangulation_failures = 0;
    diagnostics_.rejected_insufficient_observations = 0;
    diagnostics_.rejected_small_baseline = 0;
    diagnostics_.rejected_negative_depth = 0;
    diagnostics_.rejected_depth_range = 0;
    diagnostics_.rejected_degenerate_geometry = 0;
    diagnostics_.rejected_non_finite_input = 0;
    diagnostics_.rejected_reprojection = 0;
    diagnostics_.active_landmarks = 0;
    diagnostics_.feature_tracks = 0;
    diagnostics_.feature_tracks_created = 0;
    diagnostics_.feature_tracks_removed = 0;
    diagnostics_.feature_updates_attempted = 0;
    diagnostics_.feature_updates_applied = 0;
    diagnostics_.feature_updates_rejected = 0;
    diagnostics_.feature_updates_considered = 0;
    diagnostics_.feature_updates_stacked = 0;
    diagnostics_.nullspace_failures = 0;
    diagnostics_.chi_square_failures = 0;
    diagnostics_.residual_norm = 0.0;
    diagnostics_.stacked_measurement_dimension = 0;
    diagnostics_.measurement_rank = 0;
    diagnostics_.chi_square_dof = 0;
    diagnostics_.chi_square_threshold = 0.0;
    diagnostics_.correction_norm = 0.0;
    diagnostics_.innovation_min_eigenvalue = 0.0;
    diagnostics_.innovation_condition_number = 0.0;
    diagnostics_.covariance_symmetry_error = 0.0;
    diagnostics_.covariance_min_eigenvalue = 0.0;
    diagnostics_.nullspace_annihilation_norm = 0.0;
    diagnostics_.nullspace_orthogonality_error = 0.0;
    diagnostics_.nullspace_rank_tolerance = kMsckfProjectionTolerance;
    diagnostics_.nullspace_validation_tolerance = kNullspaceValidationTolerance;
    diagnostics_.stacked_matrix_rank = 0;
}

void Phase17ESKFEstimator::refresh_triangulation_diagnostics_locked() {
    if (!msckf_diagnostics_enabled_locked()) {
        clear_triangulation_diagnostics_locked();
        return;
    }
    uint64_t active_landmarks = 0;
    for (const auto& [key, track] : feature_tracks_) {
        (void)key;
        active_landmarks += track.landmark_initialized ? 1u : 0u;
    }
    diagnostics_.feature_tracks = feature_tracks_.size();
    diagnostics_.active_landmarks = active_landmarks;
}

void Phase17ESKFEstimator::reset_diagnostics_locked() {
    diagnostics_ = {};
    diagnostics_.last_rejection_reason = EstimatorOperationResult::Accepted;
    diagnostics_.last_operation_result = EstimatorOperationResult::Accepted;
    diagnostics_.initialized = initialized_;
    diagnostics_.fej_enabled = fej_enabled_locked();
    clear_msckf_diagnostics_locked();
    clear_triangulation_diagnostics_locked();
}

Phase17ESKFEstimator::AugmentedStateLayout
Phase17ESKFEstimator::build_augmented_state_layout_locked() const {
    AugmentedStateLayout layout;
    layout.clone_ids.assign(msckf_state_order_.begin(), msckf_state_order_.end());
    layout.total_dim =
        static_cast<size_t>(kErrorDim) +
        (layout.clone_ids.size() * static_cast<size_t>(AugmentedStateLayout::kCloneErrorDim));
    return layout;
}

bool Phase17ESKFEstimator::augmented_covariance_dimensions_valid_locked(
    const AugmentedStateLayout& layout) const {
    return augmented_covariance_.rows() == static_cast<Eigen::Index>(layout.total_dim) &&
           augmented_covariance_.cols() == static_cast<Eigen::Index>(layout.total_dim);
}

void Phase17ESKFEstimator::synchronize_augmented_base_covariance_locked() {
    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        augmented_covariance_ = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(layout.total_dim),
                                                      static_cast<Eigen::Index>(layout.total_dim));
        augmented_covariance_.block(0, 0, kErrorDim, kErrorDim) = P_;
    } else {
        augmented_covariance_.block(0, 0, kErrorDim, kErrorDim) = P_;
    }
}

void Phase17ESKFEstimator::augment_covariance_for_new_clone_locked(uint64_t new_state_id) {
    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout) || layout.clone_count() == 0u) {
        synchronize_augmented_base_covariance_locked();
    }
    const auto block_offset = layout.clone_block_offset_for_state(new_state_id);
    if (!block_offset.has_value()) {
        return;
    }

    const Eigen::Index previous_dim = augmented_covariance_.rows();
    const Eigen::Index total_dim = static_cast<Eigen::Index>(layout.total_dim);
    if (previous_dim == total_dim) {
        return;
    }

    Eigen::MatrixXd previous = augmented_covariance_;
    if (previous.rows() != previous.cols() || previous.rows() != total_dim - 6) {
        previous = Eigen::MatrixXd::Zero(total_dim - 6, total_dim - 6);
        previous.block(0, 0, kErrorDim, kErrorDim) = P_;
    }

    Eigen::MatrixXd J =
        Eigen::MatrixXd::Zero(AugmentedStateLayout::kCloneErrorDim, previous.rows());
    J.block<3, 3>(AugmentedStateLayout::kClonePositionOffset, 0) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(AugmentedStateLayout::kCloneOrientationOffset, 6) = Eigen::Matrix3d::Identity();

    augmented_covariance_ = Eigen::MatrixXd::Zero(total_dim, total_dim);
    augmented_covariance_.block(0, 0, previous.rows(), previous.cols()) = previous;
    const Eigen::MatrixXd cross = previous * J.transpose();
    const Eigen::MatrixXd clone_cov = J * previous * J.transpose();
    augmented_covariance_.block(0, previous.rows(), previous.rows(), 6) = cross;
    augmented_covariance_.block(previous.rows(), 0, 6, previous.rows()) = cross.transpose();
    augmented_covariance_.block(previous.rows(), previous.rows(), 6, 6) = clone_cov;
    augmented_covariance_ = 0.5 * (augmented_covariance_ + augmented_covariance_.transpose());
    P_ = augmented_covariance_.block(0, 0, kErrorDim, kErrorDim);
}

void Phase17ESKFEstimator::remove_clone_covariance_block_locked(uint64_t state_id) {
    const auto layout_before = build_augmented_state_layout_locked();
    const auto offset = layout_before.clone_block_offset_for_state(state_id);
    if (!offset.has_value() || !augmented_covariance_dimensions_valid_locked(layout_before)) {
        synchronize_augmented_base_covariance_locked();
        return;
    }

    Eigen::MatrixXd reduced =
        Eigen::MatrixXd::Zero(augmented_covariance_.rows() - AugmentedStateLayout::kCloneErrorDim,
                              augmented_covariance_.cols() - AugmentedStateLayout::kCloneErrorDim);
    const Eigen::Index before = static_cast<Eigen::Index>(*offset);
    const Eigen::Index after = before + AugmentedStateLayout::kCloneErrorDim;
    const Eigen::Index tail = augmented_covariance_.rows() - after;

    if (before > 0) {
        reduced.block(0, 0, before, before) = augmented_covariance_.block(0, 0, before, before);
    }
    if (tail > 0) {
        reduced.block(0, before, before, tail) =
            augmented_covariance_.block(0, after, before, tail);
        reduced.block(before, 0, tail, before) =
            augmented_covariance_.block(after, 0, tail, before);
        reduced.block(before, before, tail, tail) =
            augmented_covariance_.block(after, after, tail, tail);
    }

    augmented_covariance_ = 0.5 * (reduced + reduced.transpose());
    P_ = augmented_covariance_.block(0, 0, kErrorDim, kErrorDim);
}

Eigen::MatrixXd Phase17ESKFEstimator::build_augmented_measurement_jacobian_locked(
    const Eigen::MatrixXd& H_base, const AugmentedStateLayout& layout) const {
    if (H_base.cols() == static_cast<Eigen::Index>(layout.total_dim)) {
        return H_base;
    }
    Eigen::MatrixXd H_aug =
        Eigen::MatrixXd::Zero(H_base.rows(), static_cast<Eigen::Index>(layout.total_dim));
    H_aug.block(0, 0, H_base.rows(), H_base.cols()) = H_base;
    return H_aug;
}

void Phase17ESKFEstimator::apply_clone_error_state_locked(
    std::unordered_map<uint64_t, MsckfCameraState>& camera_states,
    const AugmentedStateLayout& layout, const Eigen::VectorXd& dx_aug) const {
    for (size_t clone_index = 0; clone_index < layout.clone_count(); ++clone_index) {
        const uint64_t state_id = layout.clone_ids[clone_index];
        const auto it = camera_states.find(state_id);
        if (it == camera_states.end()) {
            continue;
        }
        const Eigen::Index offset =
            static_cast<Eigen::Index>(layout.clone_block_offset(clone_index));
        it->second.position_m +=
            dx_aug.segment<3>(offset + AugmentedStateLayout::kClonePositionOffset);
        const Eigen::Vector3d dtheta =
            dx_aug.segment<3>(offset + AugmentedStateLayout::kCloneOrientationOffset);
        it->second.orientation = (it->second.orientation * rotvec_to_quat(dtheta)).normalized();
    }
}

Eigen::MatrixXd
Phase17ESKFEstimator::build_augmented_reset_jacobian_locked(const AugmentedStateLayout& layout,
                                                            const Eigen::VectorXd& dx_aug) const {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(layout.total_dim),
                                                  static_cast<Eigen::Index>(layout.total_dim));
    J.block<3, 3>(6, 6) = attitude_reset_jacobian(dx_aug.segment<3>(6));
    for (size_t clone_index = 0; clone_index < layout.clone_count(); ++clone_index) {
        const Eigen::Index offset =
            static_cast<Eigen::Index>(layout.clone_block_offset(clone_index));
        const Eigen::Vector3d dtheta =
            dx_aug.segment<3>(offset + AugmentedStateLayout::kCloneOrientationOffset);
        J.block<3, 3>(offset + AugmentedStateLayout::kCloneOrientationOffset,
                      offset + AugmentedStateLayout::kCloneOrientationOffset) =
            attitude_reset_jacobian(dtheta);
    }
    return J;
}

double Phase17ESKFEstimator::chi_square_threshold_for_probability_locked(uint64_t dof) const {
    const uint64_t effective_dof = std::max<uint64_t>(1u, dof);
    const double probability =
        std::clamp(msckf_cfg_.update.chi_square_probability, 1.0e-6, 1.0 - 1.0e-6);
    const boost::math::chi_squared_distribution<double> dist(static_cast<double>(effective_dof));
    return boost::math::quantile(dist, probability);
}

bool Phase17ESKFEstimator::innovation_covariance_valid_locked(const Eigen::MatrixXd& innovation_cov,
                                                              double& min_eigenvalue_out,
                                                              double& condition_number_out) const {
    min_eigenvalue_out = 0.0;
    condition_number_out = 0.0;
    if (!innovation_cov.array().isFinite().all() ||
        innovation_cov.rows() != innovation_cov.cols()) {
        return false;
    }

    const Eigen::MatrixXd sym = 0.5 * (innovation_cov + innovation_cov.transpose());
    if ((sym - innovation_cov).cwiseAbs().maxCoeff() >
        validation_cfg_.covariance_symmetry_tolerance) {
        return false;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(sym);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    const auto eigenvalues = solver.eigenvalues();
    min_eigenvalue_out = eigenvalues.minCoeff();
    const double max_eigenvalue = eigenvalues.maxCoeff();
    if (!std::isfinite(min_eigenvalue_out) || !std::isfinite(max_eigenvalue) ||
        min_eigenvalue_out <= kInnovationEigenvalueTolerance) {
        return false;
    }

    condition_number_out = max_eigenvalue / min_eigenvalue_out;
    return std::isfinite(condition_number_out) &&
           condition_number_out <= kInnovationConditionNumberLimit;
}

void Phase17ESKFEstimator::set_result_locked(EstimatorOperationResult result) {
    diagnostics_.initialized = initialized_;
    diagnostics_.last_operation_result = result;
    if (result != EstimatorOperationResult::Accepted) {
        diagnostics_.last_rejection_reason = result;
    }
}

void Phase17ESKFEstimator::note_rejection_locked(EstimatorOperationResult result) {
    set_result_locked(result);
}

bool Phase17ESKFEstimator::vector_finite(const Eigen::Vector3d& value) const {
    return value.array().isFinite().all();
}

bool Phase17ESKFEstimator::matrix_finite(const CovMat& value) const {
    return value.array().isFinite().all();
}

bool Phase17ESKFEstimator::matrix_finite_dynamic(const Eigen::MatrixXd& value) const {
    return value.array().isFinite().all();
}

bool Phase17ESKFEstimator::quaternion_valid(const Eigen::Quaterniond& value) const {
    return value.coeffs().array().isFinite().all() &&
           value.norm() >= validation_cfg_.quaternion_min_norm;
}

bool Phase17ESKFEstimator::covariance_valid(const CovMat& value) const {
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

bool Phase17ESKFEstimator::covariance_valid_dynamic(const Eigen::MatrixXd& value) const {
    if (!matrix_finite_dynamic(value)) {
        return false;
    }
    const Eigen::MatrixXd sym = 0.5 * (value + value.transpose());
    if ((sym - value).cwiseAbs().maxCoeff() > validation_cfg_.covariance_symmetry_tolerance) {
        return false;
    }
    for (Eigen::Index i = 0; i < value.rows(); ++i) {
        if (value(i, i) < -validation_cfg_.variance_negativity_tolerance) {
            return false;
        }
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(sym);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    return solver.eigenvalues().minCoeff() >= -validation_cfg_.variance_negativity_tolerance;
}

bool Phase17ESKFEstimator::state_valid(const NominalStateSnapshot& candidate) const {
    return vector_finite(candidate.pos) && vector_finite(candidate.vel) &&
           vector_finite(candidate.ba) && vector_finite(candidate.bg) &&
           quaternion_valid(candidate.q) && covariance_valid(candidate.P) &&
           std::isfinite(candidate.total_drift);
}

bool Phase17ESKFEstimator::auto_zupt_enabled() const {
    return (validation_cfg_.enable_automatic_zupt || validation_cfg_.zupt.enabled) &&
           validation_cfg_.stationary_detector.enabled;
}

double Phase17ESKFEstimator::effective_zupt_velocity_noise_mps() const {
    if (std::isfinite(validation_cfg_.zupt.velocity_noise_mps) &&
        validation_cfg_.zupt.velocity_noise_mps > 0.0) {
        return validation_cfg_.zupt.velocity_noise_mps;
    }
    return validation_cfg_.zupt_sigma_velocity_mps;
}

double Phase17ESKFEstimator::compute_uncertainty_norm(const CovMat& cov) const {
    return cov.diagonal().head<3>().cwiseMax(0.0).cwiseSqrt().norm();
}

bool Phase17ESKFEstimator::fej_enabled_locked() const {
    return validation_cfg_.enable_fej && validation_cfg_.fej.enabled;
}

bool Phase17ESKFEstimator::msckf_enabled_locked() const {
    return msckf_cfg_.enabled;
}

bool Phase17ESKFEstimator::msckf_diagnostics_enabled_locked() const {
    return msckf_enabled_locked() && msckf_cfg_.diagnostics_enabled;
}

bool Phase17ESKFEstimator::triangulation_enabled_locked() const {
    return msckf_enabled_locked() && msckf_cfg_.triangulation.enabled;
}

bool Phase17ESKFEstimator::msckf_update_enabled_locked() const {
    return triangulation_enabled_locked() && msckf_cfg_.update.enabled;
}

bool Phase17ESKFEstimator::msckf_update_diagnostics_enabled_locked() const {
    return msckf_diagnostics_enabled_locked() && msckf_cfg_.update.diagnostics_enabled;
}

Phase17ESKFEstimator::FejFeatureKey
Phase17ESKFEstimator::make_fej_feature_key(const Eigen::Vector3d& feature_world) {
    return {
        static_cast<int64_t>(std::llround(feature_world.x() * kFejFeatureQuantizationScale)),
        static_cast<int64_t>(std::llround(feature_world.y() * kFejFeatureQuantizationScale)),
        static_cast<int64_t>(std::llround(feature_world.z() * kFejFeatureQuantizationScale)),
    };
}

int64_t Phase17ESKFEstimator::make_msckf_timestamp_key(double timestamp_s) {
    return static_cast<int64_t>(std::llround(timestamp_s * kMsckfTimestampQuantizationScale));
}

Eigen::Vector3d Phase17ESKFEstimator::pixel_to_bearing(const Eigen::Vector2d& pixel,
                                                       const Eigen::Matrix3d& K) {
    const Eigen::Vector3d homogeneous{pixel.x(), pixel.y(), 1.0};
    return (K.inverse() * homogeneous).normalized();
}

size_t
Phase17ESKFEstimator::FejFeatureKeyHasher::operator()(const FejFeatureKey& key) const noexcept {
    const size_t hx = std::hash<int64_t>{}(key.x);
    const size_t hy = std::hash<int64_t>{}(key.y);
    const size_t hz = std::hash<int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
}

Phase17ESKFEstimator::FejFeatureSnapshot*
Phase17ESKFEstimator::get_or_create_fej_snapshot_locked(const Eigen::Vector3d& feature_world) {
    if (!fej_enabled_locked()) {
        return nullptr;
    }

    const FejFeatureKey key = make_fej_feature_key(feature_world);
    if (auto it = fej_snapshots_.find(key); it != fej_snapshots_.end()) {
        return &it->second;
    }

    FejFeatureSnapshot snapshot;
    snapshot.snapshot_id = next_fej_snapshot_id_++;
    snapshot.key = key;
    snapshot.feature_world = feature_world;
    snapshot.first_position_m = pos_;
    snapshot.first_orientation = q_;
    snapshot.last_observed_epoch = fej_observation_epoch_;
    auto [it, inserted] = fej_snapshots_.emplace(key, snapshot);
    if (inserted) {
        ++diagnostics_.fej_snapshots_created;
    }
    return &it->second;
}

Phase17ESKFEstimator::FeatureTrack*
Phase17ESKFEstimator::get_or_create_feature_track_locked(const Eigen::Vector3d& feature_world) {
    const FejFeatureKey key = make_fej_feature_key(feature_world);
    if (auto it = feature_tracks_.find(key); it != feature_tracks_.end()) {
        return &it->second;
    }

    FeatureTrack track;
    track.track_id = next_feature_track_id_++;
    track.key = key;
    track.feature_identity_world = feature_world;
    auto [it, inserted] = feature_tracks_.emplace(key, track);
    if (inserted) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.feature_tracks_created;
        }
        refresh_triangulation_diagnostics_locked();
    }
    return &it->second;
}

bool Phase17ESKFEstimator::fej_snapshot_valid_locked(const FejFeatureSnapshot& snapshot) const {
    return snapshot.snapshot_id != 0 && vector_finite(snapshot.feature_world) &&
           vector_finite(snapshot.first_position_m) && quaternion_valid(snapshot.first_orientation);
}

void Phase17ESKFEstimator::release_inactive_fej_snapshots_locked(uint64_t observation_epoch) {
    for (auto it = fej_snapshots_.begin(); it != fej_snapshots_.end();) {
        if (it->second.last_observed_epoch != observation_epoch) {
            ++diagnostics_.fej_snapshots_released;
            it = fej_snapshots_.erase(it);
        } else {
            ++it;
        }
    }
}

void Phase17ESKFEstimator::reset_fej_locked() {
    fej_snapshots_.clear();
    next_fej_snapshot_id_ = 1;
    fej_observation_epoch_ = 0;
    diagnostics_.fej_enabled = fej_enabled_locked();
    diagnostics_.fej_snapshots_created = 0;
    diagnostics_.fej_snapshots_released = 0;
    diagnostics_.fej_jacobian_evaluations = 0;
    diagnostics_.fej_validation_failures = 0;
}

void Phase17ESKFEstimator::refresh_msckf_oldest_state_age_locked() {
    if (!msckf_diagnostics_enabled_locked()) {
        clear_msckf_diagnostics_locked();
        return;
    }
    diagnostics_.msckf_window_size = msckf_state_order_.size();
    if (msckf_state_order_.empty()) {
        diagnostics_.msckf_oldest_state_age_s = 0.0;
        return;
    }
    const auto oldest_id = msckf_state_order_.front();
    const auto it = msckf_camera_states_.find(oldest_id);
    const double reference_timestamp_s = last_accepted_imu_timestamp_s_.value_or(timestamp_);
    if (it == msckf_camera_states_.end() || !std::isfinite(reference_timestamp_s) ||
        !std::isfinite(it->second.timestamp_s)) {
        diagnostics_.msckf_oldest_state_age_s = 0.0;
        return;
    }
    diagnostics_.msckf_oldest_state_age_s =
        std::max(0.0, reference_timestamp_s - it->second.timestamp_s);
}

void Phase17ESKFEstimator::evict_msckf_state_locked() {
    if (msckf_state_order_.empty()) {
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    const uint64_t oldest_id = msckf_state_order_.front();
    const auto state_it = msckf_camera_states_.find(oldest_id);
    if (state_it == msckf_camera_states_.end()) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    std::vector<MarginalizationTrackSummary> summaries;
    summaries.reserve(feature_tracks_.size());
    for (const auto& [key, track] : feature_tracks_) {
        (void)key;
        MarginalizationTrackSummary summary;
        summary.track_id = track.track_id;
        summary.landmark_initialized = track.landmark_initialized;
        summary.observation_state_ids.reserve(track.observations.size());
        for (const auto& observation : track.observations) {
            summary.observation_state_ids.push_back(observation.state_id);
        }
        summaries.push_back(std::move(summary));
    }
    const auto plan = MsckfMarginalization::build_plan(
        oldest_id, summaries, msckf_cfg_.update.minimum_track_length);
    const std::vector<uint64_t> ordered_clone_ids(msckf_state_order_.begin(),
                                                   msckf_state_order_.end());

    MsckfRetirementRequest request;
    request.retiring_state_id = oldest_id;
    request.base_error_dim = static_cast<std::size_t>(AugmentedStateLayout::kBaseErrorDim);
    request.clone_error_dim = static_cast<std::size_t>(AugmentedStateLayout::kCloneErrorDim);
    request.symmetry_tolerance = validation_cfg_.covariance_symmetry_tolerance;
    request.negativity_tolerance = validation_cfg_.variance_negativity_tolerance;

    if (msckf_diagnostics_enabled_locked()) {
        ++diagnostics_.marginalization_attempts;
        diagnostics_.marginalization_retiring_state_id = oldest_id;
        diagnostics_.marginalization_affected_tracks = plan.affected_track_ids.size();
        diagnostics_.marginalization_constraint_candidates =
            plan.constraint_candidate_track_ids.size();
        diagnostics_.marginalization_covariance_dim_before =
            static_cast<uint64_t>(augmented_covariance_.rows());
    }

    const auto prepared = MsckfRetirementTransaction::prepare(
        request, ordered_clone_ids, augmented_covariance_);
    if (!prepared.has_value() || !prepared->committed) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    // Build the complete metadata candidate before publishing any retirement mutation.
    auto candidate_order = msckf_state_order_;
    auto candidate_states = msckf_camera_states_;
    auto candidate_timestamp_map = msckf_timestamp_to_state_id_;
    auto candidate_tracks = feature_tracks_;
    candidate_order.pop_front();
    candidate_timestamp_map.erase(state_it->second.timestamp_key);
    candidate_states.erase(oldest_id);
    for (auto track_it = candidate_tracks.begin(); track_it != candidate_tracks.end();) {
        auto& observations = track_it->second.observations;
        observations.erase(std::remove_if(observations.begin(), observations.end(),
                                          [oldest_id](const FeatureObservation& observation) {
                                              return observation.state_id == oldest_id;
                                          }),
                           observations.end());
        if (observations.empty()) {
            track_it = candidate_tracks.erase(track_it);
        } else {
            ++track_it;
        }
    }

    uint64_t stale_references = 0;
    for (const auto& [key, track] : candidate_tracks) {
        (void)key;
        stale_references += static_cast<uint64_t>(std::count_if(
            track.observations.begin(), track.observations.end(),
            [oldest_id](const FeatureObservation& observation) {
                return observation.state_id == oldest_id;
            }));
    }
    const auto health = MsckfMarginalization::covariance_health(
        prepared->retained_covariance, validation_cfg_.covariance_symmetry_tolerance,
        validation_cfg_.variance_negativity_tolerance);
    const std::size_t expected_dim = static_cast<std::size_t>(kErrorDim) +
        candidate_order.size() * static_cast<std::size_t>(AugmentedStateLayout::kCloneErrorDim);
    const bool candidate_valid =
        health.finite && health.symmetric && health.psd_within_tolerance &&
        prepared->retained_covariance.rows() == static_cast<Eigen::Index>(expected_dim) &&
        prepared->retained_covariance.cols() == static_cast<Eigen::Index>(expected_dim) &&
        stale_references == 0u;
    if (!candidate_valid) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.marginalization_failures;
        }
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        refresh_msckf_oldest_state_age_locked();
        return;
    }

    const uint64_t removed_track_count =
        static_cast<uint64_t>(feature_tracks_.size() - candidate_tracks.size());
    msckf_state_order_ = std::move(candidate_order);
    msckf_camera_states_ = std::move(candidate_states);
    msckf_timestamp_to_state_id_ = std::move(candidate_timestamp_map);
    feature_tracks_ = std::move(candidate_tracks);
    augmented_covariance_ = prepared->retained_covariance;
    P_ = augmented_covariance_.block(0, 0, kErrorDim, kErrorDim);
    P_ = 0.5 * (P_ + P_.transpose());

    if (msckf_diagnostics_enabled_locked()) {
        ++diagnostics_.marginalizations_completed;
        ++diagnostics_.msckf_states_removed;
        ++diagnostics_.msckf_deterministic_evictions;
        diagnostics_.feature_tracks_removed += removed_track_count;
        diagnostics_.marginalization_covariance_dim_after =
            static_cast<uint64_t>(augmented_covariance_.rows());
        diagnostics_.marginalization_stale_references = stale_references;
        diagnostics_.marginalization_covariance_symmetry_error = health.symmetry_error;
        diagnostics_.marginalization_covariance_min_eigenvalue = health.minimum_eigenvalue;
    }
    refresh_triangulation_diagnostics_locked();
    refresh_msckf_oldest_state_age_locked();
}

uint64_t
Phase17ESKFEstimator::capture_or_get_msckf_camera_state_locked(double capture_timestamp_s) {
    if (!msckf_enabled_locked() || !std::isfinite(capture_timestamp_s)) {
        refresh_msckf_oldest_state_age_locked();
        return 0u;
    }

    const int64_t timestamp_key = make_msckf_timestamp_key(capture_timestamp_s);
    if (const auto it = msckf_timestamp_to_state_id_.find(timestamp_key);
        it != msckf_timestamp_to_state_id_.end()) {
        refresh_msckf_oldest_state_age_locked();
        return it->second;
    }

    MsckfCameraState state;
    state.state_id = next_msckf_state_id_++;
    state.timestamp_key = timestamp_key;
    state.timestamp_s = capture_timestamp_s;
    state.position_m = pos_;
    state.orientation = q_;
    state.fej_position_m = pos_;
    state.fej_orientation = q_;
    state.velocity_reference_mps = vel_;

    msckf_state_order_.push_back(state.state_id);
    msckf_timestamp_to_state_id_.emplace(timestamp_key, state.state_id);
    msckf_camera_states_.emplace(state.state_id, state);
    if (msckf_diagnostics_enabled_locked()) {
        ++diagnostics_.msckf_states_created;
    }
    augment_covariance_for_new_clone_locked(state.state_id);

    // Retirement is deferred until the caller has consumed the current frame's
    // feature information. Non-feature callers retire in maybe_capture_msckf_camera_state_locked().
    refresh_msckf_oldest_state_age_locked();
    return state.state_id;
}

void Phase17ESKFEstimator::maybe_capture_msckf_camera_state_locked(double capture_timestamp_s) {
    (void)capture_or_get_msckf_camera_state_locked(capture_timestamp_s);
    while (msckf_state_order_.size() > msckf_cfg_.max_camera_states) {
        const auto before = msckf_state_order_.size();
        evict_msckf_state_locked();
        if (msckf_state_order_.size() >= before) {
            break;
        }
    }
}

void Phase17ESKFEstimator::record_feature_observations_locked(
    const std::vector<Eigen::Vector2d>& z_pixels, const std::vector<Eigen::Vector3d>& feature_world,
    const Eigen::Matrix3d& K, uint64_t state_id) {
    if (!triangulation_enabled_locked() || state_id == 0u ||
        z_pixels.size() != feature_world.size()) {
        refresh_triangulation_diagnostics_locked();
        return;
    }
    const auto state_it = msckf_camera_states_.find(state_id);
    if (state_it == msckf_camera_states_.end()) {
        refresh_triangulation_diagnostics_locked();
        return;
    }

    for (size_t i = 0; i < z_pixels.size(); ++i) {
        if (!z_pixels[i].array().isFinite().all() || !feature_world[i].array().isFinite().all()) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_non_finite_input;
            }
            continue;
        }
        FeatureTrack* track = get_or_create_feature_track_locked(feature_world[i]);
        if (!track) {
            continue;
        }
        const auto duplicate = std::find_if(track->observations.begin(), track->observations.end(),
                                            [state_id](const FeatureObservation& observation) {
                                                return observation.state_id == state_id;
                                            });
        if (duplicate != track->observations.end()) {
            continue;
        }

        FeatureObservation observation;
        observation.state_id = state_id;
        observation.timestamp_key = state_it->second.timestamp_key;
        observation.bearing_c = pixel_to_bearing(z_pixels[i], K);
        observation.pixel = z_pixels[i];
        if (!observation.bearing_c.array().isFinite().all()) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_non_finite_input;
            }
            continue;
        }
        track->observations.push_back(observation);
    }
    refresh_triangulation_diagnostics_locked();
}

void Phase17ESKFEstimator::prune_feature_tracks_for_state_locked(uint64_t state_id) {
    for (auto it = feature_tracks_.begin(); it != feature_tracks_.end();) {
        auto& observations = it->second.observations;
        observations.erase(std::remove_if(observations.begin(), observations.end(),
                                          [state_id](const FeatureObservation& observation) {
                                              return observation.state_id == state_id;
                                          }),
                           observations.end());
        if (observations.empty()) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_tracks_removed;
            }
            it = feature_tracks_.erase(it);
        } else {
            ++it;
        }
    }
    refresh_triangulation_diagnostics_locked();
}

void Phase17ESKFEstimator::prune_empty_feature_tracks_locked() {
    for (auto it = feature_tracks_.begin(); it != feature_tracks_.end();) {
        if (it->second.observations.empty()) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_tracks_removed;
            }
            it = feature_tracks_.erase(it);
        } else {
            ++it;
        }
    }
    refresh_triangulation_diagnostics_locked();
}

void Phase17ESKFEstimator::try_initialize_triangulated_landmarks_locked(const Eigen::Matrix3d& K) {
    if (!triangulation_enabled_locked()) {
        refresh_triangulation_diagnostics_locked();
        return;
    }

    for (auto& [key, track] : feature_tracks_) {
        (void)key;
        if (track.landmark_initialized) {
            continue;
        }
        if (track.observations.size() < msckf_cfg_.triangulation.minimum_observations) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_insufficient_observations;
            }
            continue;
        }

        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.triangulation_attempts;
        }
        Eigen::Vector3d landmark_world = Eigen::Vector3d::Zero();
        double baseline_m = 0.0;
        double max_reprojection_error_px = 0.0;
        if (triangulate_track_locked(track, K, landmark_world, baseline_m,
                                     max_reprojection_error_px)) {
            track.landmark_world = landmark_world;
            track.landmark_initialized = true;
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.triangulation_successes;
            }
        } else {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.triangulation_failures;
            }
        }
    }
    refresh_triangulation_diagnostics_locked();
}

void Phase17ESKFEstimator::try_apply_msckf_feature_updates_locked(const Eigen::Matrix3d& K,
                                                                  uint64_t state_id) {
    if (!msckf_update_enabled_locked() || state_id == 0u) {
        return;
    }
    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        synchronize_augmented_base_covariance_locked();
    }

    std::vector<FeatureTrack*> ordered_tracks;
    ordered_tracks.reserve(feature_tracks_.size());
    for (auto& [key, track] : feature_tracks_) {
        (void)key;
        ordered_tracks.push_back(&track);
    }
    std::sort(ordered_tracks.begin(), ordered_tracks.end(),
              [](const FeatureTrack* lhs, const FeatureTrack* rhs) {
                  return lhs->track_id < rhs->track_id;
              });

    struct AcceptedFeatureSystem {
        FeatureTrack* track{nullptr};
        ProjectedMeasurementSystem projected{};
    };
    std::vector<AcceptedFeatureSystem> accepted_features;
    uint64_t features_considered = 0;

    for (FeatureTrack* track : ordered_tracks) {
        if (!track || !track->landmark_initialized || track->last_update_state_id == state_id) {
            continue;
        }
        const auto has_latest_observation =
            std::find_if(track->observations.begin(), track->observations.end(),
                         [state_id](const FeatureObservation& observation) {
                             return observation.state_id == state_id;
                         }) != track->observations.end();
        if (!has_latest_observation ||
            track->observations.size() < msckf_cfg_.update.minimum_track_length) {
            continue;
        }
        ++features_considered;

        if (msckf_update_diagnostics_enabled_locked()) {
            ++diagnostics_.feature_updates_attempted;
        }

        FeatureUpdateBuildResult build_result;
        if (!build_feature_update_linearization_locked(*track, K, build_result)) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
                diagnostics_.residual_norm = build_result.raw_residual_norm;
                diagnostics_.measurement_rank = build_result.nullspace_rank;
                diagnostics_.stacked_measurement_dimension = 0;
            }
            continue;
        }

        ProjectedMeasurementSystem projected;
        uint64_t nullspace_rank = build_result.nullspace_rank;
        if (!project_feature_nullspace_locked(build_result.linearizations, projected,
                                              nullspace_rank)) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
                ++diagnostics_.nullspace_failures;
                diagnostics_.residual_norm = build_result.raw_residual_norm;
                diagnostics_.measurement_rank = nullspace_rank;
                diagnostics_.stacked_measurement_dimension = 0;
            }
            continue;
        }

        const double projected_residual_norm = projected.residual.norm();
        if (msckf_update_diagnostics_enabled_locked()) {
            diagnostics_.residual_norm = projected_residual_norm;
            diagnostics_.measurement_rank = projected.rank;
            diagnostics_.stacked_measurement_dimension = projected.residual.size();
            diagnostics_.nullspace_annihilation_norm = projected.annihilation_norm;
            diagnostics_.nullspace_orthogonality_error = projected.orthogonality_error;
        }
        if (!std::isfinite(projected_residual_norm) ||
            projected_residual_norm > msckf_cfg_.update.maximum_residual) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
            }
            continue;
        }

        const Eigen::MatrixXd innovation_cov =
            projected.H * augmented_covariance_ * projected.H.transpose() + projected.R;
        double min_eigenvalue = 0.0;
        double condition_number = 0.0;
        if (!innovation_covariance_valid_locked(innovation_cov, min_eigenvalue, condition_number)) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
                diagnostics_.innovation_min_eigenvalue = min_eigenvalue;
                diagnostics_.innovation_condition_number = condition_number;
            }
            note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
            continue;
        }
        const Eigen::LDLT<Eigen::MatrixXd> gate_ldlt(0.5 *
                                                     (innovation_cov + innovation_cov.transpose()));
        if (gate_ldlt.info() != Eigen::Success) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
            }
            note_rejection_locked(EstimatorOperationResult::FailedFactorization);
            continue;
        }
        const double chi_square_value =
            projected.residual.transpose() * gate_ldlt.solve(projected.residual);
        const uint64_t dof = static_cast<uint64_t>(projected.residual.size());
        const double chi_square_threshold = chi_square_threshold_for_probability_locked(dof);
        if (msckf_update_diagnostics_enabled_locked()) {
            diagnostics_.chi_square_dof = dof;
            diagnostics_.chi_square_threshold = chi_square_threshold;
            diagnostics_.innovation_min_eigenvalue = min_eigenvalue;
            diagnostics_.innovation_condition_number = condition_number;
        }
        if (!std::isfinite(chi_square_value) || !std::isfinite(chi_square_threshold) ||
            chi_square_value > chi_square_threshold) {
            if (msckf_update_diagnostics_enabled_locked()) {
                ++diagnostics_.feature_updates_rejected;
                ++diagnostics_.chi_square_failures;
            }
            continue;
        }
        accepted_features.push_back({track, projected});
    }

    if (msckf_update_diagnostics_enabled_locked()) {
        diagnostics_.feature_updates_considered = features_considered;
        diagnostics_.feature_updates_stacked = accepted_features.size();
    }
    if (accepted_features.empty()) {
        return;
    }

    Eigen::Index total_rows = 0;
    for (const auto& accepted : accepted_features) {
        total_rows += accepted.projected.residual.size();
    }
    Eigen::VectorXd stacked_residual = Eigen::VectorXd::Zero(total_rows);
    Eigen::MatrixXd stacked_H =
        Eigen::MatrixXd::Zero(total_rows, static_cast<Eigen::Index>(layout.total_dim));
    Eigen::MatrixXd stacked_R = Eigen::MatrixXd::Zero(total_rows, total_rows);
    Eigen::Index row_offset = 0;
    for (const auto& accepted : accepted_features) {
        const Eigen::Index rows = accepted.projected.residual.size();
        stacked_residual.segment(row_offset, rows) = accepted.projected.residual;
        stacked_H.block(row_offset, 0, rows, stacked_H.cols()) = accepted.projected.H;
        stacked_R.block(row_offset, row_offset, rows, rows) = accepted.projected.R;
        row_offset += rows;
    }

    if (msckf_update_diagnostics_enabled_locked()) {
        diagnostics_.stacked_measurement_dimension = static_cast<uint64_t>(stacked_residual.size());
        diagnostics_.stacked_matrix_rank =
            static_cast<uint64_t>(Eigen::FullPivLU<Eigen::MatrixXd>(stacked_H).rank());
    }

    const auto result = apply_error_state_update_locked(
        stacked_residual, stacked_H, stacked_R,
        [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
            candidate.ba += dx.segment<3>(9);
            candidate.bg += dx.segment<3>(12);
        },
        true);
    if (result == EstimatorOperationResult::Accepted) {
        for (const auto& accepted : accepted_features) {
            accepted.track->last_update_state_id = state_id;
        }
        if (msckf_update_diagnostics_enabled_locked()) {
            diagnostics_.feature_updates_applied += accepted_features.size();
        }
    } else if (msckf_update_diagnostics_enabled_locked()) {
        diagnostics_.feature_updates_rejected += accepted_features.size();
    }
}

bool Phase17ESKFEstimator::triangulate_track_locked(const FeatureTrack& track,
                                                    const Eigen::Matrix3d& K,
                                                    Eigen::Vector3d& landmark_world_out,
                                                    double& baseline_m_out,
                                                    double& max_reprojection_error_px_out) {
    baseline_m_out = 0.0;
    max_reprojection_error_px_out = 0.0;
    const auto& triangulation = msckf_cfg_.triangulation;
    if (track.observations.size() < triangulation.minimum_observations) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.rejected_insufficient_observations;
        }
        return false;
    }

    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    std::vector<std::pair<const FeatureObservation*, const MsckfCameraState*>> valid_observations;
    valid_observations.reserve(track.observations.size());
    for (const auto& observation : track.observations) {
        const auto it = msckf_camera_states_.find(observation.state_id);
        if (it == msckf_camera_states_.end()) {
            continue;
        }
        valid_observations.emplace_back(&observation, &it->second);
    }
    if (valid_observations.size() < triangulation.minimum_observations) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.rejected_insufficient_observations;
        }
        return false;
    }

    for (size_t i = 0; i < valid_observations.size(); ++i) {
        for (size_t j = i + 1; j < valid_observations.size(); ++j) {
            baseline_m_out = std::max(baseline_m_out, (valid_observations[i].second->position_m -
                                                       valid_observations[j].second->position_m)
                                                          .norm());
        }
    }
    if (baseline_m_out < triangulation.minimum_baseline) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.rejected_small_baseline;
        }
        return false;
    }

    for (const auto& [observation, state] : valid_observations) {
        const Eigen::Vector3d direction_w =
            (state->orientation.toRotationMatrix() * observation->bearing_c).normalized();
        const Eigen::Matrix3d projection =
            Eigen::Matrix3d::Identity() - (direction_w * direction_w.transpose());
        A += projection;
        b += projection * state->position_m;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(A);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues().minCoeff() <= kTriangulationEigenvalueTolerance) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.rejected_degenerate_geometry;
        }
        return false;
    }

    landmark_world_out = solver.operatorInverseSqrt() * solver.operatorInverseSqrt() * b;
    if (!landmark_world_out.array().isFinite().all()) {
        if (msckf_diagnostics_enabled_locked()) {
            ++diagnostics_.rejected_non_finite_input;
        }
        return false;
    }

    const double fx = K(0, 0);
    const double fy = K(1, 1);
    const double cx = K(0, 2);
    const double cy = K(1, 2);
    for (const auto& [observation, state] : valid_observations) {
        const Eigen::Vector3d p_c = state->orientation.toRotationMatrix().transpose() *
                                    (landmark_world_out - state->position_m);
        if (!p_c.array().isFinite().all()) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_non_finite_input;
            }
            return false;
        }
        if (p_c.z() < triangulation.minimum_depth) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_negative_depth;
            }
            return false;
        }
        if (p_c.z() > triangulation.maximum_depth) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_depth_range;
            }
            return false;
        }
        const Eigen::Vector2d reprojection{
            (fx * p_c.x() / p_c.z()) + cx,
            (fy * p_c.y() / p_c.z()) + cy,
        };
        const double reprojection_error = (reprojection - observation->pixel).norm();
        max_reprojection_error_px_out = std::max(max_reprojection_error_px_out, reprojection_error);
        if (!std::isfinite(reprojection_error) ||
            reprojection_error > triangulation.maximum_reprojection_error) {
            if (msckf_diagnostics_enabled_locked()) {
                ++diagnostics_.rejected_reprojection;
            }
            return false;
        }
    }
    return true;
}

bool Phase17ESKFEstimator::build_feature_update_linearization_locked(
    const FeatureTrack& track, const Eigen::Matrix3d& K, FeatureUpdateBuildResult& build_out) {
    build_out = {};

    if (!track.landmark_initialized) {
        return false;
    }

    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        return false;
    }

    struct OrderedObservation {
        size_t order_index{0};
        const FeatureObservation* observation{nullptr};
        const MsckfCameraState* state{nullptr};
    };
    std::vector<OrderedObservation> valid_observations;
    valid_observations.reserve(track.observations.size());
    for (const auto& observation : track.observations) {
        const auto it = msckf_camera_states_.find(observation.state_id);
        const auto clone_index = layout.clone_index_for_state(observation.state_id);
        if (it != msckf_camera_states_.end() && clone_index.has_value()) {
            valid_observations.push_back(
                {.order_index = *clone_index, .observation = &observation, .state = &it->second});
        }
    }
    std::sort(valid_observations.begin(), valid_observations.end(),
              [](const OrderedObservation& lhs, const OrderedObservation& rhs) {
                  return lhs.order_index < rhs.order_index;
              });
    if (valid_observations.size() < msckf_cfg_.update.minimum_track_length) {
        return false;
    }
    if (valid_observations.size() > msckf_cfg_.update.maximum_track_length) {
        valid_observations.erase(
            valid_observations.begin(),
            valid_observations.end() -
                static_cast<std::ptrdiff_t>(msckf_cfg_.update.maximum_track_length));
    }

    build_out.linearizations.reserve(valid_observations.size());
    const bool use_fej = fej_enabled_locked();
    for (const auto& ordered : valid_observations) {
        const auto* observation = ordered.observation;
        const auto* state = ordered.state;
        if (!observation || !state) {
            return false;
        }
        const Eigen::Matrix3d Rcw = state->orientation.toRotationMatrix().transpose();
        const Eigen::Vector3d p_c = Rcw * (track.landmark_world - state->position_m);
        if (!p_c.array().isFinite().all() || p_c.z() <= msckf_cfg_.triangulation.minimum_depth) {
            return false;
        }

        const Eigen::Vector2d predicted_norm{p_c.x() / p_c.z(), p_c.y() / p_c.z()};
        const Eigen::Vector2d observed_norm{observation->bearing_c.x() / observation->bearing_c.z(),
                                            observation->bearing_c.y() /
                                                observation->bearing_c.z()};
        const Eigen::Vector2d residual = observed_norm - predicted_norm;
        if (!residual.array().isFinite().all()) {
            return false;
        }

        Eigen::Quaterniond linearization_orientation = state->orientation;
        Eigen::Vector3d linearization_position = state->position_m;
        if (use_fej) {
            linearization_orientation = state->fej_orientation;
            linearization_position = state->fej_position_m;
            if (!quaternion_valid(linearization_orientation) ||
                !linearization_position.array().isFinite().all()) {
                ++diagnostics_.fej_validation_failures;
                note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
                return false;
            }
            ++diagnostics_.fej_jacobian_evaluations;
        }

        const Eigen::Matrix3d Rcw_linear = linearization_orientation.toRotationMatrix().transpose();
        const Eigen::Vector3d p_c_linear =
            Rcw_linear * (track.landmark_world - linearization_position);
        if (!p_c_linear.array().isFinite().all() ||
            p_c_linear.z() <= msckf_cfg_.triangulation.minimum_depth) {
            return false;
        }

        Eigen::Matrix<double, 2, 3> J_proj = Eigen::Matrix<double, 2, 3>::Zero();
        J_proj(0, 0) = 1.0 / p_c_linear.z();
        J_proj(0, 2) = -p_c_linear.x() / (p_c_linear.z() * p_c_linear.z());
        J_proj(1, 1) = 1.0 / p_c_linear.z();
        J_proj(1, 2) = -p_c_linear.y() / (p_c_linear.z() * p_c_linear.z());

        MsckfObservationLinearization item;
        item.state_id = observation->state_id;
        item.residual = residual;
        item.H_feature = -J_proj * Rcw_linear;
        item.H_state = Eigen::MatrixXd::Zero(2, static_cast<Eigen::Index>(layout.total_dim));
        const auto clone_offset = layout.clone_block_offset_for_state(observation->state_id);
        if (!clone_offset.has_value()) {
            return false;
        }
        item.H_state.block<2, 3>(0, static_cast<Eigen::Index>(*clone_offset)) = J_proj * Rcw_linear;
        item.H_state.block<2, 3>(
            0, static_cast<Eigen::Index>(*clone_offset +
                                         AugmentedStateLayout::kCloneOrientationOffset)) =
            -J_proj * skew(p_c_linear);
        if (!item.H_feature.array().isFinite().all() || !item.H_state.array().isFinite().all()) {
            return false;
        }
        build_out.linearizations.push_back(item);
        build_out.raw_residual_norm += residual.squaredNorm();
    }

    const double fx = K(0, 0);
    const double fy = K(1, 1);
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
        return false;
    }
    build_out.raw_residual_norm = std::sqrt(build_out.raw_residual_norm);
    build_out.nullspace_rank = 0u;
    return !build_out.linearizations.empty();
}

bool Phase17ESKFEstimator::project_feature_nullspace_locked(
    const std::vector<MsckfObservationLinearization,
                      Eigen::aligned_allocator<MsckfObservationLinearization>>& linearizations,
    ProjectedMeasurementSystem& projected_out, uint64_t& nullspace_rank_out) {
    projected_out = {};
    nullspace_rank_out = 0;
    if (linearizations.empty()) {
        return false;
    }

    const Eigen::Index measurement_dim = static_cast<Eigen::Index>(linearizations.size() * 2u);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(measurement_dim);
    const Eigen::Index augmented_dim = linearizations.front().H_state.cols();
    Eigen::MatrixXd H_state = Eigen::MatrixXd::Zero(measurement_dim, augmented_dim);
    Eigen::MatrixXd H_feature = Eigen::MatrixXd::Zero(measurement_dim, 3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
    const double sigma = std::max(1.0e-9, cfg_.sigma_px / 320.0);

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(linearizations.size()); ++i) {
        residual.segment<2>(i * 2) = linearizations[static_cast<size_t>(i)].residual;
        H_state.block(i * 2, 0, 2, augmented_dim) = linearizations[static_cast<size_t>(i)].H_state;
        H_feature.block(i * 2, 0, 2, 3) = linearizations[static_cast<size_t>(i)].H_feature;
        R.block<2, 2>(i * 2, i * 2) = Eigen::Matrix2d::Identity() * sigma * sigma;
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_feature, Eigen::ComputeFullU);
    if (svd.info() != Eigen::Success) {
        return false;
    }
    const auto& singular_values = svd.singularValues();
    const double rank_tolerance =
        singular_values.size() > 0
            ? std::max(kMsckfProjectionTolerance, singular_values(0) * kMsckfProjectionTolerance)
            : kMsckfProjectionTolerance;
    Eigen::Index rank = 0;
    for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
        if (singular_values(i) > rank_tolerance) {
            ++rank;
        }
    }
    nullspace_rank_out = static_cast<uint64_t>(rank);
    const Eigen::Index nullspace_dim = measurement_dim - rank;
    if (nullspace_dim <= 0) {
        return false;
    }

    const Eigen::MatrixXd A = svd.matrixU().rightCols(nullspace_dim);
    projected_out.residual = A.transpose() * residual;
    projected_out.H = A.transpose() * H_state;
    projected_out.R = A.transpose() * R * A;
    projected_out.rank = static_cast<uint64_t>(rank);
    projected_out.annihilation_norm = (A.transpose() * H_feature).norm();
    projected_out.orthogonality_error =
        (A.transpose() * A - Eigen::MatrixXd::Identity(nullspace_dim, nullspace_dim)).norm();
    if (!projected_out.residual.array().isFinite().all() ||
        !projected_out.H.array().isFinite().all() || !projected_out.R.array().isFinite().all()) {
        return false;
    }
    if (msckf_cfg_.update.validation_checks &&
        (projected_out.annihilation_norm > kNullspaceValidationTolerance ||
         projected_out.orthogonality_error > kNullspaceValidationTolerance)) {
        return false;
    }
    if (msckf_cfg_.update.validation_checks && projected_out.R.rows() > 0 &&
        (projected_out.R - projected_out.R.transpose()).cwiseAbs().maxCoeff() >
            validation_cfg_.covariance_symmetry_tolerance) {
        return false;
    }
    return projected_out.residual.size() > 0;
}

void Phase17ESKFEstimator::reset_msckf_locked() {
    msckf_state_order_.clear();
    msckf_camera_states_.clear();
    msckf_timestamp_to_state_id_.clear();
    feature_tracks_.clear();
    next_msckf_state_id_ = 1;
    next_feature_track_id_ = 1;
    augmented_covariance_ = P_;
    clear_msckf_diagnostics_locked();
    clear_triangulation_diagnostics_locked();
}

Phase17ESKFEstimator::NominalStateSnapshot Phase17ESKFEstimator::snapshot_locked() const {
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

void Phase17ESKFEstimator::commit_locked(const NominalStateSnapshot& candidate) {
    pos_ = candidate.pos;
    vel_ = candidate.vel;
    q_ = candidate.q.normalized();
    ba_ = candidate.ba;
    bg_ = candidate.bg;
    P_ = 0.5 * (candidate.P + candidate.P.transpose());
    if (augmented_covariance_.rows() >= kErrorDim && augmented_covariance_.cols() >= kErrorDim) {
        augmented_covariance_.block(0, 0, kErrorDim, kErrorDim) = P_;
    }
    total_drift_ = candidate.total_drift;
    refresh_msckf_oldest_state_age_locked();
}

void Phase17ESKFEstimator::reset_stationary_detector_locked() {
    stationary_window_.clear();
    stationary_candidate_start_timestamp_s_.reset();
    stationary_start_timestamp_s_.reset();
    last_automatic_zupt_timestamp_s_.reset();
    current_stationary_interval_zupt_count_ = 0;
    diagnostics_.stationary_detected = false;
    diagnostics_.stationary_duration_s = 0.0;
    diagnostics_.detector_state_change_count = 0;
    diagnostics_.stationary_intervals.clear();
}

void Phase17ESKFEstimator::finalize_stationary_interval_locked(double end_timestamp_s) {
    if (!stationary_start_timestamp_s_.has_value()) {
        return;
    }
    const double start_timestamp_s = *stationary_start_timestamp_s_;
    StationaryIntervalRecord record;
    record.start_timestamp_s = start_timestamp_s;
    record.end_timestamp_s = std::max(end_timestamp_s, start_timestamp_s);
    record.duration_s = std::max(0.0, record.end_timestamp_s - start_timestamp_s);
    record.zupt_updates = current_stationary_interval_zupt_count_;
    diagnostics_.stationary_intervals.push_back(record);
    stationary_start_timestamp_s_.reset();
    current_stationary_interval_zupt_count_ = 0;
}

void Phase17ESKFEstimator::update_stationary_detector_locked(const Eigen::Vector3d& accel_mps2,
                                                             const Eigen::Vector3d& gyro_rads,
                                                             double timestamp_s) {
    if (!validation_cfg_.stationary_detector.enabled || !std::isfinite(timestamp_s)) {
        diagnostics_.stationary_detected = false;
        diagnostics_.stationary_duration_s = 0.0;
        return;
    }

    const double accel_error = std::abs(accel_mps2.norm() - kGravity);
    const double gyro_mag = gyro_rads.norm();
    const bool enter_candidate =
        accel_error <= validation_cfg_.stationary_detector.accel_threshold_mps2 &&
        gyro_mag <= validation_cfg_.stationary_detector.gyro_threshold_rads;
    const bool exit_candidate =
        accel_error <= validation_cfg_.stationary_detector.accel_exit_threshold_mps2 &&
        gyro_mag <= validation_cfg_.stationary_detector.gyro_exit_threshold_rads;

    stationary_window_.push_back({timestamp_s, enter_candidate, exit_candidate});
    while (stationary_window_.size() > validation_cfg_.stationary_detector.window_size) {
        stationary_window_.pop_front();
    }

    const auto enter_count = static_cast<uint32_t>(
        std::count_if(stationary_window_.begin(), stationary_window_.end(),
                      [](const StationaryWindowSample& sample) { return sample.enter_candidate; }));
    const auto exit_count = static_cast<uint32_t>(
        std::count_if(stationary_window_.begin(), stationary_window_.end(),
                      [](const StationaryWindowSample& sample) { return sample.exit_candidate; }));

    if (!diagnostics_.stationary_detected) {
        if (enter_count >= validation_cfg_.stationary_detector.enter_count &&
            !stationary_window_.empty()) {
            if (!stationary_candidate_start_timestamp_s_.has_value()) {
                for (const auto& sample : stationary_window_) {
                    if (sample.enter_candidate) {
                        stationary_candidate_start_timestamp_s_ = sample.timestamp_s;
                        break;
                    }
                }
            }
            const auto candidate_start_timestamp_s = stationary_candidate_start_timestamp_s_;
            if (candidate_start_timestamp_s.has_value() &&
                (timestamp_s - *candidate_start_timestamp_s) >=
                    validation_cfg_.stationary_detector.minimum_stationary_time_s) {
                diagnostics_.stationary_detected = true;
                diagnostics_.stationary_duration_s =
                    std::max(0.0, timestamp_s - *candidate_start_timestamp_s);
                stationary_start_timestamp_s_ = candidate_start_timestamp_s;
                stationary_candidate_start_timestamp_s_.reset();
                ++diagnostics_.detector_state_change_count;
            }
        } else {
            stationary_candidate_start_timestamp_s_.reset();
            diagnostics_.stationary_duration_s = 0.0;
        }
    } else {
        diagnostics_.stationary_duration_s =
            stationary_start_timestamp_s_.has_value()
                ? std::max(0.0, timestamp_s - *stationary_start_timestamp_s_)
                : 0.0;
        if (exit_count < validation_cfg_.stationary_detector.exit_count) {
            finalize_stationary_interval_locked(timestamp_s);
            diagnostics_.stationary_detected = false;
            diagnostics_.stationary_duration_s = 0.0;
            stationary_candidate_start_timestamp_s_.reset();
            ++diagnostics_.detector_state_change_count;
        }
    }
}

void Phase17ESKFEstimator::maybe_apply_automatic_zupt_locked(double timestamp_s) {
    if (!auto_zupt_enabled() || !diagnostics_.stationary_detected || !std::isfinite(timestamp_s)) {
        return;
    }
    const double min_interval_s = 1.0 / validation_cfg_.zupt.max_update_rate_hz;
    if (last_automatic_zupt_timestamp_s_.has_value() &&
        (timestamp_s - *last_automatic_zupt_timestamp_s_) < min_interval_s) {
        return;
    }
    const auto result = update_zupt_locked(effective_zupt_velocity_noise_mps());
    if (result == EstimatorOperationResult::Accepted) {
        last_automatic_zupt_timestamp_s_ = timestamp_s;
    }
}

EstimatorOperationResult Phase17ESKFEstimator::update_zupt_locked(double sigma_velocity_mps) {
    if (!std::isfinite(sigma_velocity_mps) || sigma_velocity_mps <= 0.0) {
        diagnostics_.zupt_rejected_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidConfiguration);
        return diagnostics_.last_operation_result;
    }

    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_meas =
        Eigen::Matrix3d::Identity() * sigma_velocity_mps * sigma_velocity_mps;
    const Eigen::Vector3d innov = -vel_;
    const auto result = apply_error_state_update_locked(
        innov, H, R_meas,
        [](NominalStateSnapshot& candidate, const ErrorVec& dx) {
            candidate.pos += dx.segment<3>(0);
            candidate.vel += dx.segment<3>(3);
            candidate.ba += dx.segment<3>(9);
            candidate.bg += dx.segment<3>(12);
            candidate.vel.setZero();
        },
        false, true);
    if (result == EstimatorOperationResult::Accepted && diagnostics_.stationary_detected) {
        ++current_stationary_interval_zupt_count_;
    }
    return result;
}

EstimatorOperationResult
Phase17ESKFEstimator::propagate_imu_locked(const Eigen::Vector3d& accel_mps2,
                                           const Eigen::Vector3d& gyro_rads, double dt,
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

    const Eigen::Vector3d accel_body = accel_mps2 - ba_;
    const Eigen::Vector3d omega_body = gyro_rads - bg_;
    if (!vector_finite(accel_body) || !vector_finite(omega_body)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return diagnostics_.last_operation_result;
    }

    const Eigen::Quaterniond q_mid = propagate_quat(candidate.q, omega_body, 0.5 * dt);
    if (!quaternion_valid(q_mid)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        return diagnostics_.last_operation_result;
    }

    const PropagationModel model = build_propagation_model(accel_body, omega_body, q_mid, dt);
    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        synchronize_augmented_base_covariance_locked();
    }
    Eigen::MatrixXd candidate_augmented = augmented_covariance_;
    Eigen::MatrixXd Phi_aug = Eigen::MatrixXd::Identity(
        static_cast<Eigen::Index>(layout.total_dim), static_cast<Eigen::Index>(layout.total_dim));
    Phi_aug.block(0, 0, kErrorDim, kErrorDim) = model.Phi;
    Eigen::MatrixXd Q_aug = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(layout.total_dim),
                                                  static_cast<Eigen::Index>(layout.total_dim));
    Q_aug.block(0, 0, kErrorDim, kErrorDim) = model.Qd;
    candidate.pos += candidate.vel * dt + 0.5 * model.accel_world * dt * dt;
    candidate.vel += model.accel_world * dt;
    candidate.q = propagate_quat(candidate.q, omega_body, dt);
    candidate_augmented = Phi_aug * candidate_augmented * Phi_aug.transpose() + Q_aug;
    candidate_augmented = 0.5 * (candidate_augmented + candidate_augmented.transpose());
    candidate.P = candidate_augmented.block(0, 0, kErrorDim, kErrorDim);
    candidate.P = 0.5 * (candidate.P + candidate.P.transpose());
    candidate.total_drift = compute_uncertainty_norm(candidate.P);

    if (!state_valid(candidate) || !covariance_valid_dynamic(candidate_augmented)) {
        diagnostics_.rejected_propagation_count++;
        diagnostics_.covariance_failure_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return diagnostics_.last_operation_result;
    }

    commit_locked(candidate);
    augmented_covariance_ = candidate_augmented;
    if (update_internal_timestamp) {
        timestamp_ += dt;
    }
    diagnostics_.accepted_propagation_count++;
    set_result_locked(EstimatorOperationResult::Accepted);
    return diagnostics_.last_operation_result;
}

EstimatorOperationResult Phase17ESKFEstimator::inject_error_state_locked(const ErrorVec& dx) {
    if (!dx.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.non_finite_input_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedNonFiniteInput);
        return diagnostics_.last_operation_result;
    }

    NominalStateSnapshot candidate = snapshot_locked();
    candidate.pos += dx.segment<3>(0);
    candidate.vel += dx.segment<3>(3);
    const Eigen::Vector3d dtheta = dx.segment<3>(6);
    candidate.q = (candidate.q * rotvec_to_quat(dtheta)).normalized();
    candidate.ba += dx.segment<3>(9);
    candidate.bg += dx.segment<3>(12);

    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        synchronize_augmented_base_covariance_locked();
    }
    Eigen::VectorXd dx_aug = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(layout.total_dim));
    dx_aug.head<kErrorDim>() = dx;
    const Eigen::Matrix3d G_reset = attitude_reset_jacobian(dtheta);
    CovMat J = CovMat::Identity();
    J.block<3, 3>(6, 6) = G_reset;
    Eigen::MatrixXd J_aug = build_augmented_reset_jacobian_locked(layout, dx_aug);
    Eigen::MatrixXd candidate_augmented = J_aug * augmented_covariance_ * J_aug.transpose();
    candidate.P = candidate_augmented.block(0, 0, kErrorDim, kErrorDim);
    candidate.P = 0.5 * (candidate.P + candidate.P.transpose());
    candidate.total_drift = compute_uncertainty_norm(candidate.P);

    if (!state_valid(candidate) || !covariance_valid_dynamic(candidate_augmented)) {
        diagnostics_.rejected_update_count++;
        diagnostics_.covariance_failure_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        return diagnostics_.last_operation_result;
    }

    commit_locked(candidate);
    augmented_covariance_ = 0.5 * (candidate_augmented + candidate_augmented.transpose());
    diagnostics_.accepted_update_count++;
    set_result_locked(EstimatorOperationResult::Accepted);
    return diagnostics_.last_operation_result;
}

EstimatorOperationResult Phase17ESKFEstimator::apply_error_state_update_locked(
    const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R_meas,
    const std::function<void(NominalStateSnapshot&, const ErrorVec&)>& apply_dx,
    bool inject_attitude, bool count_as_zupt) {
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
    if (innovation.size() != H.rows() || R_meas.rows() != H.rows() || R_meas.cols() != H.rows()) {
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

    const auto layout = build_augmented_state_layout_locked();
    if (!augmented_covariance_dimensions_valid_locked(layout)) {
        synchronize_augmented_base_covariance_locked();
    }
    if (H.cols() != kErrorDim && H.cols() != static_cast<Eigen::Index>(layout.total_dim)) {
        diagnostics_.rejected_update_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedDimensionMismatch);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    const Eigen::MatrixXd H_aug = build_augmented_measurement_jacobian_locked(H, layout);
    const Eigen::MatrixXd S = H_aug * augmented_covariance_ * H_aug.transpose() + R_meas;
    if (!S.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    double innovation_min_eigenvalue = 0.0;
    double innovation_condition_number = 0.0;
    if (!innovation_covariance_valid_locked(S, innovation_min_eigenvalue,
                                            innovation_condition_number)) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(0.5 * (S + S.transpose()));
    if (ldlt.info() != Eigen::Success) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedFactorization);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    const Eigen::MatrixXd PHt = augmented_covariance_ * H_aug.transpose();
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
    if (dx_dyn.size() != static_cast<Eigen::Index>(layout.total_dim) ||
        !dx_dyn.array().isFinite().all()) {
        diagnostics_.rejected_update_count++;
        diagnostics_.numerical_failure_count++;
        note_rejection_locked(EstimatorOperationResult::FailedNumericalValidation);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }
    const ErrorVec dx = dx_dyn.head<kErrorDim>();

    NominalStateSnapshot candidate = snapshot_locked();
    apply_dx(candidate, dx);
    if (inject_attitude) {
        candidate.q = (candidate.q * rotvec_to_quat(dx.segment<3>(6))).normalized();
    }
    auto candidate_camera_states = msckf_camera_states_;
    apply_clone_error_state_locked(candidate_camera_states, layout, dx_dyn);
    if (!quaternion_valid(candidate.q)) {
        diagnostics_.rejected_update_count++;
        diagnostics_.quaternion_failure_count++;
        note_rejection_locked(EstimatorOperationResult::RejectedInvalidQuaternion);
        if (count_as_zupt) {
            diagnostics_.zupt_rejected_count++;
        }
        return diagnostics_.last_operation_result;
    }

    const Eigen::MatrixXd I_KH =
        Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(layout.total_dim),
                                  static_cast<Eigen::Index>(layout.total_dim)) -
        (K_gain * H_aug);
    Eigen::MatrixXd candidate_augmented =
        I_KH * augmented_covariance_ * I_KH.transpose() + K_gain * R_meas * K_gain.transpose();
    if (inject_attitude || layout.clone_count() > 0u) {
        const Eigen::MatrixXd J_aug = build_augmented_reset_jacobian_locked(layout, dx_dyn);
        candidate_augmented = J_aug * candidate_augmented * J_aug.transpose();
    }
    candidate.P = candidate_augmented.block(0, 0, kErrorDim, kErrorDim);
    candidate.P = 0.5 * (candidate.P + candidate.P.transpose());
    candidate.total_drift = compute_uncertainty_norm(candidate.P);
    if (!state_valid(candidate) || !covariance_valid_dynamic(candidate_augmented)) {
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
    msckf_camera_states_ = std::move(candidate_camera_states);
    augmented_covariance_ = 0.5 * (candidate_augmented + candidate_augmented.transpose());
    diagnostics_.correction_norm = dx_dyn.norm();
    diagnostics_.innovation_min_eigenvalue = innovation_min_eigenvalue;
    diagnostics_.innovation_condition_number = innovation_condition_number;
    diagnostics_.covariance_symmetry_error =
        (augmented_covariance_ - augmented_covariance_.transpose()).cwiseAbs().maxCoeff();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covariance_solver(
        0.5 * (augmented_covariance_ + augmented_covariance_.transpose()));
    if (covariance_solver.info() == Eigen::Success) {
        diagnostics_.covariance_min_eigenvalue = covariance_solver.eigenvalues().minCoeff();
    }
    diagnostics_.accepted_update_count++;
    if (count_as_zupt) {
        diagnostics_.zupt_accepted_count++;
    }
    set_result_locked(EstimatorOperationResult::Accepted);
    return diagnostics_.last_operation_result;
}

Phase17StateEstimatorAdapter::Phase17StateEstimatorAdapter(EKFConfig cfg, std::string name,
                                                           std::string version)
    : estimator_(cfg), name_(std::move(name)), version_(std::move(version)) {}

void Phase17StateEstimatorAdapter::configure_validation(const EstimatorValidationConfig& cfg) {
    estimator_.configure_validation(cfg);
}

void Phase17StateEstimatorAdapter::configure_msckf(const MsckfConfig& cfg) {
    estimator_.configure_msckf(cfg);
}

void Phase17StateEstimatorAdapter::reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                                         const Eigen::Vector3d& v0) {
    ++generation_;
    estimator_.reset(p0, q0, v0);
}

EstimatorOperationResult
Phase17StateEstimatorAdapter::process_measurement(const MeasurementEnvelope& envelope) {
    if (!is_measurement_envelope_valid(envelope)) {
        return EstimatorOperationResult::RejectedNonFiniteInput;
    }

    switch (envelope.type) {
    case MeasurementType::Imu: {
        const auto& payload = std::get<ImuMeasurementPayload>(envelope.payload);
        return estimator_.process_imu_measurement(payload.accel_mps2, payload.gyro_rads,
                                                  envelope.timestamp_s);
    }
    case MeasurementType::VisualPose: {
        const auto& payload = std::get<VisualPoseMeasurementPayload>(envelope.payload);
        estimator_.update_visual_pose(payload.position_m, payload.velocity_mps,
                                      payload.sigma_position_m, payload.sigma_velocity_mps);
        return estimator_.diagnostics().last_operation_result;
    }
    case MeasurementType::VisualFeatures: {
        const auto& payload = std::get<VisualFeatureMeasurementPayload>(envelope.payload);
        const std::vector<Eigen::Vector2d> z_pixels(payload.z_pixels.begin(),
                                                    payload.z_pixels.end());
        const std::vector<Eigen::Vector3d> p_world(payload.p_world.begin(), payload.p_world.end());
        estimator_.update_vision(z_pixels, p_world, payload.K);
        return estimator_.diagnostics().last_operation_result;
    }
    case MeasurementType::ManualZupt:
        estimator_.update_zupt();
        return estimator_.diagnostics().last_operation_result;
    case MeasurementType::LidarDepth: {
        const auto& payload = std::get<LidarDepthMeasurementPayload>(envelope.payload);
        estimator_.update_depth(payload.depth_m, payload.sigma_m);
        return estimator_.diagnostics().last_operation_result;
    }
    case MeasurementType::DisabledLidarObservation:
        return EstimatorOperationResult::RejectedUnsupportedMeasurement;
    }
    return EstimatorOperationResult::RejectedUnsupportedMeasurement;
}

EstimatorStateSnapshot Phase17StateEstimatorAdapter::snapshot() const {
    const auto pose = estimator_.state();
    const auto cov = estimator_.covariance();
    const auto diag = estimator_.diagnostics();

    EstimatorStateSnapshot out;
    out.timestamp_s = pose.timestamp;
    out.initialized = estimator_.is_initialized();
    out.position_m = pose.position;
    out.velocity_mps = pose.velocity;
    out.orientation = pose.orientation.normalized();
    out.accel_bias = pose.accel_bias;
    out.gyro_bias = pose.gyro_bias;
    out.covariance.trace = cov.trace();
    out.covariance.position_std_m = pose.pos_std;
    out.health = health_from_result(diag.last_operation_result, out.initialized);
    out.operation_count = diag.accepted_propagation_count + diag.rejected_propagation_count +
                          diag.accepted_update_count + diag.rejected_update_count;
    out.estimator_name = name_;
    out.estimator_version = version_;
    out.frame = MeasurementFrame::World;
    out.generation = generation_;
    if (diag.last_accepted_timestamp >= 0.0) {
        out.last_accepted_timestamp_s = diag.last_accepted_timestamp;
    }
    out.fej_enabled = diag.fej_enabled;
    out.fej_snapshots_created = diag.fej_snapshots_created;
    out.fej_snapshots_released = diag.fej_snapshots_released;
    out.fej_jacobian_evaluations = diag.fej_jacobian_evaluations;
    out.fej_validation_failures = diag.fej_validation_failures;
    out.msckf_window_size = diag.msckf_window_size;
    out.msckf_states_created = diag.msckf_states_created;
    out.msckf_states_removed = diag.msckf_states_removed;
    out.msckf_oldest_state_age_s = diag.msckf_oldest_state_age_s;
    out.msckf_deterministic_evictions = diag.msckf_deterministic_evictions;
    out.marginalization_attempts = diag.marginalization_attempts;
    out.marginalizations_completed = diag.marginalizations_completed;
    out.marginalization_failures = diag.marginalization_failures;
    out.marginalization_retiring_state_id = diag.marginalization_retiring_state_id;
    out.marginalization_affected_tracks = diag.marginalization_affected_tracks;
    out.marginalization_constraint_candidates = diag.marginalization_constraint_candidates;
    out.marginalization_covariance_dim_before = diag.marginalization_covariance_dim_before;
    out.marginalization_covariance_dim_after = diag.marginalization_covariance_dim_after;
    out.marginalization_stale_references = diag.marginalization_stale_references;
    out.marginalization_covariance_symmetry_error =
        diag.marginalization_covariance_symmetry_error;
    out.marginalization_covariance_min_eigenvalue =
        diag.marginalization_covariance_min_eigenvalue;
    out.triangulation_attempts = diag.triangulation_attempts;
    out.triangulation_successes = diag.triangulation_successes;
    out.triangulation_failures = diag.triangulation_failures;
    out.rejected_insufficient_observations = diag.rejected_insufficient_observations;
    out.rejected_small_baseline = diag.rejected_small_baseline;
    out.rejected_negative_depth = diag.rejected_negative_depth;
    out.rejected_depth_range = diag.rejected_depth_range;
    out.rejected_degenerate_geometry = diag.rejected_degenerate_geometry;
    out.rejected_non_finite_input = diag.rejected_non_finite_input;
    out.rejected_reprojection = diag.rejected_reprojection;
    out.active_landmarks = diag.active_landmarks;
    out.feature_tracks = diag.feature_tracks;
    out.feature_tracks_created = diag.feature_tracks_created;
    out.feature_tracks_removed = diag.feature_tracks_removed;
    out.feature_updates_attempted = diag.feature_updates_attempted;
    out.feature_updates_applied = diag.feature_updates_applied;
    out.feature_updates_rejected = diag.feature_updates_rejected;
    out.feature_updates_considered = diag.feature_updates_considered;
    out.feature_updates_stacked = diag.feature_updates_stacked;
    out.nullspace_failures = diag.nullspace_failures;
    out.chi_square_failures = diag.chi_square_failures;
    out.residual_norm = diag.residual_norm;
    out.stacked_measurement_dimension = diag.stacked_measurement_dimension;
    out.measurement_rank = diag.measurement_rank;
    out.chi_square_dof = diag.chi_square_dof;
    out.chi_square_threshold = diag.chi_square_threshold;
    out.correction_norm = diag.correction_norm;
    out.innovation_min_eigenvalue = diag.innovation_min_eigenvalue;
    out.innovation_condition_number = diag.innovation_condition_number;
    out.covariance_symmetry_error = diag.covariance_symmetry_error;
    out.covariance_min_eigenvalue = diag.covariance_min_eigenvalue;
    out.nullspace_annihilation_norm = diag.nullspace_annihilation_norm;
    out.nullspace_orthogonality_error = diag.nullspace_orthogonality_error;
    out.nullspace_rank_tolerance = diag.nullspace_rank_tolerance;
    out.nullspace_validation_tolerance = diag.nullspace_validation_tolerance;
    out.stacked_matrix_rank = diag.stacked_matrix_rank;
    out.stationary_detected = diag.stationary_detected;
    out.stationary_duration_s = diag.stationary_duration_s;
    out.zupt_updates_applied = diag.zupt_accepted_count;
    out.zupt_updates_rejected = diag.zupt_rejected_count;
    out.detector_state_changes = diag.detector_state_change_count;
    out.stationary_intervals = diag.stationary_intervals;
    if (diag.stationary_detected && diag.last_accepted_timestamp >= 0.0) {
        StationaryIntervalRecord open_interval;
        open_interval.end_timestamp_s = diag.last_accepted_timestamp;
        open_interval.duration_s = diag.stationary_duration_s;
        open_interval.start_timestamp_s =
            std::max(0.0, open_interval.end_timestamp_s - open_interval.duration_s);
        const uint64_t committed_zupt = [&diag]() {
            uint64_t count = 0;
            for (const auto& interval : diag.stationary_intervals) {
                count += interval.zupt_updates;
            }
            return count;
        }();
        open_interval.zupt_updates = diag.zupt_accepted_count - committed_zupt;
        out.stationary_intervals.push_back(open_interval);
    }
    return out;
}

EKFDiagnostics Phase17StateEstimatorAdapter::diagnostics() const {
    return estimator_.diagnostics();
}

bool Phase17StateEstimatorAdapter::is_initialized() const {
    return estimator_.is_initialized();
}

std::string Phase17StateEstimatorAdapter::estimator_name() const {
    return name_;
}

std::string Phase17StateEstimatorAdapter::estimator_version() const {
    return version_;
}

} // namespace drone::vio
