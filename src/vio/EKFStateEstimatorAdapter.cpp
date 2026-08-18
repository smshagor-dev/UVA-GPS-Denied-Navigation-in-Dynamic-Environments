#include "vio/EKFStateEstimatorAdapter.hpp"

namespace drone::vio {

EKFStateEstimatorAdapter::EKFStateEstimatorAdapter(EKFConfig cfg, std::string name,
                                                   std::string version)
    : estimator_(cfg), name_(std::move(name)), version_(std::move(version)) {}

void EKFStateEstimatorAdapter::configure_validation(const EstimatorValidationConfig& cfg) {
    estimator_.configure_validation(cfg);
}

void EKFStateEstimatorAdapter::reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                                     const Eigen::Vector3d& v0) {
    ++generation_;
    estimator_.reset(p0, q0, v0);
}

EstimatorOperationResult
EKFStateEstimatorAdapter::process_measurement(const MeasurementEnvelope& envelope) {
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

EstimatorStateSnapshot EKFStateEstimatorAdapter::snapshot() const {
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
    out.stationary_detected = diag.stationary_detected;
    out.stationary_duration_s = diag.stationary_duration_s;
    out.zupt_updates_applied = diag.zupt_accepted_count;
    out.zupt_updates_rejected = diag.zupt_rejected_count;
    out.detector_state_changes = diag.detector_state_change_count;
    out.stationary_intervals = diag.stationary_intervals;
    return out;
}

EKFDiagnostics EKFStateEstimatorAdapter::diagnostics() const {
    return estimator_.diagnostics();
}

bool EKFStateEstimatorAdapter::is_initialized() const {
    return estimator_.is_initialized();
}

std::string EKFStateEstimatorAdapter::estimator_name() const {
    return name_;
}

std::string EKFStateEstimatorAdapter::estimator_version() const {
    return version_;
}

} // namespace drone::vio
