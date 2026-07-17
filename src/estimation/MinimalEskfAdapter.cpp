#include "estimation/MinimalEskfAdapter.hpp"

#include <cmath>

namespace drone::estimation {

MinimalEskfAdapter::MinimalEskfAdapter(drone::vio::EKFConfig config) : estimator_(config) {
    estimator_.set_validation_config(validation_);
}

void MinimalEskfAdapter::set_validation_config(
    const drone::vio::EstimatorValidationConfig& config) {
    validation_ = config;
    estimator_.set_validation_config(validation_);
}

void MinimalEskfAdapter::reset(const EstimatorInitialState& initial) {
    estimator_.reset(initial.position, initial.orientation, initial.velocity);
    estimator_.set_validation_config(validation_);
    last_sequence_id_ = 0;
    last_timestamp_s_ = -1.0;
}

EstimatorUpdateResult MinimalEskfAdapter::process(const EstimatorMeasurement& measurement) {
    EstimatorUpdateResult result;
    if (last_timestamp_s_ >= 0.0 && measurement.header.timestamp_s < last_timestamp_s_) {
        return unsupported_result(measurement, MeasurementValidationStatus::REJECTED_STALE,
                                  "measurement timestamp moved backwards");
    }

    result.validation.consumed = true;
    result.status = EstimatorUpdateStatus::ACCEPTED;
    switch (measurement.header.type) {
    case MeasurementType::IMU: {
        const auto& imu = std::get<ImuMeasurementData>(measurement.data);
        const double dt = last_timestamp_s_ >= 0.0
                              ? (measurement.header.timestamp_s - last_timestamp_s_)
                              : 0.0025;
        estimator_.propagate_imu(imu.acceleration_mps2, imu.angular_velocity_rads, dt);
        break;
    }
    case MeasurementType::VISUAL_POSE: {
        const auto& visual = std::get<VisualPoseMeasurementData>(measurement.data);
        estimator_.update_visual_pose(visual.position, visual.velocity,
                                      std::sqrt(std::max(0.0, visual.covariance.matrix(0, 0))),
                                      std::sqrt(std::max(0.0, visual.covariance.matrix(3, 3))));
        result.validation.orientation_consumed = false;
        if (visual.orientation.has_value()) {
            result.validation.reason =
                "visual orientation preserved but not fused by minimal ESKF adapter";
        }
        break;
    }
    case MeasurementType::ALTITUDE: {
        const auto& altitude = std::get<AltitudeMeasurementData>(measurement.data);
        estimator_.update_depth(altitude.altitude_m, altitude.sigma_m);
        break;
    }
    case MeasurementType::ZERO_VELOCITY:
        estimator_.update_zupt();
        break;
    case MeasurementType::TDOA_POSITION_CANDIDATE:
        return unsupported_result(measurement, MeasurementValidationStatus::IGNORED_SHADOW_ONLY,
                                  "tdoa candidate is not a mathematically valid ESKF update");
    default:
        return unsupported_result(measurement, MeasurementValidationStatus::REJECTED_UNSUPPORTED,
                                  "measurement type unsupported by minimal ESKF");
    }

    last_sequence_id_ = measurement.header.sequence_id;
    last_timestamp_s_ = measurement.header.timestamp_s;
    result.snapshot = snapshot();
    return result;
}

EstimatorSnapshot MinimalEskfAdapter::snapshot() const {
    EstimatorSnapshot out;
    out.pose = estimator_.state();
    out.diagnostics = estimator_.diagnostics();
    out.last_sequence_id = last_sequence_id_;
    return out;
}

drone::vio::EstimatorDiagnostics MinimalEskfAdapter::diagnostics() const {
    return estimator_.diagnostics();
}

EstimatorCapabilities MinimalEskfAdapter::capabilities() const {
    EstimatorCapabilities capabilities;
    capabilities.set(EstimatorCapability::IMU_PROPAGATION);
    capabilities.set(EstimatorCapability::VISUAL_POSITION_UPDATE);
    capabilities.set(EstimatorCapability::VISUAL_VELOCITY_UPDATE);
    capabilities.set(EstimatorCapability::DEPTH_UPDATE);
    capabilities.set(EstimatorCapability::ZERO_VELOCITY_UPDATE);
    return capabilities;
}

std::string_view MinimalEskfAdapter::name() const noexcept {
    return "minimal_eskf";
}

EstimatorUpdateResult
MinimalEskfAdapter::unsupported_result(const EstimatorMeasurement& measurement,
                                       MeasurementValidationStatus status,
                                       std::string reason) const {
    EstimatorUpdateResult result;
    result.status = status == MeasurementValidationStatus::IGNORED_SHADOW_ONLY
                        ? EstimatorUpdateStatus::IGNORED
                        : EstimatorUpdateStatus::REJECTED;
    result.validation.status = status;
    result.validation.reason = std::move(reason);
    result.snapshot = snapshot();
    result.snapshot.last_sequence_id = measurement.header.sequence_id;
    return result;
}

} // namespace drone::estimation
