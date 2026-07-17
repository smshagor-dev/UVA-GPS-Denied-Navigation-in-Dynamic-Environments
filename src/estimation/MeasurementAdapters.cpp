#include "estimation/MeasurementAdapters.hpp"

#include <cmath>

namespace drone::estimation {

namespace {

bool finite_vec3(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

} // namespace

MeasurementAdapterResult MeasurementAdapters::adapt_imu(const drone::sensors::ImuMeasurement& imu,
                                                        uint64_t sequence_id) {
    MeasurementAdapterResult out;
    if (!std::isfinite(imu.timestamp) || !finite_vec3(imu.accel_mps2) ||
        !finite_vec3(imu.gyro_rads)) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "imu measurement contains non-finite values";
        return out;
    }

    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = imu.timestamp;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::IMU;
    measurement.header.frame = CoordinateFrame::BODY;
    measurement.header.type = MeasurementType::IMU;
    measurement.data = ImuMeasurementData{imu.accel_mps2, imu.gyro_rads};

    out.validation.consumed = true;
    out.measurement = std::move(measurement);
    return out;
}

MeasurementAdapterResult MeasurementAdapters::adapt_visual_pose(
    double timestamp_s, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Quaterniond& orientation, double quality, bool update_accepted,
    uint64_t sequence_id, bool allow_orientation_fusion) {
    MeasurementAdapterResult out;
    if (!std::isfinite(timestamp_s) || !finite_vec3(position) || !finite_vec3(velocity) ||
        !std::isfinite(quality)) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "visual pose measurement contains non-finite values";
        return out;
    }
    if (quality < 0.0 || quality > 1.0) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "visual pose quality is outside [0, 1]";
        return out;
    }

    VisualPoseMeasurementData visual;
    visual.position = position;
    visual.velocity = velocity;
    visual.quality = quality;
    visual.covariance.dimension = 6;
    visual.covariance.matrix.setZero();
    const double sigma_position_m = std::clamp(0.50 - (quality * 0.28), 0.14, 0.50);
    const double sigma_velocity_mps = std::clamp(0.65 - (quality * 0.30), 0.18, 0.65);
    visual.covariance.matrix.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * sigma_position_m * sigma_position_m;
    visual.covariance.matrix.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * sigma_velocity_mps * sigma_velocity_mps;

    const double orientation_norm = orientation.norm();
    if (!std::isfinite(orientation_norm) || std::abs(orientation_norm - 1.0) > 1.0e-3) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "visual orientation quaternion is invalid";
        return out;
    }
    visual.orientation = orientation.normalized();
    visual.orientation_consumed = allow_orientation_fusion;

    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::VISUAL_FRONTEND;
    measurement.header.frame = CoordinateFrame::WORLD;
    measurement.header.type = MeasurementType::VISUAL_POSE;
    measurement.data = std::move(visual);

    out.validation.consumed = update_accepted;
    out.validation.orientation_consumed = allow_orientation_fusion;
    if (!update_accepted) {
        out.validation.status = MeasurementValidationStatus::REJECTED_OUTLIER;
        out.validation.reason = "visual frontend rejected the pose update";
        return out;
    }
    if (!allow_orientation_fusion) {
        out.validation.reason = "visual orientation preserved but not consumed by active estimator";
    }
    out.measurement = std::move(measurement);
    return out;
}

MeasurementAdapterResult MeasurementAdapters::adapt_manual_zupt(double timestamp_s,
                                                                uint64_t sequence_id) {
    MeasurementAdapterResult out;
    if (!std::isfinite(timestamp_s)) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "zupt timestamp must be finite";
        return out;
    }
    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::MANUAL_ZUPT;
    measurement.header.frame = CoordinateFrame::BODY;
    measurement.header.type = MeasurementType::ZERO_VELOCITY;
    measurement.data = ZeroVelocityMeasurementData{};
    out.validation.consumed = true;
    out.measurement = std::move(measurement);
    return out;
}

MeasurementAdapterResult MeasurementAdapters::adapt_altitude(double timestamp_s, double altitude_m,
                                                             double sigma_m, CoordinateFrame frame,
                                                             uint64_t sequence_id) {
    MeasurementAdapterResult out;
    if (!std::isfinite(timestamp_s) || !std::isfinite(altitude_m) || !std::isfinite(sigma_m) ||
        sigma_m <= 0.0) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "altitude measurement is invalid";
        return out;
    }
    if (frame != CoordinateFrame::WORLD && frame != CoordinateFrame::LOCAL_ENU) {
        out.validation.status = MeasurementValidationStatus::REJECTED_FRAME_MISMATCH;
        out.validation.reason = "altitude frame is unsupported";
        return out;
    }
    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::DEPTH_SENSOR;
    measurement.header.frame = frame;
    measurement.header.type = MeasurementType::ALTITUDE;
    measurement.data = AltitudeMeasurementData{altitude_m, sigma_m};
    out.validation.consumed = true;
    out.measurement = std::move(measurement);
    return out;
}

MeasurementAdapterResult MeasurementAdapters::adapt_tdoa_position_candidate(
    double timestamp_s, const Eigen::Vector3d& position, double confidence, uint64_t sequence_id) {
    MeasurementAdapterResult out;
    if (!std::isfinite(timestamp_s) || !finite_vec3(position) || !std::isfinite(confidence) ||
        confidence < 0.0 || confidence > 1.0) {
        out.validation.status = MeasurementValidationStatus::REJECTED_INVALID;
        out.validation.reason = "tdoa position candidate is invalid";
        return out;
    }

    EstimatorMeasurement measurement;
    measurement.header.timestamp_s = timestamp_s;
    measurement.header.sequence_id = sequence_id;
    measurement.header.source = SensorSource::TDOA_HEURISTIC;
    measurement.header.frame = CoordinateFrame::WORLD;
    measurement.header.type = MeasurementType::TDOA_POSITION_CANDIDATE;
    measurement.data = TdoaPositionMeasurementData{position, confidence, true};

    out.validation.status = MeasurementValidationStatus::IGNORED_SHADOW_ONLY;
    out.validation.reason = "tdoa candidate remains heuristic-only in phase 16";
    out.measurement = std::move(measurement);
    return out;
}

MeasurementValidationResult MeasurementAdapters::validate(const EstimatorMeasurement& measurement,
                                                          double newest_timestamp_s,
                                                          double max_staleness_s) {
    MeasurementValidationResult result;
    if (!std::isfinite(measurement.header.timestamp_s)) {
        result.status = MeasurementValidationStatus::REJECTED_INVALID;
        result.reason = "measurement timestamp must be finite";
        return result;
    }
    if (measurement.header.frame == CoordinateFrame::UNKNOWN) {
        result.status = MeasurementValidationStatus::REJECTED_UNSUPPORTED;
        result.reason = "measurement frame is unsupported";
        return result;
    }
    if (newest_timestamp_s >= 0.0 &&
        measurement.header.timestamp_s + max_staleness_s < newest_timestamp_s) {
        result.status = MeasurementValidationStatus::REJECTED_STALE;
        result.reason = "measurement is stale";
        return result;
    }
    result.consumed = true;
    return result;
}

} // namespace drone::estimation
