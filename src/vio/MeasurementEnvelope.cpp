#include "vio/MeasurementEnvelope.hpp"

#include <cmath>

namespace drone::vio {

namespace {

bool finite_scalar(double value) {
    return std::isfinite(value);
}

bool finite_quat_sigma(double value) {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

bool is_measurement_payload_finite(const MeasurementEnvelope& envelope) {
    switch (envelope.type) {
    case MeasurementType::Imu: {
        const auto& payload = std::get<ImuMeasurementPayload>(envelope.payload);
        return payload.accel_mps2.array().isFinite().all() &&
               payload.gyro_rads.array().isFinite().all();
    }
    case MeasurementType::VisualPose: {
        const auto& payload = std::get<VisualPoseMeasurementPayload>(envelope.payload);
        return payload.position_m.array().isFinite().all() &&
               payload.velocity_mps.array().isFinite().all() &&
               finite_quat_sigma(payload.sigma_position_m) &&
               finite_quat_sigma(payload.sigma_velocity_mps);
    }
    case MeasurementType::VisualAttitude: {
        const auto& payload = std::get<VisualAttitudeMeasurementPayload>(envelope.payload);
        const auto& q = payload.world_from_imu;
        const double norm_squared = q.squaredNorm();
        return q.coeffs().array().isFinite().all() && std::isfinite(norm_squared) &&
               norm_squared >= 0.99 * 0.99 && norm_squared <= 1.01 * 1.01 &&
               finite_quat_sigma(payload.sigma_orientation_rad) &&
               payload.sigma_orientation_rad <= 1.0;
    }
    case MeasurementType::VisualFeatures: {
        const auto& payload = std::get<VisualFeatureMeasurementPayload>(envelope.payload);
        if (!payload.K.array().isFinite().all() ||
            payload.z_pixels.size() != payload.p_world.size() || payload.z_pixels.empty()) {
            return false;
        }
        for (size_t i = 0; i < payload.z_pixels.size(); ++i) {
            if (!payload.z_pixels[i].array().isFinite().all() ||
                !payload.p_world[i].array().isFinite().all()) {
                return false;
            }
        }
        return true;
    }
    case MeasurementType::ManualZupt: {
        const auto& payload = std::get<ManualZuptMeasurementPayload>(envelope.payload);
        return finite_quat_sigma(payload.sigma_velocity_mps);
    }
    case MeasurementType::LidarDepth: {
        const auto& payload = std::get<LidarDepthMeasurementPayload>(envelope.payload);
        return finite_scalar(payload.depth_m) && finite_quat_sigma(payload.sigma_m);
    }
    case MeasurementType::DisabledLidarObservation: {
        const auto& payload = std::get<DisabledLidarObservationPayload>(envelope.payload);
        return finite_scalar(payload.inferred_height_m) && finite_quat_sigma(payload.sigma_m);
    }
    }
    return false;
}

bool is_measurement_envelope_valid(const MeasurementEnvelope& envelope) {
    if (!std::isfinite(envelope.timestamp_s) || envelope.timestamp_s < 0.0) {
        return false;
    }
    if (envelope.source_id.empty()) {
        return false;
    }
    if (envelope.covariance_hint.has_value() &&
        (!std::isfinite(*envelope.covariance_hint) || *envelope.covariance_hint < 0.0)) {
        return false;
    }
    return is_measurement_payload_finite(envelope);
}

} // namespace drone::vio
