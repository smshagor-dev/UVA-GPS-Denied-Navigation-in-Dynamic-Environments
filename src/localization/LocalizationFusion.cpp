#include "localization/LocalizationFusion.hpp"

#include <algorithm>

namespace drone::localization {

LocalizationFusionOutput LocalizationFusion::update(const LocalizationFusionInput& input) {
    LocalizationFusionOutput out;
    out.fused_position = input.vio_pose.position;

    double confidence = input.vio_pose.localization_confidence;
    double tdoa_weight = 0.0;

    if (input.tdoa_solution.has_value()) {
        const double tdoa_conf = input.tdoa_solution->confidence;
        const double drift_bias = std::clamp(input.vio_pose.drift_m / 2.0, 0.0, 1.0);
        tdoa_weight = std::clamp((tdoa_conf * 0.55) + (drift_bias * 0.45), 0.0, 0.85);
        out.fused_position =
            (input.vio_pose.position * (1.0 - tdoa_weight)) +
            (input.tdoa_solution->position * tdoa_weight);
        confidence = std::max(confidence, (input.vio_pose.localization_confidence * (1.0 - tdoa_weight)) + (tdoa_conf * tdoa_weight));
    }

    if (!input.camera_available) {
        confidence *= 0.72;
    }
    if (!input.lidar_available && !input.rangefinder_available) {
        confidence *= 0.92;
    }
    if (!input.time_sync.synchronized) {
        confidence *= std::clamp(input.time_sync.confidence, 0.35, 1.0);
    }
    confidence *= std::clamp(0.65 + input.anchor_visibility_ratio * 0.35, 0.65, 1.0);
    if (input.tdoa_solution.has_value() && input.time_sync.confidence >= 0.8 && input.anchor_visibility_ratio >= 0.5) {
        const double tdoa_floor =
            (input.tdoa_solution->confidence * 0.55) +
            (std::clamp(input.anchor_visibility_ratio, 0.0, 1.0) * 0.25) +
            (std::clamp(input.time_sync.confidence, 0.0, 1.0) * 0.20);
        confidence = std::max(confidence, tdoa_floor);
    }

    out.tdoa_weight = tdoa_weight;
    out.confidence = std::clamp(confidence, 0.0, 1.0);
    out.confidence_trend = out.confidence - last_confidence_;
    last_confidence_ = out.confidence;

    if (out.confidence < 0.22) {
        out.state = "lost";
        out.lost = true;
        out.degraded = true;
    } else if (out.confidence < 0.58) {
        out.state = "degraded";
        out.degraded = true;
    }

    if (out.lost && input.tdoa_solution.has_value()) {
        out.source = "tdoa-recovery";
    } else if (tdoa_weight > 0.35 && input.tdoa_solution.has_value()) {
        out.source = "vio-tdoa-fused";
    } else if (!input.camera_available) {
        out.source = "imu-dead-reckoning";
    } else if (input.lidar_available || input.rangefinder_available) {
        out.source = "vision-depth-fused";
    } else {
        out.source = "vision-inertial";
    }

    return out;
}

} // namespace drone::localization
