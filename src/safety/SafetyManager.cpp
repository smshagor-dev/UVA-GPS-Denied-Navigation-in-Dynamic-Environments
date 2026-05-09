// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "safety/SafetyManager.hpp"

#include <algorithm>
#include <cctype>

namespace drone::safety {

namespace {

constexpr double kMinOperationalSpeedMps = 0.10;

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

Eigen::Vector3d body_up(const vio::PoseEstimate& pose) {
    return pose.R_wb() * Eigen::Vector3d{0.0, 1.0, 0.0};
}

} // namespace

std::string_view to_string(SafetyState state) {
    switch (state) {
    case SafetyState::NORMAL: return "NORMAL";
    case SafetyState::DEGRADED_LOCALIZATION: return "DEGRADED_LOCALIZATION";
    case SafetyState::LOCALIZATION_LOST: return "LOCALIZATION_LOST";
    case SafetyState::LINK_LOST: return "LINK_LOST";
    case SafetyState::SENSOR_FAULT: return "SENSOR_FAULT";
    case SafetyState::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case SafetyState::MOTOR_LOCKED: return "MOTOR_LOCKED";
    }
    return "UNKNOWN";
}

SafetyManager::SafetyManager(SafetyConfig cfg)
    : cfg_(cfg) {}

SafetyDecision SafetyManager::evaluate(const SafetyContext& ctx) const {
    SafetyDecision out;
    out.remote_command_allowed = ctx.security.remote_command_allowed;

    if (ctx.indoor_mode) {
        out.max_speed_mps = cfg_.indoor_max_speed_mps;
        out.max_acceleration_mps2 = cfg_.indoor_max_acceleration_mps2;
        out.summary = "Indoor safety envelope active";
    }

    const bool production_requires_lidar =
        ctx.runtime_mode == runtime::RuntimeMode::PRODUCTION && ctx.lidar_required;
    const bool low_vio_confidence =
        uses_visual_localization(ctx.localization_source) &&
        ctx.localization_confidence < cfg_.low_vio_confidence_threshold;

    if (ctx.emergency_stop_requested ||
        ctx.security.state == security::DroneSecurityState::LAND_IMMEDIATELY) {
        out.state = SafetyState::EMERGENCY_LAND;
        out.arming_allowed = false;
        out.autonomous_flight_allowed = false;
        out.mission_command_allowed = false;
        out.max_speed_mps = std::min(out.max_speed_mps, cfg_.emergency_descent_mps);
        out.max_acceleration_mps2 = std::min(out.max_acceleration_mps2, cfg_.indoor_max_acceleration_mps2);
        out.summary = ctx.emergency_stop_requested
            ? "Emergency stop requested, immediate landing enforced"
            : "Security state forced immediate landing";
        return out;
    }

    if (ctx.motor_locked) {
        out.state = SafetyState::MOTOR_LOCKED;
        out.arming_allowed = false;
        out.autonomous_flight_allowed = false;
        out.mission_command_allowed = false;
        out.max_speed_mps = kMinOperationalSpeedMps;
        out.max_acceleration_mps2 = cfg_.low_vio_max_acceleration_mps2;
        out.summary = "Motor system locked or faulted, arming blocked";
        return out;
    }

    if (ctx.sensor_fault || !ctx.imu_available || !ctx.camera_available ||
        (production_requires_lidar && !ctx.lidar_available)) {
        out.state = SafetyState::SENSOR_FAULT;
        out.arming_allowed = false;
        out.autonomous_flight_allowed = false;
        out.mission_command_allowed = false;
        out.max_speed_mps = kMinOperationalSpeedMps;
        out.max_acceleration_mps2 = cfg_.low_vio_max_acceleration_mps2;
        out.summary = production_requires_lidar && !ctx.lidar_available
            ? "Required LiDAR unavailable in production mode, autonomous flight blocked"
            : "Required sensor path unavailable, arming blocked";
        return out;
    }

    if (ctx.link_lost) {
        out.state = SafetyState::LINK_LOST;
        out.autonomous_flight_allowed = false;
        out.mission_command_allowed = false;
        out.max_speed_mps = std::min(out.max_speed_mps, cfg_.low_vio_max_speed_mps);
        out.max_acceleration_mps2 = std::min(out.max_acceleration_mps2, cfg_.low_vio_max_acceleration_mps2);
        out.summary = "Link lost or stale telemetry detected, holding until trust recovers";
        return out;
    }

    if (ctx.localization_lost) {
        out.state = SafetyState::LOCALIZATION_LOST;
        out.autonomous_flight_allowed = false;
        out.mission_command_allowed = false;
        out.max_speed_mps = std::min(out.max_speed_mps, cfg_.low_vio_max_speed_mps);
        out.max_acceleration_mps2 = std::min(out.max_acceleration_mps2, cfg_.low_vio_max_acceleration_mps2);
        out.summary = "Localization lost, waypoint mission blocked and recovery posture enforced";
        return out;
    }

    if (ctx.localization_degraded || low_vio_confidence) {
        out.state = SafetyState::DEGRADED_LOCALIZATION;
        out.max_speed_mps = std::min(out.max_speed_mps, cfg_.low_vio_max_speed_mps);
        out.max_acceleration_mps2 = std::min(out.max_acceleration_mps2, cfg_.low_vio_max_acceleration_mps2);
        out.summary = low_vio_confidence
            ? "Visual localization confidence low, velocity constrained"
            : "Localization degraded, slowing to recovery envelope";
    }

    if (ctx.telemetry_stale && out.summary.find("stale telemetry") == std::string::npos) {
        out.summary += " | stale telemetry warning active";
    }
    return out;
}

void SafetyManager::enforce(const SafetyDecision& safety,
                            autonomy::DecisionCommand& command,
                            const vio::PoseEstimate& pose) const {
    if (command.max_acceleration_mps2 <= 0.0) {
        command.max_acceleration_mps2 = safety.max_acceleration_mps2;
    } else {
        command.max_acceleration_mps2 =
            std::min(command.max_acceleration_mps2, safety.max_acceleration_mps2);
    }

    const bool command_already_emergency =
        command.mode == autonomy::BehaviorMode::EMERGENCY_LAND;

    switch (safety.state) {
    case SafetyState::EMERGENCY_LAND:
    case SafetyState::MOTOR_LOCKED:
        command.mode = autonomy::BehaviorMode::EMERGENCY_LAND;
        command.requires_operator_attention = true;
        command.desired_velocity = -body_up(pose) * cfg_.emergency_descent_mps;
        command.desired_yaw_rate_rads = 0.0;
        command.summary = safety.summary;
        break;
    case SafetyState::SENSOR_FAULT:
    case SafetyState::LINK_LOST:
        if (!command_already_emergency) {
            command.mode = autonomy::BehaviorMode::HOLD_POSITION;
            command.requires_operator_attention = true;
            command.desired_velocity = clamp_speed(-pose.velocity * 0.55, safety.max_speed_mps);
            command.desired_yaw_rate_rads = 0.0;
            command.summary = safety.summary;
        }
        break;
    case SafetyState::LOCALIZATION_LOST:
        if (!command_already_emergency) {
            command.mode = autonomy::BehaviorMode::LOCALIZATION_LOST;
            command.requires_operator_attention = true;
            command.desired_velocity =
                clamp_speed((-pose.velocity * 0.60) - (body_up(pose) * cfg_.localization_lost_descent_mps),
                            safety.max_speed_mps);
            command.desired_yaw_rate_rads = 0.0;
            command.summary = safety.summary;
        }
        break;
    case SafetyState::DEGRADED_LOCALIZATION:
    case SafetyState::NORMAL:
        break;
    }

    if (command.mode != autonomy::BehaviorMode::EMERGENCY_LAND) {
        command.desired_velocity = clamp_speed(command.desired_velocity, safety.max_speed_mps);
    }
}

bool SafetyManager::uses_visual_localization(std::string_view source) {
    const auto normalized = lowercase(source);
    return normalized.find("vision") != std::string::npos ||
           normalized.find("vio") != std::string::npos ||
           normalized.find("loop-closure") != std::string::npos;
}

Eigen::Vector3d SafetyManager::clamp_speed(Eigen::Vector3d velocity, double max_speed_mps) {
    const double speed = velocity.norm();
    if (speed > max_speed_mps && speed > 1.0e-6) {
        velocity *= (max_speed_mps / speed);
    }
    return velocity;
}

} // namespace drone::safety
