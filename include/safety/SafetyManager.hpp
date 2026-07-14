// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "autonomy/DecisionEngine.hpp"
#include "runtime/RuntimeMode.hpp"
#include "security/DroneSecurity.hpp"
#include "vio/EKFEstimator.hpp"

#include <string>
#include <string_view>

namespace drone::safety {

enum class SafetyState : uint8_t {
    NORMAL = 0,
    DEGRADED_LOCALIZATION,
    LOCALIZATION_LOST,
    LINK_LOST,
    SENSOR_FAULT,
    EMERGENCY_LAND,
    MOTOR_LOCKED,
};

[[nodiscard]] std::string_view to_string(SafetyState state);

struct SafetyConfig {
    double indoor_max_speed_mps{0.75};
    double indoor_max_acceleration_mps2{0.60};
    double low_vio_confidence_threshold{0.55};
    double low_vio_max_speed_mps{0.35};
    double low_vio_max_acceleration_mps2{0.40};
    double localization_lost_descent_mps{0.18};
    double emergency_descent_mps{0.85};
};

struct SafetyContext {
    runtime::RuntimeMode runtime_mode{runtime::RuntimeMode::SIMULATION};
    bool indoor_mode{true};
    bool emergency_stop_requested{false};
    bool localization_degraded{false};
    bool localization_lost{false};
    double localization_confidence{1.0};
    std::string localization_source{"vision-inertial"};
    bool link_lost{false};
    bool telemetry_stale{false};
    bool lidar_required{false};
    bool lidar_available{true};
    bool camera_available{true};
    bool imu_available{true};
    bool sensor_fault{false};
    bool motor_locked{false};
    security::DroneSecurityAssessment security{};
};

struct SafetyDecision {
    SafetyState state{SafetyState::NORMAL};
    bool arming_allowed{true};
    bool autonomous_flight_allowed{true};
    bool mission_command_allowed{true};
    bool remote_command_allowed{true};
    double max_speed_mps{4.0};
    double max_acceleration_mps2{2.5};
    std::string summary{"Nominal safety envelope"};
};

class SafetyManager {
public:
    explicit SafetyManager(SafetyConfig cfg = SafetyConfig{});

    [[nodiscard]] SafetyDecision evaluate(const SafetyContext& ctx) const;
    void enforce(const SafetyDecision& safety, autonomy::DecisionCommand& command,
                 const vio::PoseEstimate& pose) const;

private:
    [[nodiscard]] static bool uses_visual_localization(std::string_view source);
    [[nodiscard]] static Eigen::Vector3d clamp_speed(Eigen::Vector3d velocity,
                                                     double max_speed_mps);

    SafetyConfig cfg_;
};

} // namespace drone::safety
