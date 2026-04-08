#pragma once

#include "localization/TDOALocalizer.hpp"
#include "localization/TimeSyncTracker.hpp"
#include "vio/EKFEstimator.hpp"

#include <optional>
#include <string>

namespace drone::localization {

struct LocalizationFusionInput {
    drone::vio::PoseEstimate vio_pose;
    std::optional<TDOALocalizer::Solution> tdoa_solution;
    bool camera_available{false};
    bool lidar_available{false};
    bool rangefinder_available{false};
    bool optical_flow_available{false};
    bool barometer_available{false};
    double anchor_visibility_ratio{0.0};
    TimeSyncStatus time_sync{};
    double previous_confidence{1.0};
};

struct LocalizationFusionOutput {
    Eigen::Vector3d fused_position{Eigen::Vector3d::Zero()};
    double confidence{1.0};
    double confidence_trend{-0.0};
    double tdoa_weight{0.0};
    std::string source{"vision-inertial"};
    std::string state{"nominal"};
    bool degraded{false};
    bool lost{false};
};

class LocalizationFusion {
public:
    LocalizationFusion() = default;

    [[nodiscard]] LocalizationFusionOutput update(const LocalizationFusionInput& input);

private:
    double last_confidence_{1.0};
};

} // namespace drone::localization
