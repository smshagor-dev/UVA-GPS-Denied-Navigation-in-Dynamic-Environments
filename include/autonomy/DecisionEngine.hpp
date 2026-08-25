// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "autonomy/ExperienceMemory.hpp"
#include "hal/JetsonHAL.hpp"
#include "sensors/CameraSensor.hpp"
#include "vio/EKFEstimator.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace drone::autonomy {

enum class BehaviorMode : uint8_t {
    HOLD_POSITION = 0,
    SEARCH,
    TRACK_TARGET,
    AVOID_OBSTACLE,
    RETURN_HOME,
    EMERGENCY_LAND,
    HOVER_AND_SCAN,
    SAFE_RETURN_BY_ANCHOR,
    LOCALIZATION_DEGRADED,
    LOCALIZATION_LOST
};

inline std::string_view to_string(BehaviorMode mode) {
    switch (mode) {
    case BehaviorMode::HOLD_POSITION:
        return "HOLD_POSITION";
    case BehaviorMode::SEARCH:
        return "SEARCH";
    case BehaviorMode::TRACK_TARGET:
        return "TRACK_TARGET";
    case BehaviorMode::AVOID_OBSTACLE:
        return "AVOID_OBSTACLE";
    case BehaviorMode::RETURN_HOME:
        return "RETURN_HOME";
    case BehaviorMode::EMERGENCY_LAND:
        return "EMERGENCY_LAND";
    case BehaviorMode::HOVER_AND_SCAN:
        return "HOVER_AND_SCAN";
    case BehaviorMode::SAFE_RETURN_BY_ANCHOR:
        return "SAFE_RETURN_BY_ANCHOR";
    case BehaviorMode::LOCALIZATION_DEGRADED:
        return "LOCALIZATION_DEGRADED";
    case BehaviorMode::LOCALIZATION_LOST:
        return "LOCALIZATION_LOST";
    }
    return "UNKNOWN";
}

struct DecisionConfig {
    float min_detection_confidence{0.55f};
    float target_track_confidence{0.65f};
    float critical_obstacle_score{0.72f};
    float low_battery_pct{18.0f};
    float critical_battery_pct{8.0f};
    float max_search_speed_mps{1.2f};
    float max_track_speed_mps{2.0f};
    float max_avoid_speed_mps{2.6f};
    float max_return_speed_mps{1.8f};
    float max_recovery_speed_mps{0.9f};
    float nominal_altitude_m{8.0f};
    float safe_altitude_m{12.0f};
    float degraded_localization_threshold{0.58f};
    float lost_localization_threshold{0.22f};
    float tdoa_recovery_confidence{0.55f};
};

struct PerceptionFocus {
    sensors::Detection detection;
    float score{0.0f};
    float normalized_area{0.0f};
    Eigen::Vector2f image_offset{Eigen::Vector2f::Zero()};
    float estimated_distance_m{0.0f};
};

struct DecisionContext {
    vio::PoseEstimate pose;
    hal::SystemStats system;
    std::optional<sensors::CameraFrame> frame;
    std::optional<MemoryPrior> memory_prior;
    std::optional<Eigen::Vector3d> tdoa_position;
    double tdoa_confidence{0.0};
    double localization_confidence{1.0};
    std::string localization_source{"vision-inertial"};
    bool localization_degraded{false};
    bool localization_lost{false};
    double sync_confidence{1.0};
    size_t visible_anchor_count{0};
    size_t relocalization_count{0};
    bool camera_tracking_nominal{true};
    size_t swarm_peer_count{0};
    bool swarm_follower{false};
    bool inference_ready{false};
    size_t lidar_obstacle_count{0};
    double nearest_lidar_obstacle_m{-1.0};
    double now_s{0.0};
};

struct DecisionCommand {
    BehaviorMode mode{BehaviorMode::HOLD_POSITION};
    Eigen::Vector3d desired_velocity{Eigen::Vector3d::Zero()};
    double desired_yaw_rate_rads{0.0};
    double max_acceleration_mps2{2.5};
    double target_confidence{0.0};
    bool requires_operator_attention{false};
    std::string summary;
    std::optional<PerceptionFocus> focus;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DecisionEngine {
public:
    explicit DecisionEngine(DecisionConfig cfg = DecisionConfig{});

    [[nodiscard]] DecisionCommand update(const DecisionContext& ctx);
    void reset();

    [[nodiscard]] BehaviorMode current_mode() const {
        return mode_;
    }
    [[nodiscard]] Eigen::Vector3d home_position() const {
        return home_position_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    [[nodiscard]] std::optional<PerceptionFocus>
    select_primary_detection(const sensors::CameraFrame& frame) const;

    [[nodiscard]] DecisionCommand build_emergency_land(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_return_home(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_hold(const DecisionContext& ctx, std::string summary) const;
    [[nodiscard]] DecisionCommand build_search(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_hover_and_scan(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_safe_return_by_anchor(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_localization_degraded(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_localization_lost(const DecisionContext& ctx) const;
    [[nodiscard]] DecisionCommand build_track(const DecisionContext& ctx,
                                              const PerceptionFocus& focus) const;
    [[nodiscard]] DecisionCommand build_avoid(const DecisionContext& ctx,
                                              const PerceptionFocus& focus) const;
    [[nodiscard]] DecisionCommand build_lidar_avoid(const DecisionContext& ctx) const;

    [[nodiscard]] static bool is_hazard_label(std::string_view label);
    [[nodiscard]] static bool is_target_label(std::string_view label);
    [[nodiscard]] static bool is_unknown_label(std::string_view label);
    DecisionConfig cfg_;
    BehaviorMode mode_{BehaviorMode::HOLD_POSITION};
    Eigen::Vector3d home_position_{Eigen::Vector3d::Zero()};
    bool home_initialized_{false};
    uint32_t target_miss_count_{0};
};

} // namespace drone::autonomy
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
