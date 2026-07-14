// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "hal/JetsonHAL.hpp"
#include "sensors/CameraSensor.hpp"
#include "vio/EKFEstimator.hpp"

#include <deque>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace drone::autonomy {

struct MemoryPrior {
    double drift_trend_m_per_min{0.0};
    double battery_burn_pct_per_min{0.0};
    double obstacle_frequency{0.0};
    double target_frequency{0.0};
    double localization_confidence_avg{1.0};
    double localization_dropout_frequency{0.0};
    double low_feature_frequency{0.0};
    double risk_score{0.0};
    bool recommend_caution{false};
    std::string dominant_label;
    std::string dominant_localization_source;
};

class ExperienceMemory {
public:
    struct Config {
        size_t max_observations_per_drone{512};
        double caution_risk_threshold{0.62};
    };

    ExperienceMemory();
    explicit ExperienceMemory(Config cfg);

    void observe(uint32_t drone_id,
                 const vio::PoseEstimate& pose,
                 const hal::SystemStats& system,
                 const std::optional<sensors::CameraFrame>& frame,
                 size_t swarm_peer_count,
                 double localization_confidence,
                 std::string_view localization_source,
                 bool localization_lost,
                 double now_s);

    [[nodiscard]] MemoryPrior summarize(uint32_t drone_id) const;
    [[nodiscard]] MemoryPrior summarize_fleet() const;
    void reset();

private:
    struct Observation {
        double now_s{0.0};
        double drift_m{0.0};
        double battery_pct{0.0};
        size_t swarm_peer_count{0};
        size_t hazard_count{0};
        size_t target_count{0};
        size_t detection_count{0};
        double localization_confidence{1.0};
        bool localization_lost{false};
        std::string primary_label;
        std::string localization_source;
    };

    [[nodiscard]] MemoryPrior summarize_queue(const std::deque<Observation>& queue) const;
    [[nodiscard]] static bool is_hazard_label(std::string_view label);
    [[nodiscard]] static bool is_target_label(std::string_view label);
    [[nodiscard]] static bool is_unknown_label(std::string_view label);

    Config cfg_;
    std::unordered_map<uint32_t, std::deque<Observation>> history_;
};

} // namespace drone::autonomy
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
