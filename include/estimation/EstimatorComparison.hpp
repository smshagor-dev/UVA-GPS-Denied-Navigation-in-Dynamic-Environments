#pragma once

#include "estimation/StateEstimator.hpp"

#include <string>

namespace drone::estimation {

enum class ShadowHealthState : uint8_t {
    DISABLED = 0,
    STARTING,
    SYNCHRONIZED,
    LAGGING,
    STALE,
    DIVERGED,
    FAILED,
    STOPPED,
};

struct ComparisonThresholds {
    double max_timestamp_delta_s{0.02};
    double position_divergence_m{0.25};
    double velocity_divergence_mps{0.20};
    double orientation_divergence_deg{5.0};
    uint32_t required_consecutive_divergent_samples{10};
};

struct EstimatorComparisonSnapshot {
    bool comparable{false};
    double timestamp_delta_s{0.0};
    double position_delta_m{0.0};
    double velocity_delta_mps{0.0};
    double orientation_delta_deg{0.0};
    double accel_bias_delta{0.0};
    double gyro_bias_delta{0.0};
    double uncertainty_delta{0.0};
    uint64_t comparable_snapshot_count{0};
    uint64_t skipped_comparison_count{0};
    uint32_t consecutive_divergent_samples{0};
    bool diverged{false};
    std::string last_divergence_reason{"none"};
    ShadowHealthState shadow_health{ShadowHealthState::DISABLED};
    drone::vio::EstimatorHealthState active_health{drone::vio::EstimatorHealthState::INITIALIZING};
};

class EstimatorComparison {
public:
    explicit EstimatorComparison(ComparisonThresholds thresholds = {});

    [[nodiscard]] EstimatorComparisonSnapshot compare(const EstimatorSnapshot& active,
                                                      const EstimatorSnapshot& shadow,
                                                      ShadowHealthState shadow_health);

private:
    ComparisonThresholds thresholds_{};
    EstimatorComparisonSnapshot latest_{};
};

} // namespace drone::estimation
