#include "estimation/EstimatorComparison.hpp"

#include <algorithm>
#include <cmath>

namespace drone::estimation {

EstimatorComparison::EstimatorComparison(ComparisonThresholds thresholds)
    : thresholds_(thresholds) {}

EstimatorComparisonSnapshot EstimatorComparison::compare(const EstimatorSnapshot& active,
                                                         const EstimatorSnapshot& shadow,
                                                         ShadowHealthState shadow_health) {
    latest_.shadow_health = shadow_health;
    latest_.active_health = active.diagnostics.health_state;
    latest_.timestamp_delta_s = std::abs(active.pose.timestamp - shadow.pose.timestamp);
    if (shadow_health == ShadowHealthState::FAILED || shadow_health == ShadowHealthState::STALE ||
        shadow_health == ShadowHealthState::DISABLED) {
        latest_.comparable = false;
        ++latest_.skipped_comparison_count;
        return latest_;
    }
    if (latest_.timestamp_delta_s > thresholds_.max_timestamp_delta_s ||
        active.last_sequence_id != shadow.last_sequence_id) {
        latest_.comparable = false;
        latest_.last_divergence_reason = "history_mismatch";
        ++latest_.skipped_comparison_count;
        return latest_;
    }

    latest_.comparable = true;
    latest_.position_delta_m = (active.pose.position - shadow.pose.position).norm();
    latest_.velocity_delta_mps = (active.pose.velocity - shadow.pose.velocity).norm();
    latest_.orientation_delta_deg =
        Eigen::AngleAxisd(active.pose.orientation.conjugate() * shadow.pose.orientation).angle() *
        (180.0 / std::numbers::pi_v<double>);
    latest_.accel_bias_delta = (active.pose.accel_bias - shadow.pose.accel_bias).norm();
    latest_.gyro_bias_delta = (active.pose.gyro_bias - shadow.pose.gyro_bias).norm();
    latest_.uncertainty_delta = std::abs(active.pose.pos_std.norm() - shadow.pose.pos_std.norm());
    ++latest_.comparable_snapshot_count;

    const bool divergent = latest_.position_delta_m > thresholds_.position_divergence_m ||
                           latest_.velocity_delta_mps > thresholds_.velocity_divergence_mps ||
                           latest_.orientation_delta_deg > thresholds_.orientation_divergence_deg;
    if (divergent) {
        latest_.consecutive_divergent_samples += 1;
        if (latest_.position_delta_m > thresholds_.position_divergence_m) {
            latest_.last_divergence_reason = "position_delta";
        } else if (latest_.velocity_delta_mps > thresholds_.velocity_divergence_mps) {
            latest_.last_divergence_reason = "velocity_delta";
        } else {
            latest_.last_divergence_reason = "orientation_delta";
        }
    } else {
        latest_.consecutive_divergent_samples = 0;
        latest_.last_divergence_reason = "none";
    }
    latest_.diverged =
        latest_.consecutive_divergent_samples >= thresholds_.required_consecutive_divergent_samples;
    return latest_;
}

} // namespace drone::estimation
