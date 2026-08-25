#pragma once

#include "vio/EstimatorCoordinator.hpp"

#include <cstdint>
#include <string_view>

namespace drone::vio {

enum class PromotionBlockReason : uint8_t {
    None = 0,
    ShadowDisabled,
    WorkerNotRunning,
    LifecycleNotRunning,
    ActiveUnhealthy,
    ShadowUnhealthy,
    InsufficientComparisons,
    InvalidComparison,
    QueueDropsObserved,
    StaleMeasurementsObserved,
    ShadowProcessingFailuresObserved,
    PositionDeltaExceeded,
    VelocityDeltaExceeded,
    OrientationDeltaExceeded,
    CovarianceTraceDeltaExceeded,
};

struct PromotionReadinessConfig {
    uint64_t minimum_valid_comparisons{100};
    double max_position_delta_m{0.25};
    double max_velocity_delta_mps{0.35};
    double max_orientation_delta_deg{5.0};
    double max_abs_covariance_trace_delta{1.0};
    bool require_zero_queue_drops{true};
    bool require_zero_stale_measurements{true};
    bool require_zero_processing_failures{true};
};

struct PromotionReadinessResult {
    bool ready{false};
    PromotionBlockReason reason{PromotionBlockReason::ShadowDisabled};
};

[[nodiscard]] constexpr std::string_view to_string(PromotionBlockReason reason) {
    switch (reason) {
    case PromotionBlockReason::None:
        return "ready";
    case PromotionBlockReason::ShadowDisabled:
        return "shadow_disabled";
    case PromotionBlockReason::WorkerNotRunning:
        return "worker_not_running";
    case PromotionBlockReason::LifecycleNotRunning:
        return "lifecycle_not_running";
    case PromotionBlockReason::ActiveUnhealthy:
        return "active_unhealthy";
    case PromotionBlockReason::ShadowUnhealthy:
        return "shadow_unhealthy";
    case PromotionBlockReason::InsufficientComparisons:
        return "insufficient_comparisons";
    case PromotionBlockReason::InvalidComparison:
        return "invalid_comparison";
    case PromotionBlockReason::QueueDropsObserved:
        return "queue_drops_observed";
    case PromotionBlockReason::StaleMeasurementsObserved:
        return "stale_measurements_observed";
    case PromotionBlockReason::ShadowProcessingFailuresObserved:
        return "shadow_processing_failures_observed";
    case PromotionBlockReason::PositionDeltaExceeded:
        return "position_delta_exceeded";
    case PromotionBlockReason::VelocityDeltaExceeded:
        return "velocity_delta_exceeded";
    case PromotionBlockReason::OrientationDeltaExceeded:
        return "orientation_delta_exceeded";
    case PromotionBlockReason::CovarianceTraceDeltaExceeded:
        return "covariance_trace_delta_exceeded";
    }
    return "unknown";
}

[[nodiscard]] inline PromotionReadinessResult
assess_promotion_readiness(const CoordinatorDiagnostics& diagnostics,
                           const PromotionReadinessConfig& cfg = {}) {
    auto blocked = [](PromotionBlockReason reason) {
        return PromotionReadinessResult{false, reason};
    };

    if (!diagnostics.shadow_enabled) {
        return blocked(PromotionBlockReason::ShadowDisabled);
    }
    if (!diagnostics.worker_running) {
        return blocked(PromotionBlockReason::WorkerNotRunning);
    }
    if (diagnostics.lifecycle_state != ShadowLifecycleState::Running) {
        return blocked(PromotionBlockReason::LifecycleNotRunning);
    }
    if (diagnostics.active_health != EstimatorHealth::Healthy) {
        return blocked(PromotionBlockReason::ActiveUnhealthy);
    }
    if (diagnostics.shadow_health != EstimatorHealth::Healthy) {
        return blocked(PromotionBlockReason::ShadowUnhealthy);
    }
    if (diagnostics.valid_comparison_count < cfg.minimum_valid_comparisons) {
        return blocked(PromotionBlockReason::InsufficientComparisons);
    }
    if (!diagnostics.last_comparison.valid) {
        return blocked(PromotionBlockReason::InvalidComparison);
    }
    if (cfg.require_zero_queue_drops && diagnostics.queue.dropped_count != 0u) {
        return blocked(PromotionBlockReason::QueueDropsObserved);
    }
    if (cfg.require_zero_stale_measurements && diagnostics.queue.stale_count != 0u) {
        return blocked(PromotionBlockReason::StaleMeasurementsObserved);
    }
    if (cfg.require_zero_processing_failures && diagnostics.queue.processing_failure_count != 0u) {
        return blocked(PromotionBlockReason::ShadowProcessingFailuresObserved);
    }
    if (diagnostics.last_comparison.position_delta_norm_m > cfg.max_position_delta_m) {
        return blocked(PromotionBlockReason::PositionDeltaExceeded);
    }
    if (diagnostics.last_comparison.velocity_delta_norm_mps > cfg.max_velocity_delta_mps) {
        return blocked(PromotionBlockReason::VelocityDeltaExceeded);
    }
    if (diagnostics.last_comparison.orientation_delta_deg > cfg.max_orientation_delta_deg) {
        return blocked(PromotionBlockReason::OrientationDeltaExceeded);
    }
    const double covariance_delta = diagnostics.last_comparison.covariance_trace_delta;
    if (covariance_delta > cfg.max_abs_covariance_trace_delta ||
        covariance_delta < -cfg.max_abs_covariance_trace_delta) {
        return blocked(PromotionBlockReason::CovarianceTraceDeltaExceeded);
    }

    return {true, PromotionBlockReason::None};
}

} // namespace drone::vio
