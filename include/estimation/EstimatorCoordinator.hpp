#pragma once

#include "estimation/EstimatorComparison.hpp"
#include "estimation/StateEstimator.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <chrono>

namespace drone::estimation {

struct ShadowEstimatorConfig {
    bool enabled{false};
    bool comparison_enabled{true};
    size_t max_queue_depth{512};
    double max_lag_ms{100.0};
    std::string implementation{"minimal_eskf_clone"};
    ComparisonThresholds thresholds{};
};

struct ShadowEstimatorTelemetry {
    bool enabled{false};
    std::string active_estimator_name{"unknown"};
    std::string shadow_estimator_name{"disabled"};
    ShadowHealthState shadow_health{ShadowHealthState::DISABLED};
    drone::vio::EstimatorHealthState active_health{drone::vio::EstimatorHealthState::INITIALIZING};
    size_t queue_depth{0};
    size_t queue_high_water_mark{0};
    uint64_t dropped_events{0};
    double lag_ms{0.0};
    bool comparable{false};
    double position_delta_m{0.0};
    double velocity_delta_mps{0.0};
    double orientation_delta_deg{0.0};
    uint64_t comparable_snapshot_count{0};
    uint64_t skipped_comparison_count{0};
    std::string last_failure_reason{"none"};
    std::string last_divergence_reason{"none"};
    bool divergence_active{false};
};

class EstimatorCoordinator {
public:
    explicit EstimatorCoordinator(std::shared_ptr<StateEstimator> active_estimator);
    ~EstimatorCoordinator();

    void reset(const EstimatorInitialState& initial);
    void configure_shadow(ShadowEstimatorConfig config, std::unique_ptr<StateEstimator> shadow_estimator);
    void stop_shadow();
    [[nodiscard]] bool wait_for_shadow_idle(std::chrono::milliseconds timeout);
    [[nodiscard]] EstimatorUpdateResult process(const EstimatorMeasurement& measurement);
    [[nodiscard]] EstimatorSnapshot active_snapshot() const;
    [[nodiscard]] std::optional<EstimatorSnapshot> shadow_snapshot() const;
    [[nodiscard]] ShadowEstimatorTelemetry telemetry() const;

private:
    struct QueuedMeasurement {
        EstimatorMeasurement measurement{};
        std::chrono::steady_clock::time_point enqueued_at{};
    };

    void worker_loop();
    void update_shadow_telemetry_locked(const EstimatorComparisonSnapshot& comparison);

    std::shared_ptr<StateEstimator> active_estimator_;
    std::unique_ptr<StateEstimator> shadow_estimator_;
    ShadowEstimatorConfig shadow_config_{};
    EstimatorComparison comparison_{};

    mutable std::mutex mutex_;
    mutable std::mutex shadow_mutex_;
    std::queue<QueuedMeasurement> shadow_queue_;
    std::condition_variable shadow_cv_;
    std::condition_variable shadow_idle_cv_;
    std::thread shadow_thread_;
    std::atomic<bool> stop_requested_{false};
    size_t shadow_inflight_{0};
    EstimatorInitialState last_initial_state_{};
    bool has_initial_state_{false};
    std::optional<EstimatorSnapshot> last_shadow_snapshot_;
    ShadowEstimatorTelemetry telemetry_{};
};

} // namespace drone::estimation
