#include "estimation/EstimatorCoordinator.hpp"

#include <chrono>
#include <utility>

namespace drone::estimation {

EstimatorCoordinator::EstimatorCoordinator(std::shared_ptr<StateEstimator> active_estimator)
    : active_estimator_(std::move(active_estimator)) {
    telemetry_.active_estimator_name = std::string(active_estimator_->name());
}

EstimatorCoordinator::~EstimatorCoordinator() {
    stop_shadow();
}

void EstimatorCoordinator::reset(const EstimatorInitialState& initial) {
    std::lock_guard lock(mutex_);
    last_initial_state_ = initial;
    has_initial_state_ = true;
    active_estimator_->reset(initial);
    if (shadow_estimator_) {
        shadow_estimator_->reset(initial);
    }
}

void EstimatorCoordinator::configure_shadow(ShadowEstimatorConfig config,
                                            std::unique_ptr<StateEstimator> shadow_estimator) {
    stop_shadow();
    std::scoped_lock lock(mutex_, shadow_mutex_);
    shadow_config_ = std::move(config);
    comparison_ = EstimatorComparison(shadow_config_.thresholds);
    telemetry_.enabled = shadow_config_.enabled;
    telemetry_.dropped_events = 0;
    telemetry_.queue_depth = 0;
    telemetry_.queue_high_water_mark = 0;
    telemetry_.lag_ms = 0.0;
    telemetry_.last_failure_reason = "none";
    telemetry_.last_divergence_reason = "none";
    telemetry_.divergence_active = false;
    telemetry_.comparable = false;
    telemetry_.comparable_snapshot_count = 0;
    telemetry_.skipped_comparison_count = 0;
    shadow_inflight_ = 0;
    last_shadow_snapshot_.reset();
    while (!shadow_queue_.empty()) {
        shadow_queue_.pop();
    }
    if (!shadow_config_.enabled) {
        telemetry_.shadow_health = ShadowHealthState::DISABLED;
        telemetry_.shadow_estimator_name = "disabled";
        return;
    }
    shadow_estimator_ = std::move(shadow_estimator);
    telemetry_.shadow_estimator_name = std::string(shadow_estimator_->name());
    if (has_initial_state_) {
        shadow_estimator_->reset(last_initial_state_);
    }
    telemetry_.shadow_health = ShadowHealthState::STARTING;
    stop_requested_.store(false);
    shadow_thread_ = std::thread([this] { worker_loop(); });
}

void EstimatorCoordinator::stop_shadow() {
    stop_requested_.store(true);
    shadow_cv_.notify_all();
    shadow_idle_cv_.notify_all();
    if (shadow_thread_.joinable()) {
        shadow_thread_.join();
    }
    std::scoped_lock lock(mutex_, shadow_mutex_);
    while (!shadow_queue_.empty()) {
        shadow_queue_.pop();
    }
    shadow_inflight_ = 0;
    last_shadow_snapshot_.reset();
    shadow_estimator_.reset();
    telemetry_.queue_depth = 0;
    telemetry_.lag_ms = 0.0;
    telemetry_.shadow_health =
        telemetry_.enabled ? ShadowHealthState::STOPPED : ShadowHealthState::DISABLED;
}

bool EstimatorCoordinator::wait_for_shadow_idle(std::chrono::milliseconds timeout) {
    std::unique_lock lock(shadow_mutex_);
    return shadow_idle_cv_.wait_for(
        lock, timeout, [this] { return shadow_queue_.empty() && shadow_inflight_ == 0; });
}

EstimatorUpdateResult EstimatorCoordinator::process(const EstimatorMeasurement& measurement) {
    EstimatorUpdateResult result;
    {
        std::lock_guard lock(mutex_);
        result = active_estimator_->process(measurement);
        telemetry_.active_health = result.snapshot.diagnostics.health_state;
        telemetry_.active_estimator_name = std::string(active_estimator_->name());

        if (!shadow_config_.enabled || !shadow_estimator_) {
            telemetry_.enabled = false;
            telemetry_.shadow_health = ShadowHealthState::DISABLED;
            return result;
        }
    }

    std::scoped_lock lock(mutex_, shadow_mutex_);
    if (shadow_queue_.size() >= shadow_config_.max_queue_depth) {
        shadow_queue_.pop();
        ++telemetry_.dropped_events;
        telemetry_.last_failure_reason = "shadow queue overflow dropped oldest event";
        telemetry_.shadow_health = ShadowHealthState::LAGGING;
    }
    shadow_queue_.push({measurement, std::chrono::steady_clock::now()});
    telemetry_.queue_depth = shadow_queue_.size();
    telemetry_.queue_high_water_mark =
        std::max(telemetry_.queue_high_water_mark, telemetry_.queue_depth);
    shadow_cv_.notify_one();
    return result;
}

EstimatorSnapshot EstimatorCoordinator::active_snapshot() const {
    std::lock_guard lock(mutex_);
    return active_estimator_->snapshot();
}

ShadowEstimatorTelemetry EstimatorCoordinator::telemetry() const {
    std::lock_guard lock(mutex_);
    return telemetry_;
}

std::optional<EstimatorSnapshot> EstimatorCoordinator::shadow_snapshot() const {
    std::lock_guard lock(mutex_);
    return last_shadow_snapshot_;
}

void EstimatorCoordinator::worker_loop() {
    while (!stop_requested_.load()) {
        std::optional<QueuedMeasurement> queued;
        {
            std::unique_lock lock(shadow_mutex_);
            shadow_cv_.wait(lock,
                            [this] { return stop_requested_.load() || !shadow_queue_.empty(); });
            if (stop_requested_.load()) {
                return;
            }
            queued = shadow_queue_.front();
            shadow_queue_.pop();
            shadow_inflight_ += 1;
        }

        try {
            auto shadow_result = shadow_estimator_->process(queued->measurement);
            const double lag_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - queued->enqueued_at)
                                      .count();
            std::scoped_lock lock(mutex_, shadow_mutex_);
            shadow_inflight_ -= 1;
            last_shadow_snapshot_ = shadow_result.snapshot;
            telemetry_.queue_depth = shadow_queue_.size();
            telemetry_.lag_ms = lag_ms;
            telemetry_.shadow_health = lag_ms > shadow_config_.max_lag_ms
                                           ? ShadowHealthState::LAGGING
                                           : ShadowHealthState::SYNCHRONIZED;
            if (telemetry_.dropped_events > 0 && telemetry_.queue_depth == 0) {
                telemetry_.shadow_health = ShadowHealthState::STALE;
            }

            if (shadow_config_.comparison_enabled) {
                const auto comparison =
                    comparison_.compare(active_estimator_->snapshot(), shadow_result.snapshot,
                                        telemetry_.shadow_health);
                update_shadow_telemetry_locked(comparison);
            }
            shadow_idle_cv_.notify_all();
        } catch (const std::exception& ex) {
            std::scoped_lock lock(mutex_, shadow_mutex_);
            if (shadow_inflight_ > 0) {
                shadow_inflight_ -= 1;
            }
            telemetry_.shadow_health = ShadowHealthState::FAILED;
            telemetry_.last_failure_reason = ex.what();
            shadow_idle_cv_.notify_all();
        } catch (...) {
            std::scoped_lock lock(mutex_, shadow_mutex_);
            if (shadow_inflight_ > 0) {
                shadow_inflight_ -= 1;
            }
            telemetry_.shadow_health = ShadowHealthState::FAILED;
            telemetry_.last_failure_reason = "shadow estimator failed with unknown exception";
            shadow_idle_cv_.notify_all();
        }
    }
}

void EstimatorCoordinator::update_shadow_telemetry_locked(
    const EstimatorComparisonSnapshot& comparison) {
    telemetry_.comparable = comparison.comparable;
    telemetry_.position_delta_m = comparison.position_delta_m;
    telemetry_.velocity_delta_mps = comparison.velocity_delta_mps;
    telemetry_.orientation_delta_deg = comparison.orientation_delta_deg;
    telemetry_.comparable_snapshot_count = comparison.comparable_snapshot_count;
    telemetry_.skipped_comparison_count = comparison.skipped_comparison_count;
    telemetry_.last_divergence_reason = comparison.last_divergence_reason;
    telemetry_.divergence_active = comparison.diverged;
    if (comparison.diverged) {
        telemetry_.shadow_health = ShadowHealthState::DIVERGED;
    }
}

} // namespace drone::estimation
