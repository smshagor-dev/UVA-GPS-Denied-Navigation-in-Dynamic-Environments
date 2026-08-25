#include "vio/EstimatorCoordinator.hpp"

#include "sensors/IMUSensor.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace drone::vio {

namespace {

bool finite_snapshot(const EstimatorStateSnapshot& snapshot) {
    return snapshot.position_m.array().isFinite().all() &&
           snapshot.velocity_mps.array().isFinite().all() &&
           snapshot.orientation.coeffs().array().isFinite().all() &&
           snapshot.accel_bias.array().isFinite().all() &&
           snapshot.gyro_bias.array().isFinite().all() &&
           std::isfinite(snapshot.covariance.trace) &&
           snapshot.covariance.position_std_m.array().isFinite().all();
}

} // namespace

EstimatorCoordinator::EstimatorCoordinator(std::unique_ptr<StateEstimator> active,
                                           std::unique_ptr<StateEstimator> shadow,
                                           ShadowCoordinatorConfig shadow_config)
    : active_(std::move(active)), shadow_(std::move(shadow)), shadow_cfg_(shadow_config) {
    diagnostics_.active_estimator_name = active_ ? active_->estimator_name() : "none";
    diagnostics_.shadow_estimator_name = shadow_ ? shadow_->estimator_name() : "none";
    diagnostics_.shadow_enabled = shadow_cfg_.enabled && static_cast<bool>(shadow_);
    diagnostics_.lifecycle_state = diagnostics_.shadow_enabled ? ShadowLifecycleState::Stopped
                                                               : ShadowLifecycleState::Disabled;
    diagnostics_.queue.capacity = shadow_cfg_.queue_capacity;
}

EstimatorCoordinator::~EstimatorCoordinator() {
    stop();
}

void EstimatorCoordinator::configure_validation(const EstimatorValidationConfig& cfg) {
    validation_cfg_ = cfg;
    shadow_cfg_.enabled = cfg.enable_shadow_estimator;
    shadow_cfg_.comparison_enabled = cfg.shadow_comparison_enabled;
    shadow_cfg_.queue_capacity = cfg.shadow_max_queue_depth;
    shadow_cfg_.max_comparison_age_ms = cfg.shadow_max_lag_ms;
    shadow_cfg_.stale_measurement_threshold_ms = cfg.shadow_max_lag_ms;
    diagnostics_.shadow_enabled = shadow_cfg_.enabled && static_cast<bool>(shadow_);
    diagnostics_.queue.capacity = shadow_cfg_.queue_capacity;
    if (!diagnostics_.shadow_enabled) {
        diagnostics_.lifecycle_state = ShadowLifecycleState::Disabled;
    }
    if (active_) {
        active_->configure_validation(cfg);
    }
    if (shadow_) {
        shadow_->configure_validation(cfg);
    }
}

void EstimatorCoordinator::configure_shadow_msckf(const MsckfConfig& cfg) {
    shadow_msckf_cfg_ = cfg;
    if (auto* phase17_shadow = dynamic_cast<Phase17StateEstimatorAdapter*>(shadow_.get())) {
        phase17_shadow->configure_msckf(cfg);
    }
}

void EstimatorCoordinator::initialize(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                                      const Eigen::Vector3d& v0) {
    bool shadow_reset_failed = false;
    if (active_) {
        active_->reset(p0, q0, v0);
    }
    if (shadow_) {
        try {
            shadow_->reset(p0, q0, v0);
        } catch (const std::exception& ex) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = ex.what();
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            shadow_reset_failed = true;
        } catch (...) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = "shadow reset failure";
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            shadow_reset_failed = true;
        }
    }
    {
        std::lock_guard lock(mutex_);
        ++reset_generation_;
        diagnostics_.reset_generation = reset_generation_;
        diagnostics_.active_health =
            active_ ? active_->snapshot().health : EstimatorHealth::Uninitialized;
        diagnostics_.shadow_health =
            shadow_reset_failed
                ? EstimatorHealth::Failed
                : (shadow_ ? shadow_->snapshot().health : EstimatorHealth::Uninitialized);
        diagnostics_.last_comparison = {};
        diagnostics_.queue.current_depth = 0;
        shadow_snapshot_.reset();
    }
    if (!shadow_reset_failed && diagnostics_.shadow_enabled &&
        shadow_cfg_.start_worker_on_initialize) {
        (void)start();
    } else if (!diagnostics_.shadow_enabled) {
        stop();
    }
}

EstimatorOperationResult
EstimatorCoordinator::process_measurement(const MeasurementEnvelope& envelope) {
    if (!active_) {
        return EstimatorOperationResult::RejectedInvalidConfiguration;
    }
    if (!is_measurement_envelope_valid(envelope)) {
        std::lock_guard lock(mutex_);
        diagnostics_.active_health = EstimatorHealth::Failed;
        return EstimatorOperationResult::RejectedNonFiniteInput;
    }

    const auto result = active_->process_measurement(envelope);
    {
        std::lock_guard lock(mutex_);
        ++diagnostics_.active_processed_count;
        diagnostics_.active_health = active_->snapshot().health;
    }

    uint64_t generation = 0;
    bool shadow_enabled = false;
    {
        std::lock_guard lock(mutex_);
        shadow_enabled = diagnostics_.shadow_enabled && static_cast<bool>(shadow_);
        generation = reset_generation_;
    }
    if (shadow_enabled) {
        publish_shadow_measurement(envelope, generation);
    }
    return result;
}

void EstimatorCoordinator::reset() {
    bool shadow_reset_failed = false;
    if (active_) {
        active_->reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d::Zero());
    }
    if (shadow_) {
        try {
            shadow_->reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                           Eigen::Vector3d::Zero());
        } catch (const std::exception& ex) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = ex.what();
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            shadow_reset_failed = true;
        } catch (...) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = "shadow reset failure";
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            shadow_reset_failed = true;
        }
    }
    {
        std::lock_guard lock(mutex_);
        ++reset_generation_;
        diagnostics_.queue.stale_count += queue_.size();
        queue_.clear();
        diagnostics_.queue.current_depth = 0;
        diagnostics_.last_comparison = {};
        diagnostics_.reset_generation = reset_generation_;
        if (shadow_reset_failed) {
            diagnostics_.shadow_health = EstimatorHealth::Failed;
        }
        shadow_snapshot_.reset();
    }
    idle_cv_.notify_all();
}

bool EstimatorCoordinator::start() {
    std::lock_guard lock(mutex_);
    if (!diagnostics_.shadow_enabled || !shadow_) {
        diagnostics_.lifecycle_state = ShadowLifecycleState::Disabled;
        return false;
    }
    if (worker_running_) {
        return true;
    }
    worker_stop_requested_ = false;
    worker_running_ = true;
    diagnostics_.worker_running = true;
    if (worker_started_once_) {
        ++diagnostics_.shadow_restart_count;
    } else {
        worker_started_once_ = true;
    }
    diagnostics_.lifecycle_state = ShadowLifecycleState::Starting;
    worker_ = std::thread([this] { worker_loop(); });
    return true;
}

void EstimatorCoordinator::stop() {
    {
        std::lock_guard lock(mutex_);
        if (!worker_running_ && !worker_.joinable()) {
            diagnostics_.worker_running = false;
            if (diagnostics_.shadow_enabled) {
                diagnostics_.lifecycle_state = ShadowLifecycleState::Stopped;
            }
            return;
        }
        worker_stop_requested_ = true;
        diagnostics_.lifecycle_state = ShadowLifecycleState::Stopping;
    }
    queue_cv_.notify_all();
    idle_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    {
        std::lock_guard lock(mutex_);
        worker_running_ = false;
        diagnostics_.worker_running = false;
        diagnostics_.lifecycle_state = diagnostics_.shadow_enabled ? ShadowLifecycleState::Stopped
                                                                   : ShadowLifecycleState::Disabled;
    }
}

void EstimatorCoordinator::flush_shadow() {
    std::unique_lock lock(mutex_);
    idle_cv_.wait(lock, [this] { return queue_.empty() && worker_inflight_ == 0; });
}

PoseEstimate EstimatorCoordinator::active_pose() const {
    PoseEstimate pose;
    const auto snapshot = active_snapshot();
    pose.timestamp = snapshot.timestamp_s;
    pose.position = snapshot.position_m;
    pose.velocity = snapshot.velocity_mps;
    pose.orientation = snapshot.orientation;
    pose.accel_bias = snapshot.accel_bias;
    pose.gyro_bias = snapshot.gyro_bias;
    pose.pos_std = snapshot.covariance.position_std_m;
    pose.localization_source = snapshot.estimator_name;
    pose.localization_degraded = snapshot.health == EstimatorHealth::Degraded;
    pose.localization_lost = snapshot.health == EstimatorHealth::Failed;
    return pose;
}

std::optional<EstimatorStateSnapshot> EstimatorCoordinator::shadow_snapshot() const {
    std::lock_guard lock(mutex_);
    return shadow_snapshot_;
}

CoordinatorDiagnostics EstimatorCoordinator::diagnostics() const {
    std::lock_guard lock(mutex_);
    return diagnostics_;
}

EstimatorStateSnapshot EstimatorCoordinator::active_snapshot() const {
    return active_ ? active_->snapshot() : EstimatorStateSnapshot{};
}

size_t EstimatorCoordinator::queue_depth() const {
    std::lock_guard lock(mutex_);
    return queue_.size();
}

void EstimatorCoordinator::publish_shadow_measurement(const MeasurementEnvelope& envelope,
                                                      uint64_t generation) {
    std::lock_guard lock(mutex_);
    diagnostics_.queue.capacity = shadow_cfg_.queue_capacity;
    if (!worker_running_ || worker_stop_requested_) {
        return;
    }

    QueuedMeasurement item{envelope, generation, next_enqueue_index_++};
    if (shadow_cfg_.queue_capacity == 0) {
        ++diagnostics_.queue.dropped_count;
        return;
    }
    if (queue_.size() >= shadow_cfg_.queue_capacity) {
        ++diagnostics_.queue.dropped_count;
        if (shadow_cfg_.queue_full_policy == ShadowQueueFullPolicy::DropNewest) {
            return;
        }
        queue_.pop_front();
    }

    queue_.push_back(std::move(item));
    ++diagnostics_.queue.enqueued_count;
    diagnostics_.queue.current_depth = queue_.size();
    diagnostics_.queue.peak_depth = std::max(diagnostics_.queue.peak_depth, queue_.size());
    queue_cv_.notify_one();
}

void EstimatorCoordinator::worker_loop() {
    {
        std::lock_guard lock(mutex_);
        diagnostics_.lifecycle_state = ShadowLifecycleState::Running;
    }

    while (true) {
        QueuedMeasurement item;
        {
            std::unique_lock lock(mutex_);
            queue_cv_.wait(lock, [this] { return worker_stop_requested_ || !queue_.empty(); });
            if (worker_stop_requested_ && queue_.empty()) {
                break;
            }
            item = std::move(queue_.front());
            queue_.pop_front();
            ++worker_inflight_;
            ++diagnostics_.queue.dequeued_count;
            diagnostics_.queue.current_depth = queue_.size();
        }

        {
            std::lock_guard lock(mutex_);
            if (item.generation == reset_generation_) {
                // continue below with current generation
            } else {
                ++diagnostics_.queue.stale_count;
                --worker_inflight_;
                idle_cv_.notify_all();
                continue;
            }
        }

        try {
            const auto result = shadow_->process_measurement(item.envelope);
            const auto shadow_state = shadow_->snapshot();
            const auto active_state = active_->snapshot();

            std::lock_guard lock(mutex_);
            if (item.generation != reset_generation_) {
                ++diagnostics_.queue.stale_count;
                --worker_inflight_;
                idle_cv_.notify_all();
                continue;
            }
            ++diagnostics_.shadow_processed_count;
            diagnostics_.shadow_health = shadow_state.health;
            update_shadow_snapshot_locked(shadow_state);
            if (result != EstimatorOperationResult::Accepted) {
                ++diagnostics_.queue.processing_failure_count;
            }
            update_comparison_locked(active_state, shadow_state, item.generation);
            --worker_inflight_;
            idle_cv_.notify_all();
        } catch (const std::exception& ex) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = ex.what();
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            ++diagnostics_.queue.processing_failure_count;
            queue_.clear();
            diagnostics_.queue.current_depth = 0;
            --worker_inflight_;
            idle_cv_.notify_all();
        } catch (...) {
            std::lock_guard lock(mutex_);
            ++diagnostics_.shadow_failure_count;
            diagnostics_.last_shadow_failure_reason = "unknown shadow worker failure";
            diagnostics_.shadow_health = EstimatorHealth::Failed;
            diagnostics_.lifecycle_state = ShadowLifecycleState::Failed;
            ++diagnostics_.queue.processing_failure_count;
            queue_.clear();
            diagnostics_.queue.current_depth = 0;
            --worker_inflight_;
            idle_cv_.notify_all();
        }
    }
}

void EstimatorCoordinator::set_lifecycle_locked(ShadowLifecycleState state) {
    diagnostics_.lifecycle_state = state;
}

void EstimatorCoordinator::update_shadow_snapshot_locked(const EstimatorStateSnapshot& snapshot) {
    shadow_snapshot_ = snapshot;
}

void EstimatorCoordinator::update_comparison_locked(const EstimatorStateSnapshot& active,
                                                    const EstimatorStateSnapshot& shadow,
                                                    uint64_t generation) {
    diagnostics_.last_comparison = {};
    diagnostics_.last_comparison.generation = generation;

    if (generation != reset_generation_) {
        diagnostics_.last_comparison.reason = "obsolete_generation";
        ++diagnostics_.invalid_comparison_count;
        return;
    }
    if (!active.initialized || !shadow.initialized) {
        diagnostics_.last_comparison.reason = "uninitialized";
        ++diagnostics_.invalid_comparison_count;
        return;
    }
    if (active.frame != shadow.frame) {
        diagnostics_.last_comparison.reason = "frame_mismatch";
        ++diagnostics_.invalid_comparison_count;
        return;
    }
    if (shadow.health == EstimatorHealth::Failed || !finite_snapshot(active) ||
        !finite_snapshot(shadow)) {
        diagnostics_.last_comparison.reason = "invalid_snapshot";
        ++diagnostics_.invalid_comparison_count;
        return;
    }

    const double timestamp_delta_ms = std::abs(active.timestamp_s - shadow.timestamp_s) * 1000.0;
    diagnostics_.last_comparison.timestamp_delta_ms = timestamp_delta_ms;
    diagnostics_.last_comparison.active_to_shadow_lag_ms = timestamp_delta_ms;
    diagnostics_.shadow_lag_ms = timestamp_delta_ms;
    diagnostics_.peak_shadow_lag_ms = std::max(diagnostics_.peak_shadow_lag_ms, timestamp_delta_ms);
    diagnostics_.last_comparison_timestamp_s = std::max(active.timestamp_s, shadow.timestamp_s);
    if (timestamp_delta_ms > shadow_cfg_.max_comparison_age_ms) {
        diagnostics_.last_comparison.reason = "stale";
        ++diagnostics_.invalid_comparison_count;
        return;
    }

    diagnostics_.last_comparison.valid = true;
    diagnostics_.last_comparison.reason = "ok";
    diagnostics_.last_comparison.position_delta_norm_m =
        (active.position_m - shadow.position_m).norm();
    diagnostics_.last_comparison.velocity_delta_norm_mps =
        (active.velocity_mps - shadow.velocity_mps).norm();
    diagnostics_.last_comparison.orientation_delta_deg =
        quaternion_angular_delta_deg(active.orientation, shadow.orientation);
    diagnostics_.last_comparison.accel_bias_delta_norm =
        (active.accel_bias - shadow.accel_bias).norm();
    diagnostics_.last_comparison.gyro_bias_delta_norm =
        (active.gyro_bias - shadow.gyro_bias).norm();
    diagnostics_.last_comparison.covariance_trace_delta =
        std::abs(active.covariance.trace - shadow.covariance.trace);
    ++diagnostics_.valid_comparison_count;
}

double EstimatorCoordinator::quaternion_angular_delta_deg(const Eigen::Quaterniond& a,
                                                          const Eigen::Quaterniond& b) {
    const Eigen::Quaterniond an = a.normalized();
    const Eigen::Quaterniond bn = b.normalized();
    const double dot = std::clamp(std::abs(an.dot(bn)), 0.0, 1.0);
    return 2.0 * std::acos(dot) * (180.0 / std::numbers::pi_v<double>);
}

MeasurementEnvelope make_imu_envelope(const sensors::ImuMeasurement& imu, uint64_t sequence_id) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::Imu;
    envelope.source_id = "imu";
    envelope.timestamp_s = imu.timestamp;
    envelope.sequence_id = sequence_id;
    envelope.frame = MeasurementFrame::Body;
    envelope.payload = ImuMeasurementPayload{imu.accel_mps2, imu.gyro_rads};
    envelope.sensor_reference = imu.source_id;
    return envelope;
}

MeasurementEnvelope make_visual_pose_envelope(const VisualPoseMeasurementPayload& payload,
                                              MeasurementStamp stamp) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::VisualPose;
    envelope.source_id = "camera";
    envelope.timestamp_s = stamp.timestamp_s;
    envelope.sequence_id = stamp.sequence_id;
    envelope.frame = MeasurementFrame::World;
    envelope.payload = payload;
    envelope.covariance_hint = payload.sigma_position_m * payload.sigma_position_m;
    return envelope;
}

MeasurementEnvelope make_visual_features_envelope(const VisualFeatureMeasurementPayload& payload,
                                                  MeasurementStamp stamp) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::VisualFeatures;
    envelope.source_id = "camera_features";
    envelope.timestamp_s = stamp.timestamp_s;
    envelope.sequence_id = stamp.sequence_id;
    envelope.frame = MeasurementFrame::World;
    envelope.payload = payload;
    return envelope;
}

MeasurementEnvelope make_manual_zupt_envelope(MeasurementStamp stamp, double sigma_velocity_mps) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::ManualZupt;
    envelope.source_id = "manual_zupt";
    envelope.timestamp_s = stamp.timestamp_s;
    envelope.sequence_id = stamp.sequence_id;
    envelope.frame = MeasurementFrame::World;
    envelope.payload = ManualZuptMeasurementPayload{sigma_velocity_mps};
    return envelope;
}

MeasurementEnvelope make_lidar_depth_envelope(MeasurementStamp stamp, double depth_m,
                                              double sigma_m, bool correction_enabled) {
    MeasurementEnvelope envelope;
    envelope.type = correction_enabled ? MeasurementType::LidarDepth
                                       : MeasurementType::DisabledLidarObservation;
    envelope.source_id = "lidar";
    envelope.timestamp_s = stamp.timestamp_s;
    envelope.sequence_id = stamp.sequence_id;
    envelope.frame = MeasurementFrame::Lidar;
    if (correction_enabled) {
        envelope.payload = LidarDepthMeasurementPayload{depth_m, sigma_m};
    } else {
        envelope.payload = DisabledLidarObservationPayload{depth_m, sigma_m};
    }
    envelope.covariance_hint = sigma_m * sigma_m;
    return envelope;
}

} // namespace drone::vio
