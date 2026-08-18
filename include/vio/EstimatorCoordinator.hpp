#pragma once

#include "sensors/IMUSensor.hpp"
#include "vio/EKFStateEstimatorAdapter.hpp"

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace drone::vio {

enum class ShadowQueueFullPolicy : uint8_t {
    DropOldest = 0,
    DropNewest,
};

enum class ShadowLifecycleState : uint8_t {
    Disabled = 0,
    Starting,
    Running,
    Degraded,
    Failed,
    Stopping,
    Stopped,
};

struct ShadowCoordinatorConfig {
    bool enabled{false};
    bool comparison_enabled{false};
    uint32_t queue_capacity{128};
    double max_comparison_age_ms{250.0};
    double stale_measurement_threshold_ms{250.0};
    ShadowQueueFullPolicy queue_full_policy{ShadowQueueFullPolicy::DropOldest};
    bool diagnostics_enabled{true};
    size_t recent_history_capacity{32};
    bool start_worker_on_initialize{true};
};

struct ShadowQueueStats {
    size_t capacity{0};
    size_t current_depth{0};
    size_t peak_depth{0};
    uint64_t enqueued_count{0};
    uint64_t dequeued_count{0};
    uint64_t dropped_count{0};
    uint64_t stale_count{0};
    uint64_t processing_failure_count{0};
};

struct EstimatorComparison {
    bool valid{false};
    double timestamp_delta_ms{0.0};
    double active_to_shadow_lag_ms{0.0};
    double position_delta_norm_m{0.0};
    double velocity_delta_norm_mps{0.0};
    double orientation_delta_deg{0.0};
    double accel_bias_delta_norm{0.0};
    double gyro_bias_delta_norm{0.0};
    double covariance_trace_delta{0.0};
    std::string reason{"unavailable"};
    uint64_t generation{0};
};

struct CoordinatorDiagnostics {
    std::string active_estimator_name{"unknown"};
    std::string shadow_estimator_name{"unknown"};
    EstimatorHealth active_health{EstimatorHealth::Uninitialized};
    EstimatorHealth shadow_health{EstimatorHealth::Uninitialized};
    bool shadow_enabled{false};
    bool worker_running{false};
    ShadowLifecycleState lifecycle_state{ShadowLifecycleState::Disabled};
    uint64_t active_processed_count{0};
    uint64_t shadow_processed_count{0};
    uint64_t valid_comparison_count{0};
    uint64_t invalid_comparison_count{0};
    uint64_t shadow_restart_count{0};
    uint64_t shadow_failure_count{0};
    uint64_t reset_generation{0};
    double shadow_lag_ms{0.0};
    double peak_shadow_lag_ms{0.0};
    double last_comparison_timestamp_s{0.0};
    std::string last_shadow_failure_reason{};
    ShadowQueueStats queue{};
    EstimatorComparison last_comparison{};
};

class EstimatorCoordinator {
public:
    EstimatorCoordinator(std::unique_ptr<StateEstimator> active,
                         std::unique_ptr<StateEstimator> shadow,
                         ShadowCoordinatorConfig shadow_config = {});
    ~EstimatorCoordinator();

    void configure_validation(const EstimatorValidationConfig& cfg);
    void configure_shadow_msckf(const MsckfConfig& cfg);
    void initialize(const Eigen::Vector3d& p0 = Eigen::Vector3d::Zero(),
                    const Eigen::Quaterniond& q0 = Eigen::Quaterniond::Identity(),
                    const Eigen::Vector3d& v0 = Eigen::Vector3d::Zero());
    [[nodiscard]] EstimatorOperationResult process_measurement(const MeasurementEnvelope& envelope);
    void reset();
    bool start();
    void stop();
    void flush_shadow();

    [[nodiscard]] PoseEstimate active_pose() const;
    [[nodiscard]] std::optional<EstimatorStateSnapshot> shadow_snapshot() const;
    [[nodiscard]] CoordinatorDiagnostics diagnostics() const;
    [[nodiscard]] EstimatorStateSnapshot active_snapshot() const;
    [[nodiscard]] size_t queue_depth() const;

private:
    struct QueuedMeasurement {
        MeasurementEnvelope envelope{};
        uint64_t generation{0};
        uint64_t enqueue_index{0};
    };

    void publish_shadow_measurement(const MeasurementEnvelope& envelope, uint64_t generation);
    void worker_loop();
    void set_lifecycle_locked(ShadowLifecycleState state);
    void update_shadow_snapshot_locked(const EstimatorStateSnapshot& snapshot);
    void update_comparison_locked(const EstimatorStateSnapshot& active,
                                  const EstimatorStateSnapshot& shadow, uint64_t generation);
    [[nodiscard]] uint64_t current_generation_locked() const {
        return reset_generation_;
    }
    [[nodiscard]] static double quaternion_angular_delta_deg(const Eigen::Quaterniond& a,
                                                             const Eigen::Quaterniond& b);

    std::unique_ptr<StateEstimator> active_;
    std::unique_ptr<StateEstimator> shadow_;
    ShadowCoordinatorConfig shadow_cfg_{};
    EstimatorValidationConfig validation_cfg_{};
    MsckfConfig shadow_msckf_cfg_{};

    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable idle_cv_;
    std::deque<QueuedMeasurement> queue_;
    std::thread worker_;
    bool worker_stop_requested_{false};
    bool worker_running_{false};
    bool worker_started_once_{false};
    uint64_t next_enqueue_index_{0};
    uint64_t reset_generation_{0};
    uint64_t worker_inflight_{0};
    CoordinatorDiagnostics diagnostics_{};
    std::optional<EstimatorStateSnapshot> shadow_snapshot_{};
};

[[nodiscard]] MeasurementEnvelope make_imu_envelope(const sensors::ImuMeasurement& imu,
                                                    uint64_t sequence_id);
[[nodiscard]] MeasurementEnvelope
make_visual_pose_envelope(const VisualPoseMeasurementPayload& payload, MeasurementStamp stamp);
[[nodiscard]] MeasurementEnvelope
make_visual_features_envelope(const VisualFeatureMeasurementPayload& payload,
                              MeasurementStamp stamp);
[[nodiscard]] MeasurementEnvelope make_manual_zupt_envelope(MeasurementStamp stamp,
                                                            double sigma_velocity_mps = 0.01);
[[nodiscard]] MeasurementEnvelope make_lidar_depth_envelope(MeasurementStamp stamp, double depth_m,
                                                            double sigma_m,
                                                            bool correction_enabled);

} // namespace drone::vio
