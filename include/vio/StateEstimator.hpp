#pragma once

#include "vio/EKFEstimator.hpp"
#include "vio/MeasurementEnvelope.hpp"

#include <optional>
#include <string>
#include <vector>

namespace drone::vio {

enum class EstimatorHealth : uint8_t {
    Healthy = 0,
    Uninitialized,
    Degraded,
    Failed,
};

struct CovarianceSummary {
    double trace{0.0};
    Eigen::Vector3d position_std_m{Eigen::Vector3d::Zero()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EstimatorStateSnapshot {
    double timestamp_s{0.0};
    bool initialized{false};
    Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    CovarianceSummary covariance{};
    EstimatorHealth health{EstimatorHealth::Uninitialized};
    uint64_t operation_count{0};
    std::string estimator_name{"unknown"};
    std::string estimator_version{"phase15"};
    MeasurementFrame frame{MeasurementFrame::World};
    uint64_t generation{0};
    std::optional<double> last_accepted_timestamp_s{};
    bool fej_enabled{false};
    uint64_t fej_snapshots_created{0};
    uint64_t fej_snapshots_released{0};
    uint64_t fej_jacobian_evaluations{0};
    uint64_t fej_validation_failures{0};
    uint64_t msckf_window_size{0};
    uint64_t msckf_states_created{0};
    uint64_t msckf_states_removed{0};
    double msckf_oldest_state_age_s{0.0};
    uint64_t msckf_deterministic_evictions{0};
    uint64_t triangulation_attempts{0};
    uint64_t triangulation_successes{0};
    uint64_t triangulation_failures{0};
    uint64_t rejected_insufficient_observations{0};
    uint64_t rejected_small_baseline{0};
    uint64_t rejected_negative_depth{0};
    uint64_t rejected_depth_range{0};
    uint64_t rejected_degenerate_geometry{0};
    uint64_t rejected_non_finite_input{0};
    uint64_t rejected_reprojection{0};
    uint64_t active_landmarks{0};
    uint64_t feature_tracks{0};
    uint64_t feature_tracks_created{0};
    uint64_t feature_tracks_removed{0};
    uint64_t feature_updates_attempted{0};
    uint64_t feature_updates_applied{0};
    uint64_t feature_updates_rejected{0};
    uint64_t feature_updates_considered{0};
    uint64_t feature_updates_stacked{0};
    uint64_t nullspace_failures{0};
    uint64_t chi_square_failures{0};
    double residual_norm{0.0};
    uint64_t stacked_measurement_dimension{0};
    uint64_t measurement_rank{0};
    uint64_t chi_square_dof{0};
    double chi_square_threshold{0.0};
    double correction_norm{0.0};
    double innovation_min_eigenvalue{0.0};
    double innovation_condition_number{0.0};
    double covariance_symmetry_error{0.0};
    double covariance_min_eigenvalue{0.0};
    double nullspace_annihilation_norm{0.0};
    double nullspace_orthogonality_error{0.0};
    double nullspace_rank_tolerance{0.0};
    double nullspace_validation_tolerance{0.0};
    uint64_t stacked_matrix_rank{0};
    bool stationary_detected{false};
    double stationary_duration_s{0.0};
    uint64_t zupt_updates_applied{0};
    uint64_t zupt_updates_rejected{0};
    uint64_t detector_state_changes{0};
    std::vector<StationaryIntervalRecord> stationary_intervals{};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StateEstimator {
public:
    virtual ~StateEstimator() = default;

    virtual void configure_validation(const EstimatorValidationConfig& cfg) = 0;
    virtual void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                       const Eigen::Vector3d& v0) = 0;
    [[nodiscard]] virtual EstimatorOperationResult
    process_measurement(const MeasurementEnvelope& envelope) = 0;
    [[nodiscard]] virtual EstimatorStateSnapshot snapshot() const = 0;
    [[nodiscard]] virtual EKFDiagnostics diagnostics() const = 0;
    [[nodiscard]] virtual bool is_initialized() const = 0;
    [[nodiscard]] virtual std::string estimator_name() const = 0;
    [[nodiscard]] virtual std::string estimator_version() const = 0;
};

[[nodiscard]] std::string_view to_string(EstimatorHealth health);
[[nodiscard]] EstimatorHealth health_from_result(EstimatorOperationResult result, bool initialized);

} // namespace drone::vio
