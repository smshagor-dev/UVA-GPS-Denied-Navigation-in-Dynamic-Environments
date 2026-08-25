// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

// EKFEstimator.hpp    Extended Kalman Filter for Visual-Inertial Odometry
// State: [pos(3), vel(3), quat(4), ba(3), bg(3)] = 16-dim
// Drone Swarm Sensor Fusion  |  Phase 2  VIO Pipeline

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace drone::vio {

// State vector layout  (16-dim)
//   [0:2]  position      p  (m, world frame)
//   [3:5]  velocity      v  (m/s, world frame)
//   [6:9]  quaternion    q  (w,x,y,z   world body)
//   [10:12] accel bias   ba (m/sÂ²)
//   [13:15] gyro bias    bg (rad/s)

constexpr int kStateDim = 16;
constexpr int kErrorDim = 15;    // error-state (quaternion  3-param)
constexpr int kImuNoiseDim = 12; // na, ng, nba, nbg (3 each)

using StateVec = Eigen::Matrix<double, kStateDim, 1>;
using ErrorVec = Eigen::Matrix<double, kErrorDim, 1>;
using CovMat = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using FMat = Eigen::Matrix<double, kErrorDim, kErrorDim>;
using GMat = Eigen::Matrix<double, kErrorDim, kImuNoiseDim>;
using QNoiseMat = Eigen::Matrix<double, kImuNoiseDim, kImuNoiseDim>;

struct EKFConfig {
    // IMU noise parameters
    double sigma_na{0.02};    // accel noise  (m/sÂ²)
    double sigma_ng{0.005};   // gyro  noise  (rad/s)
    double sigma_nba{1.0e-4}; // accel bias walk
    double sigma_nbg{1.0e-5}; // gyro  bias walk

    // Vision noise
    double sigma_px{1.5};  // pixel reprojection noise
    double sigma_pz{0.05}; // depth measurement noise (if available)

    // Initial covariance
    double init_pos_std{0.1};
    double init_vel_std{0.05};
    double init_att_std{0.05};
    double init_ba_std{0.01};
    double init_bg_std{0.001};

    // Outlier rejection gate (chiÂ² threshold, 3 dof)
    double mahal_gate{7.815};
};

struct StationaryDetectorConfig {
    bool enabled{false};
    double accel_threshold_mps2{0.15};
    double gyro_threshold_rads{0.02};
    uint32_t window_size{20};
    uint32_t enter_count{16};
    uint32_t exit_count{10};
    double minimum_stationary_time_s{0.15};
    double accel_exit_threshold_mps2{0.25};
    double gyro_exit_threshold_rads{0.04};
};

struct ZuptConfig {
    bool enabled{false};
    double velocity_noise_mps{0.01};
    double max_update_rate_hz{10.0};
};

struct FejConfig {
    bool enabled{false};
    bool validation_checks{true};
    bool diagnostics_enabled{true};
};

struct MsckfConfig {
    bool enabled{false};
    uint32_t max_camera_states{8};
    std::string eviction_policy{"oldest_first"};
    bool diagnostics_enabled{true};
    struct TriangulationConfig {
        bool enabled{false};
        uint32_t minimum_observations{2};
        double minimum_baseline{0.05};
        double maximum_reprojection_error{2.5};
        double minimum_depth{0.1};
        double maximum_depth{50.0};
    } triangulation{};
    struct UpdateConfig {
        bool enabled{false};
        double chi_square_probability{0.95};
        uint32_t minimum_track_length{3};
        uint32_t maximum_track_length{8};
        double maximum_residual{2.5};
        bool validation_checks{true};
        bool diagnostics_enabled{true};
    } update{};
};

struct EstimatorValidationConfig {
    std::string mode{"baseline"};
    bool enable_experimental_hybrid{false};
    bool enable_fej{false};
    bool enable_loop_closure_correction{false};
    bool enable_automatic_zupt{false};
    FejConfig fej{};
    bool enable_shadow_estimator{false};
    bool shadow_comparison_enabled{false};
    uint32_t shadow_max_queue_depth{128};
    double shadow_max_lag_ms{250.0};
    double shadow_position_divergence_m{2.0};
    double shadow_velocity_divergence_mps{1.5};
    double shadow_orientation_divergence_deg{15.0};
    uint32_t shadow_required_consecutive_divergent_samples{3};
    bool reject_non_finite_measurements{true};
    bool require_monotonic_timestamps{true};
    double max_imu_dt_s{0.1};
    double min_imu_dt_s{1.0e-6};
    double covariance_symmetry_tolerance{1.0e-9};
    double variance_negativity_tolerance{1.0e-10};
    double quaternion_min_norm{1.0e-6};
    double zupt_sigma_velocity_mps{0.01};
    StationaryDetectorConfig stationary_detector{};
    ZuptConfig zupt{};
    bool lidar_depth_correction_enabled{false};
    bool diagnostics_enabled{true};
};

enum class EstimatorOperationResult : uint8_t {
    Accepted = 0,
    RejectedNotInitialized,
    RejectedNonFiniteInput,
    RejectedInvalidTimestamp,
    RejectedDuplicateTimestamp,
    RejectedBackwardTimestamp,
    RejectedTimeStepTooSmall,
    RejectedTimeStepTooLarge,
    RejectedInvalidCovariance,
    RejectedInvalidQuaternion,
    RejectedDimensionMismatch,
    RejectedUnsupportedMeasurement,
    RejectedInvalidConfiguration,
    FailedFactorization,
    FailedNumericalValidation,
};

[[nodiscard]] std::string_view to_string(EstimatorOperationResult result);

struct StationaryIntervalRecord {
    double start_timestamp_s{0.0};
    double end_timestamp_s{0.0};
    double duration_s{0.0};
    uint64_t zupt_updates{0};
};

struct EKFDiagnostics {
    bool initialized{false};
    double last_accepted_timestamp{-1.0};
    double last_received_timestamp{-1.0};
    uint64_t accepted_propagation_count{0};
    uint64_t rejected_propagation_count{0};
    uint64_t duplicate_timestamp_count{0};
    uint64_t backward_timestamp_count{0};
    uint64_t invalid_timestep_count{0};
    uint64_t non_finite_input_count{0};
    uint64_t accepted_update_count{0};
    uint64_t rejected_update_count{0};
    uint64_t numerical_failure_count{0};
    uint64_t covariance_failure_count{0};
    uint64_t quaternion_failure_count{0};
    uint64_t zupt_accepted_count{0};
    uint64_t zupt_rejected_count{0};
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
    uint64_t marginalization_attempts{0};
    uint64_t marginalizations_completed{0};
    uint64_t marginalization_failures{0};
    uint64_t marginalization_retiring_state_id{0};
    uint64_t marginalization_affected_tracks{0};
    uint64_t marginalization_constraint_candidates{0};
    uint64_t marginalization_constraints_consumed{0};
    uint64_t marginalization_constraint_failures{0};
    uint64_t marginalization_covariance_dim_before{0};
    uint64_t marginalization_covariance_dim_after{0};
    uint64_t marginalization_stale_references{0};
    double marginalization_covariance_symmetry_error{0.0};
    double marginalization_covariance_min_eigenvalue{0.0};
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
    uint64_t detector_state_change_count{0};
    uint64_t disabled_lidar_correction_count{0};
    std::vector<StationaryIntervalRecord> stationary_intervals{};
    EstimatorOperationResult last_rejection_reason{EstimatorOperationResult::Accepted};
    EstimatorOperationResult last_operation_result{EstimatorOperationResult::Accepted};
};

// Pose output

struct PoseEstimate {
    double timestamp{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};

    // Uncertainty (diagonal of position block)
    Eigen::Vector3d pos_std{Eigen::Vector3d::Ones() * 0.1};
    double drift_m{0.0};
    double localization_confidence{1.0};
    std::string localization_source{"imu-dead-reckoning"};
    bool localization_degraded{false};
    bool localization_lost{false};

    // Derived
    [[nodiscard]] Eigen::Matrix3d R_wb() const {
        return orientation.toRotationMatrix();
    }
    [[nodiscard]] Eigen::Vector3d euler_zyx_deg() const {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
        const Eigen::Vector3d euler_zyx_rad = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        return euler_zyx_rad * (180.0 / std::numbers::pi_v<double>);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EKFEstimator {
public:
    explicit EKFEstimator(EKFConfig cfg = EKFConfig{});

    //  Initialize with known pose
    void reset(const Eigen::Vector3d& p0 = Eigen::Vector3d::Zero(),
               const Eigen::Quaterniond& q0 = Eigen::Quaterniond::Identity(),
               const Eigen::Vector3d& v0 = Eigen::Vector3d::Zero());

    //  IMU propagation
    //   Call at IMU rate (~400 Hz).  dt in seconds.
    void propagate_imu(const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rads,
                       double dt);
    [[nodiscard]] EstimatorOperationResult
    process_imu_measurement(const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rads,
                            double timestamp_s);

    //  Camera update (feature-based)
    //   z_pixels: Nx2 observed pixel coordinates
    //   p_world:  Nx3 corresponding 3-D map points
    //   K:        3x3 camera intrinsic matrix
    void update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                       const std::vector<Eigen::Vector3d>& p_world, const Eigen::Matrix3d& K);

    //  Depth update (from LiDAR plane fit) â”€
    void update_depth(double z_depth_m, double sigma_m = 0.05);

    //  Visual pose correction from frame-to-frame frontend
    void update_visual_pose(const Eigen::Vector3d& observed_position,
                            const Eigen::Vector3d& observed_velocity,
                            double sigma_position_m = 0.35, double sigma_velocity_mps = 0.45);

    //  Zero velocity update (on ground detection)
    void update_zupt();

    //  Query â”€
    [[nodiscard]] PoseEstimate state() const;
    [[nodiscard]] double total_drift_m() const {
        return total_drift_;
    }
    [[nodiscard]] bool is_initialized() const {
        return initialized_;
    }
    void configure_validation(const EstimatorValidationConfig& cfg);
    [[nodiscard]] EstimatorValidationConfig validation_config() const;
    [[nodiscard]] EKFDiagnostics diagnostics() const;
    [[nodiscard]] CovMat covariance() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    struct NominalStateSnapshot {
        Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
        Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
        CovMat P{CovMat::Identity() * 0.01};
        double total_drift{0.0};
    };

    //  Core EKF math
    FMat compute_F(const Eigen::Vector3d& accel_body, const Eigen::Matrix3d& R_wb, double dt) const;

    GMat compute_G(const Eigen::Matrix3d& R_wb) const;

    // Quaternion kinematics
    static Eigen::Quaterniond propagate_quat(const Eigen::Quaterniond& q,
                                             const Eigen::Vector3d& omega, double dt);
    // Box-plus / box-minus for error-state
    static Eigen::Vector3d quat_to_rotvec(const Eigen::Quaterniond& q);
    static Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d& rv);

    [[nodiscard]] bool validate_validation_config(const EstimatorValidationConfig& cfg) const;
    void reset_diagnostics_locked();
    void set_result_locked(EstimatorOperationResult result);
    void note_rejection_locked(EstimatorOperationResult result);
    [[nodiscard]] bool vector_finite(const Eigen::Vector3d& value) const;
    [[nodiscard]] bool matrix_finite(const CovMat& value) const;
    [[nodiscard]] bool quaternion_valid(const Eigen::Quaterniond& value) const;
    [[nodiscard]] bool covariance_valid(const CovMat& value) const;
    [[nodiscard]] bool state_valid(const NominalStateSnapshot& candidate) const;
    [[nodiscard]] double compute_uncertainty_norm(const CovMat& cov) const;
    [[nodiscard]] NominalStateSnapshot snapshot_locked() const;
    void commit_locked(const NominalStateSnapshot& candidate);
    [[nodiscard]] EstimatorOperationResult propagate_imu_locked(const Eigen::Vector3d& accel_mps2,
                                                                const Eigen::Vector3d& gyro_rads,
                                                                double dt,
                                                                bool update_internal_timestamp);
    [[nodiscard]] EstimatorOperationResult apply_error_state_update_locked(
        const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R_meas,
        const std::function<void(NominalStateSnapshot&, const ErrorVec&)>& apply_dx,
        bool count_as_zupt = false);

    // Nominal state
    Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};

    // Error-state covariance
    CovMat P_{CovMat::Identity() * 0.01};

    // IMU noise matrix (constant)
    QNoiseMat Q_imu_{QNoiseMat::Zero()};

    double timestamp_{0.0};
    double total_drift_{0.0};
    bool initialized_{false};
    double last_vision_update_ts_{-1.0};
    double last_depth_update_ts_{-1.0};
    std::optional<double> last_accepted_imu_timestamp_s_{};
    EstimatorValidationConfig validation_cfg_{};
    bool validation_cfg_valid_{true};
    EKFDiagnostics diagnostics_{};

    EKFConfig cfg_;
    mutable std::mutex mtx_;

    std::shared_ptr<spdlog::logger> logger_{spdlog::get("EKF")};
};

} // namespace drone::vio
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
