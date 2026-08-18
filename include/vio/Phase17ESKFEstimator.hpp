#pragma once

#include "vio/EKFEstimator.hpp"
#include "vio/MeasurementEnvelope.hpp"
#include "vio/StateEstimator.hpp"

#include <Eigen/StdVector>

#include <functional>
#include <deque>
#include <optional>
#include <unordered_map>
#include <mutex>

namespace drone::vio {

class Phase17ESKFEstimatorTestAccess;

class Phase17ESKFEstimator {
public:
    struct FeatureUpdateLinearizationForTest {
        std::vector<uint64_t> state_ids{};
        Eigen::VectorXd residual{};
        Eigen::MatrixXd H_state{};
        Eigen::MatrixXd H_feature{};
        double raw_residual_norm{0.0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct ProjectedFeatureUpdateForTest {
        Eigen::VectorXd residual{};
        Eigen::MatrixXd H{};
        Eigen::MatrixXd R{};
        uint64_t rank{0};
        double annihilation_norm{0.0};
        double orthogonality_error{0.0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MsckfCameraStateForTest {
        uint64_t state_id{0};
        double timestamp_s{0.0};
        Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d fej_position_m{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond fej_orientation{Eigen::Quaterniond::Identity()};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    explicit Phase17ESKFEstimator(EKFConfig cfg = EKFConfig{});

    void reset(const Eigen::Vector3d& p0 = Eigen::Vector3d::Zero(),
               const Eigen::Quaterniond& q0 = Eigen::Quaterniond::Identity(),
               const Eigen::Vector3d& v0 = Eigen::Vector3d::Zero());

    void propagate_imu(const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rads,
                       double dt);
    [[nodiscard]] EstimatorOperationResult
    process_imu_measurement(const Eigen::Vector3d& accel_mps2, const Eigen::Vector3d& gyro_rads,
                            double timestamp_s);

    void update_vision(const std::vector<Eigen::Vector2d>& z_pixels,
                       const std::vector<Eigen::Vector3d>& p_world, const Eigen::Matrix3d& K);
    void update_depth(double z_depth_m, double sigma_m = 0.05);
    void update_visual_pose(const Eigen::Vector3d& observed_position,
                            const Eigen::Vector3d& observed_velocity,
                            double sigma_position_m = 0.35, double sigma_velocity_mps = 0.45);
    void update_zupt();

    [[nodiscard]] PoseEstimate state() const;
    [[nodiscard]] bool is_initialized() const {
        return initialized_;
    }
    void configure_validation(const EstimatorValidationConfig& cfg);
    void configure_msckf(const MsckfConfig& cfg);
    [[nodiscard]] EstimatorValidationConfig validation_config() const;
    [[nodiscard]] MsckfConfig msckf_config() const;
    [[nodiscard]] EKFDiagnostics diagnostics() const;
    [[nodiscard]] CovMat covariance() const;

    [[nodiscard]] static Eigen::Matrix3d
    attitude_reset_jacobian(const Eigen::Vector3d& injected_delta_theta);
    [[nodiscard]] EstimatorOperationResult inject_error_for_test(const ErrorVec& dx);
    [[nodiscard]] std::vector<uint64_t> fej_snapshot_ids_for_test() const;
    [[nodiscard]] std::vector<uint64_t> msckf_state_ids_for_test() const;
    [[nodiscard]] std::vector<double> msckf_state_timestamps_for_test() const;
    [[nodiscard]] std::optional<double>
    msckf_state_timestamp_for_id_for_test(uint64_t state_id) const;
    [[nodiscard]] std::optional<uint64_t>
    msckf_state_id_for_timestamp_for_test(double timestamp_s) const;
    [[nodiscard]] std::optional<MsckfCameraStateForTest>
    msckf_camera_state_for_test(uint64_t state_id) const;
    [[nodiscard]] uint64_t feature_track_count_for_test() const;
    [[nodiscard]] uint64_t active_landmark_count_for_test() const;
    [[nodiscard]] std::vector<uint64_t> feature_track_ids_for_test() const;
    [[nodiscard]] std::optional<uint64_t>
    feature_track_id_for_feature_for_test(const Eigen::Vector3d& feature_world) const;
    [[nodiscard]] std::optional<size_t> feature_track_observation_count_for_feature_for_test(
        const Eigen::Vector3d& feature_world) const;
    [[nodiscard]] std::optional<Eigen::Vector3d>
    triangulated_landmark_for_feature_for_test(const Eigen::Vector3d& feature_world) const;
    [[nodiscard]] std::optional<FeatureUpdateLinearizationForTest>
    feature_update_linearization_for_test(const Eigen::Vector3d& feature_world,
                                          const Eigen::Matrix3d& K) const;
    [[nodiscard]] std::optional<ProjectedFeatureUpdateForTest>
    projected_feature_update_for_test(const Eigen::Vector3d& feature_world,
                                      const Eigen::Matrix3d& K) const;
    [[nodiscard]] Eigen::MatrixXd augmented_covariance_for_test() const;
    void corrupt_first_fej_snapshot_for_test();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    friend class Phase17ESKFEstimatorTestAccess;

    [[nodiscard]] uint64_t capture_msckf_camera_state_for_test();
    [[nodiscard]] bool process_msckf_observations_for_test(
        uint64_t state_id, const std::vector<Eigen::Vector2d>& z_pixels,
        const std::vector<Eigen::Vector3d>& p_world, const Eigen::Matrix3d& K);
    void corrupt_first_msckf_fej_clone_for_test();

    struct NominalStateSnapshot {
        Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
        Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
        CovMat P{CovMat::Identity() * 0.01};
        double total_drift{0.0};
    };

    struct PropagationModel {
        FMat Fc{FMat::Zero()};
        FMat Phi{FMat::Identity()};
        GMat Gc{GMat::Zero()};
        CovMat Qd{CovMat::Zero()};
        Eigen::Vector3d accel_world{Eigen::Vector3d::Zero()};
    };

    struct StationaryWindowSample {
        double timestamp_s{0.0};
        bool enter_candidate{false};
        bool exit_candidate{false};
    };

    struct FejFeatureKey {
        int64_t x{0};
        int64_t y{0};
        int64_t z{0};

        [[nodiscard]] bool operator==(const FejFeatureKey& other) const = default;
    };

    struct FejFeatureKeyHasher {
        [[nodiscard]] size_t operator()(const FejFeatureKey& key) const noexcept;
    };

    struct FejFeatureSnapshot {
        uint64_t snapshot_id{0};
        FejFeatureKey key{};
        Eigen::Vector3d feature_world{Eigen::Vector3d::Zero()};
        Eigen::Vector3d first_position_m{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond first_orientation{Eigen::Quaterniond::Identity()};
        uint64_t last_observed_epoch{0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MsckfCameraState {
        uint64_t state_id{0};
        int64_t timestamp_key{0};
        double timestamp_s{0.0};
        Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d fej_position_m{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond fej_orientation{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d velocity_reference_mps{Eigen::Vector3d::Zero()};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FeatureObservation {
        uint64_t state_id{0};
        int64_t timestamp_key{0};
        Eigen::Vector3d bearing_c{Eigen::Vector3d::UnitZ()};
        Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FeatureTrack {
        uint64_t track_id{0};
        FejFeatureKey key{};
        Eigen::Vector3d feature_identity_world{Eigen::Vector3d::Zero()};
        std::vector<FeatureObservation, Eigen::aligned_allocator<FeatureObservation>>
            observations{};
        bool landmark_initialized{false};
        Eigen::Vector3d landmark_world{Eigen::Vector3d::Zero()};
        uint64_t last_update_state_id{0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MsckfObservationLinearization {
        uint64_t state_id{0};
        Eigen::Vector2d residual{Eigen::Vector2d::Zero()};
        Eigen::MatrixXd H_state{};
        Eigen::Matrix<double, 2, 3> H_feature{Eigen::Matrix<double, 2, 3>::Zero()};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct ProjectedMeasurementSystem {
        Eigen::VectorXd residual{};
        Eigen::MatrixXd H{};
        Eigen::MatrixXd R{};
        uint64_t rank{0};
        double annihilation_norm{0.0};
        double orthogonality_error{0.0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FeatureUpdateBuildResult {
        std::vector<MsckfObservationLinearization,
                    Eigen::aligned_allocator<MsckfObservationLinearization>>
            linearizations{};
        uint64_t nullspace_rank{0};
        double raw_residual_norm{0.0};
        uint64_t features_considered{0};
        uint64_t projected_dimension{0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct AugmentedStateLayout {
        static constexpr int kBaseErrorDim = kErrorDim;
        static constexpr int kCloneErrorDim = 6;
        static constexpr int kClonePositionOffset = 0;
        static constexpr int kCloneOrientationOffset = 3;

        std::vector<uint64_t> clone_ids{};
        size_t total_dim{static_cast<size_t>(kErrorDim)};

        [[nodiscard]] size_t clone_count() const {
            return clone_ids.size();
        }
        [[nodiscard]] size_t clone_block_offset(size_t clone_index) const {
            return static_cast<size_t>(kBaseErrorDim) +
                   (clone_index * static_cast<size_t>(kCloneErrorDim));
        }
        [[nodiscard]] std::optional<size_t> clone_index_for_state(uint64_t state_id) const;
        [[nodiscard]] std::optional<size_t> clone_block_offset_for_state(uint64_t state_id) const;
    };

    [[nodiscard]] static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    [[nodiscard]] bool fej_enabled_locked() const;
    [[nodiscard]] bool msckf_enabled_locked() const;
    [[nodiscard]] bool msckf_diagnostics_enabled_locked() const;
    [[nodiscard]] bool triangulation_enabled_locked() const;
    [[nodiscard]] bool msckf_update_enabled_locked() const;
    [[nodiscard]] bool msckf_update_diagnostics_enabled_locked() const;
    [[nodiscard]] static FejFeatureKey make_fej_feature_key(const Eigen::Vector3d& feature_world);
    [[nodiscard]] static int64_t make_msckf_timestamp_key(double timestamp_s);
    [[nodiscard]] static Eigen::Vector3d pixel_to_bearing(const Eigen::Vector2d& pixel,
                                                          const Eigen::Matrix3d& K);
    [[nodiscard]] FejFeatureSnapshot*
    get_or_create_fej_snapshot_locked(const Eigen::Vector3d& feature_world);
    [[nodiscard]] FeatureTrack*
    get_or_create_feature_track_locked(const Eigen::Vector3d& feature_world);
    [[nodiscard]] bool fej_snapshot_valid_locked(const FejFeatureSnapshot& snapshot) const;
    void release_inactive_fej_snapshots_locked(uint64_t observation_epoch);
    void reset_fej_locked();
    [[nodiscard]] bool validate_msckf_config(const MsckfConfig& cfg) const;
    void refresh_config_validity_locked();
    void clear_msckf_diagnostics_locked();
    void clear_triangulation_diagnostics_locked();
    void refresh_triangulation_diagnostics_locked();
    void refresh_msckf_oldest_state_age_locked();
    void evict_msckf_state_locked();
    [[nodiscard]] uint64_t capture_or_get_msckf_camera_state_locked(double capture_timestamp_s);
    void maybe_capture_msckf_camera_state_locked(double capture_timestamp_s);
    void record_feature_observations_locked(const std::vector<Eigen::Vector2d>& z_pixels,
                                            const std::vector<Eigen::Vector3d>& feature_world,
                                            const Eigen::Matrix3d& K, uint64_t state_id);
    void prune_feature_tracks_for_state_locked(uint64_t state_id);
    void prune_empty_feature_tracks_locked();
    void try_initialize_triangulated_landmarks_locked(const Eigen::Matrix3d& K);
    void try_apply_msckf_feature_updates_locked(const Eigen::Matrix3d& K, uint64_t state_id);
    [[nodiscard]] bool triangulate_track_locked(const FeatureTrack& track, const Eigen::Matrix3d& K,
                                                Eigen::Vector3d& landmark_world_out,
                                                double& baseline_m_out,
                                                double& max_reprojection_error_px_out);
    [[nodiscard]] bool
    build_feature_update_linearization_locked(const FeatureTrack& track, const Eigen::Matrix3d& K,
                                              FeatureUpdateBuildResult& build_out);
    [[nodiscard]] bool project_feature_nullspace_locked(
        const std::vector<MsckfObservationLinearization,
                          Eigen::aligned_allocator<MsckfObservationLinearization>>& linearizations,
        ProjectedMeasurementSystem& projected_out, uint64_t& nullspace_rank_out);
    void reset_msckf_locked();
    [[nodiscard]] PropagationModel build_propagation_model(const Eigen::Vector3d& accel_body,
                                                           const Eigen::Vector3d& omega_body,
                                                           const Eigen::Quaterniond& q_mid,
                                                           double dt) const;
    [[nodiscard]] static Eigen::Quaterniond propagate_quat(const Eigen::Quaterniond& q,
                                                           const Eigen::Vector3d& omega, double dt);
    [[nodiscard]] static Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d& rv);
    [[nodiscard]] bool validate_validation_config(const EstimatorValidationConfig& cfg) const;
    void reset_diagnostics_locked();
    void set_result_locked(EstimatorOperationResult result);
    void note_rejection_locked(EstimatorOperationResult result);
    [[nodiscard]] bool vector_finite(const Eigen::Vector3d& value) const;
    [[nodiscard]] bool matrix_finite(const CovMat& value) const;
    [[nodiscard]] bool matrix_finite_dynamic(const Eigen::MatrixXd& value) const;
    [[nodiscard]] bool quaternion_valid(const Eigen::Quaterniond& value) const;
    [[nodiscard]] bool covariance_valid(const CovMat& value) const;
    [[nodiscard]] bool covariance_valid_dynamic(const Eigen::MatrixXd& value) const;
    [[nodiscard]] bool state_valid(const NominalStateSnapshot& candidate) const;
    [[nodiscard]] bool auto_zupt_enabled() const;
    [[nodiscard]] double effective_zupt_velocity_noise_mps() const;
    [[nodiscard]] double compute_uncertainty_norm(const CovMat& cov) const;
    [[nodiscard]] NominalStateSnapshot snapshot_locked() const;
    void commit_locked(const NominalStateSnapshot& candidate);
    [[nodiscard]] AugmentedStateLayout build_augmented_state_layout_locked() const;
    [[nodiscard]] bool
    augmented_covariance_dimensions_valid_locked(const AugmentedStateLayout& layout) const;
    void synchronize_augmented_base_covariance_locked();
    void augment_covariance_for_new_clone_locked(uint64_t new_state_id);
    void remove_clone_covariance_block_locked(uint64_t state_id);
    [[nodiscard]] Eigen::MatrixXd
    build_augmented_measurement_jacobian_locked(const Eigen::MatrixXd& H_base,
                                                const AugmentedStateLayout& layout) const;
    void
    apply_clone_error_state_locked(std::unordered_map<uint64_t, MsckfCameraState>& camera_states,
                                   const AugmentedStateLayout& layout,
                                   const Eigen::VectorXd& dx_aug) const;
    [[nodiscard]] Eigen::MatrixXd
    build_augmented_reset_jacobian_locked(const AugmentedStateLayout& layout,
                                          const Eigen::VectorXd& dx_aug) const;
    [[nodiscard]] double chi_square_threshold_for_probability_locked(uint64_t dof) const;
    [[nodiscard]] bool innovation_covariance_valid_locked(const Eigen::MatrixXd& innovation_cov,
                                                          double& min_eigenvalue_out,
                                                          double& condition_number_out) const;
    void reset_stationary_detector_locked();
    void finalize_stationary_interval_locked(double end_timestamp_s);
    void update_stationary_detector_locked(const Eigen::Vector3d& accel_mps2,
                                           const Eigen::Vector3d& gyro_rads, double timestamp_s);
    void maybe_apply_automatic_zupt_locked(double timestamp_s);
    [[nodiscard]] EstimatorOperationResult update_zupt_locked(double sigma_velocity_mps);
    [[nodiscard]] EstimatorOperationResult propagate_imu_locked(const Eigen::Vector3d& accel_mps2,
                                                                const Eigen::Vector3d& gyro_rads,
                                                                double dt,
                                                                bool update_internal_timestamp);
    [[nodiscard]] EstimatorOperationResult inject_error_state_locked(const ErrorVec& dx);
    [[nodiscard]] EstimatorOperationResult apply_error_state_update_locked(
        const Eigen::VectorXd& innovation, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R_meas,
        const std::function<void(NominalStateSnapshot&, const ErrorVec&)>& apply_dx,
        bool inject_attitude, bool count_as_zupt = false);

    Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
    CovMat P_{CovMat::Identity() * 0.01};
    Eigen::MatrixXd augmented_covariance_{CovMat::Identity() * 0.01};
    QNoiseMat Q_imu_{QNoiseMat::Zero()};
    double timestamp_{0.0};
    double total_drift_{0.0};
    bool initialized_{false};
    double last_vision_update_ts_{-1.0};
    double last_depth_update_ts_{-1.0};
    std::optional<double> last_accepted_imu_timestamp_s_{};
    std::unordered_map<FejFeatureKey, FejFeatureSnapshot, FejFeatureKeyHasher, std::equal_to<>,
                       Eigen::aligned_allocator<std::pair<const FejFeatureKey, FejFeatureSnapshot>>>
        fej_snapshots_{};
    uint64_t next_fej_snapshot_id_{1};
    uint64_t fej_observation_epoch_{0};
    std::deque<uint64_t> msckf_state_order_{};
    std::unordered_map<uint64_t, MsckfCameraState> msckf_camera_states_{};
    std::unordered_map<int64_t, uint64_t> msckf_timestamp_to_state_id_{};
    std::unordered_map<FejFeatureKey, FeatureTrack, FejFeatureKeyHasher, std::equal_to<>,
                       Eigen::aligned_allocator<std::pair<const FejFeatureKey, FeatureTrack>>>
        feature_tracks_{};
    uint64_t next_msckf_state_id_{1};
    uint64_t next_feature_track_id_{1};
    MsckfConfig msckf_cfg_{};
    std::deque<StationaryWindowSample> stationary_window_{};
    std::optional<double> stationary_candidate_start_timestamp_s_{};
    std::optional<double> stationary_start_timestamp_s_{};
    std::optional<double> last_automatic_zupt_timestamp_s_{};
    uint64_t current_stationary_interval_zupt_count_{0};
    EstimatorValidationConfig validation_cfg_{};
    bool validation_cfg_valid_{true};
    EKFDiagnostics diagnostics_{};
    EKFConfig cfg_{};
    mutable std::mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_{spdlog::get("EKF")};
};

class Phase17StateEstimatorAdapter : public StateEstimator {
public:
    explicit Phase17StateEstimatorAdapter(EKFConfig cfg = EKFConfig{},
                                          std::string name = "eskf_phase17_shadow",
                                          std::string version = "phase17");

    void configure_validation(const EstimatorValidationConfig& cfg) override;
    void configure_msckf(const MsckfConfig& cfg);
    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override;
    [[nodiscard]] EstimatorOperationResult
    process_measurement(const MeasurementEnvelope& envelope) override;
    [[nodiscard]] EstimatorStateSnapshot snapshot() const override;
    [[nodiscard]] EKFDiagnostics diagnostics() const override;
    [[nodiscard]] bool is_initialized() const override;
    [[nodiscard]] std::string estimator_name() const override;
    [[nodiscard]] std::string estimator_version() const override;
    [[nodiscard]] const Phase17ESKFEstimator& estimator() const {
        return estimator_;
    }

private:
    Phase17ESKFEstimator estimator_;
    std::string name_;
    std::string version_;
    uint64_t generation_{0};
};

} // namespace drone::vio
