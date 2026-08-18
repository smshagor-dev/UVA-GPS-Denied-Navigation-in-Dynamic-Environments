#include <gtest/gtest.h>

#include "phase17_test_access.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <Eigen/Geometry>

#include <limits>

using namespace drone::vio;

namespace {

struct Phase22MsckfOptions {
    bool fej_enabled{false};
    uint32_t minimum_observations{3};
    uint32_t minimum_track_length{3};
    uint32_t maximum_track_length{8};
    double minimum_baseline{0.05};
    double maximum_reprojection_error{2.5};
    double maximum_residual{0.1};
    double chi_square_probability{0.999999};
};

struct ObservationSpec {
    Eigen::Vector3d feature_identity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d projection_feature{Eigen::Vector3d::Zero()};
    Eigen::Vector2d pixel_offset{Eigen::Vector2d::Zero()};
};

EstimatorValidationConfig make_phase22_validation_cfg(bool fej_enabled = false) {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.enable_fej = fej_enabled;
    cfg.fej.enabled = fej_enabled;
    cfg.fej.validation_checks = true;
    return cfg;
}

MsckfConfig make_phase22_msckf_cfg(const Phase22MsckfOptions& options = {}) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = 8;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = true;
    cfg.triangulation.enabled = true;
    cfg.triangulation.minimum_observations = options.minimum_observations;
    cfg.triangulation.minimum_baseline = options.minimum_baseline;
    cfg.triangulation.maximum_reprojection_error = options.maximum_reprojection_error;
    cfg.triangulation.minimum_depth = 0.1;
    cfg.triangulation.maximum_depth = 50.0;
    cfg.update.enabled = true;
    cfg.update.chi_square_probability = options.chi_square_probability;
    cfg.update.minimum_track_length = options.minimum_track_length;
    cfg.update.maximum_track_length = options.maximum_track_length;
    cfg.update.maximum_residual = options.maximum_residual;
    cfg.update.validation_checks = true;
    cfg.update.diagnostics_enabled = true;
    return cfg;
}

EKFConfig make_phase22_ekf_cfg() {
    EKFConfig cfg;
    cfg.sigma_px = 1.5;
    cfg.mahal_gate = 1.0e9;
    return cfg;
}

Eigen::Matrix3d camera_intrinsics() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    K(2, 2) = 1.0;
    return K;
}

Eigen::Vector2d project_feature_from_pose(const Eigen::Vector3d& position,
                                          const Eigen::Quaterniond& orientation,
                                          const Eigen::Vector3d& feature) {
    const Eigen::Matrix3d R = orientation.toRotationMatrix();
    const Eigen::Vector3d p_c = R.transpose() * (feature - position);
    const auto K = camera_intrinsics();
    return {
        (K(0, 0) * p_c.x() / p_c.z()) + K(0, 2),
        (K(1, 1) * p_c.y() / p_c.z()) + K(1, 2),
    };
}

Eigen::Vector2d project_feature(const Phase17ESKFEstimator& ekf, const Eigen::Vector3d& feature) {
    const auto state = ekf.state();
    return project_feature_from_pose(state.position, state.orientation, feature);
}

Eigen::Vector2d normalized_prediction_from_pose(const Eigen::Vector3d& position,
                                                const Eigen::Quaterniond& orientation,
                                                const Eigen::Vector3d& feature) {
    const Eigen::Matrix3d R = orientation.toRotationMatrix();
    const Eigen::Vector3d p_c = R.transpose() * (feature - position);
    return {p_c.x() / p_c.z(), p_c.y() / p_c.z()};
}

Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d& rv) {
    const double angle = rv.norm();
    if (angle < 1.0e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv / angle)).normalized();
}

void advance_frame(Phase17ESKFEstimator& ekf, double& timestamp_s,
                   const Eigen::Vector3d& delta_position = Eigen::Vector3d::Zero(),
                   const Eigen::Vector3d& delta_theta = Eigen::Vector3d::Zero()) {
    ASSERT_EQ(ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(),
                                          timestamp_s),
              EstimatorOperationResult::Accepted);
    timestamp_s += 0.01;

    if (!delta_position.isZero(0.0) || !delta_theta.isZero(0.0)) {
        ErrorVec dx = ErrorVec::Zero();
        dx.segment<3>(0) = delta_position;
        dx.segment<3>(6) = delta_theta;
        ASSERT_EQ(ekf.inject_error_for_test(dx), EstimatorOperationResult::Accepted);
    }
}

void observe_feature(Phase17ESKFEstimator& ekf, const ObservationSpec& observation) {
    const Eigen::Vector2d pixel =
        project_feature(ekf, observation.projection_feature) + observation.pixel_offset;
    ekf.update_vision({pixel}, {observation.feature_identity}, camera_intrinsics());
}

void observe_msckf_feature(Phase17ESKFEstimator& ekf, const ObservationSpec& observation) {
    const Eigen::Vector2d pixel =
        project_feature(ekf, observation.projection_feature) + observation.pixel_offset;
    const uint64_t state_id = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_NE(state_id, 0u);
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id, {pixel}, {observation.feature_identity}, camera_intrinsics()));
}

void build_track(Phase17ESKFEstimator& ekf, double& timestamp_s, const Eigen::Vector3d& feature,
                 const std::vector<Eigen::Vector3d>& motion,
                 const std::vector<Eigen::Vector2d>& offsets = {}) {
    for (size_t i = 0; i < motion.size(); ++i) {
        advance_frame(ekf, timestamp_s, motion[i]);
        const Eigen::Vector2d offset = i < offsets.size() ? offsets[i] : Eigen::Vector2d::Zero();
        observe_feature(ekf, ObservationSpec{.feature_identity = feature,
                                             .projection_feature = feature,
                                             .pixel_offset = offset});
    }
}

struct DeterministicOutcome {
    uint64_t attempted{0};
    uint64_t applied{0};
    uint64_t rejected{0};
    uint64_t rank{0};
    uint64_t stacked_dim{0};
    double residual_norm{0.0};
    double covariance_trace{0.0};
};

bool pose_exactly_equal(const PoseEstimate& lhs, const PoseEstimate& rhs) {
    return lhs.timestamp == rhs.timestamp && lhs.position == rhs.position &&
           lhs.velocity == rhs.velocity && lhs.orientation.coeffs() == rhs.orientation.coeffs() &&
           lhs.accel_bias == rhs.accel_bias && lhs.gyro_bias == rhs.gyro_bias;
}

bool matrix_exactly_equal(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs) {
    return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols() && lhs.isApprox(rhs, 1.0e-12);
}

struct FeatureJacobianCase {
    Phase17ESKFEstimator ekf{make_phase22_ekf_cfg()};
    Eigen::Vector3d feature{0.25, -0.08, 4.6};
    double timestamp_s{0.0};
};

void initialize_feature_jacobian_case(FeatureJacobianCase& out, bool fej_enabled = false) {
    out.ekf.configure_validation(make_phase22_validation_cfg(fej_enabled));
    out.ekf.configure_msckf(make_phase22_msckf_cfg());
    out.ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    build_track(out.ekf, out.timestamp_s, out.feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.18, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.12, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.20, -0.10},
                });
}

DeterministicOutcome run_deterministic_case() {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.25, 0.15, 4.5};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.18, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.12, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.35, -0.20},
                });

    const auto diag = ekf.diagnostics();
    DeterministicOutcome out;
    out.attempted = diag.feature_updates_attempted;
    out.applied = diag.feature_updates_applied;
    out.rejected = diag.feature_updates_rejected;
    out.rank = diag.measurement_rank;
    out.stacked_dim = diag.stacked_measurement_dimension;
    out.residual_norm = diag.residual_norm;
    out.covariance_trace = ekf.covariance().trace();
    return out;
}

} // namespace

TEST(Phase22MsckfUpdateTest, SuccessfulFeatureUpdateAppliesShadowCorrection) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, -0.1, 4.2};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.20, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.14, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.40, -0.15},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 1u);
    EXPECT_EQ(diag.feature_updates_applied, 1u);
    EXPECT_EQ(diag.feature_updates_rejected, 0u);
    EXPECT_GT(diag.stacked_measurement_dimension, 0u);
    EXPECT_GT(diag.measurement_rank, 0u);
    EXPECT_TRUE(ekf.covariance().array().isFinite().all());
}

TEST(Phase22MsckfUpdateTest, ZeroResidualUpdateRemainsFinite) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.1, 0.05, 4.8};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.18, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.12, 0.0},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_applied, 1u);
    EXPECT_NEAR(diag.residual_norm, 0.0, 1.0e-6);
    EXPECT_TRUE(std::isfinite(diag.residual_norm));
}

TEST(Phase22MsckfUpdateTest, ExcessiveResidualIsRejected) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg(Phase22MsckfOptions{.maximum_residual = 1.0e-4}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.25, 0.1, 4.1};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.22, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.11, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{4.0, -3.0},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 1u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
    EXPECT_EQ(diag.feature_updates_rejected, 1u);
}

TEST(Phase22MsckfUpdateTest, ChiSquareGatingRejectsOutlier) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(
        make_phase22_msckf_cfg(Phase22MsckfOptions{.chi_square_probability = 1.0e-6}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{-0.2, 0.18, 4.7};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.16, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.13, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.5, 0.25},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 1u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
    EXPECT_EQ(diag.feature_updates_rejected, 1u);
    EXPECT_EQ(diag.chi_square_failures, 1u);
}

TEST(Phase22MsckfUpdateTest, FejJacobiansAreUsedWhenEnabled) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg(true));
    ekf.configure_msckf(make_phase22_msckf_cfg(Phase22MsckfOptions{.fej_enabled = true}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.3, -0.15, 4.9};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.17, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.10, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.25, -0.10},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_applied, 1u);
    EXPECT_GT(diag.fej_jacobian_evaluations, 0u);
}

TEST(Phase22MsckfUpdateTest, DeterministicOrderingIsPreserved) {
    const auto first = run_deterministic_case();
    const auto second = run_deterministic_case();

    EXPECT_EQ(first.attempted, second.attempted);
    EXPECT_EQ(first.applied, second.applied);
    EXPECT_EQ(first.rejected, second.rejected);
    EXPECT_EQ(first.rank, second.rank);
    EXPECT_EQ(first.stacked_dim, second.stacked_dim);
    EXPECT_NEAR(first.residual_norm, second.residual_norm, 1.0e-12);
    EXPECT_NEAR(first.covariance_trace, second.covariance_trace, 1.0e-12);
}

TEST(Phase22MsckfUpdateTest, InvalidUpdateConfigurationFailsValidation) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    auto invalid = make_phase22_msckf_cfg();
    invalid.update.minimum_track_length = 1;
    ekf.configure_msckf(invalid);
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    EXPECT_EQ(
        ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0),
        EstimatorOperationResult::RejectedInvalidConfiguration);
}

TEST(Phase22MsckfUpdateTest, LongTrackProducesProjectedMeasurementSystem) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg(Phase22MsckfOptions{
        .minimum_observations = 3, .minimum_track_length = 3, .maximum_track_length = 5}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{-0.1, 0.2, 5.0};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.12, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.10, 0.0},
                    Eigen::Vector3d{0.12, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.10, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.10, 0.05},
                    Eigen::Vector2d{0.12, 0.05},
                    Eigen::Vector2d{0.15, 0.08},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_GE(diag.feature_updates_attempted, 1u);
    EXPECT_GT(diag.stacked_measurement_dimension, 0u);
    EXPECT_GT(diag.measurement_rank, 0u);
}

TEST(Phase22MsckfUpdateTest, AugmentedJacobianUsesCloneWindowColumns) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario);
    const auto linearization =
        scenario.ekf.feature_update_linearization_for_test(scenario.feature, camera_intrinsics());
    ASSERT_TRUE(linearization.has_value());
    ASSERT_EQ(linearization->state_ids.size(), 3u);
    EXPECT_EQ(linearization->H_state.cols(), 33);
    EXPECT_EQ(linearization->H_feature.cols(), 3);

    for (size_t i = 0; i < linearization->state_ids.size(); ++i) {
        const Eigen::Index row = static_cast<Eigen::Index>(i * 2);
        const Eigen::Index clone_offset = 15 + static_cast<Eigen::Index>(i * 6);
        EXPECT_GT(linearization->H_state.block(row, clone_offset, 2, 6).norm(), 0.0);
        if (clone_offset > 15) {
            EXPECT_LT(linearization->H_state.block(row, 15, 2, clone_offset - 15).norm(), 1.0e-12);
        }
    }
}

TEST(Phase22MsckfUpdateTest, ResidualConventionAndJacobiansMatchFiniteDifferences) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario);
    const auto linearization =
        scenario.ekf.feature_update_linearization_for_test(scenario.feature, camera_intrinsics());
    ASSERT_TRUE(linearization.has_value());
    const auto landmark = scenario.ekf.triangulated_landmark_for_feature_for_test(scenario.feature);
    ASSERT_TRUE(landmark.has_value());
    const uint64_t state_id = linearization->state_ids.front();
    const auto clone = scenario.ekf.msckf_camera_state_for_test(state_id);
    ASSERT_TRUE(clone.has_value());

    const Eigen::Vector2d predicted =
        normalized_prediction_from_pose(clone->position_m, clone->orientation, *landmark);
    const Eigen::Vector2d observed = linearization->residual.segment<2>(0) + predicted;
    const Eigen::MatrixXd H_state = linearization->H_state.block(0, 15, 2, 6);
    const Eigen::MatrixXd H_feature = linearization->H_feature.block(0, 0, 2, 3);
    const double eps = 1.0e-6;

    Eigen::Matrix<double, 2, 3> numeric_position;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d perturbed_position = clone->position_m;
        perturbed_position(axis) += eps;
        const Eigen::Vector2d residual =
            observed -
            normalized_prediction_from_pose(perturbed_position, clone->orientation, *landmark);
        numeric_position.col(axis) = (residual - linearization->residual.segment<2>(0)) / eps;
    }

    Eigen::Matrix<double, 2, 3> numeric_orientation;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d rv = Eigen::Vector3d::Zero();
        rv(axis) = eps;
        const Eigen::Quaterniond perturbed_orientation =
            (clone->orientation * rotvec_to_quat(rv)).normalized();
        const Eigen::Vector2d residual =
            observed -
            normalized_prediction_from_pose(clone->position_m, perturbed_orientation, *landmark);
        numeric_orientation.col(axis) = (residual - linearization->residual.segment<2>(0)) / eps;
    }

    Eigen::Matrix<double, 2, 3> numeric_feature;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d perturbed_feature = *landmark;
        perturbed_feature(axis) += eps;
        const Eigen::Vector2d residual =
            observed - normalized_prediction_from_pose(clone->position_m, clone->orientation,
                                                       perturbed_feature);
        numeric_feature.col(axis) = (residual - linearization->residual.segment<2>(0)) / eps;
    }

    EXPECT_LT((numeric_position - H_state.block<2, 3>(0, 0)).norm(), 1.0e-4);
    EXPECT_LT((numeric_orientation - H_state.block<2, 3>(0, 3)).norm(), 1.0e-4);
    EXPECT_LT((numeric_feature - H_feature).norm(), 1.0e-4);
}

TEST(Phase22MsckfUpdateTest, NullspaceProjectionEliminatesFeatureJacobian) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario);
    const auto projected =
        scenario.ekf.projected_feature_update_for_test(scenario.feature, camera_intrinsics());
    ASSERT_TRUE(projected.has_value());
    EXPECT_GT(projected->residual.size(), 0);
    EXPECT_GT(projected->rank, 0u);
    EXPECT_LT(projected->annihilation_norm, 1.0e-8);
    EXPECT_LT(projected->orthogonality_error, 1.0e-8);
}

TEST(Phase22MsckfUpdateTest, DiagnosticsExposeDofAwareGatingAndCovarianceHealth) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario);
    const auto diag = scenario.ekf.diagnostics();
    EXPECT_GT(diag.feature_updates_considered, 0u);
    EXPECT_GT(diag.feature_updates_stacked, 0u);
    EXPECT_GT(diag.stacked_matrix_rank, 0u);
    EXPECT_GE(diag.chi_square_threshold, 0.0);
    EXPECT_GE(diag.correction_norm, 0.0);
    EXPECT_GT(diag.innovation_min_eigenvalue, 0.0);
    EXPECT_GT(diag.innovation_condition_number, 0.0);
    EXPECT_GE(diag.covariance_symmetry_error, 0.0);
}

TEST(Phase22MsckfUpdateTest, AugmentedCovarianceMatchesWindowSize) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario);
    const Eigen::MatrixXd augmented_covariance = scenario.ekf.augmented_covariance_for_test();
    EXPECT_EQ(augmented_covariance.rows(), 33);
    EXPECT_EQ(augmented_covariance.cols(), 33);
    EXPECT_TRUE(augmented_covariance.array().isFinite().all());
}

TEST(Phase22MsckfUpdateTest, NonFiniteObservationIsRejectedWithoutMutation) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    ASSERT_EQ(
        ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0),
        EstimatorOperationResult::Accepted);

    const auto before_state = ekf.state();
    const auto before_covariance = ekf.augmented_covariance_for_test();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    ekf.update_vision({Eigen::Vector2d{nan, 240.0}}, {Eigen::Vector3d{0.2, 0.0, 4.0}},
                      camera_intrinsics());

    EXPECT_TRUE(pose_exactly_equal(before_state, ekf.state()));
    EXPECT_TRUE(matrix_exactly_equal(before_covariance, ekf.augmented_covariance_for_test()));
    EXPECT_EQ(ekf.diagnostics().last_rejection_reason,
              EstimatorOperationResult::RejectedNonFiniteInput);
}

TEST(Phase22MsckfUpdateTest, ZeroAndNegativeDepthObservationsCreateNoTrack) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    ASSERT_EQ(
        ekf.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0),
        EstimatorOperationResult::Accepted);

    const auto before_state = ekf.state();
    const auto before_covariance = ekf.augmented_covariance_for_test();
    ekf.update_vision({Eigen::Vector2d{320.0, 240.0}, Eigen::Vector2d{320.0, 240.0}},
                      {Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, -1.0}},
                      camera_intrinsics());

    EXPECT_TRUE(pose_exactly_equal(before_state, ekf.state()));
    EXPECT_TRUE(matrix_exactly_equal(before_covariance, ekf.augmented_covariance_for_test()));
    EXPECT_EQ(ekf.feature_track_count_for_test(), 0u);
    EXPECT_EQ(ekf.diagnostics().feature_updates_attempted, 0u);
}

TEST(Phase22MsckfUpdateTest, CloneEvictionKeepsOffsetsAndCovarianceDimensionsConsistent) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    auto cfg = make_phase22_msckf_cfg();
    cfg.max_camera_states = 3;
    ekf.configure_msckf(cfg);
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.18, 0.11, 4.4};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.12, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.12, 0.0},
                    Eigen::Vector3d{0.12, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.12, 0.0},
                });

    const auto state_ids = ekf.msckf_state_ids_for_test();
    ASSERT_EQ(state_ids.size(), 3u);
    EXPECT_EQ(ekf.diagnostics().msckf_deterministic_evictions, 2u);
    const Eigen::MatrixXd augmented_covariance = ekf.augmented_covariance_for_test();
    EXPECT_EQ(augmented_covariance.rows(), 33);
    EXPECT_EQ(augmented_covariance.cols(), 33);

    const auto linearization =
        ekf.feature_update_linearization_for_test(feature, camera_intrinsics());
    ASSERT_TRUE(linearization.has_value());
    ASSERT_EQ(linearization->state_ids.size(), 3u);
    for (size_t i = 0; i < linearization->state_ids.size(); ++i) {
        EXPECT_EQ(linearization->state_ids[i], state_ids[i]);
        const Eigen::Index row = static_cast<Eigen::Index>(i * 2);
        const Eigen::Index clone_offset = 15 + static_cast<Eigen::Index>(i * 6);
        EXPECT_GT(linearization->H_state.block(row, clone_offset, 2, 6).norm(), 0.0);
    }
}

TEST(Phase22MsckfUpdateTest, FejDisabledUsesCurrentCloneStateWithoutFejEvaluations) {
    FeatureJacobianCase scenario;
    initialize_feature_jacobian_case(scenario, false);
    const auto linearization =
        scenario.ekf.feature_update_linearization_for_test(scenario.feature, camera_intrinsics());
    ASSERT_TRUE(linearization.has_value());
    EXPECT_EQ(scenario.ekf.diagnostics().fej_jacobian_evaluations, 0u);
    EXPECT_EQ(scenario.ekf.diagnostics().fej_validation_failures, 0u);
}

TEST(Phase22MsckfUpdateTest, CorruptFejSnapshotRejectsFeatureUpdate) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg(true));
    ekf.configure_msckf(make_phase22_msckf_cfg(Phase22MsckfOptions{.fej_enabled = true}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.22, -0.05, 4.6};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_feature(ekf,
                    ObservationSpec{.feature_identity = feature, .projection_feature = feature});
    ASSERT_GT(ekf.fej_snapshot_ids_for_test().size(), 0u);
    ekf.corrupt_first_fej_snapshot_for_test();
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d{0.16, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.11, 0.0},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_GT(diag.fej_validation_failures, 0u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
}

TEST(Phase22MsckfUpdateTest, CorruptFejCloneRejectsAtMsckfValidationPathWithoutMutation) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg(true));
    ekf.configure_msckf(make_phase22_msckf_cfg(Phase22MsckfOptions{.fej_enabled = true}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.22, -0.05, 4.6};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_msckf_feature(
        ekf, ObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.16, 0.0, 0.0});
    observe_msckf_feature(
        ekf, ObservationSpec{.feature_identity = feature, .projection_feature = feature});

    Phase17ESKFEstimatorTestAccess::corrupt_first_msckf_fej_clone(ekf);
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.11, 0.0});
    const Eigen::Vector2d pixel = project_feature(ekf, feature);
    const uint64_t state_id = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_NE(state_id, 0u);
    const auto before_state = ekf.state();
    const auto before_covariance = ekf.augmented_covariance_for_test();
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id, {pixel}, {feature}, camera_intrinsics()));

    const auto after_state = ekf.state();
    const auto after_covariance = ekf.augmented_covariance_for_test();
    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 1u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
    EXPECT_EQ(diag.feature_updates_rejected, 1u);
    EXPECT_GT(diag.fej_validation_failures, 0u);
    EXPECT_TRUE(pose_exactly_equal(before_state, after_state));
    EXPECT_TRUE(matrix_exactly_equal(before_covariance, after_covariance));
}

TEST(Phase22MsckfUpdateTest, ChiSquareRejectedUpdatePreservesEstimatorState) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(
        make_phase22_msckf_cfg(Phase22MsckfOptions{.chi_square_probability = 1.0e-6}));
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{-0.2, 0.18, 4.7};
    double timestamp_s = 0.0;
    advance_frame(ekf, timestamp_s);
    observe_msckf_feature(
        ekf, ObservationSpec{.feature_identity = feature, .projection_feature = feature});
    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.16, 0.0, 0.0});
    observe_msckf_feature(
        ekf, ObservationSpec{.feature_identity = feature, .projection_feature = feature});

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.13, 0.0});
    const Eigen::Vector2d pixel = project_feature(ekf, feature) + Eigen::Vector2d{0.5, 0.25};
    const uint64_t state_id = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_NE(state_id, 0u);
    const auto before_state = ekf.state();
    const auto before_covariance = ekf.augmented_covariance_for_test();
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id, {pixel}, {feature}, camera_intrinsics()));

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 1u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
    EXPECT_EQ(diag.feature_updates_rejected, 1u);
    EXPECT_EQ(diag.chi_square_failures, 1u);
    EXPECT_TRUE(pose_exactly_equal(before_state, ekf.state()));
    EXPECT_TRUE(matrix_exactly_equal(before_covariance, ekf.augmented_covariance_for_test()));
}

TEST(Phase22MsckfUpdateTest, UpdateDisabledControlProducesNoPhase22Correction) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    auto cfg = make_phase22_msckf_cfg();
    cfg.update.enabled = false;
    ekf.configure_msckf(cfg);
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, -0.1, 4.2};
    double timestamp_s = 0.0;
    build_track(ekf, timestamp_s, feature,
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.20, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.14, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.40, -0.15},
                });

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(diag.feature_updates_attempted, 0u);
    EXPECT_EQ(diag.feature_updates_applied, 0u);
    EXPECT_EQ(diag.feature_updates_rejected, 0u);
}

TEST(Phase22MsckfUpdateTest, InvalidMsckfHookStateIdIsRejectedSafely) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const auto before_state = ekf.state();
    const auto before_covariance = ekf.augmented_covariance_for_test();
    const bool accepted = Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, 999999u, {Eigen::Vector2d{320.0, 240.0}}, {Eigen::Vector3d{0.1, 0.0, 4.0}},
        camera_intrinsics());

    const auto diag = ekf.diagnostics();
    EXPECT_FALSE(accepted);
    EXPECT_EQ(diag.last_rejection_reason, EstimatorOperationResult::RejectedDimensionMismatch);
    EXPECT_EQ(diag.feature_updates_attempted, 0u);
    EXPECT_TRUE(pose_exactly_equal(before_state, ekf.state()));
    EXPECT_TRUE(matrix_exactly_equal(before_covariance, ekf.augmented_covariance_for_test()));
}

TEST(Phase22MsckfUpdateTest, MsckfHookCapturesNormalTrackLifecycleSemantics) {
    Phase17ESKFEstimator ekf(make_phase22_ekf_cfg());
    ekf.configure_validation(make_phase22_validation_cfg());
    ekf.configure_msckf(make_phase22_msckf_cfg());
    ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d feature{0.2, -0.08, 4.4};
    double timestamp_s = 0.0;

    advance_frame(ekf, timestamp_s);
    const uint64_t state_id0 = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id0, {project_feature(ekf, feature)}, {feature}, camera_intrinsics()));
    EXPECT_EQ(ekf.feature_track_count_for_test(), 1u);
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 0u);
    EXPECT_EQ(ekf.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{1u});

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.16, 0.0, 0.0});
    const uint64_t state_id1 = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id1, {project_feature(ekf, feature)}, {feature}, camera_intrinsics()));
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 0u);
    EXPECT_EQ(ekf.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{2u});

    advance_frame(ekf, timestamp_s, Eigen::Vector3d{0.0, 0.11, 0.0});
    const uint64_t state_id2 = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(ekf);
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        ekf, state_id2, {project_feature(ekf, feature)}, {feature}, camera_intrinsics()));

    const auto diag = ekf.diagnostics();
    EXPECT_EQ(ekf.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{3u});
    EXPECT_EQ(ekf.active_landmark_count_for_test(), 1u);
    EXPECT_GE(diag.feature_updates_attempted, 1u);
    EXPECT_GE(diag.feature_updates_applied, 1u);
    EXPECT_EQ(diag.feature_updates_rejected, 0u);
}

TEST(Phase22MsckfUpdateTest, ChiSquareThresholdTracksProjectedDof) {
    Phase17ESKFEstimator short_track(make_phase22_ekf_cfg());
    short_track.configure_validation(make_phase22_validation_cfg());
    short_track.configure_msckf(make_phase22_msckf_cfg());
    short_track.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                      Eigen::Vector3d::Zero());
    double short_timestamp_s = 0.0;
    build_track(short_track, short_timestamp_s, Eigen::Vector3d{0.2, 0.05, 4.5},
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.15, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.15, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.1, -0.1},
                });

    Phase17ESKFEstimator long_track(make_phase22_ekf_cfg());
    long_track.configure_validation(make_phase22_validation_cfg());
    long_track.configure_msckf(make_phase22_msckf_cfg());
    long_track.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero());
    double long_timestamp_s = 0.0;
    build_track(long_track, long_timestamp_s, Eigen::Vector3d{0.2, 0.05, 4.5},
                {
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d{0.15, 0.0, 0.0},
                    Eigen::Vector3d{0.0, 0.15, 0.0},
                    Eigen::Vector3d{0.12, 0.0, 0.0},
                },
                {
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d::Zero(),
                    Eigen::Vector2d{0.1, -0.1},
                    Eigen::Vector2d{0.12, -0.08},
                });

    const auto short_diag = short_track.diagnostics();
    const auto long_diag = long_track.diagnostics();
    EXPECT_GT(short_diag.chi_square_dof, 0u);
    EXPECT_GT(long_diag.chi_square_dof, short_diag.chi_square_dof);
    EXPECT_GT(short_diag.chi_square_threshold, 0.0);
    EXPECT_GT(long_diag.chi_square_threshold, short_diag.chi_square_threshold);
}
