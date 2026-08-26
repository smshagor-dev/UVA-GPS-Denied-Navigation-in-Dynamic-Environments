#include "phase17_test_access.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <gtest/gtest.h>

namespace drone::vio {
namespace {

MsckfConfig make_config(bool diagnostics = true, uint32_t max_camera_states = 2) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = max_camera_states;
    cfg.diagnostics_enabled = diagnostics;
    cfg.triangulation.enabled = false;
    cfg.update.enabled = false;
    return cfg;
}

MsckfConfig make_feature_retirement_config() {
    MsckfConfig cfg = make_config(true, 2);
    cfg.triangulation.enabled = true;
    cfg.triangulation.minimum_observations = 2;
    cfg.triangulation.minimum_baseline = 0.05;
    cfg.triangulation.maximum_reprojection_error = 10.0;
    cfg.update.enabled = false;
    return cfg;
}

MsckfConfig make_feature_update_retirement_config(bool update_enabled) {
    MsckfConfig cfg = make_feature_retirement_config();
    cfg.update.enabled = update_enabled;
    cfg.update.minimum_track_length = 2;
    cfg.update.maximum_track_length = 8;
    cfg.update.maximum_residual = 1.0;
    cfg.update.chi_square_probability = 0.95;
    cfg.update.validation_checks = true;
    cfg.update.diagnostics_enabled = true;
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

Eigen::Vector2d project_feature(const PoseEstimate& pose, const Eigen::Vector3d& feature,
                                const Eigen::Matrix3d& K) {
    const Eigen::Vector3d p_c = pose.R_wb().transpose() * (feature - pose.position);
    EXPECT_GT(p_c.z(), 0.0);
    return {
        (K(0, 0) * p_c.x() / p_c.z()) + K(0, 2),
        (K(1, 1) * p_c.y() / p_c.z()) + K(1, 2),
    };
}

void drive_visual_pose_clones(Phase17ESKFEstimator& estimator, int clone_count) {
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    for (int i = 1; i <= clone_count; ++i) {
        const double timestamp = 0.01 * static_cast<double>(i);
        ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                    Eigen::Vector3d::Zero(), timestamp),
                  EstimatorOperationResult::Accepted);
        const auto pose = estimator.state();
        estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);
    }
}

TEST(MsckfMarginalizationIntegration, TransactionalOldestRetirementKeepsWindowBounded) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config());
    drive_visual_pose_clones(estimator, 3);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    const auto covariance = estimator.augmented_covariance_for_test();
    EXPECT_EQ(covariance.rows(), 27);
    EXPECT_EQ(covariance.cols(), 27);
    EXPECT_TRUE(covariance.array().isFinite().all());

    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 1u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 1u);
    EXPECT_EQ(diagnostics.marginalization_failures, 0u);
    EXPECT_EQ(diagnostics.marginalization_retiring_state_id, 1u);
    EXPECT_EQ(diagnostics.marginalization_covariance_dim_before, 33u);
    EXPECT_EQ(diagnostics.marginalization_covariance_dim_after, 27u);
    EXPECT_EQ(diagnostics.marginalization_stale_references, 0u);
    EXPECT_LE(diagnostics.marginalization_covariance_symmetry_error, 1.0e-9);
}

TEST(MsckfMarginalizationIntegration, DiagnosticsDisabledDoesNotDisableRetirement) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config(false));
    drive_visual_pose_clones(estimator, 3);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), 27);
    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 0u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 0u);
}

TEST(MsckfMarginalizationIntegration, RepeatedRolloverRemainsDeterministicAndBounded) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config(true, 3));
    drive_visual_pose_clones(estimator, 9);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{7u, 8u, 9u}));
    const auto covariance = estimator.augmented_covariance_for_test();
    EXPECT_EQ(covariance.rows(), 33);
    EXPECT_EQ(covariance.cols(), 33);
    EXPECT_TRUE(covariance.array().isFinite().all());
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1.0e-9));

    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 6u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 6u);
    EXPECT_EQ(diagnostics.marginalization_failures, 0u);
    EXPECT_EQ(diagnostics.msckf_states_removed, 6u);
    EXPECT_EQ(diagnostics.msckf_deterministic_evictions, 6u);
    EXPECT_EQ(diagnostics.msckf_window_size, 3u);
    EXPECT_EQ(diagnostics.marginalization_stale_references, 0u);
}

TEST(MsckfMarginalizationIntegration, RetiredCloneMetadataIsRemovedAtomically) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config());
    drive_visual_pose_clones(estimator, 3);

    EXPECT_FALSE(estimator.msckf_camera_state_for_test(1u).has_value());
    EXPECT_FALSE(estimator.msckf_state_timestamp_for_id_for_test(1u).has_value());
    EXPECT_FALSE(estimator.msckf_state_id_for_timestamp_for_test(0.01).has_value());

    ASSERT_TRUE(estimator.msckf_camera_state_for_test(2u).has_value());
    ASSERT_TRUE(estimator.msckf_camera_state_for_test(3u).has_value());
    EXPECT_EQ(estimator.msckf_state_id_for_timestamp_for_test(0.02), std::optional<uint64_t>{2u});
    EXPECT_EQ(estimator.msckf_state_id_for_timestamp_for_test(0.03), std::optional<uint64_t>{3u});
}

TEST(MsckfMarginalizationIntegration, RetiringClonePrunesOnlyItsFeatureObservation) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_feature_retirement_config());

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.01),
              EstimatorOperationResult::Accepted);
    const uint64_t first_id = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(estimator);
    ASSERT_EQ(first_id, 1u);

    const Eigen::Vector3d feature{0.0, 0.0, 4.0};
    const Eigen::Vector2d pixel{320.0, 240.0};
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        estimator, first_id, {pixel}, {feature}, camera_intrinsics()));

    ErrorVec motion = ErrorVec::Zero();
    motion(0) = 0.20;
    ASSERT_EQ(estimator.inject_error_for_test(motion), EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.02),
              EstimatorOperationResult::Accepted);
    const uint64_t second_id =
        Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(estimator);
    ASSERT_EQ(second_id, 2u);
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        estimator, second_id, {pixel}, {feature}, camera_intrinsics()));
    ASSERT_EQ(estimator.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{2u});

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.03),
              EstimatorOperationResult::Accepted);
    const auto pose = estimator.state();
    estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(estimator.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{1u});
    EXPECT_EQ(estimator.diagnostics().marginalization_stale_references, 0u);
}

TEST(MsckfMarginalizationIntegration, TrackIsRetiredWhenItsLastCloneReferenceLeavesWindow) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_feature_retirement_config());

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.01),
              EstimatorOperationResult::Accepted);
    const uint64_t first_id = Phase17ESKFEstimatorTestAccess::capture_msckf_camera_state(estimator);
    const Eigen::Vector3d feature{0.0, 0.0, 4.0};
    ASSERT_TRUE(Phase17ESKFEstimatorTestAccess::process_msckf_observations(
        estimator, first_id, {{320.0, 240.0}}, {feature}, camera_intrinsics()));

    for (int i = 2; i <= 4; ++i) {
        const double timestamp = 0.01 * static_cast<double>(i);
        ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                    Eigen::Vector3d::Zero(), timestamp),
                  EstimatorOperationResult::Accepted);
        const auto pose = estimator.state();
        estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);
    }

    EXPECT_FALSE(estimator.feature_track_id_for_feature_for_test(feature).has_value());
    EXPECT_EQ(estimator.diagnostics().marginalization_stale_references, 0u);
}

TEST(MsckfMarginalizationIntegration, ResetClearsRetirementWindowAndRestartsCloneIds) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_config());
    drive_visual_pose_clones(estimator, 5);

    ASSERT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{4u, 5u}));
    estimator.reset();

    EXPECT_TRUE(estimator.msckf_state_ids_for_test().empty());
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), kErrorDim);

    drive_visual_pose_clones(estimator, 1);
    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{1u}));
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), kErrorDim + 6);
}

TEST(MsckfMarginalizationIntegration, RetirementConsumesEligibleConstraintBeforeCloneRemoval) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_feature_update_retirement_config(false));

    const Eigen::Matrix3d K = camera_intrinsics();
    const Eigen::Vector3d feature{0.0, 0.0, 4.0};

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.01),
              EstimatorOperationResult::Accepted);
    auto pose = estimator.state();
    estimator.update_vision({project_feature(pose, feature, K)}, {feature}, K);
    ASSERT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{1u}));

    ErrorVec motion = ErrorVec::Zero();
    motion(0) = 0.20;
    ASSERT_EQ(estimator.inject_error_for_test(motion), EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.02),
              EstimatorOperationResult::Accepted);
    pose = estimator.state();
    estimator.update_vision({project_feature(pose, feature, K)}, {feature}, K);

    ASSERT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{1u, 2u}));
    ASSERT_TRUE(estimator.triangulated_landmark_for_feature_for_test(feature).has_value());
    ASSERT_EQ(estimator.feature_track_observation_count_for_feature_for_test(feature),
              std::optional<size_t>{2u});

    estimator.configure_msckf(make_feature_update_retirement_config(true));
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.03),
              EstimatorOperationResult::Accepted);
    pose = estimator.state();
    estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);

    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(diagnostics.marginalization_attempts, 1u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 1u);
    EXPECT_EQ(diagnostics.marginalization_failures, 0u);
    EXPECT_GE(diagnostics.marginalization_constraint_candidates, 1u);
    EXPECT_GE(diagnostics.marginalization_constraints_consumed, 1u);
    EXPECT_EQ(diagnostics.marginalization_constraint_failures, 0u);
    EXPECT_EQ(diagnostics.marginalization_stale_references, 0u);
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), 27);
}

TEST(MsckfMarginalizationIntegration, RetirementConstraintPassDoesNotDuplicateConsumedTrack) {
    Phase17ESKFEstimator estimator;
    estimator.reset();
    estimator.configure_msckf(make_feature_update_retirement_config(true));

    const Eigen::Matrix3d K = camera_intrinsics();
    const Eigen::Vector3d feature{0.0, 0.0, 4.0};

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.01),
              EstimatorOperationResult::Accepted);
    auto pose = estimator.state();
    estimator.update_vision({project_feature(pose, feature, K)}, {feature}, K);

    ErrorVec motion = ErrorVec::Zero();
    motion(0) = 0.20;
    ASSERT_EQ(estimator.inject_error_for_test(motion), EstimatorOperationResult::Accepted);
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.02),
              EstimatorOperationResult::Accepted);
    pose = estimator.state();
    estimator.update_vision({project_feature(pose, feature, K)}, {feature}, K);
    const uint64_t applied_after_second_observation =
        estimator.diagnostics().feature_updates_applied;

    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.03),
              EstimatorOperationResult::Accepted);
    pose = estimator.state();
    estimator.update_visual_pose(pose.position, pose.velocity, 0.35, 0.45);

    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(diagnostics.feature_updates_applied, applied_after_second_observation);
    EXPECT_EQ(diagnostics.marginalization_constraints_consumed, 0u);
    EXPECT_EQ(diagnostics.marginalization_constraint_failures, 0u);
    EXPECT_EQ(diagnostics.marginalization_stale_references, 0u);
}

} // namespace
} // namespace drone::vio
