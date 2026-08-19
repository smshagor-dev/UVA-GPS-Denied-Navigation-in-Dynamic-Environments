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

} // namespace
} // namespace drone::vio
