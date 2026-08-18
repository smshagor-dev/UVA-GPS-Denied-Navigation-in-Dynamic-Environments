#include "vio/Phase17ESKFEstimator.hpp"

#include <gtest/gtest.h>

namespace drone::vio {
namespace {

MsckfConfig make_config(bool diagnostics = true) {
    MsckfConfig cfg;
    cfg.enabled = true;
    cfg.max_camera_states = 2;
    cfg.diagnostics_enabled = diagnostics;
    cfg.triangulation.enabled = false;
    cfg.update.enabled = false;
    return cfg;
}

void drive_three_visual_pose_clones(Phase17ESKFEstimator& estimator) {
    ASSERT_EQ(estimator.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                                Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::Accepted);
    for (int i = 1; i <= 3; ++i) {
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
    drive_three_visual_pose_clones(estimator);

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
    drive_three_visual_pose_clones(estimator);

    EXPECT_EQ(estimator.msckf_state_ids_for_test(), (std::vector<uint64_t>{2u, 3u}));
    EXPECT_EQ(estimator.augmented_covariance_for_test().rows(), 27);
    const auto diagnostics = estimator.diagnostics();
    EXPECT_EQ(diagnostics.marginalization_attempts, 0u);
    EXPECT_EQ(diagnostics.marginalizations_completed, 0u);
}

} // namespace
} // namespace drone::vio
