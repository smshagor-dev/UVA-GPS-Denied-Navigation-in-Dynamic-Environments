#include <gtest/gtest.h>

#include "vio/Phase17ESKFEstimator.hpp"

#include <cmath>

using namespace drone::vio;

namespace {

EstimatorValidationConfig make_phase19_cfg() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.enable_fej = true;
    cfg.fej.enabled = true;
    cfg.fej.validation_checks = true;
    cfg.fej.diagnostics_enabled = true;
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

Eigen::Vector2d project_feature(const Phase17ESKFEstimator& ekf, const Eigen::Vector3d& feature) {
    const auto state = ekf.state();
    const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
    const Eigen::Vector3d p_c = R.transpose() * (feature - state.position);
    const auto K = camera_intrinsics();
    return {
        (K(0, 0) * p_c.x() / p_c.z()) + K(0, 2),
        (K(1, 1) * p_c.y() / p_c.z()) + K(1, 2),
    };
}

std::vector<Eigen::Vector2d> project_features(const Phase17ESKFEstimator& ekf,
                                              const std::vector<Eigen::Vector3d>& features) {
    std::vector<Eigen::Vector2d> pixels;
    pixels.reserve(features.size());
    for (const auto& feature : features) {
        pixels.push_back(project_feature(ekf, feature));
    }
    return pixels;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void propagate_motion(Phase17ESKFEstimator& ekf, int steps, double dt = 0.01) {
    double timestamp_s = 0.0;
    for (int i = 0; i < steps; ++i) {
        ASSERT_EQ(ekf.process_imu_measurement(Eigen::Vector3d{0.05, 0.0, 9.81},
                                              Eigen::Vector3d{0.0, 0.0, 0.01}, timestamp_s),
                  EstimatorOperationResult::Accepted);
        timestamp_s += dt;
    }
}

} // namespace

class Phase19FejTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.configure_validation(make_phase19_cfg());
        ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
        propagate_motion(ekf_, 5);
    }

    Phase17ESKFEstimator ekf_;
};

TEST_F(Phase19FejTest, SnapshotCreationIsDeterministic) {
    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    const auto ids = ekf_.fej_snapshot_ids_for_test();
    ASSERT_EQ(ids.size(), 2u);
    EXPECT_EQ(ids[0], 1u);
    EXPECT_EQ(ids[1], 2u);
    EXPECT_EQ(ekf_.diagnostics().fej_snapshots_created, 2u);
}

TEST_F(Phase19FejTest, SnapshotLifetimeReleasesDiscardedFeatures) {
    std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    features.pop_back();
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    EXPECT_EQ(ekf_.fej_snapshot_ids_for_test().size(), 1u);
    EXPECT_EQ(ekf_.diagnostics().fej_snapshots_released, 1u);
}

TEST(Phase19FejStandalone, SnapshotIdsRepeatAcrossEquivalentRuns) {
    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };

    auto run_once = [&features]() {
        Phase17ESKFEstimator ekf;
        ekf.configure_validation(make_phase19_cfg());
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        propagate_motion(ekf, 5);
        ekf.update_vision(project_features(ekf, features), features, camera_intrinsics());
        return ekf.fej_snapshot_ids_for_test();
    };

    EXPECT_EQ(run_once(), run_once());
}

TEST_F(Phase19FejTest, FejEnableDisableControlsJacobianEvaluation) {
    const std::vector<Eigen::Vector3d> features{{0.2, 0.1, 4.5}};
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    EXPECT_GE(ekf_.diagnostics().fej_jacobian_evaluations, 1u);

    Phase17ESKFEstimator disabled;
    EstimatorValidationConfig cfg = make_phase19_cfg();
    cfg.enable_fej = false;
    cfg.fej.enabled = false;
    disabled.configure_validation(cfg);
    disabled.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    propagate_motion(disabled, 5);
    disabled.update_vision(project_features(disabled, features), features, camera_intrinsics());
    EXPECT_EQ(disabled.diagnostics().fej_jacobian_evaluations, 0u);
    EXPECT_TRUE(disabled.fej_snapshot_ids_for_test().empty());
}

TEST_F(Phase19FejTest, InvalidSnapshotHandlingIncrementsValidationFailures) {
    const std::vector<Eigen::Vector3d> features{{0.2, 0.1, 4.5}};
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    ekf_.corrupt_first_fej_snapshot_for_test();
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    const auto diag = ekf_.diagnostics();
    EXPECT_GE(diag.fej_validation_failures, 1u);
    EXPECT_EQ(diag.last_operation_result, EstimatorOperationResult::FailedNumericalValidation);
}

TEST_F(Phase19FejTest, FiniteJacobiansPreserveFiniteCovariance) {
    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
        {0.05, -0.12, 4.9},
    };
    for (int i = 0; i < 12; ++i) {
        ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
        ASSERT_EQ(ekf_.process_imu_measurement(Eigen::Vector3d{0.02, 0.0, 9.81},
                                               Eigen::Vector3d{0.0, 0.0, 0.005}, 0.1 + (i * 0.01)),
                  EstimatorOperationResult::Accepted);
    }
    const auto cov = ekf_.covariance();
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
}

TEST_F(Phase19FejTest, InvalidFejConfigurationFailsValidation) {
    Phase17ESKFEstimator invalid;
    EstimatorValidationConfig cfg = make_phase19_cfg();
    cfg.enable_fej = true;
    cfg.fej.enabled = false;
    invalid.configure_validation(cfg);
    invalid.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(invalid.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                              Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::RejectedInvalidConfiguration);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
