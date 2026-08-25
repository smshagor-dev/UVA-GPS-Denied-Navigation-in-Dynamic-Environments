#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <cmath>
#include <numbers>

using namespace drone::vio;

namespace {

EstimatorValidationConfig make_cfg(bool shadow_enabled = false) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = 64;
    cfg.shadow_max_lag_ms = 50.0;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
MeasurementEnvelope imu_env(double timestamp_s, uint64_t sequence_id,
                            const Eigen::Vector3d& accel = Eigen::Vector3d{0.0, 0.0, 9.81},
                            const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
    MeasurementEnvelope envelope;
    envelope.type = MeasurementType::Imu;
    envelope.source_id = "imu";
    envelope.timestamp_s = timestamp_s;
    envelope.sequence_id = sequence_id;
    envelope.frame = MeasurementFrame::Body;
    envelope.payload = ImuMeasurementPayload{accel, gyro};
    return envelope;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
struct PropagationRun {
    int steps;
    double dt;
};

void propagate_constant(Phase17ESKFEstimator& ekf, const Eigen::Vector3d& accel,
                        const Eigen::Vector3d& gyro, PropagationRun run) {
    for (int i = 0; i < run.steps; ++i) {
        ekf.propagate_imu(accel, gyro, run.dt);
    }
}

} // namespace

class Phase17ESKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.configure_validation(make_cfg(false));
        ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    }

    Phase17ESKFEstimator ekf_;
};

TEST_F(Phase17ESKFTest, ZeroMotionPropagationStaysNearOrigin) {
    propagate_constant(ekf_, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(),
                       PropagationRun{400, 0.0025});
    const auto s = ekf_.state();
    EXPECT_NEAR(s.position.norm(), 0.0, 1.0e-4);
    EXPECT_NEAR(s.velocity.norm(), 0.0, 1.0e-4);
}

TEST_F(Phase17ESKFTest, ConstantAccelerationMatchesKinematics) {
    propagate_constant(ekf_, Eigen::Vector3d{1.0, 0.0, 9.81}, Eigen::Vector3d::Zero(),
                       PropagationRun{100, 0.01});
    const auto s = ekf_.state();
    EXPECT_NEAR(s.velocity.x(), 1.0, 0.05);
    EXPECT_NEAR(s.position.x(), 0.5, 0.05);
}

TEST_F(Phase17ESKFTest, ConstantAngularVelocityMatchesYaw) {
    propagate_constant(ekf_, Eigen::Vector3d{0.0, 0.0, 9.81},
                       Eigen::Vector3d{0.0, 0.0, std::numbers::pi_v<double> / 2.0},
                       PropagationRun{400, 0.0025});
    const auto euler = ekf_.state().euler_zyx_deg();
    EXPECT_NEAR(euler.x(), 90.0, 1.0);
}

TEST_F(Phase17ESKFTest, BiasOnlyInjectionChangesOnlyBiases) {
    const auto before = ekf_.state();
    ErrorVec dx = ErrorVec::Zero();
    dx.segment<3>(9) = Eigen::Vector3d{0.1, -0.2, 0.05};
    dx.segment<3>(12) = Eigen::Vector3d{-0.01, 0.02, -0.03};
    ASSERT_EQ(ekf_.inject_error_for_test(dx), EstimatorOperationResult::Accepted);
    const auto after = ekf_.state();
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.velocity - before.velocity).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.accel_bias - dx.segment<3>(9)).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.gyro_bias - dx.segment<3>(12)).norm(), 0.0, 1.0e-12);
}

TEST_F(Phase17ESKFTest, RepeatedErrorInjectionRemainsFiniteAndSymmetric) {
    for (int i = 0; i < 100; ++i) {
        ErrorVec dx = ErrorVec::Zero();
        dx.segment<3>(0) = Eigen::Vector3d{1.0e-4, -2.0e-4, 1.0e-4};
        dx.segment<3>(3) = Eigen::Vector3d{2.0e-4, 1.0e-4, -1.0e-4};
        dx.segment<3>(6) = Eigen::Vector3d{1.0e-4, -1.5e-4, 2.0e-4};
        ASSERT_EQ(ekf_.inject_error_for_test(dx), EstimatorOperationResult::Accepted);
    }
    const auto cov = ekf_.covariance();
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
}

TEST_F(Phase17ESKFTest, QuaternionSignEquivalentInitializationMatches) {
    Phase17ESKFEstimator a;
    Phase17ESKFEstimator b;
    a.configure_validation(make_cfg(false));
    b.configure_validation(make_cfg(false));
    const Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    a.reset(Eigen::Vector3d::Zero(), q, Eigen::Vector3d::Zero());
    b.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond{-q.w(), -q.x(), -q.y(), -q.z()},
            Eigen::Vector3d::Zero());

    propagate_constant(a, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.2},
                       PropagationRun{50, 0.01});
    propagate_constant(b, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.2},
                       PropagationRun{50, 0.01});

    const auto sa = a.state();
    const auto sb = b.state();
    EXPECT_NEAR((sa.position - sb.position).norm(), 0.0, 1.0e-10);
    EXPECT_NEAR((sa.velocity - sb.velocity).norm(), 0.0, 1.0e-10);
    EXPECT_NEAR(std::abs(sa.orientation.dot(sb.orientation)), 1.0, 1.0e-10);
}

TEST_F(Phase17ESKFTest, ResetJacobianMatchesSmallAngleForm) {
    const Eigen::Vector3d dtheta{0.01, -0.02, 0.03};
    const Eigen::Matrix3d expected =
        Eigen::Matrix3d::Identity() - 0.5 * (Eigen::Matrix3d() << 0.0, -dtheta.z(), dtheta.y(),
                                             dtheta.z(), 0.0, -dtheta.x(), -dtheta.y(), dtheta.x(),
                                             0.0)
                                                .finished();
    const Eigen::Matrix3d actual = Phase17ESKFEstimator::attitude_reset_jacobian(dtheta);
    EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1.0e-12);
}

TEST_F(Phase17ESKFTest, CovarianceSymmetryPreservedAcrossMixedOperations) {
    propagate_constant(ekf_, Eigen::Vector3d{0.2, -0.1, 9.81}, Eigen::Vector3d{0.01, 0.02, -0.03},
                       PropagationRun{200, 0.005});
    ekf_.update_zupt();
    ekf_.update_visual_pose(Eigen::Vector3d{0.3, -0.1, 0.2}, Eigen::Vector3d{0.0, 0.0, 0.0});
    const auto cov = ekf_.covariance();
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
}

TEST_F(Phase17ESKFTest, LongDurationPropagationRemainsFinite) {
    propagate_constant(ekf_, Eigen::Vector3d{0.05, 0.0, 9.81}, Eigen::Vector3d{0.0, 0.0, 0.01},
                       PropagationRun{8000, 0.0025});
    const auto s = ekf_.state();
    const auto cov = ekf_.covariance();
    EXPECT_TRUE(s.position.array().isFinite().all());
    EXPECT_TRUE(s.velocity.array().isFinite().all());
    EXPECT_TRUE(std::isfinite(s.orientation.norm()));
    EXPECT_TRUE(cov.array().isFinite().all());
}

TEST_F(Phase17ESKFTest, InvalidInputIsNoOp) {
    propagate_constant(ekf_, Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(),
                       PropagationRun{5, 0.01});
    const auto before = ekf_.state();
    ASSERT_EQ(ekf_.process_imu_measurement(
                  Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.81},
                  Eigen::Vector3d::Zero(), 0.2),
              EstimatorOperationResult::RejectedNonFiniteInput);
    const auto after = ekf_.state();
    EXPECT_NEAR((after.position - before.position).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.velocity - before.velocity).norm(), 0.0, 1.0e-12);
}

TEST(Phase17Coordinator, ActiveEstimatorUnchangedWhenPhase17ShadowEnabled) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_only", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    active_only.configure_validation(make_cfg(false));
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_shadowed", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_cfg(true));
    with_shadow.initialize();
    ASSERT_TRUE(with_shadow.start());

    for (int i = 0; i < 800; ++i) {
        const double timestamp = i * 0.0025;
        ASSERT_EQ(active_only.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))),
                  with_shadow.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))));
    }
    with_shadow.flush_shadow();

    const auto a = active_only.active_snapshot();
    const auto b = with_shadow.active_snapshot();
    EXPECT_NEAR((a.position_m - b.position_m).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((a.velocity_mps - b.velocity_mps).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(std::abs(a.orientation.dot(b.orientation)), 1.0, 1.0e-12);

    const auto diagnostics = with_shadow.diagnostics();
    EXPECT_TRUE(diagnostics.shadow_enabled);
    EXPECT_EQ(diagnostics.active_estimator_name, "ekf_active_shadowed");
    EXPECT_EQ(diagnostics.shadow_estimator_name, "eskf_shadow");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
