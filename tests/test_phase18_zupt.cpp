#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

#include <cmath>

using namespace drone::vio;

namespace {

EstimatorValidationConfig make_phase18_cfg(bool shadow_enabled = false) {
    EstimatorValidationConfig cfg;
    cfg.enable_shadow_estimator = shadow_enabled;
    cfg.shadow_comparison_enabled = shadow_enabled;
    cfg.shadow_max_queue_depth = 128;
    cfg.shadow_max_lag_ms = 50.0;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    cfg.stationary_detector.enabled = true;
    cfg.stationary_detector.accel_threshold_mps2 = 0.12;
    cfg.stationary_detector.gyro_threshold_rads = 0.02;
    cfg.stationary_detector.window_size = 12;
    cfg.stationary_detector.enter_count = 10;
    cfg.stationary_detector.exit_count = 6;
    cfg.stationary_detector.minimum_stationary_time_s = 0.05;
    cfg.stationary_detector.accel_exit_threshold_mps2 = 0.22;
    cfg.stationary_detector.gyro_exit_threshold_rads = 0.04;
    cfg.zupt.enabled = true;
    cfg.zupt.velocity_noise_mps = 0.01;
    cfg.zupt.max_update_rate_hz = 20.0;
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
void feed_stationary_window(Phase17ESKFEstimator& estimator, double start_timestamp_s,
                            int steps = 48, double dt_s = 0.01,
                            const Eigen::Vector3d& accel = Eigen::Vector3d{0.0, 0.0, 9.81},
                            const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
    for (int i = 0; i < steps; ++i) {
        const double timestamp_s = start_timestamp_s + (static_cast<double>(i) * dt_s);
        ASSERT_EQ(estimator.process_imu_measurement(accel, gyro, timestamp_s),
                  EstimatorOperationResult::Accepted);
    }
}

} // namespace

class Phase18ZuptTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.configure_validation(make_phase18_cfg(false));
        ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    }

    Phase17ESKFEstimator ekf_;
};

TEST_F(Phase18ZuptTest, PerfectStationaryImuTriggersDetector) {
    feed_stationary_window(ekf_, 0.0);
    const auto diag = ekf_.diagnostics();
    EXPECT_TRUE(diag.stationary_detected);
    EXPECT_GT(diag.stationary_duration_s, 0.05);
    EXPECT_EQ(diag.detector_state_change_count, 1u);
}

TEST_F(Phase18ZuptTest, NoisyStationaryImuStillTriggersDetector) {
    double timestamp_s = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double noise = (i % 2 == 0) ? 0.04 : -0.03;
        const Eigen::Vector3d accel{0.0, 0.0, 9.81 + noise};
        const Eigen::Vector3d gyro{0.004, -0.003, 0.002};
        ASSERT_EQ(ekf_.process_imu_measurement(accel, gyro, timestamp_s),
                  EstimatorOperationResult::Accepted);
        timestamp_s += 0.01;
    }
    const auto diag = ekf_.diagnostics();
    EXPECT_TRUE(diag.stationary_detected);
    EXPECT_GE(diag.zupt_accepted_count, 1u);
}

TEST_F(Phase18ZuptTest, SlowMotionDoesNotTriggerStationaryDetector) {
    for (int i = 0; i < 60; ++i) {
        ASSERT_EQ(ekf_.process_imu_measurement(Eigen::Vector3d{2.0, 0.0, 9.81},
                                               Eigen::Vector3d::Zero(), i * 0.01),
                  EstimatorOperationResult::Accepted);
    }
    const auto diag = ekf_.diagnostics();
    EXPECT_FALSE(diag.stationary_detected);
    EXPECT_EQ(diag.zupt_accepted_count, 0u);
}

TEST_F(Phase18ZuptTest, ConstantRotationDoesNotTriggerStationaryDetector) {
    for (int i = 0; i < 60; ++i) {
        ASSERT_EQ(ekf_.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                               Eigen::Vector3d{0.0, 0.0, 0.08}, i * 0.01),
                  EstimatorOperationResult::Accepted);
    }
    const auto diag = ekf_.diagnostics();
    EXPECT_FALSE(diag.stationary_detected);
    EXPECT_EQ(diag.detector_state_change_count, 0u);
}

TEST_F(Phase18ZuptTest, HysteresisPreventsRapidOscillationUntilExitThresholdExceeded) {
    feed_stationary_window(ekf_, 0.0, 36);
    auto diag = ekf_.diagnostics();
    ASSERT_TRUE(diag.stationary_detected);

    for (int i = 0; i < 6; ++i) {
        ASSERT_EQ(ekf_.process_imu_measurement(Eigen::Vector3d{0.18, 0.0, 9.81},
                                               Eigen::Vector3d{0.0, 0.0, 0.03}, 0.36 + (i * 0.01)),
                  EstimatorOperationResult::Accepted);
    }
    diag = ekf_.diagnostics();
    EXPECT_TRUE(diag.stationary_detected);

    for (int i = 0; i < 12; ++i) {
        ASSERT_EQ(ekf_.process_imu_measurement(Eigen::Vector3d{0.45, 0.0, 9.81},
                                               Eigen::Vector3d{0.0, 0.0, 0.09}, 0.42 + (i * 0.01)),
                  EstimatorOperationResult::Accepted);
    }
    diag = ekf_.diagnostics();
    EXPECT_FALSE(diag.stationary_detected);
    EXPECT_EQ(diag.detector_state_change_count, 2u);
    ASSERT_EQ(diag.stationary_intervals.size(), 1u);
    EXPECT_GT(diag.stationary_intervals.front().duration_s, 0.05);
}

TEST_F(Phase18ZuptTest, AutomaticZuptDrivesVelocityTowardZero) {
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
               Eigen::Vector3d{0.6, -0.2, 0.1});
    feed_stationary_window(ekf_, 0.0, 80);
    const auto state = ekf_.state();
    const auto diag = ekf_.diagnostics();
    EXPECT_LT(state.velocity.norm(), 0.08);
    EXPECT_GE(diag.zupt_accepted_count, 1u);
}

TEST_F(Phase18ZuptTest, AutomaticZuptReducesVelocityCovariance) {
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
               Eigen::Vector3d{0.3, 0.0, 0.0});
    const double before_trace = ekf_.covariance().block<3, 3>(3, 3).trace();
    feed_stationary_window(ekf_, 0.0, 80);
    const double after_trace = ekf_.covariance().block<3, 3>(3, 3).trace();
    EXPECT_LT(after_trace, before_trace);
}

TEST_F(Phase18ZuptTest, InvalidPhase18ConfigurationFailsValidation) {
    EstimatorValidationConfig invalid = make_phase18_cfg(false);
    invalid.stationary_detector.window_size = 0;
    ekf_.configure_validation(invalid);
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(
        ekf_.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81}, Eigen::Vector3d::Zero(), 0.0),
        EstimatorOperationResult::RejectedInvalidConfiguration);
}

TEST_F(Phase18ZuptTest, LongStationaryIntervalRemainsFiniteWithRepeatedZupt) {
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
               Eigen::Vector3d{0.4, 0.2, -0.1});
    feed_stationary_window(ekf_, 0.0, 400, 0.01);
    const auto cov = ekf_.covariance();
    const auto diag = ekf_.diagnostics();
    EXPECT_TRUE(cov.array().isFinite().all());
    EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1.0e-8);
    EXPECT_GE(diag.zupt_accepted_count, 5u);
    EXPECT_TRUE(diag.stationary_detected);
}

TEST(Phase18Coordinator, ActiveEstimatorRemainsUnchangedWithShadowAutomaticZupt) {
    auto active_only = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_only", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    active_only.configure_validation(make_phase18_cfg(false));
    active_only.initialize();

    auto with_shadow = EstimatorCoordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active_shadowed", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));
    with_shadow.configure_validation(make_phase18_cfg(true));
    with_shadow.initialize();
    ASSERT_TRUE(with_shadow.start());

    for (int i = 0; i < 240; ++i) {
        const double timestamp = i * 0.01;
        ASSERT_EQ(active_only.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))),
                  with_shadow.process_measurement(imu_env(timestamp, static_cast<uint64_t>(i))));
    }
    with_shadow.flush_shadow();

    const auto a = active_only.active_snapshot();
    const auto b = with_shadow.active_snapshot();
    const auto shadow = with_shadow.shadow_snapshot();
    if (!shadow.has_value()) {
        GTEST_FAIL() << "Expected shadow snapshot";
    }
    const auto& shadow_snapshot = *shadow;
    EXPECT_NEAR((a.position_m - b.position_m).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((a.velocity_mps - b.velocity_mps).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(std::abs(a.orientation.dot(b.orientation)), 1.0, 1.0e-12);
    EXPECT_TRUE(shadow_snapshot.zupt_updates_applied >= 1u);
    EXPECT_TRUE(shadow_snapshot.stationary_detected);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
