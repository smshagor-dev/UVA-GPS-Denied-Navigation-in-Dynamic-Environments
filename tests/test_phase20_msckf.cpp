#include <gtest/gtest.h>

#include "vio/EKFStateEstimatorAdapter.hpp"
#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"

using namespace drone::vio;

namespace {

EstimatorValidationConfig make_phase20_cfg() {
    EstimatorValidationConfig cfg;
    cfg.max_imu_dt_s = 0.1;
    cfg.min_imu_dt_s = 1.0e-6;
    return cfg;
}

MsckfConfig make_phase20_msckf_cfg(bool enabled = true, uint32_t max_camera_states = 3,
                                   bool diagnostics_enabled = true) {
    MsckfConfig cfg;
    cfg.enabled = enabled;
    cfg.max_camera_states = max_camera_states;
    cfg.eviction_policy = "oldest_first";
    cfg.diagnostics_enabled = diagnostics_enabled;
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

void propagate_motion(Phase17ESKFEstimator& ekf, double& timestamp_s, int step_count) {
    for (int i = 0; i < step_count; ++i) {
        ASSERT_EQ(ekf.process_imu_measurement(Eigen::Vector3d{0.04, 0.0, 9.81},
                                              Eigen::Vector3d{0.0, 0.0, 0.01}, timestamp_s),
                  EstimatorOperationResult::Accepted);
        timestamp_s += 0.01;
    }
}

void capture_camera_state(Phase17ESKFEstimator& ekf, double& timestamp_s) {
    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };
    propagate_motion(ekf, timestamp_s, 4);
    ekf.update_vision(project_features(ekf, features), features, camera_intrinsics());
}

} // namespace

class Phase20MsckfTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_.configure_validation(make_phase20_cfg());
        ekf_.configure_msckf(make_phase20_msckf_cfg());
        ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero());
    }

    Phase17ESKFEstimator ekf_;
    double timestamp_s_{0.0};
};

TEST_F(Phase20MsckfTest, InsertionOrderFollowsCaptureSequence) {
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);

    const auto ids = ekf_.msckf_state_ids_for_test();
    ASSERT_EQ(ids.size(), 3u);
    EXPECT_EQ(ids[0], 1u);
    EXPECT_EQ(ids[1], 2u);
    EXPECT_EQ(ids[2], 3u);
}

TEST_F(Phase20MsckfTest, DeterministicIdsRepeatAcrossEquivalentRuns) {
    auto run_once = []() {
        Phase17ESKFEstimator ekf;
        double timestamp_s = 0.0;
        ekf.configure_validation(make_phase20_cfg());
        ekf.configure_msckf(make_phase20_msckf_cfg());
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        capture_camera_state(ekf, timestamp_s);
        capture_camera_state(ekf, timestamp_s);
        capture_camera_state(ekf, timestamp_s);
        return ekf.msckf_state_ids_for_test();
    };

    EXPECT_EQ(run_once(), run_once());
}

TEST_F(Phase20MsckfTest, DeterministicEvictionRemovesOldestState) {
    ekf_.configure_msckf(make_phase20_msckf_cfg(true, 2, true));
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);

    const auto ids = ekf_.msckf_state_ids_for_test();
    ASSERT_EQ(ids.size(), 2u);
    EXPECT_EQ(ids[0], 3u);
    EXPECT_EQ(ids[1], 4u);
    EXPECT_EQ(ekf_.diagnostics().msckf_states_removed, 2u);
    EXPECT_EQ(ekf_.diagnostics().msckf_deterministic_evictions, 2u);
}

TEST_F(Phase20MsckfTest, LookupByIdAndTimestampIsStable) {
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);

    const auto ids = ekf_.msckf_state_ids_for_test();
    const auto timestamps = ekf_.msckf_state_timestamps_for_test();
    ASSERT_EQ(ids.size(), 2u);
    ASSERT_EQ(timestamps.size(), 2u);

    const auto timestamp = ekf_.msckf_state_timestamp_for_id_for_test(ids[1]);
    ASSERT_TRUE(timestamp.has_value());
    EXPECT_DOUBLE_EQ(timestamp.value_or(0.0), timestamps[1]);

    const auto state_id = ekf_.msckf_state_id_for_timestamp_for_test(timestamps[0]);
    ASSERT_TRUE(state_id.has_value());
    EXPECT_EQ(state_id.value_or(0u), ids[0]);
}

TEST_F(Phase20MsckfTest, ResetClearsWindowAndRestartsIds) {
    capture_camera_state(ekf_, timestamp_s_);
    capture_camera_state(ekf_, timestamp_s_);
    ASSERT_EQ(ekf_.msckf_state_ids_for_test().size(), 2u);

    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_TRUE(ekf_.msckf_state_ids_for_test().empty());

    timestamp_s_ = 0.0;
    capture_camera_state(ekf_, timestamp_s_);
    const auto ids = ekf_.msckf_state_ids_for_test();
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_EQ(ids[0], 1u);
}

TEST_F(Phase20MsckfTest, DuplicateTimestampHandlingDoesNotCreateDuplicateState) {
    const std::vector<Eigen::Vector3d> features{
        {0.2, 0.1, 4.5},
        {-0.1, 0.15, 5.2},
    };

    propagate_motion(ekf_, timestamp_s_, 4);
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());
    ekf_.update_vision(project_features(ekf_, features), features, camera_intrinsics());

    const auto ids = ekf_.msckf_state_ids_for_test();
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_EQ(ekf_.diagnostics().msckf_states_created, 1u);
}

TEST_F(Phase20MsckfTest, MaximumWindowSizeIsPreserved) {
    ekf_.configure_msckf(make_phase20_msckf_cfg(true, 3, true));
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    for (int i = 0; i < 7; ++i) {
        capture_camera_state(ekf_, timestamp_s_);
    }

    EXPECT_EQ(ekf_.msckf_state_ids_for_test().size(), 3u);
    EXPECT_EQ(ekf_.diagnostics().msckf_window_size, 3u);
}

TEST_F(Phase20MsckfTest, RepeatedReplayConsistencyPreservesWindowContents) {
    auto run_once = []() {
        Phase17ESKFEstimator ekf;
        double timestamp_s = 0.0;
        ekf.configure_validation(make_phase20_cfg());
        ekf.configure_msckf(make_phase20_msckf_cfg());
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        for (int i = 0; i < 5; ++i) {
            capture_camera_state(ekf, timestamp_s);
        }
        return std::make_pair(ekf.msckf_state_ids_for_test(),
                              ekf.msckf_state_timestamps_for_test());
    };

    EXPECT_EQ(run_once(), run_once());
}

TEST_F(Phase20MsckfTest, InvalidConfigurationFailsValidation) {
    Phase17ESKFEstimator invalid;
    invalid.configure_validation(make_phase20_cfg());
    invalid.configure_msckf(make_phase20_msckf_cfg(true, 0, true));
    invalid.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(invalid.process_imu_measurement(Eigen::Vector3d{0.0, 0.0, 9.81},
                                              Eigen::Vector3d::Zero(), 0.0),
              EstimatorOperationResult::RejectedInvalidConfiguration);
}

TEST_F(Phase20MsckfTest, DiagnosticsEnabledCountersUpdate) {
    for (int i = 0; i < 4; ++i) {
        capture_camera_state(ekf_, timestamp_s_);
    }

    const auto diag = ekf_.diagnostics();
    EXPECT_EQ(diag.msckf_window_size, 3u);
    EXPECT_EQ(diag.msckf_states_created, 4u);
    EXPECT_EQ(diag.msckf_states_removed, 1u);
    EXPECT_EQ(diag.msckf_deterministic_evictions, 1u);
    EXPECT_GE(diag.msckf_oldest_state_age_s, 0.0);
}

TEST_F(Phase20MsckfTest, DiagnosticsDisabledKeepsWindowFunctionalAndCountersNeutral) {
    ekf_.configure_msckf(make_phase20_msckf_cfg(true, 2, false));
    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    for (int i = 0; i < 4; ++i) {
        capture_camera_state(ekf_, timestamp_s_);
    }

    const auto ids = ekf_.msckf_state_ids_for_test();
    ASSERT_EQ(ids.size(), 2u);
    EXPECT_EQ(ids[0], 3u);
    EXPECT_EQ(ids[1], 4u);

    const auto diag = ekf_.diagnostics();
    EXPECT_EQ(diag.msckf_window_size, 0u);
    EXPECT_EQ(diag.msckf_states_created, 0u);
    EXPECT_EQ(diag.msckf_states_removed, 0u);
    EXPECT_EQ(diag.msckf_deterministic_evictions, 0u);
    EXPECT_DOUBLE_EQ(diag.msckf_oldest_state_age_s, 0.0);
}

TEST_F(Phase20MsckfTest, ResetWithDiagnosticsDisabledPreservesNeutralPublication) {
    ekf_.configure_msckf(make_phase20_msckf_cfg(true, 3, false));
    for (int i = 0; i < 3; ++i) {
        capture_camera_state(ekf_, timestamp_s_);
    }

    ekf_.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    EXPECT_TRUE(ekf_.msckf_state_ids_for_test().empty());
    const auto diag = ekf_.diagnostics();
    EXPECT_EQ(diag.msckf_window_size, 0u);
    EXPECT_EQ(diag.msckf_states_created, 0u);
    EXPECT_EQ(diag.msckf_states_removed, 0u);
    EXPECT_EQ(diag.msckf_deterministic_evictions, 0u);
    EXPECT_DOUBLE_EQ(diag.msckf_oldest_state_age_s, 0.0);
}

TEST_F(Phase20MsckfTest, DiagnosticsDisabledReplayRemainsDeterministic) {
    auto run_once = []() {
        Phase17ESKFEstimator ekf;
        double timestamp_s = 0.0;
        ekf.configure_validation(make_phase20_cfg());
        ekf.configure_msckf(make_phase20_msckf_cfg(true, 3, false));
        ekf.reset(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        for (int i = 0; i < 5; ++i) {
            capture_camera_state(ekf, timestamp_s);
        }
        return std::make_tuple(
            ekf.msckf_state_ids_for_test(), ekf.msckf_state_timestamps_for_test(),
            ekf.diagnostics().msckf_states_created, ekf.diagnostics().msckf_window_size);
    };

    EXPECT_EQ(run_once(), run_once());
}

TEST(Phase20MsckfIsolationTest, ActiveEstimatorDoesNotDependOnMsckfConfig) {
    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "phase16"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "phase17"));

    auto validation_cfg = make_phase20_cfg();
    validation_cfg.enable_shadow_estimator = true;
    validation_cfg.shadow_comparison_enabled = true;
    coordinator.configure_validation(validation_cfg);
    coordinator.configure_shadow_msckf(make_phase20_msckf_cfg(true, 2, false));
    coordinator.initialize();
    (void)coordinator.start();

    for (int i = 0; i < 80; ++i) {
        MeasurementEnvelope env;
        env.type = MeasurementType::Imu;
        env.source_id = "imu";
        env.timestamp_s = i * 0.01;
        env.sequence_id = static_cast<uint64_t>(i);
        env.frame = MeasurementFrame::Body;
        env.payload = ImuMeasurementPayload{Eigen::Vector3d{0.04, 0.0, 9.81},
                                            Eigen::Vector3d{0.0, 0.0, 0.01}};
        ASSERT_EQ(coordinator.process_measurement(env), EstimatorOperationResult::Accepted);
    }

    coordinator.flush_shadow();
    const auto active = coordinator.active_snapshot();
    const auto shadow = coordinator.shadow_snapshot();
    coordinator.stop();

    ASSERT_TRUE(shadow.has_value());
    if (!shadow.has_value()) {
        return;
    }
    EXPECT_EQ(active.estimator_name, "ekf_active");
    EXPECT_EQ(active.msckf_window_size, 0u);
    EXPECT_EQ(active.msckf_states_created, 0u);
    EXPECT_EQ(active.msckf_states_removed, 0u);
    EXPECT_EQ(active.msckf_deterministic_evictions, 0u);
    EXPECT_TRUE(active.initialized);
    ASSERT_TRUE(shadow->initialized);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
