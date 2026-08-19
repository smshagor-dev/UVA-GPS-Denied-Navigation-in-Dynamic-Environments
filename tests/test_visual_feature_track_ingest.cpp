#include <gtest/gtest.h>

#include "vio/EstimatorCoordinator.hpp"
#include "vio/Phase17ESKFEstimator.hpp"
#include "vio/VisualFeatureTrackManager.hpp"

namespace drone::vio {
namespace {

Eigen::Matrix3d intrinsics() {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0, 0) = 320.0;
    K(1, 1) = 320.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    K(2, 2) = 1.0;
    return K;
}

PoseEstimate pose_at(double x) {
    PoseEstimate pose;
    pose.position = Eigen::Vector3d{x, 0.0, 0.0};
    pose.orientation = Eigen::Quaterniond::Identity();
    return pose;
}

VisualFeatureTrackBatch make_batch() {
    VisualFeatureTrackConfig cfg;
    cfg.minimum_baseline_m = 0.05;
    cfg.minimum_parallax_deg = 0.1;
    cfg.maximum_ray_gap_m = 0.5;
    VisualFeatureTrackManager manager(cfg);
    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}, {340.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{312.0, 240.0}, {332.0, 240.0}};
    return manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());
}

TEST(VisualFeatureTrackIngest, ShadowOnlyFeatureSubmissionPreservesActiveState) {
    ShadowCoordinatorConfig shadow_cfg;
    shadow_cfg.enabled = true;
    shadow_cfg.start_worker_on_initialize = true;

    EstimatorCoordinator coordinator(
        std::make_unique<EKFStateEstimatorAdapter>(EKFConfig{}, "ekf_active", "baseline"),
        std::make_unique<Phase17StateEstimatorAdapter>(EKFConfig{}, "eskf_shadow", "hardened"),
        shadow_cfg);

    coordinator.initialize();
    ASSERT_TRUE(coordinator.start());

    const auto before = coordinator.active_snapshot();
    const auto batch = make_batch();
    ASSERT_FALSE(batch.features.empty());

    VisualFeatureMeasurementPayload payload;
    payload.K = intrinsics();
    for (const auto& feature : batch.features) {
        payload.z_pixels.push_back(feature.pixel);
        payload.p_world.push_back(feature.world_point);
    }

    EXPECT_EQ(coordinator.submit_shadow_measurement(
                  make_visual_features_envelope(payload, MeasurementStamp{0.10, 1})),
              EstimatorOperationResult::Accepted);
    coordinator.flush_shadow();

    const auto after = coordinator.active_snapshot();
    EXPECT_NEAR((after.position_m - before.position_m).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((after.velocity_mps - before.velocity_mps).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR(std::abs(after.orientation.dot(before.orientation)), 1.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(after.covariance.trace, before.covariance.trace);

    const auto diagnostics = coordinator.diagnostics();
    EXPECT_EQ(diagnostics.active_processed_count, 0u);
    EXPECT_EQ(diagnostics.shadow_only_submission_count, 1u);
    EXPECT_EQ(diagnostics.queue.current_depth, 0u);
    EXPECT_GE(diagnostics.shadow_processed_count, 1u);
    coordinator.stop();
}

TEST(VisualFeatureTrackIngest, RepeatedSyntheticTracksRemainDeterministic) {
    VisualFeatureTrackConfig cfg;
    cfg.minimum_baseline_m = 0.05;
    cfg.minimum_parallax_deg = 0.1;
    cfg.maximum_ray_gap_m = 0.5;

    auto run = [&cfg]() {
        VisualFeatureTrackManager manager(cfg);
        std::vector<uint64_t> ids;
        const std::vector<Eigen::Vector2d> p0{{320.0, 240.0}, {340.0, 240.0}};
        const std::vector<Eigen::Vector2d> p1{{312.0, 240.0}, {332.0, 240.0}};
        const std::vector<Eigen::Vector2d> p2{{304.0, 240.0}, {324.0, 240.0}};

        const auto first = manager.update(p0, p1, pose_at(0.0), pose_at(0.10), intrinsics());
        const auto second = manager.update(p1, p2, pose_at(0.10), pose_at(0.20), intrinsics());
        for (const auto& feature : first.features) ids.push_back(feature.track_id);
        for (const auto& feature : second.features) ids.push_back(feature.track_id);
        return ids;
    };

    EXPECT_EQ(run(), run());
}

} // namespace
} // namespace drone::vio
