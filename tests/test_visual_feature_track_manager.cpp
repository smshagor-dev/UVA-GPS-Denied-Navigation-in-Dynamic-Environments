#include <gtest/gtest.h>

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

TEST(VisualFeatureTrackManager, StableTrackIdAndWorldPointAcrossFrames) {
    VisualFeatureTrackConfig cfg;
    cfg.minimum_baseline_m = 0.05;
    cfg.minimum_parallax_deg = 0.1;
    cfg.maximum_ray_gap_m = 0.5;
    VisualFeatureTrackManager manager(cfg);

    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{312.0, 240.0}};
    const auto first = manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());
    ASSERT_EQ(first.features.size(), 1u);
    const auto track_id = first.features.front().track_id;
    const auto world_point = first.features.front().world_point;
    EXPECT_EQ(track_id, 1u);
    EXPECT_TRUE(world_point.array().isFinite().all());

    const std::vector<Eigen::Vector2d> next{{304.0, 240.0}};
    const auto second = manager.update(current, next, pose_at(0.10), pose_at(0.20), intrinsics());
    ASSERT_EQ(second.features.size(), 1u);
    EXPECT_EQ(second.features.front().track_id, track_id);
    EXPECT_NEAR((second.features.front().world_point - world_point).norm(), 0.0, 1.0e-12);
    EXPECT_EQ(second.carried_tracks, 1u);
}

TEST(VisualFeatureTrackManager, LowParallaxDoesNotPublishUninitializedTrack) {
    VisualFeatureTrackConfig cfg;
    cfg.minimum_baseline_m = 0.05;
    cfg.minimum_parallax_deg = 2.0;
    VisualFeatureTrackManager manager(cfg);

    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{319.9, 240.0}};
    const auto batch = manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());

    EXPECT_TRUE(batch.features.empty());
    EXPECT_EQ(batch.initialized_tracks, 0u);
    EXPECT_EQ(batch.rejected_geometry, 1u);
    EXPECT_EQ(manager.track_count(), 1u);
}

TEST(VisualFeatureTrackManager, InvalidInputFailsClosedAndClearsTransientTracks) {
    VisualFeatureTrackManager manager;
    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{312.0, 240.0}};
    (void)manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());
    ASSERT_EQ(manager.track_count(), 1u);

    const std::vector<Eigen::Vector2d> mismatched{{312.0, 240.0}, {300.0, 220.0}};
    const auto rejected =
        manager.update(previous, mismatched, pose_at(0.10), pose_at(0.20), intrinsics());
    EXPECT_TRUE(rejected.features.empty());
    EXPECT_EQ(manager.track_count(), 0u);
}

TEST(VisualFeatureTrackManager, ResetRestartsIdsDeterministically) {
    VisualFeatureTrackManager manager;
    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{312.0, 240.0}};
    (void)manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());
    EXPECT_GT(manager.next_track_id(), 1u);

    manager.reset();
    EXPECT_EQ(manager.track_count(), 0u);
    EXPECT_EQ(manager.next_track_id(), 1u);
}

TEST(VisualFeatureTrackManager, DeterministicTieBreakUsesLowestTrackId) {
    VisualFeatureTrackConfig cfg;
    cfg.association_radius_px = 5.0;
    cfg.minimum_parallax_deg = 0.1;
    VisualFeatureTrackManager manager(cfg);

    const std::vector<Eigen::Vector2d> previous{{320.0, 240.0}, {324.0, 240.0}};
    const std::vector<Eigen::Vector2d> current{{312.0, 240.0}, {316.0, 240.0}};
    const auto first = manager.update(previous, current, pose_at(0.0), pose_at(0.10), intrinsics());
    ASSERT_EQ(first.features.size(), 2u);
    EXPECT_LT(first.features[0].track_id, first.features[1].track_id);

    const std::vector<Eigen::Vector2d> ambiguous_previous{{314.0, 240.0}};
    const std::vector<Eigen::Vector2d> next{{306.0, 240.0}};
    const auto second =
        manager.update(ambiguous_previous, next, pose_at(0.10), pose_at(0.20), intrinsics());
    ASSERT_EQ(second.features.size(), 1u);
    EXPECT_EQ(second.features.front().track_id, first.features.front().track_id);
}

} // namespace
} // namespace drone::vio
