// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include <filesystem>

#include <opencv2/imgproc.hpp>

#include "slam/KeyframeManager.hpp"

using namespace drone::slam;

namespace {

cv::Mat make_feature_rich_image(int seed = 0) {
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::RNG rng(seed + 1234);
    for (int i = 0; i < 80; ++i) {
        const cv::Point center(rng.uniform(20, 620), rng.uniform(20, 460));
        cv::circle(image, center, rng.uniform(4, 14), cv::Scalar(rng.uniform(120, 255)), -1);
        cv::line(image, cv::Point(rng.uniform(0, 639), rng.uniform(0, 479)),
                 cv::Point(rng.uniform(0, 639), rng.uniform(0, 479)),
                 cv::Scalar(rng.uniform(80, 220)), rng.uniform(1, 3));
    }
    return image;
}

} // namespace

TEST(KeyframeManager, CreatesKeyframeAndMapPoints) {
    KeyframeManager manager(1, nullptr);
    const auto maybe_id = manager.try_add_frame(make_feature_rich_image(), Eigen::Vector3d::Zero(),
                                                Eigen::Quaterniond::Identity(), 1.0);

    ASSERT_TRUE(maybe_id.has_value());
    EXPECT_EQ(manager.keyframe_count(), 1u);
    EXPECT_GT(manager.map_point_count(), 0u);
}

TEST(KeyframeManager, EnforcesSelectionPolicyOnNearlyIdenticalFrames) {
    KeyframeSelectionPolicy policy;
    policy.min_time_s = 0.5;
    policy.min_translation_m = 0.4f;
    policy.min_rotation_deg = 10.0f;

    KeyframeManager manager(1, nullptr, policy);
    ASSERT_TRUE(manager
                    .try_add_frame(make_feature_rich_image(1), Eigen::Vector3d::Zero(),
                                   Eigen::Quaterniond::Identity(), 1.0)
                    .has_value());

    const auto second =
        manager.try_add_frame(make_feature_rich_image(1), Eigen::Vector3d(0.05, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 1.2);

    EXPECT_FALSE(second.has_value());
    EXPECT_EQ(manager.keyframe_count(), 1u);
}

TEST(KeyframeManager, FindsLoopClosureCandidatesFromRepeatedView) {
    KeyframeSelectionPolicy policy;
    policy.min_time_s = 0.1;
    policy.min_translation_m = 0.1f;
    policy.min_rotation_deg = 2.0f;

    KeyframeManager manager(1, nullptr, policy);
    const cv::Mat repeated = make_feature_rich_image(7);

    ASSERT_TRUE(
        manager
            .try_add_frame(repeated, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 1.0)
            .has_value());

    ASSERT_TRUE(manager
                    .try_add_frame(repeated, Eigen::Vector3d(1.0, 0.0, 0.0),
                                   Eigen::Quaterniond::Identity(), 2.0)
                    .has_value());

    auto keyframes = manager.get_recent_keyframes(1);
    ASSERT_EQ(keyframes.size(), 1u);
    const auto candidates = manager.find_loop_closure_candidates(keyframes.front().descriptors, 3);

    EXPECT_FALSE(candidates.empty());
}

TEST(KeyframeManager, SavesAndLoadsMapState) {
    KeyframeSelectionPolicy policy;
    policy.min_time_s = 0.1;
    policy.min_translation_m = 0.1f;

    KeyframeManager manager(1, nullptr, policy);
    ASSERT_TRUE(manager
                    .try_add_frame(make_feature_rich_image(3), Eigen::Vector3d::Zero(),
                                   Eigen::Quaterniond::Identity(), 1.0)
                    .has_value());
    ASSERT_TRUE(manager
                    .try_add_frame(make_feature_rich_image(4), Eigen::Vector3d(0.8, 0.0, 0.0),
                                   Eigen::Quaterniond::Identity(), 2.0)
                    .has_value());

    const auto path = std::filesystem::temp_directory_path() / "drone_swarm_slam_map.bin";
    ASSERT_TRUE(manager.save_map(path.string()));

    KeyframeManager restored(1, nullptr, policy);
    ASSERT_TRUE(restored.load_map(path.string()));
    std::filesystem::remove(path);

    EXPECT_EQ(restored.keyframe_count(), manager.keyframe_count());
    EXPECT_EQ(restored.map_point_count(), manager.map_point_count());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
