// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include "autonomy/ExperienceMemory.hpp"
#include "localization/TDOALocalizer.hpp"

using namespace drone;

namespace {

sensors::CameraFrame make_frame(std::string label, float confidence) {
    sensors::CameraFrame frame;
    sensors::Detection detection;
    detection.label = std::move(label);
    detection.confidence = confidence;
    detection.bbox = cv::Rect2f{0.35f, 0.35f, 0.20f, 0.20f};
    frame.detections.push_back(std::move(detection));
    return frame;
}

} // namespace

TEST(ExperienceMemory, SummarizesRiskFromHistory) {
    autonomy::ExperienceMemory memory;
    vio::PoseEstimate pose;
    hal::SystemStats stats;
    stats.battery_pct = 92.0f;
    pose.pos_std = Eigen::Vector3d(0.02, 0.02, 0.02);

    memory.observe(7, pose, stats, make_frame("tree", 0.9f), 3, 0.0);

    pose.pos_std = Eigen::Vector3d(0.30, 0.25, 0.20);
    stats.battery_pct = 78.0f;
    memory.observe(7, pose, stats, make_frame("tree", 0.92f), 4, 60.0);

    const auto prior = memory.summarize(7);
    EXPECT_TRUE(prior.recommend_caution);
    EXPECT_GT(prior.risk_score, 0.6);
    EXPECT_EQ(prior.dominant_label, "tree");
}

TEST(TdoaLocalizer, SolvesKnownPosition) {
    localization::TDOALocalizer localizer;
    localizer.set_anchors({
        {1, Eigen::Vector3d{-10.0, -10.0, 0.0}},
        {2, Eigen::Vector3d{ 10.0, -10.0, 0.0}},
        {3, Eigen::Vector3d{ 10.0,  10.0, 0.0}},
        {4, Eigen::Vector3d{-10.0,  10.0, 0.0}},
        {5, Eigen::Vector3d{  0.0,   0.0, 15.0}},
    });

    const Eigen::Vector3d target{2.0, -3.0, 4.5};
    constexpr double kSignalSpeedMps = 299702547.0;
    constexpr double kTxBiasS = 4.2e-7;

    std::vector<localization::TDOALocalizer::Measurement> measurements;
    for (const auto& anchor : localizer.anchors()) {
        measurements.push_back({
            anchor.id,
            kTxBiasS + ((target - anchor.position).norm() / kSignalSpeedMps)
        });
    }

    const auto solution = localizer.estimate(measurements, Eigen::Vector3d::Zero());
    ASSERT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->converged);
    EXPECT_LT((solution->position - target).norm(), 0.2);
    EXPECT_GT(solution->confidence, 0.8);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
