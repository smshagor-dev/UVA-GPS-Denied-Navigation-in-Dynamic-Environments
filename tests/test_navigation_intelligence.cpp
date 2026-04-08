// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <opencv2/imgproc.hpp>

#include "autonomy/ExperienceMemory.hpp"
#include "localization/LocalizationFusion.hpp"
#include "localization/TDOAIngestor.hpp"
#include "localization/TimeSyncTracker.hpp"
#include "localization/TDOALocalizer.hpp"
#include "slam/KeyframeManager.hpp"
#include "slam/MapPlanner.hpp"
#include "slam/OccupancyGridMap.hpp"

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

    memory.observe(7, pose, stats, make_frame("tree", 0.9f), 3, 0.92, "vision-inertial", false, 0.0);

    pose.pos_std = Eigen::Vector3d(0.30, 0.25, 0.20);
    stats.battery_pct = 78.0f;
    memory.observe(7, pose, stats, make_frame("tree", 0.92f), 4, 0.35, "imu-dead-reckoning", true, 60.0);

    const auto prior = memory.summarize(7);
    EXPECT_TRUE(prior.recommend_caution);
    EXPECT_GT(prior.risk_score, 0.6);
    EXPECT_EQ(prior.dominant_label, "tree");
    EXPECT_LT(prior.localization_confidence_avg, 0.8);
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

TEST(TimeSyncTracker, ReportsDegradedWhenOffsetsGrow) {
    localization::TimeSyncTracker tracker;
    tracker.observe_imu(10.000);
    tracker.observe_camera(10.014);
    tracker.observe_anchor(1, 10.030, 10.000);
    tracker.observe_peer(3, 9.970, 10.005);

    const auto status = tracker.status();
    EXPECT_LT(status.confidence, 1.0);
    EXPECT_FALSE(status.synchronized);
}

TEST(LocalizationFusion, PrefersTdoaWhenVioDriftIsHigh) {
    localization::LocalizationFusion fusion;
    localization::LocalizationFusionInput input;
    input.vio_pose.position = Eigen::Vector3d(8.0, 0.0, 8.0);
    input.vio_pose.drift_m = 2.4;
    input.vio_pose.localization_confidence = 0.36;
    input.camera_available = false;
    input.anchor_visibility_ratio = 0.8;
    input.time_sync.confidence = 0.9;
    input.time_sync.synchronized = true;

    localization::TDOALocalizer::Solution tdoa_solution;
    tdoa_solution.position = Eigen::Vector3d(2.0, 0.0, 8.0);
    tdoa_solution.confidence = 0.88;
    input.tdoa_solution = tdoa_solution;

    const auto output = fusion.update(input);
    EXPECT_GT(output.tdoa_weight, 0.35);
    EXPECT_EQ(output.source, "vio-tdoa-fused");
    EXPECT_GT(output.confidence, 0.5);
}

TEST(OccupancyGridMap, TracksAnchorAndObstacleCoverage) {
    slam::OccupancyGridMap map;
    sensors::LidarMeasurement scan;
    scan.cloud.reset(new sensors::PointCloud());
    scan.cloud->push_back(pcl::PointXYZI{1.0f, 0.5f, 0.0f, 1.0f});
    scan.cloud->push_back(pcl::PointXYZI{-0.5f, 1.0f, 0.0f, 1.0f});
    map.integrate_lidar(scan, Eigen::Vector3d::Zero());
    map.mark_anchor({1, Eigen::Vector3d{2.0, 0.0, 0.0}}, true);

    const auto status = map.status();
    EXPECT_GT(status.occupied_cells, 0u);
    EXPECT_EQ(status.known_anchor_count, 1u);
    EXPECT_EQ(status.visible_anchor_count, 1u);
}

TEST(TdoaIngestor, AcceptsGatewayStyleKeyValueMeasurementsFromCsv) {
    const auto temp_path = std::filesystem::temp_directory_path() / "tdoa_gateway_measurements.csv";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << "anchor_id=1,arrival_time_s=10.001\n";
        output << "anchor_id=2,arrival_time_s=10.002\n";
        output << "anchor_id=3,arrival_time_s=10.003\n";
        output << "anchor_id=4,arrival_time_s=10.004\n";
    }

    localization::TDOAIngestor ingestor({
        localization::TDOAIngestor::Mode::CSV_FILE,
        temp_path.string(),
        0,
        8,
    });
    ASSERT_TRUE(ingestor.start());

    const auto batch = ingestor.poll();
    ASSERT_TRUE(batch.has_value());
    ASSERT_EQ(batch->size(), 4u);
    EXPECT_EQ((*batch)[0].anchor_id, 1u);
    EXPECT_NEAR((*batch)[3].arrival_time_s, 10.004, 1e-9);
    EXPECT_EQ(ingestor.visible_anchor_count(), 4u);

    ingestor.stop();
    std::filesystem::remove(temp_path);
}

TEST(KeyframeManager, RelocalizesAgainstStoredKeyframe) {
    slam::KeyframeManager manager(7, nullptr);

    cv::Mat image(320, 320, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(40, 40, 60, 60), cv::Scalar(255), cv::FILLED);
    cv::circle(image, cv::Point(220, 90), 28, cv::Scalar(190), cv::FILLED);
    cv::line(image, cv::Point(30, 260), cv::Point(290, 250), cv::Scalar(220), 4);

    const auto id = manager.try_add_frame(
        image,
        Eigen::Vector3d(4.0, -1.0, 8.0),
        Eigen::Quaterniond::Identity(),
        1.0);
    ASSERT_TRUE(id.has_value());

    const auto relocalized = manager.attempt_relocalization(
        image,
        Eigen::Vector3d(7.0, -1.5, 8.2),
        Eigen::Quaterniond::Identity());
    ASSERT_TRUE(relocalized.has_value());
    EXPECT_GT(relocalized->confidence, 0.45);
    EXPECT_NEAR(relocalized->corrected_position.x(), 4.9, 1.5);
}

TEST(MapPlanner, BuildsWaypointChainAcrossOccupancyMap) {
    slam::OccupancyGridMap map;
    sensors::LidarMeasurement scan;
    scan.cloud.reset(new sensors::PointCloud());
    scan.cloud->push_back(pcl::PointXYZI{1.0f, 0.0f, 0.0f, 1.0f});
    scan.cloud->push_back(pcl::PointXYZI{2.0f, 0.8f, 0.0f, 1.0f});
    map.integrate_lidar(scan, Eigen::Vector3d::Zero());
    map.mark_anchor({1, Eigen::Vector3d{3.0, 0.0, 0.0}}, true);

    slam::MapPlanner planner;
    const auto plan = planner.plan(
        map.status(),
        Eigen::Vector3d(0.0, 0.0, 8.0),
        Eigen::Vector3d(6.0, 0.0, 8.0));

    ASSERT_TRUE(plan.has_value());
    EXPECT_FALSE(plan->waypoints.empty());
    EXPECT_TRUE(plan->used_anchor_guidance);
    EXPECT_GT(plan->total_cost, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
