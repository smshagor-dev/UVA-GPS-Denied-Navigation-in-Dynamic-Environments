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
#include "runtime/RuntimeMode.hpp"
#include "security/CommandPolicy.hpp"
#include "security/DroneSecurity.hpp"
#include "slam/KeyframeManager.hpp"
#include "slam/MapPlanner.hpp"
#include "slam/OccupancyGridMap.hpp"
#include "vio/VIOPipeline.hpp"

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

cv::Mat make_feature_image(int seed) {
    cv::Mat image(320, 320, CV_8UC1, cv::Scalar(0));
    cv::RNG rng(seed);
    for (int i = 0; i < 60; ++i) {
        const cv::Point center(rng.uniform(20, 300), rng.uniform(20, 300));
        cv::circle(image, center, rng.uniform(3, 9), cv::Scalar(rng.uniform(120, 255)), cv::FILLED);
    }
    cv::line(image, cv::Point(20, 40), cv::Point(300, 280), cv::Scalar(255), 2);
    cv::rectangle(image, cv::Rect(60, 180, 90, 50), cv::Scalar(180), 2);
    return image;
}

} // namespace

TEST(ExperienceMemory, SummarizesRiskFromHistory) {
    autonomy::ExperienceMemory memory;
    vio::PoseEstimate pose;
    hal::SystemStats stats;
    stats.battery_pct = 92.0f;
    pose.pos_std = Eigen::Vector3d(0.02, 0.02, 0.02);

    memory.observe(7, pose, stats, make_frame("tree", 0.9f), 3, 0.92, "vision-inertial", false,
                   0.0);

    pose.pos_std = Eigen::Vector3d(0.30, 0.25, 0.20);
    stats.battery_pct = 78.0f;
    memory.observe(7, pose, stats, make_frame("tree", 0.92f), 4, 0.35, "imu-dead-reckoning", true,
                   60.0);

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
        {2, Eigen::Vector3d{10.0, -10.0, 0.0}},
        {3, Eigen::Vector3d{10.0, 10.0, 0.0}},
        {4, Eigen::Vector3d{-10.0, 10.0, 0.0}},
        {5, Eigen::Vector3d{0.0, 0.0, 15.0}},
    });

    const Eigen::Vector3d target{2.0, -3.0, 4.5};
    constexpr double kSignalSpeedMps = 299702547.0;
    constexpr double kTxBiasS = 4.2e-7;

    std::vector<localization::TDOALocalizer::Measurement> measurements;
    for (const auto& anchor : localizer.anchors()) {
        measurements.push_back(
            {anchor.id, kTxBiasS + ((target - anchor.position).norm() / kSignalSpeedMps)});
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

TEST(LocalizationFusion, MarksLocalizationLostWhenCameraAndSyncCollapse) {
    localization::LocalizationFusion fusion;
    localization::LocalizationFusionInput input;
    input.vio_pose.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    input.vio_pose.drift_m = 2.8;
    input.vio_pose.localization_confidence = 0.24;
    input.camera_available = false;
    input.lidar_available = false;
    input.rangefinder_available = false;
    input.anchor_visibility_ratio = 0.0;
    input.time_sync.confidence = 0.25;
    input.time_sync.synchronized = false;

    const auto output = fusion.update(input);
    EXPECT_TRUE(output.lost);
    EXPECT_TRUE(output.degraded);
    EXPECT_EQ(output.state, "lost");
    EXPECT_EQ(output.source, "imu-dead-reckoning");
    EXPECT_LT(output.confidence, 0.22);
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

TEST(TdoaIngestor, RejectsIncompleteCsvBatchAndTracksOnlyValidAnchors) {
    const auto temp_path = std::filesystem::temp_directory_path() / "tdoa_invalid_measurements.csv";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << "anchor_id=1,arrival_time_s=10.001\n";
        output << "anchor_id=oops,arrival_time_s=10.002\n";
        output << "malformed line\n";
        output << "anchor_id=3,arrival_time_s=10.003\n";
    }

    localization::TDOAIngestor ingestor({
        localization::TDOAIngestor::Mode::CSV_FILE,
        temp_path.string(),
        0,
        8,
    });
    ASSERT_TRUE(ingestor.start());

    const auto batch = ingestor.poll();
    EXPECT_FALSE(batch.has_value());
    EXPECT_EQ(ingestor.visible_anchor_count(), 2u);
    EXPECT_DOUBLE_EQ(ingestor.visibility_ratio(5), 0.4);

    ingestor.stop();
    std::filesystem::remove(temp_path);
}

TEST(CommandPolicy, BlocksRemoteCommandsWhenSecurityStateIsolated) {
    security::DroneSecurityInputs inputs;
    inputs.security_profile = "production";
    inputs.hardened_profile = true;
    inputs.swarm_security_enabled = false;
    inputs.placeholder_secret = true;

    const auto assessment = security::assess_security(inputs);
    ASSERT_EQ(assessment.state, security::DroneSecurityState::ISOLATED_AUTONOMY);

    security::RemoteCommandEnvelope command;
    command.action = security::RemoteCommandAction::RETURN_HOME;
    command.src_id = 9;
    command.summary = "remote return home";

    const auto policy = security::evaluate_remote_command(assessment, command);
    EXPECT_FALSE(policy.accepted);
    EXPECT_NE(policy.reason.find("blocked"), std::string::npos);
}

TEST(RuntimeMode, ProductionRejectsMissingAnchorAndLiveTdoaSource) {
    const auto result = runtime::validate_runtime_configuration({
        runtime::RuntimeMode::PRODUCTION,
        "",
        false,
        false,
        false,
    });
    EXPECT_FALSE(result.ok);
    ASSERT_GE(result.errors.size(), 2u);
}

TEST(RuntimeMode, SimulationAllowsSyntheticFallbackWithoutAnchorConfig) {
    const auto result = runtime::validate_runtime_configuration({
        runtime::RuntimeMode::SIMULATION,
        "",
        false,
        false,
        false,
    });
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(runtime::determine_localization_data_source(runtime::RuntimeMode::SIMULATION, true,
                                                          false, false, true),
              "simulation");
}

TEST(RuntimeMode, RuntimeFileLoadsEscapedPaths) {
    const auto temp_path = std::filesystem::temp_directory_path() / "runtime_valid.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "runtime_mode": "edge_swarm",
  "anchor_config_path": "config\/anchors.json",
  "lidar_config_path": "config\/lidar.json",
  "detector_labels_path": "config\/detector_labels.json"
})";
    }

    const auto result = runtime::load_runtime_file(temp_path.string());
    EXPECT_TRUE(result.loaded);
    ASSERT_TRUE(result.runtime_mode.has_value());
    EXPECT_EQ(*result.runtime_mode, runtime::RuntimeMode::EDGE_SWARM);
    EXPECT_EQ(result.anchor_config_path, "config/anchors.json");
    EXPECT_EQ(result.lidar_config_path, "config/lidar.json");
    EXPECT_EQ(result.detector_labels_path, "config/detector_labels.json");

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, ValidAnchorJsonLoads) {
    const auto temp_path = std::filesystem::temp_directory_path() / "anchors_valid.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "coordinate_frame": "local_enu",
  "units": "meters",
  "anchors": [
    { "id": "A0", "x": 0.0, "y": 0.0, "z": 2.5 },
    { "id": "A1", "x": 8.0, "y": 0.0, "z": 2.5 },
    { "id": "A2", "x": 0.0, "y": 8.0, "z": 2.5 },
    { "id": "A3", "x": 8.0, "y": 8.0, "z": 2.5 }
  ]
})";
    }

    const auto result = runtime::load_anchor_config_json(temp_path.string());
    EXPECT_TRUE(result.ok);
    ASSERT_EQ(result.anchors.size(), 4u);
    EXPECT_EQ(result.coordinate_frame, "local_enu");
    EXPECT_EQ(result.units, "meters");
    EXPECT_EQ(result.anchors.front().source_id, "A0");
    EXPECT_EQ(result.anchors.front().id, 0u);

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, DuplicateAnchorsFail) {
    const auto temp_path = std::filesystem::temp_directory_path() / "anchors_duplicate.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "coordinate_frame": "local_enu",
  "units": "meters",
  "anchors": [
    { "id": "A0", "x": 0.0, "y": 0.0, "z": 2.5 },
    { "id": "A0", "x": 8.0, "y": 0.0, "z": 2.5 },
    { "id": "A2", "x": 0.0, "y": 8.0, "z": 2.5 },
    { "id": "A3", "x": 8.0, "y": 8.0, "z": 2.5 }
  ]
})";
    }

    const auto result = runtime::load_anchor_config_json(temp_path.string());
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, InsufficientAnchorsFail) {
    const auto temp_path = std::filesystem::temp_directory_path() / "anchors_insufficient.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "coordinate_frame": "local_enu",
  "units": "meters",
  "anchors": [
    { "id": "A0", "x": 0.0, "y": 0.0, "z": 2.5 },
    { "id": "A1", "x": 8.0, "y": 0.0, "z": 2.5 },
    { "id": "A2", "x": 0.0, "y": 8.0, "z": 2.5 }
  ]
})";
    }

    const auto result = runtime::load_anchor_config_json(temp_path.string());
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.errors.empty());

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, ValidLidarConfigLoads) {
    const auto temp_path = std::filesystem::temp_directory_path() / "lidar_valid.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "host": "0.0.0.0",
  "port": 2368,
  "model": "generic_udp_cartesian_v1",
  "frame_id": "lidar_front",
  "min_range_m": 0.3,
  "max_range_m": 80.0,
  "required": true
})";
    }

    const auto result = runtime::load_lidar_config_json(temp_path.string());
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.host, "0.0.0.0");
    EXPECT_EQ(result.port, 2368);
    EXPECT_EQ(result.model, "generic_udp_cartesian_v1");
    EXPECT_TRUE(result.required);

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, LidarConfigFalseRequiredLoads) {
    const auto temp_path = std::filesystem::temp_directory_path() / "lidar_optional.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "host": "192.168.1.10",
  "port": 2368,
  "model": "generic_udp_cartesian_v1",
  "frame_id": "lidar_rear",
  "min_range_m": 0.4,
  "max_range_m": 60.0,
  "required": false
})";
    }

    const auto result = runtime::load_lidar_config_json(temp_path.string());
    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(result.required);
    EXPECT_EQ(result.frame_id, "lidar_rear");

    std::filesystem::remove(temp_path);
}

TEST(RuntimeMode, ProductionFailsIfRequiredLidarUnavailable) {
    const auto result = runtime::validate_lidar_runtime_configuration({
        runtime::RuntimeMode::PRODUCTION,
        true,
        true,
        false,
    });
    EXPECT_FALSE(result.ok);
    ASSERT_FALSE(result.errors.empty());
}

TEST(VIOFrontend, LowFeatureCountLowersConfidence) {
    Eigen::Matrix3d K;
    K << 800, 0, 160, 0, 800, 160, 0, 0, 1;
    vio::PoseEstimate previous_pose;
    vio::PoseEstimate predicted_pose;
    predicted_pose.position = Eigen::Vector3d(0.1, 0.0, 0.0);
    predicted_pose.velocity = Eigen::Vector3d(0.5, 0.0, 0.0);

    const cv::Mat blank(320, 320, CV_8UC1, cv::Scalar(0));
    const auto result =
        vio::run_visual_frontend(blank, blank, K, previous_pose, predicted_pose, 0.2);

    EXPECT_LT(result.metrics.tracked_feature_count, 8u);
    EXPECT_LT(result.metrics.visual_update_confidence, 0.3);
    EXPECT_FALSE(result.metrics.update_accepted);
}

TEST(VIOFrontend, HighOutlierRatioRejectsUpdate) {
    Eigen::Matrix3d K;
    K << 800, 0, 160, 0, 800, 160, 0, 0, 1;
    vio::PoseEstimate previous_pose;
    vio::PoseEstimate predicted_pose;
    predicted_pose.position = Eigen::Vector3d(0.2, 0.0, 0.0);
    predicted_pose.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

    const auto previous = make_feature_image(1);
    const auto unrelated = make_feature_image(99);
    const auto result =
        vio::run_visual_frontend(previous, unrelated, K, previous_pose, predicted_pose, 0.2);

    EXPECT_FALSE(result.metrics.update_accepted);
    EXPECT_TRUE(result.metrics.inlier_ratio < 0.55 || result.metrics.reprojection_error > 3.5);
}

TEST(VIOFrontend, PlaceholderCannotRunInProductionMode) {
    EXPECT_FALSE(vio::visual_placeholder_allowed(runtime::RuntimeMode::PRODUCTION));
    EXPECT_FALSE(vio::visual_placeholder_allowed(runtime::RuntimeMode::BENCH));
    EXPECT_TRUE(vio::visual_placeholder_allowed(runtime::RuntimeMode::SIMULATION));
}

TEST(KeyframeManager, RelocalizesAgainstStoredKeyframe) {
    slam::KeyframeManager manager(7, nullptr);

    cv::Mat image(320, 320, CV_8UC1, cv::Scalar(0));
    cv::rectangle(image, cv::Rect(40, 40, 60, 60), cv::Scalar(255), cv::FILLED);
    cv::circle(image, cv::Point(220, 90), 28, cv::Scalar(190), cv::FILLED);
    cv::line(image, cv::Point(30, 260), cv::Point(290, 250), cv::Scalar(220), 4);

    const auto id = manager.try_add_frame(image, Eigen::Vector3d(4.0, -1.0, 8.0),
                                          Eigen::Quaterniond::Identity(), 1.0);
    ASSERT_TRUE(id.has_value());

    const auto relocalized = manager.attempt_relocalization(image, Eigen::Vector3d(7.0, -1.5, 8.2),
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
    const auto plan =
        planner.plan(map.status(), Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(6.0, 0.0, 8.0));

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
