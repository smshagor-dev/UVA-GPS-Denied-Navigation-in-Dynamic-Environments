// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include "autonomy/DecisionEngine.hpp"

using namespace drone;

namespace {

sensors::Detection make_detection(std::string label,
                                  float confidence,
                                  float x,
                                  float y,
                                  float w,
                                  float h) {
    sensors::Detection detection;
    detection.label = std::move(label);
    detection.confidence = confidence;
    detection.bbox = cv::Rect2f{x, y, w, h};
    return detection;
}

autonomy::DecisionContext make_context() {
    autonomy::DecisionContext ctx;
    ctx.inference_ready = true;
    ctx.pose.position = Eigen::Vector3d(0.0, 0.0, 8.0);
    ctx.pose.orientation = Eigen::Quaterniond::Identity();
    ctx.system.battery_pct = 70.0f;
    ctx.localization_confidence = 0.95;
    ctx.localization_source = "vision-inertial";
    ctx.now_s = 10.0;
    return ctx;
}

} // namespace

TEST(DecisionEngine, TracksCenteredTargetWhenSafe) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();

    sensors::CameraFrame frame;
    frame.detections.push_back(make_detection("drone", 0.92f, 0.40f, 0.35f, 0.18f, 0.18f));
    ctx.frame = frame;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::TRACK_TARGET);
    EXPECT_GT(command.desired_velocity.z(), 0.0);
    EXPECT_NEAR(command.desired_yaw_rate_rads, 0.0, 0.3);
}

TEST(DecisionEngine, AvoidsLargeCentralHazard) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();

    sensors::CameraFrame frame;
    frame.detections.push_back(make_detection("person", 0.95f, 0.25f, 0.20f, 0.45f, 0.55f));
    ctx.frame = frame;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::AVOID_OBSTACLE);
    EXPECT_LT(command.desired_velocity.z(), 0.0);
    EXPECT_TRUE(command.requires_operator_attention);
}

TEST(DecisionEngine, ReturnsHomeOnLowBattery) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    static_cast<void>(engine.update(ctx)); // initialize home

    ctx.pose.position = Eigen::Vector3d(5.0, 0.0, 8.0);
    ctx.system.battery_pct = 12.0f;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::RETURN_HOME);
    EXPECT_LT(command.desired_velocity.x(), 0.0);
}

TEST(DecisionEngine, FollowerHoldsWhenNoActionableDetection) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    ctx.swarm_follower = true;
    ctx.swarm_peer_count = 3;
    ctx.pose.velocity = Eigen::Vector3d(0.6, -0.2, 0.1);
    ctx.frame = sensors::CameraFrame{};

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::HOLD_POSITION);
    EXPECT_LT(command.desired_velocity.x(), 0.0);
}

TEST(DecisionEngine, UsesMemoryPriorToSlowSearchBehavior) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    ctx.frame = sensors::CameraFrame{};

    autonomy::MemoryPrior prior;
    prior.risk_score = 0.9;
    prior.recommend_caution = true;
    ctx.memory_prior = prior;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::SEARCH);
    EXPECT_LT(command.desired_velocity.norm(), 1.2);
    EXPECT_NE(command.summary.find("cautiously"), std::string::npos);
}

TEST(DecisionEngine, UsesTdoaReferenceForReturnHome) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    static_cast<void>(engine.update(ctx));

    ctx.pose.position = Eigen::Vector3d(5.0, 0.0, 8.0);
    ctx.system.battery_pct = 12.0f;
    ctx.tdoa_position = Eigen::Vector3d(8.0, 0.0, 8.0);
    ctx.tdoa_confidence = 0.8;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::RETURN_HOME);
    EXPECT_LT(command.desired_velocity.x(), 0.0);
}

TEST(DecisionEngine, EntersLocalizationDegradedModeWhenConfidenceDrops) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    ctx.localization_confidence = 0.42;
    ctx.localization_degraded = true;
    ctx.sync_confidence = 0.9;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::LOCALIZATION_DEGRADED);
    EXPECT_NE(command.summary.find("Localization degraded"), std::string::npos);
}

TEST(DecisionEngine, EntersLocalizationLostModeAndUsesTdoaRecovery) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    ctx.localization_confidence = 0.12;
    ctx.localization_lost = true;
    ctx.tdoa_position = Eigen::Vector3d(2.0, 0.0, 8.0);
    ctx.tdoa_confidence = 0.9;
    ctx.visible_anchor_count = 3;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::SAFE_RETURN_BY_ANCHOR);
    EXPECT_GT(command.desired_velocity.x(), 0.0);
    EXPECT_TRUE(command.requires_operator_attention);
}

TEST(DecisionEngine, HoversAndScansWhenLocalizationAndSyncArePoor) {
    autonomy::DecisionEngine engine;
    auto ctx = make_context();
    ctx.localization_confidence = 0.18;
    ctx.localization_lost = true;
    ctx.sync_confidence = 0.3;
    ctx.camera_tracking_nominal = false;

    const auto command = engine.update(ctx);
    EXPECT_EQ(command.mode, autonomy::BehaviorMode::HOVER_AND_SCAN);
    EXPECT_NE(command.summary.find("hovering and scanning"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
