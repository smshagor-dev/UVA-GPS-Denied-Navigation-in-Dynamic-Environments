#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "estimation/ReplayRunner.hpp"

using namespace drone;

namespace {

using json = nlohmann::json;

std::filesystem::path phase15_dataset_path() {
#ifdef DRONE_SOURCE_DIR
    return std::filesystem::path(DRONE_SOURCE_DIR) /
           "datasets/estimator/phase15_stationary_replay.json";
#else
    return std::filesystem::path("datasets/estimator/phase15_stationary_replay.json");
#endif
}

std::filesystem::path write_json_file(const std::string& name, const json& payload) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream output(path.string(), std::ios::trunc);
    EXPECT_TRUE(output.is_open());
    output << payload.dump(2);
    output.close();
    return path;
}

json baseline_payload() {
    return json{
        {"schema_version", 1},
        {"coordinate_frame", "local_enu"},
        {"initial_state",
         {{"position", {0.0, 0.0, 0.0}}, {"velocity", {0.0, 0.0, 0.0}}, {"yaw_rad", 0.0}}},
        {"records",
         json::array({
             {{"type", "imu"},
              {"timestamp_s", 0.01},
              {"accel_mps2", {0.0, 0.0, 9.81}},
              {"gyro_rads", {0.0, 0.0, 0.0}}},
             {{"type", "imu"},
              {"timestamp_s", 0.02},
              {"accel_mps2", {0.0, 0.0, 9.81}},
              {"gyro_rads", {0.0, 0.0, 0.0}}},
             {{"type", "depth"}, {"timestamp_s", 0.03}, {"z_world_m", 0.02}, {"sigma_m", 0.05}},
             {{"type", "visual_pose"},
              {"timestamp_s", 0.04},
              {"position_m", {0.01, 0.0, 0.02}},
              {"velocity_mps", {0.0, 0.0, 0.0}},
              {"sigma_position_m", 0.2},
              {"sigma_velocity_mps", 0.2}},
         })},
    };
}

} // namespace

TEST(EstimatorReplay, ActiveOnlySucceedsOnPhase15Dataset) {
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(phase15_dataset_path(), config);
    EXPECT_EQ(result.exit_code, 0);
    EXPECT_TRUE(result.report.success);
    EXPECT_EQ(result.report.replay_mode, "active_only");
}

TEST(EstimatorReplay, RepeatingReplayProducesSameHash) {
    const auto path = write_json_file("replay_repeat_phase16b.json", baseline_payload());
    estimation::ReplayRunConfig config;
    const auto a = estimation::run_replay_file(path, config);
    const auto b = estimation::run_replay_file(path, config);
    EXPECT_EQ(a.exit_code, 0);
    EXPECT_EQ(b.exit_code, 0);
    EXPECT_EQ(a.report.deterministic_result_hash, b.report.deterministic_result_hash);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, ChangingMeasurementChangesHash) {
    auto payload_a = baseline_payload();
    auto payload_b = baseline_payload();
    payload_b["records"][1]["accel_mps2"][0] = 0.2;
    const auto path_a = write_json_file("replay_hash_a_phase16b.json", payload_a);
    const auto path_b = write_json_file("replay_hash_b_phase16b.json", payload_b);

    estimation::ReplayRunConfig config;
    const auto a = estimation::run_replay_file(path_a, config);
    const auto b = estimation::run_replay_file(path_b, config);
    EXPECT_EQ(a.exit_code, 0);
    EXPECT_EQ(b.exit_code, 0);
    EXPECT_NE(a.report.deterministic_result_hash, b.report.deterministic_result_hash);

    std::filesystem::remove(path_a);
    std::filesystem::remove(path_b);
}

TEST(EstimatorReplay, IdenticalShadowKeepsActiveStateCompatible) {
    const auto path = write_json_file("replay_shadow_phase16b.json", baseline_payload());

    estimation::ReplayRunConfig active_only;
    estimation::ReplayRunConfig with_shadow;
    with_shadow.mode = estimation::ReplayMode::ACTIVE_WITH_IDENTICAL_SHADOW;

    const auto a = estimation::run_replay_file(path, active_only);
    const auto b = estimation::run_replay_file(path, with_shadow);
    EXPECT_EQ(a.exit_code, 0);
    EXPECT_EQ(b.exit_code, 0);
    EXPECT_TRUE(a.report.final_pose.position.isApprox(b.report.final_pose.position, 1e-12));
    EXPECT_TRUE(a.report.final_pose.velocity.isApprox(b.report.final_pose.velocity, 1e-12));
    EXPECT_LT(b.report.active_shadow_position_delta_m, 1e-9);

    std::filesystem::remove(path);
}

TEST(EstimatorReplay, MalformedJsonFails) {
    const auto path = std::filesystem::temp_directory_path() / "replay_malformed_phase16b.json";
    std::ofstream output(path.string(), std::ios::trunc);
    ASSERT_TRUE(output.is_open());
    output << "{ invalid";
    output.close();

    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    EXPECT_FALSE(result.report.success);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, UnknownSchemaVersionFails) {
    auto payload = baseline_payload();
    payload["schema_version"] = 99;
    const auto path = write_json_file("replay_bad_schema_phase16b.json", payload);

    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    EXPECT_FALSE(result.report.success);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, OversizedInputFails) {
    const auto path = write_json_file("replay_oversized_phase16b.json", baseline_payload());
    estimation::ReplayRunConfig config;
    config.limits.max_file_bytes = 32;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, ExcessiveRecordCountFails) {
    auto payload = baseline_payload();
    payload["records"] = json::array();
    for (int i = 0; i < 5; ++i) {
        payload["records"].push_back(
            {{"type", "imu"},
             {"timestamp_s", 0.01 * static_cast<double>(i + 1)},
             {"accel_mps2", {0.0, 0.0, 9.81}},
             {"gyro_rads", {0.0, 0.0, 0.0}}});
    }
    const auto path = write_json_file("replay_record_limit_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    config.limits.max_record_count = 2;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, BackwardTimestampFails) {
    auto payload = baseline_payload();
    payload["records"][1]["timestamp_s"] = 0.005;
    const auto path = write_json_file("replay_backwards_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    EXPECT_EQ(result.report.timestamp_violation_count, 1u);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, InvalidCovarianceFails) {
    auto payload = baseline_payload();
    payload["records"][3]["sigma_position_m"] = -1.0;
    const auto path = write_json_file("replay_bad_cov_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, NonFiniteValueFails) {
    auto payload = baseline_payload();
    payload["records"][0]["accel_mps2"][0] = "NaN";
    const auto path = write_json_file("replay_nan_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    EXPECT_FALSE(result.report.success);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, InvalidQuaternionFails) {
    auto payload = baseline_payload();
    payload["records"][3]["orientation_wxyz"] = {0.0, 0.0, 0.0, 0.0};
    const auto path = write_json_file("replay_bad_quat_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, UnsupportedMeasurementFailsWithNonZeroStatus) {
    auto payload = baseline_payload();
    payload["records"][2]["type"] = "loop_closure";
    const auto path = write_json_file("replay_unsupported_measurement_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    EXPECT_FALSE(result.report.success);
    EXPECT_GT(result.report.unsupported_measurement_count, 0u);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, UnsupportedFrameFails) {
    auto payload = baseline_payload();
    payload["coordinate_frame"] = "map_magic";
    const auto path = write_json_file("replay_bad_frame_phase16b.json", payload);
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    EXPECT_NE(result.exit_code, 0);
    std::filesystem::remove(path);
}

TEST(EstimatorReplay, ReportContainsRequiredFields) {
    const auto path = write_json_file("replay_report_phase16b.json", baseline_payload());
    estimation::ReplayRunConfig config;
    const auto result = estimation::run_replay_file(path, config);
    ASSERT_EQ(result.exit_code, 0);

    const auto output = nlohmann::json::parse(estimation::replay_report_json(result.report));
    EXPECT_TRUE(output.contains("report_schema_version"));
    EXPECT_TRUE(output.contains("replay_mode"));
    EXPECT_TRUE(output.contains("deterministic_result_hash"));
    EXPECT_TRUE(output.contains("final_position_m"));
    EXPECT_TRUE(output.contains("success"));

    std::filesystem::remove(path);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
