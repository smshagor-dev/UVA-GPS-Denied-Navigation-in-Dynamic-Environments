// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include "telemetry/ControlPlaneTelemetryClient.hpp"

using namespace drone::telemetry;

namespace {

TelemetrySnapshot make_snapshot() {
    TelemetrySnapshot snapshot;
    snapshot.drone_id = 7;
    snapshot.cluster_id = "cluster-01";
    snapshot.role = "FOLLOWER";
    snapshot.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    snapshot.velocity = Eigen::Vector3d(0.1, 0.2, 0.3);
    snapshot.localization_source = "vision-inertial";
    snapshot.localization_data_source = "real";
    snapshot.localization_state = "degraded";
    snapshot.localization_confidence = 0.42;
    snapshot.security_state = "SAFE_RETURN";
    snapshot.safety_state = "DEGRADED_LOCALIZATION";
    snapshot.battery_pct = 71.0;
    snapshot.rssi_dbm = -58.0;
    snapshot.health_flags = {"localization_degraded", "link_low"};
    snapshot.source = "real";
    return snapshot;
}

} // namespace

TEST(ControlPlaneTelemetryClient, SerializesTelemetryPayload) {
    std::string captured_body;
    ControlPlaneTelemetryClient client(
        "http://127.0.0.1:8080/api/v1/telemetry", "shared-secret", 1000, 500,
        [&](const ControlPlaneTelemetryClient::ParsedEndpoint& endpoint, std::string_view body,
            const ControlPlaneTelemetryClient::HeaderList& headers,
            int timeout_ms) -> ControlPlaneTelemetryClient::HttpResponse {
            EXPECT_EQ(endpoint.host, "127.0.0.1");
            EXPECT_EQ(endpoint.path, "/api/v1/telemetry");
            EXPECT_EQ(timeout_ms, 500);
            EXPECT_GE(headers.size(), 3u);
            captured_body.assign(body.begin(), body.end());
            return ControlPlaneTelemetryClient::HttpResponse{true, 202, "accepted"};
        });

    const auto now = std::chrono::steady_clock::now();
    EXPECT_TRUE(client.publish(make_snapshot(), now));
    EXPECT_NE(captured_body.find("\"drone_id\":7"), std::string::npos);
    EXPECT_NE(captured_body.find("\"source\":\"real\""), std::string::npos);
    EXPECT_NE(captured_body.find("\"localization_source\":\"vision-inertial\""), std::string::npos);
    EXPECT_NE(captured_body.find("\"localization_confidence\":0.4200"), std::string::npos);
    EXPECT_NE(captured_body.find("\"security_state\":\"SAFE_RETURN\""), std::string::npos);
    EXPECT_NE(captured_body.find("\"safety_state\":\"DEGRADED_LOCALIZATION\""), std::string::npos);
}

TEST(ControlPlaneTelemetryClient, SerializesNestedSensorPayloadsAndCapsLidarPoints) {
    auto snapshot = make_snapshot();
    snapshot.camera.status = "live";
    snapshot.camera.fps = 29.5;
    snapshot.camera.frame_age_ms = 42.0;
    snapshot.camera.resolution = "1280x720";
    snapshot.camera.dropped_frames = 1;
    snapshot.camera.source = "real";
    snapshot.camera.latest_frame_ref = "frame-123";
    snapshot.imu.status = "live";
    snapshot.imu.sample_rate_hz = 200.0;
    snapshot.imu.last_sample_age_ms = 3.0;
    snapshot.imu.accel = {0.1, 0.2, 9.8};
    snapshot.imu.gyro = {0.01, 0.02, 0.03};
    snapshot.imu.health = "good";
    snapshot.imu.source = "real";
    snapshot.lidar.status = "live";
    snapshot.lidar.source = "real";
    snapshot.lidar.point_count = 2;
    snapshot.lidar.points_2d = {{1.0, 2.0, 0.8}, {-1.0, 0.5, 0.4}};
    snapshot.tdoa.status = "playback";
    snapshot.tdoa.source = "playback";
    snapshot.tdoa.visible_anchor_count = 2;
    snapshot.tdoa.anchors = {{"A0", 0.0, 0.0, 2.5, true, 20.0}, {"A1", 8.0, 0.0, 2.5, true, 22.0}};
    snapshot.replay.status = "playback";
    snapshot.replay.active = true;
    snapshot.replay.file_name = "session.csv";
    snapshot.replay.progress = 0.4;
    snapshot.replay.current_time = 12.5;
    snapshot.replay.confidence_series = {0.9, 0.85, 0.8};
    snapshot.replay.source = "playback";

    const std::string body = ControlPlaneTelemetryClient::serialize_payload(snapshot);
    EXPECT_NE(body.find("\"camera\":{"), std::string::npos);
    EXPECT_NE(body.find("\"latest_frame_ref\":\"frame-123\""), std::string::npos);
    EXPECT_NE(body.find("\"sample_rate_hz\":200.0000"), std::string::npos);
    EXPECT_NE(body.find("\"points_2d\":"), std::string::npos);
    EXPECT_NE(body.find("\"id\":\"A0\""), std::string::npos);
    EXPECT_NE(body.find("\"file_name\":\"session.csv\""), std::string::npos);
}

TEST(ControlPlaneTelemetryClient, MissingSensorsSerializeAsUnavailable) {
    auto snapshot = make_snapshot();
    snapshot.source = "simulation";
    const std::string body = ControlPlaneTelemetryClient::serialize_payload(snapshot);
    EXPECT_NE(body.find("\"camera\":{\"status\":\"unavailable\""), std::string::npos);
    EXPECT_NE(body.find("\"imu\":{\"status\":\"unavailable\""), std::string::npos);
    EXPECT_NE(body.find("\"lidar\":{\"status\":\"unavailable\""), std::string::npos);
    EXPECT_NE(body.find("\"source\":\"simulation\""), std::string::npos);
}

TEST(ControlPlaneTelemetryClient, FailedBackendDoesNotCrashDrone) {
    int attempts = 0;
    ControlPlaneTelemetryClient client(
        "http://127.0.0.1:8080/api/v1/telemetry", "shared-secret", 1000, 500,
        [&](const ControlPlaneTelemetryClient::ParsedEndpoint&, std::string_view,
            const ControlPlaneTelemetryClient::HeaderList&, int) {
            ++attempts;
            return ControlPlaneTelemetryClient::HttpResponse{false, 0, "connection refused"};
        });

    const auto now = std::chrono::steady_clock::now();
    EXPECT_FALSE(client.publish(make_snapshot(), now));
    EXPECT_EQ(attempts, 1);
    EXPECT_NE(client.last_status().find("error:"), std::string::npos);
}

TEST(ControlPlaneTelemetryClient, RetryLogicBacksOffAfterFailure) {
    int attempts = 0;
    ControlPlaneTelemetryClient client(
        "http://127.0.0.1:8080/api/v1/telemetry", "shared-secret", 1000, 500,
        [&](const ControlPlaneTelemetryClient::ParsedEndpoint&, std::string_view,
            const ControlPlaneTelemetryClient::HeaderList&, int) {
            ++attempts;
            if (attempts == 1) {
                return ControlPlaneTelemetryClient::HttpResponse{false, 0, "timeout"};
            }
            return ControlPlaneTelemetryClient::HttpResponse{true, 202, "accepted"};
        });

    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(client.publish(make_snapshot(), t0));
    EXPECT_EQ(client.consecutive_failures(), 1);
    EXPECT_FALSE(client.should_publish(t0 + std::chrono::milliseconds(100)));
    EXPECT_TRUE(client.should_publish(t0 + std::chrono::milliseconds(350)));
    EXPECT_TRUE(client.publish(make_snapshot(), t0 + std::chrono::milliseconds(350)));
    EXPECT_EQ(client.consecutive_failures(), 0);
}

TEST(ControlPlaneTelemetryClient, LinuxPathCompiles) {
#ifdef _WIN32
    SUCCEED();
#else
    ControlPlaneTelemetryClient client("http://127.0.0.1:8080");
    EXPECT_TRUE(client.enabled());
#endif
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
