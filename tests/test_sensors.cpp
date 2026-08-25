// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// test_sensors.cpp    GoogleTest suite for sensor subsystem

#include <gtest/gtest.h>
#include "sensors/SensorBase.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/CameraSensor.hpp"
#include "sensors/LidarSensor.hpp"
#include "swarm/V2XMeshNetwork.hpp"

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>

#ifdef _WIN32
#define NOMINMAX
#include <Winsock2.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace drone::sensors;

//  Minimal concrete sensor for testing the base class â”€
class MockSensor : public SensorBase {
public:
    explicit MockSensor(std::string id) : SensorBase(std::move(id), "MOCK") {}

    bool initialize() override {
        poll_rate_hz_ = 100;
        set_state(SensorState::RUNNING);
        return true;
    }

    bool reconfigure(const std::string&) override {
        return true;
    }

    void poll() override {
        ++poll_calls_;
    }

    int poll_calls_{0};
};

//  SensorBase lifecycle
TEST(SensorBase, InitialStateIsUninitialized) {
    MockSensor s("test_sensor");
    EXPECT_EQ(s.state(), SensorState::UNINITIALIZED);
}

TEST(SensorBase, InitializeChangesState) {
    MockSensor s("test_sensor");
    ASSERT_TRUE(s.initialize());
    EXPECT_EQ(s.state(), SensorState::RUNNING);
}

TEST(SensorBase, StartAndStopAcquisitionThread) {
    MockSensor s("test_sensor");
    s.initialize();
    s.start();
    EXPECT_EQ(s.state(), SensorState::RUNNING);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    s.stop();

    // At 100Hz for 150ms, expect ~15 polls (allow generous bounds)
    EXPECT_GT(s.poll_calls_, 5);
    EXPECT_LT(s.poll_calls_, 40);
    EXPECT_EQ(s.state(), SensorState::DISCONNECTED);
}

TEST(SensorBase, SampleCountIncrementsOnPoll) {
    MockSensor s("count_test");
    s.initialize();
    s.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    s.stop();
    EXPECT_GT(s.sample_count(), 0u);
}

TEST(SensorBase, ErrorCallbackFires) {
    class ErrorSensor : public SensorBase {
    public:
        using SensorBase::SensorBase;
        bool initialize() override {
            return true;
        }
        bool reconfigure(const std::string&) override {
            return true;
        }
        void poll() override {
            report_error("test error");
        }
    };

    ErrorSensor s("error_sensor", "ERROR");
    bool cb_fired = false;
    s.set_error_callback([&](const std::string& msg) {
        cb_fired = true;
        EXPECT_EQ(msg, "test error");
    });
    s.initialize();
    s.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    s.stop();
    EXPECT_TRUE(cb_fired);
}

//  IMUSensor construction
TEST(IMUSensor, ConstructWithDefaults) {
    auto imu = std::make_shared<IMUSensor>("imu0");
    EXPECT_EQ(imu->sensor_id(), "imu0");
    EXPECT_EQ(imu->sensor_type(), "IMU");
    EXPECT_EQ(imu->state(), SensorState::UNINITIALIZED);
}

TEST(IMUSensor, LatestReturnsNulloptBeforeInit) {
    IMUSensor imu("imu_test");
    EXPECT_FALSE(imu.latest().has_value());
}

TEST(IMUSensor, DrainBufferReturnsEmptyBeforeInit) {
    IMUSensor imu("imu_drain");
    auto buf = imu.drain_buffer();
    EXPECT_TRUE(buf.empty());
}

//  LidarSensor construction â”€
TEST(LidarSensor, ConstructAndState) {
    LidarSensor lidar("lidar0", "127.0.0.1:2368");
    EXPECT_EQ(lidar.sensor_type(), "LiDAR");
    EXPECT_FALSE(lidar.latest().has_value());
}

TEST(LidarSensor, UdpReceiveTimeoutReturnsNoPacket) {
#ifdef _WIN32
    WSADATA wsa_data;
    ASSERT_EQ(WSAStartup(MAKEWORD(2, 2), &wsa_data), 0);
#endif
    const int sock = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    ASSERT_GE(sock, 0);

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(0);
    bind_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)), 0);

    const auto packet = receive_lidar_udp_packet(sock, 2048, 20);
    EXPECT_FALSE(packet.has_value());

#ifdef _WIN32
    closesocket(static_cast<SOCKET>(sock));
#else
    close(sock);
#endif
}

TEST(LidarSensor, InvalidPacketRejected) {
    RawLidarPacket packet;
    packet.bytes = {0x00, 0x01, 0x02, 0x03, 0x04};
    packet.timestamp = now_sec();

    auto parser = create_lidar_parser("generic_udp_cartesian_v1", "lidar_front", false);
    ASSERT_TRUE(parser != nullptr);
    EXPECT_FALSE(parser->parse(packet).has_value());
}

TEST(LidarSensor, ValidSamplePacketParsed) {
    RawLidarPacket packet;
    packet.timestamp = now_sec();
    packet.bytes.resize(8 + (2 * sizeof(float) * 4));
    std::memcpy(packet.bytes.data(), "LDR1", 4);
    const uint16_t version = 1;
    const uint16_t point_count = 2;
    std::memcpy(packet.bytes.data() + 4, &version, sizeof(version));
    std::memcpy(packet.bytes.data() + 6, &point_count, sizeof(point_count));

    const std::array<float, 8> values{{
        1.0f,
        0.0f,
        0.0f,
        0.8f,
        1.2f,
        0.2f,
        0.1f,
        0.6f,
    }};
    std::memcpy(packet.bytes.data() + 8, values.data(), values.size() * sizeof(float));

    auto parser = create_lidar_parser("generic_udp_cartesian_v1", "lidar_front", false);
    ASSERT_TRUE(parser != nullptr);
    const auto scan = parser->parse(packet);
    ASSERT_TRUE(scan.has_value());
    ASSERT_EQ(scan->points.size(), 2u);
    EXPECT_EQ(scan->frame_id, "lidar_front");
    EXPECT_GT(scan->points.front().range_m, 0.9f);
}

TEST(LidarSensor, ObstacleListGeneratedFromValidScan) {
    RawLidarPacket packet;
    packet.timestamp = now_sec();
    packet.bytes.resize(8 + (3 * sizeof(float) * 4));
    std::memcpy(packet.bytes.data(), "LDR1", 4);
    const uint16_t version = 1;
    const uint16_t point_count = 3;
    std::memcpy(packet.bytes.data() + 4, &version, sizeof(version));
    std::memcpy(packet.bytes.data() + 6, &point_count, sizeof(point_count));

    const std::array<float, 12> values{{
        1.0f,
        0.0f,
        0.0f,
        0.9f,
        1.3f,
        0.1f,
        0.0f,
        0.7f,
        1.6f,
        -0.1f,
        0.1f,
        0.5f,
    }};
    std::memcpy(packet.bytes.data() + 8, values.data(), values.size() * sizeof(float));

    auto parser = create_lidar_parser("generic_udp_cartesian_v1", "lidar_front", false);
    ASSERT_TRUE(parser != nullptr);
    const auto scan = parser->parse(packet);
    ASSERT_TRUE(scan.has_value());

    LidarMeasurement measurement;
    measurement.timestamp = scan->timestamp;
    measurement.points = scan->points;
    measurement.cloud = point_cloud_from_scan(*scan, 0.3f, 80.0f);
    measurement.num_points = static_cast<uint32_t>(measurement.cloud->size());

    const auto obstacles = drone::swarm::LeaderFollowerController::obstacles_from_lidar(
        measurement, Eigen::Vector3d(0.0, 0.0, 2.0), 1, 0.5f);
    EXPECT_EQ(obstacles.size(), 3u);
    EXPECT_GT(obstacles.front().position.x(), 0.5);
}

TEST(CameraSensor, ClassIdsMapToSemanticLabels) {
    const auto temp_path = std::filesystem::temp_directory_path() / "detector_labels_valid.json";
    {
        std::ofstream output(temp_path.string(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << R"({
  "labels": {
    "0": "person",
    "2": "car",
    "70": "drone"
  }
})";
    }

    const auto label_map = load_detector_label_map_json(temp_path.string());
    EXPECT_EQ(resolve_detector_label(0, label_map), "person");
    EXPECT_EQ(resolve_detector_label(2, label_map), "car");
    EXPECT_EQ(resolve_detector_label(70, label_map), "drone");

    std::filesystem::remove(temp_path);
}

TEST(CameraSensor, UnknownClassBecomesUnknownClassId) {
    const std::unordered_map<int, std::string> label_map{{0, "person"}};
    EXPECT_EQ(resolve_detector_label(99, label_map), "unknown_class_99");
}

//  SensorState string conversion â”€
TEST(SensorState, ToStringAllValues) {
    EXPECT_EQ(to_string(SensorState::UNINITIALIZED), "UNINITIALIZED");
    EXPECT_EQ(to_string(SensorState::RUNNING), "RUNNING");
    EXPECT_EQ(to_string(SensorState::DEGRADED), "DEGRADED");
    EXPECT_EQ(to_string(SensorState::FAILED), "FAILED");
    EXPECT_EQ(to_string(SensorState::DISCONNECTED), "DISCONNECTED");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
