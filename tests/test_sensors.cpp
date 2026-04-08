// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// test_sensors.cpp  â€”  GoogleTest suite for sensor subsystem
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#include <gtest/gtest.h>
#include "sensors/SensorBase.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/LidarSensor.hpp"

using namespace drone::sensors;

// â”€â”€â”€ Minimal concrete sensor for testing the base class â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
class MockSensor : public SensorBase {
public:
    explicit MockSensor(std::string id)
        : SensorBase(std::move(id), "MOCK") {}

    bool initialize() override {
        poll_rate_hz_ = 100;
        set_state(SensorState::RUNNING);
        return true;
    }

    bool reconfigure(const std::string&) override { return true; }

    void poll() override { ++poll_calls_; }

    int poll_calls_{0};
};

// â”€â”€â”€ SensorBase lifecycle â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
        bool initialize() override { return true; }
        bool reconfigure(const std::string&) override { return true; }
        void poll() override { report_error("test error"); }
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

// â”€â”€â”€ IMUSensor construction â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

// â”€â”€â”€ LidarSensor construction â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
TEST(LidarSensor, ConstructAndState) {
    LidarSensor lidar("lidar0", "127.0.0.1:2368");
    EXPECT_EQ(lidar.sensor_type(), "LiDAR");
    EXPECT_FALSE(lidar.latest().has_value());
}

// â”€â”€â”€ SensorState string conversion â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
TEST(SensorState, ToStringAllValues) {
    EXPECT_EQ(to_string(SensorState::UNINITIALIZED), "UNINITIALIZED");
    EXPECT_EQ(to_string(SensorState::RUNNING),       "RUNNING");
    EXPECT_EQ(to_string(SensorState::DEGRADED),      "DEGRADED");
    EXPECT_EQ(to_string(SensorState::FAILED),        "FAILED");
    EXPECT_EQ(to_string(SensorState::DISCONNECTED),  "DISCONNECTED");
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
