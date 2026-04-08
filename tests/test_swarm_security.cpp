// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include <gtest/gtest.h>

#include "sensors/MotorSensor.hpp"
#include "swarm/SwarmSecurity.hpp"
#include "swarm/V2XMeshNetwork.hpp"

using namespace drone::swarm;

namespace {

SwarmMessage make_message(uint32_t src, uint32_t seq, SwarmMessage::Type type = SwarmMessage::Type::MISSION_SYNC) {
    SwarmMessage msg;
    msg.src_id = src;
    msg.dst_id = 0xFFFFFFFF;
    msg.seq_num = seq;
    msg.timestamp = 1234.0 + static_cast<double>(seq);
    msg.type = type;
    msg.payload = {0x10, 0x20, 0x30, static_cast<uint8_t>(seq & 0xFF)};
    msg.payload_len = static_cast<uint16_t>(msg.payload.size());
    return msg;
}

SwarmSecurityConfig secure_cfg() {
    SwarmSecurityConfig cfg;
    cfg.enabled = true;
    cfg.swarm_secret = "unit-test-swarm-secret";
    cfg.pbkdf2_iterations = 2000;
    return cfg;
}

} // namespace

TEST(SwarmSecurity, SecureRoundTripWorks) {
    SwarmSecurityContext sender(1, secure_cfg());
    SwarmSecurityContext receiver(2, secure_cfg());

    const auto frame = sender.seal(make_message(1, 1));
    const auto opened = receiver.open(frame.data(), frame.size());

    ASSERT_TRUE(opened.has_value());
    EXPECT_EQ(opened->src_id, 1u);
    EXPECT_EQ(opened->seq_num, 1u);
    EXPECT_EQ(opened->type, SwarmMessage::Type::MISSION_SYNC);
}

TEST(SwarmSecurity, TamperedFrameIsRejected) {
    SwarmSecurityContext sender(1, secure_cfg());
    SwarmSecurityContext receiver(2, secure_cfg());

    auto frame = sender.seal(make_message(1, 2));
    frame.back() ^= 0x7A;

    const auto opened = receiver.open(frame.data(), frame.size());
    EXPECT_FALSE(opened.has_value());
}

TEST(SwarmSecurity, ReplayIsRejected) {
    SwarmSecurityContext sender(1, secure_cfg());
    SwarmSecurityContext receiver(2, secure_cfg());

    const auto frame = sender.seal(make_message(1, 3));
    ASSERT_TRUE(receiver.open(frame.data(), frame.size()).has_value());
    EXPECT_FALSE(receiver.open(frame.data(), frame.size()).has_value());
}

TEST(SwarmSecurity, WrongSecretIsRejected) {
    SwarmSecurityContext sender(1, secure_cfg());
    auto bad_cfg = secure_cfg();
    bad_cfg.swarm_secret = "wrong-secret";
    SwarmSecurityContext receiver(2, std::move(bad_cfg));

    const auto frame = sender.seal(make_message(1, 4));
    EXPECT_FALSE(receiver.open(frame.data(), frame.size()).has_value());
}

TEST(SwarmSecurity, LedgerChainRejectsBrokenLeaderCommandHistory) {
    SwarmSecurityContext sender(1, secure_cfg());
    SwarmSecurityContext receiver(2, secure_cfg());

    const auto first = sender.seal(make_message(1, 5, SwarmMessage::Type::FORMATION_CMD));
    ASSERT_TRUE(receiver.open(first.data(), first.size()).has_value());

    SwarmSecurityContext restarted_sender(1, secure_cfg());
    const auto second = restarted_sender.seal(make_message(1, 6, SwarmMessage::Type::FORMATION_CMD));
    EXPECT_FALSE(receiver.open(second.data(), second.size()).has_value());
}

TEST(V2XElection, LeadershipScoreRewardsHealthyCandidates) {
    SwarmHealthMetrics strong;
    strong.battery_pct = 92.0f;
    strong.motor_health = 0.96f;
    strong.link_quality = 0.90f;
    strong.cpu_headroom = 0.75f;
    strong.thermal_headroom = 0.82f;

    SwarmHealthMetrics weak;
    weak.battery_pct = 28.0f;
    weak.motor_health = 0.41f;
    weak.link_quality = 0.33f;
    weak.cpu_headroom = 0.36f;
    weak.thermal_headroom = 0.40f;

    EXPECT_GT(V2XMeshNetwork::compute_leadership_score(strong),
              V2XMeshNetwork::compute_leadership_score(weak));
}

TEST(V2XElection, EmergencyFaultZeroesLeadershipScore) {
    SwarmHealthMetrics faulted;
    faulted.battery_pct = 90.0f;
    faulted.motor_health = 0.95f;
    faulted.link_quality = 0.9f;
    faulted.cpu_headroom = 0.7f;
    faulted.thermal_headroom = 0.8f;
    faulted.emergency_fault = true;

    EXPECT_FLOAT_EQ(V2XMeshNetwork::compute_leadership_score(faulted), 0.0f);
}

TEST(MotorSensor, ProducesHealthMeasurement) {
    drone::sensors::MotorSensor sensor("motor-test");
    ASSERT_TRUE(sensor.initialize());
    sensor.poll();
    const auto measurement = sensor.latest();
    ASSERT_TRUE(measurement.has_value());
    EXPECT_GT(measurement->average_health, 0.0f);
    EXPECT_LE(measurement->average_health, 1.0f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
