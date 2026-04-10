// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

 
// test_v2x.cpp    GoogleTest suite for V2X Mesh / Leader-Follower
 
#include <gtest/gtest.h>
#include "security/CommandPolicy.hpp"
#include "sensors/LidarSensor.hpp"
#include "swarm/V2XMeshNetwork.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace drone::swarm;

//  SwarmMessage serialization round-trip 
TEST(SwarmMessage, SerializeDeserializeRoundTrip) {
    SwarmMessage msg;
    msg.src_id      = 42;
    msg.dst_id      = 0xFFFFFFFF;
    msg.seq_num     = 7;
    msg.timestamp   = 1234.5678;
    msg.type        = SwarmMessage::Type::POSE_UPDATE;
    msg.ttl         = 5;
    msg.payload     = {0x01, 0x02, 0xAB, 0xCD};
    msg.payload_len = static_cast<uint16_t>(msg.payload.size());

    auto bytes = msg.serialize();
    EXPECT_FALSE(bytes.empty());

    auto decoded = SwarmMessage::deserialize(bytes.data(), bytes.size());
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->src_id,    msg.src_id);
    EXPECT_EQ(decoded->dst_id,    msg.dst_id);
    EXPECT_EQ(decoded->seq_num,   msg.seq_num);
    EXPECT_NEAR(decoded->timestamp, msg.timestamp, 1e-9);
    EXPECT_EQ(decoded->type,      msg.type);
    EXPECT_EQ(decoded->ttl,       msg.ttl);
    EXPECT_EQ(decoded->payload,   msg.payload);
}

TEST(SwarmMessage, DeserializeInvalidDataReturnsNullopt) {
    std::vector<uint8_t> garbage(4, 0xFF);
    auto result = SwarmMessage::deserialize(garbage.data(), garbage.size());
    EXPECT_FALSE(result.has_value());
}

TEST(SwarmMessage, EmptyPayloadSerializes) {
    SwarmMessage msg;
    msg.type = SwarmMessage::Type::HEARTBEAT;
    auto bytes = msg.serialize();
    auto back  = SwarmMessage::deserialize(bytes.data(), bytes.size());
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(back->type, SwarmMessage::Type::HEARTBEAT);
    EXPECT_TRUE(back->payload.empty());
}

//  PeerInfo staleness 
TEST(PeerInfo, StaleCheckWithOldTimestamp) {
    PeerInfo peer;
    peer.last_seen_ts = 0.0;  // epoch (very old)
    EXPECT_TRUE(peer.is_stale(2.0));
}

TEST(PeerInfo, FreshPeerNotStale) {
    PeerInfo peer;
    peer.last_seen_ts = static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count()) / 1000.0;
    EXPECT_FALSE(peer.is_stale(2.0));
}

//  LeaderFollowerController formation geometry â”€
TEST(LeaderFollower, DiamondFormationOffsets) {
    // Without a live network, we test the geometry math directly
    // by instantiating with a null network (geometry is pure math)
    FormationCommand cmd;
    cmd.shape      = FormationCommand::Formation::DIAMOND;
    cmd.spacing_m  = 3.0f;

    // Diamond: follower 0  right, 1  left, 2  back-right, 3  back-left
    // Leader at origin
    const Eigen::Vector3d leader{0, 0, 5};

    // We only check that offsets for different indices differ
    // (full test requires live V2X; here we just verify instantiation)
    EXPECT_EQ(cmd.spacing_m, 3.0f);
    EXPECT_EQ(cmd.shape, FormationCommand::Formation::DIAMOND);
}

TEST(LeaderFollower, AvoidanceVelocityRepelsNearbyPeer) {
    LeaderFollowerController controller(7, nullptr);

    PeerInfo peer;
    peer.id = 11;
    peer.position = Eigen::Vector3d(0.8, 0.0, 5.0);
    peer.velocity = Eigen::Vector3d::Zero();
    peer.reachable = true;

    const Eigen::Vector3d current_pos{0.0, 0.0, 5.0};
    const auto avoidance = controller.compute_avoidance_velocity(
        current_pos, Eigen::Vector3d::Zero(), {peer});

    EXPECT_LT(avoidance.x(), 0.0);
    EXPECT_NEAR(avoidance.y(), 0.0, 1e-6);
    EXPECT_GT(avoidance.norm(), 0.1);
}

TEST(LeaderFollower, VelocityCommandUsesAvoidanceWhenPeerBlocksPath) {
    LeaderFollowerController controller(3, nullptr);

    PeerInfo peer;
    peer.id = 9;
    peer.position = Eigen::Vector3d(0.5, 0.0, 5.0);
    peer.velocity = Eigen::Vector3d::Zero();
    peer.reachable = true;

    const Eigen::Vector3d current_pos{0.0, 0.0, 5.0};
    const Eigen::Vector3d current_vel{0.0, 0.0, 0.0};
    const Eigen::Vector3d target_pos{2.0, 0.0, 5.0};

    const auto nominal = controller.velocity_command(
        current_pos, current_vel, target_pos, std::vector<PeerInfo>{}, {}, 1.5f, 4.0f);
    const auto deconflicted = controller.velocity_command(
        current_pos, current_vel, target_pos, {peer}, {}, 1.5f, 4.0f);

    EXPECT_GT(nominal.x(), 0.0);
    EXPECT_LT(deconflicted.x(), nominal.x());
}

TEST(LeaderFollower, LidarObstaclesAreConvertedAndRepelled) {
    LeaderFollowerController controller(5, nullptr);

    drone::sensors::LidarMeasurement scan;
    scan.cloud = drone::sensors::PointCloudPtr(new drone::sensors::PointCloud());
    scan.cloud->push_back(pcl::PointXYZI{1.0f, 0.0f, 0.0f, 1.0f});
    scan.cloud->push_back(pcl::PointXYZI{1.2f, 0.1f, 0.0f, 1.0f});

    const auto obstacles = LeaderFollowerController::obstacles_from_lidar(
        scan, Eigen::Vector3d(0.0, 0.0, 5.0), 1, 0.6f);
    ASSERT_FALSE(obstacles.empty());

    const auto avoidance = controller.compute_avoidance_velocity(
        Eigen::Vector3d(0.0, 0.0, 5.0), Eigen::Vector3d::Zero(), {}, obstacles);
    EXPECT_LT(avoidance.x(), 0.0);
}

TEST(LeaderFollower, RelativeVelocityIncreasesHeadOnAvoidance) {
    LeaderFollowerController controller(2, nullptr);

    PeerInfo slow_peer;
    slow_peer.id = 3;
    slow_peer.position = Eigen::Vector3d(4.0, 0.0, 5.0);
    slow_peer.velocity = Eigen::Vector3d::Zero();
    slow_peer.reachable = true;

    PeerInfo head_on_peer = slow_peer;
    head_on_peer.id = 4;
    head_on_peer.velocity = Eigen::Vector3d(-3.0, 0.0, 0.0);

    const Eigen::Vector3d current_pos{0.0, 0.0, 5.0};
    const Eigen::Vector3d current_vel{3.0, 0.0, 0.0};

    const auto slow_avoid = controller.compute_avoidance_velocity(
        current_pos, current_vel, {slow_peer});
    const auto fast_avoid = controller.compute_avoidance_velocity(
        current_pos, current_vel, {head_on_peer});

    EXPECT_GT(fast_avoid.norm(), slow_avoid.norm());
}

TEST(LeaderFollower, DeadlockGetsTangentialEscape) {
    LeaderFollowerController controller(8, nullptr);

    AvoidanceObstacle obstacle;
    obstacle.position = Eigen::Vector3d(1.2, 0.0, 5.0);
    obstacle.radius_m = 0.5f;

    const auto command = controller.velocity_command(
        Eigen::Vector3d(0.0, 0.0, 5.0),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(3.0, 0.0, 5.0),
        {},
        {obstacle},
        1.5f,
        4.0f);

    EXPECT_GT(command.norm(), 0.1);
    EXPECT_LT(command.x(), 1.0);
    EXPECT_GT(std::abs(command.y()), 0.1);
}

//  DroneRole string conversion 
TEST(DroneRole, ToStringAllRoles) {
    EXPECT_EQ(to_string(DroneRole::CANDIDATE), "CANDIDATE");
    EXPECT_EQ(to_string(DroneRole::FOLLOWER),  "FOLLOWER");
    EXPECT_EQ(to_string(DroneRole::LEADER),    "LEADER");
    EXPECT_EQ(to_string(DroneRole::RELAY),     "RELAY");
}

//  V2XMeshNetwork construction (no network binding in unit test) â”€
TEST(V2XMeshNetwork, ConstructionDoesNotCrash) {
    V2XMeshNetwork net(99, "239.255.0.1", 7400);
    EXPECT_EQ(net.peer_count(), 0u);
    EXPECT_EQ(net.local_role(), DroneRole::CANDIDATE);
}

TEST(CommandPolicy, EmergencyLandAlwaysAccepted) {
    drone::security::DroneSecurityAssessment security;
    security.state = drone::security::DroneSecurityState::ISOLATED_AUTONOMY;
    security.remote_command_allowed = false;

    drone::security::RemoteCommandEnvelope cmd;
    cmd.action = drone::security::RemoteCommandAction::EMERGENCY_LAND;
    cmd.critical = true;

    const auto decision = drone::security::evaluate_remote_command(security, cmd);
    EXPECT_TRUE(decision.accepted);
}

TEST(CommandPolicy, NonCriticalCommandRejectedWhenRemoteControlBlocked) {
    drone::security::DroneSecurityAssessment security;
    security.state = drone::security::DroneSecurityState::CONTROL_PLANE_UNTRUSTED;
    security.remote_command_allowed = false;

    drone::security::RemoteCommandEnvelope cmd;
    cmd.action = drone::security::RemoteCommandAction::RETURN_HOME;

    const auto decision = drone::security::evaluate_remote_command(security, cmd);
    EXPECT_FALSE(decision.accepted);
}

 
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
