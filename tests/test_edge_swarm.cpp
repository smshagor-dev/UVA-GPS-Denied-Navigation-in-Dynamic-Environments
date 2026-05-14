#include <gtest/gtest.h>

#include "swarm/EdgeConsensusManager.hpp"
#include "swarm/EdgePeerProtocol.hpp"
#include "swarm/SwarmStateCache.hpp"

using namespace drone::swarm;

namespace {

EdgePeerPacket heartbeat(uint32_t sender_id, uint64_t timestamp_ms, uint32_t sequence_number) {
    EdgePeerPacket packet;
    packet.packet_type = EdgePacketType::HEARTBEAT;
    packet.sender_id = sender_id;
    packet.timestamp_ms = timestamp_ms;
    packet.sequence_number = sequence_number;
    packet.trust_epoch = 1;
    packet.source = "real";
    packet.ttl_ms = 1000;
    packet.auth_hook = "unit-test-auth-hook";
    packet.heartbeat = HeartbeatPayload{
        2,
        0,
        88.0,
        0.96,
        0.84,
        false,
        "nominal",
        "distributed_autonomy",
    };
    return packet;
}

EdgePeerPacket consensus(uint32_t sender_id,
                         uint64_t timestamp_ms,
                         uint32_t sequence_number,
                         uint64_t epoch,
                         ConsensusProposalType proposal) {
    EdgePeerPacket packet;
    packet.packet_type = EdgePacketType::CONSENSUS_STATE;
    packet.sender_id = sender_id;
    packet.timestamp_ms = timestamp_ms;
    packet.sequence_number = sequence_number;
    packet.trust_epoch = 1;
    packet.source = "real";
    packet.ttl_ms = 1000;
    packet.auth_hook = "unit-test-auth-hook";
    packet.consensus_state = ConsensusStatePayload{
        proposal,
        "quorum_pending",
        epoch,
        2,
        false,
    };
    return packet;
}

EdgePeerPacket emergency_corridor(uint32_t sender_id, uint64_t timestamp_ms, uint32_t sequence_number) {
    EdgePeerPacket packet;
    packet.packet_type = EdgePacketType::EMERGENCY_CORRIDOR;
    packet.sender_id = sender_id;
    packet.timestamp_ms = timestamp_ms;
    packet.sequence_number = sequence_number;
    packet.trust_epoch = 1;
    packet.source = "real";
    packet.ttl_ms = 1000;
    packet.auth_hook = "unit-test-auth-hook";
    packet.emergency_corridor = EmergencyCorridorPayload{
        Eigen::Vector3d{1.0, 2.0, 3.0},
        2.5,
        1000,
        "unit test emergency corridor",
    };
    return packet;
}

} // namespace

TEST(EdgePeerProtocol, ValidHeartbeatParse) {
    const auto wire = serialize_edge_packet_json(heartbeat(7, 1000, 1));
    const auto parsed = parse_edge_packet_json(wire);

    ASSERT_TRUE(parsed.ok) << parsed.error;
    EXPECT_EQ(parsed.packet.packet_type, EdgePacketType::HEARTBEAT);
    EXPECT_EQ(parsed.packet.sender_id, 7u);
    ASSERT_TRUE(parsed.packet.heartbeat.has_value());
    EXPECT_EQ(parsed.packet.source, "real");
}

TEST(EdgePeerProtocol, MalformedPacketRejected) {
    const auto parsed = parse_edge_packet_json("\"packet_type\":\"heartbeat\"");
    EXPECT_FALSE(parsed.ok);
    EXPECT_EQ(parsed.error, "malformed packet");
}

TEST(EdgePeerProtocol, UnknownPacketTypeRejected) {
    const auto parsed = parse_edge_packet_json(
        "{\"packet_type\":\"mystery\",\"sender_id\":7,\"timestamp_ms\":1000,"
        "\"sequence_number\":1,\"trust_epoch\":1,\"source\":\"real\","
        "\"ttl_ms\":1000,\"payload\":{}}");
    EXPECT_FALSE(parsed.ok);
    EXPECT_EQ(parsed.error, "unknown packet_type");
}

TEST(EdgePeerProtocol, ExpiredPacketRejected) {
    const auto result = validate_edge_packet(
        heartbeat(7, 1000, 1),
        EdgePacketValidationOptions{2501, 0, false, 1400});
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.error, "packet expired");
}

TEST(EdgePeerProtocol, OversizedPacketRejected) {
    auto packet = heartbeat(7, 1000, 1);
    packet.auth_hook = std::string(2048, 'x');
    const auto result = validate_edge_packet(packet, EdgePacketValidationOptions{1000, 0, false, 256});
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.error, "packet exceeds max size");
}

TEST(EdgePeerProtocol, StaleSequenceRejected) {
    SwarmStateCache cache;
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 10), 1000).accepted);

    const auto observed = cache.observe_packet(heartbeat(7, 1100, 10), 1100);
    EXPECT_FALSE(observed.accepted);
    EXPECT_EQ(observed.reason, "stale sequence number");
}

TEST(SwarmStateCache, PeerBecomesStaleAfterTimeout) {
    SwarmStateCache cache({8, 100, 5000});
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 1), 1000).accepted);
    EXPECT_EQ(cache.stale_peer_count(), 0u);

    cache.expire(1150);
    EXPECT_EQ(cache.peer_count(), 1u);
    EXPECT_EQ(cache.stale_peer_count(), 1u);
    EXPECT_EQ(cache.safety_eligible_peer_count(), 0u);
    EXPECT_TRUE(cache.split_swarm_isolated());
}

TEST(EdgeConsensusManager, StalePeerExcludedFromConsensus) {
    SwarmStateCache cache({8, 100, 5000});
    EdgeConsensusManager manager(1, 2);
    manager.propose(ConsensusProposalType::COLLECTIVE_HALT, 10, 2);

    ASSERT_TRUE(cache.observe_packet(heartbeat(2, 1000, 1), 1000).accepted);
    const auto peer_vote = consensus(2, 1050, 2, 10, ConsensusProposalType::COLLECTIVE_HALT);
    ASSERT_TRUE(cache.observe_packet(peer_vote, 1050).accepted);
    manager.observe_packet(peer_vote, cache);
    EXPECT_TRUE(manager.should_apply_collective_action());

    cache.expire(1200);
    manager.expire(cache);
    EXPECT_FALSE(manager.should_apply_collective_action());
    EXPECT_EQ(manager.snapshot(cache).state, "split_swarm_recovery");
}

TEST(EdgeConsensusManager, EmergencyPacketAcceptedDuringDegradedState) {
    EdgeConsensusManager manager(1, 2);
    manager.set_local_safety_override(true);

    const auto packet = emergency_corridor(2, 2000, 1);
    const auto validation = validate_edge_packet(packet, EdgePacketValidationOptions{2000, 0, false, 1400});
    ASSERT_TRUE(validation.ok) << validation.error;
    EXPECT_TRUE(manager.should_accept_emergency_packet(packet));
}

TEST(EdgeConsensusManager, LocalSafetyPriorityOverConsensus) {
    EdgeConsensusManager manager(1, 1);
    manager.propose(ConsensusProposalType::EMERGENCY_REROUTE, 4, 1);
    ASSERT_TRUE(manager.should_apply_collective_action());

    manager.set_local_safety_override(true);
    EXPECT_FALSE(manager.should_apply_collective_action());
}
