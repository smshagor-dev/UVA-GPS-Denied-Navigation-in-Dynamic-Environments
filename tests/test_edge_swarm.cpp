#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <iostream>

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

EdgePeerPacket obstacle_digest(uint32_t sender_id, uint64_t timestamp_ms, uint32_t sequence_number) {
    EdgePeerPacket packet;
    packet.packet_type = EdgePacketType::OBSTACLE_DIGEST;
    packet.sender_id = sender_id;
    packet.timestamp_ms = timestamp_ms;
    packet.sequence_number = sequence_number;
    packet.trust_epoch = 1;
    packet.source = "simulation";
    packet.ttl_ms = 1000;
    packet.auth_hook = "unit-test-auth-hook";
    packet.obstacle_digest = ObstacleDigestPayload{4, 7, 75, "digest-a1"};
    return packet;
}

bool equivalent_header(const EdgePeerPacket& lhs, const EdgePeerPacket& rhs) {
    return lhs.packet_type == rhs.packet_type &&
        lhs.sender_id == rhs.sender_id &&
        lhs.timestamp_ms == rhs.timestamp_ms &&
        lhs.sequence_number == rhs.sequence_number &&
        lhs.trust_epoch == rhs.trust_epoch &&
        lhs.source == rhs.source &&
        lhs.ttl_ms == rhs.ttl_ms &&
        lhs.auth_hook == rhs.auth_hook;
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

TEST(EdgePeerProtocol, JsonEncodeDecodeGenericMode) {
    EdgeSerializationMetrics encode_metrics;
    const auto wire = serialize_edge_packet(
        heartbeat(7, 1000, 1),
        EdgeSerializationMode::JSON,
        &encode_metrics);
    EdgeSerializationMetrics decode_metrics;
    const auto parsed = parse_edge_packet(wire, EdgeSerializationMode::JSON, &decode_metrics);

    ASSERT_TRUE(parsed.ok) << parsed.error;
    EXPECT_EQ(parsed.packet.sender_id, 7u);
    EXPECT_EQ(encode_metrics.mode, EdgeSerializationMode::JSON);
    EXPECT_GT(encode_metrics.encoded_packet_size_bytes, 0u);
    EXPECT_GT(decode_metrics.deserialization_time_us, 0.0);
}

TEST(EdgePeerProtocol, CborEncodeDecodeHeartbeat) {
    EdgeSerializationMetrics metrics;
    const auto input = heartbeat(7, 1000, 1);
    const auto wire = serialize_edge_packet(input, EdgeSerializationMode::CBOR, &metrics);
    const auto parsed = parse_edge_packet_cbor(wire);

    ASSERT_TRUE(parsed.ok) << parsed.error;
    EXPECT_TRUE(equivalent_header(input, parsed.packet));
    ASSERT_TRUE(parsed.packet.heartbeat.has_value());
    EXPECT_EQ(parsed.packet.heartbeat->peer_count, 2);
    EXPECT_LT(wire.size(), serialize_edge_packet_json(input).size());
    EXPECT_EQ(metrics.mode, EdgeSerializationMode::CBOR);
    EXPECT_LT(metrics.compression_ratio_vs_json, 1.0);
}

TEST(EdgePeerProtocol, MalformedCborRejected) {
    const std::vector<uint8_t> malformed{0x8A, 0x01, 0x00, 0x07};
    const auto parsed = parse_edge_packet_cbor(malformed);
    EXPECT_FALSE(parsed.ok);
}

TEST(EdgePeerProtocol, CborPacketEquivalenceObstacleDigest) {
    const auto input = obstacle_digest(8, 2000, 3);
    const auto wire = serialize_edge_packet_cbor(input);
    const auto parsed = parse_edge_packet_cbor(wire);

    ASSERT_TRUE(parsed.ok) << parsed.error;
    EXPECT_TRUE(equivalent_header(input, parsed.packet));
    ASSERT_TRUE(parsed.packet.obstacle_digest.has_value());
    EXPECT_EQ(parsed.packet.obstacle_digest->local_obstacle_count, 4);
    EXPECT_EQ(parsed.packet.obstacle_digest->digest_id, "digest-a1");
}

TEST(EdgePeerProtocol, SerializationModeSwitching) {
    const auto packet = consensus(9, 3000, 4, 12, ConsensusProposalType::EMERGENCY_REROUTE);
    const auto json_wire = serialize_edge_packet(packet, EdgeSerializationMode::JSON);
    const auto cbor_wire = serialize_edge_packet(packet, EdgeSerializationMode::CBOR);

    ASSERT_TRUE(parse_edge_packet(json_wire, EdgeSerializationMode::JSON).ok);
    ASSERT_TRUE(parse_edge_packet(cbor_wire, EdgeSerializationMode::CBOR).ok);
    EXPECT_NE(json_wire.size(), cbor_wire.size());
    EXPECT_EQ(parse_edge_serialization_mode("protobuf_placeholder"), EdgeSerializationMode::PROTOBUF_PLACEHOLDER);
}

TEST(EdgePeerProtocol, CborBoundedPacketEnforcement) {
    const auto packet = obstacle_digest(8, 2000, 3);
    const auto wire = serialize_edge_packet_cbor(packet);
    const auto validation = validate_edge_packet(
        packet,
        EdgePacketValidationOptions{2000, 0, false, wire.size() - 1, wire.size()});
    EXPECT_FALSE(validation.ok);
    EXPECT_EQ(validation.error, "packet exceeds max size");
}

TEST(EdgePeerProtocol, CborStalePacketRejectedByCache) {
    SwarmStateCache cache;
    const auto first = parse_edge_packet_cbor(serialize_edge_packet_cbor(heartbeat(7, 1000, 10)));
    ASSERT_TRUE(first.ok);
    ASSERT_TRUE(cache.observe_packet(first.packet, 1000).accepted);

    const auto stale = parse_edge_packet_cbor(serialize_edge_packet_cbor(heartbeat(7, 1100, 10)));
    ASSERT_TRUE(stale.ok);
    const auto observed = cache.observe_packet(stale.packet, 1100);
    EXPECT_FALSE(observed.accepted);
    EXPECT_EQ(observed.reason, "stale sequence number");
}

TEST(EdgePeerProtocol, MalformedPacketRejected) {
    const auto parsed = parse_edge_packet_json("\"packet_type\":\"heartbeat\"");
    EXPECT_FALSE(parsed.ok);
    EXPECT_EQ(parsed.error, "malformed packet");
}

TEST(EdgePeerProtocol, JsonVsCborBenchmarkSamples) {
    const std::array<EdgePeerPacket, 3> packets{
        heartbeat(7, 1000, 1),
        obstacle_digest(8, 1000, 1),
        consensus(9, 1000, 1, 3, ConsensusProposalType::COLLECTIVE_HALT),
    };

    for (const auto& packet : packets) {
        EdgeSerializationMetrics json_metrics;
        EdgeSerializationMetrics cbor_metrics;
        const auto json_wire = serialize_edge_packet(packet, EdgeSerializationMode::JSON, &json_metrics);
        const auto cbor_wire = serialize_edge_packet(packet, EdgeSerializationMode::CBOR, &cbor_metrics);

        ASSERT_TRUE(parse_edge_packet(json_wire, EdgeSerializationMode::JSON).ok);
        ASSERT_TRUE(parse_edge_packet(cbor_wire, EdgeSerializationMode::CBOR).ok);
        EXPECT_LT(cbor_wire.size(), json_wire.size());
        std::cout << "[edge_swarm serialization benchmark] type=" << to_string(packet.packet_type)
                  << " json_bytes=" << json_wire.size()
                  << " cbor_bytes=" << cbor_wire.size()
                  << " cbor_vs_json=" << cbor_metrics.compression_ratio_vs_json
                  << " encode_us=" << cbor_metrics.serialization_time_us << "\n";
    }
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
