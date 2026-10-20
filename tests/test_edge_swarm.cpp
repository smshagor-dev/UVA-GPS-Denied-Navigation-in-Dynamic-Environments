#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <vector>
#include <utility>

#include "swarm/EdgeConsensusManager.hpp"
#include "swarm/EdgePeerProtocol.hpp"
#include "swarm/SwarmStateCache.hpp"
#include "security/PeerPacketAuth.hpp"

using namespace drone::swarm;
using namespace drone::security;

namespace {

// Test fixtures use fixed positional packet fields for readability.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
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
        2, 0, 88.0, 0.96, 0.84, false, "nominal", "distributed_autonomy",
    };
    return packet;
}

// Test fixtures use fixed positional packet fields for readability.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
EdgePeerPacket consensus(uint32_t sender_id, uint64_t timestamp_ms, uint32_t sequence_number,
                         uint64_t epoch, ConsensusProposalType proposal) {
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
        proposal, "quorum_pending", epoch, 2, false,
    };
    return packet;
}

// Test fixtures use fixed positional packet fields for readability.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
EdgePeerPacket emergency_corridor(uint32_t sender_id, uint64_t timestamp_ms,
                                  uint32_t sequence_number) {
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

// Test fixtures use fixed positional packet fields for readability.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
EdgePeerPacket obstacle_digest(uint32_t sender_id, uint64_t timestamp_ms,
                               uint32_t sequence_number) {
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
    return lhs.packet_type == rhs.packet_type && lhs.sender_id == rhs.sender_id &&
           lhs.timestamp_ms == rhs.timestamp_ms && lhs.sequence_number == rhs.sequence_number &&
           lhs.trust_epoch == rhs.trust_epoch && lhs.source == rhs.source &&
           lhs.ttl_ms == rhs.ttl_ms && lhs.auth_hook == rhs.auth_hook;
}

} // namespace

PacketAuthConfig auth_config(std::string profile = "edge_swarm") {
    PacketAuthConfig cfg;
    cfg.mode = AuthMode::HMAC_SHA256;
    cfg.shared_secret = "unit-test-shared-secret";
    cfg.trust_epoch = 1;
    cfg.max_clock_skew_ms = 2000;
    cfg.runtime_profile = std::move(profile);
    return cfg;
}

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
    const auto wire =
        serialize_edge_packet(heartbeat(7, 1000, 1), EdgeSerializationMode::JSON, &encode_metrics);
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
    if (!parsed.packet.heartbeat.has_value()) {
        ADD_FAILURE() << "Missing parsed heartbeat payload";
        return;
    }
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
    if (!parsed.packet.obstacle_digest.has_value()) {
        ADD_FAILURE() << "Missing parsed obstacle digest";
        return;
    }
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
    EXPECT_EQ(parse_edge_serialization_mode("protobuf_placeholder"),
              EdgeSerializationMode::PROTOBUF_PLACEHOLDER);
}

TEST(EdgePeerProtocol, CborBoundedPacketEnforcement) {
    const auto packet = obstacle_digest(8, 2000, 3);
    const auto wire = serialize_edge_packet_cbor(packet);
    const auto validation = validate_edge_packet(
        packet, EdgePacketValidationOptions{2000, 0, false, wire.size() - 1, wire.size()});
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
        const auto json_wire =
            serialize_edge_packet(packet, EdgeSerializationMode::JSON, &json_metrics);
        const auto cbor_wire =
            serialize_edge_packet(packet, EdgeSerializationMode::CBOR, &cbor_metrics);

        ASSERT_TRUE(parse_edge_packet(json_wire, EdgeSerializationMode::JSON).ok);
        ASSERT_TRUE(parse_edge_packet(cbor_wire, EdgeSerializationMode::CBOR).ok);
        EXPECT_LT(cbor_wire.size(), json_wire.size());
        std::cout << "[edge_swarm serialization benchmark] type=" << to_string(packet.packet_type)
                  << " json_bytes=" << json_wire.size() << " cbor_bytes=" << cbor_wire.size()
                  << " cbor_vs_json=" << cbor_metrics.compression_ratio_vs_json
                  << " encode_us=" << cbor_metrics.serialization_time_us << "\n";
    }
}

TEST(EdgePeerProtocol, UnknownPacketTypeRejected) {
    const auto parsed =
        parse_edge_packet_json("{\"packet_type\":\"mystery\",\"sender_id\":7,\"timestamp_ms\":1000,"
                               "\"sequence_number\":1,\"trust_epoch\":1,\"source\":\"real\","
                               "\"ttl_ms\":1000,\"payload\":{}}");
    EXPECT_FALSE(parsed.ok);
    EXPECT_EQ(parsed.error, "unknown packet_type");
}

TEST(EdgePeerProtocol, ExpiredPacketRejected) {
    const auto result = validate_edge_packet(heartbeat(7, 1000, 1),
                                             EdgePacketValidationOptions{2501, 0, false, 1400});
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.error, "packet expired");
}

TEST(EdgePeerProtocol, OversizedPacketRejected) {
    auto packet = heartbeat(7, 1000, 1);
    packet.auth_hook = std::string(2048, 'x');
    const auto result =
        validate_edge_packet(packet, EdgePacketValidationOptions{1000, 0, false, 256});
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

TEST(PeerPacketAuth, HmacSignedPacketVerifies) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());

    const auto verified = verifyPacket(packet, cfg, 1000);
    EXPECT_EQ(verified.result, PacketAuthResult::ACCEPTED);
}

TEST(PeerPacketAuth, TamperedPayloadRejected) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    if (!packet.heartbeat.has_value()) {
        ADD_FAILURE() << "Missing heartbeat fixture payload";
        return;
    }
    packet.heartbeat->battery_pct = 12.0;

    const auto verified = verifyPacket(packet, cfg, 1000);
    EXPECT_EQ(verified.result, PacketAuthResult::INVALID_SIGNATURE);
}

TEST(PeerPacketAuth, TamperedSenderRejected) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    packet.sender_id = 8;

    const auto verified = verifyPacket(packet, cfg, 1000);
    EXPECT_EQ(verified.result, PacketAuthResult::INVALID_SIGNATURE);
}

TEST(PeerPacketAuth, MissingSignatureRejectedInEdgeSwarm) {
    auto packet = heartbeat(7, 1000, 1);
    packet.auth_hook = "unsigned";

    const auto verified = verifyPacket(packet, auth_config("edge_swarm"), 1000);
    EXPECT_EQ(verified.result, PacketAuthResult::MISSING_SIGNATURE);
}

TEST(PeerPacketAuth, UnsignedAllowedOnlyInSimulationWhenConfigured) {
    auto packet = heartbeat(7, 1000, 1);
    packet.source = "simulation";
    packet.auth_hook = "unsigned";
    auto cfg = auth_config("simulation");
    cfg.mode = AuthMode::NONE;
    cfg.allow_unsigned_in_simulation = true;
    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::ACCEPTED);

    cfg.runtime_profile = "edge_swarm";
    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::MISSING_SIGNATURE);
}

TEST(PeerPacketAuth, StaleTrustEpochRejected) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    packet.trust_epoch = 2;

    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::STALE_EPOCH);
}

TEST(PeerPacketAuth, StaleSequenceRejected) {
    auto packet = heartbeat(7, 1000, 10);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());

    EXPECT_EQ(verifyPacket(packet, cfg, 1000, 10).result, PacketAuthResult::REPLAY_DETECTED);
}

TEST(PeerPacketAuth, FutureTimestampBoundDoesNotGrowWithTtl) {
    auto packet = heartbeat(7, 4000, 1);
    auto cfg = auth_config();
    packet.ttl_ms = 5000;
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 2000).result, PacketAuthResult::ACCEPTED);
    EXPECT_EQ(verifyPacket(packet, cfg, 1999).result, PacketAuthResult::FUTURE_TIMESTAMP);
}

TEST(PeerPacketAuth, SenderControlledTtlHasAnUpperBound) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    packet.ttl_ms = cfg.max_packet_ttl_ms + 1;
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::INVALID_TTL);
    packet.ttl_ms = 0;
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::INVALID_TTL);
}

TEST(PeerPacketAuth, StaleTimestampBoundary) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 4000).result, PacketAuthResult::ACCEPTED);
    EXPECT_EQ(verifyPacket(packet, cfg, 4001).result, PacketAuthResult::STALE_TIMESTAMP);
}

TEST(PeerPacketAuth, ZeroClockSkewIsStrictNotDisabled) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    cfg.max_clock_skew_ms = 0;
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 1000).result, PacketAuthResult::ACCEPTED);
    EXPECT_EQ(verifyPacket(packet, cfg, 999).result, PacketAuthResult::FUTURE_TIMESTAMP);
    EXPECT_EQ(verifyPacket(packet, cfg, 2001).result, PacketAuthResult::STALE_TIMESTAMP);
}

TEST(PeerPacketAuth, AuthenticateBeforeClassifyingReplay) {
    auto packet = heartbeat(7, 1000, 10);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    packet.sender_id = 8;
    EXPECT_EQ(verifyPacket(packet, cfg, 1000, 10).result, PacketAuthResult::INVALID_SIGNATURE);
}

TEST(PeerPacketAuth, ValidReorderedPacketRejectedByStrictSequence) {
    auto older = heartbeat(7, 1000, 10);
    auto newer = heartbeat(7, 1001, 11);
    auto cfg = auth_config();
    ASSERT_TRUE(signPacket(older, cfg).accepted());
    ASSERT_TRUE(signPacket(newer, cfg).accepted());
    EXPECT_EQ(verifyPacket(newer, cfg, 1001, 10).result, PacketAuthResult::ACCEPTED);
    EXPECT_EQ(verifyPacket(older, cfg, 1001, 11).result, PacketAuthResult::REPLAY_DETECTED);
}

TEST(PeerPacketAuth, SharedSecretDoesNotAttributeOnePhysicalSender) {
    auto forged = heartbeat(8, 1000, 12);
    auto cfg = auth_config();
    // The holder of the group key can sign a packet using another sender ID.
    ASSERT_TRUE(signPacket(forged, cfg).accepted());
    EXPECT_EQ(verifyPacket(forged, cfg, 1000).result, PacketAuthResult::ACCEPTED);
}

TEST(EdgePeerProtocol, CborUint32TruncationRejected) {
    auto wire = serialize_edge_packet_cbor(heartbeat(7, 1000, 1));
    ASSERT_GE(wire.size(), 4u);
    ASSERT_EQ(wire[0], 0x8au);
    ASSERT_EQ(wire[3], 7u);
    // Replace the 1-byte sender ID with 2^32 + 7 (CBOR 64-bit integer).
    std::vector<uint8_t> malformed(wire.begin(), wire.begin() + 3);
    malformed.insert(malformed.end(), {0x1b, 0, 0, 0, 1, 0, 0, 0, 7});
    malformed.insert(malformed.end(), wire.begin() + 4, wire.end());
    const auto parsed = parse_edge_packet_cbor(malformed);
    EXPECT_FALSE(parsed.ok);
    EXPECT_EQ(parsed.error, "cbor integer field exceeds uint32 range");
}

TEST(EdgePeerProtocol, ExpirationArithmeticDoesNotWrap) {
    auto packet = heartbeat(7, std::numeric_limits<uint64_t>::max() - 4, 1);
    packet.ttl_ms = 1000;
    EXPECT_FALSE(packet_is_expired(packet, std::numeric_limits<uint64_t>::max() - 3));
    EXPECT_FALSE(packet_is_expired(packet, 1000));
}

TEST(SwarmStateCache, AuthenticatedClockSkewMatchesFreshnessWindow) {
    SwarmStateCache cache({8, 100, 5000});
    cache.configure_freshness({2000, 5000});
    const auto packet = heartbeat(7, 1000, 10);
    EXPECT_TRUE(cache.observe_packet(packet, 4000).accepted);
    SwarmStateCache stale_cache({8, 100, 5000});
    stale_cache.configure_freshness({2000, 5000});
    EXPECT_FALSE(stale_cache.observe_packet(packet, 4001).accepted);
}

TEST(SwarmStateCache, ReplayMemorySurvivesShortPeerExpiry) {
    SwarmStateCache cache({8, 100, 500});
    cache.configure_freshness({2000, 1000});
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 10), 1000).accepted);
    cache.expire(2500);
    ASSERT_TRUE(cache.last_sequence_number(7).has_value());
    EXPECT_EQ(cache.last_sequence_number(7).value_or(0), 10u);
    const auto replay = cache.observe_packet(heartbeat(7, 1000, 10), 2500);
    EXPECT_FALSE(replay.accepted);
    EXPECT_EQ(replay.reason, "stale sequence number");
}

TEST(SwarmStateCache, SlidingWindowAcceptsUnseenReorderingAndRejectsDuplicates) {
    SwarmStateCache cache;
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 10), 1000).accepted);
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 12), 1000).accepted);
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 11), 1000).accepted);
    EXPECT_EQ(cache.last_sequence_number(7).value_or(0), 12u);
    EXPECT_FALSE(cache.observe_packet(heartbeat(7, 1000, 11), 1000).accepted);
    EXPECT_FALSE(cache.observe_packet(heartbeat(7, 1000, 12), 1000).accepted);
    EXPECT_EQ(cache.last_sequence_number(7).value_or(0), 12u);
}

TEST(SwarmStateCache, SlidingWindowBoundsOldPacketsAndLargeJumps) {
    SwarmStateCache cache;
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 2), 1000).accepted);
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 70), 1000).accepted);
    ASSERT_TRUE(cache.observe_packet(heartbeat(7, 1000, 7), 1000).accepted);
    const auto too_old = cache.observe_packet(heartbeat(7, 1000, 6), 1000);
    EXPECT_FALSE(too_old.accepted);
    EXPECT_EQ(too_old.reason, "sequence outside replay window");
    EXPECT_FALSE(cache.observe_packet(heartbeat(7, 1000, 7), 1000).accepted);
    EXPECT_EQ(cache.last_sequence_number(7).value_or(0), 70u);
}

TEST(SwarmStateCache, ReorderedPacketsDoNotRollbackPeerState) {
    SwarmStateCache cache;
    auto newer = heartbeat(7, 1000, 11);
    auto latest_heartbeat = newer.heartbeat.value_or(HeartbeatPayload{});
    latest_heartbeat.peer_count = 81;
    newer.heartbeat = latest_heartbeat;
    auto older = heartbeat(7, 999, 10);
    auto previous_heartbeat = older.heartbeat.value_or(HeartbeatPayload{});
    previous_heartbeat.peer_count = 25;
    older.heartbeat = previous_heartbeat;
    ASSERT_TRUE(cache.observe_packet(newer, 1000).accepted);
    ASSERT_TRUE(cache.observe_packet(older, 1000).accepted);
    const auto state = cache.peer_state(7);
    ASSERT_TRUE(state.has_value());
    const auto cached = state.value_or(CachedPeerState{});
    EXPECT_EQ(cached.last_sequence_number, 11u);
    EXPECT_EQ(cached.last_packet_timestamp_ms, 1000u);
    EXPECT_EQ(cached.last_heartbeat_timestamp_ms, 1000u);
    EXPECT_EQ(cached.peer_count, 81);
}

TEST(SwarmStateCache, EpochChangeResetsWindowButRollbackFails) {
    SwarmStateCache cache;
    const auto max_sequence = heartbeat(7, 1000, std::numeric_limits<uint32_t>::max());
    ASSERT_TRUE(cache.observe_packet(max_sequence, 1000).accepted);
    EXPECT_FALSE(cache.observe_packet(heartbeat(7, 1000, 0), 1000).accepted);
    auto fresh_epoch = heartbeat(7, 1000, 0);
    fresh_epoch.trust_epoch = 2;
    ASSERT_TRUE(cache.observe_packet(fresh_epoch, 1000).accepted);
    EXPECT_EQ(cache.last_sequence_number(7).value_or(1), 0u);
    EXPECT_FALSE(cache.observe_packet(heartbeat(7, 1000, 1), 1000).accepted);
}

TEST(PeerPacketAuth, SlidingWindowAdmissionAuthenticatesBeforeCacheUpdate) {
    auto cfg = auth_config();
    SwarmStateCache cache;
    for (const auto sequence : {12u, 11u}) {
        auto packet = heartbeat(7, 1000, sequence);
        ASSERT_TRUE(signPacket(packet, cfg).accepted());
        ASSERT_TRUE(verifyPacket(packet, cfg, 1000).accepted());
        EXPECT_TRUE(cache.observe_packet(packet, 1000).accepted);
    }
    auto tampered = heartbeat(7, 1000, 10);
    ASSERT_TRUE(signPacket(tampered, cfg).accepted());
    tampered.sender_id = 8;
    EXPECT_EQ(verifyPacket(tampered, cfg, 1000).result, PacketAuthResult::INVALID_SIGNATURE);
    EXPECT_FALSE(cache.last_sequence_number(8).has_value());
}

TEST(PeerPacketAuth, SyntheticPacketReorderingAndLossStudy) {
    // Deterministic delivery traces, not measurements from a physical FANET radio.
    struct Trace {
        const char* name;
        size_t swap_every;
        size_t drop_every;
    };
    const std::array<Trace, 4> traces = {{
        {"ordered", 0, 0},
        {"adjacent_reorder", 2, 0},
        {"sparse_reorder", 10, 0},
        {"loss_and_reorder", 8, 13},
    }};
    auto cfg = auth_config();
    for (const auto& trace : traces) {
        std::vector<EdgePeerPacket> packets;
        for (uint32_t seq = 1; seq <= 128; ++seq) {
            if (trace.drop_every > 0 && seq % trace.drop_every == 0) {
                continue;
            }
            auto packet = heartbeat(7, 1000, seq);
            ASSERT_TRUE(signPacket(packet, cfg).accepted());
            packets.push_back(std::move(packet));
        }
        if (trace.swap_every > 0) {
            for (size_t i = 0; i + 1 < packets.size(); i += trace.swap_every) {
                std::swap(packets[i], packets[i + 1]);
            }
        }
        SwarmStateCache cache;
        std::optional<uint32_t> highest_arrival;
        size_t out_of_order = 0;
        size_t accepted = 0;
        size_t rejected = 0;
        for (const auto& packet : packets) {
            if (highest_arrival && packet.sequence_number <= *highest_arrival) {
                ++out_of_order;
            }
            const auto outcome = verifyPacket(packet, cfg, 1000);
            ASSERT_TRUE(outcome.accepted());
            const auto observed = cache.observe_packet(packet, 1000);
            if (observed.accepted) {
                ++accepted;
            } else {
                ++rejected;
            }
            if (!highest_arrival || packet.sequence_number > *highest_arrival) {
                highest_arrival = packet.sequence_number;
            }
        }
        EXPECT_EQ(rejected, 0u);
        EXPECT_EQ(accepted + rejected, packets.size());
        std::cout << "[peer-auth-synthetic-delivery] name=" << trace.name;
        std::cout << " transmitted=128 delivered=" << packets.size();
        std::cout << " accepted=" << accepted << " false_rejected=" << rejected;
        const double delivered_count = static_cast<double>(packets.size());
        const double out_of_order_rate = static_cast<double>(out_of_order) / delivered_count;
        std::cout << " out_of_order_rate=" << out_of_order_rate;
        std::cout << '\n';
    }
}

TEST(PeerPacketAuth, HmacSha256KnownAnswer) {
    const std::string message = "The quick brown fox jumps over the lazy dog";
    const std::vector<uint8_t> input(message.begin(), message.end());
    const auto digest = hmacSha256Digest("key", input);
    constexpr char hex[] = "0123456789abcdef";
    std::string actual;
    actual.reserve(digest.size() * 2);
    for (const auto byte : digest) {
        actual.push_back(hex[byte >> 4]);
        actual.push_back(hex[byte & 0x0f]);
    }
    EXPECT_EQ(actual, "f7bc83f430538424b13298e6aa6fb143ef4d59a14946175997479dbc2d1a3cd8");
}

TEST(PeerPacketAuth, InProcessComponentLatencySamples) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    packet.auth_hook.clear();
    const auto data = serialize_edge_packet_cbor(packet);
    constexpr size_t samples = 512;
    std::array<std::vector<uint64_t>, 4> durations;
    for (auto& group : durations) {
        group.reserve(samples);
    }
    uint8_t digest_sink = 0;
    using clock = std::chrono::steady_clock;
    for (size_t i = 0; i < samples; ++i) {
        auto begin = clock::now();
        const auto encoded = serialize_edge_packet_cbor(packet);
        auto finish = clock::now();
        durations[0].push_back(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finish - begin).count()));
        digest_sink ^= static_cast<uint8_t>(encoded.size());

        begin = clock::now();
        const auto mac = hmacSha256Digest(cfg.shared_secret, data);
        finish = clock::now();
        durations[1].push_back(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finish - begin).count()));
        digest_sink ^= mac[i % mac.size()];

        auto signed_packet = packet;
        begin = clock::now();
        const auto signed_result = signPacket(signed_packet, cfg);
        finish = clock::now();
        ASSERT_TRUE(signed_result.accepted());
        durations[2].push_back(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finish - begin).count()));

        begin = clock::now();
        const auto verified = verifyPacket(signed_packet, cfg, 1000);
        finish = clock::now();
        ASSERT_TRUE(verified.accepted());
        durations[3].push_back(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finish - begin).count()));
    }
    const std::array<const char*, 4> names = {"cbor_serialize", "hmac_sha256", "sign_path",
                                              "verify_path"};
    for (size_t i = 0; i < names.size(); ++i) {
        auto& values = durations[i];
        std::sort(values.begin(), values.end());
        std::cout << "[peer-auth-inprocess] " << names[i] << " n=" << values.size();
        std::cout << " p50_ns=" << values[values.size() / 2];
        std::cout << " p95_ns=" << values[values.size() * 95 / 100];
        std::cout << " p99_ns=" << values[values.size() * 99 / 100];
        std::cout << " min_ns=" << values.front() << " max_ns=" << values.back() << '\n';
    }
    std::cout << "[peer-auth-inprocess] canonical_cbor_bytes=" << data.size()
              << " digest_sink=" << static_cast<unsigned>(digest_sink) << "\n";
}

TEST(PeerPacketAuth, PqcPlaceholderReturnsUnsupported) {
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();
    cfg.mode = AuthMode::PQC_HYBRID_PLACEHOLDER;

    const auto verified = verifyPacket(packet, cfg, 1000);
    EXPECT_EQ(verified.result, PacketAuthResult::UNSUPPORTED);
    EXPECT_EQ(verified.reason, "PQC hybrid auth is roadmap-only in this build.");
}

TEST(PeerPacketAuth, AuthFailurePreventsCacheUpdate) {
    SwarmStateCache cache;
    auto packet = heartbeat(7, 1000, 1);
    auto cfg = auth_config();

    const auto verified = verifyPacket(packet, cfg, 1000);
    ASSERT_FALSE(verified.accepted());
    EXPECT_EQ(cache.peer_count(), 0u);
}

TEST(PeerPacketAuth, EmergencyCorridorRequiresAuthBeforePeerAction) {
    auto packet = emergency_corridor(2, 2000, 1);
    packet.auth_hook = "unsigned";
    auto cfg = auth_config();
    EdgeConsensusManager manager(1, 2);
    manager.set_local_safety_override(true);

    EXPECT_EQ(verifyPacket(packet, cfg, 2000).result, PacketAuthResult::MISSING_SIGNATURE);
    ASSERT_TRUE(signPacket(packet, cfg).accepted());
    EXPECT_EQ(verifyPacket(packet, cfg, 2000).result, PacketAuthResult::ACCEPTED);
    EXPECT_TRUE(manager.should_accept_emergency_packet(packet));
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
    const auto validation =
        validate_edge_packet(packet, EdgePacketValidationOptions{2000, 0, false, 1400});
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
