#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone::swarm {

enum class EdgePacketType : uint8_t {
    HEARTBEAT = 0,
    POSE_STATE,
    EDGE_HEALTH,
    OBSTACLE_DIGEST,
    THREAT_DIGEST,
    CONSENSUS_STATE,
    EMERGENCY_CORRIDOR,
    PEER_GOODBYE,
};

enum class ConsensusProposalType : uint8_t {
    NONE = 0,
    COLLECTIVE_HALT,
    EMERGENCY_REROUTE,
    LEADER_CONTINUITY_HINT,
    SPLIT_SWARM_RECOVERY_HINT,
};

enum class EdgeSerializationMode : uint8_t {
    JSON = 0,
    CBOR,
    PROTOBUF_PLACEHOLDER,
};

struct EdgeSerializationMetrics {
    EdgeSerializationMode mode{EdgeSerializationMode::JSON};
    size_t encoded_packet_size_bytes{0};
    size_t json_equivalent_size_bytes{0};
    double serialization_time_us{0.0};
    double deserialization_time_us{0.0};
    double compression_ratio_vs_json{1.0};
    uint64_t auth_failures{0};
    uint64_t unsigned_packets{0};
    std::string auth_mode{"none"};
    std::string last_auth_result{"accepted"};
    std::string pqc_ready_status{"not_implemented"};
    bool estimated{false};
};

struct HeartbeatPayload {
    int peer_count{0};
    int stale_peer_count{0};
    double battery_pct{0.0};
    double motor_health{0.0};
    double link_quality{0.0};
    bool emergency_fault{false};
    std::string edge_health_status{"nominal"};
    std::string autonomy_state{"distributed_autonomy"};
};

struct PoseStatePayload {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double localization_confidence{0.0};
};

struct EdgeHealthPayload {
    std::string edge_health_status{"nominal"};
    std::string autonomy_state{"distributed_autonomy"};
    std::string consensus_state{"single_node"};
    double mesh_bandwidth_kbps{0.0};
    bool disconnected_operation{false};
};

struct ObstacleDigestPayload {
    int local_obstacle_count{0};
    int shared_obstacle_count{0};
    uint32_t freshness_ms{0};
    std::string digest_id{};
};

struct ThreatDigestPayload {
    std::string threat_level{"none"};
    std::string summary{};
    double confidence{0.0};
};

struct ConsensusStatePayload {
    ConsensusProposalType proposal_type{ConsensusProposalType::NONE};
    std::string consensus_state{"single_node"};
    uint64_t consensus_epoch{0};
    int quorum_count{0};
    bool local_safety_override{false};
};

struct EmergencyCorridorPayload {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double radius_m{0.0};
    uint32_t hold_ttl_ms{0};
    std::string summary{};
};

struct PeerGoodbyePayload {
    std::string reason{"shutdown"};
};

struct EdgePeerPacket {
    EdgePacketType packet_type{EdgePacketType::HEARTBEAT};
    uint32_t sender_id{0};
    uint64_t timestamp_ms{0};
    uint32_t sequence_number{0};
    uint64_t trust_epoch{0};
    std::string source{"unavailable"};
    uint32_t ttl_ms{0};
    std::string auth_hook{"unsigned"};

    std::optional<HeartbeatPayload> heartbeat{};
    std::optional<PoseStatePayload> pose_state{};
    std::optional<EdgeHealthPayload> edge_health{};
    std::optional<ObstacleDigestPayload> obstacle_digest{};
    std::optional<ThreatDigestPayload> threat_digest{};
    std::optional<ConsensusStatePayload> consensus_state{};
    std::optional<EmergencyCorridorPayload> emergency_corridor{};
    std::optional<PeerGoodbyePayload> peer_goodbye{};
};

struct EdgePacketValidationOptions {
    uint64_t now_ms{0};
    uint32_t last_sequence_number{0};
    bool has_last_sequence{false};
    size_t max_packet_size_bytes{1400};
    size_t serialized_packet_size_bytes{0};
};

struct EdgePacketValidationResult {
    bool ok{false};
    std::string error{};
};

struct EdgePacketParseResult {
    bool ok{false};
    EdgePeerPacket packet{};
    std::string error{};
};

[[nodiscard]] std::string_view to_string(EdgePacketType type);
[[nodiscard]] std::optional<EdgePacketType> parse_edge_packet_type(std::string_view value);
[[nodiscard]] std::string_view to_string(ConsensusProposalType type);
[[nodiscard]] std::optional<ConsensusProposalType> parse_consensus_proposal_type(std::string_view value);
[[nodiscard]] std::string_view to_string(EdgeSerializationMode mode);
[[nodiscard]] std::optional<EdgeSerializationMode> parse_edge_serialization_mode(std::string_view value);
[[nodiscard]] std::string normalize_edge_source_tag(std::string_view value);
[[nodiscard]] bool packet_is_expired(const EdgePeerPacket& packet, uint64_t now_ms);
[[nodiscard]] std::string serialize_edge_packet_json(const EdgePeerPacket& packet);
[[nodiscard]] EdgePacketParseResult parse_edge_packet_json(std::string_view wire);
[[nodiscard]] std::vector<uint8_t> serialize_edge_packet_cbor(const EdgePeerPacket& packet);
[[nodiscard]] EdgePacketParseResult parse_edge_packet_cbor(std::span<const uint8_t> wire);
[[nodiscard]] std::vector<uint8_t> serialize_edge_packet(
    const EdgePeerPacket& packet,
    EdgeSerializationMode mode,
    EdgeSerializationMetrics* metrics = nullptr);
[[nodiscard]] EdgePacketParseResult parse_edge_packet(
    std::span<const uint8_t> wire,
    EdgeSerializationMode mode,
    EdgeSerializationMetrics* metrics = nullptr);
[[nodiscard]] EdgePacketValidationResult validate_edge_packet(
    const EdgePeerPacket& packet,
    const EdgePacketValidationOptions& options);

} // namespace drone::swarm
