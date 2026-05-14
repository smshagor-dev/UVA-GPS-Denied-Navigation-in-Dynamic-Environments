#include "swarm/EdgePeerProtocol.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <regex>
#include <sstream>

namespace drone::swarm {

namespace {

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

std::string json_escape(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const unsigned char c : value) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out.push_back(static_cast<char>(c)); break;
        }
    }
    return out;
}

std::string format_double(double value) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(4);
    oss << value;
    return oss.str();
}

std::optional<std::string> extract_string(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"", std::regex::icase);
    std::smatch match;
    if (std::regex_search(content, match, pattern) && match.size() >= 2) {
        return match[1].str();
    }
    return std::nullopt;
}

std::optional<double> extract_number(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)", std::regex::icase);
    std::smatch match;
    if (std::regex_search(content, match, pattern) && match.size() >= 2) {
        try {
            return std::stod(match[1].str());
        } catch (...) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::optional<bool> extract_bool(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)", std::regex::icase);
    std::smatch match;
    if (std::regex_search(content, match, pattern) && match.size() >= 2) {
        return lowercase(match[1].str()) == "true";
    }
    return std::nullopt;
}

std::optional<Eigen::Vector3d> extract_vec3(const std::string& content, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*\\[\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*\\]", std::regex::icase);
    std::smatch match;
    if (std::regex_search(content, match, pattern) && match.size() >= 4) {
        try {
            return Eigen::Vector3d{std::stod(match[1].str()), std::stod(match[2].str()), std::stod(match[3].str())};
        } catch (...) {
            return std::nullopt;
        }
    }
    return std::nullopt;
}

bool finite_vec3(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

} // namespace

std::string_view to_string(EdgePacketType type) {
    switch (type) {
    case EdgePacketType::HEARTBEAT: return "heartbeat";
    case EdgePacketType::POSE_STATE: return "pose_state";
    case EdgePacketType::EDGE_HEALTH: return "edge_health";
    case EdgePacketType::OBSTACLE_DIGEST: return "obstacle_digest";
    case EdgePacketType::THREAT_DIGEST: return "threat_digest";
    case EdgePacketType::CONSENSUS_STATE: return "consensus_state";
    case EdgePacketType::EMERGENCY_CORRIDOR: return "emergency_corridor";
    case EdgePacketType::PEER_GOODBYE: return "peer_goodbye";
    }
    return "heartbeat";
}

std::optional<EdgePacketType> parse_edge_packet_type(std::string_view value) {
    const auto normalized = lowercase(value);
    if (normalized == "heartbeat") return EdgePacketType::HEARTBEAT;
    if (normalized == "pose_state") return EdgePacketType::POSE_STATE;
    if (normalized == "edge_health") return EdgePacketType::EDGE_HEALTH;
    if (normalized == "obstacle_digest") return EdgePacketType::OBSTACLE_DIGEST;
    if (normalized == "threat_digest") return EdgePacketType::THREAT_DIGEST;
    if (normalized == "consensus_state") return EdgePacketType::CONSENSUS_STATE;
    if (normalized == "emergency_corridor") return EdgePacketType::EMERGENCY_CORRIDOR;
    if (normalized == "peer_goodbye") return EdgePacketType::PEER_GOODBYE;
    return std::nullopt;
}

std::string_view to_string(ConsensusProposalType type) {
    switch (type) {
    case ConsensusProposalType::NONE: return "none";
    case ConsensusProposalType::COLLECTIVE_HALT: return "collective_halt";
    case ConsensusProposalType::EMERGENCY_REROUTE: return "emergency_reroute";
    case ConsensusProposalType::LEADER_CONTINUITY_HINT: return "leader_continuity_hint";
    case ConsensusProposalType::SPLIT_SWARM_RECOVERY_HINT: return "split_swarm_recovery_hint";
    }
    return "none";
}

std::optional<ConsensusProposalType> parse_consensus_proposal_type(std::string_view value) {
    const auto normalized = lowercase(value);
    if (normalized == "none") return ConsensusProposalType::NONE;
    if (normalized == "collective_halt") return ConsensusProposalType::COLLECTIVE_HALT;
    if (normalized == "emergency_reroute") return ConsensusProposalType::EMERGENCY_REROUTE;
    if (normalized == "leader_continuity_hint") return ConsensusProposalType::LEADER_CONTINUITY_HINT;
    if (normalized == "split_swarm_recovery_hint") return ConsensusProposalType::SPLIT_SWARM_RECOVERY_HINT;
    return std::nullopt;
}

std::string normalize_edge_source_tag(std::string_view value) {
    const auto normalized = lowercase(value);
    if (normalized == "real" || normalized == "simulation" || normalized == "playback" || normalized == "unavailable") {
        return normalized;
    }
    return "unavailable";
}

bool packet_is_expired(const EdgePeerPacket& packet, uint64_t now_ms) {
    if (packet.ttl_ms == 0) {
        return true;
    }
    return now_ms > (packet.timestamp_ms + packet.ttl_ms);
}

std::string serialize_edge_packet_json(const EdgePeerPacket& packet) {
    std::ostringstream oss;
    oss << "{"
        << "\"packet_type\":\"" << json_escape(to_string(packet.packet_type)) << "\","
        << "\"sender_id\":" << packet.sender_id << ","
        << "\"timestamp_ms\":" << packet.timestamp_ms << ","
        << "\"sequence_number\":" << packet.sequence_number << ","
        << "\"trust_epoch\":" << packet.trust_epoch << ","
        << "\"source\":\"" << json_escape(normalize_edge_source_tag(packet.source)) << "\","
        << "\"ttl_ms\":" << packet.ttl_ms << ","
        << "\"auth_hook\":\"" << json_escape(packet.auth_hook) << "\","
        << "\"payload\":{";

    switch (packet.packet_type) {
    case EdgePacketType::HEARTBEAT:
        if (packet.heartbeat.has_value()) {
            const auto& p = *packet.heartbeat;
            oss << "\"peer_count\":" << p.peer_count << ","
                << "\"stale_peer_count\":" << p.stale_peer_count << ","
                << "\"battery_pct\":" << format_double(p.battery_pct) << ","
                << "\"motor_health\":" << format_double(p.motor_health) << ","
                << "\"link_quality\":" << format_double(p.link_quality) << ","
                << "\"emergency_fault\":" << (p.emergency_fault ? "true" : "false") << ","
                << "\"edge_health_status\":\"" << json_escape(p.edge_health_status) << "\","
                << "\"autonomy_state\":\"" << json_escape(p.autonomy_state) << "\"";
        }
        break;
    case EdgePacketType::POSE_STATE:
        if (packet.pose_state.has_value()) {
            const auto& p = *packet.pose_state;
            oss << "\"position_xyz\":[" << format_double(p.position.x()) << "," << format_double(p.position.y()) << "," << format_double(p.position.z()) << "],"
                << "\"velocity_xyz\":[" << format_double(p.velocity.x()) << "," << format_double(p.velocity.y()) << "," << format_double(p.velocity.z()) << "],"
                << "\"localization_confidence\":" << format_double(p.localization_confidence);
        }
        break;
    case EdgePacketType::EDGE_HEALTH:
        if (packet.edge_health.has_value()) {
            const auto& p = *packet.edge_health;
            oss << "\"edge_health_status\":\"" << json_escape(p.edge_health_status) << "\","
                << "\"autonomy_state\":\"" << json_escape(p.autonomy_state) << "\","
                << "\"consensus_state\":\"" << json_escape(p.consensus_state) << "\","
                << "\"mesh_bandwidth_kbps\":" << format_double(p.mesh_bandwidth_kbps) << ","
                << "\"disconnected_operation\":" << (p.disconnected_operation ? "true" : "false");
        }
        break;
    case EdgePacketType::OBSTACLE_DIGEST:
        if (packet.obstacle_digest.has_value()) {
            const auto& p = *packet.obstacle_digest;
            oss << "\"local_obstacle_count\":" << p.local_obstacle_count << ","
                << "\"shared_obstacle_count\":" << p.shared_obstacle_count << ","
                << "\"freshness_ms\":" << p.freshness_ms << ","
                << "\"digest_id\":\"" << json_escape(p.digest_id) << "\"";
        }
        break;
    case EdgePacketType::THREAT_DIGEST:
        if (packet.threat_digest.has_value()) {
            const auto& p = *packet.threat_digest;
            oss << "\"threat_level\":\"" << json_escape(p.threat_level) << "\","
                << "\"summary\":\"" << json_escape(p.summary) << "\","
                << "\"confidence\":" << format_double(p.confidence);
        }
        break;
    case EdgePacketType::CONSENSUS_STATE:
        if (packet.consensus_state.has_value()) {
            const auto& p = *packet.consensus_state;
            oss << "\"proposal_type\":\"" << json_escape(to_string(p.proposal_type)) << "\","
                << "\"consensus_state\":\"" << json_escape(p.consensus_state) << "\","
                << "\"consensus_epoch\":" << p.consensus_epoch << ","
                << "\"quorum_count\":" << p.quorum_count << ","
                << "\"local_safety_override\":" << (p.local_safety_override ? "true" : "false");
        }
        break;
    case EdgePacketType::EMERGENCY_CORRIDOR:
        if (packet.emergency_corridor.has_value()) {
            const auto& p = *packet.emergency_corridor;
            oss << "\"center_xyz\":[" << format_double(p.center.x()) << "," << format_double(p.center.y()) << "," << format_double(p.center.z()) << "],"
                << "\"radius_m\":" << format_double(p.radius_m) << ","
                << "\"hold_ttl_ms\":" << p.hold_ttl_ms << ","
                << "\"summary\":\"" << json_escape(p.summary) << "\"";
        }
        break;
    case EdgePacketType::PEER_GOODBYE:
        if (packet.peer_goodbye.has_value()) {
            oss << "\"reason\":\"" << json_escape(packet.peer_goodbye->reason) << "\"";
        }
        break;
    }

    oss << "}}";
    return oss.str();
}

EdgePacketParseResult parse_edge_packet_json(std::string_view wire) {
    EdgePacketParseResult result;
    const std::string content(wire.begin(), wire.end());
    const auto first = content.find_first_not_of(" \t\r\n");
    const auto last = content.find_last_not_of(" \t\r\n");
    if (first == std::string::npos || content[first] != '{' || content[last] != '}' ||
        content.find("\"payload\"") == std::string::npos) {
        result.error = "malformed packet";
        return result;
    }

    const auto type_text = extract_string(content, "packet_type");
    if (!type_text.has_value()) {
        result.error = "missing packet_type";
        return result;
    }
    const auto type = parse_edge_packet_type(*type_text);
    if (!type.has_value()) {
        result.error = "unknown packet_type";
        return result;
    }

    const auto sender_id = extract_number(content, "sender_id");
    const auto timestamp_ms = extract_number(content, "timestamp_ms");
    const auto sequence_number = extract_number(content, "sequence_number");
    const auto trust_epoch = extract_number(content, "trust_epoch");
    const auto ttl_ms = extract_number(content, "ttl_ms");
    if (!sender_id.has_value() || !timestamp_ms.has_value() || !sequence_number.has_value() ||
        !trust_epoch.has_value() || !ttl_ms.has_value()) {
        result.error = "missing required numeric field";
        return result;
    }

    result.packet.packet_type = *type;
    result.packet.sender_id = static_cast<uint32_t>(*sender_id);
    result.packet.timestamp_ms = static_cast<uint64_t>(*timestamp_ms);
    result.packet.sequence_number = static_cast<uint32_t>(*sequence_number);
    result.packet.trust_epoch = static_cast<uint64_t>(*trust_epoch);
    result.packet.source = normalize_edge_source_tag(extract_string(content, "source").value_or("unavailable"));
    result.packet.ttl_ms = static_cast<uint32_t>(*ttl_ms);
    result.packet.auth_hook = extract_string(content, "auth_hook").value_or("unsigned");

    switch (*type) {
    case EdgePacketType::HEARTBEAT: {
        HeartbeatPayload payload;
        payload.peer_count = static_cast<int>(extract_number(content, "peer_count").value_or(0.0));
        payload.stale_peer_count = static_cast<int>(extract_number(content, "stale_peer_count").value_or(0.0));
        payload.battery_pct = extract_number(content, "battery_pct").value_or(0.0);
        payload.motor_health = extract_number(content, "motor_health").value_or(0.0);
        payload.link_quality = extract_number(content, "link_quality").value_or(0.0);
        payload.emergency_fault = extract_bool(content, "emergency_fault").value_or(false);
        payload.edge_health_status = extract_string(content, "edge_health_status").value_or("nominal");
        payload.autonomy_state = extract_string(content, "autonomy_state").value_or("distributed_autonomy");
        result.packet.heartbeat = payload;
        break;
    }
    case EdgePacketType::POSE_STATE: {
        PoseStatePayload payload;
        payload.position = extract_vec3(content, "position_xyz").value_or(Eigen::Vector3d::Zero());
        payload.velocity = extract_vec3(content, "velocity_xyz").value_or(Eigen::Vector3d::Zero());
        payload.localization_confidence = extract_number(content, "localization_confidence").value_or(0.0);
        result.packet.pose_state = payload;
        break;
    }
    case EdgePacketType::EDGE_HEALTH: {
        EdgeHealthPayload payload;
        payload.edge_health_status = extract_string(content, "edge_health_status").value_or("nominal");
        payload.autonomy_state = extract_string(content, "autonomy_state").value_or("distributed_autonomy");
        payload.consensus_state = extract_string(content, "consensus_state").value_or("single_node");
        payload.mesh_bandwidth_kbps = extract_number(content, "mesh_bandwidth_kbps").value_or(0.0);
        payload.disconnected_operation = extract_bool(content, "disconnected_operation").value_or(false);
        result.packet.edge_health = payload;
        break;
    }
    case EdgePacketType::OBSTACLE_DIGEST: {
        ObstacleDigestPayload payload;
        payload.local_obstacle_count = static_cast<int>(extract_number(content, "local_obstacle_count").value_or(0.0));
        payload.shared_obstacle_count = static_cast<int>(extract_number(content, "shared_obstacle_count").value_or(0.0));
        payload.freshness_ms = static_cast<uint32_t>(extract_number(content, "freshness_ms").value_or(0.0));
        payload.digest_id = extract_string(content, "digest_id").value_or("");
        result.packet.obstacle_digest = payload;
        break;
    }
    case EdgePacketType::THREAT_DIGEST: {
        ThreatDigestPayload payload;
        payload.threat_level = extract_string(content, "threat_level").value_or("none");
        payload.summary = extract_string(content, "summary").value_or("");
        payload.confidence = extract_number(content, "confidence").value_or(0.0);
        result.packet.threat_digest = payload;
        break;
    }
    case EdgePacketType::CONSENSUS_STATE: {
        ConsensusStatePayload payload;
        payload.proposal_type = parse_consensus_proposal_type(extract_string(content, "proposal_type").value_or("none")).value_or(ConsensusProposalType::NONE);
        payload.consensus_state = extract_string(content, "consensus_state").value_or("single_node");
        payload.consensus_epoch = static_cast<uint64_t>(extract_number(content, "consensus_epoch").value_or(0.0));
        payload.quorum_count = static_cast<int>(extract_number(content, "quorum_count").value_or(0.0));
        payload.local_safety_override = extract_bool(content, "local_safety_override").value_or(false);
        result.packet.consensus_state = payload;
        break;
    }
    case EdgePacketType::EMERGENCY_CORRIDOR: {
        EmergencyCorridorPayload payload;
        payload.center = extract_vec3(content, "center_xyz").value_or(Eigen::Vector3d::Zero());
        payload.radius_m = extract_number(content, "radius_m").value_or(0.0);
        payload.hold_ttl_ms = static_cast<uint32_t>(extract_number(content, "hold_ttl_ms").value_or(0.0));
        payload.summary = extract_string(content, "summary").value_or("");
        result.packet.emergency_corridor = payload;
        break;
    }
    case EdgePacketType::PEER_GOODBYE: {
        PeerGoodbyePayload payload;
        payload.reason = extract_string(content, "reason").value_or("shutdown");
        result.packet.peer_goodbye = payload;
        break;
    }
    }

    result.ok = true;
    return result;
}

EdgePacketValidationResult validate_edge_packet(
    const EdgePeerPacket& packet,
    const EdgePacketValidationOptions& options) {
    EdgePacketValidationResult result;

    if (serialize_edge_packet_json(packet).size() > options.max_packet_size_bytes) {
        result.error = "packet exceeds max size";
        return result;
    }
    if (packet.sender_id == 0) {
        result.error = "sender_id must be non-zero";
        return result;
    }
    if (packet.ttl_ms == 0) {
        result.error = "ttl_ms must be positive";
        return result;
    }
    if (packet_is_expired(packet, options.now_ms)) {
        result.error = "packet expired";
        return result;
    }
    if (options.has_last_sequence && packet.sequence_number <= options.last_sequence_number) {
        result.error = "stale sequence number";
        return result;
    }

    switch (packet.packet_type) {
    case EdgePacketType::HEARTBEAT:
        if (!packet.heartbeat.has_value()) result.error = "missing heartbeat payload";
        break;
    case EdgePacketType::POSE_STATE:
        if (!packet.pose_state.has_value()) result.error = "missing pose_state payload";
        break;
    case EdgePacketType::EDGE_HEALTH:
        if (!packet.edge_health.has_value()) result.error = "missing edge_health payload";
        break;
    case EdgePacketType::OBSTACLE_DIGEST:
        if (!packet.obstacle_digest.has_value()) result.error = "missing obstacle_digest payload";
        break;
    case EdgePacketType::THREAT_DIGEST:
        if (!packet.threat_digest.has_value()) result.error = "missing threat_digest payload";
        break;
    case EdgePacketType::CONSENSUS_STATE:
        if (!packet.consensus_state.has_value()) result.error = "missing consensus_state payload";
        break;
    case EdgePacketType::EMERGENCY_CORRIDOR:
        if (!packet.emergency_corridor.has_value()) result.error = "missing emergency_corridor payload";
        break;
    case EdgePacketType::PEER_GOODBYE:
        if (!packet.peer_goodbye.has_value()) result.error = "missing peer_goodbye payload";
        break;
    }
    if (!result.error.empty()) {
        return result;
    }

    if (packet.pose_state.has_value()) {
        if (!finite_vec3(packet.pose_state->position) || !finite_vec3(packet.pose_state->velocity)) {
            result.error = "pose vectors must be finite";
            return result;
        }
    }
    if (packet.emergency_corridor.has_value()) {
        if (!finite_vec3(packet.emergency_corridor->center) || packet.emergency_corridor->radius_m <= 0.0) {
            result.error = "emergency corridor invalid";
            return result;
        }
    }
    result.ok = true;
    return result;
}

} // namespace drone::swarm
