#include "swarm/EdgePeerProtocol.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <regex>
#include <sstream>

namespace drone::swarm {

namespace {

constexpr uint8_t kCborPacketVersion = 1;
constexpr size_t kCborMaxStringBytes = 512;

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

void cbor_push_type_and_len(std::vector<uint8_t>& out, uint8_t major, uint64_t value) {
    const uint8_t prefix = static_cast<uint8_t>(major << 5);
    if (value < 24) {
        out.push_back(static_cast<uint8_t>(prefix | value));
    } else if (value <= 0xFFu) {
        out.push_back(static_cast<uint8_t>(prefix | 24u));
        out.push_back(static_cast<uint8_t>(value));
    } else if (value <= 0xFFFFu) {
        out.push_back(static_cast<uint8_t>(prefix | 25u));
        out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
        out.push_back(static_cast<uint8_t>(value & 0xFFu));
    } else if (value <= 0xFFFFFFFFull) {
        out.push_back(static_cast<uint8_t>(prefix | 26u));
        for (int shift = 24; shift >= 0; shift -= 8) {
            out.push_back(static_cast<uint8_t>((value >> shift) & 0xFFu));
        }
    } else {
        out.push_back(static_cast<uint8_t>(prefix | 27u));
        for (int shift = 56; shift >= 0; shift -= 8) {
            out.push_back(static_cast<uint8_t>((value >> shift) & 0xFFu));
        }
    }
}

void cbor_push_uint(std::vector<uint8_t>& out, uint64_t value) {
    cbor_push_type_and_len(out, 0, value);
}

void cbor_push_int(std::vector<uint8_t>& out, int64_t value) {
    if (value >= 0) {
        cbor_push_uint(out, static_cast<uint64_t>(value));
        return;
    }
    cbor_push_type_and_len(out, 1, static_cast<uint64_t>(-1 - value));
}

void cbor_push_double(std::vector<uint8_t>& out, double value) {
    out.push_back(0xFB);
    static_assert(sizeof(double) == sizeof(uint64_t));
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<uint8_t>((bits >> shift) & 0xFFu));
    }
}

void cbor_push_bool(std::vector<uint8_t>& out, bool value) {
    out.push_back(value ? 0xF5 : 0xF4);
}

void cbor_push_string(std::vector<uint8_t>& out, std::string_view value) {
    cbor_push_type_and_len(out, 3, value.size());
    out.insert(out.end(), value.begin(), value.end());
}

void cbor_push_array(std::vector<uint8_t>& out, size_t size) {
    cbor_push_type_and_len(out, 4, size);
}

void cbor_push_vec3(std::vector<uint8_t>& out, const Eigen::Vector3d& value) {
    cbor_push_array(out, 3);
    cbor_push_double(out, value.x());
    cbor_push_double(out, value.y());
    cbor_push_double(out, value.z());
}

class CborReader {
public:
    explicit CborReader(std::span<const uint8_t> bytes) : bytes_(bytes) {}

    [[nodiscard]] bool eof() const { return offset_ == bytes_.size(); }
    [[nodiscard]] std::string error() const { return error_; }

    std::optional<uint64_t> read_uint() {
        const auto header = read_header(0);
        if (!header.has_value()) return std::nullopt;
        return *header;
    }

    std::optional<int64_t> read_int() {
        if (!ensure(1)) return std::nullopt;
        const uint8_t initial = bytes_[offset_];
        const uint8_t major = initial >> 5;
        if (major != 0 && major != 1) {
            error_ = "expected integer";
            return std::nullopt;
        }
        ++offset_;
        const auto value = read_len(initial & 0x1F);
        if (!value.has_value()) return std::nullopt;
        if (major == 0) return static_cast<int64_t>(*value);
        if (*value > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
            error_ = "negative integer overflow";
            return std::nullopt;
        }
        return -1 - static_cast<int64_t>(*value);
    }

    std::optional<double> read_double() {
        if (!ensure(9)) return std::nullopt;
        if (bytes_[offset_++] != 0xFB) {
            error_ = "expected double";
            return std::nullopt;
        }
        uint64_t bits = 0;
        for (int i = 0; i < 8; ++i) {
            bits = (bits << 8) | bytes_[offset_++];
        }
        double value = 0.0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::optional<bool> read_bool() {
        if (!ensure(1)) return std::nullopt;
        const uint8_t value = bytes_[offset_++];
        if (value == 0xF4) return false;
        if (value == 0xF5) return true;
        error_ = "expected bool";
        return std::nullopt;
    }

    std::optional<std::string> read_string() {
        const auto len = read_header(3);
        if (!len.has_value()) return std::nullopt;
        if (*len > kCborMaxStringBytes) {
            error_ = "string exceeds max size";
            return std::nullopt;
        }
        if (!ensure(static_cast<size_t>(*len))) return std::nullopt;
        std::string out(
            reinterpret_cast<const char*>(bytes_.data() + offset_),
            reinterpret_cast<const char*>(bytes_.data() + offset_ + *len));
        offset_ += static_cast<size_t>(*len);
        return out;
    }

    std::optional<size_t> read_array_size() {
        const auto len = read_header(4);
        if (!len.has_value()) return std::nullopt;
        if (*len > 64) {
            error_ = "array exceeds max size";
            return std::nullopt;
        }
        return static_cast<size_t>(*len);
    }

    std::optional<Eigen::Vector3d> read_vec3() {
        const auto size = read_array_size();
        if (!size.has_value()) return std::nullopt;
        if (*size != 3) {
            error_ = "expected vec3 array";
            return std::nullopt;
        }
        const auto x = read_double();
        const auto y = read_double();
        const auto z = read_double();
        if (!x.has_value() || !y.has_value() || !z.has_value()) return std::nullopt;
        return Eigen::Vector3d{*x, *y, *z};
    }

private:
    bool ensure(size_t count) {
        if (offset_ + count > bytes_.size()) {
            error_ = "truncated cbor packet";
            return false;
        }
        return true;
    }

    std::optional<uint64_t> read_header(uint8_t expected_major) {
        if (!ensure(1)) return std::nullopt;
        const uint8_t initial = bytes_[offset_++];
        const uint8_t major = initial >> 5;
        if (major != expected_major) {
            error_ = "unexpected cbor major type";
            return std::nullopt;
        }
        return read_len(initial & 0x1F);
    }

    std::optional<uint64_t> read_len(uint8_t additional) {
        if (additional < 24) return additional;
        if (additional == 24) {
            if (!ensure(1)) return std::nullopt;
            return bytes_[offset_++];
        }
        if (additional == 25) {
            if (!ensure(2)) return std::nullopt;
            uint64_t value = (static_cast<uint64_t>(bytes_[offset_]) << 8) | bytes_[offset_ + 1];
            offset_ += 2;
            return value;
        }
        if (additional == 26) {
            if (!ensure(4)) return std::nullopt;
            uint64_t value = 0;
            for (int i = 0; i < 4; ++i) value = (value << 8) | bytes_[offset_++];
            return value;
        }
        if (additional == 27) {
            if (!ensure(8)) return std::nullopt;
            uint64_t value = 0;
            for (int i = 0; i < 8; ++i) value = (value << 8) | bytes_[offset_++];
            return value;
        }
        error_ = "indefinite cbor length rejected";
        return std::nullopt;
    }

    std::span<const uint8_t> bytes_;
    size_t offset_{0};
    std::string error_{};
};

void write_cbor_payload(std::vector<uint8_t>& out, const EdgePeerPacket& packet) {
    switch (packet.packet_type) {
    case EdgePacketType::HEARTBEAT: {
        const auto& p = packet.heartbeat.value_or(HeartbeatPayload{});
        cbor_push_array(out, 8);
        cbor_push_int(out, p.peer_count);
        cbor_push_int(out, p.stale_peer_count);
        cbor_push_double(out, p.battery_pct);
        cbor_push_double(out, p.motor_health);
        cbor_push_double(out, p.link_quality);
        cbor_push_bool(out, p.emergency_fault);
        cbor_push_string(out, p.edge_health_status);
        cbor_push_string(out, p.autonomy_state);
        break;
    }
    case EdgePacketType::POSE_STATE: {
        const auto& p = packet.pose_state.value_or(PoseStatePayload{});
        cbor_push_array(out, 3);
        cbor_push_vec3(out, p.position);
        cbor_push_vec3(out, p.velocity);
        cbor_push_double(out, p.localization_confidence);
        break;
    }
    case EdgePacketType::EDGE_HEALTH: {
        const auto& p = packet.edge_health.value_or(EdgeHealthPayload{});
        cbor_push_array(out, 5);
        cbor_push_string(out, p.edge_health_status);
        cbor_push_string(out, p.autonomy_state);
        cbor_push_string(out, p.consensus_state);
        cbor_push_double(out, p.mesh_bandwidth_kbps);
        cbor_push_bool(out, p.disconnected_operation);
        break;
    }
    case EdgePacketType::OBSTACLE_DIGEST: {
        const auto& p = packet.obstacle_digest.value_or(ObstacleDigestPayload{});
        cbor_push_array(out, 4);
        cbor_push_int(out, p.local_obstacle_count);
        cbor_push_int(out, p.shared_obstacle_count);
        cbor_push_uint(out, p.freshness_ms);
        cbor_push_string(out, p.digest_id);
        break;
    }
    case EdgePacketType::THREAT_DIGEST: {
        const auto& p = packet.threat_digest.value_or(ThreatDigestPayload{});
        cbor_push_array(out, 3);
        cbor_push_string(out, p.threat_level);
        cbor_push_string(out, p.summary);
        cbor_push_double(out, p.confidence);
        break;
    }
    case EdgePacketType::CONSENSUS_STATE: {
        const auto& p = packet.consensus_state.value_or(ConsensusStatePayload{});
        cbor_push_array(out, 5);
        cbor_push_uint(out, static_cast<uint8_t>(p.proposal_type));
        cbor_push_string(out, p.consensus_state);
        cbor_push_uint(out, p.consensus_epoch);
        cbor_push_int(out, p.quorum_count);
        cbor_push_bool(out, p.local_safety_override);
        break;
    }
    case EdgePacketType::EMERGENCY_CORRIDOR: {
        const auto& p = packet.emergency_corridor.value_or(EmergencyCorridorPayload{});
        cbor_push_array(out, 4);
        cbor_push_vec3(out, p.center);
        cbor_push_double(out, p.radius_m);
        cbor_push_uint(out, p.hold_ttl_ms);
        cbor_push_string(out, p.summary);
        break;
    }
    case EdgePacketType::PEER_GOODBYE: {
        cbor_push_array(out, 1);
        cbor_push_string(out, packet.peer_goodbye.value_or(PeerGoodbyePayload{}).reason);
        break;
    }
    }
}

bool read_cbor_payload(CborReader& reader, EdgePeerPacket& packet, std::string& error) {
    const auto payload_size = reader.read_array_size();
    if (!payload_size.has_value()) {
        error = reader.error();
        return false;
    }

    switch (packet.packet_type) {
    case EdgePacketType::HEARTBEAT: {
        if (*payload_size != 8) { error = "heartbeat payload shape mismatch"; return false; }
        HeartbeatPayload p;
        const auto peer_count = reader.read_int();
        const auto stale_peer_count = reader.read_int();
        const auto battery = reader.read_double();
        const auto motor = reader.read_double();
        const auto link = reader.read_double();
        const auto fault = reader.read_bool();
        const auto health = reader.read_string();
        const auto autonomy = reader.read_string();
        if (!peer_count || !stale_peer_count || !battery || !motor || !link || !fault || !health || !autonomy) break;
        p.peer_count = static_cast<int>(*peer_count);
        p.stale_peer_count = static_cast<int>(*stale_peer_count);
        p.battery_pct = *battery;
        p.motor_health = *motor;
        p.link_quality = *link;
        p.emergency_fault = *fault;
        p.edge_health_status = *health;
        p.autonomy_state = *autonomy;
        packet.heartbeat = p;
        return true;
    }
    case EdgePacketType::POSE_STATE: {
        if (*payload_size != 3) { error = "pose_state payload shape mismatch"; return false; }
        PoseStatePayload p;
        const auto pos = reader.read_vec3();
        const auto vel = reader.read_vec3();
        const auto conf = reader.read_double();
        if (!pos || !vel || !conf) break;
        p.position = *pos;
        p.velocity = *vel;
        p.localization_confidence = *conf;
        packet.pose_state = p;
        return true;
    }
    case EdgePacketType::EDGE_HEALTH: {
        if (*payload_size != 5) { error = "edge_health payload shape mismatch"; return false; }
        EdgeHealthPayload p;
        const auto health = reader.read_string();
        const auto autonomy = reader.read_string();
        const auto consensus = reader.read_string();
        const auto bandwidth = reader.read_double();
        const auto disconnected = reader.read_bool();
        if (!health || !autonomy || !consensus || !bandwidth || !disconnected) break;
        p.edge_health_status = *health;
        p.autonomy_state = *autonomy;
        p.consensus_state = *consensus;
        p.mesh_bandwidth_kbps = *bandwidth;
        p.disconnected_operation = *disconnected;
        packet.edge_health = p;
        return true;
    }
    case EdgePacketType::OBSTACLE_DIGEST: {
        if (*payload_size != 4) { error = "obstacle_digest payload shape mismatch"; return false; }
        ObstacleDigestPayload p;
        const auto local = reader.read_int();
        const auto shared = reader.read_int();
        const auto freshness = reader.read_uint();
        const auto digest = reader.read_string();
        if (!local || !shared || !freshness || !digest) break;
        p.local_obstacle_count = static_cast<int>(*local);
        p.shared_obstacle_count = static_cast<int>(*shared);
        p.freshness_ms = static_cast<uint32_t>(*freshness);
        p.digest_id = *digest;
        packet.obstacle_digest = p;
        return true;
    }
    case EdgePacketType::THREAT_DIGEST: {
        if (*payload_size != 3) { error = "threat_digest payload shape mismatch"; return false; }
        ThreatDigestPayload p;
        const auto level = reader.read_string();
        const auto summary = reader.read_string();
        const auto confidence = reader.read_double();
        if (!level || !summary || !confidence) break;
        p.threat_level = *level;
        p.summary = *summary;
        p.confidence = *confidence;
        packet.threat_digest = p;
        return true;
    }
    case EdgePacketType::CONSENSUS_STATE: {
        if (*payload_size != 5) { error = "consensus_state payload shape mismatch"; return false; }
        ConsensusStatePayload p;
        const auto proposal = reader.read_uint();
        const auto state = reader.read_string();
        const auto epoch = reader.read_uint();
        const auto quorum = reader.read_int();
        const auto override = reader.read_bool();
        if (!proposal || !state || !epoch || !quorum || !override) break;
        if (*proposal > static_cast<uint8_t>(ConsensusProposalType::SPLIT_SWARM_RECOVERY_HINT)) {
            error = "unknown consensus proposal type";
            return false;
        }
        p.proposal_type = static_cast<ConsensusProposalType>(*proposal);
        p.consensus_state = *state;
        p.consensus_epoch = *epoch;
        p.quorum_count = static_cast<int>(*quorum);
        p.local_safety_override = *override;
        packet.consensus_state = p;
        return true;
    }
    case EdgePacketType::EMERGENCY_CORRIDOR: {
        if (*payload_size != 4) { error = "emergency_corridor payload shape mismatch"; return false; }
        EmergencyCorridorPayload p;
        const auto center = reader.read_vec3();
        const auto radius = reader.read_double();
        const auto hold = reader.read_uint();
        const auto summary = reader.read_string();
        if (!center || !radius || !hold || !summary) break;
        p.center = *center;
        p.radius_m = *radius;
        p.hold_ttl_ms = static_cast<uint32_t>(*hold);
        p.summary = *summary;
        packet.emergency_corridor = p;
        return true;
    }
    case EdgePacketType::PEER_GOODBYE: {
        if (*payload_size != 1) { error = "peer_goodbye payload shape mismatch"; return false; }
        PeerGoodbyePayload p;
        const auto reason = reader.read_string();
        if (!reason) break;
        p.reason = *reason;
        packet.peer_goodbye = p;
        return true;
    }
    }

    error = reader.error().empty() ? "malformed cbor payload" : reader.error();
    return false;
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

std::string_view to_string(EdgeSerializationMode mode) {
    switch (mode) {
    case EdgeSerializationMode::JSON: return "json";
    case EdgeSerializationMode::CBOR: return "cbor";
    case EdgeSerializationMode::PROTOBUF_PLACEHOLDER: return "protobuf_placeholder";
    }
    return "json";
}

std::optional<EdgeSerializationMode> parse_edge_serialization_mode(std::string_view value) {
    const auto normalized = lowercase(value);
    if (normalized == "json") return EdgeSerializationMode::JSON;
    if (normalized == "cbor") return EdgeSerializationMode::CBOR;
    if (normalized == "protobuf_placeholder") return EdgeSerializationMode::PROTOBUF_PLACEHOLDER;
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

std::vector<uint8_t> serialize_edge_packet_cbor(const EdgePeerPacket& packet) {
    std::vector<uint8_t> out;
    out.reserve(192);
    // Compact deterministic CBOR array:
    // [version, type, sender_id, timestamp_ms, sequence, trust_epoch, source, ttl_ms, auth_hook, payload_array]
    cbor_push_array(out, 10);
    cbor_push_uint(out, kCborPacketVersion);
    cbor_push_uint(out, static_cast<uint8_t>(packet.packet_type));
    cbor_push_uint(out, packet.sender_id);
    cbor_push_uint(out, packet.timestamp_ms);
    cbor_push_uint(out, packet.sequence_number);
    cbor_push_uint(out, packet.trust_epoch);
    cbor_push_string(out, normalize_edge_source_tag(packet.source));
    cbor_push_uint(out, packet.ttl_ms);
    cbor_push_string(out, packet.auth_hook);
    write_cbor_payload(out, packet);
    return out;
}

EdgePacketParseResult parse_edge_packet_cbor(std::span<const uint8_t> wire) {
    EdgePacketParseResult result;
    if (wire.empty()) {
        result.error = "malformed cbor packet";
        return result;
    }

    CborReader reader(wire);
    const auto top_size = reader.read_array_size();
    if (!top_size.has_value() || *top_size != 10) {
        result.error = reader.error().empty() ? "malformed cbor packet" : reader.error();
        return result;
    }

    const auto version = reader.read_uint();
    const auto type_value = reader.read_uint();
    const auto sender_id = reader.read_uint();
    const auto timestamp_ms = reader.read_uint();
    const auto sequence_number = reader.read_uint();
    const auto trust_epoch = reader.read_uint();
    const auto source = reader.read_string();
    const auto ttl_ms = reader.read_uint();
    const auto auth_hook = reader.read_string();
    if (!version || !type_value || !sender_id || !timestamp_ms || !sequence_number ||
        !trust_epoch || !source || !ttl_ms || !auth_hook) {
        result.error = reader.error().empty() ? "missing cbor header field" : reader.error();
        return result;
    }
    if (*version != kCborPacketVersion) {
        result.error = "unsupported cbor packet version";
        return result;
    }
    if (*type_value > static_cast<uint8_t>(EdgePacketType::PEER_GOODBYE)) {
        result.error = "unknown packet_type";
        return result;
    }

    result.packet.packet_type = static_cast<EdgePacketType>(*type_value);
    result.packet.sender_id = static_cast<uint32_t>(*sender_id);
    result.packet.timestamp_ms = *timestamp_ms;
    result.packet.sequence_number = static_cast<uint32_t>(*sequence_number);
    result.packet.trust_epoch = *trust_epoch;
    result.packet.source = normalize_edge_source_tag(*source);
    result.packet.ttl_ms = static_cast<uint32_t>(*ttl_ms);
    result.packet.auth_hook = *auth_hook;

    std::string payload_error;
    if (!read_cbor_payload(reader, result.packet, payload_error)) {
        result.error = payload_error;
        return result;
    }
    if (!reader.eof()) {
        result.error = "trailing cbor bytes rejected";
        return result;
    }
    result.ok = true;
    return result;
}

std::vector<uint8_t> serialize_edge_packet(
    const EdgePeerPacket& packet,
    EdgeSerializationMode mode,
    EdgeSerializationMetrics* metrics) {
    const auto started = std::chrono::steady_clock::now();
    std::vector<uint8_t> wire;
    if (mode == EdgeSerializationMode::CBOR) {
        wire = serialize_edge_packet_cbor(packet);
    } else {
        const auto json = serialize_edge_packet_json(packet);
        wire.assign(json.begin(), json.end());
    }
    const auto finished = std::chrono::steady_clock::now();

    if (metrics != nullptr) {
        const auto json_size = serialize_edge_packet_json(packet).size();
        metrics->mode = mode;
        metrics->encoded_packet_size_bytes = wire.size();
        metrics->json_equivalent_size_bytes = json_size;
        metrics->serialization_time_us =
            std::chrono::duration<double, std::micro>(finished - started).count();
        metrics->compression_ratio_vs_json =
            json_size == 0 ? 1.0 : static_cast<double>(wire.size()) / static_cast<double>(json_size);
        metrics->estimated = false;
    }
    return wire;
}

EdgePacketParseResult parse_edge_packet(
    std::span<const uint8_t> wire,
    EdgeSerializationMode mode,
    EdgeSerializationMetrics* metrics) {
    const auto started = std::chrono::steady_clock::now();
    EdgePacketParseResult parsed;
    if (mode == EdgeSerializationMode::CBOR) {
        parsed = parse_edge_packet_cbor(wire);
    } else {
        parsed = parse_edge_packet_json(std::string_view(
            reinterpret_cast<const char*>(wire.data()),
            wire.size()));
    }
    const auto finished = std::chrono::steady_clock::now();

    if (metrics != nullptr) {
        metrics->mode = mode;
        metrics->encoded_packet_size_bytes = wire.size();
        metrics->deserialization_time_us =
            std::chrono::duration<double, std::micro>(finished - started).count();
        if (parsed.ok) {
            const auto json_size = serialize_edge_packet_json(parsed.packet).size();
            metrics->json_equivalent_size_bytes = json_size;
            metrics->compression_ratio_vs_json =
                json_size == 0 ? 1.0 : static_cast<double>(wire.size()) / static_cast<double>(json_size);
        }
        metrics->estimated = false;
    }
    return parsed;
}

EdgePacketValidationResult validate_edge_packet(
    const EdgePeerPacket& packet,
    const EdgePacketValidationOptions& options) {
    EdgePacketValidationResult result;

    const size_t packet_size = options.serialized_packet_size_bytes == 0
        ? serialize_edge_packet_json(packet).size()
        : options.serialized_packet_size_bytes;
    if (packet_size > options.max_packet_size_bytes) {
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
