#pragma once

#include "swarm/EdgePeerProtocol.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace drone::security {

enum class AuthMode : uint8_t {
    NONE = 0,
    HMAC_SHA256,
    ED25519_PLACEHOLDER,
    PQC_HYBRID_PLACEHOLDER,
    PER_NODE_HMAC_SHA256,
};

enum class PacketAuthResult : uint8_t {
    ACCEPTED = 0,
    REJECTED,
    UNSUPPORTED,
    MISSING_SIGNATURE,
    INVALID_SIGNATURE,
    STALE_EPOCH,
    REPLAY_DETECTED,
    INVALID_TTL,
    FUTURE_TIMESTAMP,
    STALE_TIMESTAMP,
    UNTRUSTED_SENDER,
};

struct PacketAuthConfig {
    AuthMode mode{AuthMode::HMAC_SHA256};
    bool allow_unsigned_in_simulation{true};
    bool allow_unsigned_in_bench{false};
    uint64_t trust_epoch{1};
    uint64_t max_clock_skew_ms{2000};
    uint32_t max_packet_ttl_ms{5000};
    std::string shared_secret{};
    std::string shared_secret_env{"DRONE_SWARM_SECRET"};
    // Strict per-node HMAC mode requires a local signing identity and a distinct
    // verification secret for each accepted sender. Never falls back to group HMAC.
    uint32_t local_node_id{0};
    std::string local_signing_secret{};
    std::unordered_map<uint32_t, std::string> trusted_node_secrets{};
    std::string runtime_profile{"edge_swarm"};
    // Optional durable replay state on trusted, single-writer local storage.
    std::string replay_ledger_path{};
    bool initialize_new_replay_ledger{false};
};

struct PacketAuthOutcome {
    PacketAuthResult result{PacketAuthResult::REJECTED};
    std::string reason{};

    [[nodiscard]] bool accepted() const {
        return result == PacketAuthResult::ACCEPTED;
    }
};

[[nodiscard]] std::string_view to_string(AuthMode mode);
[[nodiscard]] std::optional<AuthMode> parse_auth_mode(std::string_view value);
[[nodiscard]] std::string_view to_string(PacketAuthResult result);

// Exposed for reproducible, in-process measurements of the HMAC primitive.
using HmacDigest = std::array<uint8_t, 32>;
[[nodiscard]] HmacDigest hmacSha256Digest(std::string_view secret,
                                          const std::vector<uint8_t>& data);
[[nodiscard]] std::array<uint8_t, 32>
canonicalPayloadHash(const drone::swarm::EdgePeerPacket& packet);
[[nodiscard]] PacketAuthOutcome validateTrustEpoch(const drone::swarm::EdgePeerPacket& packet,
                                                   const PacketAuthConfig& config);
[[nodiscard]] PacketAuthOutcome
validateNonceOrSequence(const drone::swarm::EdgePeerPacket& packet,
                        std::optional<uint32_t> last_sequence_number);
[[nodiscard]] PacketAuthOutcome signPacket(drone::swarm::EdgePeerPacket& packet,
                                           const PacketAuthConfig& config);
[[nodiscard]] PacketAuthOutcome
verifyPacket(const drone::swarm::EdgePeerPacket& packet, const PacketAuthConfig& config,
             uint64_t now_ms, std::optional<uint32_t> last_sequence_number = std::nullopt);

} // namespace drone::security
