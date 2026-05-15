#pragma once

#include "swarm/EdgePeerProtocol.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace drone::security {

enum class AuthMode : uint8_t {
    NONE = 0,
    HMAC_SHA256,
    ED25519_PLACEHOLDER,
    PQC_HYBRID_PLACEHOLDER,
};

enum class PacketAuthResult : uint8_t {
    ACCEPTED = 0,
    REJECTED,
    UNSUPPORTED,
    MISSING_SIGNATURE,
    INVALID_SIGNATURE,
    STALE_EPOCH,
    REPLAY_DETECTED,
};

struct PacketAuthConfig {
    AuthMode mode{AuthMode::HMAC_SHA256};
    bool allow_unsigned_in_simulation{true};
    bool allow_unsigned_in_bench{false};
    uint64_t trust_epoch{1};
    uint64_t max_clock_skew_ms{2000};
    std::string shared_secret{};
    std::string shared_secret_env{"DRONE_SWARM_SECRET"};
    std::string runtime_profile{"edge_swarm"};
};

struct PacketAuthOutcome {
    PacketAuthResult result{PacketAuthResult::REJECTED};
    std::string reason{};

    [[nodiscard]] bool accepted() const { return result == PacketAuthResult::ACCEPTED; }
};

[[nodiscard]] std::string_view to_string(AuthMode mode);
[[nodiscard]] std::optional<AuthMode> parse_auth_mode(std::string_view value);
[[nodiscard]] std::string_view to_string(PacketAuthResult result);

[[nodiscard]] std::array<uint8_t, 32> canonicalPayloadHash(const drone::swarm::EdgePeerPacket& packet);
[[nodiscard]] PacketAuthOutcome validateTrustEpoch(const drone::swarm::EdgePeerPacket& packet,
                                                   const PacketAuthConfig& config);
[[nodiscard]] PacketAuthOutcome validateNonceOrSequence(const drone::swarm::EdgePeerPacket& packet,
                                                        std::optional<uint32_t> last_sequence_number);
[[nodiscard]] PacketAuthOutcome signPacket(drone::swarm::EdgePeerPacket& packet,
                                           const PacketAuthConfig& config);
[[nodiscard]] PacketAuthOutcome verifyPacket(const drone::swarm::EdgePeerPacket& packet,
                                             const PacketAuthConfig& config,
                                             uint64_t now_ms,
                                             std::optional<uint32_t> last_sequence_number = std::nullopt);

} // namespace drone::security
