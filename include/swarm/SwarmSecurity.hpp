// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "swarm/V2XMeshNetwork.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone::swarm {

struct SwarmSecurityConfig {
    bool enabled{false};
    std::string swarm_secret;
    uint32_t pbkdf2_iterations{120000};
};

class SwarmSecurityContext {
public:
    explicit SwarmSecurityContext(uint32_t local_id, SwarmSecurityConfig cfg);

    [[nodiscard]] bool enabled() const {
        return cfg_.enabled && !cfg_.swarm_secret.empty();
    }
    [[nodiscard]] std::string last_error() const {
        return last_error_;
    }

    [[nodiscard]] std::vector<uint8_t> seal(const SwarmMessage& msg);
    [[nodiscard]] std::optional<SwarmMessage> open(const uint8_t* data, size_t len);

    void reset();

private:
    struct KeyMaterial {
        std::array<uint8_t, 32> enc_key{};
        std::array<uint8_t, 32> mac_key{};
        std::array<uint8_t, 32> eddsa_seed{};
        std::array<uint8_t, 32> future_key{};
        std::array<uint8_t, 64> secret_key{};
        std::array<uint8_t, 32> public_key{};
    };

    struct PeerState {
        uint32_t last_seq{0};
        std::array<uint8_t, 32> last_frame_hash{};
        std::array<uint8_t, 32> last_ledger_hash{};
        bool has_last_frame{false};
        bool has_last_ledger{false};
    };

    [[nodiscard]] bool is_ledger_message(SwarmMessage::Type type) const;
    [[nodiscard]] const KeyMaterial& keys_for(uint32_t node_id);
    [[nodiscard]] std::vector<uint8_t>
    build_header(uint32_t src_id, uint32_t seq_num, uint64_t issued_ns, uint8_t flags,
                 const std::array<uint8_t, 16>& iv, const std::array<uint8_t, 20>& past_sha1,
                 const std::array<uint8_t, 32>& present_sha256,
                 const std::array<uint8_t, 32>& future_sha3,
                 const std::array<uint8_t, 32>& chain_prev_hash, uint32_t cipher_len) const;

    [[nodiscard]] std::array<uint8_t, 32> sha256_bytes(const std::vector<uint8_t>& bytes) const;
    [[nodiscard]] std::array<uint8_t, 20> sha1_bytes(const uint8_t* data, size_t len) const;
    [[nodiscard]] std::array<uint8_t, 32> sha256_bytes(const uint8_t* data, size_t len) const;
    [[nodiscard]] std::array<uint8_t, 32> sha3_256_bytes(const uint8_t* data, size_t len) const;
    [[nodiscard]] std::array<uint8_t, 32> hmac_sha256(const std::array<uint8_t, 32>& key,
                                                      const uint8_t* data, size_t len) const;
    [[nodiscard]] std::vector<uint8_t> aes256_encrypt(const std::array<uint8_t, 32>& key,
                                                      const std::array<uint8_t, 16>& iv,
                                                      const std::vector<uint8_t>& plain) const;
    [[nodiscard]] std::optional<std::vector<uint8_t>>
    aes256_decrypt(const std::array<uint8_t, 32>& key, const std::array<uint8_t, 16>& iv,
                   const uint8_t* cipher, size_t len) const;
    [[nodiscard]] KeyMaterial derive_key_material(uint32_t node_id) const;
    [[nodiscard]] std::array<uint8_t, 16> random_iv() const;
    void set_error(std::string error) const;

    uint32_t local_id_;
    SwarmSecurityConfig cfg_;
    mutable std::string last_error_;
    mutable std::unordered_map<uint32_t, KeyMaterial> key_cache_;
    std::unordered_map<uint32_t, PeerState> peer_state_;
};

} // namespace drone::swarm
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
