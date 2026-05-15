#include "security/PeerPacketAuth.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#pragma comment(lib, "bcrypt.lib")
#endif

namespace drone::security {

namespace {

std::string lowercase(std::string_view value) {
    std::string out(value.begin(), value.end());
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

std::string hex_encode(const uint8_t* data, size_t len) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i) {
        oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::optional<std::vector<uint8_t>> hex_decode(std::string_view value) {
    if ((value.size() % 2u) != 0u) {
        return std::nullopt;
    }
    std::vector<uint8_t> out;
    out.reserve(value.size() / 2u);
    auto hex_value = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (size_t i = 0; i < value.size(); i += 2) {
        const int hi = hex_value(value[i]);
        const int lo = hex_value(value[i + 1]);
        if (hi < 0 || lo < 0) {
            return std::nullopt;
        }
        out.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return out;
}

bool constant_time_equal(const std::vector<uint8_t>& lhs, const std::array<uint8_t, 32>& rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    uint8_t diff = 0;
    for (size_t i = 0; i < rhs.size(); ++i) {
        diff |= static_cast<uint8_t>(lhs[i] ^ rhs[i]);
    }
    return diff == 0;
}

std::string effective_secret(const PacketAuthConfig& config) {
    if (!config.shared_secret.empty()) {
        return config.shared_secret;
    }
    if (!config.shared_secret_env.empty()) {
#ifdef _WIN32
        char* env = nullptr;
        size_t len = 0;
        if (_dupenv_s(&env, &len, config.shared_secret_env.c_str()) == 0 && env != nullptr) {
            std::string out(env);
            std::free(env);
            return out;
        }
#else
        if (const char* env = std::getenv(config.shared_secret_env.c_str())) {
            return std::string(env);
        }
#endif
    }
    return {};
}

#ifdef _WIN32
std::array<uint8_t, 32> bcrypt_sha256(const uint8_t* data, size_t len) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    DWORD obj_len = 0;
    DWORD result = 0;
    std::array<uint8_t, 32> out{};
    if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0) {
        throw std::runtime_error("BCryptOpenAlgorithmProvider SHA256 failed");
    }
    if (BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&obj_len),
                          sizeof(obj_len), &result, 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptGetProperty SHA256 failed");
    }
    std::vector<uint8_t> hash_object(obj_len);
    if (BCryptCreateHash(alg, &hash, hash_object.data(), obj_len, nullptr, 0, 0) < 0 ||
        BCryptHashData(hash, const_cast<PUCHAR>(data), static_cast<ULONG>(len), 0) < 0 ||
        BCryptFinishHash(hash, out.data(), static_cast<ULONG>(out.size()), 0) < 0) {
        if (hash) BCryptDestroyHash(hash);
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCrypt SHA256 failed");
    }
    BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(alg, 0);
    return out;
}

std::array<uint8_t, 32> bcrypt_hmac_sha256(std::string_view key, const uint8_t* data, size_t len) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    DWORD obj_len = 0;
    DWORD result = 0;
    std::array<uint8_t, 32> out{};
    if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr,
                                    BCRYPT_ALG_HANDLE_HMAC_FLAG) < 0) {
        throw std::runtime_error("BCryptOpenAlgorithmProvider HMAC failed");
    }
    if (BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&obj_len),
                          sizeof(obj_len), &result, 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptGetProperty HMAC failed");
    }
    std::vector<uint8_t> hash_object(obj_len);
    if (BCryptCreateHash(alg, &hash, hash_object.data(), obj_len,
                         reinterpret_cast<PUCHAR>(const_cast<char*>(key.data())),
                         static_cast<ULONG>(key.size()), 0) < 0 ||
        BCryptHashData(hash, const_cast<PUCHAR>(data), static_cast<ULONG>(len), 0) < 0 ||
        BCryptFinishHash(hash, out.data(), static_cast<ULONG>(out.size()), 0) < 0) {
        if (hash) BCryptDestroyHash(hash);
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCrypt HMAC failed");
    }
    BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(alg, 0);
    return out;
}
#else
uint32_t rotr(uint32_t value, uint32_t bits) {
    return (value >> bits) | (value << (32u - bits));
}

std::array<uint8_t, 32> portable_sha256(const uint8_t* data, size_t len) {
    static constexpr std::array<uint32_t, 64> k{
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
    std::array<uint32_t, 8> h{
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    std::vector<uint8_t> msg(data, data + len);
    const uint64_t bit_len = static_cast<uint64_t>(len) * 8u;
    msg.push_back(0x80u);
    while ((msg.size() % 64u) != 56u) {
        msg.push_back(0u);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        msg.push_back(static_cast<uint8_t>((bit_len >> shift) & 0xFFu));
    }
    for (size_t offset = 0; offset < msg.size(); offset += 64u) {
        std::array<uint32_t, 64> w{};
        for (size_t i = 0; i < 16; ++i) {
            const size_t j = offset + (i * 4u);
            w[i] = (static_cast<uint32_t>(msg[j]) << 24u) |
                   (static_cast<uint32_t>(msg[j + 1]) << 16u) |
                   (static_cast<uint32_t>(msg[j + 2]) << 8u) |
                   static_cast<uint32_t>(msg[j + 3]);
        }
        for (size_t i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3u);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10u);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (size_t i = 0; i < 64; ++i) {
            const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + s1 + ch + k[i] + w[i];
            const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = s0 + maj;
            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
        h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }
    std::array<uint8_t, 32> out{};
    for (size_t i = 0; i < h.size(); ++i) {
        out[i * 4u] = static_cast<uint8_t>((h[i] >> 24u) & 0xFFu);
        out[i * 4u + 1u] = static_cast<uint8_t>((h[i] >> 16u) & 0xFFu);
        out[i * 4u + 2u] = static_cast<uint8_t>((h[i] >> 8u) & 0xFFu);
        out[i * 4u + 3u] = static_cast<uint8_t>(h[i] & 0xFFu);
    }
    return out;
}
#endif

std::array<uint8_t, 32> sha256_bytes(const std::vector<uint8_t>& data) {
#ifdef _WIN32
    return bcrypt_sha256(data.data(), data.size());
#else
    return portable_sha256(data.data(), data.size());
#endif
}

std::array<uint8_t, 32> hmac_sha256_bytes(std::string_view secret, const std::vector<uint8_t>& data) {
#ifdef _WIN32
    return bcrypt_hmac_sha256(secret, data.data(), data.size());
#else
    std::vector<uint8_t> key(secret.begin(), secret.end());
    if (key.size() > 64u) {
        const auto digest = portable_sha256(key.data(), key.size());
        key.assign(digest.begin(), digest.end());
    }
    key.resize(64u, 0u);
    std::vector<uint8_t> inner(64u + data.size());
    std::vector<uint8_t> outer(64u + 32u);
    for (size_t i = 0; i < 64u; ++i) {
        inner[i] = static_cast<uint8_t>(key[i] ^ 0x36u);
        outer[i] = static_cast<uint8_t>(key[i] ^ 0x5cu);
    }
    std::copy(data.begin(), data.end(), inner.begin() + 64);
    const auto inner_digest = portable_sha256(inner.data(), inner.size());
    std::copy(inner_digest.begin(), inner_digest.end(), outer.begin() + 64);
    return portable_sha256(outer.data(), outer.size());
#endif
}

std::vector<uint8_t> canonical_bytes(const drone::swarm::EdgePeerPacket& packet) {
    auto canonical = packet;
    canonical.auth_hook.clear();
    return drone::swarm::serialize_edge_packet_cbor(canonical);
}

bool unsigned_allowed_for_profile(const drone::swarm::EdgePeerPacket& packet,
                                  const PacketAuthConfig& config) {
    const std::string profile = lowercase(config.runtime_profile);
    if (profile == "simulation" && config.allow_unsigned_in_simulation &&
        drone::swarm::normalize_edge_source_tag(packet.source) == "simulation") {
        return true;
    }
    if ((profile == "bench" || profile == "hil") && config.allow_unsigned_in_bench) {
        return true;
    }
    return false;
}

PacketAuthOutcome outcome(PacketAuthResult result, std::string reason) {
    return {result, std::move(reason)};
}

} // namespace

std::string_view to_string(AuthMode mode) {
    switch (mode) {
    case AuthMode::NONE: return "none";
    case AuthMode::HMAC_SHA256: return "hmac_sha256";
    case AuthMode::ED25519_PLACEHOLDER: return "ed25519_placeholder";
    case AuthMode::PQC_HYBRID_PLACEHOLDER: return "pqc_hybrid_placeholder";
    }
    return "unknown";
}

std::optional<AuthMode> parse_auth_mode(std::string_view value) {
    const auto lower = lowercase(value);
    if (lower == "none") return AuthMode::NONE;
    if (lower == "hmac_sha256") return AuthMode::HMAC_SHA256;
    if (lower == "ed25519_placeholder") return AuthMode::ED25519_PLACEHOLDER;
    if (lower == "pqc_hybrid_placeholder") return AuthMode::PQC_HYBRID_PLACEHOLDER;
    return std::nullopt;
}

std::string_view to_string(PacketAuthResult result) {
    switch (result) {
    case PacketAuthResult::ACCEPTED: return "accepted";
    case PacketAuthResult::REJECTED: return "rejected";
    case PacketAuthResult::UNSUPPORTED: return "unsupported";
    case PacketAuthResult::MISSING_SIGNATURE: return "missing_signature";
    case PacketAuthResult::INVALID_SIGNATURE: return "invalid_signature";
    case PacketAuthResult::STALE_EPOCH: return "stale_epoch";
    case PacketAuthResult::REPLAY_DETECTED: return "replay_detected";
    }
    return "unknown";
}

std::array<uint8_t, 32> canonicalPayloadHash(const drone::swarm::EdgePeerPacket& packet) {
    return sha256_bytes(canonical_bytes(packet));
}

PacketAuthOutcome validateTrustEpoch(const drone::swarm::EdgePeerPacket& packet,
                                     const PacketAuthConfig& config) {
    if (packet.trust_epoch != config.trust_epoch) {
        return outcome(PacketAuthResult::STALE_EPOCH, "trust epoch mismatch");
    }
    return outcome(PacketAuthResult::ACCEPTED, "trust epoch accepted");
}

PacketAuthOutcome validateNonceOrSequence(const drone::swarm::EdgePeerPacket& packet,
                                          std::optional<uint32_t> last_sequence_number) {
    if (last_sequence_number.has_value() && packet.sequence_number <= *last_sequence_number) {
        return outcome(PacketAuthResult::REPLAY_DETECTED, "stale sequence number");
    }
    return outcome(PacketAuthResult::ACCEPTED, "sequence accepted");
}

PacketAuthOutcome signPacket(drone::swarm::EdgePeerPacket& packet,
                             const PacketAuthConfig& config) {
    if (config.mode == AuthMode::NONE) {
        packet.auth_hook = "unsigned";
        return outcome(PacketAuthResult::ACCEPTED, "unsigned packet emitted");
    }
    if (config.mode == AuthMode::PQC_HYBRID_PLACEHOLDER) {
        return outcome(PacketAuthResult::UNSUPPORTED, "PQC hybrid auth is roadmap-only in this build.");
    }
    if (config.mode == AuthMode::ED25519_PLACEHOLDER) {
        return outcome(PacketAuthResult::UNSUPPORTED, "Ed25519 auth is placeholder-only in this build.");
    }
    const auto secret = effective_secret(config);
    if (secret.empty()) {
        return outcome(PacketAuthResult::MISSING_SIGNATURE, "HMAC shared secret missing");
    }
    const auto mac = hmac_sha256_bytes(secret, canonical_bytes(packet));
    packet.auth_hook = std::string("hmac_sha256:") + hex_encode(mac.data(), mac.size());
    return outcome(PacketAuthResult::ACCEPTED, "packet signed with hmac_sha256");
}

PacketAuthOutcome verifyPacket(const drone::swarm::EdgePeerPacket& packet,
                               const PacketAuthConfig& config,
                               uint64_t now_ms,
                               std::optional<uint32_t> last_sequence_number) {
    if (config.mode == AuthMode::PQC_HYBRID_PLACEHOLDER) {
        return outcome(PacketAuthResult::UNSUPPORTED, "PQC hybrid auth is roadmap-only in this build.");
    }
    if (config.mode == AuthMode::ED25519_PLACEHOLDER) {
        return outcome(PacketAuthResult::UNSUPPORTED, "Ed25519 auth is placeholder-only in this build.");
    }
    const auto age = now_ms > packet.timestamp_ms ? now_ms - packet.timestamp_ms : packet.timestamp_ms - now_ms;
    if (config.max_clock_skew_ms > 0 && age > config.max_clock_skew_ms + packet.ttl_ms) {
        return outcome(PacketAuthResult::REJECTED, "packet timestamp outside auth clock skew");
    }
    if (auto epoch = validateTrustEpoch(packet, config); !epoch.accepted()) {
        return epoch;
    }
    if (auto replay = validateNonceOrSequence(packet, last_sequence_number); !replay.accepted()) {
        return replay;
    }
    if (config.mode == AuthMode::NONE) {
        if (unsigned_allowed_for_profile(packet, config)) {
            return outcome(PacketAuthResult::ACCEPTED, "unsigned packet accepted for simulation/debug");
        }
        return outcome(PacketAuthResult::MISSING_SIGNATURE, "unsigned peer packets are not allowed in this runtime profile");
    }
    if (packet.auth_hook.empty() || packet.auth_hook == "unsigned") {
        return outcome(PacketAuthResult::MISSING_SIGNATURE, "missing packet signature");
    }
    constexpr std::string_view prefix = "hmac_sha256:";
    if (packet.auth_hook.rfind(prefix, 0) != 0) {
        return outcome(PacketAuthResult::INVALID_SIGNATURE, "packet signature algorithm mismatch");
    }
    const auto provided = hex_decode(std::string_view(packet.auth_hook).substr(prefix.size()));
    if (!provided.has_value()) {
        return outcome(PacketAuthResult::INVALID_SIGNATURE, "packet signature encoding invalid");
    }
    const auto secret = effective_secret(config);
    if (secret.empty()) {
        return outcome(PacketAuthResult::MISSING_SIGNATURE, "HMAC shared secret missing");
    }
    const auto expected = hmac_sha256_bytes(secret, canonical_bytes(packet));
    if (!constant_time_equal(*provided, expected)) {
        return outcome(PacketAuthResult::INVALID_SIGNATURE, "packet signature verification failed");
    }
    return outcome(PacketAuthResult::ACCEPTED, "packet authentication accepted");
}

} // namespace drone::security
