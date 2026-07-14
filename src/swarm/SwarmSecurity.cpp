// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "swarm/SwarmSecurity.hpp"
#include "utils/RuntimeLogging.hpp"

extern "C" {
#include "monocypher.h"
#include "sha3.h"
}

#include <algorithm>
#include <array>
#include <cstring>
#include <random>
#include <stdexcept>
#include <utility>
#include <string_view>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#pragma comment(lib, "bcrypt.lib")
#else
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/sha.h>
#endif

namespace drone::swarm {

namespace {

constexpr uint32_t kSecureMagic = 0x53325853; // "SX2S"
constexpr uint8_t kSecureVersion = 1;
constexpr uint8_t kFlagLedger = 0x01;
constexpr size_t kMacSize = 32;
constexpr size_t kSignatureSize = 64;
constexpr size_t kHeaderSize = 4 + 1 + 1 + 2 + 4 + 4 + 8 + 4 + 16 + 20 + 32 + 32 + 32;

template <size_t N>
void append_array(std::vector<uint8_t>& out, const std::array<uint8_t, N>& value) {
    out.insert(out.end(), value.begin(), value.end());
}

void append_u32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void append_u64(std::vector<uint8_t>& out, uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
    }
}

uint32_t read_u32(const uint8_t* data, size_t offset) {
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

uint64_t read_u64(const uint8_t* data, size_t offset) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(data[offset + i]) << (8 * i);
    }
    return value;
}

std::vector<uint8_t> concat_bytes(std::initializer_list<std::pair<const uint8_t*, size_t>> parts) {
    size_t total = 0;
    for (const auto& [ptr, len] : parts) {
        total += len;
    }
    std::vector<uint8_t> out;
    out.reserve(total);
    for (const auto& [ptr, len] : parts) {
        out.insert(out.end(), ptr, ptr + len);
    }
    return out;
}

#ifdef _WIN32

std::vector<uint8_t> bcrypt_hash(LPCWSTR algorithm,
                                 ULONG flags,
                                 const uint8_t* key,
                                 size_t key_len,
                                 const uint8_t* data,
                                 size_t len,
                                 size_t out_len) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    DWORD obj_len = 0;
    DWORD result = 0;
    if (BCryptOpenAlgorithmProvider(&alg, algorithm, nullptr, flags) < 0) {
        throw std::runtime_error("BCryptOpenAlgorithmProvider failed");
    }
    if (BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&obj_len),
                          sizeof(obj_len), &result, 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptGetProperty failed");
    }

    std::vector<uint8_t> hash_object(obj_len);
    std::vector<uint8_t> out(out_len);
    if (BCryptCreateHash(alg, &hash, hash_object.data(), obj_len,
                         const_cast<PUCHAR>(key), static_cast<ULONG>(key_len), 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptCreateHash failed");
    }
    if (BCryptHashData(hash, const_cast<PUCHAR>(data), static_cast<ULONG>(len), 0) < 0 ||
        BCryptFinishHash(hash, out.data(), static_cast<ULONG>(out.size()), 0) < 0) {
        BCryptDestroyHash(hash);
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCrypt hash failed");
    }
    BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(alg, 0);
    return out;
}

std::vector<uint8_t> pbkdf2_sha256(std::string_view passphrase,
                                   const uint8_t* salt,
                                   size_t salt_len,
                                   uint32_t iterations,
                                   size_t out_len) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr,
                                    BCRYPT_ALG_HANDLE_HMAC_FLAG) < 0) {
        throw std::runtime_error("BCryptOpenAlgorithmProvider PBKDF2 failed");
    }
    std::vector<uint8_t> out(out_len);
    if (BCryptDeriveKeyPBKDF2(alg,
                              reinterpret_cast<PUCHAR>(const_cast<char*>(passphrase.data())),
                              static_cast<ULONG>(passphrase.size()),
                              const_cast<PUCHAR>(salt),
                              static_cast<ULONG>(salt_len),
                              iterations,
                              out.data(),
                              static_cast<ULONG>(out.size()),
                              0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptDeriveKeyPBKDF2 failed");
    }
    BCryptCloseAlgorithmProvider(alg, 0);
    return out;
}

std::vector<uint8_t> aes256_crypt(bool encrypt,
                                  const std::array<uint8_t, 32>& key_bytes,
                                  const std::array<uint8_t, 16>& iv_bytes,
                                  const uint8_t* input,
                                  size_t input_len) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_KEY_HANDLE key = nullptr;
    DWORD obj_len = 0;
    DWORD result = 0;
    DWORD out_len = 0;

    if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_AES_ALGORITHM, nullptr, 0) < 0) {
        throw std::runtime_error("BCryptOpenAlgorithmProvider AES failed");
    }
    const auto* chain_mode = reinterpret_cast<const uint8_t*>(BCRYPT_CHAIN_MODE_CBC);
    const ULONG chain_mode_len = static_cast<ULONG>((wcslen(BCRYPT_CHAIN_MODE_CBC) + 1) * sizeof(wchar_t));
    if (BCryptSetProperty(alg, BCRYPT_CHAINING_MODE,
                          const_cast<PUCHAR>(chain_mode), chain_mode_len, 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptSetProperty CBC failed");
    }
    if (BCryptGetProperty(alg, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&obj_len),
                          sizeof(obj_len), &result, 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptGetProperty AES failed");
    }

    std::vector<uint8_t> key_object(obj_len);
    std::vector<uint8_t> iv(iv_bytes.begin(), iv_bytes.end());
    if (BCryptGenerateSymmetricKey(alg, &key, key_object.data(), obj_len,
                                   const_cast<PUCHAR>(key_bytes.data()),
                                   static_cast<ULONG>(key_bytes.size()), 0) < 0) {
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCryptGenerateSymmetricKey failed");
    }

    ULONG flags = BCRYPT_BLOCK_PADDING;
    NTSTATUS status = encrypt
        ? BCryptEncrypt(key, const_cast<PUCHAR>(input), static_cast<ULONG>(input_len), nullptr,
                        iv.data(), static_cast<ULONG>(iv.size()), nullptr, 0, &out_len, flags)
        : BCryptDecrypt(key, const_cast<PUCHAR>(input), static_cast<ULONG>(input_len), nullptr,
                        iv.data(), static_cast<ULONG>(iv.size()), nullptr, 0, &out_len, flags);
    if (status < 0) {
        BCryptDestroyKey(key);
        BCryptCloseAlgorithmProvider(alg, 0);
        throw std::runtime_error("BCrypt preflight failed");
    }

    std::vector<uint8_t> out(out_len);
    status = encrypt
        ? BCryptEncrypt(key, const_cast<PUCHAR>(input), static_cast<ULONG>(input_len), nullptr,
                        iv.data(), static_cast<ULONG>(iv.size()),
                        out.data(), static_cast<ULONG>(out.size()), &out_len, flags)
        : BCryptDecrypt(key, const_cast<PUCHAR>(input), static_cast<ULONG>(input_len), nullptr,
                        iv.data(), static_cast<ULONG>(iv.size()),
                        out.data(), static_cast<ULONG>(out.size()), &out_len, flags);
    BCryptDestroyKey(key);
    BCryptCloseAlgorithmProvider(alg, 0);
    if (status < 0) {
        throw std::runtime_error("BCrypt crypt failed");
    }
    out.resize(out_len);
    return out;
}

#endif

#ifndef _WIN32

std::vector<uint8_t> evp_digest(const EVP_MD* algorithm,
                                const uint8_t* data,
                                size_t len,
                                size_t out_len) {
    std::vector<uint8_t> out(out_len);
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        throw std::runtime_error("EVP_MD_CTX_new failed");
    }

    unsigned int digest_len = 0;
    if (EVP_DigestInit_ex(ctx, algorithm, nullptr) != 1 ||
        EVP_DigestUpdate(ctx, data, len) != 1 ||
        EVP_DigestFinal_ex(ctx, out.data(), &digest_len) != 1) {
        EVP_MD_CTX_free(ctx);
        throw std::runtime_error("EVP digest failed");
    }
    EVP_MD_CTX_free(ctx);
    out.resize(digest_len);
    return out;
}

std::vector<uint8_t> hmac_sha256_bytes(const uint8_t* key,
                                       size_t key_len,
                                       const uint8_t* data,
                                       size_t len,
                                       size_t out_len) {
    std::vector<uint8_t> out(out_len);
    unsigned int digest_len = 0;
    if (!HMAC(EVP_sha256(),
              key,
              static_cast<int>(key_len),
              data,
              len,
              out.data(),
              &digest_len)) {
        throw std::runtime_error("HMAC failed");
    }
    out.resize(digest_len);
    return out;
}

std::vector<uint8_t> pbkdf2_sha256(std::string_view passphrase,
                                   const uint8_t* salt,
                                   size_t salt_len,
                                   uint32_t iterations,
                                   size_t out_len) {
    std::vector<uint8_t> out(out_len);
    if (PKCS5_PBKDF2_HMAC(passphrase.data(),
                          static_cast<int>(passphrase.size()),
                          salt,
                          static_cast<int>(salt_len),
                          static_cast<int>(iterations),
                          EVP_sha256(),
                          static_cast<int>(out.size()),
                          out.data()) != 1) {
        throw std::runtime_error("PKCS5_PBKDF2_HMAC failed");
    }
    return out;
}

std::vector<uint8_t> aes256_crypt(bool encrypt,
                                  const std::array<uint8_t, 32>& key_bytes,
                                  const std::array<uint8_t, 16>& iv_bytes,
                                  const uint8_t* input,
                                  size_t input_len) {
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("EVP_CIPHER_CTX_new failed");
    }

    const EVP_CIPHER* cipher = EVP_aes_256_cbc();
    if ((encrypt
            ? EVP_EncryptInit_ex(ctx, cipher, nullptr, key_bytes.data(), iv_bytes.data())
            : EVP_DecryptInit_ex(ctx, cipher, nullptr, key_bytes.data(), iv_bytes.data())) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP cipher init failed");
    }

    std::vector<uint8_t> out(input_len + EVP_CIPHER_block_size(cipher));
    int chunk_len = 0;
    int final_len = 0;
    if ((encrypt
            ? EVP_EncryptUpdate(ctx, out.data(), &chunk_len, input, static_cast<int>(input_len))
            : EVP_DecryptUpdate(ctx, out.data(), &chunk_len, input, static_cast<int>(input_len))) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP cipher update failed");
    }

    if ((encrypt
            ? EVP_EncryptFinal_ex(ctx, out.data() + chunk_len, &final_len)
            : EVP_DecryptFinal_ex(ctx, out.data() + chunk_len, &final_len)) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("EVP cipher final failed");
    }

    EVP_CIPHER_CTX_free(ctx);
    out.resize(static_cast<size_t>(chunk_len + final_len));
    return out;
}

#endif

} // namespace

SwarmSecurityContext::SwarmSecurityContext(uint32_t local_id, SwarmSecurityConfig cfg)
    : local_id_(local_id), cfg_(std::move(cfg)) {
    drone::utils::get_or_create_logger("SECURITY")->info(
        "SwarmSecurityContext initialized local_id={} enabled={} iterations={}",
        local_id_, enabled(), cfg_.pbkdf2_iterations);
}

void SwarmSecurityContext::reset() {
    drone::utils::get_or_create_logger("SECURITY")->info("SwarmSecurityContext reset");
    peer_state_.clear();
    key_cache_.clear();
    last_error_.clear();
}

bool SwarmSecurityContext::is_ledger_message(SwarmMessage::Type type) const {
    return type == SwarmMessage::Type::FORMATION_CMD ||
           type == SwarmMessage::Type::LEADER_ELECT ||
           type == SwarmMessage::Type::MISSION_SYNC ||
           type == SwarmMessage::Type::EMERGENCY_STOP;
}

const SwarmSecurityContext::KeyMaterial& SwarmSecurityContext::keys_for(uint32_t node_id) {
    const auto it = key_cache_.find(node_id);
    if (it != key_cache_.end()) {
        return it->second;
    }
    drone::utils::get_or_create_logger("SECURITY")->debug("Deriving security material for node={}", node_id);
    return key_cache_.emplace(node_id, derive_key_material(node_id)).first->second;
}

std::vector<uint8_t> SwarmSecurityContext::build_header(uint32_t src_id,
                                                        uint32_t seq_num,
                                                        uint64_t issued_ns,
                                                        uint8_t flags,
                                                        const std::array<uint8_t, 16>& iv,
                                                        const std::array<uint8_t, 20>& past_sha1,
                                                        const std::array<uint8_t, 32>& present_sha256,
                                                        const std::array<uint8_t, 32>& future_sha3,
                                                        const std::array<uint8_t, 32>& chain_prev_hash,
                                                        uint32_t cipher_len) const {
    std::vector<uint8_t> header;
    header.reserve(kHeaderSize);
    append_u32(header, kSecureMagic);
    header.push_back(kSecureVersion);
    header.push_back(flags);
    header.push_back(0);
    header.push_back(0);
    append_u32(header, src_id);
    append_u32(header, seq_num);
    append_u64(header, issued_ns);
    append_u32(header, cipher_len);
    append_array(header, iv);
    append_array(header, past_sha1);
    append_array(header, present_sha256);
    append_array(header, future_sha3);
    append_array(header, chain_prev_hash);
    return header;
}

std::array<uint8_t, 20> SwarmSecurityContext::sha1_bytes(const uint8_t* data, size_t len) const {
    std::array<uint8_t, 20> out{};
#ifdef _WIN32
    const auto digest = bcrypt_hash(BCRYPT_SHA1_ALGORITHM, 0, nullptr, 0, data, len, out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#else
    const auto digest = evp_digest(EVP_sha1(), data, len, out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#endif
    return out;
}

std::array<uint8_t, 32> SwarmSecurityContext::sha256_bytes(const std::vector<uint8_t>& bytes) const {
    return sha256_bytes(bytes.data(), bytes.size());
}

std::array<uint8_t, 32> SwarmSecurityContext::sha256_bytes(const uint8_t* data, size_t len) const {
    std::array<uint8_t, 32> out{};
#ifdef _WIN32
    const auto digest = bcrypt_hash(BCRYPT_SHA256_ALGORITHM, 0, nullptr, 0, data, len, out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#else
    const auto digest = evp_digest(EVP_sha256(), data, len, out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#endif
    return out;
}

std::array<uint8_t, 32> SwarmSecurityContext::sha3_256_bytes(const uint8_t* data, size_t len) const {
    std::array<uint8_t, 32> out{};
    sha3(data, len, out.data(), static_cast<int>(out.size()));
    return out;
}

std::array<uint8_t, 32> SwarmSecurityContext::hmac_sha256(const std::array<uint8_t, 32>& key,
                                                          const uint8_t* data,
                                                          size_t len) const {
    std::array<uint8_t, 32> out{};
#ifdef _WIN32
    const auto digest = bcrypt_hash(BCRYPT_SHA256_ALGORITHM,
                                    BCRYPT_ALG_HANDLE_HMAC_FLAG,
                                    key.data(),
                                    key.size(),
                                    data,
                                    len,
                                    out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#else
    const auto digest = hmac_sha256_bytes(key.data(), key.size(), data, len, out.size());
    std::copy(digest.begin(), digest.end(), out.begin());
#endif
    return out;
}

std::vector<uint8_t> SwarmSecurityContext::aes256_encrypt(const std::array<uint8_t, 32>& key,
                                                          const std::array<uint8_t, 16>& iv,
                                                          const std::vector<uint8_t>& plain) const {
#ifdef _WIN32
    return aes256_crypt(true, key, iv, plain.data(), plain.size());
#else
    return aes256_crypt(true, key, iv, plain.data(), plain.size());
#endif
}

std::optional<std::vector<uint8_t>> SwarmSecurityContext::aes256_decrypt(
        const std::array<uint8_t, 32>& key,
        const std::array<uint8_t, 16>& iv,
        const uint8_t* cipher,
        size_t len) const {
    try {
#ifdef _WIN32
        return aes256_crypt(false, key, iv, cipher, len);
#else
        return aes256_crypt(false, key, iv, cipher, len);
#endif
    } catch (...) {
        return std::nullopt;
    }
}

SwarmSecurityContext::KeyMaterial SwarmSecurityContext::derive_key_material(uint32_t node_id) const {
    KeyMaterial material;
    std::string salt = "swarm-node-" + std::to_string(node_id);
    const auto derived = pbkdf2_sha256(cfg_.swarm_secret,
                                       reinterpret_cast<const uint8_t*>(salt.data()),
                                       salt.size(),
                                       cfg_.pbkdf2_iterations,
                                       128);
    std::memcpy(material.enc_key.data(), derived.data(), 32);
    std::memcpy(material.mac_key.data(), derived.data() + 32, 32);
    std::memcpy(material.eddsa_seed.data(), derived.data() + 64, 32);
    std::memcpy(material.future_key.data(), derived.data() + 96, 32);
    std::array<uint8_t, 32> seed = material.eddsa_seed;
    crypto_eddsa_key_pair(material.secret_key.data(), material.public_key.data(), seed.data());
    crypto_wipe(seed.data(), seed.size());
    return material;
}

std::array<uint8_t, 16> SwarmSecurityContext::random_iv() const {
    std::array<uint8_t, 16> iv{};
    std::random_device rd;
    for (auto& byte : iv) {
        byte = static_cast<uint8_t>(rd());
    }
    return iv;
}

void SwarmSecurityContext::set_error(std::string error) const {
    if (!error.empty()) {
        drone::utils::get_or_create_logger("SECURITY")->warn("Security error: {}", error);
    }
    last_error_ = std::move(error);
}

std::vector<uint8_t> SwarmSecurityContext::seal(const SwarmMessage& msg) {
    auto logger = drone::utils::get_or_create_logger("SECURITY");
    if (!enabled()) {
        logger->debug("Security seal bypassed type={} src={} because security disabled",
                      static_cast<int>(msg.type), msg.src_id);
        return msg.serialize();
    }

    const auto& keys = keys_for(local_id_);
    const auto plain = msg.serialize();
    const auto iv = random_iv();
    const uint64_t issued_ns = static_cast<uint64_t>(msg.timestamp * 1.0e9);

    auto& state = peer_state_[local_id_];
    std::array<uint8_t, 20> past_sha1{};
    if (state.has_last_frame) {
        past_sha1 = sha1_bytes(state.last_frame_hash.data(), state.last_frame_hash.size());
    }

    const auto present_sha256 = sha256_bytes(plain);
    auto future_material = plain;
    future_material.insert(future_material.end(), keys.future_key.begin(), keys.future_key.end());
    append_u64(future_material, issued_ns);
    const auto future_sha3 = sha3_256_bytes(future_material.data(), future_material.size());

    std::array<uint8_t, 32> chain_prev_hash{};
    uint8_t flags = 0;
    if (is_ledger_message(msg.type)) {
        flags |= kFlagLedger;
        if (state.has_last_ledger) {
            chain_prev_hash = state.last_ledger_hash;
        }
    }

    const auto cipher = aes256_encrypt(keys.enc_key, iv, plain);
    const auto header = build_header(msg.src_id, msg.seq_num, issued_ns, flags, iv,
                                     past_sha1, present_sha256, future_sha3,
                                     chain_prev_hash, static_cast<uint32_t>(cipher.size()));
    const auto mac_input = concat_bytes({
        {header.data(), header.size()},
        {cipher.data(), cipher.size()}
    });
    const auto mac = hmac_sha256(keys.mac_key, mac_input.data(), mac_input.size());

    auto sign_input = concat_bytes({
        {header.data(), header.size()},
        {mac.data(), mac.size()},
        {cipher.data(), cipher.size()}
    });
    std::array<uint8_t, 64> signature{};
    crypto_eddsa_sign(signature.data(), keys.secret_key.data(), sign_input.data(), sign_input.size());

    std::vector<uint8_t> frame;
    frame.reserve(header.size() + mac.size() + signature.size() + cipher.size());
    frame.insert(frame.end(), header.begin(), header.end());
    frame.insert(frame.end(), mac.begin(), mac.end());
    frame.insert(frame.end(), signature.begin(), signature.end());
    frame.insert(frame.end(), cipher.begin(), cipher.end());

    state.last_frame_hash = sha256_bytes(frame);
    state.has_last_frame = true;
    if (flags & kFlagLedger) {
        state.last_ledger_hash = state.last_frame_hash;
        state.has_last_ledger = true;
    }
    logger->debug("Security seal success type={} src={} dst={} seq={} bytes={}",
                  static_cast<int>(msg.type), msg.src_id, msg.dst_id, msg.seq_num, frame.size());
    return frame;
}

std::optional<SwarmMessage> SwarmSecurityContext::open(const uint8_t* data, size_t len) {
    auto logger = drone::utils::get_or_create_logger("SECURITY");
    if (!enabled()) {
        logger->debug("Security open bypassed bytes={} because security disabled", len);
        return SwarmMessage::deserialize(data, len);
    }
    if (!data || len < kHeaderSize + kMacSize + kSignatureSize) {
        set_error("frame too small");
        return std::nullopt;
    }

    if (read_u32(data, 0) != kSecureMagic || data[4] != kSecureVersion) {
        set_error("invalid secure magic/version");
        return std::nullopt;
    }

    const uint8_t flags = data[5];
    const uint32_t src_id = read_u32(data, 8);
    const uint32_t seq_num = read_u32(data, 12);
    const uint64_t issued_ns = read_u64(data, 16);
    const uint32_t cipher_len = read_u32(data, 24);

    if (len != kHeaderSize + kMacSize + kSignatureSize + cipher_len) {
        set_error("cipher length mismatch");
        return std::nullopt;
    }

    size_t offset = 28;
    std::array<uint8_t, 16> iv{};
    std::memcpy(iv.data(), data + offset, iv.size());
    offset += iv.size();

    std::array<uint8_t, 20> past_sha1{};
    std::memcpy(past_sha1.data(), data + offset, past_sha1.size());
    offset += past_sha1.size();

    std::array<uint8_t, 32> present_sha256{};
    std::memcpy(present_sha256.data(), data + offset, present_sha256.size());
    offset += present_sha256.size();

    std::array<uint8_t, 32> future_sha3{};
    std::memcpy(future_sha3.data(), data + offset, future_sha3.size());
    offset += future_sha3.size();

    std::array<uint8_t, 32> chain_prev_hash{};
    std::memcpy(chain_prev_hash.data(), data + offset, chain_prev_hash.size());
    offset += chain_prev_hash.size();

    const uint8_t* mac_ptr = data + kHeaderSize;
    const uint8_t* sig_ptr = mac_ptr + kMacSize;
    const uint8_t* cipher_ptr = sig_ptr + kSignatureSize;

    auto& state = peer_state_[src_id];
    if (seq_num <= state.last_seq) {
        set_error("replay rejected");
        return std::nullopt;
    }

    const auto& keys = keys_for(src_id);
    const auto mac_input = concat_bytes({
        {data, kHeaderSize},
        {cipher_ptr, cipher_len}
    });
    const auto mac = hmac_sha256(keys.mac_key, mac_input.data(), mac_input.size());
    if (!std::equal(mac.begin(), mac.end(), mac_ptr)) {
        set_error("mac mismatch");
        return std::nullopt;
    }

    auto sign_input = concat_bytes({
        {data, kHeaderSize},
        {mac_ptr, kMacSize},
        {cipher_ptr, cipher_len}
    });
    if (crypto_eddsa_check(sig_ptr, keys.public_key.data(), sign_input.data(), sign_input.size()) != 0) {
        set_error("signature mismatch");
        return std::nullopt;
    }

    const auto plain = aes256_decrypt(keys.enc_key, iv, cipher_ptr, cipher_len);
    if (!plain.has_value()) {
        set_error("decrypt failed");
        return std::nullopt;
    }

    const auto present_check = sha256_bytes(*plain);
    if (present_check != present_sha256) {
        set_error("present digest mismatch");
        return std::nullopt;
    }

    auto future_material = *plain;
    future_material.insert(future_material.end(), keys.future_key.begin(), keys.future_key.end());
    append_u64(future_material, issued_ns);
    if (sha3_256_bytes(future_material.data(), future_material.size()) != future_sha3) {
        set_error("future digest mismatch");
        return std::nullopt;
    }

    std::array<uint8_t, 20> expected_past{};
    if (state.has_last_frame) {
        expected_past = sha1_bytes(state.last_frame_hash.data(), state.last_frame_hash.size());
    }
    if (expected_past != past_sha1) {
        set_error("past digest mismatch");
        return std::nullopt;
    }

    if ((flags & kFlagLedger) != 0) {
        std::array<uint8_t, 32> expected_chain{};
        if (state.has_last_ledger) {
            expected_chain = state.last_ledger_hash;
        }
        if (expected_chain != chain_prev_hash) {
            set_error("ledger chain mismatch");
            return std::nullopt;
        }
    }

    auto msg = SwarmMessage::deserialize(plain->data(), plain->size());
    if (!msg.has_value()) {
        set_error("inner message decode failed");
        return std::nullopt;
    }
    if (msg->src_id != src_id || msg->seq_num != seq_num) {
        set_error("header/message identity mismatch");
        return std::nullopt;
    }

    state.last_seq = seq_num;
    state.last_frame_hash = sha256_bytes(data, len);
    state.has_last_frame = true;
    if ((flags & kFlagLedger) != 0) {
        state.last_ledger_hash = state.last_frame_hash;
        state.has_last_ledger = true;
    }

    set_error("");
    logger->debug("Security open success src={} seq={} bytes={}", src_id, seq_num, len);
    return msg;
}

} // namespace drone::swarm
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
