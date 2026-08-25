// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "sha3.h"
#ifdef __cplusplus
}
#endif

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace drone::security {

struct FirmwareManifest {
    std::string version{"0.0.0"};
    std::string measurement{"lab-local-build"};
    std::string signer{"lab-signer"};
    std::string signature{};
    bool secure_boot_attested{false};
    bool bootloader_locked{false};
    uint64_t rollback_counter{0};
};

struct FirmwareTrustPolicy {
    std::string profile{"lab"};
    std::string state_file{"state/firmware_trust.state"};
    std::vector<std::string> allowed_signers{};
    std::string signing_secret{};
    bool maintenance_mode{false};
    std::string maintenance_token{};
};

struct FirmwareTrustRecord {
    uint64_t rollback_counter{0};
    std::string version{"0.0.0"};
    std::string measurement{"lab-local-build"};
};

struct FirmwareTrustReport {
    bool accepted{true};
    bool secure_boot_attested{false};
    bool bootloader_locked{false};
    bool maintenance_mode{false};
    bool signed_firmware_valid{false};
    uint64_t rollback_counter{0};
    std::string version{"0.0.0"};
    std::string measurement{"lab-local-build"};
    std::string boot_state{"LAB_BOOT"};
    std::string update_state{"idle"};
    std::string summary{"Lab boot trust bypassed"};
};

inline std::string trim_copy(std::string value) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

inline std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

inline bool parse_bool_text(std::string_view value, bool fallback = false) {
    std::string lowered = lower_copy(trim_copy(std::string(value)));
    if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on") {
        return true;
    }
    if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off") {
        return false;
    }
    return fallback;
}

inline uint64_t parse_u64_text(std::string_view value, uint64_t fallback = 0) {
    try {
        return static_cast<uint64_t>(std::stoull(trim_copy(std::string(value))));
    } catch (...) {
        return fallback;
    }
}

inline std::vector<int> parse_version(std::string_view text) {
    std::vector<int> parts;
    std::stringstream ss{std::string(text)};
    std::string item;
    while (std::getline(ss, item, '.')) {
        try {
            parts.push_back(std::stoi(trim_copy(item)));
        } catch (...) {
            parts.push_back(0);
        }
    }
    while (parts.size() < 3u) {
        parts.push_back(0);
    }
    return parts;
}

inline int compare_versions(std::string_view lhs, std::string_view rhs) {
    const auto a = parse_version(lhs);
    const auto b = parse_version(rhs);
    for (size_t i = 0; i < std::max(a.size(), b.size()); ++i) {
        const int av = i < a.size() ? a[i] : 0;
        const int bv = i < b.size() ? b[i] : 0;
        if (av < bv) {
            return -1;
        }
        if (av > bv) {
            return 1;
        }
    }
    return 0;
}

inline std::string sha3_hex(std::string_view payload) {
    std::array<unsigned char, 32> digest{};
    sha3(payload.data(), payload.size(), digest.data(), static_cast<int>(digest.size()));
    static constexpr char hex_digits[] = "0123456789abcdef";
    std::string out;
    out.reserve(digest.size() * 2u);
    for (unsigned char byte : digest) {
        out.push_back(hex_digits[(byte >> 4u) & 0x0fu]);
        out.push_back(hex_digits[byte & 0x0fu]);
    }
    return out;
}

inline std::string sign_firmware_manifest(const FirmwareManifest& manifest,
                                          std::string_view secret) {
    return sha3_hex(trim_copy(std::string(secret)) + "\n" + manifest.version + "\n" +
                    manifest.measurement + "\n" + manifest.signer + "\n" +
                    (manifest.secure_boot_attested ? "1" : "0") + "\n" +
                    (manifest.bootloader_locked ? "1" : "0") + "\n" +
                    std::to_string(manifest.rollback_counter));
}

inline std::optional<FirmwareManifest>
load_firmware_manifest_file(const std::filesystem::path& path) {
    if (path.empty()) {
        return std::nullopt;
    }
    std::ifstream in(path);
    if (!in) {
        return std::nullopt;
    }
    FirmwareManifest manifest;
    std::string line;
    while (std::getline(in, line)) {
        const auto trimmed = trim_copy(line);
        if (trimmed.empty() || trimmed.front() == '#') {
            continue;
        }
        const auto eq = trimmed.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const auto key = lower_copy(trim_copy(trimmed.substr(0, eq)));
        const auto value = trim_copy(trimmed.substr(eq + 1));
        if (key == "version") {
            manifest.version = value;
        } else if (key == "measurement") {
            manifest.measurement = value;
        } else if (key == "signer") {
            manifest.signer = value;
        } else if (key == "signature") {
            manifest.signature = lower_copy(value);
        } else if (key == "secure_boot_attested") {
            manifest.secure_boot_attested = parse_bool_text(value, false);
        } else if (key == "bootloader_locked") {
            manifest.bootloader_locked = parse_bool_text(value, false);
        } else if (key == "rollback_counter") {
            manifest.rollback_counter = parse_u64_text(value, 0);
        }
    }
    return manifest;
}

inline FirmwareTrustRecord load_firmware_trust_record(const std::filesystem::path& path) {
    FirmwareTrustRecord record;
    std::ifstream in(path);
    if (!in) {
        return record;
    }
    std::string line;
    while (std::getline(in, line)) {
        const auto trimmed = trim_copy(line);
        if (trimmed.empty() || trimmed.front() == '#') {
            continue;
        }
        const auto eq = trimmed.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const auto key = lower_copy(trim_copy(trimmed.substr(0, eq)));
        const auto value = trim_copy(trimmed.substr(eq + 1));
        if (key == "rollback_counter") {
            record.rollback_counter = parse_u64_text(value, 0);
        } else if (key == "version") {
            record.version = value;
        } else if (key == "measurement") {
            record.measurement = value;
        }
    }
    return record;
}

inline bool persist_firmware_trust_record(const std::filesystem::path& path,
                                          const FirmwareTrustReport& report) {
    std::error_code ec;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), ec);
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        return false;
    }
    out << "rollback_counter=" << report.rollback_counter << "\n";
    out << "version=" << report.version << "\n";
    out << "measurement=" << report.measurement << "\n";
    return true;
}

inline FirmwareTrustReport validate_firmware_trust(const FirmwareManifest& manifest,
                                                   const FirmwareTrustPolicy& policy) {
    FirmwareTrustReport report;
    report.secure_boot_attested = manifest.secure_boot_attested;
    report.bootloader_locked = manifest.bootloader_locked;
    report.maintenance_mode = policy.maintenance_mode;
    report.rollback_counter = manifest.rollback_counter;
    report.version = manifest.version;
    report.measurement = manifest.measurement;

    const std::string profile = lower_copy(policy.profile);
    const bool hardened = profile == "field" || profile == "production";
    const bool production = profile == "production";

    const auto stored = load_firmware_trust_record(policy.state_file);
    const bool signer_allowed =
        policy.allowed_signers.empty() ||
        std::find_if(policy.allowed_signers.begin(), policy.allowed_signers.end(),
                     [&](const std::string& item) {
                         return trim_copy(item) == trim_copy(manifest.signer);
                     }) != policy.allowed_signers.end();
    const bool signature_matches = !policy.signing_secret.empty()
                                       ? lower_copy(manifest.signature) ==
                                             sign_firmware_manifest(manifest, policy.signing_secret)
                                       : !trim_copy(manifest.signature).empty();

    report.signed_firmware_valid = signer_allowed && signature_matches;

    if (policy.maintenance_mode) {
        report.update_state = "maintenance-window-open";
        if (trim_copy(policy.maintenance_token).empty()) {
            report.accepted = false;
            report.boot_state = hardened ? "MAINTENANCE_UNAUTHORIZED" : "LAB_MAINTENANCE";
            report.summary = "Maintenance mode requires an approval token";
            return report;
        }
        report.update_state = "maintenance-authorized";
    }

    if (!hardened) {
        report.boot_state = policy.maintenance_mode ? "LAB_MAINTENANCE" : "LAB_BOOT";
        report.summary = policy.maintenance_mode
                             ? "Lab maintenance mode active with software boot trust checks"
                             : "Lab boot trust bypassed";
        report.accepted = true;
        return report;
    }

    if (!manifest.secure_boot_attested || !manifest.bootloader_locked) {
        report.accepted = false;
        report.boot_state = "SECURE_BOOT_FAILED";
        report.summary = production ? "Production secure boot attestation failed"
                                    : "Hardened bootloader lock or secure boot attestation missing";
        return report;
    }
    if (compare_versions(manifest.version, stored.version) < 0 ||
        manifest.rollback_counter < stored.rollback_counter) {
        report.accepted = false;
        report.boot_state = "ROLLBACK_REJECTED";
        report.summary = "Firmware rollback counter or version regressed";
        return report;
    }
    if (!signer_allowed) {
        report.accepted = false;
        report.boot_state = "SIGNER_REJECTED";
        report.summary = "Firmware signer is not in the trusted signer list";
        return report;
    }
    if (!signature_matches) {
        report.accepted = false;
        report.boot_state = "SIGNATURE_INVALID";
        report.summary = "Firmware signature verification failed";
        return report;
    }
    if (lower_copy(manifest.measurement).find("unsigned") != std::string::npos ||
        lower_copy(manifest.measurement).find("lab-local") != std::string::npos ||
        trim_copy(manifest.measurement).empty()) {
        report.accepted = false;
        report.boot_state = "MEASUREMENT_UNTRUSTED";
        report.summary = "Firmware measurement is not trusted in hardened mode";
        return report;
    }

    report.boot_state = policy.maintenance_mode ? "MAINTENANCE_TRUSTED" : "SECURE_BOOT_TRUSTED";
    report.summary = policy.maintenance_mode
                         ? "Secure boot trusted and maintenance workflow authorized"
                         : "Secure boot trusted and firmware signature verified";
    persist_firmware_trust_record(policy.state_file, report);
    return report;
}

} // namespace drone::security
