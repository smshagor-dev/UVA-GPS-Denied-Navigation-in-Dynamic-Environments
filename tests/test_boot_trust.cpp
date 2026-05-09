#include "security/FirmwareTrust.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace {

std::filesystem::path unique_state_file(const char* name) {
    auto path = std::filesystem::temp_directory_path() / name;
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return path;
}

drone::security::FirmwareManifest trusted_manifest() {
    drone::security::FirmwareManifest manifest;
    manifest.version = "2.1.0";
    manifest.measurement = "fw-secure-2026-04-17";
    manifest.signer = "release-ca";
    manifest.secure_boot_attested = true;
    manifest.bootloader_locked = true;
    manifest.rollback_counter = 4;
    return manifest;
}

} // namespace

TEST(FirmwareTrust, AcceptsTrustedProductionManifest) {
    auto manifest = trusted_manifest();
    drone::security::FirmwareTrustPolicy policy;
    policy.profile = "production";
    policy.state_file = unique_state_file("drone_firmware_trust_accept.state").string();
    policy.allowed_signers = {"release-ca"};
    policy.signing_secret = "phase4-test-secret";
    manifest.signature = drone::security::sign_firmware_manifest(manifest, policy.signing_secret);

    const auto report = drone::security::validate_firmware_trust(manifest, policy);

    EXPECT_TRUE(report.accepted);
    EXPECT_EQ(report.boot_state, "SECURE_BOOT_TRUSTED");
    EXPECT_EQ(report.rollback_counter, 4u);
}

TEST(FirmwareTrust, RejectsRollbackRegression) {
    auto stateFile = unique_state_file("drone_firmware_trust_rollback.state");
    {
        auto manifest = trusted_manifest();
        drone::security::FirmwareTrustPolicy policy;
        policy.profile = "production";
        policy.state_file = stateFile.string();
        policy.allowed_signers = {"release-ca"};
        policy.signing_secret = "phase4-test-secret";
        manifest.signature = drone::security::sign_firmware_manifest(manifest, policy.signing_secret);
        const auto accepted = drone::security::validate_firmware_trust(manifest, policy);
        ASSERT_TRUE(accepted.accepted);
    }

    auto rollbackManifest = trusted_manifest();
    rollbackManifest.version = "2.0.5";
    rollbackManifest.rollback_counter = 3;
    drone::security::FirmwareTrustPolicy policy;
    policy.profile = "production";
    policy.state_file = stateFile.string();
    policy.allowed_signers = {"release-ca"};
    policy.signing_secret = "phase4-test-secret";
    rollbackManifest.signature = drone::security::sign_firmware_manifest(rollbackManifest, policy.signing_secret);

    const auto report = drone::security::validate_firmware_trust(rollbackManifest, policy);

    EXPECT_FALSE(report.accepted);
    EXPECT_EQ(report.boot_state, "ROLLBACK_REJECTED");
}

TEST(FirmwareTrust, RejectsMaintenanceModeWithoutToken) {
    auto manifest = trusted_manifest();
    drone::security::FirmwareTrustPolicy policy;
    policy.profile = "field";
    policy.state_file = unique_state_file("drone_firmware_trust_maint.state").string();
    policy.allowed_signers = {"release-ca"};
    policy.signing_secret = "phase4-test-secret";
    policy.maintenance_mode = true;
    manifest.signature = drone::security::sign_firmware_manifest(manifest, policy.signing_secret);

    const auto report = drone::security::validate_firmware_trust(manifest, policy);

    EXPECT_FALSE(report.accepted);
    EXPECT_EQ(report.boot_state, "MAINTENANCE_UNAUTHORIZED");
}
