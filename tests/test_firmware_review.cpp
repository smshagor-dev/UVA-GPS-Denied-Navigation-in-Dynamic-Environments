#include "security/FirmwareTrust.hpp"
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using drone::security::FirmwareManifest;
using drone::security::FirmwareTrustPolicy;
using drone::security::validate_firmware_trust;
using drone::security::sign_firmware_manifest;

FirmwareManifest baseline_manifest() {
    FirmwareManifest m;
    m.version = "2.1.0";
    m.rollback_counter = 4;
    m.measurement = "fw-secure-reference";
    m.signer = "release-ca";
    m.secure_boot_attested = true;
    m.bootloader_locked = true;
    return m;
}
fs::path state_path(const std::string& scenario) {
    const auto path = fs::temp_directory_path() / ("firmware_review_" + scenario + ".state");
    std::error_code ec;
    fs::remove_all(path, ec);
    return path;
}
FirmwareTrustPolicy policy_for(const fs::path& path, const std::string& profile = "production") {
    FirmwareTrustPolicy p;
    p.profile = profile;
    p.state_file = path.string();
    p.allowed_signers = {"release-ca"};
    p.signing_secret = "review-fixture-secret";
    return p;
}
std::string contents(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::ostringstream out;
    out << file.rdbuf();
    return out.str();
}
void sign(FirmwareManifest& m, const FirmwareTrustPolicy& p) {
    m.signature = sign_firmware_manifest(m, p.signing_secret);
}
enum class Mutation {
    VersionOnly, CounterOnly, BothRollback, Signer, Digest, SecureBoot,
    Bootloader, EmptyMeasurement, UnsignedMeasurement, LabMeasurement, MissingToken
};
struct NegativeCase { const char* name; Mutation mutation; const char* outcome; };

class OnePredicateRejection : public ::testing::TestWithParam<NegativeCase> {};
TEST_P(OnePredicateRejection, RejectsAndPreservesAcceptedState) {
    const NegativeCase scenario = GetParam();
    const fs::path path = state_path(scenario.name);
    auto policy = policy_for(path);
    auto baseline = baseline_manifest();
    sign(baseline, policy);
    const auto accepted = validate_firmware_trust(baseline, policy);
    ASSERT_TRUE(accepted.accepted) << accepted.summary;
    ASSERT_TRUE(fs::exists(path));
    const auto persisted_before = contents(path);

    auto candidate = baseline;
    bool should_resign = true;
    switch (scenario.mutation) {
        case Mutation::VersionOnly: candidate.version = "2.0.5"; break;
        case Mutation::CounterOnly: candidate.rollback_counter = 3; break;
        case Mutation::BothRollback:
            candidate.version = "2.0.5"; candidate.rollback_counter = 3; break;
        case Mutation::Signer: candidate.signer = "untrusted-ca"; break;
        case Mutation::Digest: candidate.signature = std::string(64, '0'); should_resign = false; break;
        case Mutation::SecureBoot: candidate.secure_boot_attested = false; break;
        case Mutation::Bootloader: candidate.bootloader_locked = false; break;
        case Mutation::EmptyMeasurement: candidate.measurement.clear(); break;
        case Mutation::UnsignedMeasurement: candidate.measurement = "unsigned-build"; break;
        case Mutation::LabMeasurement: candidate.measurement = "lab-local-build"; break;
        case Mutation::MissingToken: policy.maintenance_mode = true; break;
    }
    if (should_resign) sign(candidate, policy);
    const auto rejected = validate_firmware_trust(candidate, policy);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(rejected.boot_state, scenario.outcome);
    EXPECT_EQ(contents(path), persisted_before) << "Rejection altered persistent state";
}
INSTANTIATE_TEST_SUITE_P(
    FirmwarePolicyMatrix, OnePredicateRejection,
    ::testing::Values(
        NegativeCase{"VersionOnly", Mutation::VersionOnly, "ROLLBACK_REJECTED"},
        NegativeCase{"CounterOnly", Mutation::CounterOnly, "ROLLBACK_REJECTED"},
        NegativeCase{"BothRollback", Mutation::BothRollback, "ROLLBACK_REJECTED"},
        NegativeCase{"Signer", Mutation::Signer, "SIGNER_REJECTED"},
        NegativeCase{"Digest", Mutation::Digest, "SIGNATURE_INVALID"},
        NegativeCase{"SecureBoot", Mutation::SecureBoot, "SECURE_BOOT_FAILED"},
        NegativeCase{"Bootloader", Mutation::Bootloader, "SECURE_BOOT_FAILED"},
        NegativeCase{"EmptyMeasurement", Mutation::EmptyMeasurement, "MEASUREMENT_UNTRUSTED"},
        NegativeCase{"UnsignedMeasurement", Mutation::UnsignedMeasurement, "MEASUREMENT_UNTRUSTED"},
        NegativeCase{"LabMeasurement", Mutation::LabMeasurement, "MEASUREMENT_UNTRUSTED"},
        NegativeCase{"MissingToken", Mutation::MissingToken, "MAINTENANCE_UNAUTHORIZED"}
    ),
    [](const ::testing::TestParamInfo<NegativeCase>& info) {
        return std::string(info.param.name);
    });

TEST(FirmwareReview, AcceptsVersionAndCounterAdvanceIndependently) {
    auto path = state_path("advance");
    auto policy = policy_for(path);
    auto first = baseline_manifest(); sign(first, policy);
    ASSERT_TRUE(validate_firmware_trust(first, policy).accepted);
    auto version_advance = first; version_advance.version = "2.2.0"; sign(version_advance, policy);
    EXPECT_TRUE(validate_firmware_trust(version_advance, policy).accepted);
    auto counter_advance = version_advance; counter_advance.rollback_counter = 5; sign(counter_advance, policy);
    EXPECT_TRUE(validate_firmware_trust(counter_advance, policy).accepted);
    const auto record = drone::security::load_firmware_trust_record(path);
    EXPECT_EQ(record.version, "2.2.0");
    EXPECT_EQ(record.rollback_counter, 5u);
}
TEST(FirmwareReview, NonEmptyMaintenanceTokenIsOnlyPresenceChecked) {
    auto path = state_path("maintenance_present");
    auto policy = policy_for(path, "field");
    policy.maintenance_mode = true;
    policy.maintenance_token = "arbitrary-nonempty-string";
    auto manifest = baseline_manifest(); sign(manifest, policy);
    const auto result = validate_firmware_trust(manifest, policy);
    EXPECT_TRUE(result.accepted) << result.summary;
    EXPECT_EQ(result.boot_state, "MAINTENANCE_TRUSTED");
    EXPECT_EQ(result.update_state, "maintenance-token-present");
    // The validator does not verify the token against an issuer or approval service.
}
TEST(FirmwareReview, MissingSecretRejectsFieldAndProduction) {
    for (const char* profile : {"field", "production"}) {
        auto path = state_path(std::string("secret_") + profile);
        auto policy = policy_for(path, profile);
        policy.signing_secret.clear();
        auto manifest = baseline_manifest();
        manifest.signature = "arbitrary-nonempty";
        const auto result = validate_firmware_trust(manifest, policy);
        EXPECT_FALSE(result.accepted);
        EXPECT_EQ(result.boot_state, "TRUST_CONFIGURATION_INVALID");
        EXPECT_FALSE(fs::exists(path));
    }
}
TEST(FirmwareReview, MissingSignerPolicyRejectsHardenedBoot) {
    auto path = state_path("no_allowlist");
    auto policy = policy_for(path);
    policy.allowed_signers.clear();
    auto manifest = baseline_manifest(); sign(manifest, policy);
    const auto result = validate_firmware_trust(manifest, policy);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.boot_state, "TRUST_CONFIGURATION_INVALID");
    EXPECT_FALSE(fs::exists(path));
}
TEST(FirmwareReview, FailedPersistenceRejectsHardenedBoot) {
    auto path = state_path("directory_instead_of_file");
    fs::create_directories(path);
    auto policy = policy_for(path);
    auto manifest = baseline_manifest(); sign(manifest, policy);
    const auto result = validate_firmware_trust(manifest, policy);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.boot_state, "STATE_PERSISTENCE_FAILED");
    std::error_code ec;
    fs::remove_all(path, ec);
}
TEST(FirmwareReview, LaboratoryModeIsNotAttestation) {
    auto path = state_path("lab");
    auto policy = policy_for(path, "lab");
    policy.signing_secret.clear();
    policy.allowed_signers.clear();
    auto manifest = baseline_manifest(); manifest.signature.clear();
    const auto result = validate_firmware_trust(manifest, policy);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.boot_state, "LAB_BOOT");
    EXPECT_FALSE(fs::exists(path));
}
}  // namespace
