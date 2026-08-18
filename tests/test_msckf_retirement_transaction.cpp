#include "vio/MsckfRetirementTransaction.hpp"

#include <gtest/gtest.h>

namespace drone::vio {
namespace {

Eigen::MatrixXd make_spd(std::size_t dim) {
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(dim),
                                              static_cast<Eigen::Index>(dim));
    for (Eigen::Index i = 0; i < a.rows(); ++i) {
        a(i, i) = 2.0 + static_cast<double>(i) * 0.01;
        if (i + 1 < a.rows()) {
            a(i, i + 1) = 0.05;
            a(i + 1, i) = 0.05;
        }
    }
    return a;
}

TEST(MsckfRetirementTransactionTest, RetiresOldestCloneAndPreservesPrincipalSubmatrix) {
    const std::vector<uint64_t> clones{11, 12, 13};
    const Eigen::MatrixXd covariance = make_spd(15 + 3 * 6);
    MsckfRetirementRequest request;
    request.retiring_state_id = 11;

    const auto result = MsckfRetirementTransaction::prepare(request, clones, covariance);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->committed);
    EXPECT_EQ(result->retained_clone_ids, (std::vector<uint64_t>{12, 13}));
    ASSERT_EQ(result->retained_covariance.rows(), 27);

    const auto expected = MsckfMarginalization::retained_principal_submatrix(
        covariance, 15, 6, clones, 11);
    ASSERT_TRUE(expected.has_value());
    EXPECT_LT((result->retained_covariance - *expected).norm(), 1.0e-12);
}

TEST(MsckfRetirementTransactionTest, PreservesRetainedCrossCovariances) {
    const std::vector<uint64_t> clones{21, 22};
    Eigen::MatrixXd covariance = make_spd(27);
    covariance(0, 21) = 0.123;
    covariance(21, 0) = 0.123;
    MsckfRetirementRequest request;
    request.retiring_state_id = 21;

    const auto result = MsckfRetirementTransaction::prepare(request, clones, covariance);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->retained_covariance(0, 15), 0.123, 1.0e-12);
}

TEST(MsckfRetirementTransactionTest, RejectsNonOldestRetirement) {
    const std::vector<uint64_t> clones{31, 32};
    MsckfRetirementRequest request;
    request.retiring_state_id = 32;
    EXPECT_FALSE(MsckfRetirementTransaction::prepare(request, clones, make_spd(27)).has_value());
}

TEST(MsckfRetirementTransactionTest, RejectsInvalidDimensionsWithoutPartialResult) {
    const std::vector<uint64_t> clones{41, 42};
    MsckfRetirementRequest request;
    request.retiring_state_id = 41;
    EXPECT_FALSE(MsckfRetirementTransaction::prepare(request, clones, make_spd(26)).has_value());
}

TEST(MsckfRetirementTransactionTest, RejectsNonFiniteCovariance) {
    const std::vector<uint64_t> clones{51};
    Eigen::MatrixXd covariance = make_spd(21);
    covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
    MsckfRetirementRequest request;
    request.retiring_state_id = 51;
    EXPECT_FALSE(MsckfRetirementTransaction::prepare(request, clones, covariance).has_value());
}

TEST(MsckfRetirementTransactionTest, RejectsSignificantNegativeVariance) {
    const std::vector<uint64_t> clones{61};
    Eigen::MatrixXd covariance = make_spd(21);
    covariance(0, 0) = -1.0;
    MsckfRetirementRequest request;
    request.retiring_state_id = 61;
    EXPECT_FALSE(MsckfRetirementTransaction::prepare(request, clones, covariance).has_value());
}

TEST(MsckfRetirementTransactionTest, RepeatedRetirementRemainsFiniteSymmetricAndPsd) {
    std::vector<uint64_t> clones{71, 72, 73, 74};
    Eigen::MatrixXd covariance = make_spd(15 + 4 * 6);
    for (int step = 0; step < 4; ++step) {
        MsckfRetirementRequest request;
        request.retiring_state_id = clones.front();
        const auto result = MsckfRetirementTransaction::prepare(request, clones, covariance);
        ASSERT_TRUE(result.has_value());
        EXPECT_TRUE(result->covariance_health.finite);
        EXPECT_TRUE(result->covariance_health.symmetric);
        EXPECT_TRUE(result->covariance_health.psd_within_tolerance);
        covariance = result->retained_covariance;
        clones = result->retained_clone_ids;
    }
    EXPECT_EQ(covariance.rows(), 15);
    EXPECT_TRUE(covariance.array().isFinite().all());
}

} // namespace
} // namespace drone::vio
