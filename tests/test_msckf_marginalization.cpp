#include "vio/MsckfMarginalization.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace drone::vio {
namespace {

TEST(MsckfMarginalizationTest, OrdersAffectedTracksDeterministically) {
    const std::vector<MarginalizationTrackSummary> tracks{
        {9u, {1u, 4u, 8u}, true},
        {2u, {4u, 5u, 6u}, true},
        {7u, {3u, 4u}, false},
        {3u, {10u, 11u}, true},
    };

    const auto plan = MsckfMarginalization::build_plan(4u, tracks, 3u);

    EXPECT_EQ(plan.retiring_state_id, 4u);
    EXPECT_EQ(plan.affected_track_ids, (std::vector<uint64_t>{2u, 7u, 9u}));
    EXPECT_EQ(plan.constraint_candidate_track_ids, (std::vector<uint64_t>{2u, 9u}));
    EXPECT_EQ(plan.stale_reference_count, 0u);
}

TEST(MsckfMarginalizationTest, DuplicateRetiringReferencesAreCountedAsStale) {
    const std::vector<MarginalizationTrackSummary> tracks{
        {1u, {4u, 4u, 5u}, true},
        {2u, {4u, 6u, 7u}, true},
    };

    const auto plan = MsckfMarginalization::build_plan(4u, tracks, 2u);
    EXPECT_EQ(plan.stale_reference_count, 1u);
}

TEST(MsckfMarginalizationTest, RetainedCovarianceIsExactPrincipalSubmatrix) {
    constexpr std::size_t kBaseDim = 3u;
    constexpr std::size_t kCloneDim = 2u;
    const std::vector<uint64_t> clone_ids{10u, 20u, 30u};
    constexpr Eigen::Index kDim = 9;

    Eigen::MatrixXd A(kDim, kDim);
    for (Eigen::Index row = 0; row < kDim; ++row) {
        for (Eigen::Index col = 0; col < kDim; ++col) {
            A(row, col) = static_cast<double>((row + 1) * 10 + (col + 1));
        }
    }
    const Eigen::MatrixXd P = (A * A.transpose()) + Eigen::MatrixXd::Identity(kDim, kDim);

    const auto retained =
        MsckfMarginalization::retained_principal_submatrix(P, kBaseDim, kCloneDim, clone_ids, 20u);
    ASSERT_TRUE(retained.has_value());
    ASSERT_EQ(retained->rows(), 7);
    ASSERT_EQ(retained->cols(), 7);

    const std::vector<Eigen::Index> kept{0, 1, 2, 3, 4, 7, 8};
    for (Eigen::Index row = 0; row < retained->rows(); ++row) {
        for (Eigen::Index col = 0; col < retained->cols(); ++col) {
            EXPECT_DOUBLE_EQ((*retained)(row, col), P(kept[static_cast<std::size_t>(row)],
                                                      kept[static_cast<std::size_t>(col)]));
        }
    }
}

TEST(MsckfMarginalizationTest, RetainedCrossCovariancesArePreserved) {
    constexpr std::size_t kBaseDim = 3u;
    constexpr std::size_t kCloneDim = 2u;
    const std::vector<uint64_t> clone_ids{10u, 20u, 30u};
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(9, 9);
    P(0, 7) = 0.37;
    P(7, 0) = 0.37;
    P(4, 8) = -0.21;
    P(8, 4) = -0.21;

    const auto retained =
        MsckfMarginalization::retained_principal_submatrix(P, kBaseDim, kCloneDim, clone_ids, 20u);
    ASSERT_TRUE(retained.has_value());

    EXPECT_DOUBLE_EQ((*retained)(0, 5), 0.37);
    EXPECT_DOUBLE_EQ((*retained)(5, 0), 0.37);
    EXPECT_DOUBLE_EQ((*retained)(4, 6), -0.21);
    EXPECT_DOUBLE_EQ((*retained)(6, 4), -0.21);
}

TEST(MsckfMarginalizationTest, InvalidCovarianceDimensionsFailClosed) {
    const std::vector<uint64_t> clone_ids{10u, 20u};
    const Eigen::MatrixXd bad = Eigen::MatrixXd::Identity(6, 6);
    EXPECT_FALSE(MsckfMarginalization::retained_principal_submatrix(bad, 3u, 2u, clone_ids, 10u)
                     .has_value());
}

TEST(MsckfMarginalizationTest, MissingRetiringCloneFailsClosed) {
    const std::vector<uint64_t> clone_ids{10u, 20u};
    const Eigen::MatrixXd P = Eigen::MatrixXd::Identity(7, 7);
    EXPECT_FALSE(
        MsckfMarginalization::retained_principal_submatrix(P, 3u, 2u, clone_ids, 30u).has_value());
}

TEST(MsckfMarginalizationTest, CovarianceHealthAcceptsFiniteSymmetricPsdMatrix) {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(5, 5);
    P(0, 1) = 0.1;
    P(1, 0) = 0.1;

    const auto health = MsckfMarginalization::covariance_health(P, 1.0e-12, 1.0e-12);
    EXPECT_TRUE(health.finite);
    EXPECT_TRUE(health.symmetric);
    EXPECT_TRUE(health.psd_within_tolerance);
    EXPECT_LE(health.symmetry_error, 1.0e-12);
    EXPECT_GT(health.minimum_eigenvalue, 0.0);
}

TEST(MsckfMarginalizationTest, CovarianceHealthRejectsSignificantNegativeEigenvalue) {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);
    P(3, 3) = -0.25;

    const auto health = MsckfMarginalization::covariance_health(P, 1.0e-12, 1.0e-9);
    EXPECT_TRUE(health.finite);
    EXPECT_TRUE(health.symmetric);
    EXPECT_FALSE(health.psd_within_tolerance);
    EXPECT_LT(health.minimum_eigenvalue, 0.0);
}

TEST(MsckfMarginalizationTest, CleanupReferenceCheckDetectsRetiredState) {
    std::vector<MarginalizationTrackSummary> tracks{
        {1u, {1u, 2u, 3u}, true},
        {2u, {2u, 5u}, false},
    };
    EXPECT_TRUE(MsckfMarginalization::has_retired_state_reference(2u, tracks));

    for (auto& track : tracks) {
        std::erase(track.observation_state_ids, 2u);
    }
    EXPECT_FALSE(MsckfMarginalization::has_retired_state_reference(2u, tracks));
}

} // namespace
} // namespace drone::vio
