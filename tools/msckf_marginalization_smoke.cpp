#include "vio/MsckfMarginalization.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
    using drone::vio::MarginalizationTrackSummary;
    using drone::vio::MsckfMarginalization;

    const std::vector<MarginalizationTrackSummary> tracks{
        {4u, {10u, 11u, 12u}, true},
        {2u, {9u, 10u, 11u}, true},
        {7u, {10u, 13u}, false},
    };
    const auto plan = MsckfMarginalization::build_plan(10u, tracks, 3u);
    if (plan.affected_track_ids != std::vector<uint64_t>{2u, 4u, 7u} ||
        plan.constraint_candidate_track_ids != std::vector<uint64_t>{2u, 4u}) {
        std::cerr << "deterministic marginalization planning failed\n";
        return 1;
    }

    constexpr std::size_t kBaseDim = 15u;
    constexpr std::size_t kCloneDim = 6u;
    const std::vector<uint64_t> clone_ids{10u, 11u, 12u};
    constexpr Eigen::Index kAugmentedDim = 33;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(kAugmentedDim, kAugmentedDim);
    for (Eigen::Index row = 0; row < kAugmentedDim; ++row) {
        for (Eigen::Index col = 0; col < kAugmentedDim; ++col) {
            A(row, col) = std::sin(static_cast<double>((row + 1) * (col + 2))) * 0.01;
        }
    }
    const Eigen::MatrixXd covariance =
        (A * A.transpose()) + (Eigen::MatrixXd::Identity(kAugmentedDim, kAugmentedDim) * 0.1);

    const auto retained = MsckfMarginalization::retained_principal_submatrix(
        covariance, kBaseDim, kCloneDim, clone_ids, 10u);
    if (!retained.has_value() || retained->rows() != 27 || retained->cols() != 27) {
        std::cerr << "retained covariance dimension check failed\n";
        return 2;
    }

    const auto health = MsckfMarginalization::covariance_health(*retained, 1.0e-12, 1.0e-10);
    if (!health.finite || !health.symmetric || !health.psd_within_tolerance) {
        std::cerr << "retained covariance validation failed\n";
        return 3;
    }

    std::cout << "MSCKF marginalization invariant smoke: PASS\n";
    std::cout << "retiring_clone_id=" << plan.retiring_state_id << '\n';
    std::cout << "affected_tracks=" << plan.affected_track_ids.size() << '\n';
    std::cout << "constraint_candidates=" << plan.constraint_candidate_track_ids.size() << '\n';
    std::cout << "retained_covariance_dimension=" << retained->rows() << '\n';
    std::cout << "symmetry_error=" << health.symmetry_error << '\n';
    std::cout << "minimum_eigenvalue=" << health.minimum_eigenvalue << '\n';
    return 0;
}
