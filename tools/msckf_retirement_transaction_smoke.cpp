#include "vio/MsckfRetirementTransaction.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    using drone::vio::MsckfRetirementRequest;
    using drone::vio::MsckfRetirementTransaction;

    std::vector<uint64_t> clones{1, 2, 3};
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(33, 33);
    covariance(0, 21) = 0.125;
    covariance(21, 0) = 0.125;

    MsckfRetirementRequest request;
    request.retiring_state_id = 1;
    const auto first = MsckfRetirementTransaction::prepare(request, clones, covariance);
    if (!first || !first->committed || first->retained_covariance.rows() != 27 ||
        first->retained_clone_ids != std::vector<uint64_t>({2, 3}) ||
        std::abs(first->retained_covariance(0, 15) - 0.125) > 1.0e-12 ||
        !first->covariance_health.finite || !first->covariance_health.symmetric ||
        !first->covariance_health.psd_within_tolerance) {
        std::cerr << "retirement transaction invariant failure\n";
        return 1;
    }

    request.retiring_state_id = 2;
    const auto second = MsckfRetirementTransaction::prepare(
        request, first->retained_clone_ids, first->retained_covariance);
    if (!second || second->retained_covariance.rows() != 21 ||
        second->retained_clone_ids != std::vector<uint64_t>({3})) {
        std::cerr << "repeated retirement invariant failure\n";
        return 2;
    }

    request.retiring_state_id = 3;
    const auto third = MsckfRetirementTransaction::prepare(
        request, second->retained_clone_ids, second->retained_covariance);
    if (!third || third->retained_covariance.rows() != 15 ||
        !third->retained_clone_ids.empty()) {
        std::cerr << "final retirement invariant failure\n";
        return 3;
    }

    std::cout << "MSCKF retirement transaction smoke: PASS\n";
    return 0;
}
