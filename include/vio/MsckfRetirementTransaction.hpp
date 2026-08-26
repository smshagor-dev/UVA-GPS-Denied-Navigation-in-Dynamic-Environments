#pragma once

#include "vio/MsckfMarginalization.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone::vio {

struct MsckfRetirementRequest {
    uint64_t retiring_state_id{0};
    std::size_t base_error_dim{15};
    std::size_t clone_error_dim{6};
    double symmetry_tolerance{1.0e-9};
    double negativity_tolerance{1.0e-10};
};

struct MsckfRetirementResult {
    bool committed{false};
    uint64_t retiring_state_id{0};
    std::vector<uint64_t> retained_clone_ids{};
    Eigen::MatrixXd retained_covariance{};
    MarginalizationCovarianceHealth covariance_health{};
};

// Pure, fail-closed retirement primitive. It deliberately does not mutate estimator
// state. The caller may commit the returned clone order/covariance only when
// committed == true, after feature constraints involving the retiring clone have
// completed their own transactional update boundary.
class MsckfRetirementTransaction final {
public:
    [[nodiscard]] static std::optional<MsckfRetirementResult>
    prepare(const MsckfRetirementRequest& request, const std::vector<uint64_t>& ordered_clone_ids,
            const Eigen::MatrixXd& post_constraint_augmented_covariance) {
        if (request.retiring_state_id == 0u || request.base_error_dim == 0u ||
            request.clone_error_dim == 0u || request.symmetry_tolerance < 0.0 ||
            request.negativity_tolerance < 0.0) {
            return std::nullopt;
        }
        if (ordered_clone_ids.empty() || ordered_clone_ids.front() != request.retiring_state_id) {
            return std::nullopt;
        }
        if (std::adjacent_find(ordered_clone_ids.begin(), ordered_clone_ids.end()) !=
            ordered_clone_ids.end()) {
            return std::nullopt;
        }

        const auto retained = MsckfMarginalization::retained_principal_submatrix(
            post_constraint_augmented_covariance, request.base_error_dim, request.clone_error_dim,
            ordered_clone_ids, request.retiring_state_id);
        if (!retained.has_value()) {
            return std::nullopt;
        }

        const auto health = MsckfMarginalization::covariance_health(
            *retained, request.symmetry_tolerance, request.negativity_tolerance);
        if (!health.finite || !health.symmetric || !health.psd_within_tolerance) {
            return std::nullopt;
        }

        MsckfRetirementResult result;
        result.retiring_state_id = request.retiring_state_id;
        result.retained_clone_ids.assign(ordered_clone_ids.begin() + 1, ordered_clone_ids.end());
        result.retained_covariance = *retained;
        result.covariance_health = health;
        result.committed = true;
        return result;
    }
};

} // namespace drone::vio
