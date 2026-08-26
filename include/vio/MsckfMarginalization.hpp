#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace drone::vio {

struct MarginalizationTrackSummary {
    uint64_t track_id{0};
    std::vector<uint64_t> observation_state_ids{};
    bool landmark_initialized{false};
};

struct MarginalizationPlan {
    uint64_t retiring_state_id{0};
    std::vector<uint64_t> affected_track_ids{};
    std::vector<uint64_t> constraint_candidate_track_ids{};
    uint64_t stale_reference_count{0};
};

struct MarginalizationCovarianceHealth {
    bool finite{false};
    bool symmetric{false};
    bool psd_within_tolerance{false};
    double symmetry_error{0.0};
    double minimum_eigenvalue{0.0};
};

class MsckfMarginalization final {
public:
    [[nodiscard]] static MarginalizationPlan
    build_plan(uint64_t retiring_state_id, const std::vector<MarginalizationTrackSummary>& tracks,
               uint32_t minimum_track_length) {
        MarginalizationPlan plan;
        plan.retiring_state_id = retiring_state_id;

        std::vector<const MarginalizationTrackSummary*> ordered;
        ordered.reserve(tracks.size());
        for (const auto& track : tracks) {
            ordered.push_back(&track);
        }
        std::sort(ordered.begin(), ordered.end(),
                  [](const auto* lhs, const auto* rhs) { return lhs->track_id < rhs->track_id; });

        for (const auto* track : ordered) {
            if (track == nullptr || track->track_id == 0u) {
                continue;
            }
            const auto retiring_refs = static_cast<uint64_t>(
                std::count(track->observation_state_ids.begin(), track->observation_state_ids.end(),
                           retiring_state_id));
            if (retiring_refs == 0u) {
                continue;
            }
            plan.affected_track_ids.push_back(track->track_id);
            if (retiring_refs > 1u) {
                plan.stale_reference_count += retiring_refs - 1u;
            }
            if (track->landmark_initialized &&
                track->observation_state_ids.size() >= minimum_track_length) {
                plan.constraint_candidate_track_ids.push_back(track->track_id);
            }
        }
        return plan;
    }

    [[nodiscard]] static bool
    has_retired_state_reference(uint64_t retired_state_id,
                                const std::vector<MarginalizationTrackSummary>& tracks) {
        for (const auto& track : tracks) {
            if (std::find(track.observation_state_ids.begin(), track.observation_state_ids.end(),
                          retired_state_id) != track.observation_state_ids.end()) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] static std::optional<Eigen::MatrixXd>
    retained_principal_submatrix(const Eigen::MatrixXd& augmented_covariance,
                                 std::size_t base_error_dim, std::size_t clone_error_dim,
                                 const std::vector<uint64_t>& ordered_clone_ids,
                                 uint64_t retiring_state_id) {
        if (base_error_dim == 0u || clone_error_dim == 0u || retiring_state_id == 0u ||
            augmented_covariance.rows() != augmented_covariance.cols()) {
            return std::nullopt;
        }
        const std::size_t expected_dim =
            base_error_dim + (ordered_clone_ids.size() * clone_error_dim);
        if (augmented_covariance.rows() != static_cast<Eigen::Index>(expected_dim)) {
            return std::nullopt;
        }

        const auto clone_it =
            std::find(ordered_clone_ids.begin(), ordered_clone_ids.end(), retiring_state_id);
        if (clone_it == ordered_clone_ids.end()) {
            return std::nullopt;
        }
        const std::size_t clone_index =
            static_cast<std::size_t>(std::distance(ordered_clone_ids.begin(), clone_it));
        const std::size_t remove_begin = base_error_dim + (clone_index * clone_error_dim);
        const std::size_t remove_end = remove_begin + clone_error_dim;

        const Eigen::Index retained_dim = static_cast<Eigen::Index>(expected_dim - clone_error_dim);
        Eigen::MatrixXd retained = Eigen::MatrixXd::Zero(retained_dim, retained_dim);

        std::vector<Eigen::Index> retained_indices;
        retained_indices.reserve(static_cast<std::size_t>(retained_dim));
        for (std::size_t index = 0; index < expected_dim; ++index) {
            if (index < remove_begin || index >= remove_end) {
                retained_indices.push_back(static_cast<Eigen::Index>(index));
            }
        }
        for (Eigen::Index row = 0; row < retained_dim; ++row) {
            for (Eigen::Index col = 0; col < retained_dim; ++col) {
                retained(row, col) =
                    augmented_covariance(retained_indices[static_cast<std::size_t>(row)],
                                         retained_indices[static_cast<std::size_t>(col)]);
            }
        }
        return 0.5 * (retained + retained.transpose());
    }

    [[nodiscard]] static MarginalizationCovarianceHealth
    covariance_health(const Eigen::MatrixXd& covariance, double symmetry_tolerance,
                      double negativity_tolerance) {
        MarginalizationCovarianceHealth health;
        if (covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
            !covariance.array().isFinite().all() || !std::isfinite(symmetry_tolerance) ||
            !std::isfinite(negativity_tolerance) || symmetry_tolerance < 0.0 ||
            negativity_tolerance < 0.0) {
            return health;
        }

        health.finite = true;
        const Eigen::MatrixXd symmetric = 0.5 * (covariance + covariance.transpose());
        health.symmetry_error = (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
        health.symmetric = health.symmetry_error <= symmetry_tolerance;

        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
        if (solver.info() != Eigen::Success) {
            health.minimum_eigenvalue = -std::numeric_limits<double>::infinity();
            return health;
        }
        health.minimum_eigenvalue = solver.eigenvalues().minCoeff();
        health.psd_within_tolerance = std::isfinite(health.minimum_eigenvalue) &&
                                      health.minimum_eigenvalue >= -negativity_tolerance;
        return health;
    }
};

} // namespace drone::vio
