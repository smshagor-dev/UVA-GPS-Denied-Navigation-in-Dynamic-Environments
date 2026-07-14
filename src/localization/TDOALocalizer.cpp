// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "localization/TDOALocalizer.hpp"
#include "utils/RuntimeLogging.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace drone::localization {

namespace {

constexpr double kMinDistanceM = 1.0e-3;

} // namespace

TDOALocalizer::TDOALocalizer()
    : TDOALocalizer(Config{}) {}

TDOALocalizer::TDOALocalizer(Config cfg)
    : cfg_(cfg) {
    drone::utils::get_or_create_logger("TDOA")->info(
        "TDOALocalizer initialized iterations={} damping={} eps={}",
        cfg_.max_iterations, cfg_.damping, cfg_.convergence_eps_m);
}

void TDOALocalizer::set_anchors(std::vector<Anchor> anchors) {
    anchors_ = std::move(anchors);
    drone::utils::get_or_create_logger("TDOA")->info("TDOALocalizer anchors set count={}", anchors_.size());
}

std::optional<TDOALocalizer::Solution> TDOALocalizer::estimate(
        const std::vector<Measurement>& measurements,
        std::optional<Eigen::Vector3d> seed) const {
    auto logger = drone::utils::get_or_create_logger("TDOA");
    logger->debug("TDOALocalizer estimate anchors={} measurements={} seeded={}",
                  anchors_.size(), measurements.size(), seed.has_value());
    if (anchors_.size() < 4 || measurements.size() < 4) {
        logger->warn("TDOALocalizer estimate rejected due to insufficient anchors/measurements");
        return std::nullopt;
    }

    const auto ref_it = std::min_element(
        measurements.begin(), measurements.end(),
        [](const Measurement& lhs, const Measurement& rhs) {
            return lhs.arrival_time_s < rhs.arrival_time_s;
        });
    if (ref_it == measurements.end()) {
        logger->warn("TDOALocalizer failed to choose reference measurement");
        return std::nullopt;
    }

    const Anchor* ref_anchor = find_anchor(ref_it->anchor_id);
    if (!ref_anchor) {
        logger->warn("TDOALocalizer missing reference anchor id={}", ref_it->anchor_id);
        return std::nullopt;
    }

    Eigen::Vector3d x = seed.value_or(Eigen::Vector3d::Zero());
    if (!seed.has_value()) {
        for (const auto& anchor : anchors_) {
            x += anchor.position;
        }
        x /= static_cast<double>(anchors_.size());
    }

    Solution solution;

    for (size_t iter = 0; iter < cfg_.max_iterations; ++iter) {
        Eigen::MatrixXd J(measurements.size() - 1, 3);
        Eigen::VectorXd r(measurements.size() - 1);
        size_t row = 0;

        const double ref_range = std::max((x - ref_anchor->position).norm(), kMinDistanceM);
        const Eigen::Vector3d ref_grad = (x - ref_anchor->position) / ref_range;

        for (const auto& measurement : measurements) {
            if (measurement.anchor_id == ref_it->anchor_id) {
                continue;
            }

            const Anchor* anchor = find_anchor(measurement.anchor_id);
            if (!anchor) {
                logger->warn("TDOALocalizer missing anchor id={}", measurement.anchor_id);
                return std::nullopt;
            }

            const double range = std::max((x - anchor->position).norm(), kMinDistanceM);
            const double predicted = range - ref_range;
            const double observed =
                (measurement.arrival_time_s - ref_it->arrival_time_s) * cfg_.signal_speed_mps;

            r(static_cast<Eigen::Index>(row)) = observed - predicted;
            J.row(static_cast<Eigen::Index>(row)) =
                ((x - anchor->position) / range - ref_grad).transpose();
            ++row;
        }

        Eigen::Matrix3d hessian =
            J.transpose() * J + (cfg_.damping * Eigen::Matrix3d::Identity());
        const Eigen::Vector3d step = hessian.ldlt().solve(J.transpose() * r);
        x += step;

        solution.rms_residual_m = std::sqrt(r.squaredNorm() / std::max<Eigen::Index>(r.size(), 1));
        if (step.norm() <= cfg_.convergence_eps_m) {
            solution.converged = true;
            break;
        }
    }

    solution.position = x;
    if (!solution.converged) {
        solution.rms_residual_m = std::max(solution.rms_residual_m, 5.0);
    }
    solution.confidence = std::clamp(1.0 - (solution.rms_residual_m / 6.0), 0.0, 1.0);
    logger->info("TDOALocalizer solution converged={} rms={:.3f} conf={:.3f} pos=[{:.2f},{:.2f},{:.2f}]",
                 solution.converged,
                 solution.rms_residual_m,
                 solution.confidence,
                 solution.position.x(), solution.position.y(), solution.position.z());
    return solution;
}

const TDOALocalizer::Anchor* TDOALocalizer::find_anchor(uint32_t id) const {
    const auto it = std::find_if(anchors_.begin(), anchors_.end(),
                                 [id](const Anchor& anchor) { return anchor.id == id; });
    return (it != anchors_.end()) ? &(*it) : nullptr;
}

} // namespace drone::localization
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
