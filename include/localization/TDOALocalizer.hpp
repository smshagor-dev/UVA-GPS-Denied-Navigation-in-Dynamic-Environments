// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone::localization {

class TDOALocalizer {
public:
    struct Anchor {
        uint32_t id{0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Measurement {
        uint32_t anchor_id{0};
        double arrival_time_s{0.0};
    };

    struct Solution {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        double rms_residual_m{0.0};
        double confidence{0.0};
        bool converged{false};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Config {
        double signal_speed_mps{299702547.0};
        size_t max_iterations{10};
        double convergence_eps_m{1.0e-4};
        double damping{1.0e-3};
    };

    TDOALocalizer();
    explicit TDOALocalizer(Config cfg);

    void set_anchors(std::vector<Anchor> anchors);
    [[nodiscard]] const std::vector<Anchor>& anchors() const { return anchors_; }

    [[nodiscard]] std::optional<Solution> estimate(
        const std::vector<Measurement>& measurements,
        std::optional<Eigen::Vector3d> seed = std::nullopt) const;

private:
    [[nodiscard]] const Anchor* find_anchor(uint32_t id) const;

    Config cfg_;
    std::vector<Anchor> anchors_;
};

} // namespace drone::localization
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
