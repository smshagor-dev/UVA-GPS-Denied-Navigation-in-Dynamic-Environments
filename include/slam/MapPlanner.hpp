#pragma once

#include "slam/OccupancyGridMap.hpp"

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace drone::slam {

class MapPlanner {
public:
    struct Waypoint {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        double cost{0.0};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Plan {
        std::vector<Waypoint> waypoints;
        double total_cost{0.0};
        bool used_anchor_guidance{false};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    explicit MapPlanner(OccupancyGridMap::Config map_cfg = {});

    [[nodiscard]] std::optional<Plan> plan(const OccupancyGridMap::Status& status,
                                           const Eigen::Vector3d& start,
                                           const Eigen::Vector3d& goal) const;

private:
    [[nodiscard]] int to_grid(double coordinate) const;
    [[nodiscard]] double to_world(int cell) const;

    OccupancyGridMap::Config map_cfg_;
};

} // namespace drone::slam
