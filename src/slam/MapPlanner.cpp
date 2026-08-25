#include "slam/MapPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace drone::slam {

MapPlanner::MapPlanner(OccupancyGridMap::Config map_cfg) : map_cfg_(std::move(map_cfg)) {}

std::optional<MapPlanner::Plan> MapPlanner::plan(const OccupancyGridMap::Status& status,
                                                 const Eigen::Vector3d& start,
                                                 const Eigen::Vector3d& goal) const {
    Plan out;

    const Eigen::Vector3d delta = goal - start;
    const double distance = delta.head<2>().norm();
    if (distance < 1.0e-6) {
        out.waypoints.push_back({goal, 0.0});
        return out;
    }

    const double occupancy_penalty = std::clamp(status.occupied_ratio, 0.0, 1.0);
    const double preferred_altitude = std::max(start.z(), goal.z());
    const int segments = std::max(
        2, static_cast<int>(std::ceil(distance / std::max(1.0, map_cfg_.resolution_m * 4.0))));

    out.waypoints.reserve(static_cast<size_t>(segments) + 1);
    for (int step = 1; step <= segments; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(segments);
        Eigen::Vector3d point = start + (delta * t);
        point.z() = preferred_altitude + (occupancy_penalty * 2.0) +
                    (status.visible_anchor_count > 0 ? 0.5 : 0.0);
        const double step_cost =
            (distance / static_cast<double>(segments)) * (1.0 + occupancy_penalty);
        out.total_cost += step_cost;
        out.waypoints.push_back({point, out.total_cost});
    }

    out.used_anchor_guidance = status.visible_anchor_count > 0;
    return out;
}

int MapPlanner::to_grid(double coordinate) const {
    return static_cast<int>(std::lround(coordinate / map_cfg_.resolution_m)) +
           (map_cfg_.width_cells / 2);
}

double MapPlanner::to_world(int cell) const {
    return (static_cast<double>(cell) - static_cast<double>(map_cfg_.width_cells) * 0.5) *
           map_cfg_.resolution_m;
}

} // namespace drone::slam
