#include "slam/OccupancyGridMap.hpp"

#include <algorithm>
#include <cmath>

namespace drone::slam {

OccupancyGridMap::OccupancyGridMap()
    : OccupancyGridMap(Config{}) {}

OccupancyGridMap::OccupancyGridMap(Config cfg)
    : cfg_(cfg)
    , cells_(static_cast<size_t>(cfg_.width_cells * cfg_.height_cells), 0) {}

void OccupancyGridMap::clear() {
    std::fill(cells_.begin(), cells_.end(), 0);
    known_anchor_count_ = 0;
    visible_anchor_count_ = 0;
}

void OccupancyGridMap::integrate_lidar(const drone::sensors::LidarMeasurement& scan, const Eigen::Vector3d& drone_position) {
    if (!scan.cloud) {
        return;
    }
    for (const auto& point : *scan.cloud) {
        const float world_x = static_cast<float>(drone_position.x()) + point.x;
        const float world_y = static_cast<float>(drone_position.y()) + point.y;
        const int gx = static_cast<int>(std::floor(world_x / cfg_.resolution_m)) + (cfg_.width_cells / 2);
        const int gy = static_cast<int>(std::floor(world_y / cfg_.resolution_m)) + (cfg_.height_cells / 2);
        if (!in_bounds(gx, gy)) {
            continue;
        }
        cells_[index(gx, gy)] = 1;
    }
}

void OccupancyGridMap::mark_anchor(const drone::localization::TDOALocalizer::Anchor& anchor, bool visible) {
    const int gx = static_cast<int>(std::floor(anchor.position.x() / cfg_.resolution_m)) + (cfg_.width_cells / 2);
    const int gy = static_cast<int>(std::floor(anchor.position.y() / cfg_.resolution_m)) + (cfg_.height_cells / 2);
    if (in_bounds(gx, gy)) {
        cells_[index(gx, gy)] = 2;
    }
    ++known_anchor_count_;
    if (visible) {
        ++visible_anchor_count_;
    }
}

OccupancyGridMap::Status OccupancyGridMap::status() const {
    Status out;
    out.total_cells = cells_.size();
    out.occupied_cells = static_cast<size_t>(std::count_if(cells_.begin(), cells_.end(), [](uint8_t cell) {
        return cell != 0;
    }));
    if (out.total_cells > 0) {
        out.occupied_ratio = static_cast<double>(out.occupied_cells) / static_cast<double>(out.total_cells);
    }
    out.known_anchor_count = known_anchor_count_;
    out.visible_anchor_count = visible_anchor_count_;
    return out;
}

size_t OccupancyGridMap::index(int x, int y) const {
    return static_cast<size_t>(y * cfg_.width_cells + x);
}

bool OccupancyGridMap::in_bounds(int x, int y) const {
    return x >= 0 && x < cfg_.width_cells && y >= 0 && y < cfg_.height_cells;
}

} // namespace drone::slam
