#pragma once

#include "localization/TDOALocalizer.hpp"
#include "sensors/LidarSensor.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace drone::slam {

class OccupancyGridMap {
public:
    struct Config {
        float resolution_m{0.5f};
        int width_cells{120};
        int height_cells{120};
    };

    struct Status {
        double occupied_ratio{0.0};
        size_t occupied_cells{0};
        size_t total_cells{0};
        size_t known_anchor_count{0};
        size_t visible_anchor_count{0};
    };

    OccupancyGridMap();
    explicit OccupancyGridMap(Config cfg);

    void clear();
    void integrate_lidar(const drone::sensors::LidarMeasurement& scan,
                         const Eigen::Vector3d& drone_position);
    void mark_anchor(const drone::localization::TDOALocalizer::Anchor& anchor, bool visible);

    [[nodiscard]] Status status() const;

private:
    size_t index(int x, int y) const;
    bool in_bounds(int x, int y) const;

    Config cfg_;
    std::vector<uint8_t> cells_;
    size_t known_anchor_count_{0};
    size_t visible_anchor_count_{0};
};

} // namespace drone::slam
