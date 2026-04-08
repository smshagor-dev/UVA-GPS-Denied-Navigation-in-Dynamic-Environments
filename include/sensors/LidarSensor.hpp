// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// LidarSensor.hpp    LiDAR interface (Velodyne VLP-16 / RPLIDAR A3)
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "sensors/SensorBase.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <optional>

namespace drone::sensors {

using PointCloud    = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;

//  LiDAR Measurement 
struct LidarMeasurement : SensorMeasurement {
    PointCloudPtr cloud;           // raw scan
    uint32_t      num_points{0};
    float         range_min_m{0.1f};
    float         range_max_m{100.0f};
    float         angular_res_deg{0.1f};
};

 
class LidarSensor : public SensorBase {
public:
    explicit LidarSensor(std::string id, std::string endpoint = "192.168.1.201:2368")
        : SensorBase(std::move(id), "LiDAR")
        , endpoint_(std::move(endpoint)) {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<LidarMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

    void set_data_callback(DataCallback<LidarMeasurement> cb) {
        std::lock_guard lock(cb_mutex_);
        data_cb_ = std::move(cb);
    }

    //  Filtering 
    void set_voxel_leaf_size(float leaf) { voxel_leaf_ = leaf; }
    void set_range_filter(float min_m, float max_m) {
        range_min_ = min_m;
        range_max_ = max_m;
    }

private:
    PointCloudPtr receive_udp_packet();
    PointCloudPtr downsample(PointCloudPtr in) const;
    PointCloudPtr remove_outliers(PointCloudPtr in) const;

    std::string                   endpoint_;
    int                           udp_sock_{-1};

    std::optional<LidarMeasurement> latest_;
    DataCallback<LidarMeasurement>  data_cb_;

    float voxel_leaf_{0.05f};   // 5 cm voxel grid
    float range_min_{0.3f};
    float range_max_{80.0f};
    int   outlier_neighbors_{10};
    float outlier_std_mult_{1.0f};
};

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
