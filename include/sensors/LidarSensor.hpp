// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once
 
// LidarSensor.hpp    LiDAR interface (Velodyne VLP-16 / RPLIDAR A3)
// Drone Swarm Sensor Fusion  |  Phase 2
 
#include "sensors/SensorBase.hpp"
#include "runtime/RuntimeMode.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace drone::sensors {

using PointCloud    = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;

struct RawLidarPacket {
    std::vector<uint8_t> bytes{};
    std::string source_host{};
    uint16_t source_port{0};
    Timestamp timestamp{0.0};
};

struct LidarPoint {
    Eigen::Vector3f xyz{Eigen::Vector3f::Zero()};
    float intensity{0.0f};
    float range_m{0.0f};
    float azimuth_deg{0.0f};
    float elevation_deg{0.0f};
};

struct LidarScan {
    Timestamp timestamp{0.0};
    std::string frame_id{"lidar"};
    std::vector<LidarPoint> points{};
    bool simulated{false};
};

class ILidarParser {
public:
    virtual ~ILidarParser() = default;
    [[nodiscard]] virtual std::string_view model_name() const noexcept = 0;
    [[nodiscard]] virtual std::optional<LidarScan> parse(const RawLidarPacket& packet) const = 0;
};

[[nodiscard]] std::optional<RawLidarPacket> receive_lidar_udp_packet(
    int socket_fd,
    size_t max_bytes,
    int timeout_ms);

[[nodiscard]] std::unique_ptr<ILidarParser> create_lidar_parser(
    std::string_view model,
    const std::string& frame_id,
    bool allow_placeholder_parser);

[[nodiscard]] PointCloudPtr point_cloud_from_scan(
    const LidarScan& scan,
    float min_range_m,
    float max_range_m);

//  LiDAR Measurement 
struct LidarMeasurement : SensorMeasurement {
    PointCloudPtr cloud;           // raw scan
    std::vector<LidarPoint> points;
    uint32_t      num_points{0};
    float         range_min_m{0.1f};
    float         range_max_m{100.0f};
    float         angular_res_deg{0.1f};
    std::string   frame_id{"lidar"};
    bool          simulated{false};
};

 
class LidarSensor : public SensorBase {
public:
    struct TelemetryStats {
        bool scan_active{false};
        bool simulated{false};
        double packet_rate_hz{0.0};
        double scan_age_ms{0.0};
        uint32_t point_count{0};
        float min_range_m{0.0f};
        float max_range_m{0.0f};
        std::vector<LidarPoint> latest_points{};
        std::string status{"unavailable"};
    };

    explicit LidarSensor(std::string id, std::string endpoint = "192.168.1.201:2368")
        : SensorBase(std::move(id), "LiDAR")
        , endpoint_(std::move(endpoint)) {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;
    void stop() override;

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
    void set_runtime_mode(drone::runtime::RuntimeMode mode) { runtime_mode_ = mode; }
    void configure_socket(std::string host, uint16_t port) {
        bind_host_ = std::move(host);
        udp_port_ = port;
        endpoint_ = bind_host_ + ":" + std::to_string(udp_port_);
    }
    void configure_parser(std::string model, std::string frame_id, int timeout_ms = 75) {
        model_ = std::move(model);
        frame_id_ = std::move(frame_id);
        udp_timeout_ms_ = timeout_ms;
    }
    [[nodiscard]] TelemetryStats telemetry_stats() const;
    [[nodiscard]] bool has_recent_scan(double max_age_s = 1.0) const;
    [[nodiscard]] std::string last_status() const {
        std::lock_guard lock(status_mutex_);
        return last_status_;
    }

private:
    std::optional<LidarScan> receive_udp_packet();
    PointCloudPtr downsample(PointCloudPtr in) const;
    PointCloudPtr remove_outliers(PointCloudPtr in) const;
    void set_status(std::string value) {
        std::lock_guard lock(status_mutex_);
        last_status_ = std::move(value);
    }

    std::string                   endpoint_;
    std::string                   bind_host_{"0.0.0.0"};
    uint16_t                      udp_port_{2368};
    int                           udp_sock_{-1};
    int                           udp_timeout_ms_{75};
    std::string                   model_{"generic_udp_cartesian_v1"};
    std::string                   frame_id_{"lidar"};
    drone::runtime::RuntimeMode   runtime_mode_{drone::runtime::RuntimeMode::SIMULATION};
    std::unique_ptr<ILidarParser> parser_{};

    std::optional<LidarMeasurement> latest_;
    DataCallback<LidarMeasurement>  data_cb_;
    mutable std::mutex              status_mutex_;
    std::string                     last_status_{"not initialized"};
    Timestamp                       last_packet_timestamp_{0.0};
    double                          packet_rate_estimate_hz_{0.0};
    double                          previous_packet_timestamp_{0.0};

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
