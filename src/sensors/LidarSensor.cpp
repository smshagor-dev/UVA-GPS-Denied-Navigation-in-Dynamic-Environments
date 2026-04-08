// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "sensors/LidarSensor.hpp"

namespace drone::sensors {

bool LidarSensor::initialize() {
    if (logger_) {
        logger_->info("[{}] initialize lidar endpoint={} range=[{:.1f},{:.1f}]",
                      id_, endpoint_, range_min_, range_max_);
    }
    poll_rate_hz_ = 10;
    set_state(SensorState::RUNNING);
    return true;
}

bool LidarSensor::reconfigure(const std::string&) {
    if (logger_) {
        logger_->info("[{}] reconfigure requested", id_);
    }
    return true;
}

void LidarSensor::poll() {
    auto cloud = receive_udp_packet();
    if (!cloud || cloud->empty()) {
        if (logger_) {
            logger_->debug("[{}] poll no point cloud available", id_);
        }
        return;
    }

    cloud = downsample(remove_outliers(cloud));

    LidarMeasurement measurement;
    measurement.timestamp = now_sec();
    measurement.source_id = id_;
    measurement.cloud = cloud;
    measurement.num_points = static_cast<uint32_t>(cloud->size());
    measurement.range_min_m = range_min_;
    measurement.range_max_m = range_max_;

    if (logger_) {
        logger_->debug("[{}] cloud received points={} ts={:.3f}",
                       id_, measurement.num_points, measurement.timestamp);
    }

    {
        std::lock_guard lock(data_mutex_);
        latest_ = measurement;
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) {
        data_cb_(measurement);
    }
}

PointCloudPtr LidarSensor::receive_udp_packet() {
    if (logger_) {
        logger_->debug("[{}] receive_udp_packet invoked", id_);
    }
    return {};
}

PointCloudPtr LidarSensor::downsample(PointCloudPtr in) const {
    if (logger_) {
        logger_->debug("[{}] downsample passthrough", id_);
    }
    return in;
}

PointCloudPtr LidarSensor::remove_outliers(PointCloudPtr in) const {
    if (logger_) {
        logger_->debug("[{}] remove_outliers passthrough", id_);
    }
    return in;
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
