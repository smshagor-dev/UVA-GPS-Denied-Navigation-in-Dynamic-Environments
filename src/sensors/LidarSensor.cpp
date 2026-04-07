#include "sensors/LidarSensor.hpp"

namespace drone::sensors {

bool LidarSensor::initialize() {
    poll_rate_hz_ = 10;
    set_state(SensorState::RUNNING);
    return true;
}

bool LidarSensor::reconfigure(const std::string&) {
    return true;
}

void LidarSensor::poll() {
    auto cloud = receive_udp_packet();
    if (!cloud || cloud->empty()) {
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
    return {};
}

PointCloudPtr LidarSensor::downsample(PointCloudPtr in) const {
    return in;
}

PointCloudPtr LidarSensor::remove_outliers(PointCloudPtr in) const {
    return in;
}

} // namespace drone::sensors
