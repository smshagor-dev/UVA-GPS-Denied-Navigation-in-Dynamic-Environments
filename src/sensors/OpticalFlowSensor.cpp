#include "sensors/OpticalFlowSensor.hpp"

namespace drone::sensors {

bool OpticalFlowSensor::initialize() {
    set_state(SensorState::RUNNING);
    return true;
}

bool OpticalFlowSensor::reconfigure(const std::string&) {
    return true;
}

void OpticalFlowSensor::poll() {
    OpticalFlowMeasurement measurement;
    measurement.timestamp = now_sec();
    measurement.flow_mps = Eigen::Vector2d{0.05, -0.03};
    measurement.quality_score = 0.82;
    measurement.source_id = std::string(sensor_id());
    std::lock_guard lock(data_mutex_);
    latest_ = measurement;
}

} // namespace drone::sensors
