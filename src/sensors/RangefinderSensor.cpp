#include "sensors/RangefinderSensor.hpp"

namespace drone::sensors {

bool RangefinderSensor::initialize() {
    set_state(SensorState::RUNNING);
    return true;
}

bool RangefinderSensor::reconfigure(const std::string&) {
    return true;
}

void RangefinderSensor::poll() {
    RangefinderMeasurement measurement;
    measurement.timestamp = now_sec();
    measurement.distance_m = 7.8;
    measurement.valid = true;
    measurement.source_id = std::string(sensor_id());
    std::lock_guard lock(data_mutex_);
    latest_ = measurement;
}

} // namespace drone::sensors
