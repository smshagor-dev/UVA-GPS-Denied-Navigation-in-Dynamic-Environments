#include "sensors/BarometerSensor.hpp"

namespace drone::sensors {

bool BarometerSensor::initialize() {
    set_state(SensorState::RUNNING);
    return true;
}

bool BarometerSensor::reconfigure(const std::string&) {
    return true;
}

void BarometerSensor::poll() {
    BarometerMeasurement measurement;
    measurement.timestamp = now_sec();
    measurement.altitude_m = 8.0;
    measurement.pressure_pa = 101325.0 - (measurement.altitude_m * 12.0);
    measurement.temperature_c = 25.0;
    measurement.source_id = std::string(sensor_id());
    std::lock_guard lock(data_mutex_);
    latest_ = measurement;
}

} // namespace drone::sensors
