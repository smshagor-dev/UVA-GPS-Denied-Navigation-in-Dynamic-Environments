// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#include "sensors/MotorSensor.hpp"

#include <algorithm>
#include <cmath>

namespace drone::sensors {

bool MotorSensor::initialize() {
    poll_rate_hz_ = 20;
    set_state(SensorState::RUNNING);
    return true;
}

bool MotorSensor::reconfigure(const std::string&) {
    return true;
}

void MotorSensor::poll() {
    MotorHealthMeasurement measurement;
    measurement.timestamp = now_sec();
    measurement.source_id = id_;

    const double t = measurement.timestamp;
    float summed_health = 0.0f;
    for (size_t i = 0; i < measurement.motors.size(); ++i) {
        auto& motor = measurement.motors[i];
        const float phase = static_cast<float>(t * 0.6 + static_cast<double>(i));
        motor.rpm = 4200.0f + (180.0f * std::sin(phase));
        motor.current_a = 4.5f + (0.4f * std::cos(phase * 0.7f));
        motor.temperature_c = 48.0f + (3.0f * std::sin(phase * 0.35f));
        motor.vibration_g = 0.18f + (0.04f * std::cos(phase * 0.9f));

        const float temp_penalty = std::clamp((motor.temperature_c - 55.0f) / 35.0f, 0.0f, 0.5f);
        const float vibration_penalty = std::clamp((motor.vibration_g - 0.25f) * 1.5f, 0.0f, 0.4f);
        const float current_penalty = std::clamp((motor.current_a - 6.0f) * 0.15f, 0.0f, 0.2f);
        motor.health = std::clamp(1.0f - temp_penalty - vibration_penalty - current_penalty, 0.0f, 1.0f);
        summed_health += motor.health;
    }

    measurement.average_health = summed_health / static_cast<float>(measurement.motors.size());
    measurement.critical_fault = measurement.average_health < 0.45f;
    measurement.confidence = 0.92f;
    measurement.quality = measurement.critical_fault ? SensorState::DEGRADED : SensorState::RUNNING;

    {
        std::lock_guard lock(data_mutex_);
        latest_ = measurement;
    }

    std::lock_guard lock(cb_mutex_);
    if (data_cb_) {
        data_cb_(measurement);
    }
}

} // namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
