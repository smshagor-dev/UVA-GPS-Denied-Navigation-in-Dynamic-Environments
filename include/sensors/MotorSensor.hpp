// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

#pragma once

#include "sensors/SensorBase.hpp"

#include <array>
#include <optional>

namespace drone::sensors {

struct MotorSample {
    float rpm{0.0f};
    float current_a{0.0f};
    float temperature_c{0.0f};
    float vibration_g{0.0f};
    float health{1.0f};
};

struct MotorHealthMeasurement : SensorMeasurement {
    std::array<MotorSample, 4> motors{};
    float average_health{1.0f};
    bool critical_fault{false};
};

class MotorSensor : public SensorBase {
public:
    explicit MotorSensor(std::string id = "motor0") : SensorBase(std::move(id), "MotorHealth") {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<MotorHealthMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

    void set_data_callback(DataCallback<MotorHealthMeasurement> cb) {
        std::lock_guard lock(cb_mutex_);
        data_cb_ = std::move(cb);
    }

private:
    std::optional<MotorHealthMeasurement> latest_;
    DataCallback<MotorHealthMeasurement> data_cb_;
};

} // namespace drone::sensors

// namespace drone::sensors
// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake
