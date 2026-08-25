#pragma once

#include "sensors/SensorBase.hpp"

#include <optional>

namespace drone::sensors {

struct BarometerMeasurement : SensorMeasurement {
    double altitude_m{0.0};
    double pressure_pa{101325.0};
    double temperature_c{25.0};
};

class BarometerSensor : public SensorBase {
public:
    explicit BarometerSensor(std::string id) : SensorBase(std::move(id), "Barometer") {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<BarometerMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

private:
    std::optional<BarometerMeasurement> latest_;
};

} // namespace drone::sensors
