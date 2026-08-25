#pragma once

#include "sensors/SensorBase.hpp"

#include <optional>

namespace drone::sensors {

struct RangefinderMeasurement : SensorMeasurement {
    double distance_m{0.0};
    bool valid{true};
};

class RangefinderSensor : public SensorBase {
public:
    explicit RangefinderSensor(std::string id) : SensorBase(std::move(id), "Rangefinder") {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<RangefinderMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

private:
    std::optional<RangefinderMeasurement> latest_;
};

} // namespace drone::sensors
