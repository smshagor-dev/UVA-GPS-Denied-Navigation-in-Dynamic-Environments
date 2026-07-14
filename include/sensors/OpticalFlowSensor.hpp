#pragma once

#include "sensors/SensorBase.hpp"

#include <Eigen/Core>
#include <optional>

namespace drone::sensors {

struct OpticalFlowMeasurement : SensorMeasurement {
    Eigen::Vector2d flow_mps{Eigen::Vector2d::Zero()};
    double quality_score{1.0};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class OpticalFlowSensor : public SensorBase {
public:
    explicit OpticalFlowSensor(std::string id) : SensorBase(std::move(id), "OpticalFlow") {}

    bool initialize() override;
    bool reconfigure(const std::string& config_json) override;
    void poll() override;

    [[nodiscard]] std::optional<OpticalFlowMeasurement> latest() const {
        std::lock_guard lock(data_mutex_);
        return latest_;
    }

private:
    std::optional<OpticalFlowMeasurement> latest_;
};

} // namespace drone::sensors
