#pragma once

#include "estimation/EstimatorMeasurement.hpp"
#include "vio/EKFEstimator.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace drone::estimation {

enum class EstimatorCapability : uint8_t {
    IMU_PROPAGATION = 0,
    VISUAL_POSITION_UPDATE,
    VISUAL_VELOCITY_UPDATE,
    VISUAL_ORIENTATION_UPDATE,
    DEPTH_UPDATE,
    ZERO_VELOCITY_UPDATE,
    UWB_RANGE_UPDATE,
    TDOA_UPDATE,
    LOOP_CLOSURE_CORRECTION,
    FEJ,
    MSCKF,
    COUNT,
};

struct EstimatorCapabilities {
    std::array<bool, static_cast<size_t>(EstimatorCapability::COUNT)> supported{};

    [[nodiscard]] bool has(EstimatorCapability capability) const {
        return supported[static_cast<size_t>(capability)];
    }

    void set(EstimatorCapability capability, bool value = true) {
        supported[static_cast<size_t>(capability)] = value;
    }
};

enum class EstimatorUpdateStatus : uint8_t {
    ACCEPTED = 0,
    REJECTED,
    IGNORED,
};

struct EstimatorInitialState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EstimatorSnapshot {
    drone::vio::PoseEstimate pose{};
    drone::vio::EstimatorDiagnostics diagnostics{};
    uint64_t last_sequence_id{0};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EstimatorUpdateResult {
    EstimatorUpdateStatus status{EstimatorUpdateStatus::IGNORED};
    MeasurementValidationResult validation{};
    EstimatorSnapshot snapshot{};
};

class StateEstimator {
public:
    virtual ~StateEstimator() = default;

    virtual void reset(const EstimatorInitialState& initial) = 0;
    virtual EstimatorUpdateResult process(const EstimatorMeasurement& measurement) = 0;
    [[nodiscard]] virtual EstimatorSnapshot snapshot() const = 0;
    [[nodiscard]] virtual drone::vio::EstimatorDiagnostics diagnostics() const = 0;
    [[nodiscard]] virtual EstimatorCapabilities capabilities() const = 0;
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

} // namespace drone::estimation
