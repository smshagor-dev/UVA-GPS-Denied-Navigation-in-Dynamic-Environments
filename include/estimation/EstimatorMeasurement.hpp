#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace drone::estimation {

enum class SensorSource : uint8_t {
    UNKNOWN = 0,
    IMU,
    VISUAL_FRONTEND,
    MANUAL_ZUPT,
    DEPTH_SENSOR,
    TDOA_HEURISTIC,
};

enum class CoordinateFrame : uint8_t {
    UNKNOWN = 0,
    BODY,
    WORLD,
    LOCAL_ENU,
    SENSOR_LOCAL,
};

enum class MeasurementType : uint8_t {
    IMU = 0,
    VISUAL_POSE,
    ALTITUDE,
    ZERO_VELOCITY,
    TDOA_POSITION_CANDIDATE,
};

enum class MeasurementValidationStatus : uint8_t {
    ACCEPTED = 0,
    REJECTED_INVALID,
    REJECTED_STALE,
    REJECTED_UNSUPPORTED,
    REJECTED_FRAME_MISMATCH,
    REJECTED_OUTLIER,
    IGNORED_SHADOW_ONLY,
};

struct MeasurementCovariance {
    Eigen::Matrix<double, 6, 6> matrix{Eigen::Matrix<double, 6, 6>::Zero()};
    uint8_t dimension{0};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MeasurementHeader {
    double timestamp_s{0.0};
    uint64_t sequence_id{0};
    SensorSource source{SensorSource::UNKNOWN};
    CoordinateFrame frame{CoordinateFrame::UNKNOWN};
    MeasurementType type{MeasurementType::IMU};
};

struct ImuMeasurementData {
    Eigen::Vector3d acceleration_mps2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity_rads{Eigen::Vector3d::Zero()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct VisualPoseMeasurementData {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    std::optional<Eigen::Quaterniond> orientation;
    MeasurementCovariance covariance{};
    double quality{0.0};
    bool orientation_consumed{false};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct AltitudeMeasurementData {
    double altitude_m{0.0};
    double sigma_m{0.0};
};

struct ZeroVelocityMeasurementData {
    double sigma_velocity_mps{1.0e-3};
};

struct TdoaPositionMeasurementData {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double confidence{0.0};
    bool heuristic_only{true};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using EstimatorMeasurementData =
    std::variant<ImuMeasurementData, VisualPoseMeasurementData, AltitudeMeasurementData,
                 ZeroVelocityMeasurementData, TdoaPositionMeasurementData>;

struct EstimatorMeasurement {
    MeasurementHeader header{};
    EstimatorMeasurementData data{};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MeasurementValidationResult {
    MeasurementValidationStatus status{MeasurementValidationStatus::ACCEPTED};
    std::string reason{"accepted"};
    bool consumed{false};
    bool orientation_consumed{false};
};

} // namespace drone::estimation
