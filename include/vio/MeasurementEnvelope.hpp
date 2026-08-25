#pragma once

#include "vio/EKFEstimator.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace drone::vio {

enum class MeasurementType : uint8_t {
    Imu = 0,
    VisualPose,
    VisualFeatures,
    ManualZupt,
    LidarDepth,
    DisabledLidarObservation,
};

enum class MeasurementFrame : uint8_t {
    Unknown = 0,
    Body,
    World,
    Lidar,
};

struct ImuMeasurementPayload {
    Eigen::Vector3d accel_mps2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_rads{Eigen::Vector3d::Zero()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct VisualPoseMeasurementPayload {
    Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
    double sigma_position_m{0.35};
    double sigma_velocity_mps{0.45};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct VisualFeatureMeasurementPayload {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> z_pixels{};
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> p_world{};
    Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ManualZuptMeasurementPayload {
    double sigma_velocity_mps{0.01};
};

struct LidarDepthMeasurementPayload {
    double depth_m{0.0};
    double sigma_m{0.05};
};

struct DisabledLidarObservationPayload {
    double inferred_height_m{0.0};
    double sigma_m{0.05};
};

using MeasurementPayload =
    std::variant<ImuMeasurementPayload, VisualPoseMeasurementPayload,
                 VisualFeatureMeasurementPayload, ManualZuptMeasurementPayload,
                 LidarDepthMeasurementPayload, DisabledLidarObservationPayload>;

struct MeasurementEnvelope {
    MeasurementType type{MeasurementType::Imu};
    std::string source_id{"unknown"};
    double timestamp_s{0.0};
    uint64_t sequence_id{0};
    MeasurementFrame frame{MeasurementFrame::Unknown};
    MeasurementPayload payload{};
    std::optional<double> covariance_hint{};
    std::string sensor_reference{};
    std::string metadata{};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MeasurementStamp {
    double timestamp_s{0.0};
    uint64_t sequence_id{0};
};

[[nodiscard]] bool is_measurement_payload_finite(const MeasurementEnvelope& envelope);
[[nodiscard]] bool is_measurement_envelope_valid(const MeasurementEnvelope& envelope);

} // namespace drone::vio
