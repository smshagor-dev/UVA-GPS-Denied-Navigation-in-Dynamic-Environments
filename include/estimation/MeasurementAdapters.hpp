#pragma once

#include "estimation/EstimatorMeasurement.hpp"
#include "sensors/IMUSensor.hpp"

#include <optional>

namespace drone::estimation {

struct MeasurementAdapterResult {
    MeasurementValidationResult validation{};
    std::optional<EstimatorMeasurement> measurement;
};

class MeasurementAdapters {
public:
    [[nodiscard]] static MeasurementAdapterResult
    adapt_imu(const drone::sensors::ImuMeasurement& imu, uint64_t sequence_id);
    [[nodiscard]] static MeasurementAdapterResult
    adapt_visual_pose(double timestamp_s, const Eigen::Vector3d& position,
                      const Eigen::Vector3d& velocity, const Eigen::Quaterniond& orientation,
                      double quality, bool update_accepted, uint64_t sequence_id,
                      bool allow_orientation_fusion);
    [[nodiscard]] static MeasurementAdapterResult adapt_manual_zupt(double timestamp_s,
                                                                    uint64_t sequence_id);
    [[nodiscard]] static MeasurementAdapterResult adapt_altitude(double timestamp_s,
                                                                 double altitude_m, double sigma_m,
                                                                 CoordinateFrame frame,
                                                                 uint64_t sequence_id);
    [[nodiscard]] static MeasurementAdapterResult
    adapt_tdoa_position_candidate(double timestamp_s, const Eigen::Vector3d& position,
                                  double confidence, uint64_t sequence_id);
    [[nodiscard]] static MeasurementValidationResult
    validate(const EstimatorMeasurement& measurement, double newest_timestamp_s,
             double max_staleness_s);
};

} // namespace drone::estimation
