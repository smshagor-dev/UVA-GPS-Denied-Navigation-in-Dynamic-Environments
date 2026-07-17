#pragma once

#include "estimation/StateEstimator.hpp"

namespace drone::estimation {

class MinimalEskfAdapter final : public StateEstimator {
public:
    explicit MinimalEskfAdapter(drone::vio::EKFConfig config = {});

    void set_validation_config(const drone::vio::EstimatorValidationConfig& config);

    void reset(const EstimatorInitialState& initial) override;
    EstimatorUpdateResult process(const EstimatorMeasurement& measurement) override;
    [[nodiscard]] EstimatorSnapshot snapshot() const override;
    [[nodiscard]] drone::vio::EstimatorDiagnostics diagnostics() const override;
    [[nodiscard]] EstimatorCapabilities capabilities() const override;
    [[nodiscard]] std::string_view name() const noexcept override;

    [[nodiscard]] drone::vio::EKFEstimator& estimator() noexcept {
        return estimator_;
    }
    [[nodiscard]] const drone::vio::EKFEstimator& estimator() const noexcept {
        return estimator_;
    }

private:
    [[nodiscard]] EstimatorUpdateResult unsupported_result(const EstimatorMeasurement& measurement,
                                                           MeasurementValidationStatus status,
                                                           std::string reason) const;

    drone::vio::EKFEstimator estimator_;
    drone::vio::EstimatorValidationConfig validation_{};
    uint64_t last_sequence_id_{0};
    double last_timestamp_s_{-1.0};
};

} // namespace drone::estimation
