#pragma once

#include "vio/StateEstimator.hpp"

namespace drone::vio {

class EKFStateEstimatorAdapter : public StateEstimator {
public:
    explicit EKFStateEstimatorAdapter(EKFConfig cfg = EKFConfig{}, std::string name = "ekf_active",
                                      std::string version = "phase16");

    void configure_validation(const EstimatorValidationConfig& cfg) override;
    void reset(const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
               const Eigen::Vector3d& v0) override;
    [[nodiscard]] EstimatorOperationResult
    process_measurement(const MeasurementEnvelope& envelope) override;
    [[nodiscard]] EstimatorStateSnapshot snapshot() const override;
    [[nodiscard]] EKFDiagnostics diagnostics() const override;
    [[nodiscard]] bool is_initialized() const override;
    [[nodiscard]] std::string estimator_name() const override;
    [[nodiscard]] std::string estimator_version() const override;
    [[nodiscard]] const EKFEstimator& estimator() const {
        return estimator_;
    }

private:
    EKFEstimator estimator_;
    std::string name_;
    std::string version_;
    uint64_t generation_{0};
};

} // namespace drone::vio
