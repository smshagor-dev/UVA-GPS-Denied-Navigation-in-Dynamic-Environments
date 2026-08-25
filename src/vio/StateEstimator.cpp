#include "vio/StateEstimator.hpp"

namespace drone::vio {

std::string_view to_string(EstimatorHealth health) {
    switch (health) {
    case EstimatorHealth::Healthy:
        return "healthy";
    case EstimatorHealth::Uninitialized:
        return "uninitialized";
    case EstimatorHealth::Degraded:
        return "degraded";
    case EstimatorHealth::Failed:
        return "failed";
    }
    return "unknown";
}

EstimatorHealth health_from_result(EstimatorOperationResult result, bool initialized) {
    if (!initialized) {
        return EstimatorHealth::Uninitialized;
    }
    switch (result) {
    case EstimatorOperationResult::Accepted:
        return EstimatorHealth::Healthy;
    case EstimatorOperationResult::RejectedUnsupportedMeasurement:
    case EstimatorOperationResult::RejectedDuplicateTimestamp:
    case EstimatorOperationResult::RejectedBackwardTimestamp:
    case EstimatorOperationResult::RejectedTimeStepTooSmall:
    case EstimatorOperationResult::RejectedTimeStepTooLarge:
        return EstimatorHealth::Degraded;
    case EstimatorOperationResult::RejectedNotInitialized:
        return EstimatorHealth::Uninitialized;
    case EstimatorOperationResult::RejectedNonFiniteInput:
    case EstimatorOperationResult::RejectedInvalidTimestamp:
    case EstimatorOperationResult::RejectedInvalidCovariance:
    case EstimatorOperationResult::RejectedInvalidQuaternion:
    case EstimatorOperationResult::RejectedDimensionMismatch:
    case EstimatorOperationResult::RejectedInvalidConfiguration:
    case EstimatorOperationResult::FailedFactorization:
    case EstimatorOperationResult::FailedNumericalValidation:
        return EstimatorHealth::Failed;
    }
    return EstimatorHealth::Failed;
}

} // namespace drone::vio
