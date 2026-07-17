# Estimator Interface

## Added types

- `drone::estimation::StateEstimator`
- `EstimatorInitialState`
- `EstimatorUpdateResult`
- `EstimatorSnapshot`
- `EstimatorCapabilities`

## Current production-backed implementation

`MinimalEskfAdapter` delegates to the existing `drone::vio::EKFEstimator` and does not duplicate estimator mathematics.

## Capability reporting

The adapter reports only capabilities that are currently implemented:

- IMU propagation
- Visual position update
- Visual velocity update
- Depth update
- Zero-velocity update

It does not report:

- Visual orientation fusion
- TDOA update
- Loop-closure correction
- FEJ
- MSCKF
