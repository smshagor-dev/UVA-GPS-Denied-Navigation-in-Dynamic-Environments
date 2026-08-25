# Academic Technical Reference

This repository separates implementation-level mathematics from planning models and from physical validation claims.

The detailed code-to-equation reference is maintained in:

**[Mathematical Formulation and Implementation Mapping](docs/MATHEMATICAL_FORMULATION.md)**

It documents the mathematical operations used across the current codebase, including:

- IMU scaling, calibration, and sensor-side geometry;
- baseline EKF propagation, covariance prediction, measurement updates, Mahalanobis gating, and Joseph-form covariance correction;
- secondary ESKF propagation, quaternion error injection/reset, stationary detection, ZUPT, and FEJ linearization;
- MSCKF camera-state augmentation, feature triangulation, reprojection, null-space projection, chi-square gating, augmented-state correction, and clone marginalization;
- optical-flow and visual-front-end confidence calculations;
- UWB/TDOA range-difference localization, Jacobians, damped least-squares solution, synchronization, and VIO/TDOA fusion;
- occupancy-grid, map-planning, keyframe, descriptor-matching, map-point fusion, and relocalization calculations;
- autonomy scoring, target tracking, obstacle avoidance, localization recovery, and experience-memory risk functions;
- local safety constraints and descent/hold behavior;
- swarm leader scoring, formation geometry, P-control, predictive separation, time-to-collision weighting, consensus quorum, peer freshness, and cache expiry;
- peer packet lifetime, serialization metrics, trust epochs, replay protection, HMAC authentication, secure frame construction, and audit hash chaining;
- control-plane command signatures, TTL/skew/nonce rules, fleet aggregation, estimator comparison, and sustained-readiness gates.

The mathematical reference labels each item according to what it actually represents. Runtime equations and code defaults are distinguished from deterministic software heuristics, simulation placeholders, and analytical planning equations. This distinction is important when using the repository in a paper or benchmark: software covariance, confidence scores, replay repeatability, and simulated telemetry must not be presented as independent physical measurements.

## Reproducibility

For research results, record at minimum:

- repository commit SHA;
- compiler and dependency versions;
- runtime and estimator configuration;
- sensor calibration and camera intrinsics;
- UWB anchor coordinates and clock source;
- dataset, replay log, or hardware input source;
- radio and compute hardware where applicable;
- whether the result came from simulation, replay, bench/HIL, or physical flight;
- the independent reference used for position/error measurements.

## Citation

Repository citation metadata is available in [`CITATION.cff`](CITATION.cff).

Archived software record: `10.5281/zenodo.20195953`.

When citing numerical results, include the exact commit SHA and identify whether each number is analytical, runtime-derived, simulated/replayed, bench/HIL measured, or physically measured.
