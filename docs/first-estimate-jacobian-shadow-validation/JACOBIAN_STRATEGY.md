# Phase 19 Jacobian Strategy

Date: July 18, 2026

## Linearization Policy

Phase 19 applies FEJ only to measurement Jacobians inside `Phase17ESKFEstimator::update_vision`.

The nominal prediction path still uses the current state:

- current orientation for projection
- current position for projection
- current covariance propagation and update

The FEJ path affects only the linearization point used for selected Jacobian terms.

## FEJ Jacobian Evaluation

For each observed feature:

- the feature is mapped to a deterministic quantized world-space key
- the FEJ snapshot is created or reused
- Jacobian position terms use the stored first position estimate
- Jacobian rotation terms use the stored first orientation estimate
- Jacobian dimensions and sparsity remain unchanged

If FEJ is disabled, the existing current-estimate Jacobian path is used and FEJ Jacobian evaluation counters remain zero.

## Validation Checks

Snapshot validity checks ensure the stored FEJ state remains finite and the stored quaternion stays normalized enough for safe use.

If a snapshot becomes invalid:

- `fej_validation_failures` is incremented
- `FailedNumericalValidation` is recorded when FEJ validation checks are enabled

This preserves the existing estimator-validation style rather than silently continuing through invalid state.

## Determinism And Stability

Determinism is preserved by:

- deterministic snapshot id allocation
- deterministic quantized feature keys
- deterministic snapshot release policy
- unchanged nominal propagation path

Covariance stability is preserved because Phase 19 does not alter:

- Joseph-form covariance update
- transactional measurement application
- covariance symmetry validation
- finite-value validation

Replay evidence for deterministic FEJ behavior is persisted in `artifacts/phase19/ekf_phase19_replay_report.json`.
