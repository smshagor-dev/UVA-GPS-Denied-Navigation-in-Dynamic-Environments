# Research Release — v2.0.0

## Release Scope

v2.0.0 packages the repository as a publication-ready software research framework for GPS-denied UAV swarm experimentation, deterministic validation, open collaboration, and external software review.

This document describes the intended v2.0.0 release scope. Publication of the `v2.0.0` tag remains gated on the repository's release-validation and packaging workflows.

## Validated Software Capabilities

The repository contains software and reproducibility evidence for:

- native C++20 autonomy and sensor-fusion stack
- Go supervisory control plane
- Python/PySide6 dashboard and operator tooling
- deterministic mission, replay, software-HIL, simulation, and failure-injection scenario runners
- AI perception, planning, swarm coordination, benchmark, and safety workflows
- deployment, Kubernetes, monitoring, and backup/recovery software artifacts
- software-only multi-agent AI, RL abstraction, digital twin, world-model, and XAI evidence
- publication, citation, artifact-evaluation, replication, and DOI-oriented documentation
- tracked configuration validation, benchmark, and regression evidence
- fail-closed estimator validation and deterministic EKF replay
- active/shadow estimator architecture with unchanged active-estimator authority
- hardened shadow ESKF mathematics and deterministic replay
- stationary detection and automatic shadow-only ZUPT
- First-Estimate Jacobian support for shadow-estimator validation
- deterministic MSCKF camera-state sliding-window groundwork
- shadow-only feature triangulation and geometric landmark initialization
- shadow-only MSCKF feature constraint updates with statistical gating
- sustained estimator-readiness soak monitoring with fail-closed progress reset

## v2.0.0 Maturity Status

The software lifecycle through Stage 22 is complete in repository scope.

Release classification:

- research readiness: PASS for scientific replication of repository-supported software experiments
- software engineering maturity: PASS
- production-oriented software architecture: PASS
- external-facing software review readiness: PASS
- active/shadow estimator validation architecture: implemented
- Stage 22 shadow-only MSCKF feature update: implemented
- sustained promotion-readiness observation: implemented
- automatic estimator authority promotion: NOT INCLUDED
- physical HIL readiness: pending physical validation
- production-radio qualification: NOT VALIDATED
- tethered/free-flight readiness: NOT VALIDATED
- operational airworthiness/certification: NOT CLAIMED

## Complete Stage History

- Stage 1: foundation, repository hygiene, and baseline validation
- Stage 2: build system, dependencies, and cross-platform validation
- Stage 3: CI/CD and control-plane orchestration documentation
- Stage 4: architecture, safety, sanitizer, and software-validation hardening
- Stage 5: deployment, reproducibility, observability, and production-readiness foundations
- Stage 6: performance engineering and stability validation
- Stage 7: research validation, software HIL, and scenario framework
- Stage 8: open research release and advanced scenario evaluation
- Stage 9: AI autonomy and intelligent decision layer
- Stage 10: enterprise deployment, security, and production readiness
- Stage 11: multi-agent AI, reinforcement-learning abstraction, digital twin, and XAI
- Stage 12: scientific publication and artifact evaluation
- Stage 13: final system maturity, readiness classification, and lifecycle presentation
- Stage 14: external validation, engineering scorecard, benchmark framework, and industry-facing readiness package
- Stage 15: estimator safety hardening and deterministic replay baseline
- Stage 16: shadow estimator architecture and native replay validation
- Stage 17: ESKF mathematical hardening and shadow validation
- Stage 18: stationary detection and ZUPT shadow validation
- Stage 19: First-Estimate Jacobian shadow validation
- Stage 20: MSCKF sliding-window shadow validation
- Stage 21: feature triangulation and geometric initialization
- Stage 22: MSCKF feature constraint update

Post-Stage-22 release hardening also includes sustained estimator-readiness soak validation. This observes promotion readiness over consecutive healthy samples but does not switch estimator authority.

## Estimator Safety Boundary

v2.0.0 intentionally keeps the validated visual estimator in shadow/readiness-observation scope.

The release does not introduce:

- automatic estimator promotion
- automatic active/shadow authority switching
- flight-control activation from soak readiness
- bypass of fail-closed readiness blockers

Readiness progress resets when the underlying point-in-time readiness gate regresses, including stale or dropped measurements, estimator health degradation, processing failures, or divergence conditions represented by the existing readiness contract.

## Release Validation Contract

Before the final v2.0.0 tag is considered release-ready, applicable hosted gates should pass for the release candidate:

- Linux GCC and Clang native build/test lanes
- Windows MSVC native build/test lane
- warnings-as-errors validation
- clang-format and clang-tidy
- sanitizer and regression validation
- Go and Python checks
- package/install-consumer validation
- configuration schema and release container smoke validation
- Linux and Windows release package generation
- SBOM and SHA-256 checksum generation

GitHub Actions and software validation are repository-quality evidence only. They are not evidence of flight certification or physical deployment clearance.

## Known Limitations

- no physical free-flight validation is claimed
- no hardware qualification is claimed
- no production-radio qualification is claimed
- physical HIL remains incomplete
- no multi-drone tethered validation is claimed
- no PX4/Gazebo/Ignition/SITL validation is claimed unless separately evidenced by a future repository revision
- no ROS2 hardware digital-twin integration is claimed
- no peer-reviewed publication acceptance is claimed merely by this software release
- workstation-local or CI benchmark evidence must not be represented as field performance
- deployment manifests and production-oriented packaging exist, but no live customer or operational flight deployment is claimed

## Planned Follow-On Work

Post-v2.0.0 work should remain separated from this release baseline. Candidate directions include:

- explicitly controlled estimator authority promotion and rollback with deterministic safety evidence
- physical HIL validation
- multi-drone bench and tethered validation
- production-radio testing
- CBOR transport HIL validation and protobuf transport experiments
- packet signing, replay-nonce hardening, and post-quantum authentication experiments
- formal safety-invariant verification

## Citation Instructions

When referencing v2.0.0 in academic or research packaging:

- cite the repository name and author
- include the exact `v2.0.0` release tag or immutable commit revision used
- identify whether evidence came from replay, simulation, software HIL, benchmark, or regression artifacts
- do not describe software-only evidence as physical flight validation

## Collaboration Instructions

- read `README.md`
- read `CONTRIBUTING.md`
- read `CHANGELOG.md`
- read `docs/releases/v2.0.0.md` for release-specific scope and validation notes
- preserve prior lifecycle evidence under `docs/`
- contribute new experiments with reproducible scripts and machine-readable outputs
- keep future estimator-authority work isolated from the v2.0.0 release baseline until separately validated

## Reproducibility Expectations

- use tracked example configurations where possible
- validate configuration shape with repository-provided schema tooling
- use documented workflows and validation scripts instead of ad hoc runtime assumptions
- preserve lifecycle evidence when comparing outcomes
- record the exact commit SHA, compiler/toolchain, platform, and validation mode used for reported results
