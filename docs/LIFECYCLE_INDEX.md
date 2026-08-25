# Lifecycle Index

Date: July 19, 2026

## Scope

This index summarizes the complete 22-stage research engineering lifecycle for the repository.

Lifecycle note:

- some in-stage documents captured intermediate closure states while work was still active
- this index reflects the current program-level estimator-hardening maturity through Stage 22

## Stage 1 - Foundation & Repository Hygiene

Description:
Foundation, repository hygiene, dependency audit, clean build validation, and baseline project organization.

Implementation summary:

- repository cleanup
- documentation and license hygiene
- dependency and build artifact review
- baseline validation environment confirmation

Evidence location:

- `docs/foundation-repository-hygiene/`

Validation report:

- `docs/foundation-repository-hygiene/FINAL_REPORT.md`

Final status:

COMPLETE

## Stage 2 - Build System, Dependencies & Cross-Platform Validation

Description:
Build system hardening, dependency management, installation validation, and cross-platform preset integration.

Implementation summary:

- CMake preset matrix
- dependency resolution with `vcpkg`
- Windows and Linux build-path documentation
- reproducibility and sanitizer-oriented validation

Evidence location:

- `docs/build-system-cross-platform-validation/`

Validation report:

- `docs/build-system-cross-platform-validation/FINAL_REPORT.md`
- `docs/build-system-cross-platform-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 3 - CI/CD & Control-Plane Orchestration Documentation

Description:
Control-plane orchestration documentation and CI/CD coverage closure for the Go backend and mixed-language validation workflow.

Implementation summary:

- CI workflow documentation
- Go control-plane orchestration evidence mapping
- workflow, release, nightly, and security pipeline indexing

Evidence location:

- `docs/ci-cd-control-plane-orchestration/`

Validation report:

- `docs/ci-cd-control-plane-orchestration/CICD_REPORT.md`

Final status:

COMPLETE

## Stage 4 - Architecture, Safety & Software Validation Hardening

Description:
Architecture review, safety hardening, sanitizer evidence, benchmark capture, race analysis, and software-validation strengthening.

Implementation summary:

- architecture and configuration review
- sanitizer and Valgrind evidence
- benchmark and reproducibility artifacts
- HIL-readiness and research-validation groundwork

Evidence location:

- `docs/architecture-safety-validation-hardening/`

Validation report:

- `docs/architecture-safety-validation-hardening/FINAL_REPORT.md`
- `docs/architecture-safety-validation-hardening/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 5 - Production Readiness, Deployment & Reproducibility Foundations

Description:
Production-readiness foundations, deployment shape, observability, reproducibility, and release-preparation assets.

Implementation summary:

- deployment architecture review
- Docker and runtime smoke validation
- release-validation workflow support
- configuration reference and observability runbooks

Evidence location:

- `docs/production-readiness-deployment-reproducibility/`

Validation report:

- `docs/production-readiness-deployment-reproducibility/FINAL_REPORT.md`
- `docs/production-readiness-deployment-reproducibility/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 6 - Performance Engineering & Stability Validation

Description:
Performance engineering, benchmark evidence, stress testing, soak sampling, memory analysis, and latency characterization.

Implementation summary:

- benchmark suite and machine-readable results
- stress and soak monitoring
- CPU and memory profiling artifacts
- TSan root-cause analysis

Evidence location:

- `docs/performance-engineering-stability-validation/`

Validation report:

- `docs/performance-engineering-stability-validation/FINAL_REPORT.md`
- `docs/performance-engineering-stability-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 7 - Research Validation, Software HIL & Scenario Framework

Description:
Research validation closure through mission scenarios, software HIL, simulator abstraction, safety evidence, and failure injection.

Implementation summary:

- mission scenario framework
- software HIL harness
- simulation abstraction layer
- failure-injection and regression validation

Evidence location:

- `docs/research-validation-software-hil-scenarios/`
- `experiments/`

Validation report:

- `docs/research-validation-software-hil-scenarios/FINAL_REPORT.md`
- `docs/research-validation-software-hil-scenarios/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 8 - Open Research Release & Advanced Scenario Evaluation

Description:
Open research release, advanced autonomy scenarios, dataset standardization, and publication-oriented research packaging.

Implementation summary:

- advanced deterministic scenarios
- dataset and experiment methodology docs
- community guide and research paper outline
- benchmark comparison package

Evidence location:

- `docs/open-research-release-advanced-scenarios/`

Validation report:

- `docs/open-research-release-advanced-scenarios/FINAL_REPORT.md`
- `docs/open-research-release-advanced-scenarios/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 9 - AI Autonomy & Intelligent Decision Layer

Description:
AI autonomy and intelligent decision layer covering perception, planning, swarm intelligence, benchmark, safety, and dataset structure.

Implementation summary:

- AI perception pipeline
- AI mission planner
- swarm intelligence simulation
- AI benchmark and safety validation

Evidence location:

- `docs/ai-autonomy-intelligent-decision-layer/`
- `research/ai/`

Validation report:

- `docs/ai-autonomy-intelligent-decision-layer/FINAL_REPORT.md`
- `docs/ai-autonomy-intelligent-decision-layer/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 10 - Enterprise Deployment, Security & Production Readiness

Description:
Enterprise deployment, security, CI/CD, monitoring, reliability, disaster recovery, and scalability evidence.

Implementation summary:

- production Docker and Compose assets
- Kubernetes manifests
- monitoring and Grafana/Prometheus assets
- security audit and deployment validation

Evidence location:

- `docs/enterprise-deployment-security-readiness/`
- `deployment/`
- `results/phase10/`

Validation report:

- `docs/enterprise-deployment-security-readiness/FINAL_REPORT.md`
- `docs/enterprise-deployment-security-readiness/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 11 - Multi-Agent AI, Reinforcement Learning & Digital Twin

Description:
Multi-agent AI, RL abstraction, digital twin, world-model reasoning, and explainable autonomy evaluation.

Implementation summary:

- multi-agent coordination framework
- reinforcement-learning abstraction
- digital twin synchronization and world-model support
- XAI and AI evaluation evidence

Evidence location:

- `docs/multi-agent-ai-rl-digital-twin/`
- `research/ai/`

Validation report:

- `docs/multi-agent-ai-rl-digital-twin/FINAL_REPORT.md`
- `docs/multi-agent-ai-rl-digital-twin/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 12 - Scientific Publication & Artifact Evaluation

Description:
Scientific publication packaging, artifact evaluation, citation closure, and reproducible academic release preparation.

Implementation summary:

- publication package
- artifact evaluation package
- academic benchmark packaging
- citation and release-readiness assets

Evidence location:

- `docs/scientific-publication-artifact-evaluation/`

Validation report:

- `docs/scientific-publication-artifact-evaluation/FINAL_REPORT.md`
- `docs/scientific-publication-artifact-evaluation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 13 - Final System Maturity, Production Readiness & Research Certification

Description:
Final system maturity, production-readiness classification, and research certification packaging.

Implementation summary:

- final architecture review
- system readiness review
- quality audit and lifecycle closure

Evidence location:

- `docs/final-system-maturity-readiness-certification/`

Validation report:

- `docs/final-system-maturity-readiness-certification/FINAL_REPORT.md`
- `docs/final-system-maturity-readiness-certification/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 14 - External Validation, Benchmarking & Industry Readiness

Description:
External validation packaging, benchmark methodology closure, engineering maturity scoring, and industry-facing software readiness documentation.

Implementation summary:

- external validation report bundle
- engineering maturity scorecard
- final security and performance baseline packaging
- community readiness and research impact summaries
- final audit and external-facing presentation assets

Evidence location:

- `docs/external-validation-benchmarking-industry-readiness/`

Validation report:

- `docs/external-validation-benchmarking-industry-readiness/FINAL_REPORT.md`
- `docs/external-validation-benchmarking-industry-readiness/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 15 - Estimator Safety Hardening & Deterministic Replay Baseline

Description:
Estimator safety hardening, transactional EKF validation, deterministic replay, and Stage 15 regression closure for the native localization stack.

Implementation summary:

- estimator validation config and fail-closed runtime wiring
- timestamp-validating IMU ingestion path
- transactional propagation and correction commit behavior
- Joseph-form covariance correction flow
- deterministic replay executable and EKF regression expansion

Evidence location:

- `docs/estimator-safety-deterministic-replay-baseline/`

Validation report:

- `docs/estimator-safety-deterministic-replay-baseline/FINAL_REPORT.md`
- `docs/estimator-safety-deterministic-replay-baseline/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 16 - Shadow Estimator Architecture & Native Replay Validation

Description:
Shadow-estimator architecture, native replay validation, estimator coordination, and measurement-adapter integration for non-authoritative estimator experimentation.

Implementation summary:

- `StateEstimator` abstraction and adapter layering
- `EstimatorCoordinator` active/shadow isolation
- native replay workflow and long-duration shadow replay
- measurement envelope and adapter integration
- shadow-mode validation and evidence closure

Evidence location:

- `docs/shadow-estimator-native-replay-validation/`

Validation report:

- `docs/shadow-estimator-native-replay-validation/FINAL_REPORT.md`
- `docs/shadow-estimator-native-replay-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 17 - ESKF Mathematical Hardening & Shadow Validation

Description:
Error-state estimator mathematical hardening, safe error injection, reset-Jacobian correctness, and shadow-only Stage 17 validation.

Implementation summary:

- nominal/error-state math convention verification
- hardened IMU propagation and process-noise discretization
- transactional error-state injection and covariance reset Jacobian
- shadow-only Stage 17 integration through `EstimatorCoordinator`
- native Linux TSan evidence and replay validation closure

Evidence location:

- `docs/eskf-mathematical-hardening-shadow-validation/`

Validation report:

- `docs/eskf-mathematical-hardening-shadow-validation/FINAL_REPORT.md`
- `docs/eskf-mathematical-hardening-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 18 - Stationary Detection & ZUPT Shadow Validation

Description:
Stationary detection and automatic zero-velocity update hardening for the shadow ESKF while preserving active-estimator authority.

Implementation summary:

- IMU-only stationary detector with hysteresis
- configurable stationary thresholds, windowing, and minimum stationary duration
- shadow-only automatic ZUPT integration
- replay, sanitizer, and evidence closure

Evidence location:

- `docs/stationary-detection-zupt-shadow-validation/`

Validation report:

- `docs/stationary-detection-zupt-shadow-validation/FINAL_REPORT.md`
- `docs/stationary-detection-zupt-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 19 - First-Estimate Jacobian Shadow Validation

Description:
First-Estimate Jacobian support for the shadow ESKF with deterministic snapshot lifecycle, measurement-only Jacobian linearization, and replay-grade validation.

Implementation summary:

- nested FEJ configuration and validation
- deterministic FEJ snapshot creation, reuse, release, and reset
- FEJ-aware measurement Jacobian evaluation in the shadow estimator
- compiler, sanitizer, replay, and documentation evidence closure

Evidence location:

- `docs/first-estimate-jacobian-shadow-validation/`

Validation report:

- `docs/first-estimate-jacobian-shadow-validation/FINAL_REPORT.md`
- `docs/first-estimate-jacobian-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 20 - MSCKF Sliding-Window Shadow Validation

Description:
MSCKF sliding-window foundation for the shadow ESKF with deterministic camera-state lifecycle, replay-safe reset behavior, and oldest-first eviction.

Implementation summary:

- nested MSCKF configuration and validation
- deterministic camera-state creation, storage, lookup, and reset
- oldest-first deterministic eviction and diagnostics
- compiler, sanitizer, replay, and documentation evidence closure

Evidence location:

- `docs/msckf-sliding-window-shadow-validation/`
- `artifacts/phase20/`

Validation report:

- `docs/msckf-sliding-window-shadow-validation/FINAL_REPORT.md`
- `docs/msckf-sliding-window-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 21 - Feature Triangulation & Geometric Initialization

Description:
Shadow-only multi-view feature triangulation and geometric landmark initialization on top of the MSCKF sliding window while preserving active-estimator authority.

Implementation summary:

- feature-track observation history with normalized bearing storage
- configurable triangulation, baseline, reprojection, and depth validation
- deterministic landmark initialization and replay-safe pruning
- compiler, sanitizer, replay, and documentation evidence closure

Evidence location:

- `docs/feature-triangulation-geometric-initialization-shadow-validation/`
- `artifacts/phase21/`

Validation report:

- `docs/feature-triangulation-geometric-initialization-shadow-validation/FINAL_REPORT.md`
- `docs/feature-triangulation-geometric-initialization-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE

## Stage 22 - MSCKF Feature Constraint Update

Description:
Shadow-only MSCKF feature measurement update with null-space projection, stacked residual formation, and statistical gating while preserving active-estimator authority.

Implementation summary:

- deterministic feature-track linearization over triangulated shadow landmarks
- FEJ-aware null-space projection and stacked residual/Jacobian construction
- configurable innovation-norm and chi-square gating with transactional Joseph-form correction
- compiler, sanitizer, replay, and documentation evidence closure

Evidence location:

- `docs/msckf-feature-constraint-update-shadow-validation/`
- `artifacts/phase22/`

Validation report:

- `docs/msckf-feature-constraint-update-shadow-validation/FINAL_REPORT.md`
- `docs/msckf-feature-constraint-update-shadow-validation/VALIDATION_REPORT.md`

Final status:

COMPLETE
