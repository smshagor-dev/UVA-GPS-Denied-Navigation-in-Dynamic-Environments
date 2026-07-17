# Research Release

## Project Scope

This repository is released as a publication-ready software research framework for GPS-denied drone-swarm experimentation, validation, and open collaboration.

## Validated Capabilities

- native C++ autonomy stack
- Go control plane
- Python dashboard and operator tooling
- deterministic mission, HIL, simulation, and failure-injection scenario runners
- deterministic AI perception, planning, swarm coordination, benchmark, and safety workflows
- enterprise deployment, Kubernetes, monitoring, and backup/recovery software artifacts
- software-only multi-agent AI, RL abstraction, digital twin, world-model, and XAI evidence
- publication package, citation package, artifact evaluation package, and replication documentation
- tracked configuration validation
- benchmark and regression evidence

## Phase 14 Maturity Status

Phase 14 completed the external-facing maturity and industry-readiness packaging for the repository.

Final maturity classification:

- research readiness: PASS
- software engineering maturity: PASS
- production architecture status: PASS
- external-facing software review readiness: PASS
- physical deployment readiness: pending hardware validation
- artifact maturity: PASS

## Complete Phase History

- Phase 1: foundation, repository hygiene, and baseline validation
- Phase 2: build system, dependencies, and cross-platform validation
- Phase 3: CI/CD and control-plane orchestration documentation
- Phase 4: architecture, safety, sanitizer, and software-validation hardening
- Phase 5: deployment, reproducibility, observability, and production-readiness foundations
- Phase 6: performance engineering and stability validation
- Phase 7: research validation, software HIL, and scenario framework
- Phase 8: open research release and advanced scenario evaluation
- Phase 9: AI autonomy and intelligent decision layer
- Phase 10: enterprise deployment, security, and production readiness
- Phase 11: multi-agent AI, RL abstraction, digital twin, and XAI
- Phase 12: scientific publication and artifact evaluation
- Phase 13: final system maturity, readiness classification, and lifecycle presentation
- Phase 14: external validation, engineering scorecard, benchmark framework, and industry-facing readiness package

## Known Limitations

- no physical flight validation is claimed
- no hardware qualification is claimed
- no PX4, Gazebo, Ignition, or SITL validation is claimed
- no ROS2 or hardware digital-twin integration is claimed
- no peer-reviewed publication acceptance is claimed
- workstation-local benchmark evidence must not be misrepresented as field performance
- deployment manifests and production packaging exist, but no live cloud or customer production rollout is claimed

## Citation Instructions

When referencing this repository in academic or research packaging:

- cite the repository name and author
- include the repository revision or release tag if available
- identify whether you relied on mission, HIL, simulation, or regression artifacts

## Collaboration Instructions

- read `README.md`
- read `CONTRIBUTING.md`
- read `docs/phase8/COMMUNITY_GUIDE.md` for external collaboration expectations
- read `docs/phase9/PHASE9_FINAL_REPORT.md` for AI autonomy scope and evidence
- read `docs/phase11/PHASE11_FINAL_REPORT.md` for multi-agent AI and digital twin scope and evidence
- read `docs/phase12/PHASE12_FINAL_REPORT.md` for publication, citation, and artifact-evaluation scope
- preserve prior evidence under `docs/phase*/`
- contribute new experiments with reproducible scripts and machine-readable outputs

## Reproducibility Expectations

- use tracked example configs where possible
- use schema validation to confirm config shape
- use documented workflows and validation scripts instead of ad hoc runtime assumptions
- preserve phase evidence when comparing outcomes
