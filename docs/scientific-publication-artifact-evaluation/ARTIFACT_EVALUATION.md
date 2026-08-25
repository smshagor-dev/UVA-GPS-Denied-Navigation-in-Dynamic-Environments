# Artifact Evaluation

Date: July 17, 2026

## Scope

This document summarizes the repository as a software artifact suitable for replication-oriented evaluation. It covers repository layout, build instructions, dependency and tool versions, supported platforms, validation workflows, expected outputs, and artifact hashes.

## Repository Layout

Top-level directories relevant to artifact evaluation:

- `.github/` workflow automation
- `build/` generated validation builds
- `cmd/` Go executable entrypoints
- `config/` tracked runtime and schema-backed configuration
- `datasets/` benchmark and autonomy datasets
- `deployment/` Docker, Compose, Kubernetes, monitoring, and validation assets
- `docs/` phase evidence and architecture documentation
- `experiments/` experiment metadata and run manifests
- `gui/` Python dashboard
- `include/` public C++ headers
- `internal/` Go control-plane implementation
- `research/` AI, evaluation, multi-agent, RL, XAI, and experiment scaffolding
- `results/` generated validation results
- `scripts/` reproducible validation and experiment runners
- `simulation/` deterministic simulation and digital twin code
- `src/` native autonomy implementation
- `tests/` Python and native test sources

## Build And Validation Instructions

Primary validation commands confirmed during Phase 12:

```powershell
go test ./...
python scripts\validate_config_schemas.py
ctest --test-dir build\validation-msvc -C Release --output-on-failure
python scripts\phase6_performance_suite.py
python scripts\phase7_mission_scenarios.py
python scripts\phase7_hil_runner.py
python scripts\phase7_simulation_runner.py
python scripts\phase7_failure_injection.py
python scripts\phase8_advanced_scenarios.py
python scripts\phase9_perception_validation.py
python scripts\phase9_mission_planner.py
python scripts\phase9_swarm_intelligence.py
python scripts\phase9_ai_benchmark.py
python scripts\phase9_ai_safety_test.py
python deployment\scripts\phase10_validate_deployment.py
```

## Toolchain Versions

Observed on the Phase 12 validation workstation:

- Python: `3.14.5`
- Go: `go1.26.4 windows/amd64`
- CMake: `4.4.0`
- Docker: `29.5.3`
- Git revision: `58daa36decd2f6a9e846acc0c60a09e863c88b4d`

## Compiler Notes

- The active shell did not expose `g++`, `clang++`, or `cl` directly during this documentation pass.
- Windows native validation was still confirmed through the existing `build/validation-msvc` tree with `114/114` passing tests.
- Linux compiler coverage is represented by the tracked GitHub Actions workflows and earlier phase reports, not by a fresh Linux rerun inside this Windows session.

## Platform Support

### Windows

- Confirmed during Phase 12 with `ctest --test-dir build/validation-msvc -C Release --output-on-failure`
- Result: `114/114` tests passed

### Linux

- Supported through tracked CI workflows in `.github/workflows/ci.yml`, `.github/workflows/nightly.yml`, and `.github/workflows/release.yml`
- No new Linux rerun was claimed from this Windows workstation

### Docker Support

- Confirmed during Phase 10 deployment validation
- `docker-compose.yml` and `docker-compose.production.yml` both rendered successfully through `docker compose ... config`

## Configuration Schema Support

Tracked configurations were revalidated in Phase 12 through:

- `config/runtime.json`
- `config/anchors.json`
- `config/lidar.json`
- `config/detector_labels.json`
- `config/swarm_edge_protocol.json`

Result: all schema validations passed.

## Validation Workflow Coverage

- unit and integration tests
- native navigation, sensor, security, telemetry, and swarm tests
- Go backend tests
- schema validation
- performance benchmark and stress evidence
- mission and software HIL scenario evidence
- advanced research scenario evidence
- AI perception, planning, swarm, benchmark, and safety evidence
- deployment and Compose validation
- multi-agent, RL abstraction, digital twin, XAI, and world-model evidence

## Expected Outputs

Representative machine-readable outputs include:

- `docs/performance-engineering-stability-validation/performance_results.json`
- `docs/research-validation-software-hil-scenarios/mission_results.json`
- `docs/research-validation-software-hil-scenarios/hil_results.json`
- `docs/research-validation-software-hil-scenarios/simulation_results.json`
- `docs/research-validation-software-hil-scenarios/failure_injection_results.json`
- `docs/open-research-release-advanced-scenarios/advanced_scenario_results.json`
- `docs/ai-autonomy-intelligent-decision-layer/perception_results.json`
- `docs/ai-autonomy-intelligent-decision-layer/mission_planning_results.json`
- `docs/ai-autonomy-intelligent-decision-layer/swarm_intelligence_results.json`
- `docs/ai-autonomy-intelligent-decision-layer/ai_benchmark_results.json`
- `docs/ai-autonomy-intelligent-decision-layer/ai_safety_results.json`
- `results/phase10/deployment_validation.json`
- `docs/multi-agent-ai-rl-digital-twin/multi_agent_results.json`
- `docs/multi-agent-ai-rl-digital-twin/rl_framework_results.json`
- `docs/multi-agent-ai-rl-digital-twin/digital_twin_results.json`
- `docs/multi-agent-ai-rl-digital-twin/world_model_results.json`
- `docs/multi-agent-ai-rl-digital-twin/xai_results.json`
- `docs/multi-agent-ai-rl-digital-twin/ai_evaluation_results.json`

## Artifact Hashes

- `docs/performance-engineering-stability-validation/performance_results.json`: `3daf81a87b86d7211e577649071b24c62f4b38cf605686db327259114448f88e`
- `docs/research-validation-software-hil-scenarios/hil_results.json`: `9cf1a714eb85bd77c800d9875bcc673e9b1bd89c023b9a00fe710d6586b23004`
- `docs/research-validation-software-hil-scenarios/simulation_results.json`: `7a31f8aae9861f60881c03d9233510d2151f2b49afb1c04823307f039fc080e5`
- `docs/research-validation-software-hil-scenarios/failure_injection_results.json`: `bd2d0ba0ebeee5950731f408849ec5942c5aa84d8c4b295cb685ef056f94df34`
- `docs/open-research-release-advanced-scenarios/advanced_scenario_results.json`: `838dd525b7dbcf119ab7898f911fbe10471b9f955f720704fa3f8785027b1c8d`
- `docs/ai-autonomy-intelligent-decision-layer/perception_results.json`: `a660bd80bce29084f9717a82619085ddfa0354ed639797c57ad42a817bcf7f8e`
- `docs/ai-autonomy-intelligent-decision-layer/mission_planning_results.json`: `37f3402be97af484aa3afa894eadf575ba41109db5435c4c75b1bf1193a17ca9`
- `docs/ai-autonomy-intelligent-decision-layer/swarm_intelligence_results.json`: `ba7ce86bb06f45ec106c8c2b47f40f91bc824baafb01ab76ac54e83710613d91`
- `docs/ai-autonomy-intelligent-decision-layer/ai_benchmark_results.json`: `754a12a5223948718912954e331512599af7e67e979ebd22641b06932901eedb`
- `docs/ai-autonomy-intelligent-decision-layer/ai_safety_results.json`: `3a41d412b84887df85b7622a03e76be7b88adcaef60332b71caa4eeee9325661`
- `results/phase10/deployment_validation.json`: `c0890d6a308b30cd4164d0d0727b4926c4a49879d6b4fec53913d58407f28ff2`
- `docs/multi-agent-ai-rl-digital-twin/multi_agent_results.json`: `8f97d8e5003cf6a3b8eb3c5292ceb9b8b85371cb9c0919366ad88d658bda3dce`
- `docs/multi-agent-ai-rl-digital-twin/rl_framework_results.json`: `57e2a114f6c4ce74d71f0e99177d5b000ad1984825f9b3b9cb989547ceb375ab`
- `docs/multi-agent-ai-rl-digital-twin/digital_twin_results.json`: `e4eaea82e3aca0c124d3d708c3e424e9a1aaf90d5d3d5b179bb9482f0f5fc932`
- `docs/multi-agent-ai-rl-digital-twin/world_model_results.json`: `b6954eb08b5e4fe5d714865dcc5891d0f110f397743ffe17eabc75bcc47ab86c`
- `docs/multi-agent-ai-rl-digital-twin/xai_results.json`: `3a42e54f20df1e6a22d783059e43807a8c44d36a4fa974988b169896e2c23e52`
- `docs/multi-agent-ai-rl-digital-twin/ai_evaluation_results.json`: `453ca956f31d8aebd07fae6e091586de30f191e15ecb43efa5936a66d9c89d32`

## Artifact Verdict

Status: PASS

This repository is artifact-ready for software research replication and documentation review. No claim is made that the artifact constitutes physical flight validation or hardware certification.

