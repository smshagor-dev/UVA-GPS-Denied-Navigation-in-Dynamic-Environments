# Replication Guide

Date: July 17, 2026

## Objective

This guide explains how another researcher can reproduce the repository evidence using only:

- the repository source tree
- the official Zenodo archive at `https://doi.org/10.5281/zenodo.20195953`
- tracked scripts
- tracked configuration

No external hardware is required.

## Minimum Inputs

1. Repository checkout or extracted Zenodo archive.
2. Python, Go, and CMake available in the environment.
3. Existing validation build tree, or the ability to regenerate the validation build using the repository’s documented workflows.
4. Docker installed if deployment validation is also being reproduced.

## Recommended Replication Order

### Core Sanity Check

```powershell
python scripts\validate_config_schemas.py
go test ./...
ctest --test-dir build\validation-msvc -C Release --output-on-failure
```

### Performance Evidence

```powershell
python scripts\phase6_performance_suite.py
```

Expected primary output:

- `docs/phase6/performance_results.json`

### Mission And Software HIL Evidence

```powershell
python scripts\phase7_mission_scenarios.py
python scripts\phase7_hil_runner.py
python scripts\phase7_simulation_runner.py
python scripts\phase7_failure_injection.py
```

Expected primary outputs:

- `docs/phase7/mission_results.json`
- `docs/phase7/hil_results.json`
- `docs/phase7/simulation_results.json`
- `docs/phase7/failure_injection_results.json`
- `experiments/phase7-*.json`

### Advanced Research Scenarios

```powershell
python scripts\phase8_advanced_scenarios.py
```

Expected primary output:

- `docs/phase8/advanced_scenario_results.json`

### AI Evidence

```powershell
python scripts\phase9_perception_validation.py
python scripts\phase9_mission_planner.py
python scripts\phase9_swarm_intelligence.py
python scripts\phase9_ai_benchmark.py
python scripts\phase9_ai_safety_test.py
```

Expected primary outputs:

- `docs/phase9/perception_results.json`
- `docs/phase9/mission_planning_results.json`
- `docs/phase9/swarm_intelligence_results.json`
- `docs/phase9/ai_benchmark_results.json`
- `docs/phase9/ai_safety_results.json`

### Deployment Evidence

```powershell
python deployment\scripts\phase10_validate_deployment.py
```

Expected primary output:

- `results/phase10/deployment_validation.json`

### Multi-Agent And Digital Twin Evidence

```powershell
python scripts\phase11_multi_agent.py
python scripts\phase11_rl_framework.py
python scripts\phase11_digital_twin.py
python scripts\phase11_xai.py
python scripts\phase11_ai_evaluation.py
```

Expected primary outputs:

- `docs/phase11/multi_agent_results.json`
- `docs/phase11/rl_framework_results.json`
- `docs/phase11/digital_twin_results.json`
- `docs/phase11/world_model_results.json`
- `docs/phase11/xai_results.json`
- `docs/phase11/ai_evaluation_results.json`

## Replication Notes

- The repository is designed to emit machine-readable JSON artifacts as evidence.
- Many reports in `docs/phase*/` are regenerated or updated by the scripts listed above.
- Generated numbers may vary slightly across machines, especially timing metrics, but the workflow structure and artifact paths should remain consistent.
- The DOI archive should be treated as the official archived snapshot; no secondary DOI is required or implied.

## What Replication Does Not Prove

Replication of this package proves reproducible software evidence only. It does not prove:

- physical flight readiness
- PX4 integration
- Gazebo or SITL operation
- ROS2 interoperability
- hardware qualification
- peer-reviewed publication acceptance
