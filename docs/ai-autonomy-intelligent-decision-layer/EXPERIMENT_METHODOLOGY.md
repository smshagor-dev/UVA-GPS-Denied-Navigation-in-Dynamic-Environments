# Phase 9 Experiment Methodology

Date: July 16, 2026

## Protocol

1. Generate deterministic software scenarios.
2. Run perception, planning, swarm, benchmark, and safety scripts.
3. Persist machine-readable JSON artifacts.
4. Validate repository-wide regressions with config, Go, and native tests.
5. Document limitations explicitly.

## Reproducibility Commands

```powershell
python scripts\phase9_perception_validation.py
python scripts\phase9_mission_planner.py
python scripts\phase9_swarm_intelligence.py
python scripts\phase9_ai_benchmark.py
python scripts\phase9_ai_safety_test.py
python scripts\validate_config_schemas.py
go test ./...
ctest --test-dir build\validation-msvc -C Release --output-on-failure
```

## Experimental Variables

- localization confidence
- telemetry delay
- packet loss
- obstacle density
- route risk
- route resource cost
- link quality
- formation fault level

## Evaluation Outputs

- detection labels and confidence
- planning route decisions
- swarm coordination outcomes
- latency distributions
- safety fallback behavior
- benchmark CPU and memory deltas

## Future Hardware Integration Roadmap

- integrate exported model adapters with onboard inference runtimes
- validate telemetry perception against real bench sensor captures
- evaluate planning and swarm coordination against HIL traces
- confirm safety-gated AI outputs under hardware-in-the-loop timing

