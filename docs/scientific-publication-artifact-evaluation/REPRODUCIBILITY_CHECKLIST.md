# Reproducibility Checklist

Date: July 17, 2026

## Archive And Source

- Official DOI identified: PASS
- DOI used consistently as `https://doi.org/10.5281/zenodo.20195953`: PASS
- Repository source tree present: PASS
- Zenodo archive treated as the official archived artifact: PASS

## Environment

- Python available: PASS
- Go available: PASS
- CMake available: PASS
- Docker available for deployment validation: PASS
- Windows validation build available: PASS

## Build And Test

- `python scripts\validate_config_schemas.py`: PASS
- `go test ./...`: PASS
- `ctest --test-dir build\validation-msvc -C Release --output-on-failure`: PASS

## Experiment Regeneration

- Phase 6 benchmark rerun: PASS
- Phase 7 mission scenarios rerun: PASS
- Phase 7 software HIL rerun: PASS
- Phase 7 simulation rerun: PASS
- Phase 7 failure injection rerun: PASS
- Phase 8 advanced scenarios rerun: PASS
- Phase 9 AI scripts rerun: PASS
- Phase 10 deployment validation rerun: PASS
- Phase 11 artifact scripts present and generated outputs: PASS

## Machine-Readable Outputs

- JSON artifacts generated and stored under `docs/`, `results/`, and `experiments/`: PASS
- Artifact hash list recorded in `ARTIFACT_EVALUATION.md`: PASS
- Configuration snapshots recorded in generated experiment artifacts: PASS

## Claim Boundaries

- No claim of physical flight validation: PASS
- No claim of PX4 validation: PASS
- No claim of Gazebo validation: PASS
- No claim of SITL validation: PASS
- No claim of hardware qualification: PASS
- No claim of peer-reviewed publication acceptance: PASS

## Verdict

Status: PASS
