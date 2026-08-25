# Contributing

## Scope

This repository is open for software, documentation, validation, and research-collaboration contributions. It is not a venue for overstating hardware or flight validation.

## Ground Rules

- preserve existing evidence under `docs/phase*/`
- do not delete validated artifacts unless they are replaced with newer evidence
- do not claim flight, hardware, PX4, Gazebo, or SITL validation without real generated evidence
- prefer reproducible scripts and machine-readable outputs over narrative-only changes

## Development Workflow

1. Create a focused branch.
2. Make the smallest change that closes the evidence or implementation gap.
3. Run the relevant validation commands.
4. Update documentation and generated artifacts together.
5. Summarize limitations honestly in the pull request or change description.

## Recommended Validation Commands

```powershell
python scripts\validate_config_schemas.py
python -m unittest tests.test_dashboard_backend_status
ctest --test-dir build\validation-msvc -C Release --output-on-failure
go test ./...
```

Phase-specific work should also rerun the affected scenario or benchmark scripts.

## Research Contributions

Welcome contribution areas:

- new reproducible experiments
- evaluation adapters
- documentation improvements
- benchmark dataset schemas
- visualization and observability tooling
- autonomy research scaffolding with evidence

## Community Expectations

Please follow [`CODE_OF_CONDUCT.md`](./CODE_OF_CONDUCT.md) and retain attribution, license terms, and safety notices.
