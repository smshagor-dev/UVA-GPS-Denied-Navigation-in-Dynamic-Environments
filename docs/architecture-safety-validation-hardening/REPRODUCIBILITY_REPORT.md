# Phase 4 Reproducibility Report

Date: 2026-07-16

## Commands Executed

### Validation

```powershell
python scripts/local_validate.py --report docs/architecture-safety-validation-hardening/local_validate_report.json
```

Observed result on 2026-07-16:

- Python syntax: PASS
- Python unit tests: `12/12` PASS
- Go tests: PASS
- Native CTest: `112/112` PASS

### Configuration audit

```powershell
python scripts/phase4_config_audit.py
```

Observed result on 2026-07-16:

- `10/10` config files passed

### Benchmark suite

```powershell
python scripts/phase4_benchmark_suite.py
```

Observed result on 2026-07-16:

- Benchmark JSON written
- Simulation telemetry smoke: PASS
- Production unavailable-source smoke: PASS

### Install check

```powershell
cmake --install build\validation-msvc --config Release --prefix build\phase4-install-check
```

Observed result on 2026-07-16:

- Install tree created under `build/phase4-install-check`
- Included `drone_node.exe`, `sensor_fusion_core.lib`, headers, docs, example configs, and exported CMake package files

### Architecture graph

```powershell
cmake --graphviz=docs\architecture-safety-validation-hardening\cmake_graph.dot --preset validation-msvc
```

Observed result on 2026-07-16:

- Graphviz target graph written successfully

### Linux memory-tool check

```powershell
wsl.exe bash -lc "cmake --version && valgrind --version"
```

Observed result on 2026-07-16:

- `cmake version 4.2.3`
- `valgrind: command not found`

## Reproducibility Outcome

Windows software reproducibility: PASS

Linux sanitizer/Valgrind reproducibility: BLOCKED

Reason:

- WSL had `cmake`, but `valgrind` was not installed, so the requested Linux memory-audit path could not be completed.


