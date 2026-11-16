# GPS-denied UAV research evidence baseline — 2026-11-16

## Immutable starting point

- Reviewed `main`: `d460ca640e2a4bb9b0f6051b4c02823478e0d087`.
- Swarm research experiment: draft PR #73, head `18c37a81b14489b1cfbfbad01f70e4ab456ee74f`. The research code and 3,600-run reports **are not in main**.
- This PR fixes a CI instrumented-linkage defect only. It does **not** change experiments, policies, numerical evidence, or manuscript conclusions.

## Source-to-claim evidence inventory

| Research area | Available primary implementation/evidence | Valid claim today | Missing evidence for original research |
| --- | --- | --- | --- |
| Fault-tolerant swarm task recovery | PR #73 `research/experiments/swarm_fault_study.py`, `swarm_stats.py`, `tests/swarm_native_formation_metrics.cpp`; completed 50-seed research workflow | Controlled *software-model* outcomes in 3 UAV sizes × 4 fault scenarios × 6 policies × 50 paired seeds (3,600 runs); one synthetic native C++ formation replay | New distinguishable method; matched modern published baselines; noisy/occluded local sensing; multi-node native tests; independent sensor/flight evidence for any operational claims |
| Local proximity separation | PR #73 `SWARM_LOCAL_PROXIMITY_BOUND_AND_RESULTS.md` | A basic conditional one-step separation bound for **perfect**, synchronous local observations and bounded linear steps; zero swept sub-1m violations in the reported model trials | Sensor false negatives, occlusion, latency, uncertainty, realistic dynamics and independent comparisons. Neither a new theorem nor a physical safety guarantee |
| ESKF/FEJ/MSCKF navigation | `src/vio/Phase17ESKFEstimator.cpp`; Phases 17–23 tests and replay reports | Shadow-estimator mathematics/invariants and synthetic replay behavior | Actual ground-truth trajectory evaluation (e.g. EuRoC), ATE/RPE/NEES and calibration, matched VIO baselines, end-to-end phase 22/23 closure |
| Peer replay and radio | `src/swarm/SwarmStateCache.cpp`, `scripts/peer_replay_scenario_study.py`, `tests/peer_replay_cache_cli.cpp`, RF evidence dossier | Software sensitivity/parity against constructed link conditions; receiver-only interpretation limits | Verified matching real sender TX and receiver RX inventories, physical FANET provenance, independent trials and radio characterization |
| UWB/TDOA fusion | `src/localization/TDOALocalizer.cpp`, `LocalizationFusion.cpp` | Executable solver and fusion interface, fail-closed metadata tests | Independent calibrated range/time datasets, clock/NLOS/multipath error, ground-truth error metrics and robust baselines |
| Firmware trust | Existing accepted/published manuscript and `RESEARCH_RELEASE.md` | Source-bound software policy/anti-rollback evaluation | **Out of scope for new swarm paper**; avoid duplicate contribution and numerical/result overlap |

## PR #73 evidence provenance and limitations

- Historical CI run: `42512323093` (Swarm Fault Study Evidence), success on `18c37a81`.
- Corresponding CI, Security and Navigation Regression Validation workflows completed successfully on the same head.
- PR #73 is **draft** and should not be described as a published-method replication or submission-ready.
- Compare arms using shared seeded workloads and **paired** confidence intervals. A central full-state oracle must never be described as a decentralized implementation.
- The CBAA-style reference is not a faithful full CBBA implementation. The 2025–2026 published methods are not replicated under matched dynamics.
- Original task coordination and proximity results represent the simplified simulator, not proven GPS-denied operation or physical flight. No independent real sensor or HIL study is claimed.

## Main nightly failure and the narrow fix

Nightly run `43700913229` (2026-11-15 on `d460ca64`) had:
- Clang ASan/UBSan **Build** failure from unresolved `__asan_*` / `__ubsan_*` references.
- GCC ASan/UBSan **Build** failure.
- GCC Coverage **Native coverage configure and build** failure from unresolved `__gcov_*` references.

The linked static `sensor_fusion_core` is instrumented in those presets, while the `peer_replay_cache_cli` executable lacked `drone_enable_sanitizers` and `drone_enable_coverage`. The change applies those existing target helpers to the CLI, enabling the required runtime linkage. This diagnosis is based on CI linker logs; **successful fixed-build validation must be observed in the new PR**, not assumed.

## Verification and next research gates

1. In the new PR, confirm Clang ASan/UBSan, GCC ASan/UBSan and native coverage **build and test** lanes pass. Do not suppress sanitizer errors, disable coverage, or omit replay CLI from build.
2. Confirm `peer_replay_native_parity` runs with production C++ sequence admission and compare results against software model.
3. Reproduce PR #73's 3,600 software trials from the exact branch head, retain full per-trial CSV, CI artifact ID, SHA-256, seed matrix, toolchain and a machine-readable summary.
4. Specify a *distinct* uncertainty-aware recovery method, its falsifiable hypothesis, baseline implementation fidelity, and predefined metrics before claiming novelty.
5. Obtain permitted real sensor logs (e.g. EuRoC), evaluate localization ATE/RPE/uncertainty calibration, then use **clearly labeled** empirically grounded error distributions for software swarm fault analysis. This does **not** turn the simulator into a real multi-UAV trial.
6. Perform matched baseline, ablation, sensitivity and paired statistical analysis, report negative results and confounders. Review any overlap with other UAV submissions before manuscript writing.

**Evidence gate:** this inventory is an audit of code and existing repository/CI records. It introduces no new scientific measurement.
