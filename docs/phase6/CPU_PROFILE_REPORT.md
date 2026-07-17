# Phase 6 CPU Profile Report

Date: 2026-07-16

Primary artifacts:

- `docs/phase6/controlplane_cpu.pprof`
- `docs/phase6/controlplane_cpu_top.txt`
- `docs/phase6/callgrind_navigation.out`
- `docs/phase6/callgrind_navigation.txt`
- `docs/phase6/callgrind_ekf.out`
- `docs/phase6/callgrind_ekf.txt`
- `docs/phase6/perf_report.txt`
- `docs/phase6/soak_results.json`

## Tooling Used

- Go `pprof` for control-plane test CPU capture
- Valgrind `callgrind` for Linux native test binaries
- Windows live-process CPU counters during the `2 h` soak

Limitation:

- `perf` was unavailable in the WSL environment, so Linux `perf record` / `perf report` could not be collected on 2026-07-16
- `wpr.exe` exists on Windows, but `wpr -start CPU -filemode` failed with `0xc5585011`, so no ETW CPU trace artifact could be collected in this pass

## Profiling Scope

Captured profiles cover:

- control-plane startup / initialization via Go `pprof`
- EKF / sensor fusion math path via `callgrind`
- navigation / SLAM representative path via `callgrind`

Not yet captured in this environment:

- Linux `perf` sample profile
- flamegraph output
- Visual Studio Profiler capture
- Windows ETW CPU trace due profiling-policy failure

## Top 25 Hottest Functions

Self share is reported directly from the saved profiler artifact. Total share is only available from the Go `pprof` sample; the current `callgrind` exports are flat top-function views, so cumulative percentages are marked accordingly.

| Profile | Function | Self share | Total share | Notes |
| --- | --- | ---: | ---: | --- |
| Control plane | `runtime.cgocall` | 100.00% | 100.00% | very short `pprof` sample dominated by runtime / init |
| EKF | `Eigen::internal::gebp_kernel` | 50.16% | n/a | dominant dense matrix multiply kernel |
| EKF | `Eigen::internal::lhs_process_one_packet` | 11.03% | n/a | packetized matrix inner-loop work |
| EKF | `Eigen::internal::gemm_pack_lhs` | 6.41% | n/a | matrix packing for multiply path |
| EKF | `_dl_lookup_symbol_x` | 4.69% | n/a | dynamic-loader cost in test binary startup |
| EKF | `_dl_lookup_symbol_x` helper | 3.29% | n/a | loader symbol lookup overhead |
| EKF | `Eigen::internal::gemm_pack_rhs` | 2.75% | n/a | RHS pack path |
| EKF | `Eigen::internal::gemm_pack_rhs` alt | 2.70% | n/a | alternate RHS pack path |
| EKF | `drone::vio::EKFEstimator::compute_F(...)` | 2.52% | n/a | state-transition matrix assembly |
| EKF | `Eigen::internal::call_dense_assignment_loop` sum | 2.47% | n/a | covariance update assignment work |
| EKF | `Eigen::internal::call_dense_assignment_loop` product | 2.05% | n/a | matrix product assignment work |
| EKF | `Eigen::internal::call_dense_assignment_loop` scaled sum | 1.84% | n/a | scaled covariance combination |
| EKF | `_dl_lookup_symbol_x` | 1.13% | n/a | loader overhead |
| Navigation | `openblas` internal kernel `0x...a0e450` | 44.13% | n/a | BLAS-heavy math dominates this path |
| Navigation | `do_lookup_x` | 31.36% | n/a | dynamic-loader overhead in test startup |
| Navigation | `strcmp` | 2.57% | n/a | loader / symbol matching path |
| Navigation | `_dl_lookup_symbol_x` | 1.57% | n/a | loader symbol lookup |
| Navigation | `_int_malloc` | 1.40% | n/a | allocation pressure during startup / init |
| Navigation | `_dl_relocate_object_no_relro` | 1.35% | n/a | relocation overhead |
| Navigation | `gdcm::Dict::LoadDefault()` | 1.06% | n/a | imaging dependency initialization |
| Navigation | `_dl_lookup_symbol_x` | 0.95% | n/a | loader symbol lookup |
| Navigation | `libgdcmDICT` internal `0x13c2a0` | 0.92% | n/a | dictionary initialization work |
| Navigation | `_int_free_merge_chunk` | 0.89% | n/a | allocator merge overhead |
| Navigation | `gdcm::PrivateDict::LoadDefault()` | 0.86% | n/a | private dictionary load |
| Navigation | `gdcm::DictEntry::CheckKeywordAgainstName(...)` | 0.80% | n/a | gdcm dictionary keyword checks |

## Call Graph Evidence

### Go `pprof`

Source: `docs/phase6/controlplane_cpu_top.txt`

Observed chain:

`TestTelemetryIngestAndFleetSnapshotRoundTrip -> newTestServer -> NewServer -> NewFleetState -> runtime.cgocall`

Interpretation:

- the sample window was only `24.23 ms`
- this capture is real, but too short to represent steady-state backend CPU behavior

### EKF callgraph excerpt

Source: `callgrind_annotate --tree=calling --threshold=5 docs/phase6/callgrind_ekf.out`

Observed relationship:

- `Eigen::internal::gebp_kernel` receives `11.03%` worth of repeated packetized calls from `Eigen::internal::lhs_process_one_packet`

Interpretation:

- the hot path is dominated by Eigen GEMM internals rather than control logic
- optimization opportunities should target matrix math structure before micro-tuning surrounding code

## Top CPU Consumers By Area

### Control plane startup / test path

Source: `docs/phase6/controlplane_cpu_top.txt`

| Function | CPU % | Reason | Possible optimization |
| --- | ---: | --- | --- |
| `runtime.cgocall` | 100% of sampled 20 ms | very short sample dominated by Windows runtime / init path | collect a longer profile under steady telemetry load for clearer attribution |

### EKF / sensor fusion microbenchmark

Source: `docs/phase6/callgrind_ekf.txt`

| Function | CPU % | Reason | Possible optimization |
| --- | ---: | --- | --- |
| Eigen `gebp_kernel` | 50.16% | dense matrix multiply in EKF propagation/update math | reduce temporary matrix work and review fixed-size multiplication hotspots |
| Eigen `lhs_process_one_packet` | 11.03% | packetized matrix kernel work | same as above; prioritize math kernel locality |
| `drone::vio::EKFEstimator::compute_F(...)` | 2.52% | state-transition matrix assembly | precompute invariant terms where possible |
| `drone::vio::EKFEstimator::propagate_imu(...)` | 0.58% | IMU propagation path | avoid redundant allocations and repeated transforms |
| `drone::vio::EKFEstimator::compute_G(...)` | 0.45% | process-noise matrix assembly | cache reusable subexpressions |

### Navigation / VIO pipeline representative path

Source: `docs/phase6/callgrind_navigation.txt`

| Function | CPU % | Reason | Possible optimization |
| --- | ---: | --- | --- |
| `openblas` internal kernel | 44.13% | linear algebra dominates this path | review algorithmic batch sizes and library initialization strategy |
| dynamic linker `do_lookup_x` | 31.36% | repeated dependency / loader overhead in test-binary startup | switch to in-process benchmarking for cleaner steady-state attribution |
| `gdcm::Dict::LoadDefault()` | 1.06% | imaging dependency initialization | avoid repeated image-stack initialization in benchmark setup |

## Soak CPU Stability

From `docs/phase6/soak_results.json`:

- CPU seconds: `0.781` -> `1.125` over `2 h`
- no sustained CPU growth trend was observed
- backend remained responsive with zero queue buildup

## Verdict

CPU status: `PASS`

Strongest-available evidence note:

- no stronger profiler artifact could be produced on 2026-07-16 beyond the saved `pprof` and `callgrind` captures
- see `docs/phase6/perf_report.txt`
