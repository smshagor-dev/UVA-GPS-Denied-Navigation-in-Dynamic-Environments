# IEEE preparation: task-coordination baselines and measured simulation outcomes

## Status and scope

This is an exploratory, **software-only** controlled comparison in a new simplified
discrete-time task model. It is not a native C++ integration test; not a real swarm
flight or radio benchmark; not an exact reproduction of an IEEE algorithm; and
not evidence of original scientific superiority. The purpose is to establish
reproducible independent control policies and event-derived outcome metrics.

Run from the repository root:

    python3 -m unittest tests.test_swarm_fault_study -v
    python3 research/experiments/swarm_fault_study.py --counts 4,8,12 --seeds 10 --out results/swarm_fault_study

The workflow `.github/workflows/swarm-fault-study.yml` reruns the same matrix.
A complete run produces `runs.csv`, `summary.json`, and `REPORT.md`.
`summary.json` records the script SHA-256. Host-time decision timings vary by machine.

## Controlled policies

1. **static_roles:** immutable, equal-size queues of tasks allocated by initial UAV ID; no reassignment after a failure.
2. **adaptive_unfiltered:** local greedy resource/distance assignment with peer claims, retaining old claims indefinitely.
3. **adaptive_freshness:** the same adaptive rule, but ignores peer claims older than 2 seconds. This isolates one freshness ablation, not a new IEEE-equivalent method.

All policies receive identical task positions and seeded link impairment in each paired condition.
The revised model uses simulated peer packets to propagate completed task IDs;
each agent maintains locally known completions. Task coordinates are common
mission inputs. This still does not model RF routing or a complete decentralized
robotic runtime.

## Experimental matrix

- 4, 8, and 12 UAVs; tasks = 2 times UAV count.
- Nominal, packet loss, permanent node failure, combined failure and packet loss.
- 10 paired seeds per condition and policy: **360 trial runs** in total.
- Duration 75 simulated seconds; step 0.5 seconds.
- Failure of UAV 0 at simulated second 16; degraded network between 16–44 seconds.
- Degraded communication uses simulated 38% per-recipient message loss and 1–8 ticks of delivery delay.
- Each task requires 2 seconds of simulated service near its target.

## Selected locally executed observations (combined-fault condition)

Each row aggregates ten seeds. Values are outputs of the simplified model, not
physical performance measurements.

| UAVs | Policy | Task completion mean | Interrupted task recovered | Median recovery if recovered | Mean sampled separation violations |
|---:|---|---:|---:|---:|---:|
| 4 | static_roles | 0.8750 | 0/10 | N/A | 1.9 |
| 4 | adaptive_unfiltered | 1.0000 | 10/10 | 30.50 s | 39.8 |
| 4 | adaptive_freshness | 1.0000 | 10/10 | 9.75 s | 106.7 |
| 8 | static_roles | 0.9375 | 0/10 | N/A | 7.6 |
| 8 | adaptive_unfiltered | 0.9938 | 10/10 | 11.25 s | 110.9 |
| 8 | adaptive_freshness | 0.9938 | 10/10 | 13.50 s | 147.0 |
| 12 | static_roles | 0.9583 | 0/10 | N/A | 11.6 |
| 12 | adaptive_unfiltered | 0.9833 | 9/10 | 16.50 s | 182.9 |
| 12 | adaptive_freshness | 0.9750 | 10/10 | 9.75 s | 184.1 |

The static policy has no recovery mechanism by design. Adaptive policies regain
orphaned task completion but can substantially increase sampled pairwise
separation violations. Freshness filtering is **not** uniformly superior.

## Actual observations recorded per trial

- Completed task count and completion fraction from discrete task-service events.
- Elapsed simulation time until another UAV completes the failed UAV's interrupted task. Failed recovery is encoded as null and is treated as censored.
- Redundant worker-ticks at shared task targets.
- Live-agent pair/step samples with inter-agent separation <1 m. This is **not** a measured collision or a safety guarantee.
- Number of simulated logical unicast packets, delivered packets, and serialized bytes.
- Total simulated travel distance.
- Host wall-clock decision timing p95, separated from simulated elapsed time.

No measured formation RMSE is claimed **by this task model**. A separate C++
replay measures formation RMSE using the native LeaderFollowerController and
compares peer avoidance on/off under a synthetic trajectory.

## Evidence limits and reviewer risks

- The model is intentionally simplified; no physical UAV, GNSS-denied localization,
  PX4/Gazebo, RF propagation, live sensor fusion,
  or physical flight-controller integration is exercised by this task model.
- The absence of a separation controller makes observed safety trade-offs important,
  not an indication of acceptable physical collision risk.
- The baselines are controls for one toy model, **not** competitive state-of-the-art
  research implementations. IEEE publication would additionally require published
  baseline replication, rigorous integrated evaluation, larger paired sample sizes,
  uncertainty intervals, and clear algorithmic novelty.
- The previous Phase 9/Phase 11 scalar recovery-time formulas must **not** be cited
  as empirical recovery measurements. Only event-derived simulated recovery is
  presented here.
- Neither no-freshness nor freshness-aware policy is claimed to be globally optimal.

This experiment therefore closes the engineering tasks of having executable
controls and measured software-model outcomes. It does **not** close the larger
scientific validation gap required for an IEEE original research manuscript.

## Supplementary evidence and strict scope

- CBAA-style one-task max-consensus reference with exact assignment oracle: research/experiments/cbaa_reference_study.py. Based on Choi et al., IEEE T-RO (2009), DOI 10.1109/TRO.2009.2022423. This is **not** CBBA, the authors' code, or the original paper's experiments.
- Paired analysis: research/experiments/swarm_stats.py, with percentile bootstrap CIs and exact sign-flip tests. Unadjusted exploratory tests do not justify unqualified significance claims.
- Native formation replay: tests/swarm_native_formation_metrics.cpp, actual C++ LeaderFollowerController, synthetic trajectories, and on/off peer avoidance. Output RMSE and proximity figures must be confirmed from CI before publication.
- **The peer-completion v2 experiment supersedes all v1 results.** Do not combine or quote the old numbers.

### Remaining scientific validation

1. Validate a faithful published baseline against a current IEEE TAES method with matched task/network assumptions, or use its official author-provided benchmark.
2. Independently validate native multi-node execution, hardware-in-the-loop/radio timing, uncertainty propagation, and real safety behavior.
3. Define and verify a new joint algorithm and run independent repeated trials with multiplicity control before asserting superiority.

These are not completed merely by green CI checks.
