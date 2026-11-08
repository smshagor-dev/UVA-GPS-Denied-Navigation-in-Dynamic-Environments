# 50-seed research validation: communication faults and local safety

**Scope:** software-only exploratory UAV task coordination. **Not** native multi-UAV
flight, RF/HIL, safety certification, or evidence of a new IEEE-contribution algorithm.

## Reproducibility

Run from the repository root:

```bash
python3 -m unittest tests.test_swarm_fault_study tests.test_swarm_advanced_study -v
python3 research/experiments/swarm_fault_study.py --counts 4,8,12 --seeds 50 --out results/swarm_fault_study
python3 research/experiments/swarm_stats.py --runs results/swarm_fault_study/runs.csv --out results/swarm_fault_study/stats.json
```

The dedicated GitHub Actions workflow also archives raw `runs.csv`, `summary.json`,
`stats.json` and the generated `REPORT.md`. The first successful 50-seed
run was captured in workflow 42510575335, artifact 13629875095. It generated
**3,000 runs**, computed as 3 swarm sizes x 4 conditions x 5 policies x 50 seeds.

## What was changed

- Replaced linear rescanning of delayed packets with a time-ordered delivery heap.
- Added a message-derived peer pose cache with timestamp monotonicity, never
  using future or globally true peer positions for the new local-safety arm.
- Added five policy arms: fixed role, unfiltered adaptive, freshness-aware
  adaptive, full-position safety oracle, and conservative local-message safety.
- Added 50-seed bootstrap effect-size intervals and sign-flip Monte Carlo for
  larger samples.
- Added a separate exact two-sided binomial paired **sign test**, excluding ties,
  with Holm correction over the 216 tested policy/metric/size/fault combinations.

The exact sign-test null is the equal probability of positive vs negative
nonzero paired effects, **not** zero mean effect; it complements rather than
replaces the bootstrap mean-difference intervals.

## Results: combined fault (per row 50 seeded simulations)

| UAVs | Policy | Completion mean | Failed-task recovery | Swept <1m pair-ticks mean |
|---:|---|---:|---:|---:|
| 4 | fixed role | 0.8750 | 0/50 | 2.04 |
| 4 | adaptive, no freshness | 0.9975 | 49/50 | 63.60 |
| 4 | adaptive freshness | 1.0000 | 50/50 | 84.18 |
| 4 | full-state safety oracle | 0.9975 | 50/50 | 0.00 |
| 4 | local stale-peer hold | 0.1850 | 0/50 | 32.36 |
| 8 | fixed role | 0.9375 | 0/50 | 11.76 |
| 8 | adaptive, no freshness | 0.99125 | 50/50 | 154.90 |
| 8 | adaptive freshness | 0.99375 | 50/50 | 150.72 |
| 8 | full-state safety oracle | 0.9925 | 50/50 | 0.00 |
| 8 | local stale-peer hold | 0.0650 | 2/50 | 108.16 |
| 12 | fixed role | 0.958333 | 0/50 | 15.62 |
| 12 | adaptive, no freshness | 0.981667 | 48/50 | 178.34 |
| 12 | adaptive freshness | 0.993333 | 50/50 | 268.60 |
| 12 | full-state safety oracle | 0.991667 | 50/50 | 0.00 |
| 12 | local stale-peer hold | 0.0475 | 1/50 | 179.84 |

The local-information conservative arm does *not* provide a safety guarantee.
Holding one's own trajectory cannot prevent another peer following stale
information from entering the separation zone. It also prevents mission
completion under sustained missing/stale messages.

Across the combined-fault task-completion comparisons (local hold minus
freshness-only), the 50-seed paired mean effects are respectively **-0.815**,
**-0.92875** and **-0.945833** for 4, 8 and 12 UAVs. In each group all 50
differences are negative. Exact sign p = 1.77636e-15 and Holm-adjusted
sign p = 3.83693e-13 for each group across the 216-comparison family.
These are conditional on the synthetic scenarios, not real flight or a
published IEEE SOTA method.

## Native C++ synthetic controller check (separate validation)

A successful GCC CTest run archived its real C++ output:

| Measure | No peer avoidance | Peer avoidance |
|---|---:|---:|
| Formation RMSE (m) | 0.525370 | 0.637371 |
| Minimum swept separation (m) | 3.132092 | 3.818625 |
| Swept pair-ticks closer than 1m | 0 | 0 |

Only one scripted trajectory was used. Increased distance came at the cost
of increased formation error. There is no real-aircraft interpretation.

## Scientific limitations and next experiments

1. The conservative local algorithm is a deliberately strict baseline, **not**
   a novel practical controller. The failure mode exposes an availability gap,
   but does not itself establish research novelty.
2. Validate local proximity sensing, contact graph changes, and cooperative
   fault membership *without* a ground-truth position oracle, then rerun the
   same 50-seed conditions and observe whether completion improves without
   unacceptable swept proximity risk.
3. Faithfully reproduce an external published algorithm on a **matched**
   task model. The existing CBAA-style static assignment experiment is not a
   head-to-head dynamic mission comparison.
4. Use native multi-node C++/communications replay, HIL and instrumented
   hardware trials if making practical safety or real-time claims.
5. Explicitly cite Choi et al. (2009, IEEE T-RO,
   DOI 10.1109/TRO.2009.2022423), and evaluate novelty against 2025 IEEE TAES
   task-allocation and 2026 IEEE TAES decentralized verification papers.

The paper is **not IEEE TAES submission-ready** as of this report.
