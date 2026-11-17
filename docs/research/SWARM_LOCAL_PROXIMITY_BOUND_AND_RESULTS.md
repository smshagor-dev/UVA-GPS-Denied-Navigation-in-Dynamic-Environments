# Local proximity reference: assumptions, conditional bound and 50-seed evidence

**Status:** idealized **software-only** reference. Not published-method replication,
physical LiDAR/camera validation, real RF/HIL, or certified collision avoidance.

## Conditional, synchronous one-step separation argument

Let the required minimum separation be `d = 1 m`, maximum speed `v = 2.5 m/s`,
and simulation step `h = 0.5 s`. Maximum displacement of each UAV during the
step is `s = vh = 1.25 m`. The perfect local sensor detects *all* live peers
within `R = 5 m`. Hence `R > d + 2s = 3.5 m`.

For every detected peer with initially observed location `p_j(0)`, a UAV
accepts its own linear one-step candidate trajectory `p_i(t)` only if

```text
min_{t in [0,h]} ||p_i(t) - p_j(0)|| >= d + s .
```

Otherwise it remains stationary. An unsensed peer begins farther than `R`.

Given pairwise separation of at least `d` **before** each step, exact and
synchronous positions, lossless proximity detection within `R`, no occlusion,
linear paths and each step bounded by `s`, the elementary triangle inequality
shows that no two simulated trajectories approach closer than `d` during
that step:

- When both move after mutual detection, their distances to the other's
  initial position remain at least `d + s`. Since the other moves at most
  `s`, actual separation is at least `d`.
- When one moves and one holds, the moving agent applies the same clearance
  against the stationary agent. If both hold, initial separation remains.
- When a pair is outside sensor range, relative displacement is at most
  `2s`, leaving separation greater than `R - 2s >= d`.

This is a **basic bounded-reachability argument, not an original theorem**.
It fails if detection has false negatives, latency, occlusion, unmodeled 3D
dynamics, localization uncertainty, inaccurate speed bounds or clocks.
No real-world safe-operation guarantee follows.

## Experimental implementation

`research/experiments/swarm_fault_study.py` contains six policies, including:

- `adaptive_freshness`: task decisions with stale peer claim filtering;
  no movement safety constraint;
- `adaptive_freshness_local_safety`: message-only cautious motion;
  missing/stale remote peers cause holding, often permanently during failure;
- `adaptive_freshness_safety`: *global true-state* sequential safety oracle,
  only a benchmark and not a decentralized method;
- `adaptive_freshness_proximity`: perfect synchronous **local** detections
  of peers within 5 m, using only sensed peer positions to check next-step
  swept clearance, while task selection still uses delayed messages.

The sensing observations are generated from simulation ground truth **only for
peers within range**. They are not measured or replayed camera/LiDAR readings.
No real sensor cost or occlusion model is included.

## Reproduced 50-seed results

Experimental matrix: 3 UAV sizes × 4 communication/failure conditions ×
6 policies × 50 paired seeds = **3,600 software-model runs**.

### Combined node failure and packet degradation

| UAVs | Freshness-only completion | Local message-hold completion | Local proximity completion | Proximity orphan-task recovery | Mean swept <1 m pair-ticks, proximity |
|---:|---:|---:|---:|---:|---:|
| 4 | 1.0000 | 0.1850 | 0.8675 | 41/50 | 0 |
| 8 | 0.99375 | 0.0650 | 0.77125 | 34/50 | 0 |
| 12 | 0.99333 | 0.0475 | 0.73667 | 26/50 | 0 |

The perfect-range reference is substantially more available than missing-peer
hold, but still has lower mission completion than unconstrained freshness-only
coordination. Under the exact idealized model, it recorded zero swept
separation violations in these simulations. This does **not** validate
practical safety.

### Paired effects against unconstrained freshness-only

| UAVs | Completion mean difference (proximity minus freshness) | 95% paired bootstrap CI | Holm-adjusted exact sign-test p |
|---:|---:|---|---:|
| 4 | -0.1325 | [-0.1875, -0.0875] | 1.91e-7 |
| 8 | -0.2225 | [-0.28375, -0.16375] | 5.87e-8 |
| 12 | -0.256667 | [-0.3100, -0.2100] | 9.06e-13 |

The exact paired sign test evaluates the direction probability among nonzero
paired differences; it is **not** a test of the mean difference. Holm
family-wise correction was applied across 360 comparisons. Sign-flip
Monte Carlo p-values have their own adjustment family and finite resolution;
do not substitute sign-test p-values for a mean-effect significance claim.

## Scientific boundaries

- The perfect local observation model is not a tested onboard LiDAR or vision
  pipeline. It cannot be called a hardware-tested safety algorithm.
- The one-step bound does not establish novel UAV swarm intelligence:
  velocity obstacles, reachable sets and decentralized collision checking
  have extensive literature, including **Phillips, Tran and Bradley**, IEEE
  Transactions on Aerospace and Electronic Systems, DOI
  `10.1109/TAES.2026.3683620`, with physical flight evidence.
- Mission decisions here are a synthetic greedy heuristic; no published
  2025 TAES allocation algorithm has been faithfully reimplemented on matched
  workloads.
- A future scientifically defensible method must treat detection errors,
  occlusion, delayed localization, and unknown peer motion, and then compare
  against faithful state-of-the-art methods.
- The result is **not submission-ready IEEE original research**, though it
  provides reproducible comparisons and identifies a concrete limitation.
