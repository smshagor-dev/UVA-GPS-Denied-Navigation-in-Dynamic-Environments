# IEEE TAES swarm intelligence: evidence and reviewer-readiness audit

**Date:** 2026-11-08  
**Target:** original work on communication/freshness-aware UAV swarm coordination.  
**Status:** software research evidence improved; **not yet submission-ready**.

## Literature anchors (verified bibliographic records)

1. H.-L. Choi, L. Brunet, J. P. How, "Consensus-Based Decentralized
   Auctions for Robust Task Allocation", *IEEE Transactions on Robotics*,
   vol. 25, no. 4, pp. 912–926, 2009.
   DOI: https://doi.org/10.1109/TRO.2009.2022423.
   Original method includes CBAA (single assignment) and CBBA (bundles).
2. "Communication-Aware Task Allocation for the UAV Swarm via
   Multiobjective Asynchronous Policy Learning", *IEEE Transactions on
   Aerospace and Electronic Systems*, published online 28 October 2025.
   DOI: https://doi.org/10.1109/TAES.2025.3626306.
3. "Communication-Efficient Asynchronous Consensus With Global
   Harmonization for Large-Scale UAV Swarms", *IEEE Transactions on
   Aerospace and Electronic Systems*, published online 7 November 2025.
   DOI: https://doi.org/10.1109/TAES.2025.3630567.

The prior literature already studies distributed auctions, communication-aware
allocation and asynchronous consensus. Therefore **none of these concepts
alone is an original novelty claim**.

## Work implemented in this draft research branch

| Research operation | Implementation | Evidence type | Status |
|---|---|---|---|
| Fixed-role vs adaptive policy controls | swarm_fault_study.py | Paired simulated experiments, 360 trials | Implemented |
| Peer-message completed-task propagation | swarm_fault_study.py | No global task completion oracle, unit test | Implemented |
| Event-derived mission completion and fault recovery | swarm_fault_study.py | Simulated task-service events | Implemented |
| Logical packet overhead/proximity violations | swarm_fault_study.py | Simulated events | Implemented |
| CBAA-style one-task reference | cbaa_reference_study.py | 160 static allocation trials | Implemented |
| Centralized exact one-to-one oracle | cbaa_reference_study.py | Bitmask dynamic-programming optimum | Implemented |
| Paired uncertainty and multiple testing | swarm_stats.py | Bootstrap CI, sign-flip, Holm FWER correction | Implemented |
| Native C++ formation controller replay | swarm_native_formation_metrics.cpp | Synthesized trajectory, native controller | CI verification required |
| Formation error and min/swept separation | swarm_native_formation_metrics.cpp | Real C++ controller output over synthetic time steps | CI verification required |
| Real multi-node radio and hardware flight | None | No hardware or flight trace | Not complete |
| Exact replication of current IEEE methods | None | No author-equivalent setup/source | Not complete |
| Establishment of a distinct new algorithm | None | No controlled novelty evidence | Not complete |

## Controlled model v2: combined node failure and packet degradation

All measurements below are **software simulation** values. Each configuration
has ten paired seeds. The reported recovery median excludes censored trials.
The static policy has no reallocation by design.

| UAV count | Policy | Mean task-completion rate | Interrupted task recovered | Recovery median, simulated seconds | Mean sub-1-m pair-ticks |
|---:|---|---:|---:|---:|---:|
| 4 | Static roles | 0.8750 | 0/10 | censored | 1.9 |
| 4 | Adaptive unfiltered | 1.0000 | 10/10 | 30.50 | 39.8 |
| 4 | Adaptive freshness | 1.0000 | 10/10 | 9.75 | 106.7 |
| 8 | Static roles | 0.9375 | 0/10 | censored | 7.6 |
| 8 | Adaptive unfiltered | 0.99375 | 10/10 | 11.25 | 110.9 |
| 8 | Adaptive freshness | 0.99375 | 10/10 | 13.50 | 147.0 |
| 12 | Static roles | 0.958333 | 0/10 | censored | 11.6 |
| 12 | Adaptive unfiltered | 0.983333 | 9/10 | 16.50 | 182.9 |
| 12 | Adaptive freshness | 0.97500 | 10/10 | 9.75 | 184.1 |

Under four UAVs with combined failure, 66.9 additional sampled pair-ticks
occur in the freshness-aware policy versus unfiltered, with paired 95% bootstrap
CI [15.8, 118.4]. The exact unadjusted sign-flip p is 0.048828125; after
Holm family-wise correction across 60 reported comparisons, **p_adj = 1.0**.
No statistically significant superiority is established.

## CBAA-style study: 160 runs, static allocation only

The reference reconstructs one-task auction bidding and neighbor max-consensus.
Twenty seeds were used for each of eight network/size configurations. It does
not implement the authors' complete CBBA code or the 2025 TAES algorithms.
These static one-task assignments **must not** be directly compared as if they
were the same mission model as the 360 dynamic task-completion trials.

## Native C++ study: evidence boundaries

Native replay calls the existing project's LeaderFollowerController rather than
a reimplementation in Python. It compares avoidance enabled/disabled under a
synthetic trajectory, and records formation RMSE, endpoint separations and
swept minimum distance across each discrete interval. The result cannot prove
collision freedom, physical UAV safety, mesh performance, flight feasibility
or an improvement over an external published method. Inspect the uploaded CI
artifact before quoting any native numbers.

## Mandatory before a defensible IEEE original paper

1. Establish a formally stated **new method** and exactly what mathematical
   property differs from the 2009 CBAA/CBBA and 2025 TAES approaches.
2. Reproduce at least one contemporary published approach with matched task
   rewards, communication models, UAV count, motion constraints and metric
   definitions. Source provided by MIT Aerospace Controls Laboratory
   (https://acl.mit.edu/projects/consensus-based-bundle-algorithm) may assist,
   but its MATLAB CBBA requires a compatible execution environment.
3. Compare simulated outcomes across larger paired sample sizes, more
   independent stress distributions, and realistic closed-loop motion plus
   network dynamics, with uncertainty and multiplicity control.
4. Validate native multi-node link faults and physical safety separately using
   HIL/radio traces or appropriate multi-robot hardware.
5. Submit only claims supported by primary recorded evidence; cite the published
   work, disclose overlapping prior manuscripts and distinguish simulation
   from actual experiments.

### Claims explicitly disallowed

- 100% accurate or 100% safe, collision-free, or hardware-proven.
- Replicated CBBA or a 2025 TAES paper, when only the CBAA-style reconstruction exists.
- Statistically significant superiority from the currently corrected p-values.
- Native flight validation from a deterministic controller replay.

