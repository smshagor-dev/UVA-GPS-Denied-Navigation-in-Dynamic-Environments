"""CBAA-style single-task auction baseline and exact assignment oracle.

Based on the two-stage auction/max-consensus idea from Choi et al., IEEE
Transactions on Robotics 25(4), 2009, DOI: 10.1109/TRO.2009.2022423.
Independent educational reconstruction, NOT author code, CBBA, or an exact
replication of the paper's numerical experiments. Software-only/static tasks.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import random
import statistics
from pathlib import Path


def scores_for(n, seed):
    rng = random.Random(10007 * seed + n)
    agents = [(float(i % 4) * 10.0, float(i // 4) * 10.0) for i in range(n)]
    tasks = [(rng.uniform(-4, 36), rng.uniform(-4, 30)) for _ in range(n)]
    return [[100.0 / (1.0 + math.dist(a, task)) for task in tasks] for a in agents]


def exact_oracle(scores):
    """Exact one-to-one optimum for n <= 12 via bitmask DP."""
    n = len(scores)
    if not 1 <= n <= 12 or any(len(row) != n for row in scores):
        raise ValueError("need a square 1..12 cost matrix")
    best = {0: (0.0, ())}
    for i in range(n):
        updated = {}
        for mask, (reward, assigned) in best.items():
            for j in range(n):
                if not mask & (1 << j):
                    next_mask = mask | (1 << j)
                    candidate = (reward + scores[i][j], assigned + (j,))
                    if next_mask not in updated or candidate[0] > updated[next_mask][0]:
                        updated[next_mask] = candidate
        best = updated
    return best[(1 << n) - 1]


def graph_for(n, topology):
    graph = {i: set() for i in range(n)}
    if topology == "complete":
        edges = list(itertools.combinations(range(n), 2))
    elif topology == "ring":
        edges = [(i, (i + 1) % n) for i in range(n)]
    elif topology == "partition":
        edges = [(a, b) for half in (range(n // 2), range(n // 2, n))
                 for a, b in itertools.combinations(half, 2)]
    else:
        raise ValueError(topology)
    for a, b in edges:
        graph[a].add(b)
        graph[b].add(a)
    return graph


def better(candidate, incumbent):
    if candidate[0] > incumbent[0] + 1e-10:
        return True
    return abs(candidate[0] - incumbent[0]) <= 1e-10 and (
        candidate[1] >= 0 and (incumbent[1] < 0 or candidate[1] < incumbent[1])
    )


def auction(scores, topology, loss, seed):
    """One-task bidding followed by neighbor max-consensus and release on loss."""
    n = len(scores)
    graph = graph_for(n, topology)
    rng = random.Random(8009 * seed + n)
    beliefs = [[(0.0, -1) for _ in range(n)] for _ in range(n)]
    assignments = [None] * n
    sent = delivered = tx_bytes = 0
    rounds = 0
    for _ in range(12 * n):
        rounds += 1
        previous = [(tuple(row), assignments[i]) for i, row in enumerate(beliefs)]
        for i in range(n):
            if assignments[i] is not None:
                continue
            valid = [j for j in range(n) if better((scores[i][j], i), beliefs[i][j])]
            if valid:
                task = max(valid, key=lambda j: (scores[i][j], -j))
                assignments[i] = task
                beliefs[i][task] = (scores[i][task], i)
        outgoing = [list(row) for row in beliefs]
        for sender in range(n):
            packet_bytes = len(json.dumps(outgoing[sender], separators=(",", ":")).encode())
            for receiver in sorted(graph[sender]):
                sent += 1
                tx_bytes += packet_bytes
                if rng.random() < loss:
                    continue
                delivered += 1
                for j in range(n):
                    if better(outgoing[sender][j], beliefs[receiver][j]):
                        beliefs[receiver][j] = outgoing[sender][j]
        for i in range(n):
            if assignments[i] is not None and beliefs[i][assignments[i]][1] != i:
                assignments[i] = None
        if previous == [(tuple(row), assignments[i]) for i, row in enumerate(beliefs)]:
            break
    return assignments, {"rounds": rounds, "sent_packets": sent,
                         "delivered_packets": delivered, "logical_unicast_tx_bytes": tx_bytes}


def outcome(scores, assignments):
    picked = [task for task in assignments if task is not None]
    best_per_task = {}
    for i, task in enumerate(assignments):
        if task is not None:
            best_per_task[task] = max(best_per_task.get(task, 0.0), scores[i][task])
    return {"unique_tasks": len(set(picked)),
            "duplicate_claims": len(picked) - len(set(picked)),
            "agents_assigned": len(picked),
            "conflict_adjusted_reward": round(sum(best_per_task.values()), 6)}


def trial(n, topology, loss, seed):
    scores = scores_for(n, seed)
    oracle_value, optimal = exact_oracle(scores)
    greedy = [max(range(n), key=lambda j: (scores[i][j], -j)) for i in range(n)]
    local_auction, network = auction(scores, topology, loss, seed)
    return {"uavs": n, "topology": topology, "loss": loss, "seed": seed,
            "greedy": outcome(scores, greedy),
            "cbaa_style": outcome(scores, local_auction),
            "oracle": outcome(scores, list(optimal)),
            "oracle_reward": round(oracle_value, 6),
            "network": network}


def run_matrix(seeds):
    configs = [(n, "ring", loss) for loss in (0.0, 0.3) for n in (4, 8, 12)]
    configs += [(8, "complete", 0.3), (8, "partition", 0.0)]
    records = [trial(n, graph, loss, seed)
               for n, graph, loss in configs for seed in range(seeds)]
    summaries = []
    for n, topology, loss in configs:
        rows = [row for row in records if (row["uavs"], row["topology"], row["loss"])
                == (n, topology, loss)]
        summaries.append({
            "uavs": n, "topology": topology, "loss": loss,
            "trials": len(rows),
            "mean_unique_tasks": statistics.mean(r["cbaa_style"]["unique_tasks"] for r in rows),
            "mean_duplicate_claims": statistics.mean(r["cbaa_style"]["duplicate_claims"] for r in rows),
            "mean_reward_fraction_oracle": statistics.mean(
                r["cbaa_style"]["conflict_adjusted_reward"] / r["oracle_reward"] for r in rows),
            "mean_logical_tx_bytes": statistics.mean(
                r["network"]["logical_unicast_tx_bytes"] for r in rows),
        })
    return records, summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--out", type=Path, default=Path("results/swarm_cbaa_reference"))
    args = parser.parse_args()
    if args.seeds <= 0:
        raise ValueError("number of seeds must be positive")
    runs, groups = run_matrix(args.seeds)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "runs.json").write_text(json.dumps(runs, indent=2) + "\n", encoding="utf-8")
    (args.out / "summary.json").write_text(json.dumps({
        "source": "10.1109/TRO.2009.2022423",
        "limits": "CBAA-style one-task approximation, not CBBA or author replication",
        "runs": len(runs), "groups": groups,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {len(runs)} software-only CBAA-style comparison records")


if __name__ == "__main__":
    main()
