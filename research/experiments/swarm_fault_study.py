"""Controlled, software-only UAV task-coordination policy comparison.

This is a *new simplified discrete-time model*, not a measurement of the
native C++ flight stack, physical radios, or a published algorithm. All policies
share identical randomized scenarios within each paired trial.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import json
import math
import random
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

POLICIES = ("static_roles", "adaptive_unfiltered", "adaptive_freshness", "adaptive_freshness_safety", "adaptive_freshness_local_safety")
SCENARIOS = ("nominal", "packet_loss", "node_failure", "combined")
DT_S = 0.5
DURATION_S = 75.0
FAULT_START_S = 16.0
FAULT_END_S = 44.0
PEER_TTL_S = 2.0
SPEED_M_S = 2.5
SERVICE_S = 2.0
SAFETY_DISTANCE_M = 1.0


@dataclass
class Agent:
    identifier: int
    x: float
    y: float
    battery: float
    task_id: int | None = None
    alive: bool = True
    cache: dict[int, tuple[float, int | None]] = field(default_factory=dict)
    distance_m: float = 0.0
    known_done: set[int] = field(default_factory=set)
    # Last authenticated-in-simulation peer pose; this is not real sensor data.
    peer_poses: dict[int, tuple[float, float, float, float, float]] = field(default_factory=dict)
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class Task:
    identifier: int
    x: float
    y: float
    service_s: float = 0.0
    done_at_s: float | None = None
    done_by: int | None = None


def make_agents(count: int) -> list[Agent]:
    return [Agent(i, float(i % 3) * 3.0, float(i // 3) * 4.0,
                  90.0 - float(i % 5) * 3.0) for i in range(count)]


def make_tasks(count: int, rng: random.Random) -> list[Task]:
    # Fixed spatial distribution per paired scenario; no policy-specific randomness.
    return [Task(i, 12.0 + (i % 5) * 16.0 + rng.uniform(-3.0, 3.0),
                 8.0 + (i // 5) * 16.0 + rng.uniform(-3.0, 3.0))
            for i in range(count * 2)]


def select_task(agent: Agent, agents: list[Agent], tasks: list[Task],
                policy: str, now_s: float) -> int | None:
    if not agent.alive:
        return None
    if policy == "static_roles":
        # Unchanging equal-size task queues: no replacement on node failure.
        assigned = [task for task in tasks
                    if task.identifier % len(agents) == agent.identifier
                    and task.identifier not in agent.known_done]
        return assigned[0].identifier if assigned else None

    pending = [task for task in tasks if task.identifier not in agent.known_done]
    if not pending:
        return None
    claims = set()
    for peer_id, (observed_s, claimed_task) in agent.cache.items():
        if peer_id == agent.identifier or claimed_task is None:
            continue
        if policy in {"adaptive_freshness", "adaptive_freshness_safety", "adaptive_freshness_local_safety"} and now_s - observed_s > PEER_TTL_S:
            continue
        claims.add(claimed_task)
    def cost(task: Task) -> tuple[float, int]:
        travel = math.hypot(task.x - agent.x, task.y - agent.y) / SPEED_M_S
        # Same resource-aware penalty and claim coordination for both adaptive policies.
        energy_penalty = max(0.0, 65.0 - agent.battery) * 0.2
        duplicate_penalty = 28.0 if task.identifier in claims else 0.0
        switching_penalty = 1.5 if agent.task_id is not None and agent.task_id != task.identifier else 0.0
        return (travel + energy_penalty + duplicate_penalty + switching_penalty,
                task.identifier)
    return min(pending, key=cost).identifier


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(len(ordered) * pct) - 1)
    return round(ordered[index], 6)


def swept_distance(a0: tuple[float, float], a1: tuple[float, float],
                   b0: tuple[float, float], b1: tuple[float, float]) -> float:
    """Minimum Euclidean gap between simultaneous linear trajectories."""
    rx, ry = a0[0] - b0[0], a0[1] - b0[1]
    vx = (a1[0] - a0[0]) - (b1[0] - b0[0])
    vy = (a1[1] - a0[1]) - (b1[1] - b0[1])
    denom = vx * vx + vy * vy
    alpha = max(0.0, min(1.0, -(rx * vx + ry * vy) / denom)) if denom > 1e-14 else 0.0
    return math.hypot(rx + alpha * vx, ry + alpha * vy)


def safety_filter(agents: list[Agent], proposals: dict[int, tuple[float, float]]) -> tuple[dict[int, tuple[float, float]], int]:
    """Idealized full-position reservation: smaller agent IDs have earlier priority.

    This uses the TRUE current location of ALL active agents (a safety oracle).
    It is an optimistic centralized reference, NOT decentralized sensing,
    RF/HIL collision avoidance, or an onboard flight-safety guarantee.
    """
    active = [a for a in agents if a.alive]
    accepted: dict[int, tuple[float, float]] = {}
    rejected = 0
    for agent in active:
        a0 = (agent.x, agent.y)
        candidate = proposals.get(agent.identifier, a0)
        if candidate != a0:
            for peer in active:
                if peer.identifier == agent.identifier:
                    continue
                b0 = (peer.x, peer.y)
                b1 = accepted.get(peer.identifier, b0)
                if swept_distance(a0, candidate, b0, b1) < SAFETY_DISTANCE_M:
                    candidate = a0
                    rejected += 1
                    break
        accepted[agent.identifier] = candidate
    return accepted, rejected


def local_safety_filter(
    agent: Agent,
    proposed: tuple[float, float],
    number_of_agents: int,
    now_s: float,
) -> tuple[tuple[float, float], bool]:
    """Conservative local-information step check; intentionally no global position access.

    Each UAV uses its own position and the last delayed, lossy peer pose. An
    unseen/stale peer causes a hold (availability cost). Peer velocities are
    extrapolated linearly, with a heuristic age margin. This *cannot guarantee*
    physical separation because peer intentions and model errors are unknown.
    """
    start = (agent.x, agent.y)
    if proposed == start:
        return proposed, False
    for peer_id in range(number_of_agents):
        if peer_id == agent.identifier:
            continue
        observation = agent.peer_poses.get(peer_id)
        if observation is None:
            return start, True
        sent_s, x, y, vx, vy = observation
        age = now_s - sent_s
        if age < 0.0 or age > PEER_TTL_S:
            return start, True
        inferred_now = (x + vx * age, y + vy * age)
        inferred_next = (inferred_now[0] + vx * DT_S, inferred_now[1] + vy * DT_S)
        # This is a tunable heuristic, not a worst-case reachability tube.
        margin = SAFETY_DISTANCE_M + 0.25 * age
        if swept_distance(start, proposed, inferred_now, inferred_next) < margin:
            return start, True
    return proposed, False


def run_trial(policy: str, scenario: str, count: int, seed: int) -> dict:
    if policy not in POLICIES or scenario not in SCENARIOS or count < 4:
        raise ValueError("invalid policy, scenario, or UAV count")
    # Separate seeds for environments and communications: initial geometry and
    # stochastic link events are not selected based on the policy outcome.
    geo_rng = random.Random(seed * 104729 + count)
    link_rng = random.Random(seed * 65537 + count)
    agents = make_agents(count)
    tasks = make_tasks(count, geo_rng)
    pending_messages: list[tuple[int, int, int, int, float, int | None, tuple[int, ...], float, float, float, float]] = []
    message_order = 0
    decision_ms: list[float] = []
    packets_sent = packets_delivered = tx_bytes = duplicate_ticks = 0
    separation_violations = 0
    swept_separation_violations = 0
    safety_rejections = 0
    faulted_task: int | None = None
    recovery_time_s: float | None = None
    fault_enabled = scenario in {"node_failure", "combined"}
    degraded_enabled = scenario in {"packet_loss", "combined"}
    steps = int(round(DURATION_S / DT_S))

    for step in range(steps):
        now_s = step * DT_S
        if fault_enabled and math.isclose(now_s, FAULT_START_S):
            # Record the interrupted work before taking the node offline.
            faulted_task = agents[0].task_id
            if faulted_task is None or tasks[faulted_task].done_at_s is not None:
                faulted_task = next((t.identifier for t in tasks
                                    if t.done_at_s is None and t.identifier % count == 0), None)
            agents[0].alive = False
            agents[0].task_id = None

        # Deliver only packets whose simulated delivery time has elapsed.
        while pending_messages and pending_messages[0][0] <= step:
            (
                _, _, receiver_id, sender_id, sent_s, claimed_task, known_done,
                x, y, vx, vy,
            ) = heapq.heappop(pending_messages)
            receiver = agents[receiver_id]
            if not receiver.alive:
                continue
            previous = receiver.cache.get(sender_id)
            # A delayed old packet must never roll back current claims or pose.
            if previous is None or sent_s >= previous[0]:
                receiver.cache[sender_id] = (sent_s, claimed_task)
                receiver.peer_poses[sender_id] = (sent_s, x, y, vx, vy)
            receiver.known_done.update(known_done)
            packets_delivered += 1

        # Decisions use local peer caches, not a centralized global peer-state oracle.
        for agent in agents:
            if not agent.alive:
                continue
            t0 = time.perf_counter_ns()
            agent.task_id = select_task(agent, agents, tasks, policy, now_s)
            decision_ms.append((time.perf_counter_ns() - t0) / 1_000_000.0)

        # Logical point-to-point messages, not RF physical-layer throughput.
        if step % 2 == 0:
            degraded = degraded_enabled and FAULT_START_S <= now_s < FAULT_END_S
            loss_prob = 0.38 if degraded else 0.03
            max_delay_ticks = 8 if degraded else 2
            for sender in agents:
                if not sender.alive:
                    continue
                payload = json.dumps({"id": sender.identifier, "t": now_s,
                                      "target": sender.task_id, "battery": round(sender.battery, 3),
                                      "x": round(sender.x, 3), "y": round(sender.y, 3),
                                      "vx": round(sender.vx, 3), "vy": round(sender.vy, 3),
                                      "done": sorted(sender.known_done)},
                                     separators=(",", ":")).encode("utf-8")
                for receiver in agents:
                    if receiver.identifier == sender.identifier or not receiver.alive:
                        continue
                    packets_sent += 1
                    tx_bytes += len(payload)
                    if link_rng.random() >= loss_prob:
                        wait_ticks = link_rng.randint(1, max_delay_ticks)
                        heapq.heappush(pending_messages, (
                            step + wait_ticks, message_order, receiver.identifier,
                            sender.identifier, now_s, sender.task_id,
                            tuple(sorted(sender.known_done)), sender.x, sender.y,
                            sender.vx, sender.vy,
                        ))
                        message_order += 1

        # Update actual simulated positions and battery, with common dynamics.
        next_positions: dict[int, tuple[float, float]] = {}
        for agent in agents:
            if not agent.alive or agent.task_id is None:
                continue
            target = tasks[agent.task_id]
            d = math.hypot(target.x - agent.x, target.y - agent.y)
            travel = min(SPEED_M_S * DT_S, d)
            if d > 1e-12:
                next_positions[agent.identifier] = (
                    agent.x + travel * (target.x - agent.x) / d,
                    agent.y + travel * (target.y - agent.y) / d)
        live = [a for a in agents if a.alive]
        if policy == "adaptive_freshness_safety":
            next_positions, rejects = safety_filter(agents, next_positions)
            safety_rejections += rejects
        elif policy == "adaptive_freshness_local_safety":
            accepted = {}
            for agent in live:
                proposed = next_positions.get(agent.identifier, (agent.x, agent.y))
                actual, rejected = local_safety_filter(agent, proposed, count, now_s)
                if rejected:
                    safety_rejections += 1
                accepted[agent.identifier] = actual
            next_positions = accepted
        for i, left in enumerate(live):
            a0 = (left.x, left.y)
            a1 = next_positions.get(left.identifier, a0)
            for right in live[i + 1:]:
                b0 = (right.x, right.y)
                b1 = next_positions.get(right.identifier, b0)
                if swept_distance(a0, a1, b0, b1) < SAFETY_DISTANCE_M:
                    swept_separation_violations += 1
        for agent in live:
            if agent.identifier in next_positions:
                new_position = next_positions[agent.identifier]
                travel = math.hypot(new_position[0] - agent.x, new_position[1] - agent.y)
                agent.distance_m += travel
                agent.battery -= travel * 0.025
                agent.vx = (new_position[0] - agent.x) / DT_S
                agent.vy = (new_position[1] - agent.y) / DT_S
                agent.x, agent.y = new_position
            else:
                agent.vx = 0.0
                agent.vy = 0.0

        for i, left in enumerate(live):
            for right in live[i + 1:]:
                if math.hypot(left.x - right.x, left.y - right.y) < SAFETY_DISTANCE_M:
                    separation_violations += 1  # discrete pair-tick violation, not a collision

        # Only physically present peers detect completed targets locally.
        for agent in live:
            if agent.task_id is not None:
                observed = tasks[agent.task_id]
                if (observed.done_at_s is not None and
                        math.hypot(agent.x - observed.x, agent.y - observed.y) <= 1.5):
                    agent.known_done.add(observed.identifier)

        for task in tasks:
            if task.done_at_s is not None:
                continue
            workers = [a for a in live if a.task_id == task.identifier and
                       math.hypot(a.x - task.x, a.y - task.y) <= 1.5]
            if workers:
                workers.sort(key=lambda a: a.identifier)
                duplicate_ticks += max(0, len(workers) - 1)
                task.service_s += DT_S  # only one worker's useful service counts
                if task.service_s + 1e-9 >= SERVICE_S:
                    task.done_at_s = now_s + DT_S
                    task.done_by = workers[0].identifier
                    workers[0].known_done.add(task.identifier)
                    if (fault_enabled and faulted_task == task.identifier and
                            recovery_time_s is None and task.done_by != 0):
                        recovery_time_s = round(task.done_at_s - FAULT_START_S, 3)

    completion_times = [task.done_at_s for task in tasks if task.done_at_s is not None]
    result = {
        "policy": policy, "scenario": scenario, "uavs": count, "seed": seed,
        "tasks_total": len(tasks), "tasks_completed": len(completion_times),
        "task_completion_rate": round(len(completion_times) / len(tasks), 6),
        "last_task_completion_sim_s": max(completion_times) if completion_times else None,
        "fault_orphan_task": faulted_task,
        "orphan_task_recovery_sim_s": recovery_time_s,
        "orphan_task_recovered": recovery_time_s is not None,
        "duplicate_worker_ticks": duplicate_ticks,
        "sampled_separation_violations": separation_violations,
        "swept_separation_violations": swept_separation_violations,
        "safety_rejected_steps": safety_rejections,
        "packets_sent": packets_sent, "packets_delivered": packets_delivered,
        "logical_unicast_tx_bytes": tx_bytes,
        "total_distance_m": round(sum(a.distance_m for a in agents), 4),
        "decision_latency_p95_wall_ms": percentile(decision_ms, 0.95),
        "decision_samples": len(decision_ms),
    }
    return result


def experiment(counts: tuple[int, ...], seeds: int) -> tuple[list[dict], dict]:
    if not counts or any(n < 4 for n in counts) or seeds < 1:
        raise ValueError("counts must be >=4 and seeds must be positive")
    records = [run_trial(policy, scenario, n, seed)
               for n in counts for scenario in SCENARIOS for seed in range(seeds)
               for policy in POLICIES]
    groups = []
    for n in counts:
        for scenario in SCENARIOS:
            for policy in POLICIES:
                subset = [r for r in records if r["uavs"] == n and
                          r["scenario"] == scenario and r["policy"] == policy]
                recovered = [r["orphan_task_recovery_sim_s"] for r in subset
                             if r["orphan_task_recovered"]]
                groups.append({
                    "uavs": n, "scenario": scenario, "policy": policy, "trials": len(subset),
                    "completion_mean": round(statistics.mean(r["task_completion_rate"] for r in subset), 6),
                    "completion_sd": round(statistics.stdev(r["task_completion_rate"] for r in subset), 6)
                    if len(subset) > 1 else None,
                    "recovered_trials": len(recovered),
                    "recovery_s_median_if_recovered": round(statistics.median(recovered), 4)
                    if recovered else None,
                    "duplicate_worker_ticks_mean": round(statistics.mean(r["duplicate_worker_ticks"] for r in subset), 3),
                    "sampled_separation_violations_mean": round(statistics.mean(r["sampled_separation_violations"] for r in subset), 3),
                    "swept_separation_violations_mean": round(statistics.mean(r["swept_separation_violations"] for r in subset), 3),
                    "safety_rejections_mean": round(statistics.mean(r["safety_rejected_steps"] for r in subset), 3),
                    "logical_unicast_tx_bytes_mean": round(statistics.mean(r["logical_unicast_tx_bytes"] for r in subset), 1),
                })
    return records, {"model": "simplified_discrete_time_task_coordination_v4_local_peer_safety",
                     "simulated_seconds": DURATION_S, "dt_seconds": DT_S,
                     "fault_start_seconds": FAULT_START_S, "fault_end_seconds": FAULT_END_S,
                     "peer_ttl_seconds": PEER_TTL_S,
                     "trial_count": len(records), "policies": list(POLICIES),
                     "scenarios": list(SCENARIOS), "counts": list(counts),
                     "seeds_per_condition": seeds, "groups": groups}


def write_outputs(output_dir: Path, records: list[dict], summary: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "runs.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                                encoding="utf-8")
    rows = ["# UAV Swarm Policy Comparison: Software-Simulation Evidence", "",
            "**Scope:** simplified, time-stepped, software-only task model; not C++ runtime, real RF,"
            "UAV flight, validated formation control or a claim of novel algorithm superiority.", "",
            "## Experimental results", "",
            "| UAVs | Scenario | Policy | Completion mean | Recovered trials | Mean TX bytes |",
            "|---:|---|---|---:|---:|---:|"]
    for g in summary["groups"]:
        rows.append(f"| {g['uavs']} | {g['scenario']} | {g['policy']} | "
                    f"{g['completion_mean']:.3f} | {g['recovered_trials']}/{g['trials']} | "
                    f"{g['logical_unicast_tx_bytes_mean']:.0f} |")
    rows.extend(["", "## Measurement definitions", "",
                 "- Task completion: number of tasks whose 2.0 s simulated service requirement was reached.",
                 "- Recovery: elapsed simulated time from permanent UAV-0 failure at 16 s to completion of its interrupted task by another UAV; unrecovered trials are censored, not zero.",
                 "- Duplicate worker ticks: redundant simultaneously present agents at a target; counts each 0.5 s step.",
                 "- Separation violations: sampled agent pairs closer than 1 m; swept violations use continuous interpolation within each step; neither represents physical collisions.",
                 "- Safety-oracle ablation: idealized full-state sequential reservation; uses true positions of all agents and cannot be claimed as distributed or onboard collision avoidance.",
                 "- Local-safety ablation: only own position and time-stamped peer pose received over modeled links; unknown/stale peer means holding motion. Uses heuristic future peer trajectory, not formal collision guarantee.",
                 "- Transmitted bytes: UTF-8 JSON sizes counted per simulated logical unicast; not measured RF throughput.",
                 "- Decision latency: host process time measured using perf_counter_ns; environment-specific, not flight end-to-end latency.",
                 "- Formation error: not measured because this task-centric model does not implement closed-loop formation dynamics.",
                 "", "## Limitations", "",
                 "- Independent toy-model policy controls only, not replications of published algorithms; the safety-oracle arm is optimistic and centralized.",
                 "- Task coordinates are common mission inputs, while completion is learned locally or via peer messages; policies have no global completion oracle.",
                 "- Peer state arrives through delayed/lossy simulated messages. The model does not simulate routing, real radios, battery physics, or sensors.",
                 "- Local-safety arm uses a simulated peer-view, but ignores peer intention and physical state estimation errors, so it does not prove collision freedom.",
                 "- These results cannot establish superiority, real decentralization of the entire mission system, safety certification, or IEEE publication readiness.", ""])
    (output_dir / "REPORT.md").write_text("\n".join(rows), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", default="4,8,12", help="comma-separated UAV counts (>=4)")
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--out", type=Path, default=Path("results/swarm_fault_study"))
    args = parser.parse_args()
    counts = tuple(int(item.strip()) for item in args.counts.split(","))
    records, summary = experiment(counts, args.seeds)
    summary["code_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    write_outputs(args.out, records, summary)
    print(f"Wrote {len(records)} paired simulation trials to {args.out}")


if __name__ == "__main__":
    main()
