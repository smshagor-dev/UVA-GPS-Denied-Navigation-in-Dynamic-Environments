"""Paired exploratory inference for the 480-trial *software-only* swarm study.

Mean differences are treatment-minus-control; bootstrap CIs and exact paired
sign-flip p-values use Holm correction across the specified comparison family.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import random
import statistics
from collections import defaultdict
from pathlib import Path

METRICS = (
    "task_completion_rate",
    "sampled_separation_violations",
    "swept_separation_violations",
    "duplicate_worker_ticks",
    "logical_unicast_tx_bytes",
    "orphan_task_recovered",
)


def number(value):
    if value in (True, "True"):
        return 1.0
    if value in (False, "False"):
        return 0.0
    return float(value)


def exact_signflip(diffs):
    if not 1 <= len(diffs) <= 16:
        raise ValueError("exact test only for 1..16 pairs")
    target = abs(sum(diffs))
    matches = sum(
        abs(sum(sign * delta for sign, delta in zip(signs, diffs)))
        >= target - 1e-12
        for signs in itertools.product((-1, 1), repeat=len(diffs))
    )
    return matches / (2 ** len(diffs))


def paired_signflip(diffs, draws=4096):
    """Exact for <=16 paired seeds, otherwise seeded Monte Carlo sign flips.

    The +1 correction is a valid conservative estimate of the tail area.
    A Monte Carlo p-value is *not* labeled exact; randomization error remains.
    """
    if not diffs:
        raise ValueError("no paired observations")
    if len(diffs) <= 16:
        return exact_signflip(diffs), "exact", 2 ** len(diffs)
    if all(abs(delta) < 1e-12 for delta in diffs):
        return 1.0, "monte_carlo_degenerate", draws
    seed_bytes = hashlib.sha256(repr(tuple(diffs)).encode("utf-8")).digest()
    rng = random.Random(int.from_bytes(seed_bytes[:8], "big"))
    threshold = abs(sum(diffs)) - 1e-12
    extreme = 0
    for _ in range(draws):
        bits = rng.getrandbits(len(diffs))
        delta_sum = sum(
            value if bits & (1 << i) else -value
            for i, value in enumerate(diffs)
        )
        extreme += abs(delta_sum) >= threshold
    return (extreme + 1) / (draws + 1), "monte_carlo", draws


def bootstrap_ci(diffs, seed, repeats=5000):
    rng = random.Random(seed)
    values = sorted(
        statistics.mean(rng.choices(diffs, k=len(diffs)))
        for _ in range(repeats)
    )
    return [round(values[int(repeats * 0.025)], 6),
            round(values[int(repeats * 0.975)], 6)]


def analyze(records):
    paired = defaultdict(dict)
    for record in records:
        key = (int(record["uavs"]), record["scenario"], int(record["seed"]))
        policy = record["policy"]
        if policy in paired[key]:
            raise ValueError(f"duplicate pair: {key}/{policy}")
        paired[key][policy] = record
    policy_pairs = (
        ("adaptive_unfiltered", "adaptive_freshness"),
        ("adaptive_freshness", "adaptive_freshness_safety"),
        ("adaptive_freshness", "adaptive_freshness_local_safety"),
    )
    grouped = defaultdict(list)
    for (size, scenario, _), policies in paired.items():
        for control, treatment in policy_pairs:
            if not {control, treatment} <= policies.keys():
                raise ValueError(f"missing paired policies: {control}/{treatment}")
            grouped[size, scenario, control, treatment].append(
                (policies[control], policies[treatment])
            )
    output = []
    for (size, scenario, control, treatment), pairs in sorted(grouped.items()):
        for metric in METRICS:
            differences = [number(t[metric]) - number(c[metric]) for c, t in pairs]
            p_value, method, draws = paired_signflip(differences)
            output.append({
                "uavs": size,
                "scenario": scenario,
                "control": control,
                "treatment": treatment,
                "metric": metric,
                "paired_seeds": len(pairs),
                "treatment_minus_control_mean": round(statistics.mean(differences), 6),
                "mean_diff_bootstrap_95pct_ci": bootstrap_ci(
                    differences,
                    1808 + size + len(scenario) + len(control) + len(treatment),
                ),
                "unadjusted_signflip_p": p_value,
                "signflip_method": method,
                "signflip_sample_count": draws,
            })
    # Holm family-wise multiplicity correction across all outcome comparisons.
    ordered = sorted(range(len(output)), key=lambda i: output[i]["unadjusted_signflip_p"])
    previous = 0.0
    for position, index in enumerate(ordered):
        raw = output[index]["unadjusted_signflip_p"]
        previous = min(1.0, max(previous, raw * (len(output) - position)))
        output[index]["holm_adjusted_p"] = round(previous, 8)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, default=Path("results/swarm_fault_study/runs.csv"))
    parser.add_argument("--out", type=Path, default=Path("results/swarm_fault_study/stats.json"))
    args = parser.parse_args()
    with args.runs.open(encoding="utf-8", newline="") as stream:
        records = list(csv.DictReader(stream))
    comparisons = analyze(records)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "scope": "exploratory paired simulation comparison, not real UAV/RF",
        "policy_pairs": [["adaptive_unfiltered", "adaptive_freshness"], ["adaptive_freshness", "adaptive_freshness_safety"], ["adaptive_freshness", "adaptive_freshness_local_safety"]],
        "multiple_comparison_adjustment": "Holm family-wise across all comparisons; exact for <=16 pairs, seeded sign-flip Monte Carlo beyond 16",
        "censored_recovery": "analyzed as recovered/not recovered, not zero seconds",
        "comparisons": comparisons,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(comparisons)} paired comparisons to {args.out}")


if __name__ == "__main__":
    main()
