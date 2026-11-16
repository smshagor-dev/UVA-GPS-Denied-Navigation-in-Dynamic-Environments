"""Fail-closed provenance and pairing checks for *software-only* swarm artifacts.

This validates internal data consistency, NOT empirical UAV validity, algorithm
novelty, physical collision avoidance or published-method replication.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
from pathlib import Path

from . import swarm_fault_study, swarm_stats

POLICY_PAIRS = (
    ("adaptive_unfiltered", "adaptive_freshness"),
    ("adaptive_freshness", "adaptive_freshness_safety"),
    ("adaptive_freshness", "adaptive_freshness_local_safety"),
    ("adaptive_freshness", "adaptive_freshness_proximity"),
    ("adaptive_freshness_local_safety", "adaptive_freshness_proximity"),
)
NONNEGATIVE_FIELDS = (
    "tasks_total", "tasks_completed", "sampled_separation_violations",
    "swept_separation_violations", "duplicate_worker_ticks",
    "safety_rejected_steps", "simulated_local_sensor_observations",
    "packets_sent", "packets_delivered", "logical_unicast_tx_bytes",
    "decision_samples",
)


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def as_int(value: object, field: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid integer {field}: {value!r}") from exc
    require(str(value) == str(parsed) or isinstance(value, int),
            f"non-canonical integer {field}: {value!r}")
    return parsed


def finite(value: object, field: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid number {field}: {value!r}") from exc
    require(math.isfinite(parsed), f"non-finite {field}")
    return parsed


def validate(records: list[dict], summary: dict, stats: dict,
             counts: tuple[int, ...], seeds: int) -> dict:
    """Reject missing/duplicate trials and inconsistent published aggregates."""
    require(bool(counts) and len(set(counts)) == len(counts) and
            all(count >= 4 for count in counts) and seeds >= 1,
            "invalid evaluation design")
    policies = tuple(swarm_fault_study.POLICIES)
    scenarios = tuple(swarm_fault_study.SCENARIOS)
    expected = {(count, scenario, seed, policy)
                for count in counts for scenario in scenarios
                for seed in range(seeds) for policy in policies}
    require(len(records) == len(expected), "trial count mismatch")
    rows: dict[tuple[int, str, int, str], dict] = {}
    for record in records:
        require(isinstance(record, dict), "malformed trial row")
        count = as_int(record.get("uavs"), "uavs")
        seed = as_int(record.get("seed"), "seed")
        scenario, policy = record.get("scenario"), record.get("policy")
        key = (count, scenario, seed, policy)
        require(key in expected, f"unexpected trial {key}")
        require(key not in rows, f"duplicate trial {key}")
        rows[key] = record

        for field in NONNEGATIVE_FIELDS:
            require(as_int(record.get(field), field) >= 0,
                    f"negative {field} in {key}")
        require(as_int(record["tasks_total"], "tasks_total") == 2 * count,
                f"task inventory mismatch in {key}")
        complete = as_int(record["tasks_completed"], "tasks_completed")
        require(complete <= 2 * count, f"completed more tasks than exist in {key}")
        rate = finite(record.get("task_completion_rate"), "task_completion_rate")
        require(0.0 <= rate <= 1.0 and
                abs(rate - round(complete / (2 * count), 6)) <= 1.1e-6,
                f"task completion denominator mismatch in {key}")
        require(as_int(record["packets_delivered"], "packets_delivered") <=
                as_int(record["packets_sent"], "packets_sent"),
                f"received more packets than transmitted in {key}")
        value = record.get("orphan_task_recovered")
        require(value in (True, False, "True", "False"), f"bad recovery flag in {key}")
        recovered = value in (True, "True")
        recovery_time = record.get("orphan_task_recovery_sim_s")
        require((recovery_time not in ("", None, "None")) == recovered,
                f"censored recovery misclassified in {key}")
        if recovered:
            require(finite(recovery_time, "recovery_time") >= 0,
                    f"negative recovery time in {key}")

    require(set(rows) == expected, "incomplete paired trial design")
    require(as_int(summary.get("trial_count"), "summary trial_count") == len(expected),
            "summary trial count mismatch")
    require(summary.get("counts") == list(counts) and
            summary.get("scenarios") == list(scenarios) and
            summary.get("policies") == list(policies) and
            as_int(summary.get("seeds_per_condition"), "seeds_per_condition") == seeds,
            "summary experiment design mismatch")
    groups = summary.get("groups")
    require(isinstance(groups, list) and
            len(groups) == len(counts) * len(scenarios) * len(policies),
            "summary group count mismatch")
    seen_groups = set()
    for group in groups:
        key = (as_int(group.get("uavs"), "group uavs"),
               group.get("scenario"), group.get("policy"))
        require(key[:1] in tuple((c,) for c in counts) and
                key[1] in scenarios and key[2] in policies,
                f"invalid group {key}")
        require(key not in seen_groups, f"duplicate group {key}")
        seen_groups.add(key)
        require(as_int(group.get("trials"), "group trials") == seeds,
                f"group trial count mismatch {key}")
        actual = statistics.mean(
            finite(rows[key[0], key[1], seed, key[2]]["task_completion_rate"],
                   "task_completion_rate")
            for seed in range(seeds))
        require(abs(finite(group.get("completion_mean"), "completion_mean") -
                    round(actual, 6)) <= 1.1e-6,
                f"group mean mismatch {key}")

    pairs = [list(pair) for pair in POLICY_PAIRS]
    require(stats.get("policy_pairs") == pairs, "statistical comparison design changed")
    comparisons = stats.get("comparisons")
    count_expected = len(counts) * len(scenarios) * len(POLICY_PAIRS) * len(swarm_stats.METRICS)
    require(isinstance(comparisons, list) and
            len(comparisons) == count_expected, "comparison count mismatch")
    seen_comparisons = set()
    for entry in comparisons:
        key = (as_int(entry.get("uavs"), "comparison uavs"), entry.get("scenario"),
               entry.get("control"), entry.get("treatment"), entry.get("metric"))
        count, scenario, control, treatment, metric = key
        require(count in counts and scenario in scenarios and
                (control, treatment) in POLICY_PAIRS and metric in swarm_stats.METRICS,
                f"unknown comparison {key}")
        require(key not in seen_comparisons, f"duplicate comparison {key}")
        seen_comparisons.add(key)
        require(as_int(entry.get("paired_seeds"), "paired_seeds") == seeds,
                f"unpaired comparison {key}")
        differences = [
            swarm_stats.number(rows[count, scenario, seed, treatment][metric]) -
            swarm_stats.number(rows[count, scenario, seed, control][metric])
            for seed in range(seeds)
        ]
        true_mean = round(statistics.mean(differences), 6)
        require(abs(finite(entry.get("treatment_minus_control_mean"), "mean effect") -
                    true_mean) <= 1.1e-6,
                f"invalid effect size {key}")
        ci = entry.get("mean_diff_bootstrap_95pct_ci")
        require(isinstance(ci, list) and len(ci) == 2 and
                finite(ci[0], "CI lower") <= finite(ci[1], "CI upper"),
                f"invalid interval {key}")
        for name in ("unadjusted_signflip_p", "exact_sign_test_p",
                     "holm_adjusted_p", "holm_adjusted_exact_sign_p"):
            value = finite(entry.get(name), name)
            require(0.0 <= value <= 1.0, f"invalid p-value {name}: {key}")
        require(as_int(entry.get("sign_informative_pairs"), "informative pairs") ==
                sum(a != 0.0 for a in differences),
                f"incorrect independent-pair sign sample {key}")

    return {"validated": True, "evidence_type": "software_simulation_only",
            "counts": list(counts), "seeds_per_condition": seeds,
            "trial_count": len(rows), "summary_groups": len(groups),
            "paired_comparisons": len(comparisons),
            "limitations": ["no_real_rf_or_flight", "not_a_novelty_test",
                            "not_a_published_algorithm_replication"]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True,
                        help="Directory containing runs.csv, summary.json, stats.json")
    parser.add_argument("--counts", default="4,8,12")
    parser.add_argument("--seeds", type=int, required=True)
    args = parser.parse_args()
    counts = tuple(int(s.strip()) for s in args.counts.split(","))
    paths = {name: args.out / name for name in ("runs.csv", "summary.json", "stats.json")}
    with paths["runs.csv"].open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    summary = json.loads(paths["summary.json"].read_text(encoding="utf-8"))
    stats = json.loads(paths["stats.json"].read_text(encoding="utf-8"))
    source = Path(swarm_fault_study.__file__)
    hash_source = hashlib.sha256(source.read_bytes()).hexdigest()
    require(summary.get("code_sha256") == hash_source,
            "source code hash differs from simulation provenance")
    result = validate(rows, summary, stats, counts, args.seeds)
    result["source_sha256"] = hash_source
    result["file_sha256"] = {
        name: hashlib.sha256(path.read_bytes()).hexdigest()
        for name, path in paths.items()
    }
    result["github_workflow_sha"] = os.environ.get("GITHUB_SHA") or None
    (args.out / "evidence-integrity.json").write_text(
        json.dumps(result, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(f"Validated {result['trial_count']} software-only trials, "
          f"{result['paired_comparisons']} paired comparisons")


if __name__ == "__main__":
    main()
