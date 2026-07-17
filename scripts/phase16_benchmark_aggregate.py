#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import random
import shutil
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCENARIOS = {
    "active_only": {"repetitions": 10},
    "active_with_shadow": {"repetitions": 10},
    "shadow_overload": {"repetitions": 5},
}


@dataclass(frozen=True)
class RunRequest:
    scenario: str
    repetition_index: int


def run_command(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )


def detect_cpu_model() -> str:
    if sys.platform.startswith("linux"):
        cpuinfo = Path("/proc/cpuinfo")
        if cpuinfo.exists():
            for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
                if line.lower().startswith("model name"):
                    return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def compute_stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {
            "mean": 0.0,
            "median": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "min": 0.0,
            "max": 0.0,
            "stddev": 0.0,
            "sample_count": 0,
        }

    ordered = sorted(values)
    mean = statistics.fmean(ordered)
    median = statistics.median(ordered)

    def percentile(p: float) -> float:
        if len(ordered) == 1:
            return ordered[0]
        index = round(p * (len(ordered) - 1))
        return ordered[index]

    stddev = statistics.pstdev(ordered) if len(ordered) > 1 else 0.0
    return {
        "mean": mean,
        "median": median,
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "min": ordered[0],
        "max": ordered[-1],
        "stddev": stddev,
        "sample_count": len(ordered),
    }


def confidence_interval_95(values: list[float]) -> dict[str, float] | None:
    if len(values) < 2:
        return None
    mean = statistics.fmean(values)
    stddev = statistics.stdev(values)
    margin = 1.96 * stddev / math.sqrt(len(values))
    return {"low": mean - margin, "high": mean + margin}


def extract_single_scenario(report: dict[str, Any]) -> dict[str, Any]:
    scenarios = report.get("scenarios", [])
    if len(scenarios) != 1:
        raise ValueError("benchmark run did not return exactly one scenario")
    return scenarios[0]


def summarize_scenario_runs(runs: list[dict[str, Any]]) -> dict[str, Any]:
    active_run_means = [run["active_process_latency_us"]["mean"] for run in runs]
    active_all_samples = [
        sample
        for run in runs
        for sample in run.get("active_process_samples_us", [])
    ]
    shadow_all_samples = [
        sample
        for run in runs
        for sample in run.get("shadow_processing_samples_us", [])
    ]
    active_medians = [run["active_process_latency_us"]["median"] for run in runs]
    shadow_medians = [
        run["shadow_processing_latency_us"]["median"]
        for run in runs
        if run["shadow_processing_latency_us"]["sample_count"] > 0
    ]
    active_output_failures = sum(
        1 for run in runs if not run["active_output_matches_baseline"]
    )
    failed_runs = sum(1 for run in runs if not run["success"])
    active_stats = compute_stats(active_all_samples)
    shadow_stats = compute_stats(shadow_all_samples)
    run_mean_stats = compute_stats(active_run_means)
    median_difference = (
        statistics.fmean(active_medians) - statistics.fmean(shadow_medians)
        if shadow_medians
        else 0.0
    )
    percentage_difference = (
        (statistics.fmean(shadow_medians) - statistics.fmean(active_medians))
        / statistics.fmean(active_medians)
        * 100.0
        if shadow_medians and statistics.fmean(active_medians) > 0.0
        else 0.0
    )

    return {
        "run_count": len(runs),
        "failed_run_count": failed_runs,
        "output_equivalence_failure_count": active_output_failures,
        "active_run_mean_us": {
            "mean_of_run_means": statistics.fmean(active_run_means) if active_run_means else 0.0,
            "median_of_run_means": statistics.median(active_run_means) if active_run_means else 0.0,
            "minimum_run_mean": min(active_run_means) if active_run_means else 0.0,
            "maximum_run_mean": max(active_run_means) if active_run_means else 0.0,
            "stddev_of_run_means": statistics.pstdev(active_run_means)
            if len(active_run_means) > 1
            else 0.0,
            "confidence_interval_95": confidence_interval_95(active_run_means),
        },
        "active_aggregate_latency_us": active_stats,
        "shadow_aggregate_latency_us": shadow_stats,
        "worst_maximum_latency_us": max(
            (run["active_process_latency_us"]["max"] for run in runs), default=0.0
        ),
        "average_queue_high_water_mark": statistics.fmean(
            [run["shadow_queue_high_water_mark"] for run in runs]
        )
        if runs
        else 0.0,
        "maximum_queue_high_water_mark": max(
            (run["shadow_queue_high_water_mark"] for run in runs), default=0
        ),
        "total_dropped_event_count": sum(
            run["shadow_dropped_event_count"] for run in runs
        ),
        "maximum_final_lag_ms": max((run["shadow_lag_ms"] for run in runs), default=0.0),
        "final_shadow_healths": sorted({run["shadow_health"] for run in runs}),
        "active_versus_shadow_median_difference_us": median_difference,
        "active_versus_shadow_percentage_difference": percentage_difference,
        "active_run_mean_distribution_us": run_mean_stats,
    }


def build_run_order(seed: int) -> list[RunRequest]:
    order = [
        RunRequest(scenario=scenario, repetition_index=index + 1)
        for scenario, metadata in SCENARIOS.items()
        for index in range(metadata["repetitions"])
    ]
    rng = random.Random(seed)
    rng.shuffle(order)
    return order


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate repeated Phase 16 benchmark runs.")
    parser.add_argument("--benchmark-exe", required=True, help="Path to estimator_shadow_benchmark executable")
    parser.add_argument("--repo-root", default=".", help="Repository root")
    parser.add_argument("--output-dir", default="artifacts", help="Artifacts output directory")
    parser.add_argument("--warmup-iterations", type=int, default=250)
    parser.add_argument("--measured-iterations", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=16016)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    benchmark_exe = Path(args.benchmark_exe)
    if not benchmark_exe.is_absolute():
        benchmark_exe = (repo_root / benchmark_exe).resolve()
    if not benchmark_exe.exists():
        raise FileNotFoundError(f"benchmark executable not found: {benchmark_exe}")

    artifacts_root = (repo_root / args.output_dir).resolve()
    runs_dir = artifacts_root / "phase16d_benchmark_runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    aggregate_path = artifacts_root / "phase16d_benchmark_aggregate.json"
    summary_path = artifacts_root / "phase16d_benchmark_summary.md"

    git_sha = run_command(["git", "rev-parse", "HEAD"], repo_root)
    commit_sha = git_sha.stdout.strip() if git_sha.returncode == 0 else "unknown"

    run_order = build_run_order(args.seed)
    run_records: list[dict[str, Any]] = []
    scenario_runs: dict[str, list[dict[str, Any]]] = {scenario: [] for scenario in SCENARIOS}
    compiler = "unknown"
    build_type = "unknown"
    operating_system = platform.system().lower()

    for run_index, request in enumerate(run_order, start=1):
        output_path = runs_dir / f"{run_index:02d}_{request.scenario}_run_{request.repetition_index:02d}.json"
        command = [
            str(benchmark_exe),
            "--scenario",
            request.scenario,
            "--warmup-iterations",
            str(args.warmup_iterations),
            "--iterations",
            str(args.measured_iterations),
            "--emit-samples",
            "--output",
            str(output_path),
        ]
        completed = run_command(command, repo_root)
        if completed.returncode != 0:
            raise RuntimeError(
                f"benchmark run failed for {request.scenario} repetition {request.repetition_index}:\n"
                f"stdout:\n{completed.stdout}\n\nstderr:\n{completed.stderr}"
            )

        report = json.loads(output_path.read_text(encoding="utf-8"))
        scenario_report = extract_single_scenario(report)
        scenario_report["command"] = command
        scenario_report["run_index"] = run_index
        scenario_report["repetition_index"] = request.repetition_index
        scenario_report["raw_report_path"] = str(output_path.relative_to(repo_root))
        run_records.append(scenario_report)
        scenario_runs[request.scenario].append(scenario_report)
        compiler = report.get("compiler", compiler)
        build_type = report.get("build_type", build_type)
        operating_system = report.get("operating_system", operating_system)

    aggregates = {
        scenario: summarize_scenario_runs(runs)
        for scenario, runs in scenario_runs.items()
    }

    output_equivalence_ok = all(
        run["active_output_matches_baseline"] for run in run_records
    )
    overload_runs = scenario_runs["shadow_overload"]
    overload_bounded = all(
        run["success"]
        and run["shadow_queue_high_water_mark"] <= run["config"]["queue_depth"]
        and run["shadow_dropped_event_count"] >= 0
        for run in overload_runs
    )

    aggregate_report = {
        "schema_version": 1,
        "commit_sha": commit_sha,
        "compiler": compiler,
        "compiler_version": compiler,
        "build_type": build_type,
        "operating_system": operating_system,
        "cpu": detect_cpu_model(),
        "timer_source": "std::chrono::steady_clock",
        "benchmark_configuration": {
            "warmup_iteration_count": args.warmup_iterations,
            "measured_iteration_count": args.measured_iterations,
            "scenario_repetitions": {
                scenario: metadata["repetitions"] for scenario, metadata in SCENARIOS.items()
            },
        },
        "run_order_strategy": "deterministic_randomized",
        "randomization_seed": args.seed,
        "run_order": [
            {
                "run_index": index + 1,
                "scenario": request.scenario,
                "repetition_index": request.repetition_index,
            }
            for index, request in enumerate(run_order)
        ],
        "individual_runs": run_records,
        "scenario_aggregates": aggregates,
        "active_output_equivalence_result": output_equivalence_ok,
        "shadow_overload_bounded_result": overload_bounded,
        "pass_status": output_equivalence_ok and overload_bounded,
        "failure_reason": "none" if output_equivalence_ok and overload_bounded else "benchmark acceptance criteria not met",
        "limitations": [
            "software benchmark only",
            "does not imply hard real-time guarantees",
            "does not imply flight readiness",
            "timing remains sensitive to scheduler, cache, CPU frequency, and background load variance",
        ],
    }
    aggregate_path.write_text(json.dumps(aggregate_report, indent=2) + "\n", encoding="utf-8")

    summary_lines = [
        "# Phase 16D Benchmark Summary",
        "",
        f"- Commit SHA: `{commit_sha}`",
        f"- Compiler: `{compiler}`",
        f"- Build type: `{build_type}`",
        f"- Operating system: `{operating_system}`",
        f"- CPU: `{aggregate_report['cpu']}`",
        f"- Warm-up iterations per run: `{args.warmup_iterations}`",
        f"- Measured iterations per run: `{args.measured_iterations}`",
        f"- Run ordering: deterministic randomized with seed `{args.seed}`",
        "",
        "## Scenario aggregates",
        "",
    ]
    for scenario, aggregate in aggregates.items():
        summary_lines.extend(
            [
                f"### {scenario}",
                "",
                f"- Run count: `{aggregate['run_count']}`",
                f"- Failed runs: `{aggregate['failed_run_count']}`",
                f"- Output-equivalence failures: `{aggregate['output_equivalence_failure_count']}`",
                f"- Mean of run means (active us): `{aggregate['active_run_mean_us']['mean_of_run_means']:.3f}`",
                f"- Median of run means (active us): `{aggregate['active_run_mean_us']['median_of_run_means']:.3f}`",
                f"- Aggregate median (active us): `{aggregate['active_aggregate_latency_us']['median']:.3f}`",
                f"- Aggregate p95 (active us): `{aggregate['active_aggregate_latency_us']['p95']:.3f}`",
                f"- Aggregate p99 (active us): `{aggregate['active_aggregate_latency_us']['p99']:.3f}`",
                f"- Worst maximum latency (active us): `{aggregate['worst_maximum_latency_us']:.3f}`",
                f"- Total dropped events: `{aggregate['total_dropped_event_count']}`",
                f"- Maximum queue high-water mark: `{aggregate['maximum_queue_high_water_mark']}`",
                f"- Final shadow healths: `{', '.join(aggregate['final_shadow_healths'])}`",
                "",
            ]
        )

    summary_lines.extend(
        [
            "## Acceptance summary",
            "",
            f"- Active-output equivalence held for all runs: `{output_equivalence_ok}`",
            f"- Overload queue remained bounded: `{overload_bounded}`",
            f"- Aggregate pass status: `{aggregate_report['pass_status']}`",
            "",
        ]
    )
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
