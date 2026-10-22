#!/usr/bin/env python3
"""Repeatable, uncalibrated network-emulation stress study of replay policy.

No generated event is an RF measurement. All runs assume authenticated, fresh
packets and a single receiver, epoch, and independent per-sender sequences.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

if __package__:
    from .peer_replay_trace_analysis import Event, analyze, read_events
else:
    from peer_replay_trace_analysis import Event, analyze, read_events

SCENARIOS = {
    "low_load": (3, 50, 0.01, 0.02, 0.20, 8, 0.004),
    "mobile_links": (8, 25, 0.05, 0.10, 0.18, 95, 0.015),
    "congested_mesh": (8, 6, 0.09, 0.14, 0.12, 150, 0.035),
    "intermittent_link": (5, 20, 0.05, 0.25, 0.06, 240, 0.020),
}
# peers, packet interval ms, ordinary loss probability, bad-state entry
# probability, bad-state recovery probability, delay-jitter scale ms,
# and duplicate delivery probability. Values are illustrative, NOT calibrated.


def generate(name: str, seed: int, packets_per_peer: int = 240):
    peers, interval, base_loss, enter_bad, recover, jitter, duplication = SCENARIOS[name]
    rng = random.Random(seed)
    sent, receptions = [], []
    for sender in range(1, peers + 1):
        bad = False
        for seq in range(1, packets_per_peer + 1):
            tx = seq * interval + sender * 3
            sent.append(Event(sender, 1, seq, tx, len(sent) + 2))
            if bad:
                bad = rng.random() >= recover
            else:
                bad = rng.random() < enter_bad
            if rng.random() < (0.70 if bad else base_loss):
                continue
            latency = max(1, round(rng.expovariate(1 / jitter)))
            rx = tx + latency
            receptions.append(Event(sender, 1, seq, rx, len(receptions) + 2))
            if rng.random() < duplication:
                receptions.append(
                    Event(sender, 1, seq, rx + rng.randrange(1, 16), len(receptions) + 2)
                )
    sent.sort(key=lambda item: (item.time_ms, item.line))
    receptions.sort(key=lambda item: (item.time_ms, item.line))
    return sent, receptions


def expected_window_decisions(received):
    states = {}
    outcomes = []
    for event in received:
        top, mask = states.get(event.sender_id, (-1, 0))
        seq = event.sequence
        if seq > top:
            shift = seq - top
            mask = 1 if shift >= 64 else ((mask << shift) | 1) & ((1 << 64) - 1)
            top = seq
            allowed = True
        elif top - seq >= 64:
            allowed = False
        else:
            bit = 1 << (top - seq)
            allowed = not bool(mask & bit)
            if allowed:
                mask |= bit
        states[event.sender_id] = (top, mask)
        outcomes.append(allowed)
    return outcomes


def write_events(path, items, time_key):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("sender_id", "epoch", "sequence", time_key))
        for event in items:
            writer.writerow((*event.identity, event.time_ms))


def verify_native(binary: Path, received):
    """Require per-arrival parity with compiled production SwarmStateCache."""
    with tempfile.TemporaryDirectory() as directory:
        rx = Path(directory) / "rx.csv"
        write_events(rx, received, "rx_ms")
        proc = subprocess.run(
            [str(binary.resolve()), str(rx)],
            capture_output=True,
            text=True,
            timeout=90,
            check=True,
        )
    rows = list(csv.DictReader(proc.stdout.splitlines()))
    expected = expected_window_decisions(received)
    if len(rows) != len(expected):
        raise AssertionError(f"native output count {len(rows)} != {len(expected)}")
    for index, (row, event, accepted) in enumerate(zip(rows, received, expected)):
        identity = (int(row["sender_id"]), int(row["epoch"]), int(row["sequence"]))
        if identity != event.identity or int(row["rx_ms"]) != event.time_ms:
            raise AssertionError(f"native packet order/provenance mismatch at {index}")
        if row["accepted"] not in ("0", "1"):
            raise AssertionError(f"invalid native admission at {index}")
        if bool(int(row["accepted"])) != accepted:
            raise AssertionError(f"native/sequence reference mismatch at arrival {index}")
    return True


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]


def confidence_interval(values, seed=2026, resamples=1000):
    """Percentile cluster bootstrap across independent seeded runs."""
    rng = random.Random(seed)
    size = len(values)
    sample_means = [
        sum(values[rng.randrange(size)] for _ in range(size)) / size
        for _ in range(resamples)
    ]
    return [percentile(sample_means, 0.025), percentile(sample_means, 0.975)]


def study(seeds=12, packets_per_peer=240, native=None):
    if seeds < 2 or packets_per_peer < 70:
        raise ValueError("at least two independent runs and 70 packets per peer required")
    summary = {
        "classification": "uncalibrated synthetic network-emulation sensitivity study",
        "not_rf_measurements": True,
        "not_a_validated_fanet_channel_model": True,
        "authenticated_and_fresh_packets_assumed": True,
        "native_cpp_per_arrival_verified": native is not None,
        "seed_count_per_scenario": seeds,
        "packets_per_peer": packets_per_peer,
        "confidence_interval": "95% percentile cluster bootstrap over independent seeds; descriptive, not population generalization",
        "scenarios": {},
    }
    for scenario_index, name in enumerate(SCENARIOS):
        records = []
        for iteration in range(seeds):
            seed = 1701 + scenario_index * 100000 + iteration * 7919
            tx, rx = generate(name, seed, packets_per_peer)
            report = analyze(tx, rx, source="synthetic")
            if native is not None:
                verify_native(native, rx)
            t, strict, window = report["trace"], report["strict"], report["window_64"]
            records.append(
                {
                    "seed": seed,
                    "transmitted": t["transmitted_distinct"],
                    "delivered_unique": t["received_distinct"],
                    "duplicates": t["observed_duplicate_deliveries"],
                    "reordering_fraction": t["observed_p_r"],
                    "loss_fraction": t["observed_loss_fraction"],
                    "strict_benign_rejection_fraction": strict["benign_rejection_fraction"],
                    "window_benign_rejection_fraction": window["benign_rejection_fraction"],
                    "window_minus_strict_benign_rejection_fraction": (
                        window["benign_rejection_fraction"]
                        - strict["benign_rejection_fraction"]
                    ),
                }
            )
        fields = (
            "loss_fraction",
            "reordering_fraction",
            "strict_benign_rejection_fraction",
            "window_benign_rejection_fraction",
            "window_minus_strict_benign_rejection_fraction",
        )
        metrics = {}
        for field_index, field in enumerate(fields):
            numbers = [record[field] for record in records]
            metrics[field] = {
                "mean": sum(numbers) / len(numbers),
                "bootstrap_95pct": confidence_interval(
                    numbers, seed=113 + field_index + scenario_index * 13
                ),
            }
        summary["scenarios"][name] = {
            "illustrative_parameters": dict(
                zip(
                    (
                        "senders", "interval_ms", "base_loss", "bad_state_entry",
                        "bad_state_recovery", "jitter_scale_ms", "duplicate_chance",
                    ),
                    SCENARIOS[name],
                )
            ),
            "metrics": metrics,
            "runs": records,
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", type=Path, help="compiled production replay cache CLI")
    parser.add_argument("--external-tx", type=Path, help="external matched sender trace")
    parser.add_argument("--external-rx", type=Path, help="external matched receiver trace")
    parser.add_argument("--seeds", type=int, default=12)
    parser.add_argument("--packets-per-peer", type=int, default=240)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.native is not None and not args.native.is_file():
        parser.error("native executable does not exist")
    if bool(args.external_tx) != bool(args.external_rx):
        parser.error("--external-tx and --external-rx must be supplied together")
    if args.external_tx:
        if args.native is None:
            parser.error("external C++ comparison requires --native")
        tx = read_events(args.external_tx, "tx_ms", 1)
        rx = read_events(args.external_rx, "rx_ms", 1)
        result = {
            "classification": "externally supplied trace, measurement origin unverified",
            "validated_rf_measurement": False,
            "source_provenance_verified": False,
            "native_cpp_per_arrival_verified": verify_native(args.native, rx),
            "sha256": {
                "tx": hashlib.sha256(args.external_tx.read_bytes()).hexdigest(),
                "rx": hashlib.sha256(args.external_rx.read_bytes()).hexdigest(),
            },
            "analysis": analyze(tx, rx, source="measured_unverified"),
        }
    else:
        result = study(args.seeds, args.packets_per_peer, args.native)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
