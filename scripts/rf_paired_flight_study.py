#!/usr/bin/env python3
"""Evaluate independent, matched sender TX and receiver RX field logs.

Input files are supplied by the investigator: this tool validates consistency,
not the alleged physical origin, completeness, or RF ground truth of any data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

if __package__:
    from .peer_replay_scenario_study import confidence_interval, verify_native
    from .peer_replay_trace_analysis import analyze, read_events
else:
    from peer_replay_scenario_study import confidence_interval, verify_native
    from peer_replay_trace_analysis import analyze, read_events

MEASURES = (
    "observed_missing_fraction",
    "observed_reordering_fraction",
    "strict_first_arrival_rejection_fraction",
    "window_first_arrival_rejection_fraction",
    "window_minus_strict_rejection_fraction",
)


def _sha256(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def analyze_manifest(manifest: Path, *, native: Path | None = None) -> dict:
    """Compare policies on each matched flight, then bootstrap whole flights."""
    entries = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(entries, list) or len(entries) < 2:
        raise ValueError("at least two independent matched captures required")
    if native is not None and not native.is_file():
        raise ValueError("compiled native C++ executable not found")

    runs = defaultdict(list)
    seen_flights = set()
    seen_trace_pairs = set()
    seen_file_hashes = set()
    for entry in entries:
        required = ("flight_id", "scenario", "tx", "rx", "source_doi")
        if not isinstance(entry, dict) or not all(
            isinstance(entry.get(key), str) and entry[key].strip()
            for key in required
        ):
            raise ValueError(f"manifest entries require: {', '.join(required)}")
        flight = entry["flight_id"].strip()
        scenario = entry["scenario"].strip()
        if flight in seen_flights:
            raise ValueError("flight_id repeated: independence cannot be assumed")
        seen_flights.add(flight)
        try:
            epoch = int(entry.get("epoch", 1))
        except (TypeError, ValueError) as exc:
            raise ValueError("epoch must be a nonnegative integer") from exc
        if epoch < 0 or (native is not None and epoch != 1):
            raise ValueError("invalid epoch, or native CLI requires epoch 1")
        tx_path = (manifest.parent / entry["tx"]).resolve()
        rx_path = (manifest.parent / entry["rx"]).resolve()
        if tx_path == rx_path:
            raise ValueError("sender and receiver must have separate log files")
        tx_hash, rx_hash = _sha256(tx_path), _sha256(rx_path)
        if (tx_hash, rx_hash) in seen_trace_pairs:
            raise ValueError("duplicate TX/RX capture pair cannot count as new trial")
        seen_trace_pairs.add((tx_hash, rx_hash))
        if tx_hash in seen_file_hashes or rx_hash in seen_file_hashes:
            raise ValueError("reused TX or RX capture across independent trials")
        seen_file_hashes.update((tx_hash, rx_hash))

        sent = read_events(tx_path, "tx_ms", epoch)
        received = read_events(rx_path, "rx_ms", epoch)
        data = analyze(sent, received, source="measured_unverified")
        if native is not None:
            verify_native(native, received)
        trace = data["trace"]
        strict = data["strict"]["benign_rejection_fraction"]
        window = data["window_64"]["benign_rejection_fraction"]
        sample = {
            "flight_id": flight,
            "scenario": scenario,
            "source_doi_claimed": entry["source_doi"],
            "epoch": epoch,
            "tx_sha256": tx_hash,
            "rx_sha256": rx_hash,
            "transmitted_distinct": trace["transmitted_distinct"],
            "received_distinct": trace["received_distinct"],
            "duplicate_receptions": trace["observed_duplicate_deliveries"],
            "observed_missing_fraction": trace["observed_loss_fraction"],
            "observed_reordering_fraction": trace["observed_p_r"],
            "strict_first_arrival_rejection_fraction": strict,
            "window_first_arrival_rejection_fraction": window,
            "window_minus_strict_rejection_fraction": window - strict,
            "native_sequence_admission_parity_verified": native is not None,
        }
        runs[scenario].append(sample)

    scenario_summaries = {}
    for scenario, records in sorted(runs.items()):
        metrics = {}
        for index, field in enumerate(MEASURES):
            values = [record[field] for record in records]
            if any(value is None for value in values):
                raise ValueError("each capture requires at least one first reception")
            metrics[field] = {
                "mean_of_flights": sum(values) / len(values),
                "bootstrap_95pct": (
                    confidence_interval(values, seed=4301 + index)
                    if len(values) >= 2
                    else None
                ),
            }
        scenario_summaries[scenario] = {
            "independent_flight_count_self_declared": len(records),
            "metrics": metrics,
            "flights": records,
        }
    return {
        "classification": "externally supplied paired TX/RX records, origin unverified",
        "matched_sender_tx_inventory_supplied": True,
        "tx_completeness_verified": False,
        "physical_rf_origin_verified": False,
        "measured_rf_packet_loss_established": False,
        "observed_loss_is_end_to_end_not_rf_attribution": True,
        "model_scope": "authenticated and fresh packets assumed; sequence policy only",
        "replay_cpp_parity_checked": native is not None,
        "confidence_interval_method": (
            "95% percentile bootstrap across entire declared flights"
        ),
        "independence_cannot_be_established_from_manifest": True,
        "scenarios": scenario_summaries,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    args = parser.parse_args()
    try:
        result = analyze_manifest(args.manifest, native=args.native)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except (OSError, ValueError, AssertionError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
