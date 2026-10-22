#!/usr/bin/env python3
"""Cluster-level descriptive uncertainty over independent LoRa reception flights.

The input manifest is user-supplied. A run is not certified as a physical
flight by this software; packet-loss and calibrated FANET claims are excluded.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

if __package__:
    from .peer_replay_scenario_study import confidence_interval, verify_native
    from .rf_reception_audit import load_receptions, reception_report
else:
    from peer_replay_scenario_study import confidence_interval, verify_native
    from rf_reception_audit import load_receptions, reception_report

FIELDS = (
    "observed_reception_order_p_r",
    "strict_first_arrival_rejection_fraction",
    "window_first_arrival_rejection_fraction",
    "window_minus_strict_rejection_fraction",
)


def analyze_flights(manifest: Path, *, native: Path | None = None) -> dict:
    entries = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(entries, list) or len(entries) < 2:
        raise ValueError("at least two independently acquired flights are required")
    seen = set()
    results = []
    for entry in entries:
        if not isinstance(entry, dict) or not all(
            isinstance(entry.get(key), str) and entry[key].strip()
            for key in ("flight_id", "csv", "device", "gateway")
        ):
            raise ValueError("flight manifest needs flight_id, csv, device, gateway")
        flight = entry["flight_id"].strip()
        if flight in seen:
            raise ValueError("duplicate flight ID cannot be an independent trial")
        seen.add(flight)
        capture = (manifest.parent / entry["csv"]).resolve()
        received, metadata = load_receptions(
            capture, device=entry["device"], gateway=entry["gateway"]
        )
        report = reception_report(
            received,
            flight=flight,
            device=entry["device"],
            gateway=entry["gateway"],
        )
        count = report["distinct_received_counters"]
        strict = report["strict"]["first_arrival_rejections"] / count
        window = report["window_64"]["first_arrival_rejections"] / count
        results.append(
            {
                "flight": flight,
                "selected_device": entry["device"],
                "selected_gateway": entry["gateway"],
                "input_sha256": hashlib.sha256(capture.read_bytes()).hexdigest(),
                "input_matched_rows": metadata["matched_rows"],
                "distinct_receptions": count,
                "observed_reception_order_p_r": report["observed_reception_order_p_r"],
                "strict_first_arrival_rejection_fraction": strict,
                "window_first_arrival_rejection_fraction": window,
                "window_minus_strict_rejection_fraction": window - strict,
                "native_cpp_per_arrival_verified": (
                    verify_native(native, received) if native is not None else False
                ),
            }
        )
    measures = {}
    for offset, key in enumerate(FIELDS):
        values = [row[key] for row in results]
        measures[key] = {
            "unweighted_flight_mean": sum(values) / len(values),
            "flight_bootstrap_95pct": confidence_interval(values, seed=8051 + offset),
        }
    return {
        "evidence_classification": "external reception-only flights, provenance unverified",
        "independent_trials_self_reported_not_verified": True,
        "sample_count": len(results),
        "packet_loss_estimable": False,
        "packet_loss_fraction": None,
        "rf_calibration_verified": False,
        "ci_method": "95% percentile bootstrap across self-declared flights",
        "ci_limitation": "Describes supplied runs only; not representative population inference",
        "metrics": measures,
        "flights": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    args = parser.parse_args()
    try:
        if args.native is not None and not args.native.is_file():
            parser.error("compiled native replay tool does not exist")
        report = analyze_flights(args.manifest, native=args.native)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, AssertionError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
