#!/usr/bin/env python3
"""Audit third-party LoRa reception logs without inventing transmitted packets.

This is a reception-only comparison of sequence admission behavior, NOT a
packet-loss measurement, a calibrated FANET experiment, or proof of RF origin.
A flight, device and receiving gateway must be selected explicitly.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .peer_replay_scenario_study import verify_native, write_events
    from .peer_replay_trace_analysis import Event, MAX_SEQUENCE
else:
    from peer_replay_scenario_study import verify_native, write_events
    from peer_replay_trace_analysis import Event, MAX_SEQUENCE

REQUIRED = {
    "dev_eui",
    "f_cnt",
    "deduplication_id",
    "rx_info_gatewayId",
    "rx_info_time",
}


def parse_time(value: str) -> int:
    """Require an explicitly timezone-qualified ISO-8601 receive timestamp."""
    if not value:
        raise ValueError("missing receive timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("invalid ISO-8601 receive timestamp") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("receive timestamp requires UTC offset")
    millis = int(parsed.astimezone(timezone.utc).timestamp() * 1000)
    if millis < 0:
        raise ValueError("timestamp predates Unix epoch")
    return millis


def load_receptions(
    path: Path, *, device: str, gateway: str, sender_id: int = 1, epoch: int = 1
) -> tuple[list[Event], dict]:
    if not (1 <= sender_id <= MAX_SEQUENCE) or epoch < 0:
        raise ValueError("invalid sender ID or epoch")
    if not device.strip() or not gateway.strip():
        raise ValueError("device and gateway must be supplied explicitly")

    received = []
    identifiers = {}
    snr_values = []
    rssi_values = []
    selected_rows = 0
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or not REQUIRED.issubset(set(reader.fieldnames)):
            raise ValueError(f"LoRa capture missing required columns: {sorted(REQUIRED)}")
        for line, row in enumerate(reader, start=2):
            if row["dev_eui"].strip().lower() != device.strip().lower():
                continue
            if row["rx_info_gatewayId"].strip() != gateway.strip():
                continue
            selected_rows += 1
            try:
                sequence = int(row["f_cnt"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"line {line}: invalid LoRa frame counter") from exc
            if not 0 <= sequence <= MAX_SEQUENCE:
                raise ValueError(f"line {line}: LoRa counter outside uint32 range")
            delivery = row["deduplication_id"].strip()
            if not delivery:
                raise ValueError(f"line {line}: missing deduplication identity")
            prior = identifiers.setdefault(sequence, delivery)
            if prior != delivery:
                raise ValueError(
                    f"line {line}: frame counter reused with different packet identity; "
                    "split capture at reset/rejoin or inspect sender provenance"
                )
            stamp = parse_time(row["rx_info_time"].strip())
            received.append(Event(sender_id, epoch, sequence, stamp, line))
            for column, values in (
                ("rx_info_snr", snr_values),
                ("rx_info_rssi", rssi_values),
            ):
                if column in row and row[column] not in (None, ""):
                    try:
                        measurement = float(row[column])
                    except ValueError as exc:
                        raise ValueError(f"line {line}: invalid {column}") from exc
                    if not math.isfinite(measurement):
                        raise ValueError(f"line {line}: nonfinite {column}")
                    values.append(measurement)
    if not received:
        raise ValueError("no receptions match the selected device and gateway")
    received.sort(key=lambda event: (event.time_ms, event.line))
    return received, {
        "matched_rows": selected_rows,
        "snr_samples": len(snr_values),
        "snr_mean_db": sum(snr_values) / len(snr_values) if snr_values else None,
        "rssi_samples": len(rssi_values),
        "rssi_mean_dbm": sum(rssi_values) / len(rssi_values) if rssi_values else None,
    }


def reception_report(received: list[Event], *, flight: str, device: str, gateway: str):
    """Compare counters only. Unobserved TX packets have no known denominator."""
    if not flight.strip():
        raise ValueError("flight identifier is mandatory")
    strict_high = None
    window_high = None
    bitmap = 0
    observed = set()
    counts = Counter()
    for packet in received:
        seq = packet.sequence
        duplicate = seq in observed
        if duplicate:
            counts["duplicate_receptions"] += 1
        else:
            observed.add(seq)
            counts["distinct_receptions"] += 1
            if strict_high is not None and seq < strict_high:
                counts["reordered_distinct"] += 1
        strict_ok = strict_high is None or seq > strict_high
        if strict_ok:
            strict_high = seq
            counts["strict_accepted"] += 1
        elif not duplicate:
            counts["strict_rejected_first_arrival"] += 1

        if window_high is None:
            window_high, bitmap, allowed = seq, 1, True
        elif seq > window_high:
            shift = seq - window_high
            bitmap = 1 if shift >= 64 else ((bitmap << shift) | 1) & ((1 << 64) - 1)
            window_high, allowed = seq, True
        elif window_high - seq >= 64:
            allowed = False
        else:
            bit = 1 << (window_high - seq)
            allowed = not bool(bitmap & bit)
            if allowed:
                bitmap |= bit
        if allowed:
            counts["window_accepted"] += 1
        elif not duplicate:
            counts["window_rejected_first_arrival"] += 1
    distinct = counts["distinct_receptions"]
    return {
        "classification": "external LoRa reception-only trace; origin not verified",
        "capture_origin_verified": False,
        "empirical_packet_loss_supported": False,
        "tx_inventory_available": False,
        "observed_packet_loss_fraction": None,
        "calibration_supported": False,
        "flight": flight,
        "selected_device": device,
        "selected_gateway": gateway,
        "received_deliveries": len(received),
        "distinct_received_counters": distinct,
        "duplicate_receptions": counts["duplicate_receptions"],
        "reordered_distinct_counters": counts["reordered_distinct"],
        "observed_reception_order_p_r": counts["reordered_distinct"] / distinct,
        "strict": {
            "accepted": counts["strict_accepted"],
            "first_arrival_rejections": counts["strict_rejected_first_arrival"],
        },
        "window_64": {
            "accepted": counts["window_accepted"],
            "first_arrival_rejections": counts["window_rejected_first_arrival"],
        },
        "limitations": [
            "Only received LoRa frames are observable; no transmission denominator.",
            "Not a measured packet-loss rate or calibrated FANET mesh model.",
            "Frame counters are not authenticated OpenMind UAV peer packet sequences.",
            "Counter reuse with different packet identity must be segmented and reviewed.",
            "Gateway-specific first arrivals can differ from LoRa network-server ordering.",
            "Sequence policy assumes authentic and unexpired inputs.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rx-log", type=Path, required=True)
    parser.add_argument("--flight-id", required=True)
    parser.add_argument("--device", required=True)
    parser.add_argument("--gateway", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--normalized-rx", type=Path)
    parser.add_argument("--native", type=Path)
    args = parser.parse_args()
    if args.native and not args.normalized_rx:
        parser.error("--native requires --normalized-rx for trace auditing")
    try:
        received, metadata = load_receptions(
            args.rx_log, device=args.device, gateway=args.gateway
        )
        result = reception_report(
            received,
            flight=args.flight_id,
            device=args.device,
            gateway=args.gateway,
        )
        result["capture"] = {
            "sha256": hashlib.sha256(args.rx_log.read_bytes()).hexdigest(),
            "selected_columns": sorted(REQUIRED),
            **metadata,
        }
        if args.native:
            result["native_cpp_per_arrival_verified"] = verify_native(
                args.native, received
            )
        else:
            result["native_cpp_per_arrival_verified"] = False
        if args.normalized_rx:
            args.normalized_rx.parent.mkdir(parents=True, exist_ok=True)
            write_events(args.normalized_rx, received, "rx_ms")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, AssertionError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
