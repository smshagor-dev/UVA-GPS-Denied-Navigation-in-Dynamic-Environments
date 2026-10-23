#!/usr/bin/env python3
"""Import independently logged packet TX and gateway RX CSVs for RF evidence.

Never reconstruct a sender transmission inventory from receiver timestamps,
counter gaps, or an RX record's embedded "sent_timestamp". Selection, integrity
and identity are checked; physical provenance is NOT certified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .peer_replay_trace_analysis import Event, MAX_SEQUENCE, validate_transmissions
    from .peer_replay_scenario_study import write_events
else:
    from peer_replay_trace_analysis import Event, MAX_SEQUENCE, validate_transmissions
    from peer_replay_scenario_study import write_events

SENDER_ROLE = "independent_sender_device_log"
RECEIVER_ROLE = "gateway_reception_log"


def hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_time(value: str, kind: str) -> int:
    if kind == "integer_milliseconds":
        try:
            stamp = int(value)
        except ValueError as exc:
            raise ValueError("timestamp must be integer milliseconds") from exc
    elif kind == "iso8601_utc":
        try:
            when = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError as exc:
            raise ValueError("timestamp must be valid ISO-8601") from exc
        if when.tzinfo is None or when.utcoffset() is None:
            raise ValueError("timestamp needs explicit timezone offset")
        stamp = int(when.astimezone(timezone.utc).timestamp() * 1000)
    else:
        raise ValueError("time_format must be integer_milliseconds or iso8601_utc")
    if stamp < 0:
        raise ValueError("negative timestamp not supported")
    return stamp


def require_text(mapping: dict, key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing mandatory value: {key}")
    return value.strip()


def read_role(
    location: Path, mapping: dict, *, sender_id: int, epoch: int, tx: bool
) -> tuple[list[Event], int]:
    identity_column = require_text(mapping, "sequence_column")
    time_column = require_text(mapping, "time_column")
    time_format = require_text(mapping, "time_format")
    filter_column = mapping.get("filter_column")
    filter_value = mapping.get("filter_value")
    if (filter_column is None) != (filter_value is None):
        raise ValueError("filter_column and filter_value must occur together")
    if filter_column is not None:
        filter_column = require_text(mapping, "filter_column")
        filter_value = require_text(mapping, "filter_value")

    with location.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = csv.DictReader(stream)
        needed = {identity_column, time_column}
        if filter_column is not None:
            needed.add(filter_column)
        if rows.fieldnames is None or not needed.issubset(set(rows.fieldnames)):
            raise ValueError(f"{location.name}: missing CSV columns: {sorted(needed)}")
        events = []
        total = 0
        for number, record in enumerate(rows, start=2):
            total += 1
            if filter_column and record[filter_column] != filter_value:
                continue
            try:
                sequence = int(record[identity_column])
            except (ValueError, TypeError) as exc:
                raise ValueError(
                    f"{location.name}:{number}: invalid packet ID"
                ) from exc
            if not 0 <= sequence <= MAX_SEQUENCE:
                raise ValueError(f"{location.name}:{number}: sequence outside uint32")
            timestamp = parse_time(record[time_column] or "", time_format)
            events.append(Event(sender_id, epoch, sequence, timestamp, number))

    if not events:
        raise ValueError(f"{location.name}: selected capture is empty")
    events.sort(key=lambda event: (event.time_ms, event.line))
    if tx:
        validate_transmissions(events)
    return events, total


def run(spec_path: Path, output_dir: Path) -> dict:
    config = json.loads(spec_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("capture specification must be a JSON object")

    flight = require_text(config, "flight_id")
    scenario = require_text(config, "scenario")
    doi = require_text(config, "source_doi")
    sender = config.get("sender_id")
    epoch = config.get("epoch", 1)
    if type(sender) is not int or not 1 <= sender <= MAX_SEQUENCE:
        raise ValueError("sender_id must be uint32 positive integer")
    if type(epoch) is not int or epoch < 0:
        raise ValueError("epoch must be a nonnegative integer")

    tx_spec = config.get("tx")
    rx_spec = config.get("rx")
    if not isinstance(tx_spec, dict) or not isinstance(rx_spec, dict):
        raise ValueError("tx and rx mappings must be objects")
    if tx_spec.get("role") != SENDER_ROLE:
        raise ValueError("TX must be declared independent sender-device log")
    if rx_spec.get("role") != RECEIVER_ROLE:
        raise ValueError("RX must be declared gateway reception log")

    # Relative paths are resolved relative to the specification, not cwd.
    tx_path = (spec_path.parent / require_text(tx_spec, "file")).resolve()
    rx_path = (spec_path.parent / require_text(rx_spec, "file")).resolve()
    if tx_path == rx_path:
        raise ValueError("TX and RX cannot be the same source file")
    # Normalization must never overwrite the investigator's original capture.
    generated = {
        (output_dir / name).resolve()
        for name in (
            "matched-tx.csv",
            "matched-rx.csv",
            "paired-manifest-entry.json",
            "capture-audit.json",
        )
    }
    if tx_path in generated or rx_path in generated:
        raise ValueError("output would overwrite original source capture")

    tx_hash, rx_hash = hash_file(tx_path), hash_file(rx_path)
    if tx_hash == rx_hash:
        raise ValueError("TX and RX cannot have identical file contents")

    sent, tx_rows = read_role(
        tx_path, tx_spec, sender_id=sender, epoch=epoch, tx=True
    )
    received, rx_rows = read_role(
        rx_path, rx_spec, sender_id=sender, epoch=epoch, tx=False
    )
    attempted_ids = {packet.identity for packet in sent}
    for packet in received:
        if packet.identity not in attempted_ids:
            raise ValueError(
                f"RX line {packet.line} has no matching sender-side packet"
            )

    output_dir.mkdir(parents=True, exist_ok=True)
    tx_out = output_dir / "matched-tx.csv"
    rx_out = output_dir / "matched-rx.csv"
    write_events(tx_out, sent, "tx_ms")
    write_events(rx_out, received, "rx_ms")

    manifest = [
        {
            "flight_id": flight,
            "scenario": scenario,
            "source_doi": doi,
            "epoch": epoch,
            "tx": tx_out.name,
            "rx": rx_out.name,
        }
    ]
    (output_dir / "paired-manifest-entry.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    report = {
        "classification": "externally supplied independent TX/RX files",
        "input_origin_verified": False,
        "complete_transmission_inventory_verified": False,
        "physical_rf_provenance_verified": False,
        "packet_loss_attribution_to_rf": False,
        "source_doi_claimed": doi,
        "flight_id": flight,
        "selected_sender_id": sender,
        "epoch": epoch,
        "source_files": {
            "tx": {"sha256": tx_hash, "rows": tx_rows, "selected": len(sent)},
            "rx": {
                "sha256": rx_hash,
                "rows": rx_rows,
                "selected": len(received),
            },
        },
        "normalized_files": {
            "tx": {"name": tx_out.name, "sha256": hash_file(tx_out)},
            "rx": {"name": rx_out.name, "sha256": hash_file(rx_out)},
        },
        "warning": (
            "Declared capture roles are not third-party proof of independent "
            "instrumentation or TX completeness. Audit original acquisition "
            "metadata before interpreting end-to-end missing observations."
        ),
    }
    (output_dir / "capture-audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = run(args.spec, args.output_dir)
    except (ValueError, OSError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
