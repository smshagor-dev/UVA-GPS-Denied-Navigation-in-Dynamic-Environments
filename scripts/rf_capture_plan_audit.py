#!/usr/bin/env python3
"""Audit paired field captures against a separately documented transmission plan.

The plan records packet IDs expected *before* comparing field captures.
Matching the plan proves internal coverage only, not physical RF provenance,
sender instrumentation completeness, or independence of the plan itself.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import re
from pathlib import Path

if __package__:
    from .peer_replay_trace_analysis import (
        MAX_SEQUENCE,
        analyze,
        read_events,
        validate_transmissions,
    )
else:
    from peer_replay_trace_analysis import (
        MAX_SEQUENCE,
        analyze,
        read_events,
        validate_transmissions,
    )

SHA256_PATTERN = re.compile(r"[a-fA-F0-9]{64}\Z")
MAX_PLANNED_PACKETS = 1_000_000


def required_text(data: dict, key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing nonempty string: {key}")
    return value.strip()


def verify_input(path: Path, expected: str) -> str:
    if not SHA256_PATTERN.fullmatch(expected):
        raise ValueError("expected raw SHA-256 must contain exactly 64 hex digits")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    observed = digest.hexdigest()
    if not hmac.compare_digest(observed, expected.lower()):
        raise ValueError(f"{path.name}: raw capture SHA-256 mismatch")
    return observed


def planned_packet_ids(spec: dict, epoch: int) -> set[tuple[int, int, int]]:
    plans = spec.get("planned_senders")
    if not isinstance(plans, list) or not plans:
        raise ValueError("planned_senders must be a nonempty list")
    expected = set()
    total = 0
    for index, entry in enumerate(plans):
        if not isinstance(entry, dict):
            raise ValueError("each sender plan must be an object")
        sender = entry.get("sender_id")
        first = entry.get("sequence_first")
        last = entry.get("sequence_last")
        if any(type(value) is not int for value in (sender, first, last)):
            raise ValueError("planned sender and sequence values must be integers")
        if not 1 <= sender <= MAX_SEQUENCE:
            raise ValueError(f"sender plan {index}: invalid sender ID")
        if not 0 <= first <= last <= MAX_SEQUENCE:
            raise ValueError(f"sender plan {index}: invalid sequence range")
        count = last - first + 1
        total += count
        if total > MAX_PLANNED_PACKETS:
            raise ValueError("planned packet count exceeds supported audit limit")
        for sequence in range(first, last + 1):
            identity = (sender, epoch, sequence)
            if identity in expected:
                raise ValueError("overlapping sender sequence ranges in packet plan")
            expected.add(identity)
    return expected


def audit(spec_path: Path) -> dict:
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    if not isinstance(spec, dict):
        raise ValueError("capture plan must be a JSON object")

    flight = required_text(spec, "flight_id")
    protocol = required_text(spec, "protocol_reference")
    source = required_text(spec, "source_doi")
    epoch = spec.get("epoch")
    if type(epoch) is not int or not 0 <= epoch <= MAX_SEQUENCE:
        raise ValueError("epoch must be a nonnegative uint32 integer")

    tx = spec.get("tx")
    rx = spec.get("rx")
    if not isinstance(tx, dict) or not isinstance(rx, dict):
        raise ValueError("tx and rx must describe two separate raw captures")
    tx_path = (spec_path.parent / required_text(tx, "file")).resolve()
    rx_path = (spec_path.parent / required_text(rx, "file")).resolve()
    if tx_path == rx_path:
        raise ValueError("sender TX and receiver RX files must be separate")

    tx_hash = verify_input(tx_path, required_text(tx, "sha256"))
    rx_hash = verify_input(rx_path, required_text(rx, "sha256"))
    if tx_hash == rx_hash:
        raise ValueError("TX and RX contents cannot be identical")

    expected = planned_packet_ids(spec, epoch)
    sent = read_events(tx_path, "tx_ms", epoch)
    received = read_events(rx_path, "rx_ms", epoch)
    sent_ids = validate_transmissions(sent)
    missing = expected - sent_ids
    extra = sent_ids - expected
    if missing or extra:
        raise ValueError(
            "TX log differs from the separately declared packet plan: "
            f"{len(missing)} missing, {len(extra)} unplanned identities"
        )
    # The existing analyzer fails closed on any RX packet not present in TX.
    outcome = analyze(sent, received, source="measured_unverified")
    return {
        "schema": "uav-capture-plan-audit-v1",
        "classification": "supplied paired capture checked against declared plan",
        "flight_id": flight,
        "source_doi_claimed": source,
        "protocol_reference_claimed": protocol,
        "epoch": epoch,
        "plan_expected_packet_count": len(expected),
        "sender_log_packet_count": len(sent),
        "plan_coverage_verified": True,
        "original_input_sha256_verified": True,
        "capture_provenance_independently_verified": False,
        "protocol_was_preregistered_verified": False,
        "physical_rf_origin_verified": False,
        "rf_layer_loss_attribution_supported": False,
        "transmitter_instrumentation_independently_verified": False,
        "input_sha256": {"tx": tx_hash, "rx": rx_hash},
        "policy_analysis": outcome,
        "limitations": [
            "Plan authenticity and creation before the trial are not verified.",
            "A complete sequence interval does not prove physical transmissions.",
            "Observed missing receptions do not establish RF-layer loss causes.",
            "A declared source DOI and checksum do not certify archive provenance.",
            "Requires independent sensor, radio and mobility evidence for FANET claims.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = audit(args.plan)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
