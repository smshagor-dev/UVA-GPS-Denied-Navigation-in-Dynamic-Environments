#!/usr/bin/env python3
"""Compare strict monotonic and 64-position replay admission on the same trace.

This analyzes delivery/sequence policies only: all input packets are assumed to
be authentic and timely. It does NOT verify MACs, model radio propagation, or
infer real flight-channel statistics from synthetic examples.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

MAX_SEQUENCE = 2**32 - 1
WINDOW_SIZE = 64
MASK = (1 << WINDOW_SIZE) - 1


@dataclass(frozen=True)
class Event:
    sender_id: int
    epoch: int
    sequence: int
    time_ms: int
    line: int

    @property
    def identity(self) -> tuple[int, int, int]:
        return (self.sender_id, self.epoch, self.sequence)


def read_events(path: Path, time_column: str, epoch: int) -> list[Event]:
    required = {"sender_id", "epoch", "sequence", time_column}
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path}: required columns: {sorted(required)}")
        result = []
        for line_number, row in enumerate(reader, start=2):
            try:
                sender = int(row["sender_id"])
                actual_epoch = int(row["epoch"])
                sequence = int(row["sequence"])
                observed_ms = int(row[time_column])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{path}:{line_number}: integer columns required"
                ) from exc
            if not (1 <= sender <= MAX_SEQUENCE):
                raise ValueError(f"{path}:{line_number}: invalid sender ID")
            if actual_epoch != epoch:
                raise ValueError(
                    f"{path}:{line_number}: epoch {actual_epoch} != selected {epoch}; "
                    "analyze each epoch separately"
                )
            if not (0 <= sequence <= MAX_SEQUENCE) or observed_ms < 0:
                raise ValueError(f"{path}:{line_number}: invalid sequence/time")
            result.append(
                Event(sender, actual_epoch, sequence, observed_ms, line_number)
            )
    if not result:
        raise ValueError(f"{path}: empty trace")
    return sorted(result, key=lambda event: (event.time_ms, event.line))


def validate_transmissions(sent: list[Event]) -> set[tuple[int, int, int]]:
    identifiers = set()
    watermarks: dict[int, int] = {}
    for frame in sent:
        if frame.identity in identifiers:
            raise ValueError(
                f"transmission line {frame.line}: duplicate transmitted identity"
            )
        previous = watermarks.get(frame.sender_id)
        if previous is not None and frame.sequence <= previous:
            raise ValueError(
                f"transmission line {frame.line}: nonmonotonic or wrapped sequence; "
                "analyze a stable sequence context"
            )
        identifiers.add(frame.identity)
        watermarks[frame.sender_id] = frame.sequence
    return identifiers


def analyze(
    sent: list[Event], received: list[Event], *, source: str = "unverified"
) -> dict:
    sent_ids = validate_transmissions(sent)
    unique_received: set[tuple[int, int, int]] = set()
    highest_arrival: dict[int, int] = {}
    strict_high: dict[int, int] = {}
    sliding_state: dict[int, tuple[int, int]] = {}
    stats = Counter()

    for frame in received:
        if frame.identity not in sent_ids:
            raise ValueError(
                f"reception line {frame.line}: no matching transmitted packet; "
                "cannot label its authenticity or origin"
            )
        stats["received_total"] += 1
        duplicate = frame.identity in unique_received
        if duplicate:
            stats["observed_duplicate_deliveries"] += 1
        else:
            unique_received.add(frame.identity)
            stats["received_distinct"] += 1
            past = highest_arrival.get(frame.sender_id)
            if past is not None and frame.sequence < past:
                stats["reordered_distinct"] += 1
            highest_arrival[frame.sender_id] = max(
                frame.sequence, past if past is not None else frame.sequence
            )

        previous = strict_high.get(frame.sender_id)
        strict_accept = previous is None or frame.sequence > previous
        if strict_accept:
            strict_high[frame.sender_id] = frame.sequence
            stats["strict_accepted"] += 1
        elif duplicate:
            stats["strict_duplicate_rejections"] += 1
        else:
            stats["strict_benign_reordering_rejections"] += 1

        state = sliding_state.get(frame.sender_id)
        if state is None:
            sliding_state[frame.sender_id] = (frame.sequence, 1)
            window_accept = True
        else:
            top, bitmap = state
            if frame.sequence > top:
                delta = frame.sequence - top
                updated = 1 if delta >= WINDOW_SIZE else (bitmap << delta) | 1
                sliding_state[frame.sender_id] = (frame.sequence, updated & MASK)
                window_accept = True
            else:
                distance = top - frame.sequence
                if distance >= WINDOW_SIZE:
                    window_accept = False
                else:
                    bit = 1 << distance
                    window_accept = not bool(bitmap & bit)
                    if window_accept:
                        sliding_state[frame.sender_id] = (top, bitmap | bit)

        if window_accept:
            stats["window_accepted"] += 1
        elif duplicate:
            stats["window_duplicate_rejections"] += 1
        else:
            stats["window_benign_outside_window_rejections"] += 1

    total_sent = len(sent_ids)
    unique_count = len(unique_received)
    lost = total_sent - unique_count

    def ratio(a: int, b: int) -> float | None:
        return round(a / b, 8) if b else None

    return {
        "schema": "uav-peer-trace-analysis-v1",
        "source_classification": source,
        "scope": "sequence policy only; authenticated and fresh inputs assumed",
        "trace": {
            "transmitted_distinct": total_sent,
            "received_deliveries": stats["received_total"],
            "received_distinct": unique_count,
            "observed_duplicate_deliveries": stats["observed_duplicate_deliveries"],
            "unobserved_transmissions": lost,
            "observed_loss_fraction": ratio(lost, total_sent),
            "reordered_distinct": stats["reordered_distinct"],
            "observed_p_r": ratio(stats["reordered_distinct"], unique_count),
        },
        "strict": {
            "accepted_deliveries": stats["strict_accepted"],
            "benign_reordering_rejections": stats[
                "strict_benign_reordering_rejections"
            ],
            "duplicate_rejections": stats["strict_duplicate_rejections"],
            "benign_rejection_fraction": ratio(
                stats["strict_benign_reordering_rejections"], unique_count
            ),
        },
        "window_64": {
            "accepted_deliveries": stats["window_accepted"],
            "benign_outside_window_rejections": stats[
                "window_benign_outside_window_rejections"
            ],
            "duplicate_rejections": stats["window_duplicate_rejections"],
            "benign_rejection_fraction": ratio(
                stats["window_benign_outside_window_rejections"], unique_count
            ),
        },
        "limitations": [
            "Requires complete sender-side transmission trace for the measured period.",
            "Observed loss includes unobserved transmissions, not RF-layer loss attribution.",
            "Duplicates are classified by trace identity, not adversarial intent.",
            "Does not verify authentication, timestamps, epoch transitions or disk recovery.",
            "Single receiver and fixed epoch; no sequence rollover or multi-receiver consensus.",
            "Synthetic data cannot establish typical FANET reordering rates.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tx", type=Path, required=True, help="Transmission CSV")
    parser.add_argument("--rx", type=Path, required=True, help="Reception CSV")
    parser.add_argument("--epoch", type=int, default=1)
    parser.add_argument(
        "--source",
        choices=("synthetic", "unverified", "measured_unverified"),
        default="unverified",
    )
    parser.add_argument("--output", type=Path, help="Optional JSON report path")
    args = parser.parse_args()
    if args.epoch < 0:
        parser.error("epoch must be nonnegative")
    try:
        result = analyze(
            read_events(args.tx, "tx_ms", args.epoch),
            read_events(args.rx, "rx_ms", args.epoch),
            source=args.source,
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    report = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
