"""Prespecified EuRoC time-offset sensitivity, never a GT-tuned clock correction.

Use only independent full-pose estimates. This offline tool compares *both*
native estimators at every fixed clock offset, reports failures, and never
selects the offset with the smallest ground-truth error.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from decimal import Decimal, InvalidOperation
from pathlib import Path

from research.experiments.euroc_pose_evaluation import (
    Pose,
    evaluate,
    load_estimated,
    load_ground_truth,
    validate_order,
)

DEFAULT_OFFSETS_MS = "-20,-10,0,10,20"


def require(condition: bool, explanation: str) -> None:
    if not condition:
        raise ValueError(explanation)


def parse_offsets_ms(specification: str) -> list[int]:
    """Strict ordered, two-sided, predeclared millisecond grid in nanoseconds."""
    cells = specification.split(",")
    require(3 <= len(cells) <= 21, "offset grid requires 3 to 21 entries")
    offsets: list[int] = []
    for cell in cells:
        require(bool(cell.strip()), "empty time-offset entry")
        try:
            ms = Decimal(cell.strip())
        except InvalidOperation as exc:
            raise ValueError("invalid decimal millisecond offset") from exc
        require(ms.is_finite() and abs(ms) <= 50, "time offset beyond +/-50 ms")
        nanos = ms * 1_000_000
        require(
            nanos == nanos.to_integral_value(),
            "time offsets must be exactly expressible as integer nanoseconds",
        )
        offsets.append(int(nanos))
    require(
        all(left < right for left, right in zip(offsets, offsets[1:])),
        "time-offset grid must be strictly increasing and unique",
    )
    require(
        offsets[0] < 0 < offsets[-1] and 0 in offsets,
        "grid requires both signs and zero",
    )
    return offsets


def digest(path: Path) -> str:
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def compare_offsets(
    ground_truth: list[Pose],
    active: list[Pose],
    shadow: list[Pose],
    offsets_ns: list[int],
    *,
    tolerance_ns: int = 20_000_000,
    min_pairs: int = 10,
    min_overlap_s: float = 5.0,
    relative_lag_s: float = 1.0,
) -> dict:
    validate_order(ground_truth, "ground truth")
    validate_order(active, "active estimate")
    validate_order(shadow, "shadow estimate")
    require(
        [x.timestamp_ns for x in active] == [x.timestamp_ns for x in shadow],
        "active and shadow timestamps differ; refusing unpaired comparison",
    )
    require(
        offsets_ns
        == parse_offsets_ms(
            ",".join(str(Decimal(value) / 1_000_000) for value in offsets_ns)
        ),
        "invalid or unsorted offset specification",
    )
    require(
        type(tolerance_ns) is int and 0 <= tolerance_ns <= 50_000_000,
        "invalid tolerance",
    )
    require(type(min_pairs) is int and min_pairs >= 4, "invalid min_pairs")
    require(
        math.isfinite(min_overlap_s)
        and min_overlap_s > 0
        and math.isfinite(relative_lag_s)
        and relative_lag_s > 0,
        "invalid durations",
    )
    records = []
    for offset_ns in offsets_ns:
        record = {
            "offset_ns": offset_ns,
            "offset_ms": offset_ns / 1_000_000,
            "status": "failed",
        }
        try:
            active_result = evaluate(
                ground_truth,
                active,
                tolerance_ns=tolerance_ns,
                min_pairs=min_pairs,
                min_overlap_s=min_overlap_s,
                relative_lag_s=relative_lag_s,
                time_offset_ns=offset_ns,
            )
            shadow_result = evaluate(
                ground_truth,
                shadow,
                tolerance_ns=tolerance_ns,
                min_pairs=min_pairs,
                min_overlap_s=min_overlap_s,
                relative_lag_s=relative_lag_s,
                time_offset_ns=offset_ns,
            )
        except ValueError as exc:
            record["failure_reason"] = str(exc)
            records.append(record)
            continue
        require(
            active_result["matched_samples"] == shadow_result["matched_samples"],
            "same timestamps unexpectedly produced unmatched pair counts",
        )
        record["status"] = "evaluated"
        record["active"] = active_result
        record["shadow"] = shadow_result
        record["active_minus_shadow"] = {
            "ate_rmse_m": (
                active_result["ate_translation"]["rmse_m"]
                - shadow_result["ate_translation"]["rmse_m"]
            ),
            "absolute_orientation_rmse_deg": (
                active_result["absolute_orientation_error"]["rmse_deg"]
                - shadow_result["absolute_orientation_error"]["rmse_deg"]
            ),
            "relative_translation_rmse_m": (
                active_result["full_se3_relative_pose_error"][
                    "translation_in_first_body_frame"
                ]["rmse_m"]
                - shadow_result["full_se3_relative_pose_error"][
                    "translation_in_first_body_frame"
                ]["rmse_m"]
            ),
            "relative_orientation_rmse_deg": (
                active_result["full_se3_relative_pose_error"]["orientation"]["rmse_deg"]
                - shadow_result["full_se3_relative_pose_error"]["orientation"][
                    "rmse_deg"
                ]
            ),
        }
        records.append(record)
    zero = next(entry for entry in records if entry["offset_ns"] == 0)
    successful = sum(entry["status"] == "evaluated" for entry in records)
    return {
        "scope": "offline_prespecified_clock_offset_sensitivity_not_calibration",
        "grid_predeclared_for_this_run": True,
        "independent_preregistration_verified": False,
        "best_offset_selected": False,
        "ground_truth_used_for_estimator": False,
        "publisher_archive_provenance_verified": False,
        "status": "completed" if successful == len(records) else "incomplete",
        "zero_offset_evaluated": zero["status"] == "evaluated",
        "requested_offsets_ns": offsets_ns,
        "completed_offsets": successful,
        "requested_offsets": len(offsets_ns),
        "association_tolerance_ns": tolerance_ns,
        "min_pairs": min_pairs,
        "min_overlap_s": min_overlap_s,
        "relative_lag_s": relative_lag_s,
        "active_shadow_timestamp_parity": True,
        "records": records,
        "limitations": [
            "clock offsets must be declared before examining publisher ground truth",
            "do not select best offset using these error curves",
            "different offsets can match different numbers of ground-truth poses",
            "only original archive and external provenance verification can validate source",
            "full-window rigid alignment is offline evaluation, not online accuracy",
            "no flight data, physical safety claim, or published method comparison",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", required=True, type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--active-estimated", required=True, type=Path)
    parser.add_argument("--shadow-estimated", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frame-agreement", required=True, choices=("same-imu-body",))
    parser.add_argument("--offsets-ms", default=DEFAULT_OFFSETS_MS)
    parser.add_argument("--tolerance-ms", type=float, default=20.0)
    parser.add_argument("--min-pairs", type=int, default=10)
    parser.add_argument("--min-overlap-s", type=float, default=5.0)
    parser.add_argument("--relative-lag-s", type=float, default=1.0)
    args = parser.parse_args()
    require(bool(args.sequence.strip()), "sequence name required")
    source_root = args.sequence_root.resolve(strict=True)
    require(source_root.is_dir(), "original sequence root must be a directory")
    ground_truth_path = (
        source_root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    )
    require(
        ground_truth_path.is_file() and not ground_truth_path.is_symlink(),
        "original EuRoC publisher ground truth not found",
    )
    active_path = args.active_estimated.resolve(strict=True)
    shadow_path = args.shadow_estimated.resolve(strict=True)
    require(
        active_path.is_file() and shadow_path.is_file(),
        "missing active or shadow trajectory",
    )
    require(
        active_path != shadow_path
        and not active_path.is_relative_to(source_root)
        and not shadow_path.is_relative_to(source_root),
        "independent estimator trajectories must be distinct and outside source",
    )
    output_path = args.output.resolve()
    require(
        not output_path.is_relative_to(source_root),
        "refusing to write evaluation inside original sequence",
    )
    require(
        not output_path.exists() and not args.output.is_symlink(),
        "refusing to overwrite an existing research-evidence artifact",
    )
    require(
        math.isfinite(args.tolerance_ms) and 0 <= args.tolerance_ms <= 50,
        "invalid timestamp tolerance",
    )
    requested_offsets = parse_offsets_ms(args.offsets_ms)
    result = compare_offsets(
        load_ground_truth(ground_truth_path),
        load_estimated(active_path),
        load_estimated(shadow_path),
        requested_offsets,
        tolerance_ns=round(args.tolerance_ms * 1_000_000),
        min_pairs=args.min_pairs,
        min_overlap_s=args.min_overlap_s,
        relative_lag_s=args.relative_lag_s,
    )
    result["sequence"] = args.sequence
    result["frame_agreement_assertion"] = args.frame_agreement
    result["source_sha256"] = {
        "publisher_ground_truth": digest(ground_truth_path),
        "active_estimated": digest(active_path),
        "shadow_estimated": digest(shadow_path),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(
        f"{result['completed_offsets']}/{result['requested_offsets']} "
        f"prespecified offsets evaluated; status={result['status']}"
    )
    if result["status"] != "completed" or not result["zero_offset_evaluated"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
