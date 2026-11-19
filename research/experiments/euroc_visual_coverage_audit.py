"""Independent EuRoC visual tracking-coverage audit; no ground truth or accuracy metrics."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def audit_coverage(manifest: dict) -> dict:
    """Account for every examined camera timestamp, including rejected frames."""
    ledger = manifest["frame_audit"]
    total = manifest["camera_timestamps_total"]
    examined = manifest["camera_timestamps_examined"]
    accepted = manifest["visual_measurement_count"]
    require(type(total) is int and total > 0, "invalid total camera frame count")
    require(
        type(examined) is int and 0 < examined <= total,
        "invalid examined camera count",
    )
    require(
        type(accepted) is int and 0 <= accepted <= examined, "invalid accepted count"
    )
    require(
        isinstance(ledger, list)
        and len(ledger) == examined == manifest["frame_audit_count"],
        "missing or incomplete frame audit",
    )
    encoded = json.dumps(ledger, sort_keys=True, separators=(",", ":")).encode()
    require(
        hashlib.sha256(encoded).hexdigest() == manifest["frame_audit_sha256"],
        "frame audit digest mismatch",
    )
    counts = Counter()
    timestamps = []
    for row in ledger:
        require(isinstance(row, dict), "invalid frame audit row")
        stamp = row.get("timestamp_ns")
        status = row.get("status")
        require(type(stamp) is int and stamp >= 0, "invalid camera timestamp")
        require(
            not timestamps or stamp > timestamps[-1],
            "camera timestamps must be strictly increasing",
        )
        require(
            status in ("anchor", "visual_pose")
            or (isinstance(status, str) and status.startswith("sync_"))
            or (isinstance(status, str) and status.startswith("failed_")),
            "unknown camera audit status",
        )
        timestamps.append(stamp)
        counts[status] += 1
        if status == "failed_sync_discontinuity":
            rejection = row.get("sync_rejection")
            require(
                isinstance(rejection, str) and bool(rejection),
                "missing synchronization rejection reason",
            )
            # The producer counts the underlying sync rejection and the
            # terminal discontinuity separately for this single frame.
            counts["sync_" + rejection] += 1
    require(
        dict(counts)
        == {key: val for key, val in manifest["visual_status_counts"].items() if val},
        "frame status counts inconsistent with ledger",
    )
    require(counts["visual_pose"] == accepted, "accepted count mismatch")
    failures = [
        i for i, row in enumerate(ledger) if row["status"].startswith("failed_")
    ]
    require(len(failures) <= 1, "more than one terminal tracking failure")
    require(
        not failures or failures[0] == examined - 1, "failure must terminate tracking"
    )
    if accepted:
        require(counts["anchor"] == 1, "accepted measurements need exactly one anchor")
    require(counts["anchor"] <= 1, "multiple camera anchors")
    failed = bool(failures)
    limited = manifest["frame_limit_truncated"]
    failure_truncated = manifest["tracking_failure_truncated"]
    require(
        type(limited) is bool and type(failure_truncated) is bool,
        "invalid truncation flags",
    )
    require(
        manifest["failed_closed_on_tracking_loss"] is failed,
        "tracking-failure declaration inconsistent with ledger",
    )
    require(not (failed and limited), "tracking failure mislabeled as frame limit")
    require(
        not failure_truncated or (failed and examined < total),
        "invalid tracking failure truncation",
    )
    require(
        not limited or (examined < total and not failed),
        "invalid frame-limit truncation",
    )
    require(
        examined == total or limited or failed,
        "incomplete sequence missing truncation disclosure",
    )
    return {
        "schema_version": 1,
        "scope": "visual_tracking_coverage_not_accuracy",
        "dataset_provenance_verified": False,
        "ground_truth_used": False,
        "camera_frames_total": total,
        "camera_frames_examined": examined,
        "visual_updates_accepted": accepted,
        "anchor_frames": counts["anchor"],
        "pre_anchor_or_sync_rejections": sum(
            value for key, value in counts.items() if key.startswith("sync_")
        ),
        "terminal_failure_count": len(failures),
        "terminal_failure_status": ledger[-1]["status"] if failed else None,
        "frame_limit_truncated": limited,
        "tracking_failure_truncated": failure_truncated,
        "complete_camera_sequence_examined": examined == total,
        "visual_update_fraction_of_all_camera_frames": accepted / total,
        "visual_update_fraction_of_examined_frames": accepted / examined,
        "examined_fraction_of_all_camera_frames": examined / total,
        "frame_audit_sha256": manifest["frame_audit_sha256"],
        "limitations": [
            "a visual update fraction is not a localization-accuracy metric",
            "a truncated run must not be reported as full-sequence performance",
            "source dataset and publisher ground truth were not independently verified",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--visual-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    require(
        args.visual_manifest.is_file() and not args.visual_manifest.is_symlink(),
        "missing or symlinked visual manifest",
    )
    require(
        not args.output.exists() and not args.output.is_symlink(),
        "refusing to overwrite existing audit output",
    )
    report = audit_coverage(
        json.loads(args.visual_manifest.read_text(encoding="utf-8"))
    )
    report["source_visual_manifest_sha256"] = hashlib.sha256(
        args.visual_manifest.read_bytes()
    ).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as writer:
        json.dump(report, writer, indent=2, sort_keys=True)
        writer.write("\n")
    print(
        f"{report['visual_updates_accepted']}/{report['camera_frames_total']} camera frames "
        "yielded VO updates (coverage only, not accuracy)."
    )


if __name__ == "__main__":
    main()
