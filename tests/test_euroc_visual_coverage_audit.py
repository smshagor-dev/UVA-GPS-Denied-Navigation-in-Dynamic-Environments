"""Synthetic-only integrity and denominator regression for EuRoC coverage audit."""

import hashlib
import json
import unittest
from copy import deepcopy

from research.experiments.euroc_visual_coverage_audit import audit_coverage


def fixture(total=5, statuses=("sync_missing_cam1", "anchor", "visual_pose")):
    ledger = [
        {"timestamp_ns": 1_000_000 + i * 10_000_000, "status": status}
        for i, status in enumerate(statuses)
    ]
    counts = {key: statuses.count(key) for key in set(statuses)}
    return {
        "frame_audit": ledger,
        "frame_audit_count": len(ledger),
        "frame_audit_sha256": hashlib.sha256(
            json.dumps(ledger, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "camera_timestamps_total": total,
        "camera_timestamps_examined": len(ledger),
        "visual_measurement_count": statuses.count("visual_pose"),
        "visual_status_counts": counts,
        "frame_limit_truncated": len(ledger) < total,
        "tracking_failure_truncated": False,
        "failed_closed_on_tracking_loss": False,
    }


class VisualCoverageAuditTests(unittest.TestCase):
    def test_partial_frame_limit_denominator_is_full_camera_sequence(self):
        report = audit_coverage(fixture())
        self.assertEqual(report["visual_updates_accepted"], 1)
        self.assertEqual(report["camera_frames_total"], 5)
        self.assertAlmostEqual(report["visual_update_fraction_of_all_camera_frames"], 0.2)
        self.assertAlmostEqual(report["visual_update_fraction_of_examined_frames"], 1 / 3)
        self.assertFalse(report["complete_camera_sequence_examined"])
        self.assertFalse(report["ground_truth_used"])

    def test_full_coverage_is_not_accuracy(self):
        data = fixture(total=3)
        data["frame_limit_truncated"] = False
        report = audit_coverage(data)
        self.assertTrue(report["complete_camera_sequence_examined"])
        self.assertEqual(report["scope"], "visual_tracking_coverage_not_accuracy")

    def test_terminal_failure_is_reported_not_hidden(self):
        data = fixture(total=5, statuses=("anchor", "visual_pose", "failed_temporal_tracking"))
        data["frame_limit_truncated"] = False
        data["tracking_failure_truncated"] = True
        data["failed_closed_on_tracking_loss"] = True
        report = audit_coverage(data)
        self.assertEqual(report["terminal_failure_count"], 1)
        self.assertEqual(report["terminal_failure_status"], "failed_temporal_tracking")
        self.assertFalse(report["complete_camera_sequence_examined"])

    def test_sync_discontinuity_preserves_dual_status_accounting(self):
        data = fixture(
            total=5,
            statuses=("anchor", "visual_pose", "failed_sync_discontinuity"),
        )
        data["frame_audit"][-1]["sync_rejection"] = "missing_cam1"
        data["frame_audit_sha256"] = hashlib.sha256(
            json.dumps(
                data["frame_audit"], sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        data["visual_status_counts"] = {
            "anchor": 1,
            "visual_pose": 1,
            "sync_missing_cam1": 1,
            "failed_sync_discontinuity": 1,
        }
        data["frame_limit_truncated"] = False
        data["tracking_failure_truncated"] = True
        data["failed_closed_on_tracking_loss"] = True
        report = audit_coverage(data)
        self.assertEqual(report["terminal_failure_status"], "failed_sync_discontinuity")
        self.assertEqual(report["pre_anchor_or_sync_rejections"], 1)

    def test_invalid_ledger_and_missing_truncation_fail_closed(self):
        data = fixture()
        variants = []
        wrong_digest = deepcopy(data)
        wrong_digest["frame_audit_sha256"] = "0" * 64
        variants.append(wrong_digest)
        duplicated = deepcopy(data)
        duplicated["frame_audit"][1]["timestamp_ns"] = duplicated["frame_audit"][0]["timestamp_ns"]
        variants.append(duplicated)
        missing_flag = deepcopy(data)
        missing_flag["frame_limit_truncated"] = False
        variants.append(missing_flag)
        wrong_count = deepcopy(data)
        wrong_count["visual_measurement_count"] = 2
        variants.append(wrong_count)
        false_success = deepcopy(data)
        false_success["frame_audit"][-1]["status"] = "failed_temporal_tracking"
        false_success["visual_status_counts"] = {
            "sync_missing_cam1": 1,
            "anchor": 1,
            "failed_temporal_tracking": 1,
        }
        false_success["visual_measurement_count"] = 0
        variants.append(false_success)
        for item in variants:
            with self.subTest(item=item):
                with self.assertRaises(ValueError):
                    audit_coverage(item)


if __name__ == "__main__":
    unittest.main()
