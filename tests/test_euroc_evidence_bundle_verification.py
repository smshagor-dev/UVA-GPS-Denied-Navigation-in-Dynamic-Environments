"""Synthetic post-run evidence tampering tests; no original EuRoC data."""

import csv
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from research.experiments.euroc_evidence_bundle_verification import (
    POSE_COLUMNS,
    POSITION_COLUMNS,
    VISUAL_COLUMNS,
    sha256,
    verify_bundle,
)


class EvidenceBundleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "run"
        self.root.mkdir()
        self.make_bundle()

    def write_json(self, name, document):
        (self.root / name).write_text(
            json.dumps(document, sort_keys=True) + "\n", encoding="utf-8"
        )

    def read_json(self, name):
        return json.loads((self.root / name).read_text(encoding="utf-8"))

    def write_csv(self, name, columns, rows):
        with (self.root / name).open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)

    def update_declared_sha(self, field, name):
        run = self.read_json("run_manifest.json")
        run[field]["sha256"] = sha256(self.root / name)
        self.write_json("run_manifest.json", run)

    def make_bundle(self):
        stamps = (1_100_000_000, 1_200_000_000)
        self.write_json("synchronization_manifest.json", {"scope": "synthetic"})
        (self.root / "synchronized_frames.jsonl").write_text(
            '{"status":"ready"}\n', encoding="utf-8"
        )
        visual = []
        for i, stamp in enumerate(stamps):
            visual.append(
                {
                    "timestamp_ns": str(stamp),
                    "x_m": str(float(i)),
                    "y_m": "0",
                    "z_m": "0",
                    "vx_mps": "1",
                    "vy_mps": "0",
                    "vz_mps": "0",
                    "sigma_position_m": "0.5",
                    "sigma_velocity_mps": "0.75",
                    "pnp_inliers": "24",
                    "pnp_reprojection_rmse_px": "0.5",
                    "qw": "1",
                    "qx": "0",
                    "qy": "0",
                    "qz": "0",
                    "sigma_orientation_rad": "0.25",
                }
            )
        self.write_csv("visual_odometry.csv", VISUAL_COLUMNS, visual)
        ledger = [
            {"timestamp_ns": 1_000_000_000, "status": "anchor"},
            *[
                {"timestamp_ns": stamp, "status": "visual_pose"}
                for stamp in stamps
            ],
        ]
        visual_manifest = {
            "frame_audit": ledger,
            "frame_audit_count": len(ledger),
            "frame_audit_sha256": hashlib.sha256(
                json.dumps(ledger, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest(),
            "camera_timestamps_total": 3,
            "camera_timestamps_examined": 3,
            "visual_measurement_count": 2,
            "visual_status_counts": {"anchor": 1, "visual_pose": 2},
            "frame_limit_truncated": False,
            "tracking_failure_truncated": False,
            "failed_closed_on_tracking_loss": False,
        }
        self.write_json("visual_manifest.json", visual_manifest)
        run = {
            "status": "completed_unverified_software_replay",
            "ground_truth_used": False,
            "dataset_provenance_verified": False,
            "publishable_accuracy_evidence": False,
            "native_exit_code": 0,
            "synchronization": {
                "manifest_sha256": sha256(self.root / "synchronization_manifest.json"),
                "events_sha256": sha256(self.root / "synchronized_frames.jsonl"),
            },
            "visual": {
                "manifest_sha256": sha256(self.root / "visual_manifest.json"),
                "csv_sha256": sha256(self.root / "visual_odometry.csv"),
                "accepted_count": 2,
                "frame_audit_count": 3,
                "frame_audit_sha256": visual_manifest["frame_audit_sha256"],
                "visual_status_counts": visual_manifest["visual_status_counts"],
                "frame_limit_truncated": False,
                "tracking_failure_truncated": False,
            },
            "native_counts": {"original_imu_rows": 12, "visual_updates": 2},
        }
        for estimator in ("active", "shadow"):
            name = estimator + "_estimated.csv"
            projection = estimator + "_position_only.csv"
            pose_rows = [
                {
                    "timestamp_ns": str(stamp),
                    "x_m": str(float(i)),
                    "y_m": "0",
                    "z_m": "0",
                    "qw": "1",
                    "qx": "0",
                    "qy": "0",
                    "qz": "0",
                }
                for i, stamp in enumerate(stamps)
            ]
            self.write_csv(name, POSE_COLUMNS, pose_rows)
            self.write_csv(
                projection,
                POSITION_COLUMNS,
                [{key: row[key] for key in POSITION_COLUMNS} for row in pose_rows],
            )
            run["native_" + estimator] = {
                "relative_file": name,
                "sample_count": 2,
                "first_timestamp_ns": stamps[0],
                "last_timestamp_ns": stamps[-1],
                "sha256": sha256(self.root / name),
            }
            run[estimator + "_evaluator_positions"] = {
                "relative_file": projection,
                "sample_count": 2,
                "sha256": sha256(self.root / projection),
            }
        self.write_json("run_manifest.json", run)

    def test_consistent_bundle_is_explicitly_not_accuracy_evidence(self):
        report = verify_bundle(self.root)
        self.assertEqual(report["visual_updates"], 2)
        self.assertEqual(report["camera_frames_total"], 3)
        self.assertFalse(report["publisher_archive_provenance_verified"])
        self.assertFalse(report["benchmark_accuracy_established"])

    def test_tampered_visual_csv_is_rejected(self):
        path = self.root / "visual_odometry.csv"
        path.write_text(path.read_text() + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            verify_bundle(self.root)

    def test_cross_file_timestamp_mismatch_rejected_even_if_rehashed(self):
        path = self.root / "active_estimated.csv"
        raw = path.read_text(encoding="utf-8")
        path.write_text(raw.replace("1200000000", "1200000100"), encoding="utf-8")
        self.update_declared_sha("native_active", "active_estimated.csv")
        run = self.read_json("run_manifest.json")
        run["native_active"]["last_timestamp_ns"] = 1_200_000_100
        self.write_json("run_manifest.json", run)
        with self.assertRaisesRegex(ValueError, "timestamps differ"):
            verify_bundle(self.root)

    def test_position_projection_mismatch_rejected_even_if_rehashed(self):
        path = self.root / "shadow_position_only.csv"
        path.write_text(
            path.read_text(encoding="utf-8").replace("1.0", "3.0"),
            encoding="utf-8",
        )
        self.update_declared_sha("shadow_evaluator_positions", "shadow_position_only.csv")
        with self.assertRaisesRegex(ValueError, "position projection differs"):
            verify_bundle(self.root)

    def test_unverified_ground_truth_claim_rejected(self):
        run = self.read_json("run_manifest.json")
        run["ground_truth_used"] = True
        self.write_json("run_manifest.json", run)
        with self.assertRaisesRegex(ValueError, "ground-truth"):
            verify_bundle(self.root)

    def test_truncated_sequence_requires_explicit_opt_in(self):
        visual = self.read_json("visual_manifest.json")
        visual["camera_timestamps_total"] = 4
        visual["frame_limit_truncated"] = True
        self.write_json("visual_manifest.json", visual)
        run = self.read_json("run_manifest.json")
        run["status"] = "completed_unverified_partial_software_replay"
        run["visual"]["manifest_sha256"] = sha256(self.root / "visual_manifest.json")
        run["visual"]["frame_limit_truncated"] = True
        self.write_json("run_manifest.json", run)
        with self.assertRaisesRegex(ValueError, "requires --allow-partial"):
            verify_bundle(self.root)
        report = verify_bundle(self.root, allow_partial=True)
        self.assertTrue(report["frame_limit_truncated"])

    def test_symlinked_artifact_rejected(self):
        path = self.root / "active_estimated.csv"
        original = self.root / "hidden_native_pose.csv"
        path.rename(original)
        path.symlink_to(original.name)
        with self.assertRaisesRegex(ValueError, "symlinked"):
            verify_bundle(self.root)

    def test_cli_never_overwrites_existing_report(self):
        output = self.root.parent / "verification.json"
        output.write_text("existing", encoding="utf-8")
        proc = subprocess.run(
            [
                sys.executable,
                "-m",
                "research.experiments.euroc_evidence_bundle_verification",
                "--bundle-dir",
                str(self.root),
                "--output",
                str(output),
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(output.read_text(encoding="utf-8"), "existing")


if __name__ == "__main__":
    unittest.main()
