"""Synthetic orchestration and fail-closed evidence manifest regression."""

import csv
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from research.experiments.euroc_vio_run import run


STAMPS = [1403636579800000000, 1403636579820000000]
VISUAL_COLUMNS = [
    "timestamp_ns",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "sigma_position_m",
    "sigma_velocity_mps",
    "pnp_inliers",
    "pnp_reprojection_rmse_px",
]


class EuRoCExperimentRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.source = base / "original_sequence"
        self.source.mkdir()
        self.output = base / "outputs"
        self.binary = Path(sys.executable)

    def export_sync(self, sequence_root, events, manifest, *, hash_images):
        self.assertEqual(sequence_root, self.source)
        self.assertTrue(hash_images)
        events.write_text('{"status":"ready"}\n', encoding="utf-8")
        manifest.write_text('{"scope":"synthetic"}\n', encoding="utf-8")
        return {
            "ready_intervals": 2,
            "rejected_frame_count": 0,
            "status_counts": {"ready": 2},
        }

    def export_visual(
        self, sequence_root, visual_csv, manifest, *, max_frames, min_inliers
    ):
        self.assertEqual(sequence_root, self.source)
        self.assertGreaterEqual(min_inliers, 8)
        with visual_csv.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(VISUAL_COLUMNS)
            for stamp in STAMPS:
                writer.writerow([stamp, 0, 0, 0, 0, 0, 0, 0.5, 0.75, 25, 0.25])
        manifest.write_text('{"scope":"synthetic"}\n', encoding="utf-8")
        audit = [{"timestamp_ns": stamp, "status": "visual_pose"} for stamp in STAMPS]
        return {
            "visual_measurement_count": 2,
            "frame_limit_truncated": max_frames > 0,
            "tracking_failure_truncated": False,
            "failed_closed_on_tracking_loss": False,
            "visual_status_counts": {"visual_pose": 2},
            "frame_audit": audit,
            "frame_audit_count": len(audit),
            "camera_timestamps_examined": len(audit),
            "frame_audit_sha256": hashlib.sha256(
                json.dumps(audit, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest(),
        }

    def native(self, argv, *, capture_output, text, check, timeout):
        self.assertTrue(capture_output)
        self.assertTrue(text)
        self.assertFalse(check)
        self.assertGreater(timeout, 0)
        active = Path(argv[argv.index("--active-output") + 1])
        shadow = Path(argv[argv.index("--shadow-output") + 1])
        for file in (active, shadow):
            with file.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["timestamp_ns", "x_m", "y_m", "z_m"])
                for stamp in STAMPS:
                    writer.writerow([stamp, 0, 0, 0])
        return subprocess.CompletedProcess(
            argv,
            0,
            stdout="IMU: 12 original rows; 1 interpolated camera-time rows; "
            "2 visual updates accepted by both filters.\n",
            stderr="",
        )

    def patches(self, native=None):
        return (
            mock.patch(
                "research.experiments.euroc_vio_run.export_sync",
                side_effect=self.export_sync,
            ),
            mock.patch(
                "research.experiments.euroc_vio_run.export_visual",
                side_effect=self.export_visual,
            ),
            mock.patch(
                "research.experiments.euroc_vio_run.subprocess.run",
                side_effect=native or self.native,
            ),
        )

    def test_success_retains_input_and_trajectory_hashes_without_accuracy_claim(self):
        patch_sync, patch_visual, patch_native = self.patches()
        with patch_sync, patch_visual, patch_native:
            result = run(self.source, self.output, self.binary)
        self.assertEqual(result["status"], "completed_unverified_software_replay")
        self.assertFalse(result["dataset_provenance_verified"])
        self.assertFalse(result["ground_truth_used"])
        self.assertFalse(result["publishable_accuracy_evidence"])
        self.assertEqual(result["native_counts"]["visual_updates"], 2)
        self.assertEqual(result["visual"]["frame_audit_count"], 2)
        self.assertEqual(len(result["visual"]["frame_audit_sha256"]), 64)
        self.assertEqual(result["native_active"]["sample_count"], 2)
        self.assertEqual(result["native_shadow"]["sample_count"], 2)
        self.assertEqual(
            json.loads((self.output / "run_manifest.json").read_text())["status"],
            "completed_unverified_software_replay",
        )

    def test_explicit_frame_limit_is_labelled_partial_not_complete(self):
        patch_sync, patch_visual, patch_native = self.patches()
        with patch_sync, patch_visual, patch_native:
            result = run(self.source, self.output, self.binary, max_frames=2)
        self.assertEqual(
            result["status"], "completed_unverified_partial_software_replay"
        )
        self.assertEqual(result["frame_limit"], 2)
        self.assertTrue(result["visual"]["frame_limit_truncated"])
        self.assertFalse(result["publishable_accuracy_evidence"])

    def test_native_process_rejection_leaves_failure_manifest(self):
        def failed(argv, **kwargs):
            return subprocess.CompletedProcess(argv, 1, stdout="", stderr="rejected")

        patch_sync, patch_visual, patch_native = self.patches(native=failed)
        with patch_sync, patch_visual, patch_native:
            with self.assertRaisesRegex(ValueError, "native C\\+\\+ estimator"):
                run(self.source, self.output, self.binary)
        result = json.loads((self.output / "run_manifest.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["native_exit_code"], 1)
        self.assertFalse(result["publishable_accuracy_evidence"])

    def test_native_trajectory_timestamp_mismatch_fails_closed(self):
        def mismatch(argv, **kwargs):
            done = self.native(argv, **kwargs)
            path = Path(argv[argv.index("--shadow-output") + 1])
            path.write_text(
                "timestamp_ns,x_m,y_m,z_m\n" f"{STAMPS[0]},0,0,0\n",
                encoding="utf-8",
            )
            return done

        patch_sync, patch_visual, patch_native = self.patches(native=mismatch)
        with patch_sync, patch_visual, patch_native:
            with self.assertRaisesRegex(ValueError, "lost, duplicated or reordered"):
                run(self.source, self.output, self.binary)
        self.assertEqual(
            json.loads((self.output / "run_manifest.json").read_text())["status"],
            "failed",
        )

    def test_visual_frame_audit_mismatch_blocks_native_estimator(self):
        original = self.export_visual

        def forged(*args, **kwargs):
            result = original(*args, **kwargs)
            result["frame_audit"][1]["timestamp_ns"] += 1_000_000
            result["frame_audit_sha256"] = hashlib.sha256(
                json.dumps(
                    result["frame_audit"],
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode()
            ).hexdigest()
            return result

        with (
            mock.patch(
                "research.experiments.euroc_vio_run.export_sync",
                side_effect=self.export_sync,
            ),
            mock.patch(
                "research.experiments.euroc_vio_run.export_visual",
                side_effect=forged,
            ),
            mock.patch(
                "research.experiments.euroc_vio_run.subprocess.run",
                side_effect=AssertionError("native called despite mismatched ledger"),
            ) as native,
        ):
            with self.assertRaisesRegex(
                ValueError, "do not match accepted frame audit"
            ):
                run(self.source, self.output, self.binary)
            native.assert_not_called()
        status = json.loads((self.output / "run_manifest.json").read_text())
        self.assertEqual(status["status"], "failed")

    def test_output_directory_inside_source_is_never_created(self):
        forbidden = self.source / "original_data_overwrite"
        with self.assertRaisesRegex(ValueError, "outside"):
            run(self.source, forbidden, self.binary)
        self.assertFalse(forbidden.exists())

    def test_existing_evidence_directory_is_never_overwritten(self):
        self.output.mkdir()
        sentinel = self.output / "previous.json"
        sentinel.write_text("original", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "overwrite"):
            run(self.source, self.output, self.binary)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "original")


if __name__ == "__main__":
    unittest.main()
