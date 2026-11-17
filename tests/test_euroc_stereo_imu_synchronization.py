"""Synthetic-only tests for stereo/IMU synchronization and drop accounting."""

import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import yaml

from research.experiments.euroc_stereo_imu_synchronization import (
    export,
    synchronize,
)
from tests.test_euroc_calibration_validation import calibrations
from tests.test_euroc_sensor_intake import make_fixture, write_rows


NS0 = 1403636579763555584
STEP_NS = 5_000_000
FRAMES = (0, 20_000_000, 40_000_000)


def make_sequence(root):
    make_fixture(root)
    for sensor, fields in calibrations().items():
        yaml_path = root / "mav0" / sensor / "sensor.yaml"
        yaml_path.write_text(yaml.safe_dump(fields), encoding="utf-8")
    set_imu(root, list(range(0, 50_000_001, STEP_NS)))
    for camera in ("cam0", "cam1"):
        set_camera(root, camera, FRAMES)


def set_imu(root, offsets):
    path = root / "mav0" / "imu0" / "data.csv"
    header = [
        "#timestamp [ns]",
        "w_RS_S_x [rad s^-1]",
        "w_RS_S_y [rad s^-1]",
        "w_RS_S_z [rad s^-1]",
        "a_RS_S_x [m s^-2]",
        "a_RS_S_y [m s^-2]",
        "a_RS_S_z [m s^-2]",
    ]
    write_rows(
        path,
        [header] + [
            [NS0 + offset, 0, 0, 0, 0, 0, 9.81]
            for offset in offsets
        ],
    )


def set_camera(root, name, offsets):
    folder = root / "mav0" / name
    rows = [["#timestamp [ns]", "filename"]]
    for offset in offsets:
        timestamp = NS0 + offset
        filename = f"{timestamp}.png"
        (folder / "data" / filename).write_bytes(
            b"offline-synthetic-intake-fixture"
        )
        rows.append([timestamp, filename])
    write_rows(folder / "data.csv", rows)


class EurocStereoImuSynchronizationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "synthetic_sequence"
        self.root.mkdir()
        make_sequence(self.root)

    def test_paired_frames_have_nonoverlapping_original_imu_segments(self):
        records, manifest = synchronize(self.root)
        self.assertEqual([r["status"] for r in records], [
            "anchor", "ready", "ready"
        ])
        self.assertEqual(manifest["ready_intervals"], 2)
        self.assertEqual(manifest["rejected_frame_count"], 0)
        self.assertEqual(manifest["imu_sample_count"], 11)
        first = [s["timestamp_ns"] for s in records[1]["imu_interframe"]]
        second = [s["timestamp_ns"] for s in records[2]["imu_interframe"]]
        self.assertEqual(first, [NS0 + n for n in range(5_000_000, 20_000_001, STEP_NS)])
        self.assertEqual(second, [NS0 + n for n in range(25_000_000, 40_000_001, STEP_NS)])
        self.assertTrue(set(first).isdisjoint(second))
        self.assertFalse(manifest["ground_truth_used"])
        self.assertFalse(manifest["world_pose_estimated"])
        self.assertFalse(manifest["dataset_provenance_verified"])

    def test_missing_second_camera_is_recorded_and_imu_is_not_dropped(self):
        set_camera(self.root, "cam1", (0, 40_000_000))
        records, manifest = synchronize(self.root)
        self.assertEqual([r["status"] for r in records],
                         ["anchor", "missing_cam1", "ready"])
        self.assertEqual(manifest["rejected_frame_count"], 1)
        self.assertEqual(
            len(records[-1]["imu_interframe"]), 8
        )
        self.assertEqual(records[1]["imu_interframe"], [])

    def test_no_imu_before_is_explicit(self):
        set_imu(self.root, list(range(5_000_000, 50_000_001, STEP_NS)))
        records, manifest = synchronize(self.root)
        self.assertEqual(records[0]["status"], "no_imu_before")
        self.assertEqual(records[1]["status"], "anchor")
        self.assertEqual(records[2]["status"], "ready")
        self.assertEqual(manifest["rejected_frame_count"], 1)

    def test_no_imu_after_is_explicit(self):
        set_imu(self.root, list(range(0, 35_000_001, STEP_NS)))
        records, _ = synchronize(self.root)
        self.assertEqual(records[2]["status"], "no_imu_after")

    def test_imu_gap_at_stereo_timestamp_is_not_silently_interpolated(self):
        set_imu(
            self.root,
            [0, 5_000_000, 10_000_000, 35_000_000,
             40_000_000, 45_000_000, 50_000_000],
        )
        records, manifest = synchronize(
            self.root, max_bracket_ns=25_000_000,
            max_imu_gap_ns=12_000_000
        )
        self.assertEqual(records[1]["status"], "imu_bracket_gap")
        self.assertEqual(records[2]["status"], "imu_interframe_gap")
        self.assertEqual(manifest["rejected_frame_count"], 2)

    def test_no_interframe_samples_is_rejected(self):
        set_camera(self.root, "cam0", (0, 1_000_000, 20_000_000, 40_000_000))
        set_camera(self.root, "cam1", (0, 1_000_000, 20_000_000, 40_000_000))
        records, _ = synchronize(self.root)
        self.assertEqual(records[1]["status"], "no_imu_between_frames")
        self.assertEqual(records[2]["status"], "ready")

    def test_unpaired_initial_frame_does_not_claim_anchor(self):
        set_camera(self.root, "cam0", FRAMES[1:])
        records, _ = synchronize(self.root)
        self.assertEqual(records[0]["status"], "missing_cam0")
        self.assertEqual(records[1]["status"], "anchor")

    def test_ground_truth_cannot_influence_event_stream(self):
        original, _ = synchronize(self.root)
        gt = self.root / "mav0" / "state_groundtruth_estimate0"
        gt.mkdir()
        (gt / "data.csv").write_text("malformed GT!\n", encoding="utf-8")
        extra, _ = synchronize(self.root)
        self.assertEqual(original, extra)

    def test_hashed_export_has_verified_content_digest(self):
        events_path = Path(self.temp.name) / "exports" / "events.jsonl"
        manifest_path = Path(self.temp.name) / "exports" / "manifest.json"
        manifest = export(
            self.root, events_path, manifest_path, hash_images=True
        )
        lines = [json.loads(line) for line in events_path.read_text().splitlines()]
        self.assertEqual(len(lines), 3)
        self.assertEqual(
            manifest["frame_stream_sha256"],
            hashlib.sha256(events_path.read_bytes()).hexdigest(),
        )
        self.assertIn("cam0", lines[1]["source_image_sha256"])
        self.assertEqual(
            json.loads(manifest_path.read_text())["ready_intervals"], 2
        )
        with self.assertRaisesRegex(ValueError, "overwrite"):
            export(self.root, events_path, manifest_path)

    def test_output_inside_dataset_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            export(
                self.root,
                self.root / "unsafe.jsonl",
                Path(self.temp.name) / "manifest.json",
            )

    def test_invalid_synchronization_thresholds_rejected(self):
        for params in (
            {"max_bracket_ns": 0},
            {"max_bracket_ns": -1},
            {"max_bracket_ns": True},
            {"max_imu_gap_ns": 0},
            {"max_imu_gap_ns": 1.2},
        ):
            with self.subTest(params=params):
                with self.assertRaisesRegex(ValueError, "positive integer"):
                    synchronize(self.root, **params)


if __name__ == "__main__":
    unittest.main()
