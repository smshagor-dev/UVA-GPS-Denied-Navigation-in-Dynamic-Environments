"""Offline synthetic fixtures for EuRoC sensor intake (not benchmark evidence)."""

import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from research.experiments.euroc_sensor_intake import collect, export


NS0 = 1403636579763555584


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerows(rows)


def make_fixture(root):
    mav = root / "mav0"
    imu = mav / "imu0"
    imu.mkdir(parents=True)
    (imu / "sensor.yaml").write_text("sensor_type: imu\n", encoding="utf-8")
    write_rows(
        imu / "data.csv",
        [
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ],
            [NS0, 0.1, 0.2, 0.3, 0, 0, 9.81],
            [NS0 + 1_000_000, 0.1, 0.2, 0.3, 0, 0, 9.81],
        ],
    )
    for camera in ("cam0", "cam1"):
        directory = mav / camera
        (directory / "data").mkdir(parents=True)
        (directory / "sensor.yaml").write_text(
            "sensor_type: camera\n", encoding="utf-8"
        )
        name = f"{NS0}.png"
        (directory / "data" / name).write_bytes(b"offline-fixture-not-a-real-png")
        write_rows(
            directory / "data.csv",
            [["#timestamp [ns]", "filename"], [NS0, name]],
        )


class EurocSensorIntakeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / "unpacked_sequence"
        self.root.mkdir()
        make_fixture(self.root)

    def test_original_sensor_stream_and_no_ground_truth_leak(self):
        # Even an intentionally broken GT file must never be parsed.
        gt = self.root / "mav0" / "state_groundtruth_estimate0"
        gt.mkdir()
        (gt / "data.csv").write_text("invalid ground truth\n", encoding="utf-8")
        events, manifest = collect(self.root, hash_images=True)
        self.assertEqual(len(events), 4)
        self.assertEqual(
            [e["sensor"] for e in events], ["imu0", "cam0", "cam1", "imu0"]
        )
        self.assertEqual(manifest["stereo_exact_timestamp_pairs"], 1)
        self.assertFalse(manifest["ground_truth_used"])
        self.assertFalse(manifest["dataset_provenance_verified"])
        self.assertIn("image_sha256", events[1])
        self.assertFalse(any("pose" in key for e in events for key in e))
        self.assertEqual(events[0]["gyro_rad_s"], [0.1, 0.2, 0.3])

    def test_export_checksums_and_no_overwrite(self):
        out = Path(self.tmp.name) / "export"
        event_path, manifest_path = out / "events.jsonl", out / "manifest.json"
        result = export(self.root, event_path, manifest_path)
        data = event_path.read_bytes()
        self.assertEqual(
            hashlib.sha256(data).hexdigest(), result["event_stream_sha256"]
        )
        self.assertEqual(json.loads(manifest_path.read_text())["total_events"], 4)
        self.assertFalse(result["images_hashed"])
        self.assertNotIn("image_sha256", data.decode())
        with self.assertRaisesRegex(ValueError, "overwrite"):
            export(self.root, event_path, manifest_path)

    def test_duplicate_imu_timestamps_rejected(self):
        path = self.root / "mav0" / "imu0" / "data.csv"
        with path.open("a", encoding="utf-8") as stream:
            stream.write(f"{NS0 + 1_000_000},0,0,0,0,0,9.81\n")
        with self.assertRaisesRegex(ValueError, "non-increasing"):
            collect(self.root)

    def test_nonfinite_imu_rejected(self):
        path = self.root / "mav0" / "imu0" / "data.csv"
        text = path.read_text().replace("0.1,0.2,0.3", "nan,0.2,0.3", 1)
        path.write_text(text, encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            collect(self.root)

    def test_missing_camera_file_rejected(self):
        (self.root / "mav0" / "cam0" / "data" / f"{NS0}.png").unlink()
        with self.assertRaises((FileNotFoundError, ValueError)):
            collect(self.root)

    def test_parent_traversal_camera_filename_rejected(self):
        path = self.root / "mav0" / "cam0" / "data.csv"
        write_rows(path, [["#timestamp [ns]", "filename"], [NS0, "../bad.png"]])
        with self.assertRaisesRegex(ValueError, "unsafe camera filename"):
            collect(self.root)

    def test_output_inside_original_dataset_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            export(
                self.root,
                self.root / "events.jsonl",
                Path(self.tmp.name) / "manifest.json",
            )

    def test_camera_timestamp_mismatch_is_reported_not_hidden(self):
        folder = self.root / "mav0" / "cam1"
        previous = folder / "data" / f"{NS0}.png"
        name = f"{NS0 + 1_000_000}.png"
        previous.rename(folder / "data" / name)
        write_rows(
            folder / "data.csv",
            [["#timestamp [ns]", "filename"], [NS0 + 1_000_000, name]],
        )
        events, manifest = collect(self.root)
        self.assertEqual(len(events), 4)
        self.assertEqual(manifest["stereo_exact_timestamp_pairs"], 0)
        self.assertEqual(manifest["cam0_unpaired"], 1)
        self.assertEqual(manifest["cam1_unpaired"], 1)


if __name__ == "__main__":
    unittest.main()
