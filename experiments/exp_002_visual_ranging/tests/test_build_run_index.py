"""Tests for rebuilding offline calibration pose labels from GPS samples."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from build_run_index import build_index


def write_run(root: Path, name: str, trial: str, z: float, heading: float = 0.0) -> None:
    run_dir = root / name
    run_dir.mkdir()
    (run_dir / "frames.csv").write_text(
        "frame_index,simulation_time_s,gps_valid,gps_x_m,gps_z_m,"
        "compass_valid,heading_rad,image_saved,image_file,trial_id\n"
        f"0,1.024,1,0,{z},1,{heading},1,frame_000000.jpg,{trial}\n",
        encoding="utf-8",
    )


class BuildRunIndexTests(unittest.TestCase):
    def test_repeated_labels_and_old_mislabeled_runs_use_gps_pose(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(root, "run_p1_r01", "calib01_tilt30_p1_r01", -9.0)
            write_run(root, "run_p1_r01_duplicate", "calib01_tilt30_p1_r01", -9.0)
            write_run(root, "run_old_p1", "calib01_tilt30_p1", -8.8)
            write_run(root, "run_corridor", "calib01_tilt30_p1", -8.0, 3.14159)

            rows, skipped = build_index(root)
            self.assertEqual(skipped, 1)
            by_run = {row["run_id"]: row for row in rows}
            self.assertEqual(by_run["run_p1_r01"]["offline_pose_label"], "P1")
            self.assertEqual(by_run["run_p1_r01"]["repetition"], "01")
            self.assertEqual(by_run["run_p1_r01_duplicate"]["repetition"], "02")
            self.assertIn("repeated logged ID", by_run["run_p1_r01_duplicate"]["notes"])
            self.assertEqual(by_run["run_old_p1"]["offline_pose_label"], "P2")
            self.assertIn("GPS-derived P2", by_run["run_old_p1"]["notes"])

    def test_unrecognized_z_is_not_guessed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(root, "run_unexpected", "calib01_tilt30_p2_r01", -8.6)
            rows, skipped = build_index(root)
            self.assertEqual(rows, [])
            self.assertEqual(skipped, 1)


if __name__ == "__main__":
    unittest.main()
