"""Tests for offline comparison against the scenario-01 wall plane."""

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from compare_wall_ranges import compare_wall_runs, wall_plane_distance_m


class WallReferenceTests(unittest.TestCase):
    def test_reference_distance_accounts_for_sector_bearing(self) -> None:
        self.assertAlmostEqual(
            wall_plane_distance_m(
                camera_origin_z_m=-9.05,
                wall_face_z_m=-9.8,
                bearing_robot_deg=0.0,
            ),
            0.75,
        )
        self.assertAlmostEqual(
            wall_plane_distance_m(
                camera_origin_z_m=-9.05,
                wall_face_z_m=-9.8,
                bearing_robot_deg=60.0,
            ),
            1.5,
        )
        with self.assertRaises(ValueError):
            wall_plane_distance_m(
                camera_origin_z_m=-9.05,
                wall_face_z_m=-9.8,
                bearing_robot_deg=90.0,
            )

    def test_comparison_uses_index_pose_and_omits_corridor(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            index = root / "run_index.csv"
            index.write_text(
                "run_id,logged_trial_id,offline_pose_label,gps_x_m,gps_z_m,heading_rad,notes\n"
                "run_wall,calib01_tilt30_p1,P1,0,-9,0,wall\n"
                "run_corridor,calib01_tilt30_p1,corridor_view,-1.5,-8,3.14159,corridor\n",
                encoding="utf-8",
            )
            run_dir = root / "analysis" / "run_wall"
            run_dir.mkdir(parents=True)
            with (run_dir / "v3_sectors.csv").open("w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=(
                        "sector_id", "camera_angle_deg", "bearing_robot_deg", "distance_m",
                        "status", "reason", "near_field_occluded",
                    ),
                )
                writer.writeheader()
                writer.writerow({
                    "sector_id": 7,
                    "camera_angle_deg": 0,
                    "bearing_robot_deg": 0,
                    "distance_m": 0.8,
                    "status": "occupied",
                    "reason": "",
                    "near_field_occluded": "True",
                })
                writer.writerow({
                    "sector_id": 8,
                    "camera_angle_deg": 5,
                    "bearing_robot_deg": "",
                    "distance_m": "",
                    "status": "unknown",
                    "reason": "ambiguous_transition",
                    "near_field_occluded": "True",
                })

            sectors, summaries = compare_wall_runs(index, root / "analysis")
            self.assertEqual(len(sectors), 2)
            self.assertEqual(len(summaries), 2)
            self.assertAlmostEqual(sectors[0]["reference_wall_plane_distance_m"], 0.75)
            self.assertAlmostEqual(sectors[0]["absolute_error_m"], 0.05)
            self.assertEqual(sectors[1]["absolute_error_m"], "")
            self.assertEqual(summaries[0]["near_field_occluded_candidate_count"], 1)
            self.assertEqual(summaries[1]["run_id"], "pooled_wall_runs")
            self.assertAlmostEqual(summaries[1]["candidate_mae_m"], 0.05)


if __name__ == "__main__":
    unittest.main()
