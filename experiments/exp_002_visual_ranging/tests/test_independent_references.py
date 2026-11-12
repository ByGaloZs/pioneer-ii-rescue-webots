"""Tests for EXP-002 independent-reference comparison and fail-closed ranges."""

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from validate_independent_references import validate_references


class IndependentReferenceTests(unittest.TestCase):
    def test_bearing_errors_and_independent_range_are_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            results = root / "results"
            run = results / "run_20261111_120000"
            analysis = root / "analysis" / run.name
            run.mkdir(parents=True)
            analysis.mkdir(parents=True)
            (run / "frames.csv").write_text(
                "image_saved,image_file,image_width_px,image_height_px,camera_fov_rad\n"
                "1,frame_000000.jpg,160,160,3.1415\n", encoding="utf-8"
            )
            (analysis / "v3_sectors.csv").write_text(
                "frame_file,sector_id,status,distance_m,near_field_occluded,reason\n"
                "frame_000000.jpg,7,occupied,1.25,False,\n"
                "frame_000000.jpg,8,occupied,1.30,True,\n"
                "frame_000000.jpg,9,unknown,,False,occlusion_after_floor\n", encoding="utf-8"
            )
            (analysis / "v3_summary.json").write_text(
                '{"camera_model":{"tilt_rad":-0.5236,"yaw_rad":0.0}}\n',
                encoding="utf-8",
            )
            references = root / "references.csv"
            with references.open("w", newline="", encoding="utf-8") as output:
                writer = csv.writer(output)
                writer.writerow([
                    "measurement_type", "run_id", "frame_file", "sector_id",
                    "image_x_px", "image_y_px", "reference_bearing_deg",
                    "reference_distance_m", "reference_method", "notes",
                ])
                writer.writerow(["bearing", run.name, "frame_000000.jpg", "", 40, 80,
                                 -49.5, "", "surveyed marker", "left marker"])
                writer.writerow(["bearing", run.name, "frame_000000.jpg", "", 120, 80,
                                 50.0, "", "surveyed marker", "right marker"])
                writer.writerow(["range", run.name, "frame_000000.jpg", 7, "", "", "",
                                 1.0, "tape from camera reference", "visible contact"])
                writer.writerow(["range", run.name, "frame_000000.jpg", 8, "", "", "",
                                 1.0, "tape from camera reference", "visible boundary; near field masked"])
                writer.writerow(["range", run.name, "frame_000000.jpg", 9, "", "", "",
                                 1.0, "tape from camera reference", "contact itself occluded"])

            rows, summary = validate_references(references, results, root / "analysis")

            self.assertEqual(summary["bearing"]["count"], 2)
            self.assertEqual(summary["bearing_reference_left_count"], 1)
            self.assertEqual(summary["bearing_reference_right_count"], 1)
            self.assertAlmostEqual(summary["bearing"]["mae"], 0.31, places=1)
            self.assertEqual(summary["range"]["count"], 2)
            self.assertAlmostEqual(summary["range"]["mae"], 0.275)
            self.assertEqual(summary["range_candidates_with_unknown_near_field"], 1)
            self.assertTrue(rows[3]["near_field_occluded"])
            self.assertEqual(rows[4]["status"], "excluded_occlusion_after_floor")
            self.assertEqual(summary["range_excluded"]["excluded_occlusion_after_floor"], 1)

    def test_empty_reference_sheet_is_not_reported_as_zero_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "references.csv"
            path.write_text(
                "measurement_type,run_id,frame_file,sector_id,image_x_px,image_y_px,"
                "reference_bearing_deg,reference_distance_m,reference_method,notes\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "contains no measurements"):
                validate_references(path, Path(temporary), Path(temporary))


if __name__ == "__main__":
    unittest.main()
