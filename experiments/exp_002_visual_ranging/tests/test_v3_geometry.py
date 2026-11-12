"""Deterministic synthetic tests; not validation of real navigation safety."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from angular_observations import AngularConfig, classify_pixels, observe_sectors
from analyze_runs import CALIBRATION_TRIAL_ID, analyze_run
from spherical_geometry import SphericalCameraModel


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.model = SphericalCameraModel()

    def test_center_ray_and_ground_geometry(self):
        x = y = 79.5
        a, e = self.model.image_angles(x, y)
        self.assertAlmostEqual(a, 0)
        self.assertAlmostEqual(e, 0)
        ray = self.model.robot_ray(x, y)
        self.assertAlmostEqual(math.hypot(*ray), 1, places=6)
        self.assertAlmostEqual(ray[1], -0.5, delta=1e-3)
        contact = self.model.contact_on_ground(x, y)
        self.assertIsNotNone(contact)
        self.assertAlmostEqual(contact.horizontal_from_camera_m, .44 / math.tan(math.pi/6), delta=.001)
        self.assertAlmostEqual(contact.z_robot_m, -.05 - .44 / math.tan(math.pi/6), delta=.001)

    def test_horizon_and_outside_rejected(self):
        self.assertIsNone(self.model.contact_on_ground(79.5, 0))
        with self.assertRaises(ValueError):
            self.model.image_angles(-1, 12)
        with self.assertRaises(ValueError):
            SphericalCameraModel(height_m=0)
        with self.assertRaises(ValueError):
            SphericalCameraModel(horizontal_fov_rad=math.nan)

    def test_azimuth_right_and_left(self):
        left = self.model.robot_ray(40, 79.5)
        right = self.model.robot_ray(120, 79.5)
        self.assertLess(left[0], 0)
        self.assertGreater(right[0], 0)

    def test_zero_pitch_horizon_and_downward_ray(self):
        model = SphericalCameraModel(tilt_rad=0)
        self.assertIsNone(model.contact_on_ground(79.5, 79.5))
        self.assertIsNotNone(model.contact_on_ground(79.5, 125))

    def test_geometry_trial_labels_are_accepted_for_camera_analysis(self):
        self.assertIsNotNone(CALIBRATION_TRIAL_ID.fullmatch("geom01_tilt30_right_r01"))
        self.assertIsNotNone(CALIBRATION_TRIAL_ID.fullmatch("geom01_tilt30_left_r02"))
        self.assertIsNotNone(CALIBRATION_TRIAL_ID.fullmatch("geom01_tilt30_front_r01"))


class AngularObservationTests(unittest.TestCase):
    def setUp(self):
        self.model = SphericalCameraModel()
        self.gray = (118, 118, 118)
        self.cyan = (255, 255, 102)

    def image_with_contact(self):
        image = np.full((160, 160, 3), self.gray, np.uint8)
        image[:80, :] = self.cyan
        return image

    def test_unverified_mask_never_produces_range(self):
        image = self.image_with_contact()
        observations, _ = observe_sectors(image, self.model)
        self.assertEqual(len(observations), 16)
        self.assertTrue(all(row.status == "unknown" and row.distance_m is None for row in observations))
        self.assertTrue(all(row.reason == "unverified_self_occlusion_mask" for row in observations))

    def test_gray_floor_is_seeded_from_robot_mask_when_robot_covers_bottom(self):
        image = np.full((160, 160, 3), (249, 247, 246), np.uint8)
        image[:70, :] = self.cyan
        image[70:105, :] = (77, 77, 77)
        robot_mask = np.zeros((160, 160), bool)
        robot_mask[105:, :] = True

        labels = classify_pixels(image, robot_mask, AngularConfig())
        self.assertTrue(np.all(labels[70:105, :] == 1))
        self.assertTrue(np.all(labels[:70, :] == 2))
        self.assertTrue(np.all(labels[105:, :] == 0))

        observations, _ = observe_sectors(
            image, self.model, occlusion_mask=robot_mask, mask_verified=True
        )
        center = [row for row in observations if row.sector_id in (7, 8)]
        self.assertTrue(all(row.status == "occupied" for row in center))
        self.assertTrue(all(row.near_field_occluded for row in center))

    def test_center_columns_have_plausible_floor_contact(self):
        image = self.image_with_contact()
        observations, labels = observe_sectors(
            image, self.model,
            occlusion_mask=np.zeros((160, 160), bool), mask_verified=True
        )
        self.assertEqual(labels[120, 80], 1)
        self.assertEqual(labels[30, 80], 2)
        center = [row for row in observations if row.sector_id in (7, 8)]
        self.assertTrue(all(r.status == "occupied" for r in center))
        self.assertTrue(all(.5 <= r.distance_m <= 1.2 for r in center))
        self.assertTrue(any(r.status == "unknown" for r in observations))

    def test_near_field_mask_preserves_visible_contact_but_not_ambiguous_contact(self):
        image = self.image_with_contact()
        mask = np.zeros((160, 160), bool)
        mask[100:, :] = True
        observations, _ = observe_sectors(image, self.model,
                                          occlusion_mask=mask, mask_verified=True)
        center = [o for o in observations if o.sector_id in (7, 8)]
        self.assertTrue(all(o.status == "occupied" for o in center))
        self.assertTrue(all(o.near_field_occluded for o in center))
        image[95:102, :] = (0, 0, 255)
        observations, _ = observe_sectors(image, self.model,
                                          occlusion_mask=np.zeros_like(mask), mask_verified=True)
        self.assertTrue(all(o.status == "unknown" for o in observations if o.sector_id in (7, 8)))

    def test_mask_between_visible_floor_and_contact_keeps_contact_unknown(self):
        image = self.image_with_contact()
        mask = np.zeros((160, 160), dtype=bool)
        mask[75:85, :] = True
        observations, _ = observe_sectors(
            image, self.model, occlusion_mask=mask, mask_verified=True
        )
        center = [row for row in observations if row.sector_id in (7, 8)]
        self.assertTrue(all(row.status == "unknown" for row in center))
        self.assertTrue(all(row.reason == "occlusion_after_floor" for row in center))
        self.assertTrue(all(row.distance_m is None for row in center))

    def test_no_floor_is_not_free(self):
        image = np.full((160, 160, 3), self.cyan, np.uint8)
        observations, _ = observe_sectors(image, self.model,
                                          occlusion_mask=np.zeros((160, 160), bool), mask_verified=True)
        self.assertTrue(all(o.distance_m is None for o in observations))

    def test_invalid_mask_and_sector_resolution(self):
        with self.assertRaises(ValueError):
            observe_sectors(self.image_with_contact(), self.model,
                            occlusion_mask=np.zeros((5, 5), bool), mask_verified=True)
        cfg = AngularConfig(sector_count=160, samples_per_sector=3)
        observations, _ = observe_sectors(self.image_with_contact(), self.model,
                                          occlusion_mask=np.zeros((160, 160), bool),
                                          mask_verified=True, config=cfg)
        self.assertTrue(all(o.status == "unknown" for o in observations))

    def test_batch_export_retains_unknown_no_fake_mae(self):
        with tempfile.TemporaryDirectory() as base:
            base = Path(base)
            run = base / "run_test"
            run.mkdir()
            image = self.image_with_contact()
            cv2.imwrite(str(run / "frame_000000.jpg"), image)
            (run / "frames.csv").write_text(
                "image_saved,image_file,camera_fov_rad,trial_id\n"
                "1,frame_000000.jpg,3.1415,calib01_tilt30_p1_r01\n", encoding="utf-8"
            )
            out = base / "analysis"
            summary = analyze_run(run, output_dir=out)
            self.assertEqual(summary["unknown"], 16)
            self.assertEqual(summary["occupied"], 0)
            self.assertIsNone(summary["metric_error_mae_m"])
            self.assertTrue((out / "v3_sectors.csv").exists())
            self.assertTrue((out / "v3_summary.json").exists())


if __name__ == "__main__":
    unittest.main()
