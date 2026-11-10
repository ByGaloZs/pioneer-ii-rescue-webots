"""Tests with synthetic imagery only; they do not establish Webots performance."""

import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from camera_projection import CameraProjection  # noqa: E402
from segment_image import (  # noqa: E402
    PixelClass,
    extract_radial_boundaries,
    segment_image,
)


class CameraProjectionTest(unittest.TestCase):
    def test_angular_coordinates_at_center_and_edges(self):
        camera = CameraProjection()
        self.assertEqual(camera.center, (79.5, 79.5))
        self.assertEqual(camera.pixel_to_angles(79.5, 79.5), (0.0, 0.0))
        left, _ = camera.pixel_to_angles(0, 79.5)
        right, _ = camera.pixel_to_angles(159, 79.5)
        self.assertAlmostEqual(left, camera.horizontal_fov_rad / 2)
        self.assertAlmostEqual(right, -camera.horizontal_fov_rad / 2)
        self.assertAlmostEqual(camera.vertical_fov, camera.horizontal_fov_rad)
        self.assertEqual(camera.max_inscribed_radius_px, 79)

    def test_angle_roundtrip_and_sampling(self):
        camera = CameraProjection(width=160, height=160)
        for x, y in [(20, 10), (150, 140), (79.5, 79.5)]:
            angles = camera.pixel_to_angles(x, y)
            new_x, new_y = camera.angles_to_pixel(*angles)
            self.assertAlmostEqual(new_x, x)
            self.assertAlmostEqual(new_y, y)
        samples = camera.radial_pixels(0.0, 8, 30)
        self.assertEqual([r for r, _, _ in samples], list(range(8, 31)))
        self.assertTrue(all(y in (79, 80) for _, _, y in samples))
        self.assertEqual(len(camera.sector_angles(0, 16)), 3)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            CameraProjection(width=0)
        with self.assertRaises(ValueError):
            CameraProjection(horizontal_fov_rad=math.nan)
        with self.assertRaises(ValueError):
            CameraProjection().pixel_to_angles(-1, 12)


class SegmentationTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraProjection()
        # BGR colors approximate the original WBT material colors.
        self.grey = (118, 118, 118)
        self.cyan = (255, 255, 102)
        self.green = (14, 255, 41)

    def test_color_classes_and_disconnected_grey(self):
        image = np.zeros((160, 160, 3), dtype=np.uint8)
        image[:, :] = self.grey
        image[:, 110:] = self.cyan
        image[10:20, 10:20] = self.green
        result = segment_image(image)
        self.assertEqual(result.labels[80, 80], PixelClass.FLOOR)
        self.assertEqual(result.labels[80, 130], PixelClass.OBSTACLE)
        self.assertEqual(result.labels[15, 15], PixelClass.VICTIM)
        self.assertEqual(result.labels.dtype, np.uint8)

    def test_known_color_boundary_for_right_image_sector(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        image[:, 105:] = self.cyan
        result = segment_image(image)
        observations = extract_radial_boundaries(result, self.camera)
        self.assertEqual(len(observations), 16)
        self.assertEqual(observations[0].status, "usable")
        self.assertAlmostEqual(observations[0].boundary_px, 26.0, delta=2.0)
        self.assertIsNone(observations[0].unknown_reason)
        self.assertEqual(observations[8].status, "unknown")
        self.assertIsNone(observations[8].boundary_px)

    def test_unknown_between_floor_and_obstacle_is_not_free(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        image[:, 99:105] = (0, 0, 210)  # Red is not a labeled surface.
        image[:, 105:] = self.cyan
        result = segment_image(image)
        boundary = extract_radial_boundaries(result, self.camera)[0]
        self.assertEqual(boundary.status, "unknown")
        self.assertIsNone(boundary.boundary_px)
        self.assertEqual(boundary.unknown_reason, "ambiguous_transition")

    def test_unbounded_floor_is_not_valid_range(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        observations = extract_radial_boundaries(segment_image(image), self.camera)
        self.assertTrue(all(s.status == "unknown" for s in observations))
        self.assertTrue(all(s.boundary_px is None for s in observations))

    def test_malformed_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            segment_image(np.empty((12, 12), dtype=np.uint8))
        with self.assertRaises(ValueError):
            segment_image(np.empty((12, 12, 3), dtype=np.float32))

class SelfOcclusionTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraProjection()
        self.grey = (118, 118, 118)
        self.cyan = (255, 255, 102)

    def _mask(self, rx: float = .19):
        from self_occlusion import make_occlusion_mask
        return make_occlusion_mask({
            "coordinate_system": "normalized", "verified": False,
            "regions": [{"shape": "ellipse", "center": [.5, .5], "radii": [rx, rx]}],
        }, (160, 160))

    def test_chassis_exclusion_and_visible_boundary(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        image[:, 125:] = self.cyan
        mask = self._mask()
        result = segment_image(image, occlusion_mask=mask)
        self.assertEqual(result.labels[80, 80], PixelClass.UNKNOWN)
        self.assertEqual(result.confidence[80, 80], 0)
        self.assertEqual(result.labels[80, 112], PixelClass.FLOOR)
        sectors = extract_radial_boundaries(result, self.camera)
        self.assertEqual(sectors[0].status, "usable")
        self.assertTrue(sectors[0].near_field_occluded)
        self.assertIsNotNone(sectors[0].first_visible_floor_px)
        self.assertGreater(sectors[0].boundary_px, sectors[0].first_visible_floor_px)

    def test_occluded_whole_ray_returns_unknown(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        image[:, 125:] = self.cyan
        mask = np.ones((160, 160), dtype=bool)
        result = segment_image(image, occlusion_mask=mask)
        sectors = extract_radial_boundaries(result, self.camera)
        self.assertTrue(all(item.status == "unknown" for item in sectors))
        self.assertTrue(all(item.boundary_px is None for item in sectors))

    def test_unknown_gap_after_body_returns_unknown(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        image[:, 115:119] = (0, 0, 220)
        image[:, 125:] = self.cyan
        result = segment_image(image, occlusion_mask=self._mask())
        sector = extract_radial_boundaries(result, self.camera)[0]
        self.assertEqual(sector.status, "unknown")
        self.assertEqual(sector.unknown_reason, "ambiguous_transition")

    def test_mask_wrong_shape_or_dtype_rejected(self):
        image = np.full((160, 160, 3), self.grey, dtype=np.uint8)
        with self.assertRaises(ValueError):
            segment_image(image, occlusion_mask=np.ones((16, 16), dtype=bool))
        with self.assertRaises(ValueError):
            segment_image(image, occlusion_mask=np.zeros((160, 160), dtype=np.uint8))

    def test_mask_config_validation(self):
        from self_occlusion import make_occlusion_mask
        with self.assertRaises(ValueError):
            make_occlusion_mask({"coordinate_system": "normalized", "verified": "yes",
                                 "regions": [{"shape": "ellipse", "center": [.5,.5], "radii": [.1,.1]}]}, (160,160))
        with self.assertRaises(ValueError):
            make_occlusion_mask({"coordinate_system": "normalized", "verified": False,
                                 "regions": [{"shape": "polygon", "points": [[1.1, 0], [.4,.5], [.3,.5]]}]}, (160,160))

    def test_polygon_exclusion(self):
        from self_occlusion import make_occlusion_mask
        mask = make_occlusion_mask({"coordinate_system": "normalized", "verified": False,
                                    "regions": [{"shape": "polygon", "points": [[.2,.2],[.8,.2],[.5,.8]]}]}, (160,160))
        self.assertTrue(mask[70,80])
        self.assertFalse(mask[0,0])


class PilotWorldTest(unittest.TestCase):
    def test_world_edit_preserves_other_content(self):
        from create_pilot_world import make_pilot_world
        original = "#VRML_SIM R2020b utf8\nPioneer2 {\n  translation 0 0 -9.05\n  rotation 0 1 0 3.14159\n  controller \"fool_controller\"\n}\n"
        changed = make_pilot_world(original)
        self.assertIn("translation 0 0 -8.8", changed)
        self.assertIn("rotation 0 1 0 0", changed)
        self.assertIn('controller "fool_controller"', changed)
        self.assertEqual(original.splitlines()[0], changed.splitlines()[0])
        self.assertIn("translation 0 0 -9.05", original)


if __name__ == "__main__":
    unittest.main()
