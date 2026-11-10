"""EXP-002 calibration world generator tests (synthetic fixtures, no Webots runtime)."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from create_pilot_world import (
    CALIBRATION_POSITIONS,
    generate_calibration_worlds,
    generate_variants,
    make_pilot_world,
)


ORIGINAL = '''#VRML_SIM R2020b utf8
WorldInfo {
  basicTimeStep 32
}
Pioneer2 {
  translation 0 0 -9.05
  rotation 0 1 0 3.141592653589793
  controller "fool_controller"
  extensionSlot [
    GPS {
      resolution 3
    }
    DEF SPHERICAL_CAMERA Camera {
      translation 0.025 0.44 -0.05
      rotation 1 0 0 -1.5708
      children [
        DEF LENS Shape {
          geometry Sphere {
            radius 0.005
          }
        }
        DEF STRUCTURE_AXES Shape {
          geometry IndexedLineSet {
            coord Coordinate {
              point [ 0 0 0 1 0 0 ]
            }
            coordIndex [ 0, 1 ]
          }
        }
        DEF CAMERA_SHAPE Transform {
          children [
            DEF CAMERA_OUTLINE Shape {
              geometry Cylinder {
                radius 0.008
              }
            }
          ]
        }
      ]
      name "camera_s"
      fieldOfView 3.1415
      width 160
      height 160
      spherical TRUE
    }
  ]
}
'''


class CalibrationWorldTests(unittest.TestCase):
    def test_all_positions_preserve_camera_and_exclude_debug_visuals(self) -> None:
        for position in CALIBRATION_POSITIONS:
            with self.subTest(position=position.key):
                world = make_pilot_world(
                    ORIGINAL, z=position.z_m, camera_tilt_rad=-0.5236,
                    strip_camera_debug=True, trial_id=f"calib01_tilt30_{position.key}",
                )
                self.assertIn(f"translation 0 0 {position.z_m:g}", world)
                self.assertIn("rotation 0 1 0 0", world)
                self.assertIn("rotation 1 0 0 -0.5236", world)
                self.assertIn("resolution -1", world)
                self.assertIn(f'controllerArgs "exp002_trial=calib01_tilt30_{position.key}"', world)
                self.assertIn("children []", world)
                self.assertIn('name "camera_s"', world)
                self.assertIn("spherical TRUE", world)
                self.assertIn("fieldOfView 3.1415", world)
                for name in ("LENS", "STRUCTURE_AXES", "CAMERA_SHAPE", "CAMERA_OUTLINE"):
                    self.assertNotIn(f"DEF {name} ", world)

    def test_unrelated_world_nodes_preserved(self) -> None:
        source = ORIGINAL.replace("Pioneer2 {", 'Solid {\n  name "landmark"\n}\nPioneer2 {', 1)
        result = make_pilot_world(
            source, strip_camera_debug=True, trial_id="calib01_tilt30_p2"
        )
        self.assertIn('Solid {\n  name "landmark"\n}', result)
        self.assertIn("WorldInfo {\n  basicTimeStep 32\n}", result)

    def test_incompatible_camera_or_trial_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unexpected camera visual children"):
            make_pilot_world(
                ORIGINAL.replace("DEF STRUCTURE_AXES", "DEF EXTRA_WIDGET"),
                strip_camera_debug=True,
            )
        with self.assertRaisesRegex(ValueError, "Children field does not belong"):
            make_pilot_world(
                ORIGINAL.replace("children [\n        DEF LENS", "items [\n        DEF LENS"),
                strip_camera_debug=True,
            )
        with self.assertRaisesRegex(ValueError, "no controllerArgs"):
            make_pilot_world(
                ORIGINAL.replace(
                    'controller "fool_controller"',
                    'controller "fool_controller"\n  controllerArgs "existing"',
                ),
                trial_id="calib01_tilt30_p1",
            )
        with self.assertRaisesRegex(ValueError, "Invalid trial ID"):
            make_pilot_world(ORIGINAL, trial_id="../unsafe")

    def test_batch_reproducibility_and_original_preservation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = root / "final_scenario01.wbt"
            original.write_text(ORIGINAL, encoding="utf-8")
            output_dir = root / "worlds"
            worlds = generate_calibration_worlds(original, output_dir)
            self.assertEqual(
                [world.name for world in worlds],
                ["exp002_calibration_01_p1.wbt",
                 "exp002_calibration_01_p2.wbt",
                 "exp002_calibration_01_p3.wbt"],
            )
            self.assertEqual(original.read_text(encoding="utf-8"), ORIGINAL)
            self.assertEqual(generate_calibration_worlds(original, output_dir), worlds)
            worlds[1].write_text("user-edited", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                generate_calibration_worlds(original, output_dir)
            generate_calibration_worlds(original, output_dir, overwrite=True)
            self.assertIn("calib01_tilt30_p2", worlds[1].read_text(encoding="utf-8"))
            self.assertEqual(original.read_text(encoding="utf-8"), ORIGINAL)
            self.assertEqual(len(generate_variants(original, output_dir)), 3)

    def test_refuse_partial_batch_write_on_conflict(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = root / "final_scenario01.wbt"
            original.write_text(ORIGINAL, encoding="utf-8")
            output_dir = root / "worlds"
            output_dir.mkdir()
            (output_dir / "exp002_calibration_01_p3.wbt").write_text("other", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                generate_calibration_worlds(original, output_dir)
            self.assertFalse((output_dir / "exp002_calibration_01_p1.wbt").exists())
            self.assertFalse((output_dir / "exp002_calibration_01_p2.wbt").exists())


if __name__ == "__main__":
    unittest.main()
