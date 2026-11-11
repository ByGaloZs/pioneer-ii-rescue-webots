"""Test regenerating the one shared world for calibration runs."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from create_pilot_world import generate_test_world, make_pilot_world


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


class SharedWorldCalibrationTests(unittest.TestCase):
    def test_calibration_overwrites_same_copy_with_explicit_parameters(self) -> None:
        world = make_pilot_world(
            ORIGINAL,
            z=-9.0,
            camera_tilt_rad=-0.5236,
            trial_id="calib01_tilt30_p1",
            strip_camera_debug=True,
        )
        self.assertIn("translation 0 0 -9", world)
        self.assertIn("rotation 0 1 0 0", world)
        self.assertIn("rotation 1 0 0 -0.5236", world)
        self.assertIn("resolution -1", world)
        self.assertIn('controller "experiment_test_controller"', world)
        self.assertIn('controllerArgs "experiment=002;trial=calib01_tilt30_p1"', world)
        self.assertIn("children []", world)
        self.assertIn('name "camera_s"', world)
        self.assertIn("spherical TRUE", world)
        for name in ("LENS", "STRUCTURE_AXES", "CAMERA_SHAPE", "CAMERA_OUTLINE"):
            self.assertNotIn(f"DEF {name} ", world)

    def test_camera_mask_removal_fails_on_unexpected_visuals(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unexpected camera visual children"):
            make_pilot_world(
                ORIGINAL.replace("DEF STRUCTURE_AXES", "DEF EXTRA_WIDGET"),
                trial_id="calib01_tilt30_p1",
                strip_camera_debug=True,
            )

    def test_single_output_refuses_conflict_without_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.wbt"
            output = root / "experiment_test_scenario01.wbt"
            source.write_text(ORIGINAL, encoding="utf-8")
            generate_test_world(source, output, trial_id="tilt45")
            with self.assertRaises(FileExistsError):
                generate_test_world(
                    source,
                    output,
                    z=-9.0,
                    camera_tilt_rad=-0.5236,
                    trial_id="calib01_tilt30_p1",
                )
            generate_test_world(
                source,
                output,
                z=-9.0,
                camera_tilt_rad=-0.5236,
                trial_id="calib01_tilt30_p1",
                strip_camera_debug=True,
                overwrite=True,
            )
            self.assertIn("calib01_tilt30_p1", output.read_text(encoding="utf-8"))
            self.assertEqual(source.read_text(encoding="utf-8"), ORIGINAL)


if __name__ == "__main__":
    unittest.main()
