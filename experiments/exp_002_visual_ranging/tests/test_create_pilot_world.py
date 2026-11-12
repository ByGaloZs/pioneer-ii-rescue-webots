"""Offline tests for the single shared experiment world generator."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from create_pilot_world import generate_test_world, make_pilot_world


class SharedTestWorldTest(unittest.TestCase):
    def setUp(self) -> None:
        self.original = (
            "#VRML_SIM R2020b utf8\n"
            "WorldInfo {\n  basicTimeStep 32\n}\n"
            "Pioneer2 {\n"
            "  translation 0 0 -9.05\n"
            "  rotation 0 1 0 3.141592653589793\n"
            '  controller "fool_controller"\n'
            "  extensionSlot [\n"
            "    GPS {\n      resolution 3\n    }\n"
            "    DEF SPHERICAL_CAMERA Camera {\n"
            "      translation 0.025 0.44 -0.05\n"
            "      rotation 1 0 0 -1.5708\n"
            '      name "camera_s"\n    }\n'
            "  ]\n}\n"
        )

    def test_world_selects_shared_controller_and_explicit_trial(self) -> None:
        actual = make_pilot_world(
            self.original, camera_tilt_rad=-0.5236, trial_id="calib01_tilt30_p1"
        )
        self.assertIn("translation 0 0 -8.8", actual)
        self.assertIn("rotation 0 1 0 0", actual)
        self.assertIn("resolution -1", actual)
        self.assertIn("rotation 1 0 0 -0.5236", actual)
        self.assertIn('controller "experiment_test_controller"', actual)
        self.assertIn('controllerArgs "experiment=002;trial=calib01_tilt30_p1"', actual)
        self.assertNotIn('controller "fool_controller"', actual)
        self.assertEqual(self.original.splitlines()[0], actual.splitlines()[0])
        self.assertIn("translation 0 0 -9.05", self.original)

    def test_generator_adds_pose_rotation_when_source_uses_default_orientation(self) -> None:
        source = self.original.replace("  rotation 0 1 0 3.141592653589793\n", "")
        actual = make_pilot_world(source, yaw=-0.785398, trial_id="geom01_tilt30_right_r01")
        self.assertIn("translation 0 0 -8.8\n  rotation 0 1 0 -0.785398", actual)

    def test_invalid_source_pose_and_trial_fail_closed(self) -> None:
        with self.assertRaises(ValueError):
            make_pilot_world(self.original.replace("Pioneer2 {", "Robot {"), trial_id="p1")
        with self.assertRaises(ValueError):
            make_pilot_world(self.original, z=float("nan"), trial_id="p1")
        with self.assertRaises(ValueError):
            make_pilot_world(self.original, trial_id="../unsafe")
        with self.assertRaises(ValueError):
            make_pilot_world(
                self.original.replace(
                    'controller "fool_controller"',
                    'controller "fool_controller"\n  controllerArgs "existing"',
                ),
                trial_id="p1",
            )

    def test_generator_adds_unquantized_gps_resolution_if_missing(self) -> None:
        source = self.original.replace("      resolution 3\n", "")
        actual = make_pilot_world(source, trial_id="geom01_tilt30_right_r01")
        self.assertIn("resolution -1", actual)

    def test_generator_writes_one_reproducible_copy_and_protects_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "final_scenario01.wbt"
            output = root / "experiment_test_scenario01.wbt"
            source.write_text(self.original, encoding="utf-8")
            generated = generate_test_world(source, output, trial_id="calib01_tilt30_p1")
            self.assertEqual(generated, output)
            content = output.read_text(encoding="utf-8")
            self.assertIn("rotation 1 0 0 -0.5236", content)
            self.assertIn('controllerArgs "experiment=002;trial=calib01_tilt30_p1"', content)
            self.assertEqual(
                generate_test_world(source, output, trial_id="calib01_tilt30_p1"), output
            )
            with self.assertRaises(FileExistsError):
                generate_test_world(source, output, trial_id="camera_tilt30")
            generate_test_world(source, output, trial_id="camera_tilt30", overwrite=True)
            self.assertIn("camera_tilt30", output.read_text(encoding="utf-8"))
            self.assertEqual(source.read_text(encoding="utf-8"), self.original)


if __name__ == "__main__":
    unittest.main()
