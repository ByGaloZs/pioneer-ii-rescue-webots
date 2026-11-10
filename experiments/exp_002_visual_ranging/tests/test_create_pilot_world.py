"""Offline tests; the Webots simulation must still be executed separately."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from create_pilot_world import VARIANTS, generate_variants, make_pilot_world


class CameraPilotWorldTest(unittest.TestCase):
    def setUp(self) -> None:
        self.original = (
            '#VRML_SIM R2020b utf8\n'
            'WorldInfo {\n  basicTimeStep 32\n}\n'
            'Pioneer2 {\n'
            '  translation 0 0 -9.05\n'
            '  rotation 0 1 0 3.141592653589793\n'
            '  controller "fool_controller"\n'
            '  extensionSlot [\n'
            '    GPS {\n      resolution 3\n    }\n'
            '    DEF SPHERICAL_CAMERA Camera {\n'
            '      translation 0.025 0.44 -0.05\n'
            '      rotation 1 0 0 -1.5708\n'
            '      name "camera_s"\n    }\n'
            '  ]\n}\n'
        )

    def test_generation_changes_exactly_three_parameters(self) -> None:
        for variant in VARIANTS:
            with self.subTest(variant=variant.key):
                actual = make_pilot_world(self.original, camera_tilt_rad=variant.tilt_rad)
                expected = self.original.replace(
                    '  translation 0 0 -9.05\n  rotation 0 1 0 3.141592653589793\n',
                    '  translation 0 0 -8.8\n  rotation 0 1 0 0\n',
                ).replace('resolution 3', 'resolution -1').replace(
                    'rotation 1 0 0 -1.5708', f'rotation 1 0 0 {variant.tilt_rad:g}'
                )
                self.assertEqual(actual, expected)
                self.assertIn('controller "fool_controller"', actual)
                self.assertIn('name "camera_s"', actual)

    def test_missing_or_duplicate_nodes_fail_closed(self) -> None:
        with self.assertRaises(ValueError):
            make_pilot_world(self.original.replace('Pioneer2 {', 'Robot {'))
        with self.assertRaises(ValueError):
            make_pilot_world(self.original.replace('resolution 3', ''))
        with self.assertRaises(ValueError):
            make_pilot_world(self.original.replace('DEF SPHERICAL_CAMERA', 'DEF OTHER_CAMERA'))
        with self.assertRaises(ValueError):
            make_pilot_world(self.original + self.original)

    def test_non_finite_values_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            make_pilot_world(self.original, z=float('nan'))
        with self.assertRaises(ValueError):
            make_pilot_world(self.original, camera_tilt_rad=float('inf'))

    def test_original_world_is_unchanged_and_outputs_are_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source = base / 'final_scenario01.wbt'
            source.write_text(self.original, encoding='utf-8')
            outputs = generate_variants(source, base / 'worlds')
            self.assertEqual(len(outputs), 3)
            self.assertEqual([p.name for p in outputs], [
                'exp002_camera_01_tilt45.wbt',
                'exp002_camera_01_tilt30.wbt',
                'exp002_camera_01_horizontal.wbt',
            ])
            self.assertEqual(source.read_text(encoding='utf-8'), self.original)
            self.assertEqual(generate_variants(source, base / 'worlds'), outputs)
            outputs[0].write_text('user changes', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                generate_variants(source, base / 'worlds')
            self.assertEqual(source.read_text(encoding='utf-8'), self.original)
            generate_variants(source, base / 'worlds', overwrite=True)
            self.assertIn('resolution -1', outputs[0].read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
