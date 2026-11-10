"""Generate controlled EXP-002 camera-orientation worlds from scenario 01.

Offline test setup only: the Webots controller must never read world geometry.
Canonical final_scenario*.wbt files are never modified by this script.
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CameraVariant:
    key: str
    tilt_rad: float


@dataclass(frozen=True)
class CalibrationPosition:
    key: str
    z_m: float
    nominal_center_wall_distance_m: float


CALIBRATION_POSITIONS = (
    CalibrationPosition("p1", -9.0, 0.8),
    CalibrationPosition("p2", -8.8, 1.0),
    CalibrationPosition("p3", -8.3, 1.5),
)


VARIANTS = (
    CameraVariant("tilt45", -0.7854),
    CameraVariant("tilt30", -0.5236),
    CameraVariant("horizontal", 0.0),
)

ROBOT_POSE = re.compile(
    r"(?P<header>^Pioneer2\s*\{\s*\n)"
    r"(?P<translation>[ \t]*translation[ \t]+[^\n]+\n)"
    r"(?P<rotation>[ \t]*rotation[ \t]+[^\n]+\n)",
    re.MULTILINE,
)
GPS_RESOLUTION = re.compile(
    r"(?P<prefix>\bGPS\s*\{\s*\n[ \t]*resolution[ \t]+)"
    r"(?P<value>[^\s\}]+)",
)
SPHERICAL_ROTATION = re.compile(
    r"(?P<prefix>^\s*DEF SPHERICAL_CAMERA Camera\s*\{\s*\n"
    r"[ \t]*translation[^\n]*\n[ \t]*rotation[ \t]+)"
    r"(?P<value>[^\n]+)",
    re.MULTILINE,
)


ROBOT_CONTROLLER = re.compile(r'(?m)^(?P<indent>[ \t]*)controller "fool_controller"[ \t]*$')
CAMERA_HEADER = re.compile(r"(?m)^[ \t]*DEF SPHERICAL_CAMERA Camera\s*\{")
CHILDREN_HEADER = re.compile(r"(?m)^[ \t]*children[ \t]*\[")


def _replace_one(source: str, pattern: re.Pattern[str], value: str, label: str) -> str:
    matches = list(pattern.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {label} definition; found {len(matches)}")
    match = matches[0]
    return source[: match.start()] + match.group("prefix") + value + source[match.end() :]


def _strip_camera_debug_geometry(source: str) -> str:
    """Remove the four expected camera visual DEF nodes, not other sensors."""
    cameras = list(CAMERA_HEADER.finditer(source))
    if len(cameras) != 1:
        raise ValueError("Expected exactly one DEF SPHERICAL_CAMERA Camera node")
    children = CHILDREN_HEADER.search(source, cameras[0].end())
    if children is None:
        raise ValueError("Camera children field missing")
    opening = source.index("[", children.start(), children.end())
    depth = 0
    closing = -1
    for pos in range(opening, len(source)):
        if source[pos] == "[":
            depth += 1
        elif source[pos] == "]":
            depth -= 1
            if depth == 0:
                closing = pos
                break
    if closing < 0:
        raise ValueError("Unterminated camera children list")
    if not re.match(r'\s*name "camera_s"', source[closing + 1:]):
        raise ValueError("Children field does not belong to camera_s")
    names = re.findall(r"\bDEF\s+([A-Za-z0-9_]+)\s+\w+", source[opening + 1:closing])
    expected = ["LENS", "STRUCTURE_AXES", "CAMERA_SHAPE", "CAMERA_OUTLINE"]
    if sorted(names) != sorted(expected):
        raise ValueError("Unexpected camera visual children; refusing to remove them")
    return source[:children.start()] + "      children []" + source[closing + 1:]


def _write_worlds(
    source_path: Path, output_dir: Path, worlds: dict[str, str], overwrite: bool
) -> list[Path]:
    source_path = source_path.resolve()
    if any((output_dir / name).resolve() == source_path for name in worlds):
        raise ValueError("Refusing to overwrite the source world")
    for name, content in worlds.items():
        output = output_dir / name
        if output.exists() and output.read_text(encoding="utf-8") != content and not overwrite:
            raise FileExistsError(f"World differs from generated version: {output}")
    output_dir.mkdir(parents=True, exist_ok=True)
    results: list[Path] = []
    for name, content in worlds.items():
        output = output_dir / name
        if not output.exists() or output.read_text(encoding="utf-8") != content:
            output.write_text(content, encoding="utf-8")
        results.append(output)
    return results


def make_pilot_world(
    source: str,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    camera_tilt_rad: float = -0.7854,
    *,
    strip_camera_debug: bool = False,
    trial_id: str | None = None,
) -> str:
    """Replace only pose, GPS resolution and camera_s rotation in the source WBT."""
    if not all(math.isfinite(value) for value in (x, z, yaw, camera_tilt_rad)):
        raise ValueError("Pose and camera angle must contain finite numbers")
    matches = list(ROBOT_POSE.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one Pioneer2 node; found {len(matches)}")
    match = matches[0]
    pose = (
        f"{match.group('header')}"
        f"  translation {x:g} 0 {z:g}\n"
        f"  rotation 0 1 0 {yaw:g}\n"
    )
    result = source[: match.start()] + pose + source[match.end() :]
    result = _replace_one(result, GPS_RESOLUTION, "-1", "GPS resolution")
    result = _replace_one(
        result, SPHERICAL_ROTATION, f"1 0 0 {camera_tilt_rad:g}", "spherical camera rotation"
    )
    if strip_camera_debug:
        result = _strip_camera_debug_geometry(result)
    if trial_id is not None:
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", trial_id):
            raise ValueError("Invalid trial ID")
        matches = list(ROBOT_CONTROLLER.finditer(result))
        if len(matches) != 1 or re.search(r"(?m)^\s*controllerArgs\b", result):
            raise ValueError("Expected one unmodified robot controller and no controllerArgs")
        match = matches[0]
        insertion = f'\n{match.group("indent")}controllerArgs "exp002_trial={trial_id}"'
        result = result[:match.end()] + insertion + result[match.end():]
    return result


def generate_variants(
    source_path: Path,
    output_dir: Path,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    overwrite: bool = False,
) -> list[Path]:
    """Keep the old orientation-comparison worlds reproducible."""
    template = source_path.read_text(encoding="utf-8")
    worlds = {
        f"exp002_camera_01_{variant.key}.wbt": make_pilot_world(template, x, z, yaw, variant.tilt_rad)
        for variant in VARIANTS
    }
    return _write_worlds(source_path, output_dir, worlds, overwrite)


def generate_calibration_worlds(
    source_path: Path, output_dir: Path, overwrite: bool = False
) -> list[Path]:
    """Three single-shot camera -30° trials, with explicit non-metric IDs."""
    template = source_path.read_text(encoding="utf-8")
    worlds = {
        f"exp002_calibration_01_{position.key}.wbt": make_pilot_world(
            template, z=position.z_m, camera_tilt_rad=-0.5236,
            strip_camera_debug=True, trial_id=f"calib01_tilt30_{position.key}",
        )
        for position in CALIBRATION_POSITIONS
    }
    return _write_worlds(source_path, output_dir, worlds, overwrite)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate EXP-002 camera tilt comparison worlds")
    parser.add_argument("--source", type=Path, default=Path("worlds/final_scenario01.wbt"))
    parser.add_argument("--mode", choices=("orientation", "calibration"), default="orientation")
    parser.add_argument("--output-dir", type=Path, default=Path("worlds"))
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=-8.8)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--overwrite", action="store_true", help="Replace only generated pilot files")
    args = parser.parse_args()
    try:
        if args.mode == "calibration":
            if (args.x, args.z, args.yaw) != (0.0, -8.8, 0.0):
                parser.error("--x/--z/--yaw are only supported in orientation mode")
            generated = generate_calibration_worlds(args.source, args.output_dir, args.overwrite)
        else:
            generated = generate_variants(
                args.source, args.output_dir, args.x, args.z, args.yaw, args.overwrite
            )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for path in generated:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
