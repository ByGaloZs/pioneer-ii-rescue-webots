"""Regenerate the single shared experiment world from scenario 01.

The output is an offline test copy. Canonical final_scenario*.wbt files are
never modified, and the runtime controller receives no world geometry.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


ROBOT_POSE = re.compile(
    r"(?P<header>^Pioneer2\s*\{\s*\n)"
    r"(?P<translation>[ \t]*translation[ \t]+[^\n]+\n)"
    r"(?P<rotation>[ \t]*rotation[ \t]+[^\n]+\n)?",
    re.MULTILINE,
)
GPS_HEADER = re.compile(
    r"(?P<prefix>^(?P<indent>[ \t]*)GPS\s*\{\s*\n)"
    r"(?:(?P<field_indent>[ \t]*)resolution[ \t]+[^\s\}]+[^\n]*\n)?",
    re.MULTILINE,
)
SPHERICAL_ROTATION = re.compile(
    r"(?P<prefix>^\s*DEF SPHERICAL_CAMERA Camera\s*\{\s*\n"
    r"[ \t]*translation[^\n]*\n[ \t]*rotation[ \t]+)"
    r"(?P<value>[^\n]+)",
    re.MULTILINE,
)
ROBOT_CONTROLLER = re.compile(
    r'(?m)^(?P<indent>[ \t]*)controller "fool_controller"[ \t]*$'
)
CAMERA_HEADER = re.compile(r"(?m)^[ \t]*DEF SPHERICAL_CAMERA Camera\s*\{")
CHILDREN_HEADER = re.compile(r"(?m)^[ \t]*children[ \t]*\[")


def _replace_one(source: str, pattern: re.Pattern[str], value: str, label: str) -> str:
    matches = list(pattern.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {label} definition; found {len(matches)}")
    match = matches[0]
    return source[: match.start()] + match.group("prefix") + value + source[match.end() :]


def _set_gps_resolution(source: str) -> str:
    matches = list(GPS_HEADER.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one GPS definition; found {len(matches)}")
    match = matches[0]
    indent = match.group("field_indent") or (match.group("indent") + "  ")
    replacement = match.group("prefix") + indent + "resolution -1\n"
    return source[: match.start()] + replacement + source[match.end() :]


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
    if not re.match(r'\s*name "camera_s"', source[closing + 1 :]):
        raise ValueError("Children field does not belong to camera_s")
    names = re.findall(r"\bDEF\s+([A-Za-z0-9_]+)\s+\w+", source[opening + 1 : closing])
    expected = ["LENS", "STRUCTURE_AXES", "CAMERA_SHAPE", "CAMERA_OUTLINE"]
    if sorted(names) != sorted(expected):
        raise ValueError("Unexpected camera visual children; refusing to remove them")
    return source[: children.start()] + "      children []" + source[closing + 1 :]


def make_pilot_world(
    source: str,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    camera_tilt_rad: float = -0.5236,
    *,
    trial_id: str,
    strip_camera_debug: bool = False,
) -> str:
    """Create the shared test world with explicit experiment and trial labels."""
    if not all(math.isfinite(value) for value in (x, z, yaw, camera_tilt_rad)):
        raise ValueError("Pose and camera angle must contain finite numbers")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", trial_id):
        raise ValueError("Invalid trial ID")

    poses = list(ROBOT_POSE.finditer(source))
    if len(poses) != 1:
        raise ValueError(f"Expected exactly one Pioneer2 node; found {len(poses)}")
    match = poses[0]
    pose = (
        f"{match.group('header')}"
        f"  translation {x:g} 0 {z:g}\n"
        f"  rotation 0 1 0 {yaw:g}\n"
    )
    result = source[: match.start()] + pose + source[match.end() :]
    result = _set_gps_resolution(result)
    result = _replace_one(
        result,
        SPHERICAL_ROTATION,
        f"1 0 0 {camera_tilt_rad:g}",
        "spherical camera rotation",
    )
    if strip_camera_debug:
        result = _strip_camera_debug_geometry(result)

    controllers = list(ROBOT_CONTROLLER.finditer(result))
    if len(controllers) != 1 or re.search(r"(?m)^\s*controllerArgs\b", result):
        raise ValueError("Expected one fool_controller and no controllerArgs")
    controller = controllers[0]
    controller_line = f'{controller.group("indent")}controller "experiment_test_controller"'
    result = result[: controller.start()] + controller_line + result[controller.end() :]
    controller_end = controller.start() + len(controller_line)
    args_line = f'\n{controller.group("indent")}controllerArgs "experiment=002;trial={trial_id}"'
    return result[:controller_end] + args_line + result[controller_end:]


def generate_test_world(
    source_path: Path,
    output_path: Path,
    *,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    camera_tilt_rad: float = -0.5236,
    trial_id: str,
    strip_camera_debug: bool = False,
    overwrite: bool = False,
) -> Path:
    source_path = source_path.resolve()
    output_path = output_path.resolve()
    if source_path == output_path:
        raise ValueError("Refusing to overwrite the source world")
    content = make_pilot_world(
        source_path.read_text(encoding="utf-8"),
        x,
        z,
        yaw,
        camera_tilt_rad,
        trial_id=trial_id,
        strip_camera_debug=strip_camera_debug,
    )
    if output_path.exists() and output_path.read_text(encoding="utf-8") != content and not overwrite:
        raise FileExistsError(f"World differs from generated version: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not output_path.exists() or output_path.read_text(encoding="utf-8") != content:
        output_path.write_text(content, encoding="utf-8")
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Regenerate the single shared Webots test world from scenario 01"
    )
    parser.add_argument("--source", type=Path, default=Path("worlds/final_scenario01.wbt"))
    parser.add_argument(
        "--output", type=Path, default=Path("worlds/experiment_test_scenario01.wbt")
    )
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=-8.8)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--camera-tilt-rad", type=float, default=-0.5236)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--strip-camera-debug", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    try:
        output = generate_test_world(
            args.source,
            args.output,
            x=args.x,
            z=args.z,
            yaw=args.yaw,
            camera_tilt_rad=args.camera_tilt_rad,
            trial_id=args.trial_id,
            strip_camera_debug=args.strip_camera_debug,
            overwrite=args.overwrite,
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
