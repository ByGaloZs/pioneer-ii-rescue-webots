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


def _replace_one(source: str, pattern: re.Pattern[str], value: str, label: str) -> str:
    matches = list(pattern.finditer(source))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {label} definition; found {len(matches)}")
    match = matches[0]
    return source[: match.start()] + match.group("prefix") + value + source[match.end() :]


def make_pilot_world(
    source: str,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    camera_tilt_rad: float = -0.7854,
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
    return _replace_one(
        result, SPHERICAL_ROTATION, f"1 0 0 {camera_tilt_rad:g}", "spherical camera rotation"
    )


def generate_variants(
    source_path: Path,
    output_dir: Path,
    x: float = 0.0,
    z: float = -8.8,
    yaw: float = 0.0,
    overwrite: bool = False,
) -> list[Path]:
    """Create all three worlds; refuse to overwrite unrelated/edited files."""
    source_path = source_path.resolve()
    template = source_path.read_text(encoding="utf-8")
    output_dir.mkdir(parents=True, exist_ok=True)
    generated: list[Path] = []
    for variant in VARIANTS:
        output = output_dir / f"exp002_camera_01_{variant.key}.wbt"
        if output.resolve() == source_path:
            raise ValueError("Cannot overwrite canonical source world")
        content = make_pilot_world(template, x, z, yaw, variant.tilt_rad)
        if output.exists():
            current = output.read_text(encoding="utf-8")
            if current == content:
                generated.append(output)
                continue
            if not overwrite:
                raise FileExistsError(f"World differs from generated version: {output}")
        output.write_text(content, encoding="utf-8")
        generated.append(output)
    return generated


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate EXP-002 camera tilt comparison worlds")
    parser.add_argument("--source", type=Path, default=Path("worlds/final_scenario01.wbt"))
    parser.add_argument("--output-dir", type=Path, default=Path("worlds"))
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=-8.8)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--overwrite", action="store_true", help="Replace only generated pilot files")
    args = parser.parse_args()
    try:
        generated = generate_variants(
            args.source, args.output_dir, args.x, args.z, args.yaw, args.overwrite
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for path, variant in zip(generated, VARIANTS):
        print(f"{path} | camera_tilt_rad={variant.tilt_rad:g} | gps_resolution=-1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
