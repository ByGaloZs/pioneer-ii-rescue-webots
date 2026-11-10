"""Create an EXP-002 test world without changing the canonical WBT file.

This helper reads a .wbt only as an offline experiment setup step. Runtime
controllers and perception modules never inspect WBT geometry or world identity.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROBOT_POSE = re.compile(
    r"(?P<header>^Pioneer2\s*\{\s*\n)"
    r"(?P<translation>[ \t]*translation[ \t]+[^\n]+\n)"
    r"(?P<rotation>[ \t]*rotation[ \t]+[^\n]+\n)",
    re.MULTILINE,
)


def make_pilot_world(source: str, x: float = 0.0, z: float = -8.8, yaw: float = 0.0) -> str:
    """Replace just the robot pose, preserving the rest of the original file."""
    matches = list(ROBOT_POSE.finditer(source))
    if len(matches) != 1:
        raise ValueError("Expected exactly one Pioneer2 node with translation and rotation")
    match = matches[0]
    replacement = (
        f"{match['header']}"
        f"  translation {x:g} 0 {z:g}\n"
        f"  rotation 0 1 0 {yaw:g}\n"
    )
    return source[:match.start()] + replacement + source[match.end():]


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate isolated EXP-002 world for wall test")
    parser.add_argument("--source", type=Path, default=Path("worlds/final_scenario01.wbt"))
    parser.add_argument("--output", type=Path, default=Path("worlds/exp002_pilot_01.wbt"))
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=-8.8)
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error("Source and output must be different")
    if not args.source.is_file():
        parser.error(f"Missing source world: {args.source}")
    if args.output.exists():
        parser.error(f"Output already exists; do not overwrite it: {args.output}")
    # The specific first pilot position is already in the start strip. Other
    # positions must be verified offline for robot/obstacle overlap.
    world = make_pilot_world(args.source.read_text(encoding="utf-8"), args.x, args.z, args.yaw)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(world, encoding="utf-8")
    print(f"EXP-002 pilot world: {args.output} (x={args.x:g}, z={args.z:g}, yaw={args.yaw:g})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
