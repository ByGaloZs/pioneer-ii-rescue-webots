"""Explicit self-occlusion masks for offline EXP-002 spherical frames.

The mask is a manually configured approximation of the robot body, not a
calibrated camera model. Each masked pixel remains unobserved/unknown.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _coordinate(value: Any, name: str) -> float:
    if type(value) not in (int, float) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be a normalized number in [0, 1]")
    return float(value)


def _pair(value: Any, name: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must be a pair of normalized coordinates")
    return _coordinate(value[0], name), _coordinate(value[1], name)


def make_occlusion_mask(config: dict[str, Any], image_shape: tuple[int, int]) -> np.ndarray:
    """Rasterize explicit normalized ellipses/polygons to a boolean mask.

    Uses normalized coordinates so a configured mask scales with image size.
    This scaling is only a geometric convenience; recheck masks for every
    camera mounting position, image size, and camera configuration.
    """
    if len(image_shape) != 2 or any(type(n) is not int or n < 3 for n in image_shape):
        raise ValueError("image_shape must be (height, width) with both >= 3")
    if not isinstance(config, dict) or config.get("coordinate_system") != "normalized":
        raise ValueError("coordinate_system must be 'normalized'")
    if type(config.get("verified")) is not bool:
        raise ValueError("Mask config must declare verified=true or verified=false")
    regions = config.get("regions")
    if not isinstance(regions, list) or not regions:
        raise ValueError("At least one occlusion region is required")
    height, width = image_shape
    mask = np.zeros((height, width), dtype=np.uint8)

    def pixel(point: tuple[float, float]) -> tuple[int, int]:
        return round(point[0] * (width - 1)), round(point[1] * (height - 1))

    for region in regions:
        if not isinstance(region, dict):
            raise ValueError("Each occlusion region must be an object")
        shape = region.get("shape")
        if shape == "ellipse":
            center = pixel(_pair(region.get("center"), "center"))
            rx, ry = _pair(region.get("radii"), "radii")
            if rx <= 0 or ry <= 0:
                raise ValueError("ellipse radii must be positive")
            radii = max(1, round(rx * width)), max(1, round(ry * height))
            cv2.ellipse(mask, center, radii, 0, 0, 360, 1, -1)
        elif shape == "polygon":
            points = region.get("points")
            if not isinstance(points, list) or len(points) < 3:
                raise ValueError("polygon requires at least three points")
            vertices = np.array([pixel(_pair(point, "point")) for point in points], np.int32)
            cv2.fillPoly(mask, [vertices], 1)
        else:
            raise ValueError(f"Unsupported occlusion region shape: {shape!r}")
    return mask.astype(bool)


def load_occlusion_mask(path: Path, image_shape: tuple[int, int]) -> np.ndarray:
    data = json.loads(path.read_text(encoding="utf-8"))
    return make_occlusion_mask(data, image_shape)
