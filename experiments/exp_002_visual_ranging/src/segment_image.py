"""Conservative color segmentation and IMAGE-space radial boundaries for EXP-002.

Masks are heuristic, not calibrated probabilities or certified traversable areas.
A radial boundary is only a pixel measurement; it is NEVER a metric range.
Images with ambiguous floor, lighting, occlusions, or inconsistent rays yield
'unknown', not evidence of free space. This module never reads Webots geometry.

Usage:
    python segment_image.py path/to/frame_000000.jpg --output-dir /tmp/exp002_preview
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path

from self_occlusion import load_occlusion_mask

import cv2
import numpy as np

from camera_projection import CameraProjection


class PixelClass(IntEnum):
    UNKNOWN = 0
    FLOOR = 1
    OBSTACLE = 2
    VICTIM = 3  # A victim is a physical obstacle for navigation.


@dataclass(frozen=True)
class SegmentationConfig:
    # Preliminary HSV thresholds for the grey floor, cyan walls/boxes and green
    # cylinders in original .wbt worlds. These require real-image validation.
    floor_max_saturation: int = 50
    floor_min_value: int = 35
    floor_max_value: int = 235
    cyan_hue_range: tuple[int, int] = (77, 108)
    green_hue_range: tuple[int, int] = (36, 76)
    object_min_saturation: int = 65
    object_min_value: int = 55
    floor_seed_inner_radius_px: int = 3
    floor_seed_outer_radius_px: int = 65
    floor_min_seed_pixels: int = 12

    def __post_init__(self) -> None:
        if not 0 <= self.floor_max_saturation <= 255:
            raise ValueError("floor_max_saturation out of range")
        if not 0 <= self.floor_min_value <= self.floor_max_value <= 255:
            raise ValueError("Invalid floor value range")
        for hue_range in (self.cyan_hue_range, self.green_hue_range):
            if not 0 <= hue_range[0] <= hue_range[1] <= 179:
                raise ValueError("Hue ranges must be within OpenCV HSV 0..179")
        if not 0 <= self.object_min_saturation <= 255 or not 0 <= self.object_min_value <= 255:
            raise ValueError("Object saturation/value outside 0..255")
        if not 0 <= self.floor_seed_inner_radius_px < self.floor_seed_outer_radius_px:
            raise ValueError("Invalid floor seed annulus")
        if self.floor_min_seed_pixels < 1:
            raise ValueError("floor_min_seed_pixels must be positive")


@dataclass(frozen=True)
class SegmentationResult:
    labels: np.ndarray  # uint8 image with PixelClass values
    confidence: np.ndarray  # float32 heuristic evidence, NOT probability
    occluded: np.ndarray | None = None  # True means robot body hides this pixel

    def mask(self, pixel_class: PixelClass) -> np.ndarray:
        return self.labels == pixel_class


@dataclass(frozen=True)
class SectorBoundary:
    sector_id: int
    image_angle_deg: float  # 0 right, 90 down: NOT robot heading
    boundary_px: float | None
    confidence: float
    status: str  # "usable" or "unknown" (never "free")
    unknown_reason: str | None
    near_field_occluded: bool = False
    first_visible_floor_px: float | None = None


def segment_image(
    image_bgr: np.ndarray,
    config: SegmentationConfig | None = None,
    occlusion_mask: np.ndarray | None = None,
) -> SegmentationResult:
    """Label visible surfaces; keep all unrecognized/ambiguous pixels unknown.

    Candidate floor must touch a seed annulus OUTSIDE any occlusion mask.
    A masked region remains unknown, even if its color resembles floor.
    Neither candidate floor nor detected boundary establishes traversability.
    """
    config = config or SegmentationConfig()
    if (
        not isinstance(image_bgr, np.ndarray)
        or image_bgr.ndim != 3
        or image_bgr.shape[2] != 3
        or image_bgr.dtype != np.uint8
        or min(image_bgr.shape[:2]) < 3
    ):
        raise ValueError("Expected a non-empty uint8 BGR image with 3 channels")

    height, width = image_bgr.shape[:2]
    if occlusion_mask is None:
        occluded = np.zeros((height, width), dtype=bool)
    else:
        if (
            not isinstance(occlusion_mask, np.ndarray)
            or occlusion_mask.shape != (height, width)
            or occlusion_mask.dtype != np.bool_
        ):
            raise ValueError("occlusion_mask must be a boolean mask matching the image")
        occluded = occlusion_mask.copy()

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    floor_candidate = (~occluded) & (
        (saturation <= config.floor_max_saturation)
        & (value >= config.floor_min_value)
        & (value <= config.floor_max_value)
    )
    object_candidate = (~occluded) & (
        (saturation >= config.object_min_saturation)
        & (value >= config.object_min_value)
    )
    cyan = object_candidate & (
        (hue >= config.cyan_hue_range[0])
        & (hue <= config.cyan_hue_range[1])
    )
    green = object_candidate & (
        (hue >= config.green_hue_range[0])
        & (hue <= config.green_hue_range[1])
    )

    labels = np.full((height, width), PixelClass.UNKNOWN, dtype=np.uint8)
    confidence = np.zeros((height, width), dtype=np.float32)

    # Reject hue-overlap pixels instead of arbitrarily assigning cyan or green.
    unambiguous_cyan = cyan & ~green
    unambiguous_green = green & ~cyan
    labels[unambiguous_cyan] = PixelClass.OBSTACLE
    confidence[unambiguous_cyan] = 0.85
    labels[unambiguous_green] = PixelClass.VICTIM
    confidence[unambiguous_green] = 0.85

    # Start floor evidence in a broad annulus outside the excluded robot body.
    # Never infer floor in hidden pixels or disconnected grey components.
    _, connected = cv2.connectedComponents(floor_candidate.astype(np.uint8), connectivity=8)
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    yy, xx = np.ogrid[:height, :width]
    radius_squared = (xx - cx) ** 2 + (yy - cy) ** 2
    seed_region = (
        (radius_squared >= config.floor_seed_inner_radius_px**2)
        & (radius_squared <= config.floor_seed_outer_radius_px**2)
        & ~occluded
    )
    seeded_ids = connected[floor_candidate & seed_region]
    if seeded_ids.size:
        unique_ids, counts = np.unique(seeded_ids, return_counts=True)
        strongest = int(np.argmax(counts))
        if counts[strongest] >= config.floor_min_seed_pixels:
            main_floor = connected == unique_ids[strongest]
            labels[main_floor] = PixelClass.FLOOR
            confidence[main_floor] = 0.60

    return SegmentationResult(labels=labels, confidence=confidence, occluded=occluded)


def _detect_ray_boundary(
    segmentation: SegmentationResult,
    ray_samples: list[tuple[int, int, int]],
    min_floor_pixels: int,
    min_obstacle_pixels: int,
) -> tuple[int | None, float, str | None, bool, int | None]:
    """Scan past a strictly leading self-occlusion, then require visible floor.

    An obstacle boundary after the body can be measured in pixels, but the
    hidden near-field remains unknown and cannot be used to clear a route.
    """
    floor_run = 0
    first_floor_radius: int | None = None
    occluded_prefix = False
    occluded = segmentation.occluded
    for index, (radius, x, y) in enumerate(ray_samples):
        if occluded is not None and occluded[y, x]:
            if floor_run > 0:
                return None, 0.0, "occlusion_after_floor", True, first_floor_radius
            occluded_prefix = True
            continue
        label = segmentation.labels[y, x]
        if label == PixelClass.FLOOR:
            if first_floor_radius is None:
                first_floor_radius = radius
            floor_run += 1
            continue
        if label == PixelClass.UNKNOWN:
            return None, 0.0, "ambiguous_transition", occluded_prefix, first_floor_radius
        if floor_run < min_floor_pixels:
            return None, 0.0, "insufficient_visible_floor", occluded_prefix, first_floor_radius
        subsequent = ray_samples[index : index + min_obstacle_pixels]
        if len(subsequent) < min_obstacle_pixels or any(
            (occluded is not None and occluded[sy, sx])
            or segmentation.labels[sy, sx] not in (PixelClass.OBSTACLE, PixelClass.VICTIM)
            for _, sx, sy in subsequent
        ):
            return None, 0.0, "unstable_obstacle_boundary", occluded_prefix, first_floor_radius
        return (radius, float(min(0.60, segmentation.confidence[y, x])),
                None, occluded_prefix, first_floor_radius)
    return None, 0.0, "no_boundary_in_frame", occluded_prefix, first_floor_radius


def extract_radial_boundaries(
    segmentation: SegmentationResult,
    projection: CameraProjection,
    sector_count: int = 16,
    rays_per_sector: int = 3,
    start_radius_px: int = 8,
    min_floor_pixels: int = 4,
    min_obstacle_pixels: int = 2,
    max_ray_disagreement_px: int = 5,
) -> list[SectorBoundary]:
    """Detect first contiguous floor-to-object boundary for each IMAGE sector.

    All rays must agree. A KNOWN leading self-occlusion may be skipped only
    to measure the farther boundary; unknown pixels after visibility begins
    invalidate the ray. Usable does NOT mean the near field is traversable.
    """
    if segmentation.labels.shape != (projection.height, projection.width):
        raise ValueError("Segmentation dimensions do not match projection")
    if segmentation.confidence.shape != segmentation.labels.shape:
        raise ValueError("Confidence dimensions do not match labels")
    if sector_count < 1 or rays_per_sector < 1:
        raise ValueError("sector_count and rays_per_sector must be positive")
    if min_floor_pixels < 1 or min_obstacle_pixels < 1 or max_ray_disagreement_px < 0:
        raise ValueError("Invalid boundary detection parameters")

    observations: list[SectorBoundary] = []
    for sector_id in range(sector_count):
        rays = projection.sector_angles(sector_id, sector_count, rays_per_sector)
        radius_values: list[int] = []
        confidence_values: list[float] = []
        reasons: list[str] = []
        near_occlusions: list[bool] = []
        first_floor_values: list[int] = []
        for angle in rays:
            samples = projection.radial_pixels(angle, start_radius_px)
            boundary, confidence, reason, occluded_prefix, first_floor = _detect_ray_boundary(
                segmentation, samples, min_floor_pixels, min_obstacle_pixels
            )
            near_occlusions.append(occluded_prefix)
            if first_floor is not None:
                first_floor_values.append(first_floor)
            if boundary is None:
                reasons.append(reason or "unknown_observation")
            else:
                radius_values.append(boundary)
                confidence_values.append(confidence)

        image_angle_deg = sector_id * 360.0 / sector_count
        if reasons:
            observations.append(SectorBoundary(
                sector_id, image_angle_deg, None, 0.0, "unknown", reasons[0],
                any(near_occlusions), None
            ))
        elif max(radius_values) - min(radius_values) > max_ray_disagreement_px:
            observations.append(SectorBoundary(
                sector_id, image_angle_deg, None, 0.0, "unknown", "inconsistent_rays",
                any(near_occlusions), None
            ))
        else:
            observations.append(SectorBoundary(
                sector_id, image_angle_deg, float(np.median(radius_values)),
                float(min(confidence_values)), "usable", None,
                any(near_occlusions), float(max(first_floor_values))
            ))
    return observations


def render_overlay(
    image_bgr: np.ndarray,
    segmentation: SegmentationResult,
    boundaries: list[SectorBoundary] | None = None,
) -> np.ndarray:
    """Draw qualitative segmentation; these colors are not a traversability map."""
    if image_bgr.shape[:2] != segmentation.labels.shape:
        raise ValueError("Image and labels do not match")
    palette = np.array([
        [30, 30, 30],      # unknown: dark gray (BGR)
        [130, 130, 130],   # candidate floor: gray
        [220, 180, 30],    # cyan obstacle: blue-ish
        [40, 180, 40],     # victim: green
    ], dtype=np.uint8)
    visualization = cv2.addWeighted(image_bgr, 0.45, palette[segmentation.labels], 0.55, 0)
    if segmentation.occluded is not None and segmentation.occluded.any():
        mask = segmentation.occluded.astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(visualization, contours, -1, (255, 0, 255), 1)
    if boundaries:
        height, width = image_bgr.shape[:2]
        cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
        for boundary in boundaries:
            if boundary.status != "usable" or boundary.boundary_px is None:
                continue
            from math import cos, radians, sin
            angle = radians(boundary.image_angle_deg)
            x = round(cx + boundary.boundary_px * cos(angle))
            y = round(cy + boundary.boundary_px * sin(angle))
            visible_start = boundary.first_visible_floor_px or 0.0
            sx = round(cx + visible_start * cos(angle))
            sy = round(cy + visible_start * sin(angle))
            cv2.line(visualization, (sx, sy), (x, y), (0, 255, 255), 1)
            cv2.circle(visualization, (x, y), 3, (0, 0, 255), -1)
    return visualization


def main() -> int:
    parser = argparse.ArgumentParser(description="EXP-002 spherical visual diagnostics")
    parser.add_argument("image", type=Path, help="JPEG from Exp002Logger")
    parser.add_argument("--output-dir", type=Path, help="Optional directory for diagnostics")
    parser.add_argument("--mask-config", type=Path, help="Self-occlusion JSON mask")
    parser.add_argument("--mode", choices=("angular", "radial-legacy"), default="angular")
    parser.add_argument("--sectors", type=int, default=16)
    parser.add_argument("--tilt-deg", type=float, default=-30.0,
                        help="Experimental camera X-axis tilt in degrees (angular mode)")
    args = parser.parse_args()

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        parser.error(f"Cannot load image: {args.image}")

    if args.mode == "angular":
        from math import radians
        from angular_observations import AngularConfig, observe_sectors
        from spherical_geometry import SphericalCameraModel

        verified = False
        if args.mask_config is not None:
            config_data = json.loads(args.mask_config.read_text(encoding="utf-8"))
            verified = config_data.get("verified") is True
            occlusion_mask = load_occlusion_mask(args.mask_config, image.shape[:2])
        else:
            occlusion_mask = np.zeros(image.shape[:2], dtype=bool)
        model = SphericalCameraModel(
            width=image.shape[1], height=image.shape[0],
            tilt_rad=radians(args.tilt_deg),
        )
        observations, labels = observe_sectors(
            image, model, occlusion_mask=occlusion_mask,
            mask_verified=verified, config=AngularConfig(sector_count=args.sectors),
        )
        data = {
            "image_file": str(args.image),
            "mode": "angular",
            "self_occlusion_mask_verified": verified,
            "interpretation": "Geometry candidates only; not validated or free space",
            "sectors": [item.as_record() for item in observations],
        }
        print(json.dumps(data, indent=2))
        if args.output_dir is not None:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            palette = np.array([
                [20, 20, 20], [130, 130, 130],
                [230, 170, 25], [40, 180, 40],
            ], dtype=np.uint8)
            if not cv2.imwrite(str(args.output_dir / "angular_classes.png"), palette[labels]):
                raise RuntimeError("Unable to save angular_classes.png")
            if not cv2.imwrite(str(args.output_dir / "occlusion_mask.png"),
                               occlusion_mask.astype(np.uint8) * 255):
                raise RuntimeError("Unable to save occlusion_mask.png")
            (args.output_dir / "angular_sectors.json").write_text(
                json.dumps(data, indent=2) + "\n", encoding="utf-8"
            )
        return 0

    # Historical V2 comparison only: image radii are NOT physical bearings.
    if args.mask_config is None:
        parser.error("--mode radial-legacy requires --mask-config")
    occlusion_mask = load_occlusion_mask(args.mask_config, image.shape[:2])
    projection = CameraProjection(width=image.shape[1], height=image.shape[0])
    segmentation = segment_image(image, occlusion_mask=occlusion_mask)
    boundaries = extract_radial_boundaries(segmentation, projection, sector_count=args.sectors)
    data = {
        "image_file": str(args.image),
        "mode": "radial-legacy",
        "interpretation": "Image-space boundaries only; NO metric calibration",
        "sectors": [asdict(item) for item in boundaries],
    }
    print(json.dumps(data, indent=2))
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        overlay = render_overlay(image, segmentation, boundaries)
        if not cv2.imwrite(str(args.output_dir / "overlay.png"), overlay):
            raise RuntimeError("Unable to write overlay.png")
        color_mask = np.array([
            [30, 30, 30], [130, 130, 130],
            [220, 180, 30], [40, 180, 40],
        ], dtype=np.uint8)
        if not cv2.imwrite(str(args.output_dir / "mask.png"), color_mask[segmentation.labels]):
            raise RuntimeError("Unable to write mask.png")
        if not cv2.imwrite(str(args.output_dir / "occlusion_mask.png"),
                           occlusion_mask.astype(np.uint8) * 255):
            raise RuntimeError("Unable to write occlusion_mask.png")
        (args.output_dir / "boundaries.json").write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
