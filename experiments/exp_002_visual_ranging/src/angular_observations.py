"""EXP-002 V3: image-column observations for a Webots spherical camera.

A metric range is emitted ONLY for a verified self-occlusion mask and a
contiguous visible FLOOR -> OBSTACLE boundary. No detection is NOT free space.
The HSV classes are provisional and require calibration on independent images.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import degrees, isfinite
from typing import Literal

import cv2
import numpy as np

from spherical_geometry import SphericalCameraModel


@dataclass(frozen=True)
class AngularConfig:
    sector_count: int = 16
    samples_per_sector: int = 3
    min_floor_run: int = 5
    min_object_run: int = 3
    max_boundary_spread_px: int = 5
    max_range_spread_m: float = 0.25
    min_range_m: float = 0.2
    max_range_m: float = 4.0
    max_floor_value: int = 235
    min_floor_value: int = 35
    floor_max_saturation: int = 50
    object_min_saturation: int = 65
    object_min_value: int = 55

    def __post_init__(self) -> None:
        if (self.sector_count < 1 or self.samples_per_sector < 1
                or self.min_floor_run < 1 or self.min_object_run < 1
                or self.max_boundary_spread_px < 0
                or not 0 < self.min_range_m < self.max_range_m
                or not 0 <= self.max_range_spread_m < self.max_range_m):
            raise ValueError("Invalid sector/range configuration")


@dataclass(frozen=True)
class SectorObservation:
    sector_id: int
    camera_angle_deg: float
    status: Literal["occupied", "unknown"]
    distance_m: float | None
    confidence: float
    contact_y_px: float | None
    bearing_robot_deg: float | None
    reason: str | None
    near_field_occluded: bool

    def as_record(self) -> dict:
        return asdict(self)


def classify_pixels(image_bgr: np.ndarray, mask: np.ndarray, config: AngularConfig) -> np.ndarray:
    """Return 0=unknown, 1=floor candidate, 2=cyan obstacle, 3=green victim.

    Floor pixels must belong to a gray component touching either the unmasked
    bottom strip or the visible edge of a self-occlusion mask. The latter
    supports camera views where the robot body occupies the image bottom.
    """
    if image_bgr.dtype != np.uint8 or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("Expected uint8 BGR image")
    height, width = image_bgr.shape[:2]
    if mask.dtype != np.bool_ or mask.shape != (height, width):
        raise ValueError("Occlusion mask must be boolean and image-sized")
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue, sat, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    floor = ((sat <= config.floor_max_saturation)
             & (value >= config.min_floor_value) & (value <= config.max_floor_value)
             & ~mask)
    eligible_obj = (sat >= config.object_min_saturation) & (value >= config.object_min_value) & ~mask
    cyan = eligible_obj & (hue >= 77) & (hue <= 108)
    green = eligible_obj & (hue >= 36) & (hue <= 76)
    labels = np.zeros((height, width), dtype=np.uint8)
    labels[cyan & ~green] = 2
    labels[green & ~cyan] = 3
    # Keep gray components connected to a known image anchor. The image bottom
    # is the legacy anchor; with the robot masked there, use gray pixels
    # immediately adjacent to the robot silhouette instead. This only creates
    # floor candidates: visual review is still required before metric use.
    count, components = cv2.connectedComponents(floor.astype(np.uint8), connectivity=8)
    bottom = max(1, round(height * .08))
    bottom_anchor = np.zeros((height, width), dtype=bool)
    bottom_anchor[-bottom:, :] = True
    if np.any(mask):
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask_edge = (cv2.dilate(mask.astype(np.uint8), kernel, iterations=1) != 0) & ~mask
    else:
        mask_edge = np.zeros((height, width), dtype=bool)
    seed = floor & (bottom_anchor | mask_edge)
    seeded_components = np.unique(components[seed])
    for component in range(1, count):
        if np.any(seeded_components == component):
            labels[components == component] = 1
    labels[mask] = 0
    return labels


def _contact_row(labels: np.ndarray, mask: np.ndarray, x: int, cfg: AngularConfig):
    """Read one vertical image column from bottom upwards (near to far).

    Masked pixels stay unknown. A mask below a visible floor run leaves near
    field unknown but does not hide a farther floor-to-obstacle boundary. A
    mask between that floor run and the boundary makes the contact unknown.
    """
    floor_run = 0
    skipped_mask = False
    height = labels.shape[0]
    for y in range(height - 1, -1, -1):
        if mask[y, x]:
            if floor_run:
                return None, "occlusion_after_floor", True
            skipped_mask = True
            continue
        label = int(labels[y, x])
        if label == 1:
            floor_run += 1
            continue
        if floor_run < cfg.min_floor_run:
            return None, "no_visible_floor", skipped_mask
        if label not in (2, 3):
            return None, "ambiguous_transition", skipped_mask
        top = max(0, y - cfg.min_object_run + 1)
        if (y - top + 1 < cfg.min_object_run
                or not np.all(np.isin(labels[top:y + 1, x], (2, 3)))):
            return None, "unstable_boundary", skipped_mask
        return y, None, skipped_mask
    return None, "no_obstacle_boundary", skipped_mask


def observe_sectors(
    image_bgr: np.ndarray,
    model: SphericalCameraModel,
    *,
    occlusion_mask: np.ndarray | None = None,
    mask_verified: bool = False,
    config: AngularConfig | None = None,
) -> tuple[list[SectorObservation], np.ndarray]:
    """Estimate FIRST visible floor-contact ranges, never free space.

    If no validated self-occlusion mask exists, all ranges stay unknown.
    The classification mask is still returned for diagnosis.
    """
    cfg = config or AngularConfig()
    if image_bgr.shape[:2] != (model.height, model.width):
        raise ValueError("Image dimensions differ from the spherical camera model")
    if occlusion_mask is None:
        occlusion_mask = np.zeros(image_bgr.shape[:2], dtype=bool)
    labels = classify_pixels(image_bgr, occlusion_mask, cfg)
    results: list[SectorObservation] = []
    for index in range(cfg.sector_count):
        left = index * model.width / cfg.sector_count
        right = (index + 1) * model.width / cfg.sector_count
        center = (left + right) / 2 - 0.5
        angle_deg = degrees(model.image_angles(center, (model.height - 1) / 2)[0])

        def unknown(reason: str, near: bool = False) -> SectorObservation:
            return SectorObservation(index, angle_deg, "unknown", None, 0.0,
                                     None, None, reason, near)

        if not mask_verified:
            results.append(unknown("unverified_self_occlusion_mask"))
            continue
        # Keep sample columns strictly within the sector, including very
        # narrow sectors. Reject duplicated columns instead of false agreement.
        samples = np.linspace(left + .15 * (right - left),
                              right - .15 * (right - left), cfg.samples_per_sector)
        columns = [max(0, min(model.width - 1, int(round(float(x))))) for x in samples]
        if len(set(columns)) < len(columns):
            results.append(unknown("insufficient_sector_resolution"))
            continue
        contacts, distances, bearings, occluded = [], [], [], []
        reason = None
        for column in columns:
            row, error, near_occluded = _contact_row(labels, occlusion_mask, column, cfg)
            occluded.append(near_occluded)
            if error is not None:
                reason = error
                break
            contact = model.contact_on_ground(column, row)
            if contact is None:
                reason = "ray_does_not_intersect_ground"
                break
            distance = contact.horizontal_from_camera_m
            if not isfinite(distance) or not (cfg.min_range_m <= distance <= cfg.max_range_m):
                reason = "outside_calibrated_range"
                break
            contacts.append(row)
            distances.append(distance)
            bearings.append(degrees(contact.bearing_robot_rad))
        if reason:
            results.append(unknown(reason, any(occluded)))
        elif (max(contacts) - min(contacts) > cfg.max_boundary_spread_px
              or max(distances) - min(distances) > cfg.max_range_spread_m):
            results.append(unknown("inconsistent_sector_rays", any(occluded)))
        else:
            results.append(SectorObservation(
                index, angle_deg, "occupied",
                round(float(np.median(distances)), 4),
                0.5,  # heuristic only, not a calibrated probability
                round(float(np.median(contacts)), 2),
                round(float(np.median(bearings)), 2), None,
                any(occluded),
            ))
    return results, labels
