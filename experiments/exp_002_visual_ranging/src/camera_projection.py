"""Pixel geometry for Webots spherical-camera EXP-002 observations.

Webots describes spherical pixels with horizontal/vertical angular coordinates
(s, t). The radial scans below are image-space probes ONLY: their radii are not
metric distances, and an image angle is not a robot/world bearing. Calibration
and camera-to-robot extrinsics must be verified separately on real captures.

Source: https://www.cyberbotics.com/doc/reference/camera
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, floor, isfinite, pi, sin


@dataclass(frozen=True)
class CameraProjection:
    width: int = 160
    height: int = 160
    horizontal_fov_rad: float = 3.1415
    vertical_fov_rad: float | None = None

    def __post_init__(self) -> None:
        if self.width < 3 or self.height < 3:
            raise ValueError("Camera dimensions must be at least 3 pixels")
        if not isfinite(self.horizontal_fov_rad) or not 0 < self.horizontal_fov_rad <= pi:
            raise ValueError("Expected a spherical horizontal FOV in (0, pi]")
        if self.vertical_fov_rad is not None and (
            not isfinite(self.vertical_fov_rad)
            or not 0 < self.vertical_fov_rad <= pi
        ):
            raise ValueError("Expected a vertical FOV in (0, pi]")

    @property
    def center(self) -> tuple[float, float]:
        return (self.width - 1) / 2.0, (self.height - 1) / 2.0

    @property
    def vertical_fov(self) -> float:
        if self.vertical_fov_rad is not None:
            return self.vertical_fov_rad
        # Webots camera documentation: vFov = 2 atan(tan(hFov/2) * H/W).
        # atan2 is numerically stable when hFov approaches pi.
        half = self.horizontal_fov_rad / 2.0
        return 2.0 * atan2(sin(half) * self.height / self.width, cos(half))

    @property
    def max_inscribed_radius_px(self) -> int:
        """Largest integer-radius circle fully inside the rectangular image."""
        cx, cy = self.center
        return floor(min(cx, cy, self.width - 1 - cx, self.height - 1 - cy))

    def pixel_to_angles(self, x: float, y: float) -> tuple[float, float]:
        """Return (horizontal, vertical) angular coordinates in radians.

        Positive horizontal means image-left; positive vertical means image-up.
        These are camera coordinates, NOT ground-plane azimuth and elevation.
        """
        if not isfinite(x) or not isfinite(y) or not (
            0 <= x <= self.width - 1 and 0 <= y <= self.height - 1
        ):
            raise ValueError("Pixel is outside the image")
        cx, cy = self.center
        horizontal = (cx - x) * self.horizontal_fov_rad / (self.width - 1)
        vertical = (cy - y) * self.vertical_fov / (self.height - 1)
        return horizontal, vertical

    def angles_to_pixel(self, horizontal: float, vertical: float) -> tuple[float, float]:
        """Invert pixel_to_angles for angular coordinates within the image FOV."""
        if not isfinite(horizontal) or not isfinite(vertical) or (
            abs(horizontal) > self.horizontal_fov_rad / 2.0
            or abs(vertical) > self.vertical_fov / 2.0
        ):
            raise ValueError("Angles are outside the camera FOV")
        cx, cy = self.center
        x = cx - horizontal * (self.width - 1) / self.horizontal_fov_rad
        y = cy - vertical * (self.height - 1) / self.vertical_fov
        return x, y

    def sector_angles(
        self, sector_index: int, sector_count: int = 16, rays_per_sector: int = 3
    ) -> tuple[float, ...]:
        """Angular offsets in the IMAGE plane, radians (0 right, pi/2 down).

        Multiple rays sample the middle half of each sector. These image-plane
        directions require an empirical registration to physical bearings.
        """
        if sector_count < 1 or rays_per_sector < 1:
            raise ValueError("sector_count and rays_per_sector must be positive")
        if not 0 <= sector_index < sector_count:
            raise ValueError("sector_index outside sector_count")
        sector_width = 2.0 * pi / sector_count
        center_angle = sector_index * sector_width
        if rays_per_sector == 1:
            return (center_angle,)
        return tuple(
            center_angle + ((index / (rays_per_sector - 1)) - 0.5) * 0.5 * sector_width
            for index in range(rays_per_sector)
        )

    def radial_pixels(
        self,
        image_angle_rad: float,
        start_radius_px: int = 8,
        end_radius_px: int | None = None,
    ) -> list[tuple[int, int, int]]:
        """Return (radius_px, x, y) sampled along an image-space radial line."""
        if not isfinite(image_angle_rad):
            raise ValueError("Image angle must be finite")
        end = self.max_inscribed_radius_px if end_radius_px is None else end_radius_px
        if not 0 <= start_radius_px <= end <= self.max_inscribed_radius_px:
            raise ValueError("Invalid radial scan range")
        cx, cy = self.center
        samples: list[tuple[int, int, int]] = []
        for radius in range(start_radius_px, end + 1):
            x = round(cx + radius * cos(image_angle_rad))
            y = round(cy + radius * sin(image_angle_rad))
            if 0 <= x < self.width and 0 <= y < self.height:
                samples.append((radius, x, y))
        return samples
