"""Spherical Webots camera rays and conservative ground-plane intersections.

Camera convention (Webots spherical/R2020b): pixel x increases right, image y
increases down; camera-local axes are X right, Y up, -Z forward. The camera
mount is a rotation about X, then Y (robot yaw). Geometry must be confirmed
with an off-axis landmark before use in a navigation occupancy map.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sin, sqrt


@dataclass(frozen=True)
class SphericalCameraModel:
    width: int = 160
    height: int = 160
    horizontal_fov_rad: float = 3.1415
    vertical_fov_rad: float | None = None
    height_m: float = 0.44
    tilt_rad: float = -0.5236
    offset_x_m: float = 0.025
    offset_z_m: float = -0.05
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.width < 3 or self.height < 3:
            raise ValueError("Image dimensions must be >= 3")
        if not 0 < self.horizontal_fov_rad <= pi:
            raise ValueError("Expected 0 < horizontal FOV <= pi for this pilot")
        for value in (self.horizontal_fov_rad, self.height_m, self.tilt_rad,
                      self.offset_x_m, self.offset_z_m, self.yaw_rad):
            if not isfinite(value):
                raise ValueError("Camera geometry must be finite")
        if self.height_m <= 0:
            raise ValueError("Camera height must be positive")
        if self.vertical_fov_rad is not None and not 0 < self.vertical_fov_rad <= pi:
            raise ValueError("Vertical FOV must be in (0, pi]")

    @property
    def vertical_fov(self) -> float:
        # For the square 160x160 spherical camera, Webots vertical FOV equals
        # horizontal FOV. For other sizes, use the spherical angular scaling.
        return (self.vertical_fov_rad if self.vertical_fov_rad is not None
                else self.horizontal_fov_rad * self.height / self.width)

    def image_angles(self, x: float, y: float) -> tuple[float, float]:
        if not isfinite(x) or not isfinite(y) or not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError("Pixel is outside the camera frame")
        azimuth = (x - (self.width - 1) / 2) * self.horizontal_fov_rad / (self.width - 1)
        elevation = ((self.height - 1) / 2 - y) * self.vertical_fov / (self.height - 1)
        return azimuth, elevation

    def robot_ray(self, x: float, y: float) -> tuple[float, float, float]:
        """Unit direction in robot frame (X right, Y up, -Z forward)."""
        a, e = self.image_angles(x, y)
        x_cam, y_cam, z_cam = cos(e) * sin(a), sin(e), -cos(e) * cos(a)
        y_tilt = cos(self.tilt_rad) * y_cam - sin(self.tilt_rad) * z_cam
        z_tilt = sin(self.tilt_rad) * y_cam + cos(self.tilt_rad) * z_cam
        x_robot = cos(self.yaw_rad) * x_cam + sin(self.yaw_rad) * z_tilt
        z_robot = -sin(self.yaw_rad) * x_cam + cos(self.yaw_rad) * z_tilt
        return x_robot, y_tilt, z_robot

    def contact_on_ground(self, x: float, y: float) -> "GroundContact | None":
        """Ray intersection with y=0; None if parallel/upwards/horizon unstable."""
        rx, ry, rz = self.robot_ray(x, y)
        if ry >= -1e-3:
            return None
        scale = self.height_m / -ry
        return GroundContact(
            x_robot_m=self.offset_x_m + scale * rx,
            z_robot_m=self.offset_z_m + scale * rz,
            horizontal_from_camera_m=scale * sqrt(rx * rx + rz * rz),
        )


@dataclass(frozen=True)
class GroundContact:
    x_robot_m: float
    z_robot_m: float
    horizontal_from_camera_m: float

    @property
    def horizontal_from_robot_m(self) -> float:
        return sqrt(self.x_robot_m**2 + self.z_robot_m**2)

    @property
    def bearing_robot_rad(self) -> float:
        return atan2(self.x_robot_m, -self.z_robot_m)
