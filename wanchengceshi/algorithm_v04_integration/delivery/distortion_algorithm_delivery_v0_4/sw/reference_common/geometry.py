"""Inverse pixel-space affine geometry; positive angles look counterclockwise."""

from __future__ import annotations

import math
from dataclasses import dataclass

from sw.reference_common.camera_profile import CameraProfile


@dataclass(frozen=True)
class InverseAffine:
    a00: float = 1.0
    a01: float = 0.0
    a02: float = 0.0
    a10: float = 0.0
    a11: float = 1.0
    a12: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in self.values()):
            raise ValueError("affine coefficients must be finite")
        if self.a00 * self.a11 - self.a01 * self.a10 == 0.0:
            raise ValueError("affine matrix must be nonsingular")

    def values(self) -> tuple[float, ...]:
        return self.a00, self.a01, self.a02, self.a10, self.a11, self.a12

    def map(self, u: float, v: float) -> tuple[float, float]:
        return (self.a00 * u + self.a01 * v + self.a02,
                self.a10 * u + self.a11 * v + self.a12)

    @classmethod
    def rotation(cls, angle_deg: float, width: int, height: int, *,
                 center: tuple[float, float] | None = None) -> "InverseAffine":
        """Map rotated output pixels back to the ideal image (x right, y down).

        The default center is ((width-1)/2, (height-1)/2), not the camera
        principal point. Specify center explicitly to rotate around another
        point. The source and output canvases have the same size here.
        """
        if not math.isfinite(angle_deg) or width <= 0 or height <= 0:
            raise ValueError("rotation requires a finite angle and positive dimensions")
        rx, ry = center if center is not None else ((width - 1) / 2, (height - 1) / 2)
        if not math.isfinite(rx) or not math.isfinite(ry):
            raise ValueError("rotation center must be finite")
        angle = math.radians(math.remainder(angle_deg, 360.0))
        c, s = math.cos(angle), math.sin(angle)
        # Snap cardinal angles to exact coefficients and avoid border noise.
        c = 0.0 if abs(c) < 1e-15 else c
        s = 0.0 if abs(s) < 1e-15 else s
        return cls(c, -s, rx - c * rx + s * ry,
                   s, c, ry - s * rx - c * ry)


@dataclass(frozen=True)
class RemapConfig:
    profile: CameraProfile
    affine: InverseAffine = InverseAffine()
    output_width: int | None = None
    output_height: int | None = None

    def __post_init__(self) -> None:
        if self.output_width is None:
            object.__setattr__(self, "output_width", self.profile.width)
        if self.output_height is None:
            object.__setattr__(self, "output_height", self.profile.height)
        for dimension in (self.output_width, self.output_height):
            if isinstance(dimension, bool) or not isinstance(dimension, int) or dimension <= 0:
                raise ValueError("output dimensions must be positive integers")
