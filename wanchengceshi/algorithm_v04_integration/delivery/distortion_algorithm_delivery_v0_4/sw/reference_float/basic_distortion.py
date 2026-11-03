"""Floating-point radial/tangential correction and RGB resampling reference.

The output-to-source map is deliberately limited to distortion correction.
There is no affine transform, rotation, crop, or scale in this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator, Literal

from sw.reference_common.camera_profile import CameraProfile
from sw.reference_common.sampling import (
    BorderMode,
    RGB,
    RGBImage,
    _sample_pixel_validated,
    floor_and_fraction,
    round_channel_half_up,
    validate_rgb_image,
)


@dataclass(frozen=True)
class CoordinateStages:
    x: float
    y: float
    x2: float
    y2: float
    xy: float
    r2: float
    r4: float
    k1_r2: float
    k2_r4: float
    gain: float
    radial_x: float
    radial_y: float
    tangential_x: float
    tangential_y: float
    xd: float
    yd: float
    src_x: float
    src_y: float


@dataclass(frozen=True)
class SourceCoordinate:
    src_x: float
    src_y: float
    in_source: bool
    four_taps_in_source: bool


def coordinate_stages(
    out_x: int | float,
    out_y: int | float,
    profile: CameraProfile,
) -> CoordinateStages:
    """Compute one output-pixel to distorted-source coordinate.

    Identity geometry is used: output and ideal-image coordinates coincide,
    and K_new equals the calibrated source K.
    """

    if not math.isfinite(out_x) or not math.isfinite(out_y):
        raise ValueError("output coordinates must be finite")

    x = (float(out_x) - profile.cx) / profile.fx
    y = (float(out_y) - profile.cy) / profile.fy
    x2 = x * x
    y2 = y * y
    xy = x * y
    r2 = x2 + y2
    r4 = r2 * r2
    k1_r2 = profile.k1 * r2
    k2_r4 = profile.k2 * r4
    gain = 1.0 + k1_r2 + k2_r4
    radial_x = x * gain
    radial_y = y * gain
    tangential_x = 2.0 * profile.p1 * xy + profile.p2 * (r2 + 2.0 * x2)
    tangential_y = profile.p1 * (r2 + 2.0 * y2) + 2.0 * profile.p2 * xy
    xd = radial_x + tangential_x
    yd = radial_y + tangential_y
    src_x = profile.fx * xd + profile.cx
    src_y = profile.fy * yd + profile.cy

    return CoordinateStages(
        x=x,
        y=y,
        x2=x2,
        y2=y2,
        xy=xy,
        r2=r2,
        r4=r4,
        k1_r2=k1_r2,
        k2_r4=k2_r4,
        gain=gain,
        radial_x=radial_x,
        radial_y=radial_y,
        tangential_x=tangential_x,
        tangential_y=tangential_y,
        xd=xd,
        yd=yd,
        src_x=src_x,
        src_y=src_y,
    )


def source_coordinate(
    out_x: int | float,
    out_y: int | float,
    profile: CameraProfile,
) -> SourceCoordinate:
    stages = coordinate_stages(out_x, out_y, profile)
    ix, _ = floor_and_fraction(stages.src_x)
    iy, _ = floor_and_fraction(stages.src_y)
    in_source = (
        0.0 <= stages.src_x <= profile.width - 1
        and 0.0 <= stages.src_y <= profile.height - 1
    )
    four_taps = (
        ix >= 0
        and iy >= 0
        and ix + 1 < profile.width
        and iy + 1 < profile.height
    )
    return SourceCoordinate(stages.src_x, stages.src_y, in_source, four_taps)


def iter_coordinate_map(
    profile: CameraProfile,
) -> Iterator[tuple[int, int, SourceCoordinate]]:
    """Yield (u, v, source-coordinate) in row-major output scan order."""

    for out_y in range(profile.height):
        for out_x in range(profile.width):
            yield out_x, out_y, source_coordinate(out_x, out_y, profile)


def bilinear_sample_rgb(
    image: RGBImage,
    src_x: float,
    src_y: float,
    border: BorderMode = BorderMode.BLACK,
) -> RGB:
    height, width = validate_rgb_image(image)
    if not math.isfinite(src_x) or not math.isfinite(src_y):
        raise ValueError("source coordinates must be finite")
    return _bilinear_sample_rgb_validated(image, width, height, src_x, src_y, border)


def _bilinear_sample_rgb_validated(
    image: RGBImage,
    width: int,
    height: int,
    src_x: float,
    src_y: float,
    border: BorderMode,
) -> RGB:
    if border is BorderMode.BLACK and (
        src_x < 0.0
        or src_x > width - 1
        or src_y < 0.0
        or src_y > height - 1
    ):
        return 0, 0, 0
    if border is BorderMode.CLAMP:
        src_x = min(max(src_x, 0.0), width - 1.0)
        src_y = min(max(src_y, 0.0), height - 1.0)

    ix, dx = floor_and_fraction(src_x)
    iy, dy = floor_and_fraction(src_y)

    # For in-range coordinates, clamping only affects a zero-weight tap at the
    # last row or column. It prevents undefined reads and a spurious edge line.
    p00 = _sample_pixel_validated(image, width, height, ix, iy, BorderMode.CLAMP)
    p10 = _sample_pixel_validated(image, width, height, ix + 1, iy, BorderMode.CLAMP)
    p01 = _sample_pixel_validated(image, width, height, ix, iy + 1, BorderMode.CLAMP)
    p11 = _sample_pixel_validated(
        image, width, height, ix + 1, iy + 1, BorderMode.CLAMP
    )

    result = []
    for channel in range(3):
        h0 = p00[channel] * (1.0 - dx) + p10[channel] * dx
        h1 = p01[channel] * (1.0 - dx) + p11[channel] * dx
        value = h0 * (1.0 - dy) + h1 * dy
        result.append(round_channel_half_up(value))
    return result[0], result[1], result[2]


def nearest_sample_rgb(
    image: RGBImage,
    src_x: float,
    src_y: float,
    border: BorderMode = BorderMode.BLACK,
) -> RGB:
    height, width = validate_rgb_image(image)
    if not math.isfinite(src_x) or not math.isfinite(src_y):
        raise ValueError("source coordinates must be finite")
    return _nearest_sample_rgb_validated(image, width, height, src_x, src_y, border)


def _nearest_sample_rgb_validated(
    image: RGBImage,
    width: int,
    height: int,
    src_x: float,
    src_y: float,
    border: BorderMode,
) -> RGB:
    if border is BorderMode.BLACK and (
        src_x < 0.0
        or src_x > width - 1
        or src_y < 0.0
        or src_y > height - 1
    ):
        return 0, 0, 0
    if border is BorderMode.CLAMP:
        src_x = min(max(src_x, 0.0), width - 1.0)
        src_y = min(max(src_y, 0.0), height - 1.0)
    # Explicit tie rule: floor(x + 0.5), including negative coordinates.
    ix = math.floor(src_x + 0.5)
    iy = math.floor(src_y + 0.5)
    return _sample_pixel_validated(image, width, height, ix, iy, BorderMode.CLAMP)


def warp_rgb(
    image: RGBImage,
    profile: CameraProfile,
    *,
    interpolation: Literal["nearest", "bilinear"] = "bilinear",
    border: BorderMode = BorderMode.BLACK,
) -> list[list[RGB]]:
    """Correct one RGB image using distortion-only inverse mapping."""

    height, width = validate_rgb_image(image)
    if (width, height) != (profile.width, profile.height):
        raise ValueError("image resolution must match the camera profile")
    if interpolation not in ("nearest", "bilinear"):
        raise ValueError("interpolation must be 'nearest' or 'bilinear'")

    output: list[list[RGB]] = []
    for out_y in range(height):
        row: list[RGB] = []
        for out_x in range(width):
            coord = source_coordinate(out_x, out_y, profile)
            if interpolation == "nearest":
                pixel = _nearest_sample_rgb_validated(
                    image, width, height, coord.src_x, coord.src_y, border
                )
            else:
                pixel = _bilinear_sample_rgb_validated(
                    image, width, height, coord.src_x, coord.src_y, border
                )
            row.append(pixel)
        output.append(row)
    return output
