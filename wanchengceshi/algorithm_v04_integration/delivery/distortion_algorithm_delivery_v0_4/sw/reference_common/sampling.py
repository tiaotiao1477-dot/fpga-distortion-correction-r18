"""Reference image sampling and explicitly defined border behavior."""

from __future__ import annotations

import math
from enum import Enum
from typing import Sequence

RGB = tuple[int, int, int]
RGBImage = Sequence[Sequence[Sequence[int]]]


class BorderMode(str, Enum):
    BLACK = "black"
    CLAMP = "clamp"


def validate_rgb_image(image: RGBImage) -> tuple[int, int]:
    if not image:
        raise ValueError("image must contain at least one row")
    height = len(image)
    width = len(image[0])
    if width == 0:
        raise ValueError("image rows must contain at least one pixel")
    for row in image:
        if len(row) != width:
            raise ValueError("image rows must have a uniform width")
        for pixel in row:
            if len(pixel) != 3:
                raise ValueError("each pixel must contain exactly three RGB channels")
            for channel in pixel:
                if isinstance(channel, bool) or not isinstance(channel, int):
                    raise ValueError("RGB channels must be integers")
                if not 0 <= channel <= 255:
                    raise ValueError("RGB channels must be in the range 0..255")
    return height, width


def floor_and_fraction(value: float) -> tuple[int, float]:
    """Return mathematical floor and a fraction in [0, 1), including negatives."""

    if not math.isfinite(value):
        raise ValueError("sample coordinate must be finite")
    integer = math.floor(value)
    return integer, value - integer


def sample_pixel(
    image: RGBImage,
    x: int,
    y: int,
    border: BorderMode = BorderMode.BLACK,
) -> tuple[int, int, int]:
    height, width = validate_rgb_image(image)
    return _sample_pixel_validated(image, width, height, x, y, border)


def _sample_pixel_validated(
    image: RGBImage,
    width: int,
    height: int,
    x: int,
    y: int,
    border: BorderMode,
) -> tuple[int, int, int]:
    if border is BorderMode.CLAMP:
        x = min(max(x, 0), width - 1)
        y = min(max(y, 0), height - 1)
        pixel = image[y][x]
        return int(pixel[0]), int(pixel[1]), int(pixel[2])
    if x < 0 or x >= width or y < 0 or y >= height:
        return 0, 0, 0
    pixel = image[y][x]
    return int(pixel[0]), int(pixel[1]), int(pixel[2])


def round_channel_half_up(value: float) -> int:
    """Round a nonnegative channel value to nearest, with exact halves upward."""

    if not math.isfinite(value):
        raise ValueError("channel value must be finite")
    return min(255, max(0, math.floor(value + 0.5)))
