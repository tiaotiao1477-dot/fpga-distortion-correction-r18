"""Parameterized integer reference model for radial/tangential correction.

The defaults are candidates for error analysis, not a frozen RTL bit-width
specification. Python keeps intermediate integers unbounded so range analysis
can determine safe hardware widths after real calibration data is available.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator

from sw.reference_common.camera_profile import CameraProfile
from sw.reference_common.sampling import (
    BorderMode,
    RGB,
    RGBImage,
    _sample_pixel_validated,
    validate_rgb_image,
)
from sw.reference_float.basic_distortion import coordinate_stages


@dataclass(frozen=True)
class FixedPointSpec:
    internal_frac_bits: int = 16
    coefficient_frac_bits: int = 24
    coordinate_width: int = 26
    coordinate_frac_bits: int = 10

    def __post_init__(self) -> None:
        if self.internal_frac_bits < 2:
            raise ValueError("internal_frac_bits must be at least 2")
        if self.coefficient_frac_bits < 2:
            raise ValueError("coefficient_frac_bits must be at least 2")
        if not 1 <= self.coordinate_frac_bits <= 16:
            raise ValueError("coordinate_frac_bits must be between 1 and 16")
        if not 2 <= self.coordinate_width <= 32:
            raise ValueError("coordinate_width must be between 2 and 32")
        if self.coordinate_width <= self.coordinate_frac_bits + 1:
            raise ValueError("coordinate_width must include a sign and integer bit")
        if self.coordinate_frac_bits > self.internal_frac_bits:
            raise ValueError(
                "coordinate_frac_bits cannot exceed internal_frac_bits in this model"
            )


@dataclass(frozen=True)
class FixedCoordinate:
    src_x: int
    src_y: int
    frac_bits: int
    saturated_x: bool = False
    saturated_y: bool = False

    @property
    def x_float(self) -> float:
        return self.src_x / (1 << self.frac_bits)

    @property
    def y_float(self) -> float:
        return self.src_y / (1 << self.frac_bits)


@dataclass(frozen=True)
class _FixedCameraTerms:
    profile: CameraProfile
    spec: FixedPointSpec
    fx_q: int
    fy_q: int
    cx_q: int
    cy_q: int
    inv_fx_q: int
    inv_fy_q: int
    k1_q: int
    k2_q: int
    p1_q: int
    p2_q: int


def quantize_signed(value: float, frac_bits: int) -> int:
    """Quantize with round-to-nearest, ties away from zero."""

    if not math.isfinite(value):
        raise ValueError("cannot quantize a non-finite value")
    if frac_bits < 0:
        raise ValueError("frac_bits cannot be negative")
    scale = 1 << frac_bits
    scaled = abs(value) * scale
    magnitude = math.floor(scaled + 0.5)
    return magnitude if value >= 0.0 else -magnitude


def round_shift_signed(value: int, shift: int) -> int:
    """Signed right shift with round-to-nearest, ties away from zero."""

    if shift < 0:
        return value << -shift
    if shift == 0:
        return value
    half = 1 << (shift - 1)
    magnitude = abs(value)
    rounded = (magnitude + half) >> shift
    return rounded if value >= 0 else -rounded


def multiply_q(a: int, b: int, frac_bits: int) -> int:
    return round_shift_signed(a * b, frac_bits)


def rescale_signed(value: int, from_frac: int, to_frac: int) -> int:
    return round_shift_signed(value, from_frac - to_frac)


def split_fixed_coordinate(value: int, frac_bits: int) -> tuple[int, int]:
    """Return mathematical floor and an unsigned fraction field."""

    if frac_bits < 0:
        raise ValueError("frac_bits cannot be negative")
    scale = 1 << frac_bits
    integer = value // scale
    return integer, value - integer * scale


def _saturate_signed(value: int, width: int) -> tuple[int, bool]:
    minimum = -(1 << (width - 1))
    maximum = (1 << (width - 1)) - 1
    clipped = min(max(value, minimum), maximum)
    return clipped, clipped != value


def _prepare_fixed_camera_terms(
    profile: CameraProfile,
    spec: FixedPointSpec,
) -> _FixedCameraTerms:
    f = spec.internal_frac_bits
    cf = spec.coefficient_frac_bits
    return _FixedCameraTerms(
        profile=profile,
        spec=spec,
        fx_q=quantize_signed(profile.fx, f),
        fy_q=quantize_signed(profile.fy, f),
        cx_q=quantize_signed(profile.cx, f),
        cy_q=quantize_signed(profile.cy, f),
        inv_fx_q=quantize_signed(1.0 / profile.fx, cf),
        inv_fy_q=quantize_signed(1.0 / profile.fy, cf),
        k1_q=quantize_signed(profile.k1, cf),
        k2_q=quantize_signed(profile.k2, cf),
        p1_q=quantize_signed(profile.p1, cf),
        p2_q=quantize_signed(profile.p2, cf),
    )


def _source_coordinate_fixed(
    out_x: int,
    out_y: int,
    terms: _FixedCameraTerms,
    *,
    trace: dict[str, int] | None = None,
    input_is_q16: bool = False,
) -> FixedCoordinate:
    profile = terms.profile
    spec = terms.spec
    f = spec.internal_frac_bits
    cf = spec.coefficient_frac_bits
    one = 1 << f

    delta_x_q16 = (out_x if input_is_q16 else out_x * one) - terms.cx_q
    delta_y_q16 = (out_y if input_is_q16 else out_y * one) - terms.cy_q
    normalized_x_product = delta_x_q16 * terms.inv_fx_q
    normalized_y_product = delta_y_q16 * terms.inv_fy_q
    x = multiply_q(delta_x_q16, terms.inv_fx_q, cf)
    y = multiply_q(delta_y_q16, terms.inv_fy_q, cf)
    x_square_product = x * x
    y_square_product = y * y
    xy_product = x * y
    x2 = multiply_q(x, x, f)
    y2 = multiply_q(y, y, f)
    xy = multiply_q(x, y, f)
    r2 = x2 + y2
    r4_product = r2 * r2
    r4 = multiply_q(r2, r2, f)
    k1_r2_product = terms.k1_q * r2
    k2_r4_product = terms.k2_q * r4
    k1_r2 = multiply_q(terms.k1_q, r2, cf)
    k2_r4 = multiply_q(terms.k2_q, r4, cf)
    gain_partial = one + k1_r2
    gain = gain_partial + k2_r4
    radial_x_product = x * gain
    radial_y_product = y * gain
    radial_x = multiply_q(x, gain, f)
    radial_y = multiply_q(y, gain, f)
    p1_xy_product = terms.p1_q * xy
    p2_xy_product = terms.p2_q * xy
    p2_x_base = r2 + 2 * x2
    p1_y_base = r2 + 2 * y2
    p1xy = multiply_q(terms.p1_q, xy, cf)
    p2xy = multiply_q(terms.p2_q, xy, cf)
    p2_x_product = terms.p2_q * p2_x_base
    p1_y_product = terms.p1_q * p1_y_base
    tangential_x = 2 * p1xy + multiply_q(terms.p2_q, p2_x_base, cf)
    tangential_y = multiply_q(terms.p1_q, p1_y_base, cf) + 2 * p2xy
    xd = radial_x + tangential_x
    yd = radial_y + tangential_y
    projection_x_product = terms.fx_q * xd
    projection_y_product = terms.fy_q * yd
    projected_x = multiply_q(terms.fx_q, xd, f)
    projected_y = multiply_q(terms.fy_q, yd, f)
    src_x_internal = projected_x + terms.cx_q
    src_y_internal = projected_y + terms.cy_q

    src_x_candidate = rescale_signed(
        src_x_internal, f, spec.coordinate_frac_bits
    )
    src_y_candidate = rescale_signed(
        src_y_internal, f, spec.coordinate_frac_bits
    )
    src_x, saturated_x = _saturate_signed(src_x_candidate, spec.coordinate_width)
    src_y, saturated_y = _saturate_signed(src_y_candidate, spec.coordinate_width)
    if trace is not None:
        trace.update(
            {
                "delta_x_q16": delta_x_q16,
                "delta_y_q16": delta_y_q16,
                "x_q16": x,
                "y_q16": y,
                "x2_q16": x2,
                "y2_q16": y2,
                "xy_q16": xy,
                "r2_q16": r2,
                "r4_q16": r4,
                "k1_r2_q16": k1_r2,
                "k2_r4_q16": k2_r4,
                "gain_partial_q16": gain_partial,
                "gain_q16": gain,
                "radial_x_q16": radial_x,
                "radial_y_q16": radial_y,
                "p1xy_q16": p1xy,
                "p2xy_q16": p2xy,
                "p2_x_base_q16": p2_x_base,
                "p1_y_base_q16": p1_y_base,
                "tangential_x_q16": tangential_x,
                "tangential_y_q16": tangential_y,
                "xd_q16": xd,
                "yd_q16": yd,
                "projected_x_q16": projected_x,
                "projected_y_q16": projected_y,
                "src_x_q16": src_x_internal,
                "src_y_q16": src_y_internal,
                f"src_x_q{spec.coordinate_frac_bits}_unclipped": src_x_candidate,
                f"src_y_q{spec.coordinate_frac_bits}_unclipped": src_y_candidate,
                f"src_x_q{spec.coordinate_frac_bits}": src_x,
                f"src_y_q{spec.coordinate_frac_bits}": src_y,
                "product_normalize_x": normalized_x_product,
                "product_normalize_y": normalized_y_product,
                "product_x2": x_square_product,
                "product_y2": y_square_product,
                "product_xy": xy_product,
                "product_r4": r4_product,
                "product_k1_r2": k1_r2_product,
                "product_k2_r4": k2_r4_product,
                "product_radial_x": radial_x_product,
                "product_radial_y": radial_y_product,
                "product_p1_xy": p1_xy_product,
                "product_p2_xy": p2_xy_product,
                "product_p2_x_base": p2_x_product,
                "product_p1_y_base": p1_y_product,
                "product_projection_x": projection_x_product,
                "product_projection_y": projection_y_product,
            }
        )
    return FixedCoordinate(
        src_x=src_x,
        src_y=src_y,
        frac_bits=spec.coordinate_frac_bits,
        saturated_x=saturated_x,
        saturated_y=saturated_y,
    )


def source_coordinate_fixed(
    out_x: int,
    out_y: int,
    profile: CameraProfile,
    spec: FixedPointSpec = FixedPointSpec(),
) -> FixedCoordinate:
    """Compute a fixed-point source coordinate with identity geometry."""

    return _source_coordinate_fixed(
        out_x, out_y, _prepare_fixed_camera_terms(profile, spec)
    )


def iter_fixed_coordinate_map(
    profile: CameraProfile,
    spec: FixedPointSpec = FixedPointSpec(),
) -> Iterator[tuple[int, int, FixedCoordinate]]:
    terms = _prepare_fixed_camera_terms(profile, spec)
    for out_y in range(profile.height):
        for out_x in range(profile.width):
            yield out_x, out_y, _source_coordinate_fixed(out_x, out_y, terms)


def coordinate_error_report(
    profile: CameraProfile,
    spec: FixedPointSpec = FixedPointSpec(),
    *,
    step: int = 1,
) -> dict[str, float | int | tuple[int, int]]:
    """Compare the candidate fixed model with the floating-point coordinate map."""

    if step <= 0:
        raise ValueError("step must be positive")
    count = 0
    sum_x2 = 0.0
    sum_y2 = 0.0
    errors: list[tuple[float, int, int]] = []
    worst_pixel = (0, 0)
    max_error = -1.0
    saturated_count = 0
    terms = _prepare_fixed_camera_terms(profile, spec)

    for out_y in range(0, profile.height, step):
        for out_x in range(0, profile.width, step):
            expected = coordinate_stages(out_x, out_y, profile)
            actual = _source_coordinate_fixed(out_x, out_y, terms)
            error_x = actual.x_float - expected.src_x
            error_y = actual.y_float - expected.src_y
            error = math.hypot(error_x, error_y)
            count += 1
            sum_x2 += error_x * error_x
            sum_y2 += error_y * error_y
            errors.append((error, out_x, out_y))
            if error > max_error:
                max_error = error
                worst_pixel = (out_x, out_y)
            saturated_count += int(actual.saturated_x or actual.saturated_y)

    if count == 0:
        raise ValueError("coordinate scan produced no samples")
    errors.sort(key=lambda item: item[0])
    p95_index = max(0, math.ceil(0.95 * count) - 1)
    return {
        "count": count,
        "rms_x": math.sqrt(sum_x2 / count),
        "rms_y": math.sqrt(sum_y2 / count),
        "p95_euclidean": errors[p95_index][0],
        "max_euclidean": max_error,
        "worst_pixel": worst_pixel,
        "saturated_count": saturated_count,
    }


def scan_float_intermediate_ranges(
    profile: CameraProfile,
    *,
    step: int = 1,
) -> dict[str, tuple[float, float]]:
    """Collect min/max values needed to select later RTL integer widths."""

    if step <= 0:
        raise ValueError("step must be positive")
    ranges: dict[str, list[float]] = {}
    for out_y in range(0, profile.height, step):
        for out_x in range(0, profile.width, step):
            stages = coordinate_stages(out_x, out_y, profile)
            values = {"u": float(out_x), "v": float(out_y), **stages.__dict__}
            for name, value in values.items():
                current = ranges.get(name)
                if current is None:
                    ranges[name] = [value, value]
                else:
                    current[0] = min(current[0], value)
                    current[1] = max(current[1], value)
    return {name: (limits[0], limits[1]) for name, limits in ranges.items()}


def bilinear_sample_rgb_fixed(
    image: RGBImage,
    src_x_fixed: int,
    src_y_fixed: int,
    frac_bits: int,
    border: BorderMode = BorderMode.BLACK,
) -> RGB:
    """Integer Fetch4 interpolation with one final round-half-up operation."""

    height, width = validate_rgb_image(image)
    return _bilinear_sample_rgb_fixed_validated(
        image, width, height, src_x_fixed, src_y_fixed, frac_bits, border
    )


def _bilinear_sample_rgb_fixed_validated(
    image: RGBImage,
    width: int,
    height: int,
    src_x_fixed: int,
    src_y_fixed: int,
    frac_bits: int,
    border: BorderMode,
) -> RGB:
    if frac_bits < 0:
        raise ValueError("frac_bits cannot be negative")
    scale = 1 << frac_bits
    max_x = (width - 1) * scale
    max_y = (height - 1) * scale
    if border is BorderMode.BLACK and (
        src_x_fixed < 0
        or src_x_fixed > max_x
        or src_y_fixed < 0
        or src_y_fixed > max_y
    ):
        return 0, 0, 0
    if border is BorderMode.CLAMP:
        src_x_fixed = min(max(src_x_fixed, 0), max_x)
        src_y_fixed = min(max(src_y_fixed, 0), max_y)

    ix, dx = split_fixed_coordinate(src_x_fixed, frac_bits)
    iy, dy = split_fixed_coordinate(src_y_fixed, frac_bits)

    p00 = _sample_pixel_validated(image, width, height, ix, iy, BorderMode.CLAMP)
    p10 = _sample_pixel_validated(image, width, height, ix + 1, iy, BorderMode.CLAMP)
    p01 = _sample_pixel_validated(image, width, height, ix, iy + 1, BorderMode.CLAMP)
    p11 = _sample_pixel_validated(
        image, width, height, ix + 1, iy + 1, BorderMode.CLAMP
    )
    denominator = scale * scale

    result = []
    for channel in range(3):
        h0 = p00[channel] * scale + (p10[channel] - p00[channel]) * dx
        h1 = p01[channel] * scale + (p11[channel] - p01[channel]) * dx
        numerator = h0 * scale + (h1 - h0) * dy
        value = (numerator + denominator // 2) // denominator
        result.append(min(255, max(0, value)))
    return result[0], result[1], result[2]


def warp_rgb_fixed(
    image: RGBImage,
    profile: CameraProfile,
    spec: FixedPointSpec = FixedPointSpec(),
    *,
    border: BorderMode = BorderMode.BLACK,
) -> list[list[RGB]]:
    height, width = validate_rgb_image(image)
    if (width, height) != (profile.width, profile.height):
        raise ValueError("image resolution must match the camera profile")
    terms = _prepare_fixed_camera_terms(profile, spec)
    output: list[list[RGB]] = []
    for out_y in range(height):
        row: list[RGB] = []
        for out_x in range(width):
            coord = _source_coordinate_fixed(out_x, out_y, terms)
            row.append(
                _bilinear_sample_rgb_fixed_validated(
                    image,
                    width,
                    height,
                    coord.src_x,
                    coord.src_y,
                    coord.frac_bits,
                    border,
                )
            )
        output.append(row)
    return output
