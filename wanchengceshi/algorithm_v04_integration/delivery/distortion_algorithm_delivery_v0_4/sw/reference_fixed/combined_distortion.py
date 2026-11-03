"""Combined integer model and the 20-word frame configuration contract.

Intermediate integers stay unbounded for range analysis. Reject any config
whose scan exceeds the candidate RTL widths before using it on hardware.
"""

from dataclasses import dataclass

from sw.reference_common.geometry import RemapConfig
from sw.reference_common.sampling import BorderMode, RGBImage, validate_rgb_image
from sw.reference_fixed.basic_distortion import (
    FixedPointSpec, FixedCoordinate, _prepare_fixed_camera_terms,
    _source_coordinate_fixed, quantize_signed, _saturate_signed,
    _bilinear_sample_rgb_fixed_validated,
)


CONFIG_WORD_NAMES = (
    "source_width", "source_height", "fx_q16", "fy_q16", "cx_q16", "cy_q16",
    "inv_fx_q24", "inv_fy_q24", "k1_q24", "k2_q24", "p1_q24", "p2_q24",
    "a00_q16", "a01_q16", "a02_q16", "a10_q16", "a11_q16", "a12_q16",
    "output_width", "output_height",
)


@dataclass(frozen=True)
class CombinedCoordinate:
    coordinate: FixedCoordinate
    border: int  # 0: interior; 1: edge taps need clamp; 2: outside/invalid
    affine_overflow: bool


def _validate_combined_spec(spec: FixedPointSpec) -> None:
    if spec.internal_frac_bits != 16 or spec.coefficient_frac_bits != 24:
        raise ValueError("combined RTL model requires internal Q16 and coefficient Q24")


def config_words(config: RemapConfig) -> tuple[int, ...]:
    terms = _prepare_fixed_camera_terms(config.profile, FixedPointSpec())
    dimensions = (config.profile.width, config.profile.height,
                  config.output_width, config.output_height)
    if any(d > 32768 for d in dimensions):
        raise ValueError("candidate RTL dimensions must be in 1..32768")
    values = (dimensions[0], dimensions[1], terms.fx_q, terms.fy_q,
              terms.cx_q, terms.cy_q, terms.inv_fx_q, terms.inv_fy_q,
              terms.k1_q, terms.k2_q, terms.p1_q, terms.p2_q,
              *(quantize_signed(a, 16) for a in config.affine.values()),
              dimensions[2], dimensions[3])
    for name, raw in zip(CONFIG_WORD_NAMES, values):
        if not -(1 << 31) <= raw < (1 << 31):
            raise ValueError(f"{name} exceeds signed 32-bit input width")
    if min(values[2], values[3], values[6], values[7]) <= 0:
        raise ValueError("quantized focal lengths and reciprocals must be positive")
    if values[12] * values[16] - values[13] * values[15] == 0:
        raise ValueError("quantized affine matrix is singular")
    return values


def pack_config(config: RemapConfig) -> int:
    """Word 0 occupies payload[31:0]; each word is two's-complement uint32."""
    return sum((word & 0xffffffff) << (32 * i)
               for i, word in enumerate(config_words(config)))


def combined_coordinate_fixed(u: int, v: int, config: RemapConfig,
                              spec: FixedPointSpec = FixedPointSpec(), *,
                              trace: dict[str, int] | None = None) -> CombinedCoordinate:
    _validate_combined_spec(spec)
    if not (isinstance(u, int) and isinstance(v, int) and 0 <= u <= 65535 and 0 <= v <= 65535):
        raise ValueError("raster request coordinates must be unsigned 16-bit integers")
    return _combined_coordinate_fixed(u, v, config, spec, config_words(config)[12:18],
                                     _prepare_fixed_camera_terms(config.profile, spec), trace=trace)


def _combined_coordinate_fixed(u, v, config, spec, affine, terms, *, trace=None):
    px = affine[0] * u + affine[1] * v + affine[2]
    py = affine[3] * u + affine[4] * v + affine[5]
    ix, ox = _saturate_signed(px, 32)
    iy, oy = _saturate_signed(py, 32)
    if ox or oy:
        # Invalid requests are never fetched; use a deterministic safe input
        # to the lens pipeline instead of propagating an overflowing geometry.
        ix = iy = 0
    stages: dict[str, int] = {}
    coord = _source_coordinate_fixed(ix, iy, terms,
                                     trace=stages, input_is_q16=True)
    profile = config.profile
    sx, sy = stages["src_x_q16"], stages["src_y_q16"]
    xmax, ymax = (profile.width - 1) << 16, (profile.height - 1) << 16
    outside = (ox or oy or coord.saturated_x or coord.saturated_y
               or sx < 0 or sy < 0 or sx > xmax or sy > ymax)
    border = 2 if outside else (1 if sx >= xmax or sy >= ymax else 0)
    if trace is not None:
        trace.update(stages)
        trace.update(affine_x_q16_wide=px, affine_y_q16_wide=py,
                     ideal_x_q16=ix, ideal_y_q16=iy)
    return CombinedCoordinate(coord, border, ox or oy)


def warp_rgb_combined_fixed(image: RGBImage, config: RemapConfig,
                            spec: FixedPointSpec = FixedPointSpec(), *,
                            border: BorderMode = BorderMode.BLACK):
    _validate_combined_spec(spec)
    height, width = validate_rgb_image(image)
    if (width, height) != (config.profile.width, config.profile.height):
        raise ValueError("image resolution must match the camera profile")
    output = []
    affine = config_words(config)[12:18]
    terms = _prepare_fixed_camera_terms(config.profile, spec)
    for v in range(config.output_height):
        row = []
        for u in range(config.output_width):
            result = _combined_coordinate_fixed(u, v, config, spec, affine, terms)
            coord = result.coordinate
            row.append((0, 0, 0) if result.affine_overflow else
                       _bilinear_sample_rgb_fixed_validated(
                           image, width, height, coord.src_x, coord.src_y,
                           coord.frac_bits, border))
        output.append(row)
    return output
