"""Integer boundary sampling and masked RGB Sobel reference for v0.4."""
from dataclasses import dataclass

from sw.reference_common.geometry import RemapConfig
from sw.reference_common.sampling import BorderMode, validate_rgb_image
from sw.reference_fixed.basic_distortion import FixedPointSpec, _prepare_fixed_camera_terms, _bilinear_sample_rgb_fixed_validated
from sw.reference_fixed.combined_distortion import config_words, _combined_coordinate_fixed, _validate_combined_spec


@dataclass(frozen=True)
class EdgeConfig:
    remap: RemapConfig
    border_mode: BorderMode = BorderMode.BLACK
    threshold: int = 128
    mode: int = 0

    def __post_init__(self):
        if not isinstance(self.border_mode, BorderMode):
            raise ValueError('border_mode must be BorderMode.BLACK or CLAMP')
        if isinstance(self.threshold, bool) or not isinstance(self.threshold, int) or not 0 <= self.threshold <= 2047:
            raise ValueError('threshold must be an integer in 0..2047')
        if isinstance(self.mode, bool) or not isinstance(self.mode, int) or not 0 <= self.mode <= 3:
            raise ValueError('mode must be an integer in 0..3')


def edge_config_words(config):
    return (*config_words(config.remap), int(config.border_mode == BorderMode.CLAMP), config.threshold, config.mode)


def pack_edge_config(config):
    return sum((word & 0xffffffff) << (32*i) for i, word in enumerate(edge_config_words(config)))


@dataclass(frozen=True)
class SamplingDescriptor:
    x0: int
    x1: int
    y0: int
    y1: int
    dx: int
    dy: int
    sample_enable: bool
    outside: bool
    boundary: bool


def sampling_descriptor(sx, sy, frac_bits, width, height, *, border=0, invalid=False, clamp=False):
    if not 1 <= frac_bits <= 16:
        raise ValueError('frac_bits must be 1..16')
    invalid_dims = not (1 <= width <= 32768 and 1 <= height <= 32768)
    scale = 1 << frac_bits
    xmax, ymax = (width-1)*scale, (height-1)*scale
    outside = border == 2 or sx < 0 or sy < 0 or sx > xmax or sy > ymax
    enabled = not invalid and not invalid_dims and (clamp or not outside)
    if not enabled:
        return SamplingDescriptor(0, 0, 0, 0, 0, 0, False, outside or invalid or invalid_dims, False)
    cx, cy = min(max(sx, 0), xmax), min(max(sy, 0), ymax)
    x0, dx = divmod(cx, scale)
    y0, dy = divmod(cy, scale)
    return SamplingDescriptor(x0, min(x0+1, width-1), y0, min(y0+1, height-1), dx, dy,
                              True, outside, outside or x0 >= width-1 or y0 >= height-1)


def grayscale(rgb):
    return (77*rgb[0]+150*rgb[1]+29*rgb[2]+128) >> 8


@dataclass(frozen=True)
class EdgePixel:
    rgb: tuple[int, int, int]
    gx: int
    gy: int
    magnitude: int
    edge: bool
    window_valid: bool
    sample_valid: bool


def sobel_rgb(image, valid_mask=None, *, threshold=128, mode=0):
    height, width = validate_rgb_image(image)
    if not isinstance(threshold, int) or isinstance(threshold, bool) or not 0 <= threshold <= 2047:
        raise ValueError('threshold must be 0..2047')
    if not isinstance(mode, int) or isinstance(mode, bool) or not 0 <= mode <= 3:
        raise ValueError('mode must be 0..3')
    if valid_mask is None:
        valid_mask = [[True]*width for _ in range(height)]
    if len(valid_mask) != height or any(len(row) != width for row in valid_mask):
        raise ValueError('valid mask dimensions must match image')
    gray = [[grayscale(pixel) for pixel in row] for row in image]
    output = []
    for y in range(height):
        row = []
        for x in range(width):
            yy = [min(max(y+d, 0), height-1) for d in (-1, 0, 1)]
            xx = [min(max(x+d, 0), width-1) for d in (-1, 0, 1)]
            p = [[gray[v][u] for u in xx] for v in yy]
            mask = all(valid_mask[v][u] for v in yy for u in xx)
            gx = p[0][2]+2*p[1][2]+p[2][2]-p[0][0]-2*p[1][0]-p[2][0] if mask else 0
            gy = p[2][0]+2*p[2][1]+p[2][2]-p[0][0]-2*p[0][1]-p[0][2] if mask else 0
            magnitude = abs(gx)+abs(gy)
            edge = magnitude != 0 and magnitude >= threshold
            strength = min(255, (magnitude+2) >> 2)
            rgb = tuple(image[y][x])
            if mode == 1:
                rgb = (strength,)*3
            elif mode == 2:
                rgb = (255,)*3 if edge else (0,)*3
            elif mode == 3 and edge:
                rgb = (255, 0, 0)
            row.append(EdgePixel(rgb, gx, gy, magnitude, edge, bool(mask), bool(valid_mask[y][x])))
        output.append(row)
    return output


def warp_rgb_edges(image, config: EdgeConfig, spec=FixedPointSpec()):
    _validate_combined_spec(spec)
    height, width = validate_rgb_image(image)
    remap = config.remap
    if (width, height) != (remap.profile.width, remap.profile.height):
        raise ValueError('source resolution must match camera profile')
    terms = _prepare_fixed_camera_terms(remap.profile, spec)
    affine = config_words(remap)[12:18]
    warped, mask, descriptors = [], [], []
    for y in range(remap.output_height):
        rgb_row, mask_row, desc_row = [], [], []
        for x in range(remap.output_width):
            result = _combined_coordinate_fixed(x, y, remap, spec, affine, terms)
            coordinate = result.coordinate
            desc = sampling_descriptor(coordinate.src_x, coordinate.src_y, spec.coordinate_frac_bits, width, height,
                                       border=result.border, invalid=result.affine_overflow or coordinate.saturated_x or coordinate.saturated_y,
                                       clamp=config.border_mode == BorderMode.CLAMP)
            if desc.sample_enable:
                pixel = _bilinear_sample_rgb_fixed_validated(image, width, height,
                    (desc.x0 << spec.coordinate_frac_bits)+desc.dx, (desc.y0 << spec.coordinate_frac_bits)+desc.dy,
                    spec.coordinate_frac_bits, BorderMode.CLAMP)
            else:
                pixel = (0, 0, 0)
            rgb_row.append(pixel); mask_row.append(desc.sample_enable); desc_row.append(desc)
        warped.append(rgb_row); mask.append(mask_row); descriptors.append(desc_row)
    return sobel_rgb(warped, mask, threshold=config.threshold, mode=config.mode), warped, mask, descriptors
