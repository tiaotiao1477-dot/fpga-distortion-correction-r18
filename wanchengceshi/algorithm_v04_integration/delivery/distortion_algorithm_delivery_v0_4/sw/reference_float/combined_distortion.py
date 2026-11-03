"""Inverse affine plus lens distortion followed by a single RGB sample."""

from sw.reference_common.geometry import RemapConfig
from sw.reference_common.sampling import BorderMode, RGBImage, validate_rgb_image
from sw.reference_float.basic_distortion import (
    coordinate_stages, _bilinear_sample_rgb_validated, _nearest_sample_rgb_validated,
)


def combined_coordinate_stages(u: int | float, v: int | float, config: RemapConfig):
    ideal_x, ideal_y = config.affine.map(u, v)
    return coordinate_stages(ideal_x, ideal_y, config.profile)


def warp_rgb_combined(image: RGBImage, config: RemapConfig, *,
                      interpolation: str = "bilinear", border: BorderMode = BorderMode.BLACK):
    height, width = validate_rgb_image(image)
    if (width, height) != (config.profile.width, config.profile.height):
        raise ValueError("image resolution must match the camera profile")
    if interpolation not in ("nearest", "bilinear"):
        raise ValueError("interpolation must be 'nearest' or 'bilinear'")
    sample = (_bilinear_sample_rgb_validated if interpolation == "bilinear"
              else _nearest_sample_rgb_validated)
    output = []
    for v in range(config.output_height):
        row = []
        for u in range(config.output_width):
            coord = combined_coordinate_stages(u, v, config)
            row.append(sample(image, width, height, coord.src_x, coord.src_y, border))
        output.append(row)
    return output
