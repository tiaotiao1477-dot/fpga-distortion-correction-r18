"""Generate a camera-specific fixed-point range report and RTL vectors.

Run from the workspace root, for example:

    python -B -m sw.reference_fixed.rtl_compare --profile camera_profile.json

The resulting text vectors are consumed by rtl/remap/tb_coord_compare.v.
Candidate widths are reported for analysis; this tool does not freeze them.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_fixed.basic_distortion import (
    FixedPointSpec,
    _prepare_fixed_camera_terms,
    _source_coordinate_fixed,
)
from sw.reference_float.basic_distortion import coordinate_stages


STAGE_FORMATS: dict[str, tuple[int, int]] = {
    "delta_x_q16": (16, 32), "delta_y_q16": (16, 32),
    "x_q16": (16, 32), "y_q16": (16, 32),
    "x2_q16": (16, 32), "y2_q16": (16, 32), "xy_q16": (16, 32),
    "r2_q16": (16, 32), "r4_q16": (16, 32),
    "k1_r2_q16": (16, 32), "k2_r4_q16": (16, 32),
    "gain_partial_q16": (16, 32), "gain_q16": (16, 32),
    "radial_x_q16": (16, 32), "radial_y_q16": (16, 32),
    "p1xy_q16": (16, 32), "p2xy_q16": (16, 32),
    "p2_x_base_q16": (16, 32), "p1_y_base_q16": (16, 32),
    "tangential_x_q16": (16, 32), "tangential_y_q16": (16, 32),
    "xd_q16": (16, 32), "yd_q16": (16, 32),
    "projected_x_q16": (16, 32), "projected_y_q16": (16, 32),
    "src_x_q16": (16, 32), "src_y_q16": (16, 32),
}
PRODUCT_FORMATS: dict[str, int] = {
    "product_normalize_x": 40, "product_normalize_y": 40,
    "product_x2": 32, "product_y2": 32, "product_xy": 32,
    "product_r4": 32, "product_k1_r2": 40, "product_k2_r4": 40,
    "product_radial_x": 32, "product_radial_y": 32,
    "product_p1_xy": 40, "product_p2_xy": 40,
    "product_p2_x_base": 40, "product_p1_y_base": 40,
    "product_projection_x": 32, "product_projection_y": 32,
}
FLOAT_FIELDS = (
    "u", "v", "x", "y", "x2", "y2", "xy", "r2", "r4",
    "k1_r2", "k2_r4", "gain", "radial_x", "radial_y",
    "tangential_x", "tangential_y", "xd", "yd", "src_x", "src_y",
)


def _required_signed_bits(minimum: int, maximum: int) -> int:
    width = 1
    while minimum < -(1 << (width - 1)) or maximum > (1 << (width - 1)) - 1:
        width += 1
    return width


def _update_range(ranges: dict[str, list[int]], name: str, value: int) -> None:
    if name not in ranges:
        ranges[name] = [value, value]
    else:
        ranges[name][0] = min(ranges[name][0], value)
        ranges[name][1] = max(ranges[name][1], value)


def _format_range(minimum: int, maximum: int, frac_bits: int, rtl_width: int) -> dict[str, int | float | bool]:
    required = _required_signed_bits(minimum, maximum)
    scale = 1 << frac_bits
    return {
        "min_raw": minimum,
        "max_raw": maximum,
        "frac_bits": frac_bits,
        "min_value": minimum / scale,
        "max_value": maximum / scale,
        "required_signed_bits": required,
        "rtl_width": rtl_width,
        "fits_rtl_width": required <= rtl_width,
    }


def _camera_terms_report(terms: object) -> tuple[dict[str, dict[str, int | float | bool]], list[int]]:
    term_values = {
        "fx_q16": (terms.fx_q, 16), "fy_q16": (terms.fy_q, 16),
        "cx_q16": (terms.cx_q, 16), "cy_q16": (terms.cy_q, 16),
        "inv_fx_q24": (terms.inv_fx_q, 24), "inv_fy_q24": (terms.inv_fy_q, 24),
        "k1_q24": (terms.k1_q, 24), "k2_q24": (terms.k2_q, 24),
        "p1_q24": (terms.p1_q, 24), "p2_q24": (terms.p2_q, 24),
    }
    report = {
        name: {
            "raw": raw,
            "frac_bits": frac,
            "value": raw / (1 << frac),
            "required_signed_bits": _required_signed_bits(raw, raw),
            "rtl_width": 32,
            "fits_rtl_width": _required_signed_bits(raw, raw) <= 32,
        }
        for name, (raw, frac) in term_values.items()
    }
    values = [value[0] for value in term_values.values()]
    return report, values


def generate(profile_path: Path, vectors_path: Path, params_path: Path,
             report_path: Path, step: int, coordinate_width: int = 26,
             coordinate_frac_bits: int = 10) -> dict[str, object]:
    if step <= 0:
        raise ValueError("step must be a positive integer")
    profile = load_camera_profile(profile_path)
    if profile.width > 0x8000 or profile.height > 0x8000:
        raise ValueError(
            "the current signed Q16 raster-coordinate path supports dimensions up to 32768"
        )

    spec = FixedPointSpec(
        coordinate_width=coordinate_width,
        coordinate_frac_bits=coordinate_frac_bits,
    )
    terms = _prepare_fixed_camera_terms(profile, spec)
    camera_terms, param_values = _camera_terms_report(terms)
    bad_params = [name for name, value in camera_terms.items() if not value["fits_rtl_width"]]
    if bad_params:
        raise ValueError("camera terms exceed signed RTL input width: " + ", ".join(bad_params))

    samples_x = (profile.width + step - 1) // step
    samples_y = (profile.height + step - 1) // step
    sample_count = samples_x * samples_y
    vectors_path.parent.mkdir(parents=True, exist_ok=True)
    params_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    fixed_ranges: dict[str, list[int]] = {}
    float_ranges: dict[str, list[float]] = {}
    errors: list[float] = []
    worst_error = -1.0
    worst_pixel = (0, 0)
    sum_x_squared = 0.0
    sum_y_squared = 0.0
    saturated_count = 0

    with vectors_path.open("w", encoding="ascii", newline="\n") as vectors:
        vectors.write(f"{sample_count}\n")
        for out_y in range(0, profile.height, step):
            for out_x in range(0, profile.width, step):
                trace: dict[str, int] = {}
                fixed = _source_coordinate_fixed(out_x, out_y, terms, trace=trace)
                stages = coordinate_stages(out_x, out_y, profile)
                vectors.write(f"{fixed.src_x} {fixed.src_y}\n")

                float_values = {"u": float(out_x), "v": float(out_y), **stages.__dict__}
                for name, value in float_values.items():
                    if name not in float_ranges:
                        float_ranges[name] = [value, value]
                    else:
                        float_ranges[name][0] = min(float_ranges[name][0], value)
                        float_ranges[name][1] = max(float_ranges[name][1], value)
                for name, raw in trace.items():
                    _update_range(fixed_ranges, name, raw)

                error_x = fixed.x_float - stages.src_x
                error_y = fixed.y_float - stages.src_y
                error = math.hypot(error_x, error_y)
                errors.append(error)
                sum_x_squared += error_x * error_x
                sum_y_squared += error_y * error_y
                if error > worst_error:
                    worst_error = error
                    worst_pixel = (out_x, out_y)
                saturated_count += int(fixed.saturated_x or fixed.saturated_y)

    with params_path.open("w", encoding="ascii", newline="\n") as params:
        params.write(" ".join(str(value) for value in (
            profile.width, profile.height, *param_values
        )) + "\n")

    errors.sort()
    p95_index = max(0, math.ceil(0.95 * len(errors)) - 1)
    stage_formats = {
        **STAGE_FORMATS,
        f"src_x_q{spec.coordinate_frac_bits}_unclipped":
            (spec.coordinate_frac_bits, 64),
        f"src_y_q{spec.coordinate_frac_bits}_unclipped":
            (spec.coordinate_frac_bits, 64),
        f"src_x_q{spec.coordinate_frac_bits}":
            (spec.coordinate_frac_bits, spec.coordinate_width),
        f"src_y_q{spec.coordinate_frac_bits}":
            (spec.coordinate_frac_bits, spec.coordinate_width),
    }
    fixed_report: dict[str, dict[str, int | float | bool]] = {}
    for name, (frac, rtl_width) in stage_formats.items():
        lo, hi = fixed_ranges[name]
        fixed_report[name] = _format_range(lo, hi, frac, rtl_width)
    for name, frac in PRODUCT_FORMATS.items():
        lo, hi = fixed_ranges[name]
        fixed_report[name] = _format_range(lo, hi, frac, 64)

    report: dict[str, object] = {
        "status": "candidate_scan; widths_not_frozen",
        "profile_path": str(profile_path.resolve()),
        "camera_profile": profile.to_mapping(),
        "scan": {
            "step": step,
            "samples_x": samples_x,
            "samples_y": samples_y,
            "count": sample_count,
            "domain": "all pixels" if step == 1 else "uniform grid anchored at (0,0)",
        },
        "fixed_point_spec": {
            "internal_frac_bits": spec.internal_frac_bits,
            "coefficient_frac_bits": spec.coefficient_frac_bits,
            "coordinate_width": spec.coordinate_width,
            "coordinate_frac_bits": spec.coordinate_frac_bits,
            "rounding": "nearest, ties away from zero",
        },
        "quantized_camera_terms": camera_terms,
        "float_intermediate_ranges": {
            name: {"min": limits[0], "max": limits[1]}
            for name, limits in float_ranges.items()
        },
        "fixed_intermediate_ranges": fixed_report,
        "fixed_vs_float_coordinate_error": {
            "rms_x_px": math.sqrt(sum_x_squared / sample_count),
            "rms_y_px": math.sqrt(sum_y_squared / sample_count),
            "p95_euclidean_px": errors[p95_index],
            "max_euclidean_px": worst_error,
            "worst_pixel": list(worst_pixel),
            "saturated_count": saturated_count,
        },
        "rtl_vectors": str(vectors_path.resolve()),
        "rtl_params": str(params_path.resolve()),
        "rtl_comparison": "not run by this generator; use tb_coord_compare.v",
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path, help="camera calibration JSON")
    parser.add_argument("--step", type=int, default=1, help="scan every Nth pixel in x and y")
    parser.add_argument("--coord-width", type=int, default=26,
                        help="signed source-coordinate width; must match RTL COORD_W")
    parser.add_argument("--coord-frac-bits", type=int, default=10,
                        help="source-coordinate fraction bits; must match RTL COORD_F")
    parser.add_argument("--vectors", type=Path, default=Path("rtl/remap/coord_vectors.txt"))
    parser.add_argument("--params", type=Path, default=Path("rtl/remap/camera_params.txt"))
    parser.add_argument("--report", type=Path, default=Path("rtl/remap/fixed_range_report.json"))
    args = parser.parse_args(argv)
    try:
        report = generate(args.profile, args.vectors, args.params, args.report,
                          args.step, args.coord_width, args.coord_frac_bits)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    summary = report["fixed_vs_float_coordinate_error"]
    print(f"Scanned {report['scan']['count']} pixels from {report['profile_path']}")
    print(f"Fixed-vs-float max coordinate error: {summary['max_euclidean_px']:.9g} px")
    print(f"Wrote range report: {args.report.resolve()}")
    print(f"Wrote RTL vectors: {args.vectors.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
