"""Qualify combined coordinate ranges and generate multi-frame RTL vectors."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from sw.reference_common.camera_profile import CameraProfile, load_camera_profile
from sw.reference_common.geometry import InverseAffine, RemapConfig
from sw.reference_fixed.basic_distortion import FixedPointSpec, _prepare_fixed_camera_terms
from sw.reference_fixed.combined_distortion import (
    config_words, pack_config, _combined_coordinate_fixed, _validate_combined_spec,
)
from sw.reference_fixed.rtl_compare import (
    STAGE_FORMATS, PRODUCT_FORMATS, _format_range, _update_range,
)
from sw.reference_float.combined_distortion import combined_coordinate_stages


def scan_config(config: RemapConfig, spec: FixedPointSpec = FixedPointSpec(), *, step: int = 1):
    _validate_combined_spec(spec)
    if not isinstance(step, int) or step <= 0:
        raise ValueError("step must be a positive integer")
    words = config_words(config)
    terms = _prepare_fixed_camera_terms(config.profile, spec)
    ranges, float_ranges, errors = {}, {}, []
    affine_overflows = saturations = 0
    for v in range(0, config.output_height, step):
        for u in range(0, config.output_width, step):
            trace = {}
            fixed = _combined_coordinate_fixed(u, v, config, spec, words[12:18], terms, trace=trace)
            floating = combined_coordinate_stages(u, v, config)
            for name, raw in trace.items():
                _update_range(ranges, name, raw)
            for name, raw in floating.__dict__.items():
                _update_range(float_ranges, name, raw)
            errors.append(math.hypot(fixed.coordinate.x_float-floating.src_x,
                                     fixed.coordinate.y_float-floating.src_y))
            affine_overflows += int(fixed.affine_overflow)
            saturations += int(fixed.coordinate.saturated_x or fixed.coordinate.saturated_y)
    formats = {**STAGE_FORMATS, **{k: (f, 64) for k, f in PRODUCT_FORMATS.items()},
               "affine_x_q16_wide": (16, 64), "affine_y_q16_wide": (16, 64),
               "ideal_x_q16": (16, 32), "ideal_y_q16": (16, 32),
               **{f"src_{axis}_q{spec.coordinate_frac_bits}{suffix}": (spec.coordinate_frac_bits, w)
                  for axis in ('x', 'y') for suffix, w in (('_unclipped', 64), ('', spec.coordinate_width))}}
    range_report = {name: _format_range(*ranges[name], frac, width)
                    for name, (frac, width) in formats.items()}
    errors.sort()
    qualified = (step == 1 and affine_overflows == 0 and saturations == 0
                 and all(item['fits_rtl_width'] for item in range_report.values()))
    return {
        "camera_profile": config.profile.to_mapping(), "inverse_affine": config.affine.__dict__,
        "output_width": config.output_width, "output_height": config.output_height,
        "step": step, "count": len(errors), "full_frame_widths_fit": qualified,
        "fixed_ranges": range_report, "float_ranges": float_ranges,
        "affine_overflow_count": affine_overflows, "coordinate_saturation_count": saturations,
        "error_rms_px": math.sqrt(sum(e*e for e in errors)/len(errors)),
        "error_p95_px": errors[max(0, math.ceil(.95*len(errors))-1)],
        "error_max_px": errors[-1],
        "qualification": "candidate widths fit this full scan; real-profile accuracy/timing remain separate" if qualified else "not qualified; inspect ranges/full-frame scan",
    }


def generate(configs: list[RemapConfig], output: Path, spec: FixedPointSpec = FixedPointSpec(), *, synthetic=False):
    if not 3 <= len(configs) <= 32:
        raise ValueError("dynamic bench requires 3..32 configurations")
    reports = [scan_config(config, spec) for config in configs]
    output.mkdir(parents=True, exist_ok=True)
    report = {"synthetic": synthetic, "coordinate_width": spec.coordinate_width,
              "coordinate_frac_bits": spec.coordinate_frac_bits, "configs": reports}
    (output/'range_report.json').write_text(json.dumps(report, indent=2)+"\n", encoding='utf-8')
    if not all(r['full_frame_widths_fit'] for r in reports):
        raise ValueError("candidate widths do not fit; see range_report.json")
    (output/'configs.hex').write_text("\n".join(f"{pack_config(c):0160x}" for c in configs)+"\n", encoding='ascii')
    # Repeat the last config for a frame without a pending update: same version.
    with (output/'vectors.txt').open('w', encoding='ascii', newline='\n') as vectors:
        vectors.write(f"{len(configs)+1}\n")
        tag = 0
        for frame in range(len(configs)+1):
            index = min(frame, len(configs)-1)
            config = configs[index]
            terms = _prepare_fixed_camera_terms(config.profile, spec)
            affine = config_words(config)[12:18]
            vectors.write(f"{100+frame} {index+1} {index} {config.output_width*config.output_height}\n")
            for v in range(config.output_height):
                for u in range(config.output_width):
                    result = _combined_coordinate_fixed(u, v, config, spec, affine, terms)
                    coord = result.coordinate
                    vectors.write(f"{u} {v} {coord.src_x} {coord.src_y} {result.border} {tag}\n")
                    tag += 1
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--profile', type=Path)
    choice.add_argument('--synthetic-regression', action='store_true')
    parser.add_argument('--angles', nargs='+', type=float, default=[0, 15, -30, 45, 90, 180, 360, 13.37])
    parser.add_argument('--output', type=Path, default=Path('reports/dynamic_distortion/q10'))
    parser.add_argument('--coord-width', type=int, default=26)
    parser.add_argument('--coord-frac-bits', type=int, default=10)
    args = parser.parse_args()
    configs = []
    for i, angle in enumerate(args.angles):
        profile = (load_camera_profile(args.profile) if args.profile else
                   CameraProfile(48 if i == 6 else 64, 32 if i == 6 else 48,
                                 64+i*2, 64+i, 30.1+i*.1, 24.2-i*.05,
                                 0 if i == 0 else -.18+i*.005,
                                 0 if i == 0 else .025, 0 if i == 0 else .003, 0 if i == 0 else -.004))
        configs.append(RemapConfig(profile, InverseAffine.rotation(angle, profile.width, profile.height),
                                   32 if args.synthetic_regression and i == 5 else None,
                                   24 if args.synthetic_regression and i == 5 else None))
    report = generate(configs, args.output, FixedPointSpec(coordinate_width=args.coord_width,
                      coordinate_frac_bits=args.coord_frac_bits), synthetic=args.synthetic_regression)
    print(json.dumps({"output": str(args.output.resolve()), "configs": len(configs),
                      "max_error_px": max(c['error_max_px'] for c in report['configs']),
                      "all_full_frame_widths_fit": True}))


if __name__ == '__main__':
    main()
