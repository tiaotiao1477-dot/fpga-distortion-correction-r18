"""Exact 1080p configuration preparation for the isolated rotation dashboard V3."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Callable

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DELIVERY = ROOT / 'wanchengceshi/algorithm_v04_integration/delivery/distortion_algorithm_delivery_v0_4'
sys.path.insert(0, str(DELIVERY))

from fast_audit import fixed_tile, prove_int64_bounds

from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_common.geometry import InverseAffine, RemapConfig
from sw.reference_common.edge_processing import EdgeConfig, BorderMode, edge_config_words, pack_edge_config
from sw.reference_fixed.basic_distortion import FixedPointSpec, _prepare_fixed_camera_terms
from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES, config_words
from sw.reference_fixed.rtl_compare import STAGE_FORMATS, PRODUCT_FORMATS, _format_range

WIDTH, HEIGHT = 1920, 1080
BIT_SHA = 'e8b8d37b8e046069f724119e22006441cd28c8e81a134f1fa99a2c51ba883ffb'
PROFILE_SHA = 'bb929129b3f6713a7b75225b554be7dea7174b704ec32dd1cde869dc69164a14'
HELPER_SHA = '64a6941b0ffaee1068a92fef0b133f9712bfab57db4fadfe7e999c9d5f46fa45'
TILE_ROWS = 32


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def reference_hashes() -> dict[str, str]:
    files = sorted((DELIVERY / 'sw').rglob('*.py'))
    required = [DELIVERY / 'sw' / name for name in
                ('reference_fixed/basic_distortion.py', 'reference_fixed/combined_distortion.py',
                 'reference_fixed/dynamic_rtl_compare.py', 'reference_fixed/rtl_compare.py',
                 'reference_float/combined_distortion.py', 'reference_common/geometry.py')]
    if not files or any(p not in files for p in required):
        raise FileNotFoundError('The complete frozen v0.4 reference package is unavailable')
    return {str(p.resolve()): sha(p) for p in files}


def affine_for(angle: float, scale: float) -> InverseAffine:
    if not math.isfinite(angle) or not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('Finite angle and inverse scale in (0,1] required')
    base = InverseAffine.rotation(angle, WIDTH, HEIGHT)
    cx, cy = (WIDTH - 1) / 2, (HEIGHT - 1) / 2
    a, b, d, e = (scale * base.a00, scale * base.a01,
                  scale * base.a10, scale * base.a11)
    return InverseAffine(a, b, cx - a * cx - b * cy,
                         d, e, cy - d * cx - e * cy)


def float_margin(profile, angle: float, scale: float) -> float:
    """All-output-pixel floating estimate; never qualifies a payload."""
    affine = affine_for(angle, scale)
    u = np.arange(WIDTH, dtype=np.float64)[None, :]
    margin = float('inf')
    for start in range(0, HEIGHT, 48):
        v = np.arange(start, min(HEIGHT, start + 48), dtype=np.float64)[:, None]
        x = (affine.a00 * u + affine.a01 * v + affine.a02 - profile.cx) / profile.fx
        y = (affine.a10 * u + affine.a11 * v + affine.a12 - profile.cy) / profile.fy
        r2 = x * x + y * y
        gain = 1 + profile.k1 * r2 + profile.k2 * r2 * r2
        sx = profile.fx * (x * gain + 2 * profile.p1 * x * y + profile.p2 * (r2 + 2 * x * x)) + profile.cx
        sy = profile.fy * (y * gain + profile.p1 * (r2 + 2 * y * y) + 2 * profile.p2 * x * y) + profile.cy
        if not np.isfinite(sx).all() or not np.isfinite(sy).all():
            return -float('inf')
        margin = min(margin, float(sx.min()), float(sy.min()),
                     WIDTH - 1 - float(sx.max()), HEIGHT - 1 - float(sy.max()))
    return margin


def choose_scale(profile, angle: float) -> tuple[float, dict]:
    margin_target = 2.0
    if float_margin(profile, angle, 1.0) >= margin_target:
        scale = 1.0
    else:
        lo, hi = 0.125, 1.0
        if float_margin(profile, angle, lo) < margin_target:
            raise ValueError('No conservative centered crop found within 8x zoom')
        for _ in range(16):
            mid = (lo + hi) / 2
            if float_margin(profile, angle, mid) >= margin_target:
                lo = mid
            else:
                hi = mid
        scale = lo * 0.999
    return scale, {'method': 'centered isotropic search, full floating raster; final integer audit mandatory',
                   'floatMinimumMarginPx': float_margin(profile, angle, scale),
                   'floatTargetMarginPx': margin_target, 'inverseScale': scale,
                   'zoom': 1 / scale, 'idealCanvasLinearFraction': scale,
                   'idealCanvasAreaFraction': scale * scale,
                   'globallyMaximumFieldOfViewProven': False}


def _update_range(ranges: dict, name: str, values: np.ndarray) -> None:
    lo, hi = int(np.min(values)), int(np.max(values))
    if name in ranges:
        ranges[name][0] = min(ranges[name][0], lo)
        ranges[name][1] = max(ranges[name][1], hi)
    else:
        ranges[name] = [lo, hi]


def _update_float_range(ranges: dict, name: str, values: np.ndarray) -> None:
    lo, hi = float(np.min(values)), float(np.max(values))
    if name in ranges:
        ranges[name][0] = min(ranges[name][0], lo)
        ranges[name][1] = max(ranges[name][1], hi)
    else:
        ranges[name] = [lo, hi]


def scan_crop(config: RemapConfig, crop: bool, *, tile_rows: int = TILE_ROWS,
              progress: Callable[[int, int], None] | None = None,
              proof_scope: str = 'measured_profile') -> tuple[dict, dict]:
    """Audit all 2,073,600 pixels with the original integer model and widths.

    The default measured-profile scope preserves the original manual/V3 gates;
    CNN output must opt into the bounded predicted-Brown proof scope.
    """
    if not isinstance(tile_rows, int) or tile_rows <= 0:
        raise ValueError('tile_rows must be positive')
    if (config.output_width, config.output_height) != (WIDTH, HEIGHT):
        raise ValueError('Full-frame v0.4 audit requires exact native 1920x1080 dimensions')
    spec = FixedPointSpec()
    words = config_words(config)
    terms = _prepare_fixed_camera_terms(config.profile, spec)
    bounds = prove_int64_bounds(config, spec, words, terms, proof_scope=proof_scope)
    ranges: dict[str, list[int]] = {}
    float_ranges: dict[str, list[float]] = {}
    errors = np.empty(WIDTH * HEIGHT, dtype=np.float64)
    error_at = 0
    affine_overflows = saturations = outside = invalid_taps = 0
    min_x = min_y = 1 << 63
    max_x = max_y = -(1 << 63)
    u = np.arange(WIDTH, dtype=np.int64)[None, :]
    for start in range(0, HEIGHT, tile_rows):
        end = min(HEIGHT, start + tile_rows)
        v = np.arange(start, end, dtype=np.int64)[:, None]
        result = fixed_tile(config, u, v, spec, words, terms, proof_scope=proof_scope)
        trace = result['trace']
        for name, raw in trace.items():
            _update_range(ranges, name, raw)
        for name, raw in result['float_trace'].items():
            _update_float_range(float_ranges, name, raw)
        tile_count = (end - start) * WIDTH
        errors[error_at:error_at + tile_count] = result['errors'].reshape(-1)
        error_at += tile_count
        affine_overflows += int(np.count_nonzero(result['affine_overflow']))
        saturations += int(np.count_nonzero(result['saturated_x'] | result['saturated_y']))
        outside += int(np.count_nonzero(result['border'] == 2))
        invalid_taps += int(np.count_nonzero(result['invalid_taps']))
        sx, sy = trace['src_x_q16'], trace['src_y_q16']
        min_x, min_y = min(min_x, int(sx.min())), min(min_y, int(sy.min()))
        max_x, max_y = max(max_x, int(sx.max())), max(max_y, int(sy.max()))
        if progress is not None:
            progress(end, HEIGHT)

    formats = {**STAGE_FORMATS, **{key: (frac, 64) for key, frac in PRODUCT_FORMATS.items()},
               'affine_x_q16_wide': (16, 64), 'affine_y_q16_wide': (16, 64),
               'ideal_x_q16': (16, 32), 'ideal_y_q16': (16, 32),
               **{f'src_{axis}_q{spec.coordinate_frac_bits}{suffix}': (spec.coordinate_frac_bits, width)
                  for axis in ('x', 'y') for suffix, width in (('_unclipped', 64), ('', spec.coordinate_width))}}
    range_report = {name: _format_range(*ranges[name], frac, width)
                    for name, (frac, width) in formats.items()}
    if error_at != WIDTH * HEIGHT:
        raise AssertionError('full raster was not visited')
    p95_index = math.ceil(.95 * error_at) - 1
    error_p95 = float(np.partition(errors, p95_index)[p95_index])
    error_rms = float(np.sqrt(np.mean(errors * errors, dtype=np.float64)))
    error_max = float(np.max(errors))
    widths_fit = all(row['fits_rtl_width'] for row in range_report.values())
    qualified = affine_overflows == 0 and saturations == 0 and widths_fit
    report = {
        'camera_profile': config.profile.to_mapping(), 'inverse_affine': config.affine.__dict__,
        'output_width': WIDTH, 'output_height': HEIGHT, 'step': 1, 'count': error_at,
        'full_frame_widths_fit': qualified, 'fixed_ranges': range_report,
        'float_ranges': float_ranges, 'affine_overflow_count': affine_overflows,
        'coordinate_saturation_count': saturations, 'int64Preflight': {'passed': True, 'bounds': bounds},
        'error_rms_px': error_rms, 'error_p95_px': error_p95, 'error_max_px': error_max,
        'int64PreflightScope': proof_scope,
        'errorMetrics': 'NumPy float64; every pixel retained; min/max/range/tap gates are integer exact',
        'qualification': 'Original integer model, all stage/product widths checked; hardware image quality separate',
    }
    crop_report = {
        'passed': qualified and (not crop or (outside == 0 and invalid_taps == 0)),
        'cropEnabled': bool(crop), 'count': error_at, 'outsideCount': outside,
        'invalidFetch4Count': invalid_taps, 'sourceXQ16Range': [min_x, max_x],
        'sourceYQ16Range': [min_y, max_y],
        'allPixelsAndFourTapsInside': outside == 0 and invalid_taps == 0,
        'hardwareImageAndThroughputVerified': False,
    }
    return report, crop_report


def prepare_config(profile_path: Path, helper: Path, angle: float, output: Path, crop: bool,
                   progress: Callable[[int, int], None] | None = None) -> Path:
    """Create a content-bound V3 92-byte config after a complete raster audit."""
    profile_path, helper, output = Path(profile_path).resolve(), Path(helper).resolve(), Path(output).resolve()
    if not __debug__ or not math.isfinite(float(angle)):
        raise ValueError('Assertions enabled and finite angle required')
    if not str(output).isascii():
        raise ValueError('ASCII output path required by the frozen send CLI')
    if sha(profile_path) != PROFILE_SHA or sha(helper) != HELPER_SHA:
        raise ValueError('Frozen measured profile/send-helper binding changed')
    ref_hashes = reference_hashes()
    source_files = [Path(__file__).resolve(), HERE / 'fast_audit.py', HERE / 'fast_control.py',
                   HERE / 'content_hash_batch.py', HERE / 'fast_sender.py']
    source_hashes = {str(path.resolve()): sha(path) for path in source_files}
    profile = load_camera_profile(profile_path)
    if (profile.width, profile.height) != (WIDTH, HEIGHT):
        raise ValueError('Measured native 1080p profile required')
    key_angle = float(angle) % 360.0
    scale, choice = choose_scale(profile, key_angle) if crop else (1.0, {
        'method': 'full-view identity scale; no crop search', 'floatTargetMarginPx': None,
        'inverseScale': 1.0, 'zoom': 1.0, 'idealCanvasLinearFraction': 1.0,
        'idealCanvasAreaFraction': 1.0, 'globallyMaximumFieldOfViewProven': False})
    config = EdgeConfig(RemapConfig(profile, affine_for(key_angle, scale)), BorderMode.CLAMP, 200, 0)
    edge_config_words(config)
    output.mkdir(parents=True, exist_ok=False)
    range_report, crop_report = scan_crop(config.remap, crop, progress=progress)
    geometry_sha = sha(Path(__file__))
    audit_sha = sha(HERE / 'fast_audit.py')
    crop_report.update(angle=key_angle, bitSha256=BIT_SHA, profileSha256=PROFILE_SHA,
                       sendHelperSha256=HELPER_SHA, generatorSha256=geometry_sha,
                       auditModuleSha256=audit_sha, provenance='rotation_dashboard_v3.fast_geometry', **choice)
    write_json(output / 'range_report.json', range_report)
    write_json(output / 'crop_report.json', crop_report)
    if not range_report['full_frame_widths_fit'] or not crop_report['passed']:
        raise ValueError('Full raster width/tap audit failed; reports preserved and no payload authorized')
    payload = pack_edge_config(config)
    (output / 'config_payload.bin').write_bytes(payload.to_bytes(92, 'little'))
    (output / 'config_payload.hex').write_text(f'{payload:0184x}\n', encoding='ascii')
    words = edge_config_words(config)
    config_doc = {
        'payload_version': '0.4', 'payload_bytes': 92, 'byte_order': 'little',
        'coordinate_width': 26, 'coordinate_frac_bits': 10, 'rtl_max_width': WIDTH,
        'profile': profile.to_mapping(), 'inverse_affine': config.remap.affine.__dict__,
        'output_width': WIDTH, 'output_height': HEIGHT, 'crop_enabled': bool(crop),
        'words': dict(zip((*CONFIG_WORD_NAMES, 'border_mode', 'edge_threshold', 'edge_mode'), words)),
        'generator': 'rotation_dashboard_v3.fast_geometry',
        'qualification': 'Full native raster integer width audit; crop tap audit required only when crop_enabled',
    }
    write_json(output / 'config.json', config_doc)
    if ref_hashes != reference_hashes() or source_hashes != {str(path.resolve()): sha(path) for path in source_files}:
        raise ValueError('V3 source/reference package changed during audit; outputs preserved, payload binding withheld')
    bound_files = [profile_path, helper, *source_files,
                   output / 'config_payload.bin', output / 'config.json',
                   output / 'range_report.json', output / 'crop_report.json']
    binding = {**ref_hashes, **{str(path.resolve()): sha(path) for path in bound_files}}
    write_json(output / 'binding.json', binding)
    write_json(output / 'qualified_binding.json', {
        'bitSha256': BIT_SHA, 'profileSha256': PROFILE_SHA, 'sendHelperSha256': HELPER_SHA,
        'angle': key_angle, 'cropEnabled': bool(crop), 'inverseScale': scale,
        'bindingSha256': sha(output / 'binding.json'),
        'dependencyManifestSha256': hashlib.sha256(json.dumps(ref_hashes, sort_keys=True).encode()).hexdigest(),
    })
    return output
