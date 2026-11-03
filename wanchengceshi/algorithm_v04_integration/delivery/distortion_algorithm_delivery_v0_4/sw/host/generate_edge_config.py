"""Export a range-qualified v0.4 92-byte payload; UART framing/CRC are external."""
import argparse
import json
from pathlib import Path

from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_common.geometry import InverseAffine, RemapConfig
from sw.reference_common.sampling import BorderMode
from sw.reference_common.edge_processing import EdgeConfig, edge_config_words, pack_edge_config
from sw.reference_fixed.basic_distortion import FixedPointSpec
from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES
from sw.reference_fixed.dynamic_rtl_compare import scan_config

MODE_NAMES = {'original': 0, 'magnitude': 1, 'binary': 2, 'overlay': 3}
WORD_NAMES = (*CONFIG_WORD_NAMES, 'border_mode', 'edge_threshold', 'edge_mode')


def qualify(config, spec=FixedPointSpec(), *, max_width=1920):
    if isinstance(max_width, bool) or not isinstance(max_width, int) or not 1 <= max_width <= 32768:
        raise ValueError('max_width must be in 1..32768 and match RTL MAX_WIDTH')
    if config.remap.output_width > max_width:
        raise ValueError('output width exceeds the Sobel line buffer MAX_WIDTH')
    # config_words performs the same structural checks as the hardware.
    edge_config_words(config)
    return scan_config(config.remap, spec, step=1)


def export(config, output, spec=FixedPointSpec(), *, max_width=1920):
    report = qualify(config, spec, max_width=max_width)
    output = Path(output)
    if any((output/name).exists() for name in ('config_payload.bin', 'config_payload.hex', 'config.json')):
        raise FileExistsError('choose a new output directory to preserve the existing payload')
    output.mkdir(parents=True, exist_ok=True)
    (output/'range_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if not report['full_frame_widths_fit']:
        raise ValueError('configuration exceeds candidate RTL widths; inspect range_report.json')
    payload = pack_edge_config(config)
    (output/'config_payload.bin').write_bytes(payload.to_bytes(92, 'little'))
    (output/'config_payload.hex').write_text(f'{payload:0184x}\n', encoding='ascii')
    (output/'config.json').write_text(json.dumps({
        'payload_version': '0.4', 'payload_bytes': 92, 'byte_order': 'little',
        'coordinate_width': spec.coordinate_width, 'coordinate_frac_bits': spec.coordinate_frac_bits,
        'rtl_max_width': max_width, 'profile': config.remap.profile.to_mapping(),
        'inverse_affine': config.remap.affine.__dict__,
        'output_width': config.remap.output_width, 'output_height': config.remap.output_height,
        'words': dict(zip(WORD_NAMES, edge_config_words(config))),
        'qualification': report['qualification'],
    }, indent=2)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--angle', type=float, required=True)
    parser.add_argument('--border', choices=('black', 'clamp'), default='black')
    parser.add_argument('--threshold', type=int, default=128)
    parser.add_argument('--mode', choices=tuple(MODE_NAMES), default='original')
    parser.add_argument('--output-width', type=int)
    parser.add_argument('--output-height', type=int)
    parser.add_argument('--max-width', type=int, default=1920)
    parser.add_argument('--coord-width', type=int, default=26)
    parser.add_argument('--coord-frac-bits', type=int, default=10)
    parser.add_argument('--output', type=Path, default=Path('reports/edge_host_config'))
    args = parser.parse_args()
    profile = load_camera_profile(args.profile)
    config = EdgeConfig(RemapConfig(profile, InverseAffine.rotation(args.angle, profile.width, profile.height),
                                  args.output_width, args.output_height),
                        BorderMode.BLACK if args.border == 'black' else BorderMode.CLAMP,
                        args.threshold, MODE_NAMES[args.mode])
    report = export(config, args.output,
                    FixedPointSpec(coordinate_width=args.coord_width, coordinate_frac_bits=args.coord_frac_bits),
                    max_width=args.max_width)
    print(f'Checked 92-byte payload: {args.output.resolve()}; max coordinate error={report["error_max_px"]:.6f} px')


if __name__ == '__main__':
    main()
