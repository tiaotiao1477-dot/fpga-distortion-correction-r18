"""Build a complete checked config payload from calibration and angle.

The binary output is the 80-byte little-endian PAYLOAD only; UART framing,
sequence and CRC belong to the board adapter's protocol.
"""

import argparse
import json
from pathlib import Path

from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_common.geometry import InverseAffine, RemapConfig
from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES, config_words, pack_config
from sw.reference_fixed.dynamic_rtl_compare import scan_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--angle', type=float, required=True)
    parser.add_argument('--output', type=Path, default=Path('reports/host_config'))
    args = parser.parse_args()
    profile = load_camera_profile(args.profile)
    config = RemapConfig(profile, InverseAffine.rotation(args.angle, profile.width, profile.height))
    report = scan_config(config)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'range_report.json').write_text(json.dumps(report, indent=2)+"\n", encoding='utf-8')
    if not report['full_frame_widths_fit']:
        raise ValueError("configuration exceeds candidate RTL widths; inspect range_report.json")
    words = config_words(config)
    (args.output/'config_payload.bin').write_bytes(pack_config(config).to_bytes(80, 'little'))
    (args.output/'config_payload.hex').write_text(f"{pack_config(config):0160x}\n", encoding='ascii')
    (args.output/'config.json').write_text(json.dumps({
        'angle_deg': args.angle, 'rotation_center': [(profile.width-1)/2, (profile.height-1)/2],
        'inverse_affine': config.affine.__dict__, 'words': dict(zip(CONFIG_WORD_NAMES, words)),
        'coordinate_format': 'S(26,10)', 'profile': profile.to_mapping(),
    }, indent=2)+"\n", encoding='utf-8')
    print(f"Checked 80-byte payload written to {args.output.resolve()}; max error={report['error_max_px']:.6f} px")


if __name__ == '__main__':
    main()
