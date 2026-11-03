"""Synthetic integer-reference previews, not board/camera validation."""
import argparse
import json
import struct
import zlib
from pathlib import Path

from sw.host.generate_edge_config import qualify
from sw.reference_common.camera_profile import CameraProfile
from sw.reference_common.geometry import InverseAffine, RemapConfig
from sw.reference_common.sampling import BorderMode
from sw.reference_common.edge_processing import EdgeConfig, sobel_rgb, warp_rgb_edges


def write_png(path, image):
    height, width = len(image), len(image[0])
    def chunk(kind, payload):
        return struct.pack('>I', len(payload))+kind+payload+struct.pack('>I', zlib.crc32(kind+payload)&0xffffffff)
    raw = b''.join(b'\x00'+bytes(channel for pixel in row for channel in pixel) for row in image)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))+
                     chunk(b'IDAT', zlib.compress(raw))+chunk(b'IEND', b''))


def generate(output, angle=23, threshold=200):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    width, height = 128, 96
    image = []
    for y in range(height):
        row = []
        for x in range(width):
            pixel = (32+x//2, 48+y//2, 85)
            if x%16<2 or y%16<2:
                pixel = (215, 220, 225)
            if 18<=x<49 and 20<=y<63:
                pixel = (20, 185, 75)
            if (x-88)**2+(y-49)**2<=22**2:
                pixel = (40, 100, 235)
            row.append(pixel)
        image.append(row)
    profile = CameraProfile(width, height, 128, 128, 63.5, 47.5, -.18, .025, .003, -.004)
    (output/'synthetic_profile.json').write_text(json.dumps(profile.to_mapping(), indent=2)+'\n', encoding='utf-8')
    remap = RemapConfig(profile, InverseAffine.rotation(angle, width, height))
    black_config = EdgeConfig(remap, BorderMode.BLACK, threshold, 3)
    clamp_config = EdgeConfig(remap, BorderMode.CLAMP, threshold, 3)
    report = qualify(black_config, max_width=width)
    if not report['full_frame_widths_fit']:
        raise ValueError('demo candidate width check failed')
    black_overlay, black, black_mask, _ = warp_rgb_edges(image, black_config)
    clamp_overlay, clamp, clamp_mask, _ = warp_rgb_edges(image, clamp_config)
    black_binary = sobel_rgb(black, black_mask, threshold=threshold, mode=2)
    panels = [image, black, clamp,
              [[p.rgb for p in row] for row in black_binary],
              [[p.rgb for p in row] for row in black_overlay],
              [[p.rgb for p in row] for row in clamp_overlay]]
    names = ['source', 'black_corrected', 'clamp_corrected', 'black_binary', 'black_overlay', 'clamp_overlay']
    for name, panel in zip(names, panels):
        write_png(output/f'{name}.png', panel)
    gap = 6
    montage = [[(240, 240, 240)]*(width*3+gap*4) for _ in range(height*2+gap*3)]
    for index, panel in enumerate(panels):
        ox, oy = gap+(index%3)*(width+gap), gap+(index//3)*(height+gap)
        for y, row in enumerate(panel):
            montage[oy+y][ox:ox+width]=row
    write_png(output/'preview.png', montage)
    data = {'synthetic': True, 'software_reference_only': True, 'angle_deg': angle, 'threshold': threshold,
            'coordinate_format': 'S(26,10)', 'width': width, 'height': height,
            'panel_order_left_to_right_top_to_bottom': names, 'profile': profile.to_mapping(),
            'black_valid_pixels': sum(sum(row) for row in black_mask),
            'clamp_valid_pixels': sum(sum(row) for row in clamp_mask),
            'black_detected_edges': sum(p.edge for row in black_overlay for p in row),
            'clamp_detected_edges': sum(p.edge for row in clamp_overlay for p in row),
            'full_frame_widths_fit': report['full_frame_widths_fit']}
    (output/'preview.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    (output/'range_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('reports/edge_preview'))
    parser.add_argument('--angle', type=float, default=23)
    parser.add_argument('--threshold', type=int, default=200)
    args = parser.parse_args()
    generate(args.output, args.angle, args.threshold)


if __name__ == '__main__':
    main()
