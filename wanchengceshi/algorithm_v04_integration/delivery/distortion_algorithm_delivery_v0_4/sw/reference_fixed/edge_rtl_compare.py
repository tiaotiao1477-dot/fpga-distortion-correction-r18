"""Deterministic border, full-frame Sobel and integrated pixel-chain vectors."""
import argparse
import json
import random
from pathlib import Path

from sw.reference_common.camera_profile import CameraProfile
from sw.reference_common.geometry import RemapConfig, InverseAffine
from sw.reference_common.sampling import BorderMode
from sw.reference_common.edge_processing import EdgeConfig, sampling_descriptor, sobel_rgb, warp_rgb_edges, pack_edge_config
from sw.reference_fixed.basic_distortion import FixedPointSpec


def rgb_hex(pixel):
    return f'{pixel[0]:02x}{pixel[1]:02x}{pixel[2]:02x}'


def image_pattern(width, height, pattern, seed=0):
    rng = random.Random(seed)
    rows = []
    for y in range(height):
        row = []
        for x in range(width):
            if pattern == 'flat':
                pixel = (255 if seed % 2 else 0,)*3
            elif pattern == 'vertical':
                pixel = (255 if x >= width//2 else 0,)*3
            elif pattern == 'horizontal':
                pixel = (255 if y >= height//2 else 0,)*3
            elif pattern == 'checker':
                pixel = (255 if (x+y) % 2 else 0,)*3
            elif pattern == 'rgb':
                pixel = ((31*x+7*y+seed) % 256, (43*y+11*x+seed) % 256, (23*x+29*y) % 256)
            else:
                pixel = tuple(rng.randrange(256) for _ in range(3))
            row.append(pixel)
        rows.append(row)
    return rows


def generate(output, spec):
    output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(404)
    border_count = 5000
    with (output/'border_vectors.txt').open('w', encoding='ascii') as f:
        f.write(f'{border_count}\n')
        for i in range(border_count):
            width, height = rng.choice(((1, 1), (2, 1), (4, 3), (64, 48), (32768, 32768), (0, 4), (40000, 3)))
            scale = 1 << spec.coordinate_frac_bits
            limit = 1 << (spec.coordinate_width-1)
            sx = rng.choice((-1, 0, scale//2, (width-1)*scale, rng.randrange(-limit, limit)))
            sy = rng.choice((-1, 0, scale//2, (height-1)*scale, rng.randrange(-limit, limit)))
            sx = min(max(sx, -limit), limit-1); sy = min(max(sy, -limit), limit-1)
            border, invalid, clamp = rng.randrange(3), int(i % 11 == 0), i % 2
            desc = sampling_descriptor(sx, sy, spec.coordinate_frac_bits, width, height,
                                       border=border, invalid=invalid, clamp=clamp)
            values = (sx, sy, width, height, border, invalid, clamp, desc.x0, desc.x1,
                      desc.y0, desc.y1, desc.dx, desc.dy, int(desc.sample_enable), int(desc.outside), int(desc.boundary))
            f.write(' '.join(map(str, values))+'\n')

    cases = [(w, h, pattern, mode) for w, h in ((1, 1), (1, 7), (7, 1), (2, 2), (8, 6), (32, 24), (17, 9), (5, 3))
             for pattern in ('flat', 'vertical', 'horizontal', 'checker', 'rgb', 'mask') for mode in range(4)]
    sobel_count = 0
    with (output/'sobel_vectors.txt').open('w', encoding='ascii') as f:
        f.write(f'{len(cases)}\n')
        for index, (w, h, pattern, mode) in enumerate(cases):
            image = image_pattern(w, h, pattern, index)
            mask = [[not (pattern == 'mask' and (x == 0 or (x+3*y) % 13 == 0)) for x in range(w)] for y in range(h)]
            threshold = (0, 1, 128, 1020, 2047)[index % 5]
            result = sobel_rgb(image, mask, threshold=threshold, mode=mode)
            f.write(f'{w} {h} {1000+index} {index+1} {threshold} {mode}\n')
            for y in range(h):
                for x in range(w):
                    p = result[y][x]
                    tag = 100000+index*10000+(y*w+x)*7
                    f.write(f'{rgb_hex(image[y][x])} {int(mask[y][x])} {tag} {rgb_hex(p.rgb)} {p.gx} {p.gy} {p.magnitude} {int(p.edge)} {int(p.window_valid)} {int(p.sample_valid)}\n')
                    sobel_count += 1

    configs = []
    for i, angle in enumerate((0, 15, 45, -30, 90, 180, 360, 13.37)):
        profile = CameraProfile(8, 6, 32+i, 32+i, 3.5, 2.5,
                                0 if i == 0 else -.18, 0 if i == 0 else .025,
                                0 if i == 0 else .003, 0 if i == 0 else -.004)
        configs.append(EdgeConfig(RemapConfig(profile, InverseAffine.rotation(angle, 8, 6)),
                                  BorderMode.CLAMP if i % 2 else BorderMode.BLACK, (0, 64, 400, 128)[i % 4], i % 4))
    # Continuous negative Q16 source coordinate rounded to zero in Q10/Q8.
    configs.append(EdgeConfig(RemapConfig(CameraProfile(8, 6, 1, 1, 0, 0), InverseAffine(a02=-1/65536)), mode=3))
    # Structurally legal but range-unqualified config intentionally injected
    # only to verify the hardware invalid flag blocks Fetch even in CLAMP.
    configs.append(EdgeConfig(RemapConfig(CameraProfile(2, 2, 16384, 16384, 0, 0),
                                         InverseAffine(a00=2, a02=32767.5)), BorderMode.CLAMP, mode=3))
    for width, height in ((1, 1), (1, 5), (5, 1), (2, 2)):
        configs.append(EdgeConfig(RemapConfig(CameraProfile(width, height, 16, 16, 0, 0)), BorderMode.CLAMP, 0, len(configs) % 4))
    configs.append(EdgeConfig(RemapConfig(CameraProfile(6, 5, 32, 32, 2.5, 2),
                                         InverseAffine.rotation(90, 6, 5), 4, 3), BorderMode.BLACK, 100, 3))
    (output/'pipeline_configs.hex').write_text('\n'.join(f'{pack_edge_config(c):0184x}' for c in configs)+'\n', encoding='ascii')
    total_pixels = total_fetches = 0
    with (output/'pipeline_vectors.txt').open('w', encoding='ascii') as f:
        f.write(f'{len(configs)}\n')
        for i, config in enumerate(configs):
            profile, remap = config.remap.profile, config.remap
            image = image_pattern(profile.width, profile.height, ('rgb', 'vertical', 'horizontal', 'checker')[i % 4], 7+i)
            result, warped, mask, desc = warp_rgb_edges(image, config, spec)
            fetch_count = sum(int(d.sample_enable) for row in desc for d in row)
            f.write(f'{200+i} {i+1} {profile.width} {profile.height} {remap.output_width} {remap.output_height} {fetch_count}\n')
            for row in image:
                for pixel in row:
                    f.write(rgb_hex(pixel)+'\n')
            for y in range(remap.output_height):
                for x in range(remap.output_width):
                    p = result[y][x]
                    tag = 300000+i*10000+(y*remap.output_width+x)*7
                    f.write(f'{rgb_hex(p.rgb)} {p.gx} {p.gy} {p.magnitude} {int(p.edge)} {int(p.window_valid)} {int(p.sample_valid)} {tag}\n')
                    total_pixels += 1
            total_fetches += fetch_count
    summary = {'synthetic': True, 'coordinate_width': spec.coordinate_width, 'coordinate_frac_bits': spec.coordinate_frac_bits,
               'border_vectors': border_count, 'sobel_frames': len(cases), 'sobel_pixels': sobel_count,
               'pipeline_frames': len(configs), 'pipeline_pixels': total_pixels, 'pipeline_fetches': total_fetches,
               'range_unqualified_fault_injection_config_index': 9}
    (output/'vector_summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(summary))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('reports/edge_v04/q10'))
    parser.add_argument('--coord-width', type=int, default=26)
    parser.add_argument('--coord-frac-bits', type=int, default=10)
    args = parser.parse_args()
    generate(args.output, FixedPointSpec(coordinate_width=args.coord_width, coordinate_frac_bits=args.coord_frac_bits))


if __name__ == '__main__':
    main()
