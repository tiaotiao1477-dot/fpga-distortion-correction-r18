"""Generate deterministic Python-reference vectors for the RTL bilinear unit.

Example:
    python -B -m sw.reference_fixed.bilinear_rtl_compare \
        --vectors rtl/remap/bilinear_vectors.txt --count 10000 --coord-frac-bits 10
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path
from typing import Iterable


def interpolate_rgb(
    p00: int, p10: int, p01: int, p11: int, dx: int, dy: int, frac_bits: int
) -> int:
    scale = 1 << frac_bits
    output_channels: list[int] = []
    for shift in (16, 8, 0):
        c00, c10, c01, c11 = (
            (pixel >> shift) & 0xFF for pixel in (p00, p10, p01, p11)
        )
        h0 = c00 * scale + (c10 - c00) * dx
        h1 = c01 * scale + (c11 - c01) * dx
        numerator = h0 * scale + (h1 - h0) * dy
        channel = (numerator + (scale * scale // 2)) // (scale * scale)
        output_channels.append(min(255, max(0, channel)))
    return (output_channels[0] << 16) | (output_channels[1] << 8) | output_channels[2]


def generate_vectors(path: Path, count: int, frac_bits: int, seed: int) -> None:
    if count < 5:
        raise ValueError("count must be at least 5 to include directed endpoint cases")
    if not 1 <= frac_bits <= 16:
        raise ValueError("coord-frac-bits must be between 1 and 16")

    scale = 1 << frac_bits
    directed = [
        (0x000000, 0x000000, 0x000000, 0x000000, 0, 0),
        (0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, scale - 1, scale - 1),
        (0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, 0, 0),
        (0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, scale - 1, scale - 1),
        (0x0064C8, 0x64C8FF, 0xC80064, 0xFFFF00, scale // 2, scale // 2),
    ]
    rng = random.Random(seed)
    rows: list[tuple[int, ...]] = []
    for index in range(count):
        if index < len(directed):
            p00, p10, p01, p11, dx, dy = directed[index]
        else:
            p00, p10, p01, p11 = (rng.getrandbits(24) for _ in range(4))
            dx = rng.randrange(scale)
            dy = rng.randrange(scale)
        expected = interpolate_rgb(p00, p10, p01, p11, dx, dy, frac_bits)
        frame_id = index // 4096
        cfg_version = (index // 2048 + 1) & 0xFFFF
        rows.append((
            p00, p10, p01, p11, dx, dy, expected,
            frame_id, cfg_version, index,
        ))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as vector_file:
        vector_file.write(f"{count}\n")
        for row in rows:
            p00, p10, p01, p11, dx, dy, expected, frame_id, cfg, tag = row
            vector_file.write(
                f"{p00:06x} {p10:06x} {p01:06x} {p11:06x} "
                f"{dx:04x} {dy:04x} {expected:06x} "
                f"{frame_id:08x} {cfg:04x} {tag:08x}\n"
            )


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vectors", type=Path, default=Path("rtl/remap/bilinear_vectors.txt"))
    parser.add_argument("--count", type=int, default=10000)
    parser.add_argument("--coord-frac-bits", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0x5640)
    args = parser.parse_args(argv)
    try:
        generate_vectors(args.vectors, args.count, args.coord_frac_bits, args.seed)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(
        f"Wrote {args.count} bilinear vectors with COORD_F={args.coord_frac_bits} "
        f"to {args.vectors.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
