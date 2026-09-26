#!/usr/bin/env python3
"""Packs rendered PNG icons into a macOS .icns file.

    make-icns.py <output.icns> <png>...

An .icns file is the bytes "icns" and its length, then one entry per image:
a four-letter type, its length, and (for the types written here) a PNG.
Each type is one size at one scale, so each PNG is placed by its pixel size
under every type that size serves. It does what Apple's iconutil does, and
runs where iconutil does not; render.sh calls it.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

# The types each pixel size is written as: 16 to 512 points, at 1x and 2x.
TYPES_BY_SIZE: dict[int, list[bytes]] = {
    16: [b"icp4"],  # 16 pt
    32: [b"icp5", b"ic11"],  # 32 pt, and 16 pt at 2x
    64: [b"ic12"],  # 32 pt at 2x
    128: [b"ic07"],  # 128 pt
    256: [b"ic08", b"ic13"],  # 256 pt, and 128 pt at 2x
    512: [b"ic09", b"ic14"],  # 512 pt, and 256 pt at 2x
    1024: [b"ic10"],  # 512 pt at 2x
}


def png_size(data: bytes, name: str) -> int:
    """The width of a square PNG, read from its header."""
    if data[:8] != PNG_SIGNATURE or data[12:16] != b"IHDR":
        raise ValueError(f"{name} is not a PNG")
    width, height = struct.unpack(">II", data[16:24])
    if width != height:
        raise ValueError(f"{name} is {width}x{height}, not square")
    return int(width)


def pack(pngs: list[Path]) -> bytes:
    """The .icns bytes holding the given PNGs, one of each size needed."""
    by_size: dict[int, bytes] = {}
    for path in pngs:
        data = path.read_bytes()
        size = png_size(data, str(path))
        if size not in TYPES_BY_SIZE:
            raise ValueError(f"{path} is {size} px; .icns sizes are {sorted(TYPES_BY_SIZE)}")
        by_size[size] = data
    missing = sorted(set(TYPES_BY_SIZE) - set(by_size))
    if missing:
        raise ValueError(f"no PNG of {missing} px")
    body = b""
    for size in sorted(TYPES_BY_SIZE):
        for icon_type in TYPES_BY_SIZE[size]:
            data = by_size[size]
            body += icon_type + struct.pack(">I", 8 + len(data)) + data
    return b"icns" + struct.pack(">I", 8 + len(body)) + body


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        icns = pack([Path(name) for name in argv[2:]])
    except (OSError, ValueError) as error:
        print(f"make-icns: {error}", file=sys.stderr)
        return 1
    Path(argv[1]).write_bytes(icns)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
