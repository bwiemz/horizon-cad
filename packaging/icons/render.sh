#!/usr/bin/env bash
# Renders the application icon from horizon-cad.svg: the PNG sizes desktops
# use, a Windows .ico and a macOS .icns holding them. Run after changing the
# SVG; the rendered files are committed, so a build needs none of these tools.
# Needs rsvg-convert (librsvg), ImageMagick and Python 3.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
sizes=(16 24 32 48 64 128 256 512)
for size in "${sizes[@]}"; do
    rsvg-convert -w "${size}" -h "${size}" horizon-cad.svg -o "horizon-cad-${size}.png"
done
magick horizon-cad-16.png horizon-cad-24.png horizon-cad-32.png horizon-cad-48.png \
    horizon-cad-64.png horizon-cad-128.png horizon-cad-256.png horizon-cad.ico
# The .icns also holds 512 pt at 2x, a size nothing else uses.
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT
rsvg-convert -w 1024 -h 1024 horizon-cad.svg -o "${work}/horizon-cad-1024.png"
python3 make-icns.py horizon-cad.icns horizon-cad-16.png horizon-cad-32.png horizon-cad-64.png \
    horizon-cad-128.png horizon-cad-256.png horizon-cad-512.png "${work}/horizon-cad-1024.png"
