#!/usr/bin/env bash
# Renders the application icon from horizon-cad.svg: the PNG sizes desktops
# use, and a Windows .ico holding them. Run after changing the SVG; the
# rendered files are committed, so a build needs none of these tools.
# Needs rsvg-convert (librsvg) and ImageMagick.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
sizes=(16 24 32 48 64 128 256 512)
for size in "${sizes[@]}"; do
    rsvg-convert -w "${size}" -h "${size}" horizon-cad.svg -o "horizon-cad-${size}.png"
done
magick horizon-cad-16.png horizon-cad-24.png horizon-cad-32.png horizon-cad-48.png \
    horizon-cad-64.png horizon-cad-128.png horizon-cad-256.png horizon-cad.ico
