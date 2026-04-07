#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------
# build-appimage.sh  --  Build an AppImage for Horizon CAD (Linux)
#
# Usage:
#   ./installer/linux/build-appimage.sh
#
# Prerequisites:
#   - CMake, Ninja, vcpkg (VCPKG_ROOT set)
#   - linuxdeploy (download from https://github.com/linuxdeploy/linuxdeploy)
# ------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/linux-release"
APPDIR="${BUILD_DIR}/AppDir"

echo "==> Configuring release build..."
cmake --preset linux-release

echo "==> Building..."
cmake --build "${BUILD_DIR}" --config Release

echo "==> Preparing AppDir structure..."
rm -rf "${APPDIR}"
mkdir -p "${APPDIR}/usr/bin"
mkdir -p "${APPDIR}/usr/share/applications"
mkdir -p "${APPDIR}/usr/share/icons/hicolor/256x256/apps"

echo "==> Copying binary..."
cp "${BUILD_DIR}/src/app/horizon" "${APPDIR}/usr/bin/horizon"

echo "==> Creating .desktop file..."
cat > "${APPDIR}/usr/share/applications/horizon-cad.desktop" << 'DESKTOP'
[Desktop Entry]
Type=Application
Name=Horizon CAD
Comment=Open-source parametric 2D/3D CAD software
Exec=horizon
Icon=horizon-cad
Categories=Graphics;Engineering;
Terminal=false
DESKTOP

echo "==> Creating placeholder icon..."
# Generate a simple SVG icon if no icon file exists
ICON_SRC="${PROJECT_ROOT}/resources/horizon-cad.png"
ICON_DST="${APPDIR}/usr/share/icons/hicolor/256x256/apps/horizon-cad.png"
if [[ -f "${ICON_SRC}" ]]; then
    cp "${ICON_SRC}" "${ICON_DST}"
else
    echo "    (no icon found at ${ICON_SRC}, creating placeholder)"
    # Create a minimal 1x1 PNG as placeholder
    printf '\x89PNG\r\n\x1a\n' > "${ICON_DST}"
    echo "    WARNING: Replace ${ICON_DST} with a real 256x256 icon."
fi

echo ""
echo "==> AppDir prepared at: ${APPDIR}"
echo ""
echo "To create the AppImage, run:"
echo ""
echo "  linuxdeploy --appdir ${APPDIR} --output appimage"
echo ""
echo "Download linuxdeploy from:"
echo "  https://github.com/linuxdeploy/linuxdeploy/releases"
echo ""
