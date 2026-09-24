#!/usr/bin/env bash
# Builds a Horizon CAD AppImage from a configured and built release tree.
#
#   packaging/linux/build-appimage.sh <build dir> [output dir]
#
# Needs linuxdeploy and linuxdeploy-plugin-qt on PATH (the release workflow
# downloads both), and QMAKE pointing at the qmake of the Qt the build used,
# so the Qt plugin bundles that Qt and its plugins.
set -euo pipefail

build="${1:?usage: build-appimage.sh <build dir> [output dir]}"
out="${2:-${PWD}}"
app_id="io.github.bwiemz.HorizonCAD"

work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT
appdir="${work}/AppDir"

# The install tree: executable, translations, desktop entry, metadata, icons,
# licences. Then linuxdeploy adds the libraries and Qt plugins it needs.
cmake --install "${build}" --prefix "${appdir}/usr"

mkdir -p "${out}"
(
    cd "${out}"
    linuxdeploy \
        --appdir "${appdir}" \
        --plugin qt \
        --desktop-file "${appdir}/usr/share/applications/${app_id}.desktop" \
        --icon-file "${appdir}/usr/share/icons/hicolor/256x256/apps/${app_id}.png" \
        --output appimage
)
echo "AppImage written to ${out}"
