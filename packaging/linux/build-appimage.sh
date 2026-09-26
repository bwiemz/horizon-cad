#!/usr/bin/env bash
# Builds a Horizon CAD AppImage from a configured and built release tree.
#
#   packaging/linux/build-appimage.sh <build dir> [output dir]
#
# Needs linuxdeploy on PATH (the release workflow downloads it). For a build
# against a shared Qt, also linuxdeploy-plugin-qt and QMAKE pointing at that
# Qt's qmake, so the plugin bundles Qt and its plugins. A build against a
# static Qt (vcpkg's x64-linux) has Qt inside the executable: leave QMAKE
# unset and the plugin is not used.
set -euo pipefail

build="${1:?usage: build-appimage.sh <build dir> [output dir]}"
out="${2:-${PWD}}"
app_id="io.github.bwiemz.HorizonCAD"

work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT
appdir="${work}/AppDir"

# The install tree: executable, translations, samples, desktop entry, metadata, icons,
# licences. Then linuxdeploy adds the libraries and Qt plugins it needs.
cmake --install "${build}" --prefix "${appdir}/usr"

mkdir -p "${out}"
(
    cd "${out}"
    plugin=()
    if [[ -n "${QMAKE:-}" ]]; then plugin=(--plugin qt); fi
    linuxdeploy \
        --appdir "${appdir}" \
        "${plugin[@]}" \
        --desktop-file "${appdir}/usr/share/applications/${app_id}.desktop" \
        --icon-file "${appdir}/usr/share/icons/hicolor/256x256/apps/${app_id}.png" \
        --output appimage
)
echo "AppImage written to ${out}"
