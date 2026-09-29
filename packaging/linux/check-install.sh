#!/usr/bin/env bash
# Installs a Linux package of Horizon CAD as a user would, on a machine that
# never built it, and starts it with --self-test: it models, edits, saves and
# reads back a part, an assembly and a drawing, opens the samples, loads the
# translations, and shows its window (docs/BETA.md). Run as root in a fresh
# container (.github/workflows/install-check.yml); the application runs as a
# user of its own.
#
#   packaging/linux/check-install.sh appimage|tarball <folder with the packages>
#
# The machine gets what a desktop has (X11's client libraries, OpenGL, fonts)
# and a display to draw on (Xvfb, with Mesa drawing in software). The tarball
# also gets the libraries docs/INSTALL.md says it needs; the AppImage carries them.
set -euo pipefail

usage="usage: check-install.sh appimage|tarball <folder with the packages>"
kind="${1:?${usage}}"
dist="$(cd "${2:?${usage}}" && pwd)"

if command -v apt-get >/dev/null; then
    export DEBIAN_FRONTEND=noninteractive
    desktop=(xvfb xauth libgl1 libglx0 libopengl0 libegl1 libgl1-mesa-dri libx11-6 libx11-xcb1
             libxcb1 libxkbcommon0 libxkbcommon-x11-0 fontconfig libfontconfig1 libfreetype6
             fonts-dejavu-core)
    # docs/INSTALL.md, "Linux": keep the lists the same.
    tarball=(libxcb-cursor0 libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1
             libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0
             libxcb-sync1 libxcb-xfixes0 libxcb-xinput0 libxcb-xkb1)
    install() { apt-get update -qq && apt-get install -y -qq --no-install-recommends "$@"; }
elif command -v dnf >/dev/null; then
    desktop=(xorg-x11-server-Xvfb mesa-libGL mesa-libEGL mesa-dri-drivers libglvnd-opengl
             libglvnd-glx libX11 libX11-xcb libxcb libxkbcommon libxkbcommon-x11 fontconfig
             freetype dejavu-sans-fonts util-linux shadow-utils)
    tarball=(xcb-util-cursor xcb-util-image xcb-util-keysyms xcb-util-renderutil xcb-util-wm)
    install() { dnf install -y -q "$@"; }
else
    echo "check-install.sh: neither apt-get nor dnf is here" >&2
    exit 2
fi
packages=("${desktop[@]}")
if [[ "${kind}" == tarball ]]; then packages+=("${tarball[@]}"); fi
install "${packages[@]}"

useradd --create-home tester
case "${kind}" in
    appimage)
        # In the user's own folder, made executable, as docs/INSTALL.md says.
        dir=/home/tester/Applications
        mkdir -p "${dir}"
        cp "${dist}"/*.AppImage "${dir}/"
        chmod +x "${dir}"/*.AppImage
        chown -R tester: "${dir}"
        appimage=("${dir}"/*.AppImage)
        # A container has no FUSE: the AppImage runs from a folder it
        # extracts itself to, as it offers to without one.
        command=("${appimage[0]}" --appimage-extract-and-run)
        ;;
    tarball)
        # Where a system-wide copy goes, which its user cannot write to.
        mkdir -p /opt/HorizonCAD
        tar -xzf "${dist}"/*.tar.gz -C /opt/HorizonCAD --strip-components=1
        command=(/opt/HorizonCAD/bin/horizon)
        ;;
    *)
        echo "check-install.sh: ${kind} is not appimage or tarball" >&2
        exit 2
        ;;
esac

Xvfb :99 -screen 0 1280x1024x24 -ac -nolisten tcp &
xvfb=$!
trap 'kill "${xvfb}" 2>/dev/null || true' EXIT
sleep 2

log="${PWD}/self-test.log"
set +e
runuser -u tester -- env HOME=/home/tester DISPLAY=:99 LANG=C.UTF-8 QT_QPA_PLATFORM=xcb \
    LIBGL_ALWAYS_SOFTWARE=1 timeout 120 "${command[@]}" --self-test >"${log}" 2>&1
status=$?
set -e
cat "${log}"

# What it wrote went to its user's folders.
userlog="/home/tester/.local/share/Horizon CAD Project/Horizon CAD/logs/horizon.log"
if [[ ! -f "${userlog}" ]]; then
    echo "::error::no log in ${userlog}"
    exit 1
fi
if grep -q 'Fontconfig error' "${log}"; then
    echo "::error::fontconfig could not load this system's configuration"
    exit 1
fi
# The system's own fontconfig reads the system's configuration (#182): a
# warning means one built into the package, for another version, is back.
warnings=$(grep -c 'Fontconfig warning' "${log}" || true)
if [[ "${warnings}" -gt 0 ]]; then
    echo "::error::fontconfig warned ${warnings} times on this system's configuration"
    exit 1
fi
if [[ "${status}" -ne 0 ]]; then
    echo "::error::horizon --self-test exited ${status}"
    tail -n 40 "${userlog}"
    exit 1
fi
echo "${kind} installed and checked"
