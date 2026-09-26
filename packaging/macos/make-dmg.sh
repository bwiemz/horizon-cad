#!/usr/bin/env bash
# Builds a Horizon CAD disk image from a configured and built release tree.
#
#   packaging/macos/make-dmg.sh <build dir> [output dir]
#
# The install tree is the application bundle, HorizonCAD.app: `cmake
# --install` runs Qt's deployment script, which has macdeployqt put Qt's
# frameworks and plugins in it. The bundle is made to load nothing from
# outside itself and the system, signed, and put on a disk image beside a link
# to /Applications to drag it to.
#
# With HZ_MACOS_SIGNING_IDENTITY set (a Developer ID Application identity in a
# keychain), the bundle is signed with it, with the hardened runtime and a
# timestamp as notarization requires, and so is the disk image. Without it,
# the bundle is signed ad hoc: an Apple silicon Mac runs no unsigned code, and
# the edits macdeployqt makes undo the linker's own signature.
set -euo pipefail

build="${1:?usage: make-dmg.sh <build dir> [output dir]}"
out="${2:-${PWD}}"
identity="${HZ_MACOS_SIGNING_IDENTITY:-}"
source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
version=$(sed -n 's/^ *VERSION \([0-9.]*\)$/\1/p' "${source_dir}/CMakeLists.txt" | head -n 1)

work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT
stage="${work}/stage"
cmake --install "${build}" --prefix "${stage}"
app="${stage}/HorizonCAD.app"
test -x "${app}/Contents/MacOS/HorizonCAD"

# Every Mach-O file in the bundle: the executable, frameworks, plugins.
binaries=()
while IFS= read -r -d '' file; do
    if [[ "$(file -b "${file}")" == Mach-O* ]]; then binaries+=("${file}"); fi
done < <(find "${app}/Contents" -type f -print0)

# A search path outside the bundle (the build's Homebrew Qt) would load that
# Qt on a Mac that has it, in place of the bundle's own.
for binary in "${binaries[@]}"; do
    while IFS= read -r rpath; do
        if [[ "${rpath}" != @* ]]; then
            echo "Removing the search path ${rpath} from ${binary#"${app}"/}"
            install_name_tool -delete_rpath "${rpath}" "${binary}"
        fi
    done < <(otool -l "${binary}" | awk '$1 == "cmd" && $2 == "LC_RPATH" { getline; getline; print $2 }')
done

# Everything they load is in the bundle or the system's.
outside=0
for binary in "${binaries[@]}"; do
    while IFS= read -r dependency; do
        case "${dependency}" in
            /System/* | /usr/lib/* | @executable_path/* | @loader_path/* | @rpath/*) ;;
            *)
                echo "error: ${binary#"${app}"/} loads ${dependency}, outside the bundle" >&2
                outside=1
                ;;
        esac
    done < <(otool -L "${binary}" | tail -n +2 | awk '{ print $1 }')
done
if [[ "${outside}" -ne 0 ]]; then exit 1; fi

# Signed inside out: each library and plugin, each framework, then the bundle.
sign=(codesign --force --sign "${identity:--}")
if [[ -n "${identity}" ]]; then sign+=(--options runtime --timestamp); fi
for binary in "${binaries[@]}"; do
    if [[ "${binary}" != "${app}/Contents/MacOS/"* ]]; then "${sign[@]}" "${binary}"; fi
done
if [[ -d "${app}/Contents/Frameworks" ]]; then
    while IFS= read -r -d '' framework; do
        "${sign[@]}" "${framework}"
    done < <(find "${app}/Contents/Frameworks" -maxdepth 1 -name '*.framework' -type d -print0)
fi
"${sign[@]}" "${app}"
codesign --verify --deep --strict --verbose=2 "${app}"

ln -s /Applications "${stage}/Applications"
mkdir -p "${out}"
dmg="${out}/HorizonCAD-${version}-macOS-$(uname -m).dmg"
rm -f "${dmg}"
# hdiutil now and then finds the new volume busy on a CI runner; try again.
for attempt in 1 2 3; do
    if hdiutil create -volname "Horizon CAD ${version}" -srcfolder "${stage}" \
        -format UDZO "${dmg}"; then
        break
    fi
    if [[ "${attempt}" -eq 3 ]]; then exit 1; fi
    sleep 10
done
if [[ -n "${identity}" ]]; then codesign --force --timestamp --sign "${identity}" "${dmg}"; fi
echo "Disk image written to ${dmg}"
