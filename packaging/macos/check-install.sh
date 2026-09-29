#!/usr/bin/env bash
# Installs Horizon CAD from its disk image as a user would, on a Mac that
# never built it, and starts it with --self-test (see
# packaging/linux/check-install.sh and .github/workflows/install-check.yml).
# --self-test models, edits, saves and reads back a part, an assembly and a
# drawing, opens the samples, loads the translations, and shows the window.
#
#   packaging/macos/check-install.sh <HorizonCAD-...-macOS-arm64.dmg>
set -euo pipefail

dmg="${1:?usage: check-install.sh <disk image>}"
app=/Applications/HorizonCAD.app

# Dragged from the disk image to Applications.
mount="$(mktemp -d)"
hdiutil attach -nobrowse -readonly -mountpoint "${mount}" "${dmg}"
ditto "${mount}/HorizonCAD.app" "${app}"
hdiutil detach "${mount}"

# Nothing in it may load from where it was built: Homebrew's Qt, and the
# rest of Homebrew, are not on a user's Mac. This Mac has Homebrew too, so a
# library found there would start here and not there.
# What each file loads, that is: not a library's own name (otool -L lists
# it first), which macdeployqt leaves as Homebrew's and nothing loads by.
outside=0
while IFS= read -r -d '' file; do
    if file -b "${file}" | grep -q 'Mach-O'; then
        loads=$(otool -L "${file}" | tail -n +2 | awk '{print $1}')
        name=$(otool -D "${file}" | tail -n +2 | head -n 1)
        if [[ -n "${name}" ]]; then loads=$(grep -vxF "${name}" <<<"${loads}" || true); fi
        if grep -E '^(/opt/homebrew/|/usr/local/)' <<<"${loads}"; then
            echo "::error::${file#"${app}/"} loads a library from outside the bundle"
            outside=1
        fi
    fi
done < <(find "${app}" -type f -print0)
test "${outside}" -eq 0
codesign --verify --deep --strict "${app}"

set +e
perl -e 'alarm shift; exec @ARGV' 120 "${app}/Contents/MacOS/HorizonCAD" --self-test 2>&1 \
    | tee self-test.log
status=${PIPESTATUS[0]}
set -e
if [[ "${status}" -eq 3 ]]; then
    # A runner is a virtual machine; its OpenGL may not draw. Everything
    # else passed (5 says otherwise).
    echo "::warning::the application started, but its viewport could not draw on this runner"
    exit 0
fi
if [[ "${status}" -ne 0 ]]; then
    echo "::error::horizon --self-test exited ${status}"
    exit 1
fi
echo "installed and checked"
