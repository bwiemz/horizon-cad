# Installing Horizon CAD

Packages are on the [Releases](https://github.com/bwiemz/horizon-cad/releases)
page. A beta is marked **Pre-release** there.

| System | Package | Needs |
|---|---|---|
| Windows 10 or 11, 64-bit | `HorizonCAD-<version>-win64.exe` | OpenGL 3.3 (any graphics driver from the last ten years) |
| macOS 14 (Sonoma) or later, Apple silicon | `HorizonCAD-<version>-macOS-arm64.dmg` | Not an Intel Mac |
| Linux, x86-64 | `HorizonCAD-<version>-x86_64.AppImage` or `HorizonCAD-<version>-Linux.tar.gz` | X11 or Wayland (it runs through XWayland), OpenGL 3.3, glibc 2.35 or later (Ubuntu 22.04, Debian 12, Fedora 36 or newer) |

`SHA256SUMS.txt` beside the packages has each one's checksum.

## Windows

Run the installer. It installs into `Program Files\HorizonCAD` and puts a
**Horizon CAD** shortcut on the desktop. To remove it, use **Settings ▸
Apps**, or `Uninstall.exe` in its folder.

If the installer is not signed, Windows SmartScreen warns before it runs:
choose **More info**, then **Run anyway**. The release notes say whether it is.

## macOS

Open the disk image and drag **HorizonCAD** to **Applications**.

If the disk image is not notarized, macOS will not open the application the
first time. After that first try, open **System Settings ▸ Privacy &
Security** and choose **Open Anyway**. The release notes say whether it is.

## Linux

**The AppImage** holds everything it needs. Make it executable and run it:

```sh
chmod +x HorizonCAD-*-x86_64.AppImage
./HorizonCAD-*-x86_64.AppImage
```

It needs FUSE (`fusermount3` or `fusermount`), which desktops have. Without
it, add `--appimage-extract-and-run`.

**The tarball** is for a system-wide copy, for example in `/opt`. It has the
desktop entry, icons and file types under `share/`. It uses these libraries
of the system; install them first:

- Debian and Ubuntu:
  `sudo apt install libgl1 libglx0 libopengl0 libegl1 libx11-6 libx11-xcb1 libxcb1 libxkbcommon0 libxkbcommon-x11-0 fontconfig libxcb-cursor0 libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0 libxcb-sync1 libxcb-xfixes0 libxcb-xinput0 libxcb-xkb1`
- Fedora:
  `sudo dnf install mesa-libGL mesa-libEGL libglvnd-opengl libglvnd-glx libX11 libX11-xcb libxcb libxkbcommon libxkbcommon-x11 fontconfig xcb-util-cursor xcb-util-image xcb-util-keysyms xcb-util-renderutil xcb-util-wm`

`libxcb-cursor0` is the one a desktop most often lacks. Without it, the
program does not start: `error while loading shared libraries:
libxcb-cursor.so.0`.

## Checking an installed copy

`--self-test` does what a new user would do, in a temporary folder. It makes,
edits, saves and reads back a part, an assembly and a drawing, and exports
the drawing. It opens the samples, loads the translations, and checks the
window can draw. It says what it found and exits 0 when all is well. Every
package passes it on clean machines before it is released.

- Linux: `./HorizonCAD-*-x86_64.AppImage --self-test`, or
  `/opt/HorizonCAD/bin/horizon --self-test`.
- macOS: `/Applications/HorizonCAD.app/Contents/MacOS/HorizonCAD --self-test`.
- Windows, in PowerShell (the program has no console of its own, so its
  output goes to a file):

  ```powershell
  Start-Process -Wait -RedirectStandardOutput self-test.txt `
      "$env:ProgramFiles\HorizonCAD\bin\horizon.exe" --self-test
  Get-Content self-test.txt
  ```

## Where your files go

Horizon CAD keeps its settings, log, crash reports and recovery snapshots
in the user's folders, never beside the program:

| System | Log, crash reports and recovery |
|---|---|
| Windows | `%LOCALAPPDATA%\Horizon CAD Project\Horizon CAD\` |
| macOS | `~/Library/Application Support/Horizon CAD Project/Horizon CAD/` |
| Linux | `~/.local/share/Horizon CAD Project/Horizon CAD/` |

The log is `logs/horizon.log`, and crash reports are in `crashes/`. Nothing
is sent anywhere.
