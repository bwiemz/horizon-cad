# Milestone 5: Shippable (Phases 115–118): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

A release someone can install:
- the version is right everywhere;
- there is an installer and an AppImage;
- a tag produces checked artifacts;
- the repository says how to contribute and how to report a vulnerability.

## Phase 115: One version

### What the audit found

- `project(VERSION 0.1.0)` reached the application through a compile
  definition on the executable only. The About box read it back from the
  QApplication, and the source revision came from a configure-time
  definition that went stale with every commit.
- The CHANGELOG called the 80-phase roadmap "1.0.0", while the build, the
  README and the installer said 0.1.0. Nothing was ever built as 1.0.0.
- **The installer configuration was never written.**
  - `cmake/` is on `CMAKE_MODULE_PATH`, and the settings file was named
    `cmake/CPack.cmake`, so `include(CPack)` found that file instead of
    CMake's CPack module and never produced `CPackConfig.cmake`.
  - The settings were also included after `include(CPack)`, which would
    have ignored them anyway.

### As built

- **Version header.** `cmake/Version.h.in` is generated into
  `build/generated/include/horizon/Version.h` (`hz::version::kString`,
  `kMajor`, `kMinor`, `kPatch`). It is exposed through the interface
  target `Horizon::Version`, linked by the application, the UI and the
  window tests.
- **Revision header.** `cmake/WriteRevision.cmake` writes
  `horizon/Revision.h` (`kRevision`):
  - at configure time, so it exists for the Static Analysis job, which
    never builds;
  - and on every build, rewriting only when the revision changed, so
    nothing recompiles for nothing.
- **Where the version is read.** The About box, `horizon --version` (through
  `QApplication::applicationVersion`) and the startup log line (version and
  revision) all read the generated header.
- **Installer.** The CPack settings are now `cmake/CPackSettings.cmake`,
  included before `include(CPack)`, and they set `CPACK_PACKAGE_VERSION`.
  `CPackConfig.cmake` is generated for the first time, with version 0.1.0.
  The generator is NSIS on Windows only, where the tool exists, and a
  tarball elsewhere. `cpack` on Linux now produces
  `HorizonCAD-0.1.0-Linux.tar.gz`, the project's first package; it holds
  only the executable, and Qt, translations and licences come in 116.
- **CHANGELOG and README.**
  - The "1.0.0" section now carries 0.1.0, not released.
  - "Post-1.0" work is called "post-roadmap".
  - A Versions paragraph says `project()` is the one source.
- **For the owner.** Whether the next release is 0.2.0 or 1.0.0 is your
  call; either way it is changed in one place.

## Phase 116: Packaging

- `include(CPack)` ordering (done in 115, with the name clash).
- Install rules for the Qt runtime (`qt_generate_deploy_app_script`),
  translations and licences.
- Icon, `.desktop` and AppStream metadata.
- Windows NSIS/WiX.
- Linux AppImage via linuxdeploy.
- Third-party notices.

**As built.**
- **Icon.** `packaging/icons/horizon-cad.svg` shows a part rising over the
  horizon. `render.sh` renders it into PNGs from 16 to 512 px and a Windows
  `.ico`, and the rendered files are committed, so a build needs no tools.
  The icon is used as:
  - the window icon (a Qt resource);
  - the executable's icon on Windows (`horizon.rc`);
  - the NSIS installer's icons;
  - the Linux hicolor icons.
- **Linux desktop integration** (`packaging/linux`), under the application
  id `io.github.bwiemz.HorizonCAD`:
  - a `.desktop` entry, valid under `desktop-file-validate`;
  - AppStream metadata, valid under `appstreamcli validate-tree --no-net`,
    configured with the version and a date that honours SOURCE_DATE_EPOCH;
  - MIME types for `.hcad`, `.hzpart` and `.hzasm`.

  `setDesktopFileName` ties the running window to the entry.
- **Install rules** (`cmake/HorizonInstall.cmake`):
  - LICENSE, `THIRD_PARTY_NOTICES.md`, the README and the CHANGELOG;
  - the licence file of every library vcpkg built into the package, taken
    from vcpkg's own records (build-only ports are left out);
  - on Linux, the desktop files and icons;
  - translations next to the executable, where `LocaleManager` looks;
  - on Windows, the Qt runtime through `qt_generate_deploy_app_script`.
- **`THIRD_PARTY_NOTICES.md`** lists each library, what it is used for and
  its licence, and says how Qt's LGPL is met: Qt is linked dynamically in a
  package.
- **AppImage.** `packaging/linux/build-appimage.sh <build dir>` installs
  into an AppDir and runs linuxdeploy with its Qt plugin. The release
  workflow (117) provides linuxdeploy; it is not downloaded here.
- **Test.** `PackagingInstallTree`, on Linux and macOS, installs into a
  scratch prefix and checks the tree, validating the desktop entry when the
  tool is there. Windows packaging is exercised by the release workflow.
- **Not done:**
  - WiX: NSIS stays the Windows installer.
  - A macOS bundle.

## Phase 117: Release pipeline

- A tag-triggered workflow producing Release artifacts with checksums.
- Release-configuration CI jobs.
- A working vcpkg binary cache: the `x-gha` backend restores nothing, so Qt
  is rebuilt from source in every job.
- Dependabot for actions.

## Phase 118: Governance

- The licence resolved (**owner decision**: LICENSE is GPL v3 text while
  the README says MIT).
- SECURITY.md, CODE_OF_CONDUCT, issue and PR templates, a CONTRIBUTING
  refresh.
- Branch protection on `master` (**owner action**).
