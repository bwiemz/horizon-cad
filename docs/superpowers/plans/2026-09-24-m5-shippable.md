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
  - the executable's icon on Windows (`horizon.rc.in`). CMake fills in the
    icon's absolute path, because rc.exe resolves a relative one against its
    working directory, which differs between Ninja and Visual Studio;
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
  its licence, and says how Qt's LGPL is met. The Windows installer links Qt
  dynamically. The Linux release packages link vcpkg's static Qt, and are
  relinkable because the whole source is published and each release tagged.
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

**As built.**
- **vcpkg binary cache.** It did nothing, for two reasons:
  - the `x-gha` backend relied on GitHub's retired cache API;
  - `VCPKG_BINARY_SOURCES` was set only on the run-vcpkg step, while vcpkg
    installs during the configure step.

  Every job now uses vcpkg's `files` backend in the workspace, set at job
  level, with `actions/cache` restoring and saving it. The key is the OS,
  triplet, manifest hash and baseline.
- **Release CI job.** `Build (ubuntu-22.04, Release)` uses `linux-release`
  with `-Werror`.
  - A local Release build found GCC reporting an `-Warray-bounds` false
    positive, an insert at index −1, in `PolylineEditTool`. The index is now
    an unsigned position checked against the size.
  - All 1310 tests pass in Release.
- **Release workflow** (`.github/workflows/release.yml`), triggered by a tag
  `v*` or run by hand:
  - **Linux:** checks the tag against `project()`, builds and tests
    Release, and makes a tarball with CPack and an AppImage with linuxdeploy
    (run without FUSE).
  - **Windows:** builds and tests Release (`ci-windows-release`, a new
    preset) and makes an NSIS installer.
  - **Release:** gathers the packages, writes `SHA256SUMS.txt`, and opens a
    draft release for the owner to publish.
  - Scripting is off in release builds until Phase 120.
- **Dependabot** checks GitHub Actions weekly, grouped into one pull request.
- **`docs/RELEASING.md`** describes how to cut a release.
- **Not verified here:** the release workflow needs GitHub's runners (NSIS,
  and downloading linuxdeploy). Run it by hand once before the first tag.

## Phase 118: Governance

- The licence resolved (**owner decision**: LICENSE is GPL v3 text while
  the README says MIT).
- SECURITY.md, CODE_OF_CONDUCT, issue and PR templates, a CONTRIBUTING
  refresh.
- Branch protection on `master` (**owner action**).

**As built.**
- **`SECURITY.md`.**
  - Reports go through GitHub's private vulnerability reporting, not public
    issues and not an e-mail address in the repository.
  - It names the attack surface: the file readers (native, DXF, STEP), the
    only code that takes input from outside.
  - It says plainly that scripts run with the user's full rights, and that
    a plugin manifest's permissions are not enforced (the application does
    not run plugins yet), so running either is trusting its author.
  - Only the latest release is supported.
- **`CODE_OF_CONDUCT.md`** adopts the Contributor Covenant 2.1 by reference;
  reports go through the same private channel.
- **Issue templates** (issue forms):
  - a bug report asks for the version from Help ▸ About, the platform, steps,
    and the log's location on each platform;
  - a feature request asks for the problem before the solution;
  - blank issues are off, and security reports are pointed at the private
    channel.
- **Pull request template:** what and why, how it was tested, and the CI
  gates as a checklist.
- **`docs/CONTRIBUTING.md`** was out of date:
  - it gave the old Windows machine's CMake path;
  - it showed tool handlers without the event and world position they take.

  It now also covers:
  - snapping and picking through the viewport;
  - `copyStyleFrom` for pieces made from an entity;
  - the window tests and their helpers;
  - the five CI gates and how to run each locally;
  - the three ways code passing a newer local toolchain fails CI's older
    one.
- **README** Contributing links all three documents.
- **Left for the owner:**
  - **The licence.** `LICENSE` is the GPL v3 and the README says MIT. Which
    one applies is the owner's call. Until it is made, the AppStream
    metadata declares no project licence.
  - **Branch protection** on `master`: require the five CI gates to pass
    before a merge.
  - **Private vulnerability reporting** has to be turned on in the
    repository's settings (Security ▸ Private vulnerability reporting) for
    `SECURITY.md`'s channel to exist.
