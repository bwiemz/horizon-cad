# Releasing Horizon CAD

A release is made from a tag. The Release workflow
(`.github/workflows/release.yml`) builds and tests it on Windows and Linux,
packages it, and opens a **draft** GitHub release for a person to check and
publish.

1. **Set the version** in `project()` in the top-level `CMakeLists.txt`.
   Nothing else holds it: the About box, `horizon --version`, the installer
   and the packages all read it from there.
2. **Move the CHANGELOG's "Unreleased" sections** under a heading for the
   version.
3. **Try the packaging first.** Run the Release workflow by hand (Actions ▸
   Release ▸ Run workflow). It builds every package without releasing, and
   keeps them on the run.
4. **Tag and push:** `git tag v0.2.0 && git push origin v0.2.0`. The workflow
   stops at once if the tag and `project()` disagree.
5. **Check the draft release:**
   - the Windows installer (`HorizonCAD-<version>-win64.exe`);
   - the Linux AppImage and tarball;
   - `SHA256SUMS.txt`.

   Install and start each package, then publish the release.

## What a package holds

- The executable, and the translations next to it.
- `LICENSE`, `THIRD_PARTY_NOTICES.md`, and the licence text of every library
  vcpkg built into it (under `third-party/`).
- On Windows, the Qt runtime (from Qt's deployment script).
- On Linux:
  - in the AppImage, the libraries linuxdeploy bundles;
  - in the tarball, the desktop entry, AppStream metadata and icons.

Python scripting is left out of release builds until it is sandboxed.
