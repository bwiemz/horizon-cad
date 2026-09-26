# Releasing Horizon CAD

A release is made from a tag. The Release workflow
(`.github/workflows/release.yml`) builds and tests it on Windows, Linux and
macOS, packages it, and opens a **draft** GitHub release for a person to
check and publish.

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
   - the macOS disk image (`HorizonCAD-<version>-macOS-arm64.dmg`), for
     Apple silicon;
   - `SHA256SUMS.txt`;
   - the notes: a package that is not signed is listed at the top, with
     what a user does to run it.

   Install and start each package, then publish the release.

## Signing

Each package is signed when the repository has the secrets for it
(Settings ▸ Secrets and variables ▸ Actions). Without them the workflow
still packages everything, and the release notes say what is not signed.

| Secret | What it is |
| --- | --- |
| `WINDOWS_CERTIFICATE` | A code-signing certificate and its key, as a `.pfx` file, base64-encoded (`base64 -w0 cert.pfx`). |
| `WINDOWS_CERTIFICATE_PASSWORD` | The `.pfx` file's password. |
| `MACOS_CERTIFICATE` | A "Developer ID Application" certificate and its key, exported from Keychain Access as a `.p12` file, base64-encoded. |
| `MACOS_CERTIFICATE_PASSWORD` | The `.p12` file's password. |
| `MACOS_SIGNING_IDENTITY` | The certificate's name, for example `Developer ID Application: Jane Doe (AB12CD34EF)`. |
| `APPLE_ID` | The Apple ID of the developer account, for notarization. |
| `APPLE_TEAM_ID` | The account's team ID (the ten characters in the identity). |
| `APPLE_APP_PASSWORD` | An app-specific password for that Apple ID (appleid.apple.com ▸ Sign-In and Security). |

- **Windows.** `signtool` signs `horizon.exe` before it is packed, then
  the installer, with a timestamp from DigiCert's public server. A
  certificate issued since June 2023 keeps its key in hardware or a cloud
  service and cannot be exported as a `.pfx`. For one of those, replace the
  two signing steps with the provider's own (Azure Trusted Signing, for
  example); the rest of the workflow is unchanged.
- **macOS.** The certificate goes into a keychain made for the run.
  `packaging/macos/make-dmg.sh` signs every library, plug-in and framework
  in the bundle, then the bundle, with the hardened runtime and a
  timestamp, then the disk image. With the three Apple secrets as well,
  the disk image is notarized and the ticket stapled to it, so it opens on
  a Mac that is offline.
- **Without the macOS secrets** the bundle is signed ad hoc (with no
  identity). An Apple silicon Mac runs no unsigned code at all, and
  macdeployqt's changes undo the linker's own signature.

## What a package holds

- The executable, and the translations and samples next to it (on macOS,
  in the bundle's `Contents/Resources`).
- `LICENSE`, `THIRD_PARTY_NOTICES.md`, and the licence text of every library
  vcpkg built into it (under `third-party/`).
- On Windows, the Qt runtime (from Qt's deployment script).
- On macOS, the application bundle `HorizonCAD.app`, with Qt's frameworks
  and plug-ins in it (Qt's deployment script runs macdeployqt), its icon,
  and the documents it opens. The licences are in its `Contents/Resources/doc`.
  The disk image holds it beside a link to Applications. It runs on the
  macOS it was built on (14, Sonoma) or later.
- On Linux:
  - in the AppImage, the libraries linuxdeploy bundles;
  - in the tarball, the desktop entry, AppStream metadata and icons.

Python scripting is left out of release builds until it is sandboxed.
