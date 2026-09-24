# Third-party notices

Horizon CAD itself is licensed under the GNU General Public License, version 3
or (at your option) any later version; see [LICENSE](LICENSE).

Horizon CAD is built on the libraries below. Each is used under its own
licence. A package of Horizon CAD carries the full licence text of every
library in it, in `third-party/<library>/copyright` next to this file (taken
from the vcpkg build that made the package).

| Library | Used for | Licence |
|---|---|---|
| [Qt 6](https://www.qt.io/) (Core, Gui, Widgets, OpenGL, OpenGL Widgets) | the application window and viewport | LGPL-3.0-only |
| [spdlog](https://github.com/gabime/spdlog) | logging | MIT |
| [{fmt}](https://github.com/fmtlib/fmt) | text formatting, through spdlog | MIT |
| [nlohmann/json](https://github.com/nlohmann/json) | the native file formats, plugin manifests | MIT |
| [Eigen](https://eigen.tuxfamily.org/) | linear algebra in the constraint solver and kernel | MPL-2.0 |
| [FlatBuffers](https://github.com/google/flatbuffers) | the binary document container | Apache-2.0 |
| [pybind11](https://github.com/pybind/pybind11) | Python scripting, when built with it | BSD-3-Clause |
| [Python](https://www.python.org/) | Python scripting, when built with it | PSF-2.0 |

Qt's own dependencies (zlib, libpng, FreeType, HarfBuzz, PCRE2 and others,
depending on how Qt was built) are listed with their licences under
`third-party/` when vcpkg built Qt into the package. A package built against
an installed Qt carries that Qt's own licence files instead.

GoogleTest (BSD-3-Clause) is used only by the test suite and is not part of
a package.

Qt is used under the LGPL-3.0, which lets a user run Horizon CAD with a
modified Qt:
- **Windows:** the installer links Qt dynamically. Qt's DLLs sit next to
  `horizon.exe` and can be replaced.
- **Linux:** the release packages link a static Qt, built by vcpkg. Horizon
  CAD's complete source and build files are published in this repository,
  and each release is tagged there, so it can be rebuilt against a modified
  Qt.
- **A build against an installed Qt** links that Qt dynamically.

Qt's source is available from <https://download.qt.io/>.
