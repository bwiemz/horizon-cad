# Milestone 13 — Foundations for new workbenches (Phases 145–146)

Roadmap: [2026-09-25-professional-workflows-roadmap.md](../specs/2026-09-25-professional-workflows-roadmap.md).

## Baseline (master @ a0a8a2a, checked in CI's own logs)

- **CI has never run a window test.** The window tests (`hz_ui_window_tests`,
  171 tests on the M12 branch) are built only when Qt has an offscreen
  platform plugin (`tests/ui/CMakeLists.txt:23`). Qt builds that plugin only
  with FreeType. `vcpkg.json` turns the qtbase defaults off and asks for
  `widgets` and `opengl` alone, and CI's log shows
  `qtbase[core,doubleconversion,gui,opengl,widgets]:x64-linux@6.9.3`. Every
  job therefore skipped the whole suite without a word:
  - Linux Debug ran 1451 tests: the 1421 non-window tests, 29 scripting
    tests and one more.
  - Windows, ASan, TSan and coverage show the same.
  - Coverage reads 1.3 % for `src/ui`, with `MainWindow.cpp` at 0 of 3,510
    lines.
- **The Linux release cannot open a window.** On Linux, vcpkg builds Qt
  statically, so the application gets the platform plugins that were built,
  and none was: no xcb, and no offscreen or minimal (those need FreeType).
  There is no font engine either (no FreeType, no fontconfig). The tarball and
  the AppImage are built from this Qt, and nothing starts them.
- The OpenGL viewport tests skip on the offscreen platform, which has no
  OpenGL. They have only run on this machine, under `QT_QPA_PLATFORM=xcb`.
- Of the 2D tools, only Line, Select, Circle, Rectangle, Text, Stretch and
  Ellipse are driven by a test. Offset, Break, Extend, Polyline Edit, Mirror,
  Rotate, Scale, Hatch, Spline, Leader, Arc, Polyline, Trim's options,
  Fillet, Chamfer and the dimension tools are only run by the smoke test.

## Phase 145: A safety net that catches

### As built
- **Qt as the product needs it.**
  - `vcpkg.json` adds `freetype`, `harfbuzz` and `png` everywhere, and
    `fontconfig`, `xcb` and `xcb-xlib` on Linux.
  - Every CI job and the release install the X11 development packages
    that Qt's xcb plugin builds against. The first attempt failed at Qt's
    configure: x11-xcb, xcb-xinput and xcb-util were missing.
  - The first run rebuilt Qt in every job (about 40–60 min); later runs
    take it from the cache.
- **The window tests run in CI**: 181 of them, on Linux Debug, Windows,
  ASan and coverage. They all passed on their first run there.
- **Every job says how many window tests it ran, and fails on none.** This
  caught a second gap on its first run: the `linux-release` preset turns
  tests off, so CI's Linux Release job and the release workflow ran none at
  all ("No tests were found"). Both now configure with `HZ_BUILD_TESTS=ON`.
- **`horizon --self-test`** shows the window, waits for the viewport to
  draw a whole frame, prints what it found and exits:
  - 0 when a frame was drawn;
  - 3 when the viewport cannot draw (the reason is printed, and the
    "Graphics Problem" box does not hold it);
  - 4 when nothing was drawn within 20 s.
  It opens no recovered session or files. CI's Linux jobs and the release
  start the application this way on Xvfb with Mesa's software OpenGL. The
  review found that the first version, "still running after 10 s", would
  pass an application stuck in that box.
- **The viewport is drawn on OpenGL in CI.** The Linux Debug job runs
  `ViewportGraphicsTest` and `OpenGLBackendTest` on Xvfb, where a skip is a
  failure. The backend tests had crashed at exit on every real platform:
  their `QGuiApplication` was a function-local static, destroyed after Qt's
  thread storage. A test environment now owns it.
- **The 2D tools are tested by what they draw**
  (`tests/ui/test_DraftingTools.cpp`, 24 tests). They cover Arc, Polyline,
  Spline, Hatch, Leader, Offset, Break, Extend, Polyline Edit, Mirror,
  Rotate, Scale, Move, Copy/Paste, Duplicate, Fillet, Chamfer and the
  linear, radial and angular dimensions. Each has exact geometry, undo and
  redo, and a locked or hidden layer left alone by every modifying tool.
  They found two bugs:
  - **Polyline Edit's Join kept the point where the two meet twice**: a
    segment of no length, which Offset turned into two corners.
  - **Tab never reached the dimension tools.** QWidget spent it on moving
    focus first, so their kind could not be cycled, and the view lost the
    keyboard. The viewport now offers Tab to the active tool first.
- **Each window test starts from cleared settings and default
  preferences.** The smoke test switches every drafting aid, and three
  tests after it failed when the binary was run as a whole.
- **Coverage floors** (`.github/coverage-floors.json`): one per module, set
  one point under the first run with the window tests. UI went from 1.3% to
  68.6%, and the total from 52% to 77%. The coverage job fails when a
  module drops under its floor.

### Not done, found on the way
- A new window opens in an isometric perspective close to the origin. A
  drawing is drafted in plan, and clicks far from the origin there land
  wide. A view kept per tab (plan for a drawing, isometric for a part)
  belongs with the workbench split (146). The tests look from the top, as
  View > Top does.
- Rotate and Scale make a turned or scaled copy and keep the original,
  where most CAD programs change the selection in place. This is left to
  the owner.

## Phase 146: Workbench controllers

Outline only, planned in detail when no open PR touches `MainWindow`:
- `AssemblyWorkbench` takes the Assembly menu's commands and their helpers.
- `PartWorkbench` takes the 3D part commands.
- `MainWindow` keeps tabs, menus, docks and file commands.
- No behaviour changes. The window tests, now in CI, are the net.

## Tracking

Each phase is its own PR on master, with README rows and CHANGELOG entries.
The phase's section here is replaced by "as built" when it lands.
