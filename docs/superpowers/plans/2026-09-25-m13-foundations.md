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

### Plan
1. **Qt as the product needs it** (`vcpkg.json`):
   - add `freetype`, `harfbuzz` and `png` everywhere;
   - add `fontconfig`, `xcb` and `xcb-xlib` on Linux.
   CI already installs the system libraries these need. The first run
   rebuilds Qt in every job.
2. **The window tests run in CI**, in every job that builds them. Fix what
   they find, since they have only ever run here.
3. **The viewport is drawn in CI.** In the Linux Debug job, the OpenGL tests
   (`ViewportGraphicsTest`, `OpenGLBackendTest`) run under Xvfb on xcb, with
   Mesa's software renderer.
4. **The application starts.** A smoke step starts the built `horizon` under
   Xvfb, in CI and in the release workflow before packaging, and fails if it
   exits within 10 s.
5. **2D tools tested by what they draw**, through `ToolDriver`: each tool
   the smoke test alone runs gets a test that checks the geometry it makes,
   and that undo takes it away.
6. **Coverage that cannot quietly fall.** A floor per module, set from the
   first run that includes the window tests, fails the coverage job when a
   module drops below it.

### Tests
- CI's test count rises by the window tests, and the job summary says how
  many ran.
- The GL tests pass under Xvfb and skip nowhere in that step.
- The smoke step fails on a build without a platform plugin. This is shown
  by the run before the `vcpkg.json` change, where it must fail.

## Phase 146: Workbench controllers

Outline only, planned in detail when no open PR touches `MainWindow`:
- `AssemblyWorkbench` takes the Assembly menu's commands and their helpers.
- `PartWorkbench` takes the 3D part commands.
- `MainWindow` keeps tabs, menus, docks and file commands.
- No behaviour changes. The window tests, now in CI, are the net.

## Tracking

Each phase is its own PR on master, with README rows and CHANGELOG entries.
The phase's section here is replaced by "as built" when it lands.
