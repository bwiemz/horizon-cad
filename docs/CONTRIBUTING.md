# Contributing to Horizon CAD

## Architecture Overview

Horizon CAD is structured as a layered set of CMake libraries. Each module lives
under `src/<module>/` and exports a namespaced C++ library.

```
math  -->  geometry  -->  drafting  -->  constraint  -->  document
                                                            |
                                                          fileio
                                                            |
                                                          render  -->  ui  -->  app
```

### Module / Namespace Map

| Module     | Namespace    | Purpose                                          |
|------------|-------------|--------------------------------------------------|
| math       | `hz::math`  | Vec2/3/4, Mat4, BoundingBox, Transform, RTree, Expression |
| geometry   | `hz::geom`  | Geometric utilities (intersection, projection)    |
| drafting   | `hz::draft` | Entities (Line, Circle, Arc, ...), Layer, SnapEngine, SpatialIndex, SketchPlane |
| constraint | `hz::cstr`  | Geometric constraints and solver (GCS)            |
| document   | `hz::doc`   | Document, UndoStack, Commands, ParameterRegistry  |
| fileio     | `hz::io`    | NativeFormat (.hcad JSON), DxfFormat (.dxf)       |
| render     | `hz::render`| GLRenderer, Camera, Grid, ShaderProgram, SelectionManager |
| ui         | `hz::ui`    | MainWindow, ViewportWidget, Tools, PropertyPanel   |
| app        | --          | Entry point (main.cpp)                            |

---

## Build Instructions

### Windows (MSVC 2022 + vcpkg)

From a Developer Command Prompt (or with `VCPKG_ROOT` set), using the CMake
that ships with Visual Studio:

```bash
cmake --preset debug
cmake --build build/debug --config Debug
ctest --test-dir build/debug -C Debug --output-on-failure
build/debug/src/app/Debug/horizon.exe
```

### Linux (GCC + vcpkg + Ninja)

```bash
cmake --preset linux-debug
cmake --build build/linux-debug
ctest --test-dir build/linux-debug --output-on-failure
./build/linux-debug/src/app/horizon
```

`linux-system-qt` builds against an installed Qt 6 instead of building Qt
with vcpkg, which is much faster the first time.

Python scripting is off by default because it is not sandboxed. To build and
test it, install Python 3 and pybind11 and configure with
`-DHZ_ENABLE_SCRIPTING=ON` (CI does, in the build and AddressSanitizer
jobs).

---

## How to Add an Entity Type

1. **Create header + source** in `src/drafting/`:
   - Inherit from `DraftEntity`
   - Implement all virtual methods:
     - `boundingBox()` -- axis-aligned bounding box
     - `hitTest(point, tolerance)` -- point proximity test
     - `snapPoints()` -- vector of snap-able positions
     - `translate(delta)` -- move by offset
     - `clone()` -- deep copy with new ID
     - `mirror(axisP1, axisP2)` -- reflect across line
     - `rotate(center, angle)` -- rotate around point (radians)
     - `scale(center, factor)` -- uniform scale around point

2. **Register in CMakeLists** (`src/drafting/CMakeLists.txt`).

3. **Add serialization** in `src/fileio/src/NativeFormat.cpp`:
   - Write: add a case to `serializeEntity()`
   - Read: add a case to `deserializeEntity()`
   - Bump the format version constant if the schema changes.

4. **Optionally create a Tool** in `src/ui/` (see below).

5. **Write tests** in `tests/drafting/`.

---

## How to Add a Tool

Tools live in `src/ui/` and inherit from the `Tool` base class.

1. **Inherit from `Tool`** and implement the state machine. Each handler
   gets the Qt event and the cursor's world position, and returns whether it
   used the event:
   - `mousePressEvent(QMouseEvent*, const math::Vec2& worldPos)`
   - `mouseMoveEvent(QMouseEvent*, const math::Vec2& worldPos)`
   - `mouseReleaseEvent(QMouseEvent*, const math::Vec2& worldPos)`
   - `keyPressEvent(QKeyEvent*)` (optional)
   - `cancel()`, to reset the state

   **Snap and pick through the viewport**, never with a tolerance of your
   own:
   - `m_viewport->snap(worldPos)` snaps within a fixed distance on screen,
     and never to hidden or locked layers;
   - `m_viewport->pickTolerance()` is the pick distance in world units at
     the current zoom.

   A piece made from an entity, as Trim, Break or Extend make, takes its
   style with `piece->copyStyleFrom(*original)`: layer, colour, width, line
   type and group.

2. **Provide rubber-band previews**:
   - Override `getPreviewLines()` and/or `getPreviewCircles()` to return
     geometry that the viewport draws during tool interaction.

3. **Register in MainWindow**:
   - Add the tool to `registerTools()` in `MainWindow.cpp`.
   - Connect a toolbar button or menu action.

4. **Layer checks**: Before operating on any entity, verify:
   ```cpp
   const auto* lp = layerMgr.getLayer(entity->layer());
   if (!lp || !lp->visible || lp->locked) continue;
   ```

5. **Undo integration**: Use `UndoStack::push()` which calls `execute()`
   internally -- never call `execute()` before `push()`. For multi-entity
   operations, wrap commands in a `CompositeCommand`.

---

## How to Add a Constraint

Constraints live in `src/constraint/` under the `hz::cstr` namespace.

1. **Inherit from `Constraint`** and implement:
   - `evaluate(params, residuals, offset)` -- write the residuals into
     `residuals`, starting at row `offset`
   - `jacobian(params, jacobian, offset)` -- write the partial derivatives
     into `jacobian`, starting at row `offset`
   - `equationCount()` -- number of scalar equations
   - `clone()` -- deep copy

2. **Add to `ConstraintTool`** mode enum so users can apply it interactively.

3. **Add serialization** in `NativeFormat` for persistence.

4. **Write solver tests** in `tests/constraint/`.

---

## Coding Standards

### Formatting

The project uses `.clang-format` (Google-based style):
- 4-space indentation
- 100-column line limit
- `#pragma once` for header guards

CI checks formatting with **clang-format 15**. Newer versions lay out a few
things differently; the one met so far is a lambda passed before a call's
last argument. Put such a lambda in a named local first.

### Conventions

- **Entity IDs**: `uint64_t` everywhere.
- **ByLayer**: entity color `0x00000000` = inherit layer color; lineWidth `0.0`
  = inherit layer lineWidth; lineType `0` = inherit layer lineType.
- **ByBlock**: definition entity color `0x00000000` = inherit block ref's
  resolved color; lineWidth `0.0` = inherit.
- **Layer checks**: Always verify `visible && !locked` before modifying entities.
- **Undo**: `UndoStack::push()` calls `execute()` -- never double-execute.
  Use `CompositeCommand` to group multi-entity operations into one undo step.
- **Coordinate system**: `Camera::screenToRay()` expects Qt-style Y
  (0 = top). Do not flip Y before calling.

---

## Testing

Tests use [Google Test](https://github.com/google/googletest) and live in
`tests/<module>/`, one directory per library.

- **The window.** `tests/ui` holds the `hz_ui_window_tests`, which drive a
  real `MainWindow` under Qt's offscreen platform. `UiTestSupport.h` has:
  - `FormFiller`, `DialogResponder` and `FilePicker`, which answer dialogs;
  - `ToolDriver`, which clicks at world points through the viewport;
  - `ModalCloser`, which dismisses any dialog.

  `SmokeTest` runs every command in the window, so a new command is
  exercised there as soon as it has an action.
- **Files.** `tests/fileio` round-trips every format and feeds the readers
  malformed input; STEP fixtures written to the standard are in
  `tests/fileio/fixtures/step`. `-DHZ_BUILD_FUZZERS=ON` (Clang) builds
  libFuzzer targets for the readers (`tests/fuzz`).

### Running tests

```bash
# Build first, then:
ctest --test-dir build/debug -C Debug --output-on-failure
```

### Adding tests

1. Create `test_YourThing.cpp` in the appropriate `tests/<module>/` directory.
2. Add it to the corresponding `tests/<module>/CMakeLists.txt` source list.
3. Link against the module library and `GTest::gtest GTest::gtest_main`.
4. Use `gtest_discover_tests()` for automatic test registration.

---

## Before you open a pull request

Horizon CAD is licensed under the GNU GPL v3 or later, and a contribution is
licensed under the same terms.

CI runs these checks on every pull request, and a pull request should pass
all of them locally first:

| Gate | What it runs | Locally |
|---|---|---|
| Build (Windows, Linux Debug, Linux Release) | MSVC; GCC 11 with `-Werror` | `cmake --preset linux-debug -DHZ_WARNINGS_AS_ERRORS=ON` and build |
| AddressSanitizer | the whole suite under ASan and UBSan | `-DHZ_ENABLE_SANITIZERS=ON` |
| Format Check | clang-format 15 | `clang-format --dry-run --Werror` on the files you changed |
| Static Analysis | clang-tidy 15, `bugprone-*` and `performance-*` | `clang-tidy -p build/linux-debug --checks='-*,bugprone-*,performance-*' <file>` |

Things that pass a newer local toolchain and fail CI's older one:
- clang 15 cannot capture a structured binding in a lambda; copy it into a
  plain local first;
- clang-format 15 lays out a lambda passed before a call's last argument
  differently (see Formatting above);
- CI's GoogleTest is built as C++17 and cannot print `char8_t` strings, so
  an `EXPECT_EQ` on two `std::u8string`s fails to link there; compare them
  as `std::string`.

Say in the CHANGELOG's Unreleased section what the change means for a user,
and fill in the pull request template. See also the [code of
conduct](../CODE_OF_CONDUCT.md), the [security policy](../SECURITY.md) and
[how releases are made](RELEASING.md).
