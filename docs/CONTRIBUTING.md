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

```bash
# Use the VS2022-bundled cmake (NOT pip's cmake):
CMAKE="C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"

# Configure
"$CMAKE" --preset debug

# Build
"$CMAKE" --build build/debug --config Debug

# Run tests
ctest --test-dir build/debug -C Debug --output-on-failure

# Run application
build/debug/src/app/Debug/horizon.exe
```

### Linux (GCC + vcpkg + Ninja)

```bash
cmake --preset linux-debug
cmake --build build/linux-debug
ctest --test-dir build/linux-debug --output-on-failure
./build/linux-debug/src/app/horizon
```

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

1. **Inherit from `Tool`** and implement the state machine:
   - `mousePressEvent(pos, button)` -- handle click
   - `mouseMoveEvent(pos)` -- handle cursor movement
   - `mouseReleaseEvent(pos, button)` -- handle release (optional)
   - `keyPressEvent(key)` -- handle keyboard input (optional)
   - `cancel()` -- reset state

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
   - `evaluate(table)` -- return residual vector
   - `jacobian(table)` -- return partial derivatives
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

Tests use [Google Test](https://github.com/google/googletest) and live in `tests/`.

```
tests/
  math/          -- Vec2, Vec3, Mat4, BoundingBox, RTree, Expression
  drafting/      -- SpatialIndex, SketchPlane, perf benchmarks
  constraint/    -- Solver, individual constraint types
  document/      -- Undo, ParameterRegistry
  fileio/        -- NativeFormat round-trip, DXF import/export
```

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
