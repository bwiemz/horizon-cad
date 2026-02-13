# Horizon CAD

An open-source 2D drafting application built from scratch in C++20. Horizon provides a familiar CAD workflow with drawing tools, dimension annotations, constraints, layers, blocks, undo/redo, and file I/O — all rendered with OpenGL and wrapped in a modern Qt6 desktop interface.

## Features

### Drawing Tools
- **Line**, **Circle** (center-radius), **Arc** (3-click), **Rectangle** (2-corner), **Polyline** (multi-click, open or closed)
- **Ellipse** (center + semi-axes), **Spline** (cubic B-spline with control points)
- **Text** (standalone text entities with height, rotation, alignment)
- **Hatch** (boundary-fill with ANSI line, cross, and custom patterns)
- Snap-to-geometry engine with endpoint, midpoint, center, and intersection snapping

### Editing Tools
- Select with **window/crossing box selection** (left-to-right = enclosed only, right-to-left = overlapping), click, and Shift multi-select
- Move, Duplicate, Offset, Trim, Fillet, Mirror, Rotate, Scale
- Copy/Paste with clipboard support (Ctrl+C/X/V)
- Rectangular and Polar array operations
- Grip editing for direct point manipulation

### Dimensions & Annotations
- **Linear** dimensions (horizontal, vertical, aligned) with auto-orientation detection
- **Radial** dimensions (radius or diameter) placed on circles and arcs
- **Angular** dimensions measuring the angle between two lines
- **Leader** annotations with custom text
- Text override support on all dimension types
- Configurable dimension style (text height, arrow size, precision)

### Measurement Tools
- **Distance** measurement between two points
- **Angle** measurement between two lines
- **Area** measurement of closed polygons

### Geometric Constraints
- Coincident, Horizontal, Vertical, Perpendicular, Parallel, Tangent
- Equal length, Fixed position, Distance, and Angle constraints
- Constraint solver with real-time visual indicators

### Blocks & Components
- Create reusable block definitions from selected entities
- Insert block references with position, rotation, and scale
- Explode block references back to individual entities

### Layers & Properties
- Layer management with visibility, lock, color, and line width
- ByLayer property inheritance — entities can inherit color and line width from their layer
- Property panel for inspecting and editing selected entities
- Layer panel with add, remove, rename, and per-layer controls

### Document System
- Full undo/redo with composite command support
- Native JSON file format (`.hcad` v9) with backward-compatible versioning
- DXF import/export (LINE, CIRCLE, ARC, LWPOLYLINE, TEXT, MTEXT, SPLINE, HATCH, INSERT)
- New, Open, Save, Save As workflow

### Modern UI
- **Dark theme** with Fusion style, custom palette, and QSS stylesheet
- **Ribbon toolbar** with tabbed categories (Home, Draw, Modify, Annotate, Constrain, Measure, Blocks, View)
- **Programmatic icons** — 45+ vector-style icons generated via QPainter (no external assets)
- **Keyboard shortcuts** for all tools (single-key access: L=Line, C=Circle, etc.)
- **Enhanced status bar** showing coordinates, active tool, snap mode, selection count, and tool prompts
- **Viewport overlays** — crosshair cursor, snap markers, axis indicator (all GL-rendered)
- **Styled panels** — consistent dark theme across property and layer panels

### Rendering
- OpenGL 3.3 Core Profile with batched line rendering
- Pan, orbit, and zoom camera controls
- Grid overlay with fit-all view
- Real-time snap indicators and tool previews
- GL texture-based text overlay for dimension and entity text
- Selection highlighting and color-aware rendering

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Language | C++20 |
| GUI | Qt 6 (Widgets, OpenGL) |
| Graphics | OpenGL 3.3 Core Profile |
| Build | CMake 3.28+ with vcpkg |
| Serialization | nlohmann/json |
| Logging | spdlog |
| Testing | Google Test |

## Building

**Prerequisites:** A C++20 compiler (MSVC, GCC, or Clang), CMake 3.28+, vcpkg

```bash
# Configure
cmake --preset debug

# Build
cmake --build build/debug --config Debug

# Run tests (109 tests)
ctest --test-dir build/debug -C Debug

# Run
./build/debug/src/app/Debug/horizon.exe
```

## Architecture

Horizon is organized into modular libraries with clean dependency boundaries. Each module is a separate CMake target.

```
src/
  math/          Linear algebra (Vec2, Vec3, Mat4, BoundingBox, Quaternion, Transform)
  drafting/      Entity model, layers, snap engine, dimension styles
  document/      Document ownership, undo/redo command stack
  render/        OpenGL renderer, camera, grid, shaders, selection manager
  constraint/    Geometric constraint solver
  fileio/        Native format (.hcad) and DXF import/export
  ui/            Qt widgets, ribbon toolbar, tools, property & layer panels
  app/           Application entry point, dark theme, resources
  geometry/      Parametric curves/surfaces (planned)
  topology/      B-Rep topology (planned)
  modeling/      3D solid modeling (planned)
tests/
  math/          85 unit tests covering vectors, matrices, transforms, bounding boxes
  constraint/    24 unit tests for the geometric constraint solver
```

## Roadmap

Horizon is under active development. Completed and planned work:

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | Done | Math library, OpenGL renderer, camera, grid, Qt shell |
| 2 | Done | Document model, undo/redo, selection, snapping, file I/O |
| 3 | Done | Arc, rectangle, polyline entities and tools |
| 4 | Done | Duplicate, offset, trim, fillet, mirror |
| 5 | Done | Rotate, scale, copy/paste, arrays |
| 6 | Done | Layers, properties, color-aware rendering |
| 7 | Done | Dimensions and annotations |
| 8 | Done | Constraint solver and geometric constraints |
| 9 | Done | Blocks and components |
| 10 | Done | Text entities |
| 11 | Done | Spline entities (cubic B-spline) |
| 12 | Done | Hatch patterns |
| 13 | Done | DXF import/export |
| 14 | Done | Ellipse entities |
| 15 | Done | Grip editing |
| 16 | Done | Measurement tools |
| 17 | Done | UI modernization (dark theme, ribbon toolbar, icons, shortcuts) |
| 18 | Done | Box/window selection (drag-rectangle with window and crossing modes) |
| 19+ | Planned | Parametric geometry, 3D modeling |

## Contributing

Contributions are welcome. Please open an issue to discuss changes before submitting a pull request.

## License

MIT
