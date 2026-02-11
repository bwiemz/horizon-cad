# Horizon CAD

An open-source 2D drafting application built from scratch in C++20. Horizon provides a familiar CAD workflow with drawing tools, dimension annotations, layers, undo/redo, and native file I/O — all rendered with OpenGL and wrapped in a Qt6 desktop interface.

## Features

### Drawing Tools
- **Line**, **Circle** (center-radius), **Arc** (3-click), **Rectangle** (2-corner), **Polyline** (multi-click, open or closed)
- Snap-to-geometry engine with endpoint, midpoint, center, and intersection snapping

### Editing Tools
- Select, Move, Duplicate, Offset, Trim, Fillet, Mirror, Rotate, Scale
- Copy/Paste with clipboard support (Ctrl+C/X/V)
- Rectangular and Polar array operations

### Dimensions & Annotations
- **Linear** dimensions (horizontal, vertical, aligned) with auto-orientation detection
- **Radial** dimensions (radius or diameter) placed on circles and arcs
- **Angular** dimensions measuring the angle between two lines
- **Leader** annotations with custom text
- Text override support on all dimension types

### Layers & Properties
- Layer management with visibility, lock, color, and line width
- ByLayer property inheritance — entities can inherit color and line width from their layer
- Property panel for inspecting and editing selected entities
- Layer panel with add, remove, rename, and per-layer controls

### Document System
- Full undo/redo with 18 command types
- Native JSON file format (`.hcad`) with backward-compatible versioning
- New, Open, Save, Save As workflow

### Rendering
- OpenGL 3.3 Core Profile with batched line rendering
- Pan, orbit, and zoom camera controls
- Grid overlay with fit-all view
- Real-time snap indicators and tool previews
- QPainter text overlay for dimension annotations
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

# Run tests
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
  fileio/        Native format (.hcad) save/load
  ui/            Qt widgets, interactive tools, property & layer panels
  app/           Application entry point
  geometry/      Parametric curves/surfaces (planned)
  topology/      B-Rep topology (planned)
  constraint/    Geometric constraint solver (planned)
  modeling/      3D solid modeling (planned)
tests/
  math/          85 unit tests covering vectors, matrices, transforms, bounding boxes
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
| 8+ | Planned | Box selection, constraints, hatching, blocks, DXF import/export, 3D |

## Contributing

Contributions are welcome. Please open an issue to discuss changes before submitting a pull request.

## License

MIT
