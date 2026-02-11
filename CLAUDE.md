# Horizon CAD

## Project Overview
Open-source 2D CAD application. C++20, Qt6, OpenGL 3.3 Core, CMake + vcpkg.

## Build Commands
```bash
# MUST use VS2022's cmake (not pip's):
"C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"

# Configure
cmake --preset debug

# Build
cmake --build build/debug --config Debug

# Test (85 math tests)
ctest --test-dir build/debug -C Debug

# Executable
build/debug/src/app/Debug/horizon.exe
```

## Architecture
| Module | Namespace | Key Types |
|--------|-----------|-----------|
| math | `hz::math` | Vec2, Vec3, Vec4, Mat4, BoundingBox, Transform |
| drafting | `hz::draft` | DraftDocument, DraftEntity, Layer, SnapEngine, DimensionStyle |
| document | `hz::doc` | Document, UndoStack, Command subclasses |
| render | `hz::render` | GLRenderer, Camera, Grid, ShaderProgram, SelectionManager |
| ui | `hz::ui` | MainWindow, ViewportWidget, Tool subclasses, PropertyPanel, LayerPanel |
| fileio | `hz::io` | NativeFormat (v4 JSON) |

## Code Conventions
- Namespaces: `hz::math`, `hz::draft`, `hz::doc`, `hz::render`, `hz::ui`, `hz::io`
- Entity IDs: `uint64_t` everywhere (DraftEntity, SelectionManager, Commands)
- Formatting: `.clang-format` (Google-based, 4-space indent, 100 col)
- ByLayer convention: entity color `0x00000000` = inherit layer color; lineWidth `0.0` = inherit layer lineWidth
- All tools that touch entities MUST check layer visibility/lock before operating

## Undo System
- `UndoStack::push()` calls `execute()` immediately — never call execute before push
- Multi-entity operations use `CompositeCommand` for single-step undo
- `selectedIds()` returns `std::vector<uint64_t>` (NOT a set)

## Rendering
- Dimension text uses QPainter overlay in `renderDimensionText()` — after GL pass in `paintGL()`
- `Camera::screenToRay()` expects Qt-style Y coords (0=top) — do NOT flip Y before calling

## File Format
- NativeFormat v4 (.hcad JSON): entities, layers, dimension style
- Backward-compatible with v1-v3 files

## Testing
- Math tests in `tests/math/` using Google Test
- No UI tests yet — test manually by running the exe
