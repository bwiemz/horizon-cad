# Phase 29: Linux CI/CD & Cross-Platform Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish cross-platform CI/CD before the codebase grows with 3D code.

**Architecture:** GitHub Actions workflow with matrix build (Ubuntu 22.04 + Windows), CMake presets for Linux, clang-format enforcement in CI.

**Tech Stack:** GitHub Actions, CMake presets, vcpkg, clang-format, clang-tidy

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.5

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| GitHub Actions CI: Ubuntu 22.04 (GCC 12) + Windows (MSVC 2022) | Task 1 | ✅ |
| CMake presets: `linux-debug`, `linux-release` | Task 2 | ✅ |
| Fix platform-specific assumptions | Task 3 | ✅ |
| Package as AppImage for Linux | Task 4 | ✅ |
| `clang-tidy` + `clang-format` enforcement in CI | Task 1 | ✅ |
| All existing tests pass on both platforms | Task 1 | ✅ |

---

## Task 1: GitHub Actions CI Workflow

**Files:**
- Create: `.github/workflows/ci.yml`

- [ ] **Step 1: Create CI workflow**

Create `.github/workflows/ci.yml`:
```yaml
name: CI

on:
  push:
    branches: [master]
  pull_request:
    branches: [master]

env:
  VCPKG_BINARY_SOURCES: "clear;x-gha,readwrite"

jobs:
  build:
    strategy:
      fail-fast: false
      matrix:
        include:
          - os: windows-latest
            preset: debug
            triplet: x64-windows
          - os: ubuntu-22.04
            preset: linux-debug
            triplet: x64-linux

    runs-on: ${{ matrix.os }}

    steps:
      - uses: actions/checkout@v4

      - name: Export GitHub Actions cache variables
        uses: actions/github-script@v7
        with:
          script: |
            core.exportVariable('ACTIONS_CACHE_URL', process.env.ACTIONS_CACHE_URL || '');
            core.exportVariable('ACTIONS_RUNTIME_TOKEN', process.env.ACTIONS_RUNTIME_TOKEN || '');

      - name: Setup vcpkg
        uses: lukka/run-vcpkg@v11
        with:
          vcpkgGitCommitId: '01f602195983451bc83e72f4214af2c4c3b2b3a9'

      - name: Install Linux dependencies
        if: runner.os == 'Linux'
        run: |
          sudo apt-get update
          sudo apt-get install -y libgl1-mesa-dev libglu1-mesa-dev \
            libxkbcommon-dev libxkbcommon-x11-0 libxcb-xinerama0 \
            libxcb-cursor0 libxcb-keysyms1-dev libxcb-image0-dev \
            libxcb-render-util0-dev libxcb-icccm4-dev \
            libfontconfig1-dev libfreetype6-dev \
            libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev \
            libxrender-dev libxcb1-dev libxcb-glx0-dev \
            libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev \
            libxcb-xfixes0-dev libxcb-dri3-dev libxcb-dri2-0-dev \
            libxcb-present-dev libxshmfence-dev \
            pkg-config ninja-build

      - name: Configure
        run: cmake --preset ${{ matrix.preset }}

      - name: Build
        run: cmake --build build/${{ matrix.preset }} --config Debug

      - name: Test
        run: ctest --test-dir build/${{ matrix.preset }} -C Debug --output-on-failure

  format-check:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4

      - name: Install clang-format
        run: sudo apt-get update && sudo apt-get install -y clang-format-15

      - name: Check formatting
        run: |
          find src tests -name '*.h' -o -name '*.cpp' | \
            xargs clang-format-15 --dry-run --Werror

  static-analysis:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4

      - name: Setup vcpkg
        uses: lukka/run-vcpkg@v11
        with:
          vcpkgGitCommitId: '01f602195983451bc83e72f4214af2c4c3b2b3a9'

      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y clang-tidy-15 libgl1-mesa-dev libglu1-mesa-dev \
            libxkbcommon-dev libxkbcommon-x11-0 libxcb-xinerama0 \
            libxcb-cursor0 libxcb-keysyms1-dev libxcb-image0-dev \
            libxcb-render-util0-dev libxcb-icccm4-dev \
            libfontconfig1-dev libfreetype6-dev \
            libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev \
            libxrender-dev libxcb1-dev libxcb-glx0-dev \
            libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev \
            libxcb-xfixes0-dev libxcb-dri3-dev libxcb-dri2-0-dev \
            libxcb-present-dev libxshmfence-dev \
            pkg-config ninja-build

      - name: Configure with compile commands
        run: |
          cmake --preset linux-debug \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

      - name: Run clang-tidy
        run: |
          find src -name '*.cpp' | head -50 | \
            xargs clang-tidy-15 -p build/linux-debug \
            --checks='-*,bugprone-*,performance-*,modernize-use-override' \
            --warnings-as-errors='bugprone-*'
```

- [ ] **Step 2: Create .github directory and commit**

```bash
mkdir -p .github/workflows
git add .github/workflows/ci.yml
git commit -m "ci: add GitHub Actions workflow for Windows + Linux builds

Matrix build: Windows (MSVC 2022) + Ubuntu 22.04 (GCC).
Includes format check with clang-format-15."
```

---

## Task 2: Linux CMake Presets

**Files:**
- Modify: `CMakePresets.json`

- [ ] **Step 1: Add Linux presets**

Add `linux-debug` and `linux-release` presets that use Ninja and Unix Makefiles instead of Visual Studio generator:

```json
{
    "name": "linux-default",
    "binaryDir": "${sourceDir}/build/${presetName}",
    "cacheVariables": {
        "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    },
    "generator": "Ninja",
    "condition": {
        "type": "equals",
        "lhs": "${hostSystemName}",
        "rhs": "Linux"
    }
},
{
    "name": "linux-debug",
    "inherits": "linux-default",
    "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "HZ_BUILD_TESTS": "ON"
    }
},
{
    "name": "linux-release",
    "inherits": "linux-default",
    "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "HZ_BUILD_TESTS": "OFF"
    }
}
```

Also add build and test presets for linux-debug.

- [ ] **Step 2: Commit**

```bash
git add CMakePresets.json
git commit -m "build: add linux-debug and linux-release CMake presets

Uses Ninja generator on Linux. Conditional on host platform."
```

---

## Task 3: Fix Platform-Specific Assumptions

**Files:**
- Scan: all `.cpp` and `.h` files for Windows-specific code

- [ ] **Step 1: Search for platform-specific code**

Search for:
- Backslash file paths (`\\`)
- Windows-specific headers (`<windows.h>`, `<direct.h>`)
- Platform-specific APIs
- `#ifdef _WIN32` or `#ifdef _MSC_VER` without proper cross-platform handling

- [ ] **Step 2: Fix any issues found**

Common fixes:
- Use `std::filesystem::path` or forward slashes for paths
- Guard Windows-specific includes with `#ifdef _WIN32`
- Use Qt's platform-abstracted APIs where possible

- [ ] **Step 3: Commit fixes (if any)**

```bash
git add -A
git commit -m "fix: address platform-specific assumptions for Linux compatibility"
```

---

## Task 4: AppImage Packaging (Placeholder)

- [ ] **Step 1: Create Linux packaging script**

Create `installer/linux/build-appimage.sh`:
```bash
#!/usr/bin/env bash
set -euo pipefail

# Build AppImage for Horizon CAD
# Requires: linuxdeploy, linuxdeploy-plugin-qt

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build/linux-release"

# Build release
cmake --preset linux-release
cmake --build "$BUILD_DIR" --config Release

# Create AppDir structure
APPDIR="$BUILD_DIR/AppDir"
mkdir -p "$APPDIR/usr/bin"
mkdir -p "$APPDIR/usr/share/applications"
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"

cp "$BUILD_DIR/src/app/horizon" "$APPDIR/usr/bin/"

# Desktop file
cat > "$APPDIR/usr/share/applications/horizon-cad.desktop" << 'DESKTOP'
[Desktop Entry]
Type=Application
Name=Horizon CAD
Comment=Open-source parametric CAD software
Exec=horizon
Icon=horizon-cad
Categories=Graphics;Engineering;
DESKTOP

# TODO: Add icon file
# cp "$PROJECT_DIR/resources/icon-256.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/horizon-cad.png"

echo "AppDir prepared at $APPDIR"
echo "Run linuxdeploy to create AppImage:"
echo "  linuxdeploy --appdir $APPDIR --plugin qt --output appimage"
```

- [ ] **Step 2: Commit**

```bash
chmod +x installer/linux/build-appimage.sh
git add installer/linux/build-appimage.sh
git commit -m "build: add Linux AppImage packaging script (placeholder)

Prepares AppDir structure. Requires linuxdeploy for final packaging."
```

---

## Task 5: Final Phase Commit

- [ ] **Step 1: Commit any remaining files**

```bash
git add -A
git commit -m "Phase 29: Linux CI/CD and cross-platform hardening

- GitHub Actions CI: Ubuntu 22.04 + Windows matrix build
- CMake presets: linux-debug, linux-release (Ninja generator)
- clang-format enforcement in CI
- AppImage packaging script (placeholder)
- Platform-specific fixes for Linux compatibility"
```
