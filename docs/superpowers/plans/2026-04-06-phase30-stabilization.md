# Phase 30: Stabilization & Refactor Pass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clean up and harden all Era 0 work before the 3D kernel push.

**Architecture:** Extract ViewportWidget into focused components, add test coverage for new modules, performance benchmark, and contributor documentation.

**Tech Stack:** C++20, Qt6, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.6

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| Refactor ViewportWidget: extract ViewportRenderer + ViewportInputHandler | Task 1 | ✅ |
| Unit test coverage for R*-tree, expression engine, sketch plane — >80% branch coverage | Task 2 | ✅ |
| Performance benchmark: 10k-entity stress test, verify no regressions | Task 3 | ✅ |
| Update .hcad format with expressions, design variables, sketch planes | N/A — already done | ✅ (v14 has all) |
| Write contributor docs: architecture overview, how to add entity/tool, coding standards | Task 4 | ✅ |

---

## Task 1: Refactor ViewportWidget

ViewportWidget is likely the largest file in the project. Extract rendering logic and input handling into focused classes.

**Files:**
- Create: `src/ui/include/horizon/ui/ViewportRenderer.h`
- Create: `src/ui/src/ViewportRenderer.cpp`
- Create: `src/ui/include/horizon/ui/ViewportInputHandler.h`
- Create: `src/ui/src/ViewportInputHandler.cpp`
- Modify: `src/ui/include/horizon/ui/ViewportWidget.h`
- Modify: `src/ui/src/ViewportWidget.cpp`
- Modify: `src/ui/CMakeLists.txt`

- [ ] **Step 1: Read ViewportWidget.cpp and measure its size**

Count lines, identify logical sections:
- Entity rendering loop (batch building, GL draw calls)
- Text overlay rendering (QPainter)
- Constraint annotation rendering
- DOF computation
- Mouse event routing (press/move/release/wheel)
- Coordinate conversion helpers
- Tool preview rendering

- [ ] **Step 2: Extract ViewportRenderer**

Move rendering-related code out of ViewportWidget into a new `ViewportRenderer` class:
- Entity batch building and GL draw calls
- Grid rendering
- Tool preview rendering (lines, circles, arcs)
- Selection highlight rendering
- Snap indicator rendering
- Constraint annotation rendering
- Text overlay rendering (dimensions, text entities)
- DOF visualization

`ViewportRenderer` takes references to Document, Camera, SelectionManager, and the GL functions pointer.

```cpp
class ViewportRenderer {
public:
    void renderScene(QOpenGLExtraFunctions* gl,
                     const doc::Document& doc,
                     const render::Camera& camera,
                     const render::SelectionManager& selection,
                     render::GLRenderer& renderer);
    
    void renderOverlay(QPainter& painter,
                       const doc::Document& doc,
                       const render::Camera& camera,
                       const render::SelectionManager& selection,
                       const QSize& viewportSize);

    void setDOFAnalysis(const cstr::DOFAnalysis& analysis);
    
private:
    cstr::DOFAnalysis m_dofAnalysis;
};
```

- [ ] **Step 3: Extract ViewportInputHandler**

Move mouse/keyboard event routing into `ViewportInputHandler`:
- Pan (middle-mouse drag)
- Zoom (mouse wheel)
- Orbit (right-mouse drag, for future 3D)
- Tool dispatch (left-click → active tool)
- Coordinate conversion delegation

```cpp
class ViewportInputHandler {
public:
    void handleMousePress(QMouseEvent* event, ViewportWidget* viewport);
    void handleMouseMove(QMouseEvent* event, ViewportWidget* viewport);
    void handleMouseRelease(QMouseEvent* event, ViewportWidget* viewport);
    void handleWheel(QWheelEvent* event, ViewportWidget* viewport);
    void handleKeyPress(QKeyEvent* event, ViewportWidget* viewport);

private:
    bool m_panning = false;
    QPoint m_lastMousePos;
};
```

- [ ] **Step 4: Simplify ViewportWidget**

After extraction, ViewportWidget becomes a thin coordinator:
```cpp
class ViewportWidget : public QOpenGLWidget {
    // Owns:
    Camera m_camera;
    SelectionManager m_selectionManager;
    SnapEngine m_snapEngine;
    ViewportRenderer m_viewportRenderer;    // NEW
    ViewportInputHandler m_inputHandler;     // NEW
    OverlayRenderer m_overlayRenderer;
    
    // Delegates:
    void paintGL() override {
        m_viewportRenderer.renderScene(...);
        // QPainter overlay
        m_viewportRenderer.renderOverlay(...);
    }
    
    void mousePressEvent(QMouseEvent* e) override {
        m_inputHandler.handleMousePress(e, this);
    }
};
```

- [ ] **Step 5: Build and run all tests**

Expected: All tests pass. No behavioral changes — pure refactor.

- [ ] **Step 6: Commit**

```bash
git add src/ui/include/horizon/ui/ViewportRenderer.h \
        src/ui/src/ViewportRenderer.cpp \
        src/ui/include/horizon/ui/ViewportInputHandler.h \
        src/ui/src/ViewportInputHandler.cpp \
        src/ui/include/horizon/ui/ViewportWidget.h \
        src/ui/src/ViewportWidget.cpp \
        src/ui/CMakeLists.txt
git commit -m "refactor(ui): extract ViewportRenderer and ViewportInputHandler from ViewportWidget

ViewportWidget is now a thin coordinator. Rendering logic lives in
ViewportRenderer. Mouse/keyboard event routing in ViewportInputHandler."
```

---

## Task 2: Test Coverage Expansion

Add tests targeting >80% branch coverage for new Era 0 modules.

**Files:**
- Create: `tests/math/test_ExpressionEdgeCases.cpp`
- Create: `tests/drafting/test_SketchPlaneEdgeCases.cpp`
- Modify: `tests/math/CMakeLists.txt`
- Modify: `tests/drafting/CMakeLists.txt`

- [ ] **Step 1: Add expression engine edge case tests**

```cpp
// test_ExpressionEdgeCases.cpp
TEST(ExpressionEdgeCaseTest, DivisionByZero)         // "1 / 0" → inf
TEST(ExpressionEdgeCaseTest, NestedFunctions)         // "sin(cos(0))" → sin(1)
TEST(ExpressionEdgeCaseTest, DeepNesting)             // "((((1+2)+3)+4)+5)" → 15
TEST(ExpressionEdgeCaseTest, WhitespaceHandling)      // "  2  +  3  " → 5
TEST(ExpressionEdgeCaseTest, ScientificNotation)      // "1e3" → 1000 (if supported)
TEST(ExpressionEdgeCaseTest, MultipleOperators)       // "2 + 3 - 1 * 4 / 2" with precedence
TEST(ExpressionEdgeCaseTest, PowerAssociativity)      // "2 ^ 3 ^ 2" → 512 (right-assoc)
TEST(ExpressionEdgeCaseTest, NegativeExponent)        // "2 ^ -1" → 0.5
TEST(ExpressionEdgeCaseTest, EmptyParens)             // "()" → nullptr
TEST(ExpressionEdgeCaseTest, MultiArgFunction)        // "atan2(0, -1)" → pi
```

- [ ] **Step 2: Add sketch plane edge case tests**

```cpp
// test_SketchPlaneEdgeCases.cpp
TEST(SketchPlaneEdgeCaseTest, RayParallelToPlane)      // rayIntersect returns false
TEST(SketchPlaneEdgeCaseTest, RayFromBehindPlane)      // ray from negative normal side
TEST(SketchPlaneEdgeCaseTest, DegenerateNormal)        // zero-length normal → safe handling
TEST(SketchPlaneEdgeCaseTest, CollinearXAxisAndNormal)  // xAxis parallel to normal → orthogonalize
TEST(SketchPlaneEdgeCaseTest, LargeCoordinates)        // 1e6 scale coords round-trip
TEST(SketchPlaneEdgeCaseTest, VerySmallCoordinates)    // 1e-8 scale coords round-trip
```

- [ ] **Step 3: Build and run all tests**

Report exact test count.

- [ ] **Step 4: Commit**

```bash
git add tests/math/test_ExpressionEdgeCases.cpp \
        tests/drafting/test_SketchPlaneEdgeCases.cpp \
        tests/math/CMakeLists.txt tests/drafting/CMakeLists.txt
git commit -m "test: add edge case tests for expression engine and sketch planes

Targets >80% branch coverage for Era 0 modules."
```

---

## Task 3: Performance Benchmark — 10k-Entity Stress Test

**Files:**
- Modify: `tests/drafting/test_SpatialIndexPerf.cpp` (add more comprehensive benchmarks)

- [ ] **Step 1: Verify existing performance benchmarks**

Run existing perf tests and check they still pass under 1ms/5ms targets.

- [ ] **Step 2: Add new benchmarks if needed**

```cpp
TEST(SpatialIndexPerfTest, TenThousandEntityInsertUnder100ms) {
    // Benchmark bulk insert time.
    SpatialIndex index;
    auto start = std::chrono::high_resolution_clock::now();
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        index.insert(line);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "[PERF] 10k entity insert: " << ms << " ms" << std::endl;
    EXPECT_LT(ms, 100.0);
}
```

- [ ] **Step 3: Run and verify**

Expected: All perf targets met.

- [ ] **Step 4: Commit**

```bash
git add tests/drafting/test_SpatialIndexPerf.cpp
git commit -m "test: add 10k-entity bulk insert performance benchmark"
```

---

## Task 4: Contributor Documentation

**Files:**
- Create: `docs/CONTRIBUTING.md`

- [ ] **Step 1: Write contributor documentation**

Create `docs/CONTRIBUTING.md` covering:

1. **Architecture Overview** — module diagram, dependency DAG, namespace conventions
2. **Build Instructions** — Windows (MSVC + vcpkg) and Linux (GCC + vcpkg)
3. **How to Add an Entity Type** — step-by-step with DraftEntity virtual methods, serialization, and tool registration
4. **How to Add a Tool** — Tool base class, state machine pattern, preview rendering, snap integration
5. **How to Add a Constraint** — Constraint base class, evaluate/jacobian, ConstraintTool integration
6. **Coding Standards** — .clang-format reference, naming conventions, ByLayer/ByBlock conventions
7. **Testing** — Google Test patterns, how to run, where to add tests

- [ ] **Step 2: Commit**

```bash
git add docs/CONTRIBUTING.md
git commit -m "docs: add contributor documentation

Architecture overview, build instructions, how to add entities/tools/constraints,
coding standards, testing guide."
```

---

## Task 5: Final Phase Commit

- [ ] **Step 1: Run complete test suite**

Report exact count.

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "Phase 30: Stabilization and refactor pass

- ViewportWidget refactored: extracted ViewportRenderer and ViewportInputHandler
- Edge case tests for expression engine and sketch planes
- Performance benchmarks verified for 10k entities
- Contributor documentation (architecture, adding entities/tools/constraints)
- Era 0 complete: foundation hardened for Era 1 geometry kernel"
```
