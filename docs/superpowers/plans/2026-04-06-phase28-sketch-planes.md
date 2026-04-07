# Phase 28: Sketch Planes & Local Coordinate Systems Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decouple 2D sketches from absolute world XY so they can live on arbitrary 3D planes — the prerequisite for all 3D modeling.

**Architecture:** A `SketchPlane` defines a local coordinate frame (origin + orthonormal basis). A `Sketch` owns a `SketchPlane` + entities + constraints. `ViewportWidget` gains an active sketch concept — when editing a sketch, mouse coordinates project onto the sketch plane and tools receive local 2D coordinates. Backward compatible: existing drawings use a default XY sketch.

**Tech Stack:** C++20, Qt6, Eigen, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.4 (Phase 28)

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| `SketchPlane`: origin(Vec3) + normal(Vec3) + X-axis(Vec3), Y derived via cross product | Task 1 | ✅ |
| `Sketch`: container owning SketchPlane + DraftEntity collection + ConstraintSystem | Task 2 | ✅ |
| All DraftEntity inside Sketch store coords in local 2D | Task 2 | ✅ |
| `Sketch::localToWorld(Vec2)→Vec3` and `worldToLocal(Vec3)→Vec2` via Mat4 | Task 1 | ✅ |
| Rendering: camera aligns to sketch plane normal ("Look At") when editing | Task 4 | ✅ |
| 2D tools work in local coords exactly as today when editing sketch | Task 3 | ✅ |
| When orbiting away, sketch renders as flat 2D in 3D space | Task 4 | ✅ |
| Backward compatibility: default sketch on world XY, existing files load into it | Task 5 | ✅ |
| `ViewportWidget::setActiveSketch(Sketch*)`: projects mouse to sketch plane | Task 3 | ✅ |
| Tools receive Vec2 local coords — no tool changes required | Task 3 | ✅ |
| Test: XY plane sketch local==world | Task 1 | ✅ |
| Test: angled plane localToWorld/worldToLocal round-trip | Task 1 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/drafting/include/horizon/drafting/SketchPlane.h` | Local coordinate frame definition |
| Create | `src/drafting/src/SketchPlane.cpp` | Implementation with Mat4 transforms |
| Create | `src/drafting/include/horizon/drafting/Sketch.h` | Sketch container (plane + entities + constraints) |
| Create | `src/drafting/src/Sketch.cpp` | Implementation |
| Modify | `src/drafting/CMakeLists.txt` | Add new source files |
| Create | `tests/drafting/test_SketchPlane.cpp` | SketchPlane unit tests |
| Create | `tests/drafting/test_Sketch.cpp` | Sketch integration tests |
| Modify | `tests/drafting/CMakeLists.txt` | Add new test files |
| Modify | `src/document/include/horizon/document/Document.h` | Add Sketch collection |
| Modify | `src/document/src/Document.cpp` | Manage sketches |
| Modify | `src/ui/include/horizon/ui/ViewportWidget.h` | Active sketch + plane projection |
| Modify | `src/ui/src/ViewportWidget.cpp` | Mouse→sketch plane projection, camera alignment |
| Modify | `src/fileio/src/NativeFormat.cpp` | Serialize sketches + sketch planes |

---

## Task 1: SketchPlane — Local Coordinate Frame

**Files:**
- Create: `src/drafting/include/horizon/drafting/SketchPlane.h`
- Create: `src/drafting/src/SketchPlane.cpp`
- Create: `tests/drafting/test_SketchPlane.cpp`
- Modify: `src/drafting/CMakeLists.txt`
- Modify: `tests/drafting/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for SketchPlane**

Create `tests/drafting/test_SketchPlane.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Tolerance.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SketchPlaneTest, DefaultIsXYPlane) {
    SketchPlane plane;
    EXPECT_TRUE(plane.origin().isApproxEqual(Vec3::Zero));
    EXPECT_TRUE(plane.normal().isApproxEqual(Vec3::UnitZ));
    EXPECT_TRUE(plane.xAxis().isApproxEqual(Vec3::UnitX));
    EXPECT_TRUE(plane.yAxis().isApproxEqual(Vec3::UnitY));
}

TEST(SketchPlaneTest, XYPlaneLocalEqualsWorld) {
    SketchPlane plane;  // Default XY
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 3.0, 1e-10);
    EXPECT_NEAR(world.z, 0.0, 1e-10);
}

TEST(SketchPlaneTest, WorldToLocalOnXYPlane) {
    SketchPlane plane;
    Vec3 world(5.0, 3.0, 0.0);
    Vec2 local = plane.worldToLocal(world);
    EXPECT_NEAR(local.x, 5.0, 1e-10);
    EXPECT_NEAR(local.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, RoundTripOnXYPlane) {
    SketchPlane plane;
    Vec2 original(7.5, -2.3);
    Vec3 world = plane.localToWorld(original);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, original.x, 1e-10);
    EXPECT_NEAR(back.y, original.y, 1e-10);
}

TEST(SketchPlaneTest, OffsetXYPlane) {
    // XY plane elevated at Z=10
    SketchPlane plane(Vec3(0, 0, 10), Vec3::UnitZ, Vec3::UnitX);
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 3.0, 1e-10);
    EXPECT_NEAR(world.z, 10.0, 1e-10);

    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, 5.0, 1e-10);
    EXPECT_NEAR(back.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, XZPlane) {
    // Sketch on XZ plane (normal = Y, X-axis = X, Y-axis = Z)
    SketchPlane plane(Vec3::Zero, Vec3::UnitY, Vec3::UnitX);
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 0.0, 1e-10);
    EXPECT_NEAR(world.z, 3.0, 1e-10);
}

TEST(SketchPlaneTest, AngledPlaneRoundTrip) {
    // 45-degree tilted plane
    Vec3 normal = Vec3(0, -1, 1).normalized();  // Tilted 45° from XY
    Vec3 xAxis = Vec3::UnitX;
    SketchPlane plane(Vec3(10, 20, 30), normal, xAxis);

    Vec2 local(4.0, 7.0);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-9);
    EXPECT_NEAR(back.y, local.y, 1e-9);
}

TEST(SketchPlaneTest, ArbitraryPlaneRoundTrip) {
    // Fully arbitrary plane
    Vec3 normal = Vec3(1, 2, 3).normalized();
    // X-axis must be perpendicular to normal — use Gram-Schmidt
    Vec3 xAxis = Vec3(1, 0, 0);
    xAxis = (xAxis - normal * xAxis.dot(normal)).normalized();
    SketchPlane plane(Vec3(-5, 10, 15), normal, xAxis);

    Vec2 local(-3.5, 12.7);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-9);
    EXPECT_NEAR(back.y, local.y, 1e-9);
}

TEST(SketchPlaneTest, ProjectWorldPointOntoPlane) {
    SketchPlane plane;  // XY at Z=0
    // Point above the plane
    Vec3 point(5.0, 3.0, 10.0);
    Vec2 projected = plane.worldToLocal(point);
    EXPECT_NEAR(projected.x, 5.0, 1e-10);
    EXPECT_NEAR(projected.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, TransformMatricesAreInverses) {
    Vec3 normal = Vec3(1, 1, 1).normalized();
    Vec3 xAxis = Vec3(1, -1, 0).normalized();
    SketchPlane plane(Vec3(5, 10, 15), normal, xAxis);

    Mat4 toWorld = plane.localToWorldMatrix();
    Mat4 toLocal = plane.worldToLocalMatrix();

    // toWorld * toLocal should be identity
    Mat4 product = toWorld * toLocal;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            EXPECT_NEAR(product.at(r, c), expected, 1e-9)
                << "at (" << r << "," << c << ")";
        }
    }
}
```

- [ ] **Step 2: Implement SketchPlane.h**

Create `src/drafting/include/horizon/drafting/SketchPlane.h`:
```cpp
#pragma once

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::draft {

/// Defines a local 2D coordinate frame embedded in 3D space.
/// Origin + orthonormal basis (xAxis, yAxis, normal).
class SketchPlane {
public:
    /// Default: XY plane at origin (normal=Z, xAxis=X, yAxis=Y).
    SketchPlane();

    /// Custom plane. yAxis is derived as normal × xAxis.
    /// xAxis must be perpendicular to normal (it will be orthogonalized if not exactly).
    SketchPlane(const math::Vec3& origin, const math::Vec3& normal,
                const math::Vec3& xAxis);

    // Accessors
    const math::Vec3& origin() const { return m_origin; }
    const math::Vec3& normal() const { return m_normal; }
    const math::Vec3& xAxis() const { return m_xAxis; }
    const math::Vec3& yAxis() const { return m_yAxis; }

    // Coordinate transforms
    [[nodiscard]] math::Vec3 localToWorld(const math::Vec2& local) const;
    [[nodiscard]] math::Vec2 worldToLocal(const math::Vec3& world) const;

    // Transform matrices
    [[nodiscard]] math::Mat4 localToWorldMatrix() const;
    [[nodiscard]] math::Mat4 worldToLocalMatrix() const;

    /// Project a 3D ray onto this plane. Returns the 2D local coordinates
    /// of the intersection point (for mouse picking).
    /// Returns false if the ray is parallel to the plane.
    [[nodiscard]] bool rayIntersect(const math::Vec3& rayOrigin,
                                     const math::Vec3& rayDir,
                                     math::Vec2& localResult) const;

private:
    math::Vec3 m_origin;
    math::Vec3 m_normal;
    math::Vec3 m_xAxis;
    math::Vec3 m_yAxis;
};

}  // namespace hz::draft
```

- [ ] **Step 3: Implement SketchPlane.cpp**

Create `src/drafting/src/SketchPlane.cpp`:

**Default constructor:** origin=(0,0,0), normal=(0,0,1), xAxis=(1,0,0), yAxis=(0,1,0).

**Custom constructor:**
1. Normalize the normal vector
2. Orthogonalize xAxis: `xAxis = (xAxis - normal * xAxis.dot(normal)).normalized()`
3. Derive yAxis: `yAxis = normal.cross(xAxis)`

**localToWorld:** `origin + xAxis * local.x + yAxis * local.y`

**worldToLocal:** Project `(world - origin)` onto xAxis and yAxis:
```cpp
Vec3 delta = world - m_origin;
return Vec2(delta.dot(m_xAxis), delta.dot(m_yAxis));
```

**localToWorldMatrix:** Build a 4x4 matrix with xAxis, yAxis, normal as column vectors and origin as translation.

**worldToLocalMatrix:** Inverse of localToWorldMatrix. For an orthonormal basis, the inverse is the transpose of the rotation part + negated-translated origin.

**rayIntersect:**
```cpp
double denom = rayDir.dot(m_normal);
if (std::abs(denom) < 1e-12) return false;  // Parallel
double t = (m_origin - rayOrigin).dot(m_normal) / denom;
Vec3 hit = rayOrigin + rayDir * t;
localResult = worldToLocal(hit);
return true;
```

- [ ] **Step 4: Update CMakeLists**

Add `src/SketchPlane.cpp` to `src/drafting/CMakeLists.txt`.
Add `test_SketchPlane.cpp` to `tests/drafting/CMakeLists.txt`.

- [ ] **Step 5: Build and run all tests**

Expected: All existing tests + 10 new SketchPlane tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/drafting/include/horizon/drafting/SketchPlane.h \
        src/drafting/src/SketchPlane.cpp \
        src/drafting/CMakeLists.txt \
        tests/drafting/test_SketchPlane.cpp \
        tests/drafting/CMakeLists.txt
git commit -m "feat(drafting): add SketchPlane for local 2D coordinate frames in 3D

Origin + normal + xAxis orthonormal basis. localToWorld/worldToLocal transforms.
Ray-plane intersection for mouse picking. Default is XY plane at origin."
```

---

## Task 2: Sketch Container — Entities + Constraints + Plane

**Files:**
- Create: `src/drafting/include/horizon/drafting/Sketch.h`
- Create: `src/drafting/src/Sketch.cpp`
- Create: `tests/drafting/test_Sketch.cpp`
- Modify: `src/drafting/CMakeLists.txt`
- Modify: `tests/drafting/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for Sketch**

Create `tests/drafting/test_Sketch.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/drafting/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/math/Vec3.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SketchTest, DefaultSketchIsOnXYPlane) {
    Sketch sketch;
    EXPECT_TRUE(sketch.plane().normal().isApproxEqual(Vec3::UnitZ));
    EXPECT_TRUE(sketch.plane().origin().isApproxEqual(Vec3::Zero));
}

TEST(SketchTest, AddAndRetrieveEntity) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    sketch.addEntity(line);
    EXPECT_EQ(sketch.entities().size(), 1u);
    EXPECT_EQ(sketch.entities()[0]->id(), line->id());
}

TEST(SketchTest, RemoveEntity) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    sketch.addEntity(line);
    sketch.removeEntity(line->id());
    EXPECT_TRUE(sketch.entities().empty());
}

TEST(SketchTest, SketchOwnsConstraintSystem) {
    Sketch sketch;
    EXPECT_TRUE(sketch.constraintSystem().empty());
}

TEST(SketchTest, ClearRemovesEverything) {
    Sketch sketch;
    sketch.addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5)));
    sketch.addEntity(std::make_shared<DraftCircle>(Vec2(10, 10), 3.0));
    sketch.clear();
    EXPECT_TRUE(sketch.entities().empty());
    EXPECT_TRUE(sketch.constraintSystem().empty());
}

TEST(SketchTest, CustomPlane) {
    SketchPlane plane(Vec3(0, 0, 10), Vec3::UnitZ, Vec3::UnitX);
    Sketch sketch(plane);
    EXPECT_NEAR(sketch.plane().origin().z, 10.0, 1e-10);
}

TEST(SketchTest, EntitiesStoreLocalCoords) {
    // Entities inside a sketch store LOCAL 2D coordinates.
    // A line from (0,0) to (10,0) in local space on an elevated plane
    // should have those exact coordinates — NOT transformed to world.
    SketchPlane plane(Vec3(0, 0, 50), Vec3::UnitZ, Vec3::UnitX);
    Sketch sketch(plane);
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
    sketch.addEntity(line);

    // Local coords unchanged.
    EXPECT_NEAR(line->start().x, 0.0, 1e-10);
    EXPECT_NEAR(line->end().x, 10.0, 1e-10);

    // World position computed via plane transform.
    Vec3 worldStart = sketch.plane().localToWorld(line->start());
    EXPECT_NEAR(worldStart.z, 50.0, 1e-10);
}

TEST(SketchTest, SpatialIndexWorksWithinSketch) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    sketch.addEntity(line);

    BoundingBox query(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = sketch.spatialIndex().query(query);
    EXPECT_EQ(results.size(), 1u);
}

TEST(SketchTest, NameAndId) {
    Sketch sketch;
    sketch.setName("Front Face Sketch");
    EXPECT_EQ(sketch.name(), "Front Face Sketch");
    EXPECT_NE(sketch.id(), 0u);  // Auto-generated non-zero ID
}
```

- [ ] **Step 2: Implement Sketch.h**

Create `src/drafting/include/horizon/drafting/Sketch.h`:
```cpp
#pragma once

#include "horizon/drafting/SketchPlane.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/constraint/ConstraintSystem.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hz::draft {

/// A constrained 2D sketch living on a 3D plane.
/// Entities store coordinates in local 2D space relative to the sketch plane.
class Sketch {
public:
    /// Create a sketch on the default XY plane.
    Sketch();

    /// Create a sketch on a custom plane.
    explicit Sketch(const SketchPlane& plane);

    // Identity
    uint64_t id() const { return m_id; }
    void setId(uint64_t id) { m_id = id; }
    const std::string& name() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    // Plane
    const SketchPlane& plane() const { return m_plane; }
    void setPlane(const SketchPlane& plane) { m_plane = plane; }

    // Entities (local 2D coordinates)
    void addEntity(std::shared_ptr<DraftEntity> entity);
    void removeEntity(uint64_t entityId);
    const std::vector<std::shared_ptr<DraftEntity>>& entities() const { return m_entities; }
    std::vector<std::shared_ptr<DraftEntity>>& entities() { return m_entities; }

    // Constraints (owned by this sketch)
    cstr::ConstraintSystem& constraintSystem() { return m_constraints; }
    const cstr::ConstraintSystem& constraintSystem() const { return m_constraints; }

    // Spatial index (local coords)
    const SpatialIndex& spatialIndex() const { return m_spatialIndex; }
    SpatialIndex& spatialIndex() { return m_spatialIndex; }
    void rebuildSpatialIndex();

    void clear();

private:
    uint64_t m_id;
    std::string m_name;
    SketchPlane m_plane;
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
    cstr::ConstraintSystem m_constraints;
    SpatialIndex m_spatialIndex;

    static uint64_t s_nextId;
};

}  // namespace hz::draft
```

- [ ] **Step 3: Implement Sketch.cpp**

Create `src/drafting/src/Sketch.cpp`:
```cpp
#include "horizon/drafting/Sketch.h"
#include <algorithm>

namespace hz::draft {

uint64_t Sketch::s_nextId = 1;

Sketch::Sketch() : m_id(s_nextId++) {}

Sketch::Sketch(const SketchPlane& plane) : m_id(s_nextId++), m_plane(plane) {}

void Sketch::addEntity(std::shared_ptr<DraftEntity> entity) {
    if (!entity) return;
    m_entities.push_back(entity);
    m_spatialIndex.insert(entity);
}

void Sketch::removeEntity(uint64_t entityId) {
    auto it = std::find_if(m_entities.begin(), m_entities.end(),
                           [entityId](const auto& e) { return e->id() == entityId; });
    if (it != m_entities.end()) {
        m_spatialIndex.remove(entityId);
        m_entities.erase(it);
    }
}

void Sketch::rebuildSpatialIndex() {
    m_spatialIndex.rebuild(m_entities);
}

void Sketch::clear() {
    m_entities.clear();
    m_constraints.clear();
    m_spatialIndex.clear();
}

}  // namespace hz::draft
```

- [ ] **Step 4: Update CMakeLists**

Add `src/Sketch.cpp` to `src/drafting/CMakeLists.txt`.
Add `test_Sketch.cpp` to `tests/drafting/CMakeLists.txt`.
Note: Sketch.h includes ConstraintSystem.h — the drafting library must link to `Horizon::Constraint`. Check if this dependency already exists.

- [ ] **Step 5: Build and run all tests**

Expected: All tests pass including new Sketch tests.

- [ ] **Step 6: Commit**

```bash
git add src/drafting/include/horizon/drafting/Sketch.h src/drafting/src/Sketch.cpp \
        src/drafting/CMakeLists.txt tests/drafting/test_Sketch.cpp tests/drafting/CMakeLists.txt
git commit -m "feat(drafting): add Sketch container with plane, entities, and constraints

Sketch owns a SketchPlane + entity collection + ConstraintSystem + SpatialIndex.
Entities store local 2D coordinates relative to the sketch plane."
```

---

## Task 3: ViewportWidget — Active Sketch + Mouse Projection

The critical integration: when a sketch is active, mouse coordinates project onto the sketch plane instead of the XY plane.

**Files:**
- Modify: `src/ui/include/horizon/ui/ViewportWidget.h`
- Modify: `src/ui/src/ViewportWidget.cpp`

- [ ] **Step 1: Add active sketch to ViewportWidget.h**

Add to ViewportWidget:
```cpp
#include "horizon/drafting/Sketch.h"

// Active sketch (nullptr = no sketch active, use world XY)
void setActiveSketch(draft::Sketch* sketch);
draft::Sketch* activeSketch() const { return m_activeSketch; }

private:
    draft::Sketch* m_activeSketch = nullptr;
```

- [ ] **Step 2: Modify worldPositionAtCursor to use active sketch plane**

The current implementation hardcodes Z=0 plane intersection:
```cpp
// Current (line 130-137):
if (std::abs(rayDir.z) < 1e-12) {
    return {rayOrigin.x, rayOrigin.y};
}
double t = -rayOrigin.z / rayDir.z;
Vec3 hit = rayOrigin + rayDir * t;
return {hit.x, hit.y};
```

Replace with:
```cpp
math::Vec2 ViewportWidget::worldPositionAtCursor(int screenX, int screenY) const {
    auto [rayOrigin, rayDir] = m_camera.screenToRay(
        static_cast<double>(screenX),
        static_cast<double>(screenY),
        width(), height());

    if (m_activeSketch) {
        // Project onto the active sketch plane.
        math::Vec2 local;
        if (m_activeSketch->plane().rayIntersect(rayOrigin, rayDir, local)) {
            return local;  // Return LOCAL 2D coordinates
        }
        // Fallback: project onto sketch plane's tangent
        return m_activeSketch->plane().worldToLocal(rayOrigin);
    }

    // Default: intersect with XY plane (Z = 0).
    if (std::abs(rayDir.z) < 1e-12) {
        return {rayOrigin.x, rayOrigin.y};
    }
    double t = -rayOrigin.z / rayDir.z;
    math::Vec3 hit = rayOrigin + rayDir * t;
    return {hit.x, hit.y};
}
```

This is the key change — when a sketch is active, tools receive **local 2D coordinates** relative to the sketch plane. No tool code needs to change.

- [ ] **Step 3: Implement setActiveSketch**

```cpp
void ViewportWidget::setActiveSketch(draft::Sketch* sketch) {
    m_activeSketch = sketch;
    if (sketch) {
        // Align camera to look at the sketch plane.
        const auto& plane = sketch->plane();
        math::Vec3 center = plane.origin();
        math::Vec3 eye = center + plane.normal() * 100.0;  // Back off along normal
        m_camera.lookAt(eye, center, plane.yAxis());
        m_camera.setOrthographic(m_camera.orthoWidth(), m_camera.orthoHeight(),
                                  m_camera.nearPlane(), m_camera.farPlane());
    }
    update();
}
```

Note: The camera needs `orthoWidth()`, `orthoHeight()`, `nearPlane()`, `farPlane()` accessors. Check if they exist — if not, add simple getters to Camera.h.

- [ ] **Step 4: Handle snapping in active sketch context**

When a sketch is active, the SnapEngine should query the sketch's own SpatialIndex, not the document's:

In the mouse event handlers, when passing entities to snap, use:
```cpp
auto& draftDoc = m_activeSketch 
    ? /* sketch's entities */ : document()->draftDocument();
auto& index = m_activeSketch 
    ? m_activeSketch->spatialIndex() : draftDoc.spatialIndex();
auto& entities = m_activeSketch 
    ? m_activeSketch->entities() : draftDoc.entities();
```

- [ ] **Step 5: Build and manually test**

1. Build the project
2. Launch horizon.exe
3. Verify existing 2D behavior is unchanged (no active sketch = XY plane)
4. No automated test for this — it's a UI integration verified manually

- [ ] **Step 6: Run all tests**

Expected: All tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/ui/include/horizon/ui/ViewportWidget.h src/ui/src/ViewportWidget.cpp
git commit -m "feat(ui): active sketch support in ViewportWidget

setActiveSketch() aligns camera to sketch plane and projects mouse
coordinates onto it. Tools receive local 2D coords transparently."
```

---

## Task 4: Sketch Rendering — Camera Alignment + 3D Overlay

When editing a sketch, the camera aligns to the sketch plane. When orbiting away, the sketch renders as a flat 2D drawing in 3D space.

**Files:**
- Modify: `src/ui/src/ViewportWidget.cpp`

- [ ] **Step 1: Render sketch entities in world space when not editing**

When a sketch is NOT the active sketch but exists in the document, its entities need to be rendered in 3D space by transforming their local 2D coordinates through the sketch plane's `localToWorld()` matrix.

In the entity rendering loop of `paintGL()`, add logic to check if an entity belongs to a sketch (non-active) and transform its coordinates for rendering:

```cpp
// For entities in non-active sketches, transform through sketch plane
if (sketchForEntity && sketchForEntity != m_activeSketch) {
    const auto& plane = sketchForEntity->plane();
    // Transform entity vertices to world space for rendering
    // ... apply plane.localToWorldMatrix() to entity coordinates
}
```

For the initial implementation, this can be deferred — the important part is that editing a sketch (active sketch) works correctly with the camera alignment from Task 3.

- [ ] **Step 2: Add "Enter Sketch" / "Exit Sketch" UI action**

Add to MainWindow:
- "Enter Sketch" action: sets the selected sketch as active via `setActiveSketch()`
- "Exit Sketch" action (Escape key or button): `setActiveSketch(nullptr)`, restores previous camera state

Save/restore camera state on enter/exit:
```cpp
void ViewportWidget::setActiveSketch(draft::Sketch* sketch) {
    if (sketch && !m_activeSketch) {
        // Save camera state before entering sketch.
        m_savedCameraState = saveCameraState();
    }
    m_activeSketch = sketch;
    if (sketch) {
        alignCameraToSketchPlane(sketch->plane());
    } else if (m_savedCameraState) {
        restoreCameraState(*m_savedCameraState);
        m_savedCameraState.reset();
    }
    update();
}
```

- [ ] **Step 3: Build and test**

1. Build the project
2. Manually test: create a document, enter a default sketch, verify camera aligns to XY
3. Exit sketch, verify camera restores

- [ ] **Step 4: Commit**

```bash
git add src/ui/src/ViewportWidget.cpp src/ui/include/horizon/ui/ViewportWidget.h \
        src/ui/include/horizon/ui/MainWindow.h src/ui/src/MainWindow.cpp
git commit -m "feat(ui): sketch enter/exit with camera alignment and state save/restore"
```

---

## Task 5: Document Integration + Backward Compatibility

Add sketch collection to Document. Existing files load all entities into a default XY sketch.

**Files:**
- Modify: `src/document/include/horizon/document/Document.h`
- Modify: `src/document/src/Document.cpp`
- Modify: `src/fileio/src/NativeFormat.cpp`

- [ ] **Step 1: Add sketch collection to Document.h**

```cpp
#include "horizon/drafting/Sketch.h"

// Sketch management
void addSketch(std::shared_ptr<draft::Sketch> sketch);
std::shared_ptr<draft::Sketch> removeSketch(uint64_t sketchId);
const std::vector<std::shared_ptr<draft::Sketch>>& sketches() const { return m_sketches; }

/// The default sketch (XY plane at origin). Always exists.
/// Entities from pre-sketch files are loaded here.
draft::Sketch& defaultSketch() { return *m_defaultSketch; }
const draft::Sketch& defaultSketch() const { return *m_defaultSketch; }

private:
    std::vector<std::shared_ptr<draft::Sketch>> m_sketches;
    std::shared_ptr<draft::Sketch> m_defaultSketch;
```

- [ ] **Step 2: Initialize default sketch in Document**

In `Document` constructor:
```cpp
m_defaultSketch = std::make_shared<draft::Sketch>();
m_defaultSketch->setName("Default Sketch");
m_sketches.push_back(m_defaultSketch);
```

In `Document::clear()`:
```cpp
m_sketches.clear();
m_defaultSketch = std::make_shared<draft::Sketch>();
m_defaultSketch->setName("Default Sketch");
m_sketches.push_back(m_defaultSketch);
```

- [ ] **Step 3: Serialize sketches in NativeFormat**

In save: add a `"sketches"` array to the JSON. Each sketch has `id`, `name`, `plane` (origin, normal, xAxis), and `entities` (same format as top-level entities).

In load: if `"sketches"` key is present (v15+), deserialize sketches with their planes. If absent (v14 and earlier), load all entities into the default XY sketch — backward compatible.

Bump format version to v15 only if there are non-default sketches. Otherwise keep v14.

- [ ] **Step 4: Build and run all tests**

Expected: All tests pass. Existing .hcad files load correctly into the default sketch.

- [ ] **Step 5: Commit**

```bash
git add src/document/include/horizon/document/Document.h src/document/src/Document.cpp \
        src/fileio/src/NativeFormat.cpp
git commit -m "feat(document): add sketch collection with default XY sketch

Document owns a collection of Sketches. Default sketch on XY plane
is always present. Existing v14 files load into default sketch.
.hcad v15 serializes sketch planes and per-sketch entities."
```

---

## Task 6: Final Regression Testing + Phase Commit

- [ ] **Step 1: Run the complete test suite**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Report exact test count.

- [ ] **Step 2: Manual smoke test**

1. Open horizon.exe
2. Draw entities — verify they work exactly as before (default XY sketch)
3. Save and reload — verify round-trip
4. Open an older .hcad file — verify backward compatibility

- [ ] **Step 3: Final commit**

```bash
git add -A
git commit -m "Phase 28: Sketch planes and local coordinate systems

- SketchPlane: origin + normal + xAxis orthonormal basis with localToWorld/worldToLocal
- Sketch: container owning plane + entities + constraints + spatial index
- ViewportWidget: active sketch projects mouse onto sketch plane
- Camera aligns to sketch plane on enter, restores on exit
- Document owns sketch collection with default XY sketch
- .hcad v15 serializes sketch planes (backward compatible with v14)
- All entities store local 2D coords relative to their sketch plane"
```
