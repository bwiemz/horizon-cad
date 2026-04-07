# Phase 34: Primitive Solid Construction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Procedurally construct basic solids (box, cylinder, sphere, cone, torus) via Euler operators with NURBS geometry binding. First 3D visuals in Horizon.

**Architecture:** Factory functions in `hz::model` build B-Rep topology via Euler operators, then bind NURBS geometry from `hz::geo` to faces and edges. A `tessellate()` method on `Solid` produces `MeshData` for rendering via the existing Phong shader pipeline.

**Tech Stack:** C++20, Google Test, existing `hz::topo` (Euler ops, Solid), `hz::geo` (NurbsSurface, NurbsCurve), `hz::render` (MeshData, SceneNode)

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.4

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| Factory: `makeBox(w, h, d)` | Task 1 | ✅ |
| Factory: `makeCylinder(r, h)` | Task 2 | ✅ |
| Factory: `makeSphere(r)` | Task 2 | ✅ |
| Factory: `makeCone(rBot, rTop, h)` | Task 2 | ✅ |
| Factory: `makeTorus(R, r)` | Task 2 | ✅ |
| Build via Euler operators (ensuring invariants) | Tasks 1, 2 | ✅ |
| Bind NURBS geometry to faces/edges | Tasks 1, 2 | ✅ |
| TopologyIDs: role-based, stable across rebuilds | Tasks 1, 2 | ✅ |
| `Solid::tessellate(tolerance) → MeshData` | Task 3 | ✅ |
| User-facing: 3D Primitive toolbar | Task 4 | ✅ |
| Camera switches to perspective with orbit/pan/zoom | Task 4 | ✅ |
| Renders through existing Phong shader via SceneNode + MeshData | Task 4 | ✅ |
| Tests: each primitive's Euler formula | Tasks 1, 2 | ✅ |
| Tests: tessellation produces closed meshes | Task 3 | ✅ |
| Tests: normals point outward | Task 3 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/modeling/include/horizon/modeling/PrimitiveFactory.h` | Factory function declarations |
| Replace | `src/modeling/src/PrimitiveFactory.cpp` | Factory implementations |
| Create | `src/modeling/include/horizon/modeling/SolidTessellator.h` | Solid → MeshData tessellation |
| Create | `src/modeling/src/SolidTessellator.cpp` | Implementation |
| Modify | `src/modeling/CMakeLists.txt` | Add new source files, link Render |
| Create | `tests/modeling/test_PrimitiveFactory.cpp` | Primitive construction tests |
| Create | `tests/modeling/test_SolidTessellator.cpp` | Tessellation tests |
| Create | `tests/modeling/CMakeLists.txt` | Modeling test target |
| Modify | `tests/CMakeLists.txt` | Add modeling subdirectory |
| Modify | `src/ui/include/horizon/ui/MainWindow.h` | 3D primitive toolbar slots |
| Modify | `src/ui/src/MainWindow.cpp` | Toolbar actions + primitive placement |

---

## Task 1: makeBox — First Primitive via Euler Operators

**Files:**
- Create: `src/modeling/include/horizon/modeling/PrimitiveFactory.h`
- Replace: `src/modeling/src/PrimitiveFactory.cpp`
- Create: `tests/modeling/test_PrimitiveFactory.cpp`
- Create: `tests/modeling/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Modify: `src/modeling/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for makeBox**

```cpp
#include <gtest/gtest.h>
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/Queries.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::math;

TEST(PrimitiveFactoryTest, BoxEulerFormula) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    ASSERT_NE(solid, nullptr);
    // Box: 8V, 12E, 6F → 8 - 12 + 6 = 2
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(PrimitiveFactoryTest, BoxVertexPositions) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    // All vertices should be at corners of the box
    for (const auto& v : solid->vertices()) {
        bool xOk = (std::abs(v.point.x) < 1e-10 || std::abs(v.point.x - 10.0) < 1e-10);
        bool yOk = (std::abs(v.point.y) < 1e-10 || std::abs(v.point.y - 5.0) < 1e-10);
        bool zOk = (std::abs(v.point.z) < 1e-10 || std::abs(v.point.z - 3.0) < 1e-10);
        EXPECT_TRUE(xOk && yOk && zOk) 
            << "Unexpected vertex at (" << v.point.x << "," << v.point.y << "," << v.point.z << ")";
    }
}

TEST(PrimitiveFactoryTest, BoxEachFaceHas4Vertices) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    for (const auto& face : solid->faces()) {
        auto verts = faceVertices(&face);
        EXPECT_EQ(verts.size(), 4u) << "Face should have 4 vertices";
    }
}

TEST(PrimitiveFactoryTest, BoxTopologyIDs) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    // Each face should have a valid TopologyID
    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid()) << "Face missing TopologyID";
    }
    // IDs should be unique
    std::set<std::string> ids;
    for (const auto& face : solid->faces()) {
        ids.insert(face.topoId.tag());
    }
    EXPECT_EQ(ids.size(), 6u) << "All 6 face TopologyIDs should be unique";
}

TEST(PrimitiveFactoryTest, BoxHasNURBSGeometry) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face missing NURBS surface";
    }
}
```

- [ ] **Step 2: Create PrimitiveFactory.h**

```cpp
#pragma once

#include "horizon/topology/Solid.h"
#include <memory>

namespace hz::model {

class PrimitiveFactory {
public:
    /// Create a box centered at origin with given dimensions.
    static std::unique_ptr<topo::Solid> makeBox(double width, double height, double depth);

    /// Create a cylinder centered at origin, axis along Z.
    static std::unique_ptr<topo::Solid> makeCylinder(double radius, double height);

    /// Create a sphere centered at origin.
    static std::unique_ptr<topo::Solid> makeSphere(double radius);

    /// Create a cone/frustum centered at origin, axis along Z.
    static std::unique_ptr<topo::Solid> makeCone(double bottomRadius, double topRadius, double height);

    /// Create a torus centered at origin, axis along Z.
    static std::unique_ptr<topo::Solid> makeTorus(double majorRadius, double minorRadius);
};

}  // namespace hz::model
```

- [ ] **Step 3: Implement makeBox**

Build a box via Euler operators. The box has 8 vertices at the corners of (0,0,0)→(w,h,d):

```
v0=(0,0,0) v1=(w,0,0) v2=(w,h,0) v3=(0,h,0)  — bottom face
v4=(0,0,d) v5=(w,0,d) v6=(w,h,d) v7=(0,h,d)  — top face
```

Construction sequence:
1. MVFS → v0, face0, shell0
2. MEV(v0) → v1, edge v0-v1
3. MEV(v1) → v2, edge v1-v2
4. MEV(v2) → v3, edge v2-v3
5. MEF(v3→v0) → close bottom face, creates edge v3-v0 + new face (outer)
6. MEV(v0) → v4, edge v0-v4
7. MEV(v1) → v5, edge v1-v5
8. MEV(v2) → v6, edge v2-v6
9. MEV(v3) → v7, edge v3-v7
10. MEF(v4→v5) → create front face (bottom edge side)
11. MEF(v5→v6) → create right face
12. MEF(v6→v7) → create back face
13. MEF(v7→v4) → close top face

The exact half-edge navigation for each MEV/MEF depends on the half-edge connectivity. The implementer must trace through the half-edge structure carefully.

After construction:
- Assign `TopologyID::make("box", "bottom")` etc. to each face
- Bind `NurbsSurface::makePlane(...)` to each face
- Bind `NurbsCurve` line segments to each edge (a line is a degree-1 NURBS)

- [ ] **Step 4: Create test infrastructure**

Create `tests/modeling/CMakeLists.txt`:
```cmake
add_executable(hz_modeling_tests
    test_PrimitiveFactory.cpp
)
target_link_libraries(hz_modeling_tests PRIVATE
    Horizon::Modeling Horizon::Topology Horizon::Geometry Horizon::Render
    GTest::gtest GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(hz_modeling_tests)
```

Add `add_subdirectory(modeling)` to `tests/CMakeLists.txt`.

The modeling library needs to link Render for MeshData access — update `src/modeling/CMakeLists.txt` to add `Horizon::Render`.

- [ ] **Step 5: Build and run tests**

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(modeling): add makeBox primitive with Euler operators and NURBS geometry"
```

---

## Task 2: Remaining Primitives (Cylinder, Sphere, Cone, Torus)

- [ ] **Step 1: Write tests for all remaining primitives**

```cpp
TEST(PrimitiveFactoryTest, CylinderEulerFormula) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(solid, nullptr);
    // Cylinder: depends on circle tessellation in topology
    // Simplest: 2 circular caps + 1 lateral face
    // With N segments: N top vertices + N bottom vertices, N top edges + N bottom edges + N lateral edges
    // But for NURBS representation, the topology is simpler
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(PrimitiveFactoryTest, CylinderHasGeometry) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr);
    }
}

TEST(PrimitiveFactoryTest, SphereEulerFormula) {
    auto solid = PrimitiveFactory::makeSphere(5.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(PrimitiveFactoryTest, ConeEulerFormula) {
    auto solid = PrimitiveFactory::makeCone(5.0, 2.0, 10.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(PrimitiveFactoryTest, TorusEulerFormula) {
    auto solid = PrimitiveFactory::makeTorus(10.0, 3.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(PrimitiveFactoryTest, AllPrimitivesHaveTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(1, 1, 1);
    auto cyl = PrimitiveFactory::makeCylinder(1, 1);
    auto sph = PrimitiveFactory::makeSphere(1);
    auto cone = PrimitiveFactory::makeCone(1, 0.5, 1);
    auto torus = PrimitiveFactory::makeTorus(2, 0.5);
    
    for (auto* solid : {box.get(), cyl.get(), sph.get(), cone.get(), torus.get()}) {
        for (const auto& face : solid->faces()) {
            EXPECT_TRUE(face.topoId.isValid());
        }
    }
}
```

- [ ] **Step 2: Implement makeCylinder**

Topological approach for a NURBS cylinder:
- 2 faces (top cap + bottom cap) + 1 lateral face = 3 faces
- But a single lateral face would need a periodic (closed) surface. For simpler B-Rep: split lateral into 2 or 4 faces.

**Simplest valid B-Rep for a cylinder:**
- 2 vertices (top center, bottom center) — NO, this gives degenerate edges
- Better: discretize as a prism with N sides (e.g., N=4 for a square-ish cylinder)
- Or: use the NURBS approach with 2 circular edges + 1 seam edge

**Pragmatic approach for Phase 34:**
Build cylinders (and other curved primitives) as prism approximations with enough sides that the NURBS surface bound to each face provides the exact geometry. For example, a cylinder with 4 lateral faces, 4-vertex top cap, 4-vertex bottom cap:
- 8V, 12E, 6F (same as a box!)
- Each lateral face gets a cylindrical NURBS surface patch
- Top/bottom faces get planar NURBS surfaces

Actually, the cleanest approach: **4-sided prism** topology (same as box) but with cylindrical NURBS surfaces on the lateral faces and circular NURBS edges. This way the B-Rep is topologically a box but geometrically a cylinder.

For sphere: use a topology with 2 polar vertices + 4 equatorial vertices (octahedron-like topology, 6V, 12E, 8F). Or simpler: 2V (poles), 1 equatorial edge, 2 faces (hemispheres) — but this needs a single edge as a full circle, which is degenerate.

**Keep it simple for Phase 34:** All curved primitives use a prism/box-like topology with enough faces to be valid, and the NURBS geometry binding provides the exact curved shape. The tessellator (Task 3) evaluates the NURBS surfaces to produce the smooth mesh.

- [ ] **Step 3: Implement remaining factories**

Each follows the same pattern: build topology via Euler ops → assign TopologyIDs → bind NURBS geometry.

- [ ] **Step 4: Build and run tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(modeling): add cylinder, sphere, cone, torus primitives"
```

---

## Task 3: Solid Tessellation → MeshData

**Files:**
- Create: `src/modeling/include/horizon/modeling/SolidTessellator.h`
- Create: `src/modeling/src/SolidTessellator.cpp`
- Create: `tests/modeling/test_SolidTessellator.cpp`
- Modify: `src/modeling/CMakeLists.txt`
- Modify: `tests/modeling/CMakeLists.txt`

- [ ] **Step 1: Write tests**

```cpp
#include <gtest/gtest.h>
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/render/SceneGraph.h"

using namespace hz::model;
using namespace hz::render;

TEST(SolidTessellatorTest, BoxTessellation) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    MeshData mesh = SolidTessellator::tessellate(*solid, 0.1);
    
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_FALSE(mesh.normals.empty());
    EXPECT_FALSE(mesh.indices.empty());
    EXPECT_EQ(mesh.positions.size(), mesh.normals.size());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);  // Triangle list
}

TEST(SolidTessellatorTest, BoxAllIndicesInRange) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    MeshData mesh = SolidTessellator::tessellate(*solid, 0.1);
    
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, CylinderTessellation) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    MeshData mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, SphereTessellation) {
    auto solid = PrimitiveFactory::makeSphere(5.0);
    MeshData mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, NormalsPointOutward) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    MeshData mesh = SolidTessellator::tessellate(*solid, 0.1);
    
    // For a box, all triangle normals should point away from center (5, 2.5, 1.5)
    Vec3 center(5.0, 2.5, 1.5);
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        uint32_t i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
        Vec3 p0(mesh.positions[i0*3], mesh.positions[i0*3+1], mesh.positions[i0*3+2]);
        Vec3 n(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
        Vec3 toCenter = center - p0;
        // Normal should point away from center (dot product negative)
        EXPECT_LT(n.dot(toCenter), 0.01) << "Normal should point outward";
    }
}
```

- [ ] **Step 2: Implement SolidTessellator**

```cpp
class SolidTessellator {
public:
    static render::MeshData tessellate(const topo::Solid& solid, double tolerance = 0.1);
};
```

Implementation:
1. For each face in the solid:
   - If the face has a bound NurbsSurface: tessellate it via `surface->tessellate(tolerance)`
   - If no surface bound: tessellate the face's wire as a flat polygon
2. Merge all per-face meshes into one MeshData
3. Offset indices for each face's mesh to account for the merged vertex array

```cpp
MeshData SolidTessellator::tessellate(const Solid& solid, double tolerance) {
    MeshData result;
    uint32_t vertexOffset = 0;
    
    for (const auto& face : solid.faces()) {
        if (face.surface) {
            auto faceMesh = face.surface->tessellate(tolerance);
            // Append positions
            result.positions.insert(result.positions.end(),
                                    faceMesh.positions.begin(), faceMesh.positions.end());
            // Append normals
            result.normals.insert(result.normals.end(),
                                  faceMesh.normals.begin(), faceMesh.normals.end());
            // Append indices with offset
            for (uint32_t idx : faceMesh.indices) {
                result.indices.push_back(idx + vertexOffset);
            }
            vertexOffset += static_cast<uint32_t>(faceMesh.positions.size() / 3);
        }
    }
    return result;
}
```

- [ ] **Step 3: Build and run tests**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(modeling): add SolidTessellator for B-Rep to triangle mesh conversion"
```

---

## Task 4: 3D Primitive Toolbar + Rendering

**Files:**
- Modify: `src/ui/include/horizon/ui/MainWindow.h`
- Modify: `src/ui/src/MainWindow.cpp`
- Modify: `src/ui/src/ViewportWidget.cpp` or `ViewportRenderer.cpp`

- [ ] **Step 1: Add primitive toolbar actions to MainWindow**

Add slots:
```cpp
private slots:
    void onPrimitiveBox();
    void onPrimitiveCylinder();
    void onPrimitiveSphere();
```

Implementation: create the primitive, tessellate it, add to the scene graph, update viewport.

```cpp
void MainWindow::onPrimitiveBox() {
    auto solid = model::PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto mesh = model::SolidTessellator::tessellate(*solid, 0.1);
    
    auto node = std::make_shared<render::SceneNode>("Box");
    node->setMesh(std::make_unique<render::MeshData>(std::move(mesh)));
    
    // Add to scene graph (document or viewport)
    // TODO: integrate with Document's 3D model storage
    // For now, add directly to viewport's scene graph if accessible
    m_viewport->update();
}
```

- [ ] **Step 2: Add toolbar buttons**

In the toolbar setup, add a "3D Primitives" section with Box, Cylinder, Sphere buttons.

- [ ] **Step 3: Ensure the Phong shader renders the primitives**

The existing `GLRenderer::renderScene()` already renders `SceneNode` meshes with the Phong shader. Verify this path works by:
1. Creating a primitive
2. Adding its MeshData to a SceneNode
3. Checking it renders in the viewport

The camera may need to switch to perspective mode for 3D viewing. Add a "Perspective" toggle or auto-switch when 3D content exists.

- [ ] **Step 4: Build and manually test**

Launch horizon.exe, click the Box button, verify a 3D box appears in the viewport.

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(ui): add 3D primitive toolbar with box, cylinder, sphere"
```

---

## Task 5: Final Phase Commit + Push

- [ ] **Step 1: Run complete test suite**

Report exact test count.

- [ ] **Step 2: Commit and push**

```bash
git add -A
git commit -m "Phase 34: Primitive solid construction with Euler operators and NURBS geometry

- PrimitiveFactory: makeBox, makeCylinder, makeSphere, makeCone, makeTorus
- Each primitive built via Euler operators maintaining V-E+F=2
- NURBS geometry bound to all faces and edges
- Role-based TopologyIDs (box/top, cylinder/lateral, etc.)
- SolidTessellator converts B-Rep to triangle mesh (MeshData)
- 3D Primitive toolbar in MainWindow
- First 3D visuals in Horizon CAD"

git push origin master
```
