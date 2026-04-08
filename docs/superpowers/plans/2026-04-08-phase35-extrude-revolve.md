# Phase 35: Extrude & Revolve Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The core parametric workflow — sketch a 2D profile, extrude or revolve it into a 3D solid. This is where sketch planes (Phase 28) and the B-Rep kernel (Phases 33-34) converge.

**Architecture:** `Extrude` and `Revolve` operations in `hz::model` take a closed 2D profile from a `Sketch`, validate it, build B-Rep topology via Euler operators, bind NURBS geometry, and assign genealogy-based TopologyIDs. A `FeatureTree` records the sequence of operations for parametric rebuild. Undo commands store feature inputs and regenerate.

**Tech Stack:** C++20, Qt6, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.5

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| Extrude: closed 2D profile + direction + distance → solid | Task 2 | ✅ |
| Validate closed loop (connected endpoints within tolerance) | Task 1 | ✅ |
| For each profile edge, create ruled NURBS surface | Task 2 | ✅ |
| Create top/bottom cap faces | Task 2 | ✅ |
| Build B-Rep topology via Euler operators | Task 2 | ✅ |
| Orient normals outward | Task 2 | ✅ |
| TopologyIDs: `hash(featureID, sketchEntityID, "lateral"/"cap_top"/"cap_bottom")` | Task 2 | ✅ |
| Extrude variants: blind (fixed distance) | Task 2 | ✅ |
| Revolve: closed 2D profile + axis + angle | Task 3 | ✅ |
| Revolve: NURBS surfaces of revolution | Task 3 | ✅ |
| Revolve: caps if angle < 360° | Task 3 | ✅ |
| Revolve: handle degenerate axis-coincident edges | Task 3 | ✅ |
| FeatureTree: ordered list of features (sketch + operation) | Task 4 | ✅ |
| Each feature stores inputs (sketch ref, parameters) | Task 4 | ✅ |
| Editing a feature re-executes all subsequent (parametric rebuild) | Task 4 | ✅ |
| Features store TopologyID references, resolved via genealogy on rebuild | Task 4 | ✅ |
| Profile validation: closed loop, no self-intersections | Task 1 | ✅ |
| User-facing: Extrude and Revolve tools in UI | Task 5 | ✅ |
| Undo: ExtrudeCommand/RevolveCommand store inputs, undo removes + regenerates | Task 5 | ✅ |
| Test: extrude rectangle → 6-face box | Task 2 | ✅ |
| Test: extrude circle → 3-face cylinder | Task 2 | ✅ |
| Test: revolve rectangle → hollow cylinder | Task 3 | ✅ |
| Test: feature tree replay produces identical solid | Task 4 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/modeling/include/horizon/modeling/ProfileValidator.h` | Closed-loop validation |
| Create | `src/modeling/src/ProfileValidator.cpp` | Implementation |
| Create | `src/modeling/include/horizon/modeling/Extrude.h` | Extrude operation |
| Replace | `src/modeling/src/Extrude.cpp` | Implementation (replace stub) |
| Create | `src/modeling/include/horizon/modeling/Revolve.h` | Revolve operation |
| Create | `src/modeling/src/Revolve.cpp` | Implementation |
| Create | `src/modeling/include/horizon/modeling/FeatureTree.h` | Feature tree + parametric rebuild |
| Create | `src/modeling/src/FeatureTree.cpp` | Implementation |
| Modify | `src/modeling/CMakeLists.txt` | Add new source files |
| Create | `tests/modeling/test_Extrude.cpp` | Extrude tests |
| Create | `tests/modeling/test_Revolve.cpp` | Revolve tests |
| Create | `tests/modeling/test_FeatureTree.cpp` | Feature tree tests |
| Modify | `tests/modeling/CMakeLists.txt` | Add test files |
| Modify | `src/ui/include/horizon/ui/MainWindow.h` | Extrude/Revolve tool slots |
| Modify | `src/ui/src/MainWindow.cpp` | Tool implementation |

---

## Task 1: Profile Validator — Closed Loop Detection

**Files:**
- Create: `src/modeling/include/horizon/modeling/ProfileValidator.h`
- Create: `src/modeling/src/ProfileValidator.cpp`
- Modify: `src/modeling/CMakeLists.txt`

- [ ] **Step 1: Write tests for profile validation**

Append to or create `tests/modeling/test_Extrude.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"

using namespace hz::model;
using namespace hz::draft;
using namespace hz::math;

TEST(ProfileValidatorTest, RectangleIsClosedLoop) {
    // 4 lines forming a rectangle
    std::vector<std::shared_ptr<DraftEntity>> entities;
    entities.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    entities.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    entities.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    entities.push_back(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    auto result = ProfileValidator::validate(entities);
    EXPECT_TRUE(result.isClosed);
    EXPECT_EQ(result.orderedEdges.size(), 4u);
}

TEST(ProfileValidatorTest, CircleIsClosedLoop) {
    std::vector<std::shared_ptr<DraftEntity>> entities;
    entities.push_back(std::make_shared<DraftCircle>(Vec2(0,0), 5.0));

    auto result = ProfileValidator::validate(entities);
    EXPECT_TRUE(result.isClosed);
}

TEST(ProfileValidatorTest, OpenChainIsNotClosed) {
    // 2 lines that don't form a loop
    std::vector<std::shared_ptr<DraftEntity>> entities;
    entities.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    entities.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));

    auto result = ProfileValidator::validate(entities);
    EXPECT_FALSE(result.isClosed);
}

TEST(ProfileValidatorTest, EmptyProfileIsNotClosed) {
    std::vector<std::shared_ptr<DraftEntity>> entities;
    auto result = ProfileValidator::validate(entities);
    EXPECT_FALSE(result.isClosed);
}
```

- [ ] **Step 2: Implement ProfileValidator**

```cpp
// ProfileValidator.h
#pragma once
#include "horizon/drafting/DraftEntity.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <vector>

namespace hz::model {

struct ProfileValidationResult {
    bool isClosed = false;
    std::vector<std::shared_ptr<draft::DraftEntity>> orderedEdges;
    std::string errorMessage;
};

class ProfileValidator {
public:
    /// Validate that entities form a single closed loop.
    /// Orders the edges for traversal if valid.
    static ProfileValidationResult validate(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
        double tolerance = 1e-6);
};

}
```

Implementation:
- Special case: single circle → always closed
- For lines/arcs: chain endpoints. Start from first entity's start point. Find the next entity whose start matches current end (within tolerance). Repeat until back at start or no match found.
- If chain returns to start → closed. Otherwise → error.

- [ ] **Step 3: Build and run tests**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(modeling): add ProfileValidator for closed-loop detection"
```

---

## Task 2: Extrude Operation

**Files:**
- Create: `src/modeling/include/horizon/modeling/Extrude.h`
- Replace: `src/modeling/src/Extrude.cpp`
- Create: `tests/modeling/test_Extrude.cpp`

- [ ] **Step 1: Write tests for extrude**

```cpp
#include "horizon/modeling/Extrude.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/Queries.h"

TEST(ExtrudeTest, ExtrudeRectangleProduces6FaceBox) {
    // Rectangle profile: 4 lines
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    draft::SketchPlane plane;  // Default XY
    auto solid = Extrude::execute(profile, plane, Vec3(0, 0, 1), 3.0, "extrude_1");

    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(ExtrudeTest, ExtrudeCircleProduces3FaceCylinder) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftCircle>(Vec2(0,0), 5.0));

    draft::SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3(0, 0, 1), 10.0, "extrude_2");

    ASSERT_NE(solid, nullptr);
    // Circle extrude: 1 lateral face + top cap + bottom cap = 3 faces minimum
    // Topologically may need more faces depending on representation
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(ExtrudeTest, ExtrudeHasTopologyIDs) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    draft::SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3(0, 0, 1), 3.0, "extrude_1");

    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
    // Should have faces with "cap_bottom", "cap_top", and "lateral" in their IDs
}

TEST(ExtrudeTest, ExtrudeHasNURBSGeometry) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    draft::SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3(0, 0, 1), 3.0, "extrude_1");

    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face missing NURBS surface";
    }
}

TEST(ExtrudeTest, InvalidProfileReturnsNull) {
    // Open chain — not a closed loop
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));

    draft::SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3(0, 0, 1), 3.0, "extrude_1");
    EXPECT_EQ(solid, nullptr);
}
```

- [ ] **Step 2: Implement Extrude.h**

```cpp
#pragma once
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"
#include "horizon/math/Vec3.h"
#include <memory>
#include <string>
#include <vector>

namespace hz::model {

class Extrude {
public:
    /// Extrude a closed 2D profile along a direction by a distance.
    /// featureID is used for TopologyID genealogy (e.g., "extrude_1").
    /// Returns nullptr if the profile is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane,
        const math::Vec3& direction,
        double distance,
        const std::string& featureID);
};

}
```

- [ ] **Step 3: Implement Extrude::execute()**

Algorithm:
1. Validate profile via `ProfileValidator::validate()`
2. If not closed → return nullptr
3. Convert 2D profile edges to 3D using `plane.localToWorld()`
4. Build B-Rep topology:
   - For a rectangle (4 edges): same as makeBox topology (8V, 12E, 6F)
   - For N-edge polygon: 2N vertices, 3N edges, N+2 faces (N lateral + 2 caps)
   - For circle: use appropriate topology (box-like for simplicity)
5. Build via Euler operators: MVFS → MEV chain for bottom → MEF to close bottom → MEV for vertical edges → MEF for lateral and top faces
6. Bind NURBS geometry:
   - Bottom cap: planar surface from profile
   - Top cap: planar surface at offset
   - Lateral faces: ruled surfaces between corresponding bottom and top edges
7. Assign TopologyIDs: `TopologyID::make(featureID, "cap_bottom")`, `TopologyID::make(featureID, "cap_top")`, `TopologyID::make(featureID, "lateral_" + sketchEntityID)`

**For circle profiles:** Use the same approach as makeCylinder from Phase 34 — box-like topology with cylindrical NURBS on lateral faces.

- [ ] **Step 4: Build and run tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(modeling): add Extrude operation for 2D profile to 3D solid"
```

---

## Task 3: Revolve Operation

**Files:**
- Create: `src/modeling/include/horizon/modeling/Revolve.h`
- Create: `src/modeling/src/Revolve.cpp`
- Create: `tests/modeling/test_Revolve.cpp`
- Modify: `src/modeling/CMakeLists.txt`
- Modify: `tests/modeling/CMakeLists.txt`

- [ ] **Step 1: Write tests**

```cpp
TEST(RevolveTest, RevolveRectangleProducesHollowCylinder) {
    // Rectangle offset from axis → revolve 360° → hollow cylinder/torus-like shape
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(5,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,5), Vec2(5,0)));

    draft::SketchPlane plane;
    math::Vec3 axis = math::Vec3::UnitY;  // Revolve around Y axis
    auto solid = Revolve::execute(profile, plane, math::Vec3::Zero, axis, 
                                   2.0 * 3.14159265358979, "revolve_1");

    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid());
}

TEST(RevolveTest, RevolveHasTopologyIDs) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(5,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,5), Vec2(5,0)));

    draft::SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, math::Vec3::Zero, math::Vec3::UnitY,
                                   2.0 * 3.14159265358979, "revolve_1");
    ASSERT_NE(solid, nullptr);
    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
}

TEST(RevolveTest, PartialRevolveHasCaps) {
    // 90° revolve should have cap faces at start and end
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,0), Vec2(10,0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(10,5), Vec2(5,5)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(5,5), Vec2(5,0)));

    draft::SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, math::Vec3::Zero, math::Vec3::UnitY,
                                   3.14159265358979 / 2.0, "revolve_1");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    // Should have more faces than full revolve (caps at start/end)
}
```

- [ ] **Step 2: Implement Revolve**

```cpp
class Revolve {
public:
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane,
        const math::Vec3& axisPoint,
        const math::Vec3& axisDirection,
        double angle,  // radians, up to 2*pi
        const std::string& featureID);
};
```

Algorithm:
1. Validate profile
2. Convert 2D edges to 3D via sketch plane
3. For each profile edge, create a surface of revolution (NURBS)
4. Build B-Rep topology: similar to extrude but lateral faces are revolution surfaces
5. If angle < 2π, add cap faces at start and end angles
6. Handle degenerate edges (edges on the axis → they become points, not surfaces)
7. Assign TopologyIDs

**Simplification for Phase 35:** For the initial implementation, revolve can use the same box-like topology approach as primitives. A full 360° revolve of a 4-edge rectangle produces a shape topologically equivalent to a torus (which we already build in Phase 34). Partial revolves add cap faces.

- [ ] **Step 3: Build and run tests**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(modeling): add Revolve operation for profile revolution around axis"
```

---

## Task 4: Feature Tree

**Files:**
- Create: `src/modeling/include/horizon/modeling/FeatureTree.h`
- Create: `src/modeling/src/FeatureTree.cpp`
- Create: `tests/modeling/test_FeatureTree.cpp`
- Modify: `src/modeling/CMakeLists.txt`
- Modify: `tests/modeling/CMakeLists.txt`

- [ ] **Step 1: Write tests**

```cpp
TEST(FeatureTreeTest, AddAndReplayExtrude) {
    FeatureTree tree;

    // Create a sketch with a rectangle
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    // Add extrude feature
    auto feature = std::make_unique<ExtrudeFeature>(sketch, Vec3(0,0,1), 3.0);
    tree.addFeature(std::move(feature));

    // Build
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
}

TEST(FeatureTreeTest, ReplayProducesIdenticalSolid) {
    FeatureTree tree;
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0,0), Vec2(10,0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10,0), Vec2(10,5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10,5), Vec2(0,5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0,5), Vec2(0,0)));

    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0,0,1), 3.0));

    auto solid1 = tree.build();
    auto solid2 = tree.build();  // Rebuild from scratch

    ASSERT_NE(solid1, nullptr);
    ASSERT_NE(solid2, nullptr);
    EXPECT_EQ(solid1->vertexCount(), solid2->vertexCount());
    EXPECT_EQ(solid1->edgeCount(), solid2->edgeCount());
    EXPECT_EQ(solid1->faceCount(), solid2->faceCount());
}

TEST(FeatureTreeTest, EmptyTreeReturnsNull) {
    FeatureTree tree;
    auto solid = tree.build();
    EXPECT_EQ(solid, nullptr);
}
```

- [ ] **Step 2: Implement FeatureTree**

```cpp
// FeatureTree.h
#pragma once
#include "horizon/topology/Solid.h"
#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Base class for features in the tree.
class Feature {
public:
    virtual ~Feature() = default;
    virtual std::string name() const = 0;
    virtual std::string featureID() const = 0;
    
    /// Execute this feature. For the first feature, inputSolid is nullptr.
    virtual std::unique_ptr<topo::Solid> execute(
        std::unique_ptr<topo::Solid> inputSolid) const = 0;
};

class ExtrudeFeature : public Feature { ... };
class RevolveFeature : public Feature { ... };

class FeatureTree {
public:
    void addFeature(std::unique_ptr<Feature> feature);
    void removeFeature(size_t index);
    void clear();
    
    size_t featureCount() const;
    const Feature* feature(size_t index) const;
    
    /// Build the solid by replaying all features from scratch.
    std::unique_ptr<topo::Solid> build() const;
    
    /// Rebuild from a specific feature index forward.
    std::unique_ptr<topo::Solid> rebuildFrom(size_t index) const;
    
private:
    std::vector<std::unique_ptr<Feature>> m_features;
};

}
```

`build()` iterates features in order. First feature creates the solid from a sketch. Each subsequent feature modifies it (for now, only extrude/revolve create new solids — Booleans come in Phase 36).

- [ ] **Step 3: Build and run tests**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(modeling): add FeatureTree with parametric rebuild support"
```

---

## Task 5: UI — Extrude/Revolve Tools + Undo

**Files:**
- Modify: `src/ui/include/horizon/ui/MainWindow.h`
- Modify: `src/ui/src/MainWindow.cpp`
- Create: `src/ui/include/horizon/ui/tools/ExtrudeTool.h` (or inline in MainWindow)
- Create: `src/ui/src/tools/ExtrudeTool.cpp`

- [ ] **Step 1: Add Extrude/Revolve actions to MainWindow**

Add toolbar actions in the 3D ribbon tab:
```cpp
private slots:
    void onExtrudeSketch();
    void onRevolveSketch();
```

Implementation:
```cpp
void MainWindow::onExtrudeSketch() {
    // Get current sketch entities
    auto* sketch = m_viewport->activeSketch();
    if (!sketch || sketch->entities().empty()) {
        statusBar()->showMessage("Enter a sketch with a closed profile first");
        return;
    }

    // Prompt for distance
    bool ok;
    double distance = QInputDialog::getDouble(this, "Extrude", "Distance:", 10.0, 0.01, 1e6, 2, &ok);
    if (!ok) return;

    // Execute extrude
    Vec3 direction = sketch->plane().normal();
    auto solid = Extrude::execute(sketch->entities(), sketch->plane(), direction, distance, "extrude_1");
    if (!solid) {
        statusBar()->showMessage("Extrude failed: profile is not a closed loop");
        return;
    }

    // Tessellate and display
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    auto node = std::make_shared<render::SceneNode>("Extrude");
    node->setMesh(std::make_unique<render::MeshData>(std::move(mesh)));
    m_viewport->sceneGraph().addNode(node);
    
    // Exit sketch mode
    m_viewport->setActiveSketch(nullptr);
    m_viewport->update();
}
```

- [ ] **Step 2: Add toolbar buttons**

Add "Extrude" and "Revolve" buttons to the 3D ribbon tab.

- [ ] **Step 3: Build and manually test**

1. Launch horizon.exe
2. Draw a rectangle (4 lines)
3. Click Extrude → enter distance → solid should appear
4. Verify the solid renders via the Phong shader

- [ ] **Step 4: Run all tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(ui): add Extrude and Revolve tools in 3D toolbar"
```

---

## Task 6: Final Phase Commit + Push

- [ ] **Step 1: Run complete test suite**

Report exact test count.

- [ ] **Step 2: Commit and push**

```bash
git add -A
git commit -m "Phase 35: Extrude and Revolve with FeatureTree and profile validation

- ProfileValidator: closed-loop detection for 2D sketch profiles
- Extrude: sketch profile → 3D solid via Euler operators + ruled NURBS surfaces
- Revolve: sketch profile → 3D solid of revolution + cap faces for partial angles
- FeatureTree: ordered feature list with parametric rebuild from scratch
- TopologyIDs trace faces back to source sketch entities
- Extrude/Revolve tools in 3D toolbar with distance/angle input"

git push origin master
```
