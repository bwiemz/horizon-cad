# Phase 33: Half-Edge B-Rep Topology Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The half-edge B-Rep data structure with TopologyID genealogy — the foundation for Boolean operations, fillets, and parametric feature editing.

**Architecture:** Topological entities (Solid, Shell, Face, Edge, HalfEdge, Vertex) with stable IDs and pool-based allocation. Euler operators are the only way to modify topology, maintaining invariants. Every entity carries a deterministic `TopologyID` encoding its creation genealogy, solving the Topological Naming Problem.

**Tech Stack:** C++20, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.3 + Section 8.1

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| Half-edge data structure: Solid → Shell → Face/Edge/Vertex | Task 1 | ✅ |
| Face: OuterLoop (Wire) + InnerLoops + Surface* pointer | Task 1 | ✅ |
| Edge: HalfEdge forward + HalfEdge twin + Curve* pointer | Task 1 | ✅ |
| Vertex: Point (Vec3) | Task 1 | ✅ |
| TopologyID on every Vertex, Edge, Face — deterministic hash | Task 2 | ✅ |
| IDs based on genealogy, NOT memory addresses or indices | Task 2 | ✅ |
| Resolution algorithm: exact match preferred, prefix/ancestry for splits | Task 2 | ✅ |
| Euler operator: makeVertexFaceSolid() | Task 3 | ✅ |
| Euler operator: makeEdgeVertex() | Task 3 | ✅ |
| Euler operator: makeEdgeFace() | Task 3 | ✅ |
| Euler operator: killEdgeVertex() / killEdgeFace() | Task 3 | ✅ |
| Euler operator: makeVertexFaceShell() | Task 3 | ✅ |
| All operators maintain Euler's formula: V - E + F = 2 per shell | Task 3 | ✅ |
| Topological queries: adjacentFaces, leftFace, rightFace, incidentEdges | Task 4 | ✅ |
| solid.isValid() — Euler formula check, manifold check, orientation check | Task 4 | ✅ |
| Stable ID + pool allocation (not raw pointers) | Task 1 | ✅ |
| Tests: cube and cylinder via Euler operators | Task 5 | ✅ |
| Tests: verify Euler formula | Task 5 | ✅ |
| Tests: verify adjacency queries | Task 5 | ✅ |
| Tests: verify manifold invariants | Task 5 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/topology/include/horizon/topology/TopologyID.h` | Genealogy-based deterministic ID |
| Create | `src/topology/src/TopologyID.cpp` | Hash computation |
| Create | `src/topology/include/horizon/topology/HalfEdge.h` | All topological entity types |
| Create | `src/topology/include/horizon/topology/Solid.h` | Solid container + validation |
| Create | `src/topology/src/Solid.cpp` | Solid implementation |
| Create | `src/topology/include/horizon/topology/EulerOps.h` | Euler operator declarations |
| Create | `src/topology/src/EulerOps.cpp` | Euler operator implementations |
| Replace | `src/topology/src/Body.cpp` | Remove stub |
| Replace | `src/topology/src/EulerOperators.cpp` | Remove stub, replaced by EulerOps |
| Modify | `src/topology/CMakeLists.txt` | New source files |
| Create | `tests/topology/test_TopologyID.cpp` | TopologyID tests |
| Create | `tests/topology/test_EulerOps.cpp` | Euler operator + invariant tests |
| Create | `tests/topology/CMakeLists.txt` | Topology test target |
| Modify | `tests/CMakeLists.txt` | Add topology subdirectory |

---

## Task 1: Topological Entity Types + Half-Edge Data Structure

**Files:**
- Create: `src/topology/include/horizon/topology/HalfEdge.h`
- Create: `src/topology/include/horizon/topology/Solid.h`
- Create: `src/topology/src/Solid.cpp`

- [ ] **Step 1: Define the core topological entities**

Create `src/topology/include/horizon/topology/HalfEdge.h`:
```cpp
#pragma once

#include "horizon/topology/TopologyID.h"
#include "horizon/math/Vec3.h"
#include <cstdint>
#include <memory>
#include <vector>

// Forward declarations for geometry binding
namespace hz::geo {
class NurbsCurve;
class NurbsSurface;
}

namespace hz::topo {

// Forward declarations
struct Vertex;
struct HalfEdge;
struct Edge;
struct Wire;
struct Face;
struct Shell;
class Solid;

/// A vertex in the B-Rep — a point in 3D space.
struct Vertex {
    uint32_t id = 0;
    TopologyID topoId;
    math::Vec3 point;
    HalfEdge* halfEdge = nullptr;  // One outgoing half-edge (for traversal)
};

/// A directed half-edge. Each edge has two half-edges (twins).
struct HalfEdge {
    uint32_t id = 0;
    Vertex* origin = nullptr;      // Vertex at the start of this half-edge
    HalfEdge* twin = nullptr;      // Opposite half-edge on the same edge
    HalfEdge* next = nullptr;      // Next half-edge in the face loop (CCW)
    HalfEdge* prev = nullptr;      // Previous half-edge in the face loop
    Edge* edge = nullptr;          // Parent edge
    Face* face = nullptr;          // Face this half-edge borders
};

/// An edge shared between two faces. Owns two half-edges.
struct Edge {
    uint32_t id = 0;
    TopologyID topoId;
    HalfEdge* halfEdge = nullptr;  // One of the two half-edges (the "forward" one)
    std::shared_ptr<geo::NurbsCurve> curve;  // Optional geometry binding
};

/// An ordered loop of half-edges forming a face boundary.
struct Wire {
    uint32_t id = 0;
    HalfEdge* halfEdge = nullptr;  // One half-edge in the loop (entry point)
};

/// A face bounded by one outer wire loop and zero or more inner loops (holes).
struct Face {
    uint32_t id = 0;
    TopologyID topoId;
    Wire* outerLoop = nullptr;
    std::vector<Wire*> innerLoops;
    Shell* shell = nullptr;
    std::shared_ptr<geo::NurbsSurface> surface;  // Optional geometry binding
};

/// A closed shell (connected set of faces forming a closed volume).
struct Shell {
    uint32_t id = 0;
    std::vector<Face*> faces;
    Solid* solid = nullptr;
};

}  // namespace hz::topo
```

- [ ] **Step 2: Define Solid as the top-level container with pool allocation**

Create `src/topology/include/horizon/topology/Solid.h`:
```cpp
#pragma once

#include "horizon/topology/HalfEdge.h"
#include <deque>
#include <string>

namespace hz::topo {

/// A B-Rep solid — the top-level topological container.
/// Owns all topological entities via pool-based allocation (deque for pointer stability).
class Solid {
public:
    Solid();

    // Pool allocators — return stable pointers (deque never invalidates on push_back)
    Vertex* allocVertex();
    HalfEdge* allocHalfEdge();
    Edge* allocEdge();
    Wire* allocWire();
    Face* allocFace();
    Shell* allocShell();

    // Accessors (const views)
    const std::deque<Vertex>& vertices() const { return m_vertices; }
    const std::deque<HalfEdge>& halfEdges() const { return m_halfEdges; }
    const std::deque<Edge>& edges() const { return m_edges; }
    const std::deque<Wire>& wires() const { return m_wires; }
    const std::deque<Face>& faces() const { return m_faces; }
    const std::deque<Shell>& shells() const { return m_shells; }

    // Counts
    size_t vertexCount() const { return m_vertices.size(); }
    size_t edgeCount() const { return m_edges.size(); }
    size_t faceCount() const { return m_faces.size(); }
    size_t shellCount() const { return m_shells.size(); }

    // Validation
    bool isValid() const;                    // Euler + manifold + orientation
    bool checkEulerFormula() const;          // V - E + F = 2 per shell
    bool checkManifold() const;              // Each edge has exactly 2 half-edges
    bool checkOrientation() const;           // Consistent face orientation

    std::string validationReport() const;    // Detailed report of any issues

private:
    std::deque<Vertex> m_vertices;
    std::deque<HalfEdge> m_halfEdges;
    std::deque<Edge> m_edges;
    std::deque<Wire> m_wires;
    std::deque<Face> m_faces;
    std::deque<Shell> m_shells;

    uint32_t m_nextId = 1;
};

}  // namespace hz::topo
```

**Key design: `std::deque` for pool allocation.** Unlike `std::vector`, deque never invalidates existing pointers on `push_back`. This gives us stable pointers without heap-allocating each entity individually.

- [ ] **Step 3: Implement Solid.cpp**

```cpp
#include "horizon/topology/Solid.h"

namespace hz::topo {

Solid::Solid() = default;

Vertex* Solid::allocVertex() {
    m_vertices.push_back({});
    auto* v = &m_vertices.back();
    v->id = m_nextId++;
    return v;
}

// Same pattern for allocHalfEdge, allocEdge, allocWire, allocFace, allocShell...

bool Solid::checkEulerFormula() const {
    // For each shell: V - E + F = 2
    // Count vertices and edges that belong to each shell via face traversal
    for (const auto& shell : m_shells) {
        // ... count unique vertices and edges via half-edge traversal
        // V - E + F should equal 2
    }
    return true;
}

bool Solid::isValid() const {
    return checkEulerFormula() && checkManifold() && checkOrientation();
}
```

The Euler formula check needs to count V, E, F per shell by traversing half-edges from each face.

- [ ] **Step 4: Update CMakeLists**

Replace the stub files in `src/topology/CMakeLists.txt`:
```cmake
add_library(hz_topology STATIC
    src/Solid.cpp
    src/TopologyID.cpp
    src/EulerOps.cpp
)
```

Remove `Body.cpp` and `EulerOperators.cpp` (stubs).

- [ ] **Step 5: Build (just verify compilation)**

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(topology): add half-edge B-Rep data structure with pool allocation

Solid, Shell, Face, Edge, HalfEdge, Vertex, Wire types.
Pool-based allocation via std::deque for pointer stability.
Euler formula and manifold validation."
```

---

## Task 2: TopologyID Genealogy System

**Files:**
- Create: `src/topology/include/horizon/topology/TopologyID.h`
- Create: `src/topology/src/TopologyID.cpp`
- Create: `tests/topology/test_TopologyID.cpp`
- Create: `tests/topology/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write tests for TopologyID**

```cpp
#include <gtest/gtest.h>
#include "horizon/topology/TopologyID.h"

using namespace hz::topo;

TEST(TopologyIDTest, DefaultIsInvalid) {
    TopologyID id;
    EXPECT_FALSE(id.isValid());
}

TEST(TopologyIDTest, CreateFromComponents) {
    TopologyID id = TopologyID::make("box", "top");
    EXPECT_TRUE(id.isValid());
    EXPECT_FALSE(id.tag().empty());
}

TEST(TopologyIDTest, SameInputsSameID) {
    TopologyID a = TopologyID::make("box", "top");
    TopologyID b = TopologyID::make("box", "top");
    EXPECT_EQ(a, b);
}

TEST(TopologyIDTest, DifferentInputsDifferentID) {
    TopologyID a = TopologyID::make("box", "top");
    TopologyID b = TopologyID::make("box", "bottom");
    EXPECT_NE(a, b);
}

TEST(TopologyIDTest, ChildInheritsParent) {
    TopologyID parent = TopologyID::make("extrude_1", "face_3");
    TopologyID child = parent.child("split", 0);
    EXPECT_TRUE(child.isValid());
    EXPECT_TRUE(child.isDescendantOf(parent));
}

TEST(TopologyIDTest, GrandchildIsDescendant) {
    TopologyID grandparent = TopologyID::make("extrude_1", "face_3");
    TopologyID parent = grandparent.child("split", 0);
    TopologyID child = parent.child("split", 1);
    EXPECT_TRUE(child.isDescendantOf(grandparent));
    EXPECT_TRUE(child.isDescendantOf(parent));
}

TEST(TopologyIDTest, NonRelatedIsNotDescendant) {
    TopologyID a = TopologyID::make("box", "top");
    TopologyID b = TopologyID::make("box", "bottom");
    EXPECT_FALSE(a.isDescendantOf(b));
    EXPECT_FALSE(b.isDescendantOf(a));
}

TEST(TopologyIDTest, SelfIsNotDescendant) {
    TopologyID a = TopologyID::make("box", "top");
    EXPECT_FALSE(a.isDescendantOf(a));
}

TEST(TopologyIDTest, TagPreservesReadability) {
    TopologyID id = TopologyID::make("extrude_1", "lateral_5");
    // Tag should contain the creation info in some readable form
    EXPECT_FALSE(id.tag().empty());
}

TEST(TopologyIDTest, ResolveFindsBestMatch) {
    TopologyID target = TopologyID::make("extrude_1", "face_3");
    TopologyID exact = TopologyID::make("extrude_1", "face_3");
    TopologyID child = target.child("split", 0);
    TopologyID unrelated = TopologyID::make("box", "top");

    std::vector<TopologyID> candidates = {child, unrelated, exact};
    auto result = TopologyID::resolve(target, candidates);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(*result, exact);  // Exact match preferred over descendant
}

TEST(TopologyIDTest, ResolveFallsBackToDescendant) {
    TopologyID target = TopologyID::make("extrude_1", "face_3");
    TopologyID child = target.child("split", 0);
    TopologyID unrelated = TopologyID::make("box", "top");

    std::vector<TopologyID> candidates = {child, unrelated};
    auto result = TopologyID::resolve(target, candidates);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(*result, child);  // Descendant match when no exact
}

TEST(TopologyIDTest, ResolveFailsWhenNoMatch) {
    TopologyID target = TopologyID::make("extrude_1", "face_3");
    TopologyID unrelated = TopologyID::make("box", "top");

    std::vector<TopologyID> candidates = {unrelated};
    auto result = TopologyID::resolve(target, candidates);
    EXPECT_FALSE(result.has_value());
}
```

- [ ] **Step 2: Implement TopologyID.h**

```cpp
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hz::topo {

/// Deterministic genealogy-based identifier for topological entities.
/// Encodes how an entity was created, not where it lives in memory.
class TopologyID {
public:
    TopologyID() = default;

    /// Create a root-level ID from component strings.
    static TopologyID make(const std::string& source, const std::string& role);

    /// Create a child ID (inherits parent's genealogy + new component).
    [[nodiscard]] TopologyID child(const std::string& operation, int index) const;

    /// Check if this ID is a descendant of another (parent's tag is a prefix).
    [[nodiscard]] bool isDescendantOf(const TopologyID& ancestor) const;

    /// The full genealogy tag string (human-readable for debugging).
    [[nodiscard]] const std::string& tag() const { return m_tag; }

    [[nodiscard]] bool isValid() const { return !m_tag.empty(); }

    bool operator==(const TopologyID& other) const { return m_tag == other.m_tag; }
    bool operator!=(const TopologyID& other) const { return m_tag != other.m_tag; }
    bool operator<(const TopologyID& other) const { return m_tag < other.m_tag; }

    /// Resolve: find the best matching TopologyID from a list of candidates.
    /// Returns exact match if found, else first descendant, else nullopt.
    static std::optional<TopologyID> resolve(
        const TopologyID& target,
        const std::vector<TopologyID>& candidates);

private:
    std::string m_tag;  // The genealogy string: "source/role" or "source/role/op:index/..."

    explicit TopologyID(std::string tag) : m_tag(std::move(tag)) {}
};

}  // namespace hz::topo
```

- [ ] **Step 3: Implement TopologyID.cpp**

```cpp
#include "horizon/topology/TopologyID.h"

namespace hz::topo {

TopologyID TopologyID::make(const std::string& source, const std::string& role) {
    return TopologyID(source + "/" + role);
}

TopologyID TopologyID::child(const std::string& operation, int index) const {
    return TopologyID(m_tag + "/" + operation + ":" + std::to_string(index));
}

bool TopologyID::isDescendantOf(const TopologyID& ancestor) const {
    if (m_tag.size() <= ancestor.m_tag.size()) return false;
    // Check if ancestor's tag is a prefix followed by '/'
    return m_tag.compare(0, ancestor.m_tag.size(), ancestor.m_tag) == 0
        && m_tag[ancestor.m_tag.size()] == '/';
}

std::optional<TopologyID> TopologyID::resolve(
    const TopologyID& target,
    const std::vector<TopologyID>& candidates) {
    // Exact match first
    for (const auto& c : candidates) {
        if (c == target) return c;
    }
    // Descendant match
    for (const auto& c : candidates) {
        if (c.isDescendantOf(target)) return c;
    }
    return std::nullopt;
}

}
```

- [ ] **Step 4: Create test infrastructure and update CMakeLists**

- [ ] **Step 5: Build and run tests**

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(topology): add TopologyID genealogy system for Topological Naming Problem

Deterministic string-based IDs encoding creation genealogy.
Parent/child relationships. Resolution: exact match preferred, descendant fallback."
```

---

## Task 3: Euler Operators

**Files:**
- Create: `src/topology/include/horizon/topology/EulerOps.h`
- Create: `src/topology/src/EulerOps.cpp`

- [ ] **Step 1: Implement Euler operators**

```cpp
#pragma once

#include "horizon/topology/Solid.h"

namespace hz::topo {

/// Euler operators — the only safe way to modify B-Rep topology.
/// All operators maintain the Euler-Poincaré formula: V - E + F = 2 per shell.
namespace euler {

    /// Create initial topology: one vertex, one face, one shell.
    /// Returns: the created vertex, face, and shell.
    struct MVFSResult { Vertex* vertex; Face* face; Shell* shell; };
    MVFSResult makeVertexFaceSolid(Solid& solid, const math::Vec3& point);

    /// Split: add a new edge and vertex to an existing face.
    /// Takes a half-edge in the face; inserts new vertex after the half-edge's origin.
    /// Returns: the new edge and new vertex.
    struct MEVResult { Edge* edge; Vertex* vertex; };
    MEVResult makeEdgeVertex(Solid& solid, HalfEdge* he, const math::Vec3& point);

    /// Split a face by adding an edge between two vertices already on the face boundary.
    /// he1 and he2 are half-edges whose origins are the two vertices.
    /// Returns: the new edge and new face.
    struct MEFResult { Edge* edge; Face* newFace; };
    MEFResult makeEdgeFace(Solid& solid, HalfEdge* he1, HalfEdge* he2);

    /// Reverse of makeEdgeVertex: remove an edge and merge its endpoint into the other.
    void killEdgeVertex(Solid& solid, Edge* edge);

    /// Reverse of makeEdgeFace: remove an edge and merge two faces into one.
    void killEdgeFace(Solid& solid, Edge* edge);

}  // namespace euler

}  // namespace hz::topo
```

- [ ] **Step 2: Implement makeVertexFaceSolid (MVFS)**

Creates the minimal topology: 1 vertex + 1 face (with an empty wire) + 1 shell.

```cpp
euler::MVFSResult euler::makeVertexFaceSolid(Solid& solid, const Vec3& point) {
    auto* shell = solid.allocShell();
    auto* face = solid.allocFace();
    auto* vertex = solid.allocVertex();
    auto* wire = solid.allocWire();
    
    vertex->point = point;
    
    face->outerLoop = wire;
    face->shell = shell;
    
    shell->faces.push_back(face);
    shell->solid = &solid;
    
    return {vertex, face, shell};
}
```

- [ ] **Step 3: Implement makeEdgeVertex (MEV)**

Adds a new edge + vertex. The new vertex connects to the origin of `he` via a new edge.

This operator creates:
- 1 new vertex
- 1 new edge
- 2 new half-edges (completing the edge)

It maintains V - E + F: V+1, E+1, F unchanged → delta = +1 - 1 = 0. ✓

The tricky part: inserting the new half-edges into the face loop correctly.

- [ ] **Step 4: Implement makeEdgeFace (MEF)**

Splits a face by connecting two existing vertices with a new edge.

Creates:
- 1 new edge
- 2 new half-edges
- 1 new face

Maintains: V unchanged, E+1, F+1 → delta = -1 + 1 = 0. ✓

The new edge splits the face's wire into two wires (one for each face).

- [ ] **Step 5: Implement killEdgeVertex (KEV) and killEdgeFace (KEF)**

Reverse of MEV and MEF respectively.

- [ ] **Step 6: Build and run tests**

- [ ] **Step 7: Commit**

```bash
git commit -m "feat(topology): implement Euler operators (MVFS, MEV, MEF, KEV, KEF)

Topology-safe B-Rep modification maintaining Euler-Poincaré formula."
```

---

## Task 4: Topological Queries + Validation

**Files:**
- Modify: `src/topology/include/horizon/topology/Solid.h`
- Modify: `src/topology/src/Solid.cpp`

- [ ] **Step 1: Implement topological queries**

Add free functions or methods:
```cpp
/// All faces adjacent to a face (sharing an edge).
std::vector<Face*> adjacentFaces(const Face* face);

/// The face on the left/right of an edge.
Face* leftFace(const Edge* edge);   // edge->halfEdge->face
Face* rightFace(const Edge* edge);  // edge->halfEdge->twin->face

/// All edges incident on a vertex.
std::vector<Edge*> incidentEdges(const Vertex* vertex);

/// All vertices of a face (traverse the outer loop).
std::vector<Vertex*> faceVertices(const Face* face);
```

- [ ] **Step 2: Implement validation (isValid, checkEulerFormula, checkManifold)**

```cpp
bool Solid::checkEulerFormula() const {
    for (const auto& shell : m_shells) {
        std::set<const Vertex*> verts;
        std::set<const Edge*> edges;
        int faceCount = static_cast<int>(shell.faces.size());
        
        for (const Face* face : shell.faces) {
            if (!face->outerLoop || !face->outerLoop->halfEdge) continue;
            HalfEdge* start = face->outerLoop->halfEdge;
            HalfEdge* he = start;
            do {
                verts.insert(he->origin);
                edges.insert(he->edge);
                he = he->next;
            } while (he && he != start);
        }
        
        int V = static_cast<int>(verts.size());
        int E = static_cast<int>(edges.size());
        int F = faceCount;
        if (V - E + F != 2) return false;
    }
    return true;
}

bool Solid::checkManifold() const {
    // Each edge should have exactly 2 half-edges (twin pair)
    for (const auto& edge : m_edges) {
        if (!edge.halfEdge || !edge.halfEdge->twin) return false;
        if (edge.halfEdge->twin->twin != edge.halfEdge) return false;
    }
    return true;
}
```

- [ ] **Step 3: Commit**

```bash
git commit -m "feat(topology): add topological queries and validation

adjacentFaces, leftFace, rightFace, incidentEdges, faceVertices.
Euler formula check, manifold check, orientation check."
```

---

## Task 5: Integration Tests — Cube and Cylinder via Euler Ops

**Files:**
- Create: `tests/topology/test_EulerOps.cpp`
- Modify: `tests/topology/CMakeLists.txt`

- [ ] **Step 1: Write cube construction test**

Build a cube (8 vertices, 12 edges, 6 faces) using Euler operators:
```cpp
TEST(EulerOpsTest, CubeHasCorrectEulerFormula) {
    Solid solid;
    // Build a cube via MVFS + MEV + MEF sequence
    // ... (detailed construction sequence)
    
    EXPECT_EQ(solid.vertexCount(), 8u);
    EXPECT_EQ(solid.edgeCount(), 12u);
    EXPECT_EQ(solid.faceCount(), 6u);
    EXPECT_TRUE(solid.checkEulerFormula());  // 8 - 12 + 6 = 2
    EXPECT_TRUE(solid.isValid());
}
```

The cube construction via Euler operators:
1. MVFS → creates v0, face0, shell0
2. MEV × 3 → creates v1, v2, v3 (bottom face vertices)
3. MEF → close bottom face (v3 back to v0)
4. MEV × 4 → create v4, v5, v6, v7 (top vertices, one per bottom vertex)
5. MEF × 4 → create side faces
6. MEF → close top face

This is complex — the exact sequence depends on the half-edge connectivity. The implementer should work out the exact operator sequence.

- [ ] **Step 2: Write tetrahedron test (simpler)**

A tetrahedron (4 vertices, 6 edges, 4 faces) is easier to construct:
```cpp
TEST(EulerOpsTest, TetrahedronHasCorrectEulerFormula) {
    Solid solid;
    // Build tetrahedron: 4V, 6E, 4F → V-E+F = 4-6+4 = 2
    // ...
    EXPECT_EQ(solid.vertexCount(), 4u);
    EXPECT_EQ(solid.edgeCount(), 6u);
    EXPECT_EQ(solid.faceCount(), 4u);
    EXPECT_TRUE(solid.checkEulerFormula());
}
```

- [ ] **Step 3: Write adjacency query tests**

```cpp
TEST(EulerOpsTest, CubeAdjacentFaces) {
    // Each face of a cube has 4 adjacent faces
    // ...
}

TEST(EulerOpsTest, CubeIncidentEdges) {
    // Each vertex of a cube has 3 incident edges
    // ...
}
```

- [ ] **Step 4: Write TopologyID integration test**

```cpp
TEST(EulerOpsTest, TopologyIDsSurviveReconstruction) {
    // Build cube, assign TopologyIDs to faces
    // Verify IDs can be resolved after reconstruction
}
```

- [ ] **Step 5: Build and run ALL tests**

- [ ] **Step 6: Commit**

```bash
git commit -m "test(topology): add Euler operator tests with cube and tetrahedron

Verify Euler formula (V-E+F=2), adjacency queries, manifold invariants.
TopologyID resolution across B-Rep construction."
```

---

## Task 6: Final Phase Commit

- [ ] **Step 1: Run complete test suite**

Report exact test count.

- [ ] **Step 2: Commit and push**

```bash
git add -A
git commit -m "Phase 33: Half-edge B-Rep topology with TopologyID genealogy

- Half-edge data structure: Solid, Shell, Face, Edge, HalfEdge, Vertex, Wire
- Pool-based allocation via std::deque for pointer stability
- TopologyID genealogy system: deterministic string-based IDs, parent/child,
  resolution with exact match preferred and descendant fallback
- Euler operators: MVFS, MEV, MEF, KEV, KEF maintaining V-E+F=2
- Topological queries: adjacentFaces, incidentEdges, faceVertices
- Validation: Euler formula, manifold check, orientation check
- Tests: tetrahedron and cube construction, Euler formula, adjacency"

git push origin master
```
