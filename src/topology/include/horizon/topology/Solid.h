#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "horizon/topology/HalfEdge.h"

namespace hz::topo {

/// Top-level B-Rep container.
///
/// Owns all topological entities via pool-based allocation (std::deque —
/// push_back never invalidates existing pointers).  Provides validation
/// helpers including the Euler–Poincaré formula and manifold checks.
///
/// The validation here is **combinatorial only**: it inspects twin/next/prev
/// linkage and entity counts and never reads a coordinate.  A solid can
/// therefore pass `isValid()` while its loops are self-intersecting,
/// non-planar, or degenerate.  Geometric validation lives in
/// `GeometryValidator` (GeometryValidator.h); construction code should gate
/// on both.
class Solid {
public:
    Solid();

    // -- Pool allocators (return stable pointers) ----------------------------

    Vertex* allocVertex();
    HalfEdge* allocHalfEdge();
    Edge* allocEdge();
    Wire* allocWire();
    Face* allocFace();
    Shell* allocShell();

    // -- Const views ---------------------------------------------------------

    const std::deque<Vertex>& vertices() const;
    const std::deque<Edge>& edges() const;
    const std::deque<Face>& faces() const;
    const std::deque<Shell>& shells() const;

    // -- Mutable views: for naming and binding geometry to entities already
    //    allocated. Adding or removing through them breaks the pools'
    //    pointer stability; use the allocators. ---------------------------

    std::deque<Vertex>& vertices() { return m_vertices; }
    std::deque<Edge>& edges() { return m_edges; }
    std::deque<Face>& faces() { return m_faces; }

    size_t vertexCount() const;
    size_t edgeCount() const;
    size_t faceCount() const;
    size_t shellCount() const;

    // -- Validation ----------------------------------------------------------

    /// True if the data structure passes all structural checks.  Structural
    /// only — see the class note and `GeometryValidator`.
    bool isValid() const;

    /// Euler–Poincaré: V − E + F − R = 2(S − G), with R the faces' inner
    /// loops (rings) and G the genus (handles: a torus has one, a plate with
    /// one hole through it has one). G is not stored, so this checks what the
    /// formula can: the left side is even and at most 2S.
    bool checkEulerFormula() const;

    /// The genus the counts imply, S − (V − E + F − R)/2 — the number of
    /// through-holes, across all shells. Meaningful when `checkEulerFormula()`
    /// holds.
    int genus() const;

    /// Every edge has exactly two half-edges that are twins of each other, and
    /// every half-edge loop is properly closed.
    bool checkManifold() const;

    /// Human-readable report of all validation issues found.
    std::string validationReport() const;

private:
    /// V − E + F − R.
    int eulerCharacteristicLessRings() const;

    std::deque<Vertex> m_vertices;
    std::deque<HalfEdge> m_halfEdges;
    std::deque<Edge> m_edges;
    std::deque<Wire> m_wires;
    std::deque<Face> m_faces;
    std::deque<Shell> m_shells;
    uint32_t m_nextId = 1;
};

}  // namespace hz::topo
