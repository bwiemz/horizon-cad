#pragma once

#include "horizon/topology/HalfEdge.h"

#include <deque>
#include <string>

namespace hz::topo {

/// Top-level B-Rep container.
///
/// Owns all topological entities via pool-based allocation (std::deque —
/// push_back never invalidates existing pointers).  Provides validation
/// helpers including the Euler–Poincaré formula and manifold checks.
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

    size_t vertexCount() const;
    size_t edgeCount() const;
    size_t faceCount() const;
    size_t shellCount() const;

    // -- Validation ----------------------------------------------------------

    /// True if the data structure passes all structural checks.
    bool isValid() const;

    /// Euler–Poincaré: V - E + F = 2(S - H) where H = total inner-loop count.
    bool checkEulerFormula() const;

    /// Every edge has exactly two half-edges that are twins of each other, and
    /// every half-edge loop is properly closed.
    bool checkManifold() const;

    /// Human-readable report of all validation issues found.
    std::string validationReport() const;

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
