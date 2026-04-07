#include "horizon/topology/Solid.h"

#include <sstream>
#include <unordered_set>

namespace hz::topo {

Solid::Solid() = default;

// -- Pool allocators ---------------------------------------------------------

Vertex* Solid::allocVertex() {
    auto& v = m_vertices.emplace_back();
    v.id = m_nextId++;
    return &v;
}

HalfEdge* Solid::allocHalfEdge() {
    auto& he = m_halfEdges.emplace_back();
    he.id = m_nextId++;
    return &he;
}

Edge* Solid::allocEdge() {
    auto& e = m_edges.emplace_back();
    e.id = m_nextId++;
    return &e;
}

Wire* Solid::allocWire() {
    auto& w = m_wires.emplace_back();
    w.id = m_nextId++;
    return &w;
}

Face* Solid::allocFace() {
    auto& f = m_faces.emplace_back();
    f.id = m_nextId++;
    return &f;
}

Shell* Solid::allocShell() {
    auto& s = m_shells.emplace_back();
    s.id = m_nextId++;
    return &s;
}

// -- Const views -------------------------------------------------------------

const std::deque<Vertex>& Solid::vertices() const {
    return m_vertices;
}

const std::deque<Edge>& Solid::edges() const {
    return m_edges;
}

const std::deque<Face>& Solid::faces() const {
    return m_faces;
}

const std::deque<Shell>& Solid::shells() const {
    return m_shells;
}

size_t Solid::vertexCount() const {
    return m_vertices.size();
}

size_t Solid::edgeCount() const {
    return m_edges.size();
}

size_t Solid::faceCount() const {
    return m_faces.size();
}

size_t Solid::shellCount() const {
    return m_shells.size();
}

// -- Validation --------------------------------------------------------------

bool Solid::isValid() const {
    return checkEulerFormula() && checkManifold();
}

bool Solid::checkEulerFormula() const {
    // Euler–Poincaré for orientable 2-manifolds:
    //   V - E + F = 2 * (S - H)
    // where S = number of shells, H = total number of inner loops (holes in faces).
    const auto V = static_cast<int>(m_vertices.size());
    const auto E = static_cast<int>(m_edges.size());
    const auto F = static_cast<int>(m_faces.size());
    const auto S = static_cast<int>(m_shells.size());

    int H = 0;
    for (const auto& face : m_faces) {
        H += static_cast<int>(face.innerLoops.size());
    }

    return (V - E + F) == 2 * (S - H);
}

bool Solid::checkManifold() const {
    // 1) Every half-edge must have a twin, and twins must be reciprocal.
    for (const auto& he : m_halfEdges) {
        if (he.twin == nullptr) {
            return false;
        }
        if (he.twin->twin != &he) {
            return false;
        }
    }

    // 2) Every half-edge loop (next chain) must be closed.
    for (const auto& he : m_halfEdges) {
        const HalfEdge* cursor = &he;
        size_t limit = m_halfEdges.size();
        size_t count = 0;
        do {
            if (cursor->next == nullptr) {
                return false;
            }
            cursor = cursor->next;
            ++count;
            if (count > limit) {
                return false;  // Infinite loop.
            }
        } while (cursor != &he);
    }

    // 3) Every half-edge's prev must be consistent with next.
    for (const auto& he : m_halfEdges) {
        if (he.next == nullptr || he.next->prev != &he) {
            return false;
        }
    }

    // 4) Every edge must have exactly two half-edges.
    for (const auto& edge : m_edges) {
        if (edge.halfEdge == nullptr) {
            return false;
        }
        if (edge.halfEdge->twin == nullptr) {
            return false;
        }
        if (edge.halfEdge->edge != &edge || edge.halfEdge->twin->edge != &edge) {
            return false;
        }
    }

    return true;
}

std::string Solid::validationReport() const {
    std::ostringstream out;

    // Euler formula
    {
        const auto V = static_cast<int>(m_vertices.size());
        const auto E = static_cast<int>(m_edges.size());
        const auto F = static_cast<int>(m_faces.size());
        const auto S = static_cast<int>(m_shells.size());
        int H = 0;
        for (const auto& face : m_faces) {
            H += static_cast<int>(face.innerLoops.size());
        }
        const int lhs = V - E + F;
        const int rhs = 2 * (S - H);
        if (lhs != rhs) {
            out << "Euler formula FAIL: V(" << V << ") - E(" << E << ") + F(" << F << ") = " << lhs
                << ", expected 2*(S(" << S << ") - H(" << H << ")) = " << rhs << "\n";
        } else {
            out << "Euler formula OK: V=" << V << " E=" << E << " F=" << F << " S=" << S
                << " H=" << H << "\n";
        }
    }

    // Manifold checks (detailed)
    int twinErrors = 0;
    int loopErrors = 0;
    int prevErrors = 0;

    for (const auto& he : m_halfEdges) {
        if (he.twin == nullptr || he.twin->twin != &he) {
            ++twinErrors;
        }
    }

    for (const auto& he : m_halfEdges) {
        const HalfEdge* cursor = &he;
        size_t limit = m_halfEdges.size();
        size_t count = 0;
        bool ok = true;
        do {
            if (cursor->next == nullptr) {
                ok = false;
                break;
            }
            cursor = cursor->next;
            ++count;
            if (count > limit) {
                ok = false;
                break;
            }
        } while (cursor != &he);
        if (!ok) {
            ++loopErrors;
        }
    }

    for (const auto& he : m_halfEdges) {
        if (he.next == nullptr || he.next->prev != &he) {
            ++prevErrors;
        }
    }

    if (twinErrors > 0) {
        out << "Twin errors: " << twinErrors << " half-edges with bad twin linkage\n";
    }
    if (loopErrors > 0) {
        out << "Loop errors: " << loopErrors << " half-edges in non-closed loops\n";
    }
    if (prevErrors > 0) {
        out << "Prev errors: " << prevErrors << " half-edges with inconsistent prev/next\n";
    }
    if (twinErrors == 0 && loopErrors == 0 && prevErrors == 0) {
        out << "Manifold checks OK\n";
    }

    return out.str();
}

}  // namespace hz::topo
