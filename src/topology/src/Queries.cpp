#include "horizon/topology/Queries.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace hz::topo {

namespace {

/// Whether the chain that @p step follows from @p start ends by itself: by
/// coming back round to @p start, as a ring does, or at a half-edge with
/// nothing after it, as a loop only partly built does. Either way it can be
/// read without noting what it passes.
///
/// Two walkers go along it, one twice as fast. On a chain that ends by itself,
/// the fast one reaches the start or the end first. On one that runs on into
/// itself and never comes back, neither ever happens, and the fast one comes
/// up behind the slow one instead. Nothing is allocated. Every loop query
/// asks this, and a drawing's projection asks for the normal of a
/// 2048-sided cap once per edge on its rim: a set of every half-edge passed,
/// built afresh each time, made that projection several times slower.
template <typename Step>
bool endsByItself(const HalfEdge* const start, Step step) {
    const HalfEdge* slow = start;
    const HalfEdge* fast = start;
    for (;;) {
        for (int i = 0; i < 2; ++i) {
            fast = step(fast);
            if (fast == nullptr || fast == start) {
                return true;
            }
        }
        slow = step(slow);
        if (slow == fast) {
            return false;  // caught up from behind: it runs on into itself
        }
    }
}

/// Hands @p visit every half-edge of the chain @p step follows from @p start,
/// in order, up to the point the walk has to stop.
///
/// A half-edge loop is a ring: it ends by coming back to where it started. A
/// loop that stops part way round — one only partly built, or one whose
/// half-edge a kill has left with no `next` — ends there instead, and is read
/// as far as it goes. A chain that runs on into itself and never comes back
/// would have no end at all, so there each half-edge passed is noted, and
/// coming on one a second time ends the walk. Without that, such a chain was
/// followed forever: the application stopped, with nothing on screen and no
/// way to stop it. A chain that ends by itself is read straight through, so
/// what a sound loop costs and returns is what it did before.
template <typename Step, typename Visit>
void walkChain(const HalfEdge* const start, Step step, Visit visit) {
    if (start == nullptr) {
        return;
    }
    if (endsByItself(start, step)) {
        const HalfEdge* cur = start;
        do {
            visit(cur);
            cur = step(cur);
        } while (cur != nullptr && cur != start);
        return;
    }
    std::unordered_set<const HalfEdge*> seen;
    for (const HalfEdge* cur = start; cur != nullptr && seen.insert(cur).second; cur = step(cur)) {
        visit(cur);
    }
}

/// Every half-edge of @p wire's loop, as walkChain reads it.
template <typename Visit>
void walkLoop(const Wire* wire, Visit visit) {
    if (wire == nullptr) {
        return;
    }
    walkChain(
        wire->halfEdge, [](const HalfEdge* he) { return he->next; }, visit);
}

}  // namespace

// ---------------------------------------------------------------------------
// adjacentFaces
// ---------------------------------------------------------------------------

std::vector<Face*> adjacentFaces(const Face* face) {
    std::vector<Face*> result;
    if (face == nullptr || face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) {
        return result;
    }

    std::unordered_set<Face*> seen;
    walkLoop(face->outerLoop, [&](const HalfEdge* cur) {
        if (cur->twin != nullptr && cur->twin->face != nullptr && cur->twin->face != face) {
            Face* neighbor = cur->twin->face;
            if (seen.insert(neighbor).second) {
                result.push_back(neighbor);
            }
        }
    });

    return result;
}

// ---------------------------------------------------------------------------
// leftFace / rightFace
// ---------------------------------------------------------------------------

Face* leftFace(const Edge* edge) {
    if (edge == nullptr || edge->halfEdge == nullptr) {
        return nullptr;
    }
    return edge->halfEdge->face;
}

Face* rightFace(const Edge* edge) {
    if (edge == nullptr || edge->halfEdge == nullptr || edge->halfEdge->twin == nullptr) {
        return nullptr;
    }
    return edge->halfEdge->twin->face;
}

// ---------------------------------------------------------------------------
// incidentEdges
// ---------------------------------------------------------------------------

std::vector<Edge*> incidentEdges(const Vertex* vertex) {
    std::vector<Edge*> result;
    if (vertex == nullptr || vertex->halfEdge == nullptr) {
        return result;
    }

    // Walk around the vertex using the twin/next pattern.
    // Starting from vertex->halfEdge, each outgoing half-edge leads to an edge.
    // he->twin->next gives the next outgoing half-edge from the same vertex.
    // The fan around a vertex closes as surely as a loop does, so it is
    // walked the same way: it ends at the half-edge it started from, or at one
    // with nothing after it, and a ring that never comes back to it is noted
    // and stopped rather than walked forever.
    std::unordered_set<Edge*> seen;
    walkChain(
        vertex->halfEdge,
        // cur->twin goes to the other end; cur->twin->next starts from our vertex again.
        [](const HalfEdge* cur) { return cur->twin != nullptr ? cur->twin->next : nullptr; },
        [&](const HalfEdge* cur) {
            if (cur->edge != nullptr && seen.insert(cur->edge).second) {
                result.push_back(cur->edge);
            }
        });

    return result;
}

// ---------------------------------------------------------------------------
// faceVertices
// ---------------------------------------------------------------------------

std::vector<Vertex*> faceVertices(const Face* face) {
    std::vector<Vertex*> result;
    if (face == nullptr || face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) {
        return result;
    }

    walkLoop(face->outerLoop, [&result](const HalfEdge* cur) { result.push_back(cur->origin); });

    return result;
}

// ---------------------------------------------------------------------------
// loopSize
// ---------------------------------------------------------------------------

int loopSize(const Wire* wire) {
    int count = 0;
    walkLoop(wire, [&count](const HalfEdge*) { ++count; });
    return count;
}

// ---------------------------------------------------------------------------
// loopNormal / signedVolume
// ---------------------------------------------------------------------------

math::Vec3 loopNormal(const Face* face) {
    const auto verts = faceVertices(face);
    // Newell's sum about the first point: its error then does not grow with
    // the loop's distance from the origin.
    math::Vec3 n(0, 0, 0);
    for (size_t i = 0; i < verts.size(); ++i) {
        const math::Vec3 a = verts[i]->point - verts[0]->point;
        const math::Vec3 b = verts[(i + 1) % verts.size()]->point - verts[0]->point;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    const double len = n.length();
    return len > 1e-12 ? n * (1.0 / len) : math::Vec3(0, 0, 0);
}

double signedVolume(const Shell& shell) {
    // About a point of the shell's own: about the origin, the rounding grows
    // as the cube of the distance, and far out it outweighed the volume.
    math::Vec3 o;
    for (const Face* face : shell.faces) {
        if (face && face->outerLoop && face->outerLoop->halfEdge &&
            face->outerLoop->halfEdge->origin) {
            o = face->outerLoop->halfEdge->origin->point;
            break;
        }
    }
    double volume = 0.0;
    const auto fan = [&volume, &o](const Wire* wire) {
        if (!wire || !wire->halfEdge || !wire->halfEdge->origin) return;
        const HalfEdge* start = wire->halfEdge;
        const math::Vec3 a = start->origin->point - o;
        for (const HalfEdge* he = start->next; he && he->next && he->next != start; he = he->next) {
            if (!he->origin || !he->next->origin) return;
            volume += a.dot((he->origin->point - o).cross(he->next->origin->point - o)) / 6.0;
        }
    };
    for (const Face* face : shell.faces) {
        fan(face->outerLoop);
        for (const Wire* inner : face->innerLoops) fan(inner);
    }
    return volume;
}

}  // namespace hz::topo
