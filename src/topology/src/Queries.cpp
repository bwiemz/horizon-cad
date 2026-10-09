#include "horizon/topology/Queries.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace hz::topo {

// ---------------------------------------------------------------------------
// adjacentFaces
// ---------------------------------------------------------------------------

std::vector<Face*> adjacentFaces(const Face* face) {
    std::vector<Face*> result;
    if (face == nullptr || face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) {
        return result;
    }

    std::unordered_set<Face*> seen;
    const HalfEdge* start = face->outerLoop->halfEdge;
    const HalfEdge* cur = start;
    do {
        if (cur->twin != nullptr && cur->twin->face != nullptr && cur->twin->face != face) {
            Face* neighbor = cur->twin->face;
            if (seen.insert(neighbor).second) {
                result.push_back(neighbor);
            }
        }
        if (cur->next == nullptr) {
            break;  // a loop that stops part way round: report what it has
        }
        cur = cur->next;
    } while (cur != start);

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
    std::unordered_set<Edge*> seen;
    const HalfEdge* start = vertex->halfEdge;
    const HalfEdge* cur = start;
    do {
        if (cur->edge != nullptr && seen.insert(cur->edge).second) {
            result.push_back(cur->edge);
        }
        // Move to the next outgoing half-edge from this vertex.
        // cur->twin goes to the other end; cur->twin->next starts from our vertex again.
        if (cur->twin == nullptr || cur->twin->next == nullptr) {
            break;
        }
        cur = cur->twin->next;
    } while (cur != start);

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

    const HalfEdge* start = face->outerLoop->halfEdge;
    const HalfEdge* cur = start;
    do {
        result.push_back(cur->origin);
        if (cur->next == nullptr) {
            break;  // a loop that stops part way round: report what it has
        }
        cur = cur->next;
    } while (cur != start);

    return result;
}

// ---------------------------------------------------------------------------
// loopSize
// ---------------------------------------------------------------------------

int loopSize(const Wire* wire) {
    if (wire == nullptr || wire->halfEdge == nullptr) {
        return 0;
    }

    // A closed loop comes back to where it started, and that is what ends the
    // walk. A chain that stops part way ends it too. A chain that runs on into
    // itself and never comes back would spin here forever, so each half-edge
    // is noted as it is passed: coming on one twice means the walk is going
    // round something other than this loop, and it stops there. The count is
    // the number of half-edges before that, which is what the loop has.
    int count = 0;
    const HalfEdge* const start = wire->halfEdge;
    std::unordered_set<const HalfEdge*> walked;
    walked.insert(start);

    for (const HalfEdge* cur = start->next; cur != nullptr && cur != start; cur = cur->next) {
        if (!walked.insert(cur).second) {
            break;  // round something other than this loop: it never closes
        }
        ++count;
    }

    return count + 1;  // the half-edge it started on
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
