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
        assert(cur->origin == vertex);
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

    int count = 0;
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    do {
        ++count;
        cur = cur->next;
    } while (cur != start);

    return count;
}

}  // namespace hz::topo
