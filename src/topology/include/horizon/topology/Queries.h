#pragma once

#include "horizon/topology/HalfEdge.h"

#include <vector>

namespace hz::topo {

/// All faces sharing an edge with the given face.
std::vector<Face*> adjacentFaces(const Face* face);

/// The face on the left side of an edge (edge->halfEdge->face).
Face* leftFace(const Edge* edge);

/// The face on the right side of an edge (edge->halfEdge->twin->face).
Face* rightFace(const Edge* edge);

/// All edges meeting at a vertex.
std::vector<Edge*> incidentEdges(const Vertex* vertex);

/// All vertices on a face's outer loop (ordered).
std::vector<Vertex*> faceVertices(const Face* face);

/// Count half-edges in a face's outer loop.
int loopSize(const Wire* wire);

}  // namespace hz::topo
