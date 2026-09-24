#pragma once

#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/HalfEdge.h"

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

/// The unit Newell normal of a face's outer loop, or zero for a degenerate
/// loop. It follows the loop's order, not the carrier surface: walking the
/// loop with this normal up, the face's interior lies to the left, whatever
/// the shape of the face and whichever way its body is wound. So it gives the
/// inward side of an edge on a non-convex face, where comparing with the
/// face's centroid does not.
math::Vec3 loopNormal(const Face* face);

/// The volume a shell encloses, signed by its winding: positive when its
/// loops' normals point out of what it encloses. Each loop, holes included, is
/// fanned from its first vertex (the divergence theorem).
double signedVolume(const Shell& shell);

}  // namespace hz::topo
