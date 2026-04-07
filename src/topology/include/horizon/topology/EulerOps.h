#pragma once

#include "horizon/math/Vec3.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::topo {

class Solid;

/// Result of makeVertexFaceSolid: the seed vertex, face, and shell.
struct MVFSResult {
    Vertex* vertex = nullptr;
    Face* face = nullptr;
    Shell* shell = nullptr;
};

/// Result of makeEdgeVertex: a new edge and the new vertex at its tip.
struct MEVResult {
    Edge* edge = nullptr;
    Vertex* vertex = nullptr;
};

/// Result of makeEdgeFace: a new edge and the new face it carved off.
struct MEFResult {
    Edge* edge = nullptr;
    Face* newFace = nullptr;
};

/// Euler operators — the ONLY safe way to modify B-Rep topology.
///
/// Every operator preserves the Euler-Poincare formula:
///     V - E + F = 2(S - H)
namespace euler {

/// Create the minimal valid topology: 1 vertex + 1 face + 1 shell.
/// @pre The solid is empty (or at least has no shells yet).
/// @post V=1, E=0, F=1, S=1.  Euler: 1 - 0 + 1 = 2.
MVFSResult makeVertexFaceSolid(Solid& solid, const math::Vec3& point);

/// Add a new edge + vertex extending from an existing vertex in a face.
/// @param solid   The owning solid.
/// @param he      A half-edge whose origin is the existing vertex (the attach
///                point).  Pass nullptr for the very first edge on an MVFS face.
/// @param face    The face to operate on (required when he is nullptr).
/// @param point   Position of the new vertex.
/// @return The new edge and new vertex.
///
/// Delta: V+1, E+1, F+0 → Euler delta = 0.
MEVResult makeEdgeVertex(Solid& solid, HalfEdge* he, Face* face, const math::Vec3& point);

/// Split a face by connecting two existing vertices with a new edge.
/// @param solid  The owning solid.
/// @param he1    Half-edge whose origin is the first vertex to connect.
/// @param he2    Half-edge whose origin is the second vertex to connect.
///               Both must lie on the same face.
/// @return The new edge and the new face carved from the old one.
///
/// Delta: V+0, E+1, F+1 → Euler delta = 0.
MEFResult makeEdgeFace(Solid& solid, HalfEdge* he1, HalfEdge* he2);

/// Reverse of makeEdgeVertex — remove an edge and one of its vertices.
/// The edge's half-edges are spliced out of the face loop and the target
/// vertex (the one that becomes dangling) is conceptually merged away.
/// @param solid  The owning solid.
/// @param edge   The edge to remove. One endpoint must have valence 1 (i.e.
///               the half-edge going to it and coming from it are twins of
///               the same edge).
///
/// Delta: V-1, E-1, F+0 → Euler delta = 0.
void killEdgeVertex(Solid& solid, Edge* edge);

/// Reverse of makeEdgeFace — remove an edge and merge the two adjacent faces.
/// The face on the twin's side is removed; all its half-edges move to the
/// other face.
/// @param solid  The owning solid.
/// @param edge   The edge to remove.  Its two half-edges must belong to
///               different faces.
///
/// Delta: V+0, E-1, F-1 → Euler delta = 0.
void killEdgeFace(Solid& solid, Edge* edge);

}  // namespace euler
}  // namespace hz::topo
