#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

struct MeshOptions {
    double chordTolerance = 0.01;  // largest distance from the mesh to the surface
    double maxAngle = 0.35;        // radians between neighbouring triangle normals on a face,
                                   // at most pi
    std::size_t maxTrianglesPerFace = 200000;
};

struct FaceMesh {
    const topo::Face* face = nullptr;
    std::vector<math::Vec3> points;
    std::vector<math::Vec3> normals;                      // the surface's, outward
    std::vector<std::array<std::uint32_t, 3>> triangles;  // wound outward
    /// Empty when the face is meshed. Otherwise why it could not be (no exact geometry, an edge
    /// with no twin, a loop that cannot be laid out in (u, v), a triangulation problem, boundary
    /// points that meet in (u, v) at different positions, the triangle cap reached before the
    /// tolerance was met), and the face has no triangles: a mesh is never partial or out of
    /// tolerance.
    std::string problem;
};

class TrimmedFaceMesher {
public:
    /// Every exact face of @p solid meshed. Each edge is discretised once, and both its
    /// faces use those very points, so the meshes meet without gaps.
    ///
    /// One FaceMesh for each live face, in the solid's order, including those that could not be
    /// meshed (see FaceMesh::problem). The solid is expected to pass topo::validateExact: loops
    /// are laid out in (u, v) as the validator lays them out (horizon/topology/LoopUnwrap.h).
    /// A boundary vertex is the edge sample itself (a vertex's own point at an edge's ends), so
    /// on a seam the two sides' vertices have the same position and distinct (u, v). Within
    /// each triangle the surface at its (u, v) centroid and edge midpoints is within
    /// chordTolerance of it, and the triangle faces the way the surface does at its centroid.
    /// Triangles that share an edge of a face's mesh differ in normal by at most maxAngle.
    /// chordTolerance is absolute: scale it with the part.
    static std::vector<FaceMesh> meshSolid(const topo::Solid& solid, const MeshOptions& options);
};

}  // namespace hz::model
