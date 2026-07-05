#pragma once

#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/simulation/TetMesh.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::sim {

/// An axis-aligned bounding box.
struct Aabb {
    math::Vec3 min{0.0, 0.0, 0.0};
    math::Vec3 max{0.0, 0.0, 0.0};
    bool valid = false;  ///< false for an empty solid (no vertices)
};

/// The axis-aligned bounding box of @p solid, computed from its B-Rep vertices.
Aabb solidAabb(const topo::Solid& solid);

/// Mesh the axis-aligned bounding box of @p solid into a structured tet mesh of
/// nx*ny*nz*6 tetrahedra, positioned at the box's location.
///
/// For an axis-aligned box solid this meshes the solid exactly; for other solids
/// it meshes the bounding box — a coarse analysis domain (a full B-Rep
/// tetrahedralizer is a follow-up). Returns an empty mesh for an empty solid,
/// a zero-extent bounding box, or a subdivision count < 1.
TetMesh meshSolidBoundingBox(const topo::Solid& solid, int nx, int ny, int nz);

/// Indices of the mesh nodes lying on the plane {component[axis] == coord} to
/// within @p tol. @p axis is 0 (x), 1 (y), or 2 (z). Use this to pick the node
/// set of a face for a fixed support or a distributed load.
std::vector<int> nodesOnPlane(const TetMesh& mesh, int axis, double coord, double tol = 1e-6);

}  // namespace hz::sim
