#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "horizon/math/Vec3.h"

namespace hz::sim {

/// A mesh node (finite-element vertex).
struct Node {
    math::Vec3 position;
};

/// A 4-node linear tetrahedron: indices into TetMesh::nodes, counter-clockwise
/// so a positive signed volume corresponds to the canonical orientation.
struct Tet4 {
    std::array<int, 4> nodes{0, 0, 0, 0};
};

/// A tetrahedral finite-element mesh: nodes plus the elements connecting them.
///
/// This is the volume mesh an analysis runs on. Building it from a B-Rep solid
/// (tetrahedralization) is a separate concern; a mesh can also be assembled
/// directly (e.g. a structured box mesh) for tests and simple geometry.
struct TetMesh {
    std::vector<Node> nodes;
    std::vector<Tet4> elements;

    /// Number of scalar degrees of freedom: three (x, y, z) per node.
    int dofCount() const { return 3 * static_cast<int>(nodes.size()); }
};

}  // namespace hz::sim
