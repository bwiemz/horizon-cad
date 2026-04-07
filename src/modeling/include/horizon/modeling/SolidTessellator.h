#pragma once

#include "horizon/render/SceneGraph.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Converts a B-Rep Solid into a triangle mesh (MeshData) by tessellating
/// each face's NURBS surface and merging the results.
class SolidTessellator {
public:
    /// Tessellate all NURBS-bound faces of @p solid and return a merged mesh.
    /// @param tolerance  Tessellation chord-height tolerance.
    static render::MeshData tessellate(const topo::Solid& solid, double tolerance = 0.1);
};

}  // namespace hz::model
