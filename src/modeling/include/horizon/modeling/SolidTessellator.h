#pragma once

#include "horizon/geometry/MeshData.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Converts a B-Rep Solid into a triangle mesh (geo::MeshData).
///
/// Planar faces are triangulated from their trimmed vertex loops — their
/// stored surfaces are bounding-rectangle patches that over-cover any
/// non-rectangular face (see Extrude).  Curved faces fall back to NURBS
/// surface tessellation for smooth shading.
class SolidTessellator {
public:
    /// Tessellate all faces of @p solid and return a merged mesh.
    /// @param tolerance  Tessellation chord-height tolerance (curved faces).
    static geo::MeshData tessellate(const topo::Solid& solid, double tolerance = 0.1);
};

}  // namespace hz::model
