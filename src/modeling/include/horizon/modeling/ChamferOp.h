#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// Result of a chamfer operation.
struct ChamferResult {
    std::unique_ptr<topo::Solid>
        solid;                 ///< The new solid with chamfered edges, or nullptr on error.
    std::string errorMessage;  ///< Non-empty if the operation failed.
};

/// Creates a new solid with planar chamfer faces replacing selected edges.
///
/// A chamfer is a local edit of the boundary polygons.  The material a chamfer
/// removes is the half-space on the far side of its chamfer plane, so a
/// touched corner simply becomes that corner clipped by the planes of every
/// chamfer meeting there, and one face is added per chamfered edge.  The
/// rewritten polygon soup is reconstructed by `SolidSewer` — the same welding,
/// T-junction and twin-pairing pipeline `BooleanOp` uses — so the result's
/// loops are geometrically consistent and its volume integrates exactly.
/// Chamfers cut a face's corners inside its own plane, so every original face
/// keeps its carrier and TopologyID; only its boundary changes.
///
/// **Vertex blends** (two or three selected edges meeting at a corner) fall
/// out of the same clipping: the chamfer planes cut each other and no separate
/// corner patch is invented, which is the standard planar-chamfer result — for
/// three equal chamfers at a box corner the three planes meet at a single
/// point and each chamfer face becomes a pentagon.  Chamfering all twelve
/// edges of a cube yields the chamfered cube (18 faces, 32 vertices) at its
/// exact volume.
///
/// The operation refuses to return a solid that fails `checkManifold()` or
/// `GeometryValidator`, reporting the diagnostic in `errorMessage` instead.
///
/// Limitations:
///  - straight edges of planar-faced solids, with orthogonal corners: the
///    offset directions come from the adjacent faces' normals at their
///    parameter midpoints, so an oblique corner's offsets do not land on the
///    third face's plane.  Such input is refused by the geometric gate rather
///    than silently mis-built;
///  - convex corners: the corner wedge the clip runs on assumes the loop turns
///    one way at the vertex;
///  - inner face loops (holes) are not carried through the rewrite.
class ChamferOp {
public:
    /// Chamfer the specified edges with equal distance on both adjacent faces.
    /// @param inputSolid  The source solid (not modified).
    /// @param edgeIds     TopologyIDs of edges to chamfer.
    /// @param distance    Chamfer distance (must be positive).
    /// @param featureID   Name used as the source in derived TopologyIDs.
    static ChamferResult executeEqual(const topo::Solid& inputSolid,
                                      const std::vector<topo::TopologyID>& edgeIds, double distance,
                                      const std::string& featureID);

    /// Chamfer the specified edges with different distances on the two adjacent faces.
    /// @param inputSolid  The source solid (not modified).
    /// @param edgeIds     TopologyIDs of edges to chamfer.
    /// @param distance1   Distance on the first adjacent face (left face of the edge).
    /// @param distance2   Distance on the second adjacent face (right face of the edge).
    /// @param featureID   Name used as the source in derived TopologyIDs.
    static ChamferResult executeTwoDistance(const topo::Solid& inputSolid,
                                            const std::vector<topo::TopologyID>& edgeIds,
                                            double distance1, double distance2,
                                            const std::string& featureID);
};

}  // namespace hz::model
