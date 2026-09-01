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
/// A chamfer is a local edit of the boundary polygons: each vertex the
/// selected edges touch is replaced by its offset point(s) on the neighbouring
/// faces, and one quad is added per chamfered edge.  The rewritten polygon
/// soup is reconstructed by `SolidSewer` — the same welding, T-junction and
/// twin-pairing pipeline `BooleanOp` uses — so the result's loops are
/// geometrically consistent and its volume integrates exactly.  Offsets slide
/// along the neighbouring faces, so every original face keeps its own carrier
/// and TopologyID; only its boundary changes.
///
/// The operation refuses to return a solid that fails `checkManifold()` or
/// `GeometryValidator`, reporting the diagnostic in `errorMessage` instead.
///
/// Limitations:
///  - straight edges of planar-faced solids (the offset directions come from
///    the adjacent faces' normals at their parameter midpoints);
///  - **no vertex blends** — two selected edges sharing a vertex are rejected,
///    since the corner where three chamfer faces would meet is not generated;
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
