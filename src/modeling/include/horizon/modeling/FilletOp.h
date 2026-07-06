#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// Result of a fillet operation.
struct FilletResult {
    std::unique_ptr<topo::Solid>
        solid;                 ///< The new solid with filleted edges, or nullptr on error.
    std::string errorMessage;  ///< Non-empty if the operation failed.
};

/// One entry of a variable-radius table: the fillet radius at normalized
/// parameter @c t along the edge (t = 0 at the edge's recorded start vertex,
/// t = 1 at its end). The radius varies piecewise-linearly between stops.
struct RadiusStop {
    double t = 0.0;
    double radius = 0.0;
};

/// Creates a new solid with fillet faces replacing selected edges.
///
/// The operation rebuilds the solid topology from scratch using Euler
/// operators, inserting NURBS fillet faces at the specified edges.  Currently
/// supports filleting straight edges of box-like (all-planar-face) solids.
///
/// Vertex blends (Phase 61): when exactly THREE selected edges meet at a
/// common vertex with equal radii, the corner is blended with a spherical
/// patch — each fillet is trimmed back by one radius and a sphere-octant face
/// is stitched across the three trimmed ends.  Two edges sharing a vertex
/// without the third remain unsupported and are refused.
class FilletOp {
public:
    /// Fillet the specified edges with a constant radius.
    /// @param inputSolid  The source solid (not modified).
    /// @param edgeIds     TopologyIDs of edges to fillet.
    /// @param radius      Fillet radius (must be positive).
    /// @param featureID   Name used as the source in derived TopologyIDs.
    /// @return A new solid with fillet faces, or an error message.
    static FilletResult execute(const topo::Solid& inputSolid,
                                const std::vector<topo::TopologyID>& edgeIds, double radius,
                                const std::string& featureID);

    /// Fillet one edge with a radius that varies along it (Phase 61).
    /// @param stops  Radius table covering t = 0 and t = 1 in increasing
    ///               order; radii interpolate linearly between stops and each
    ///               segment becomes its own ruled fillet face.
    static FilletResult executeVariable(const topo::Solid& inputSolid,
                                        const topo::TopologyID& edgeId,
                                        const std::vector<RadiusStop>& stops,
                                        const std::string& featureID);
};

}  // namespace hz::model
