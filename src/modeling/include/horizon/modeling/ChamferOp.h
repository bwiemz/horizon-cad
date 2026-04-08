#pragma once

#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Result of a chamfer operation.
struct ChamferResult {
    std::unique_ptr<topo::Solid> solid;  ///< The new solid with chamfered edges, or nullptr on error.
    std::string errorMessage;            ///< Non-empty if the operation failed.
};

/// Creates a new solid with planar chamfer faces replacing selected edges.
///
/// The operation rebuilds the solid topology from scratch using Euler operators,
/// inserting planar NURBS chamfer faces at the specified edges.  Currently
/// supports chamfering straight edges of box-like (all-planar-face) solids.
class ChamferOp {
public:
    /// Chamfer the specified edges with equal distance on both adjacent faces.
    /// @param inputSolid  The source solid (not modified).
    /// @param edgeIds     TopologyIDs of edges to chamfer.
    /// @param distance    Chamfer distance (must be positive).
    /// @param featureID   Name used as the source in derived TopologyIDs.
    static ChamferResult executeEqual(const topo::Solid& inputSolid,
                                      const std::vector<topo::TopologyID>& edgeIds,
                                      double distance, const std::string& featureID);

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
