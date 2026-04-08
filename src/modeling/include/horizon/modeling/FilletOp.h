#pragma once

#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Result of a fillet operation.
struct FilletResult {
    std::unique_ptr<topo::Solid> solid;  ///< The new solid with filleted edges, or nullptr on error.
    std::string errorMessage;            ///< Non-empty if the operation failed.
};

/// Creates a new solid with cylindrical fillet faces replacing selected edges.
///
/// The operation rebuilds the solid topology from scratch using Euler operators,
/// inserting cylindrical NURBS fillet faces at the specified edges.  Currently
/// supports filleting straight edges of box-like (all-planar-face) solids.
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
};

}  // namespace hz::model
