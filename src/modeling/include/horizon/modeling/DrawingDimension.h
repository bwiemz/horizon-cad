#pragma once

#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// A linear dimension anchored to a model edge by its TopologyID.
///
/// The value is *measured from the model*, not stored as fixed geometry, so
/// re-measuring after a rebuild reflects the current model. This is the basis
/// for model-driven dimensioning: because the anchor is a TopologyID (stable
/// across rebuilds via the topological-naming genealogy), a dimension keeps
/// pointing at the same edge and updates as the part changes.
struct LinearDimension {
    topo::TopologyID edge;  ///< the model edge this dimension measures
    double value = 0.0;     ///< measured length, recomputed from the model
};

/// Measures model geometry for drawing dimensions.
class DrawingDimensioner {
public:
    /// Straight-line length of the model edge with @p edgeId in @p solid. Returns
    /// false (leaving @p outLength untouched) if no edge with that TopologyID
    /// exists.
    static bool measureEdge(const topo::Solid& solid, const topo::TopologyID& edgeId,
                            double& outLength);

    /// Build a LinearDimension for @p edgeId, measuring its length from @p solid.
    /// Returns false if the edge is not found.
    static bool dimensionEdge(const topo::Solid& solid, const topo::TopologyID& edgeId,
                              LinearDimension& out);
};

}  // namespace hz::model
