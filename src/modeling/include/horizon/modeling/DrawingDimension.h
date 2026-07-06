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

/// An angular dimension between two model edges, anchored by their TopologyIDs.
/// The value is the unsigned angle between the two edge lines (0..pi/2),
/// re-measured from the model.
struct AngularDimension {
    topo::TopologyID edgeA;
    topo::TopologyID edgeB;
    double value = 0.0;  ///< radians (unsigned line-to-line angle)
};

/// A radial dimension anchored to a circular model edge (Phase 61).
/// The radius is re-measured from the model edge's curve, so it tracks
/// rebuilds just like linear dimensions.
struct RadialDimension {
    topo::TopologyID edge;  ///< a circular model edge
    double value = 0.0;     ///< measured radius
    bool diameter = false;  ///< display as ⌀ (2·radius) instead of R
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

    /// Unsigned angle (0..pi/2 radians) between the two model edges @p edgeA and
    /// @p edgeB. Returns false (leaving @p outRadians untouched) if either edge is
    /// missing or degenerate (zero length).
    static bool measureAngle(const topo::Solid& solid, const topo::TopologyID& edgeA,
                             const topo::TopologyID& edgeB, double& outRadians);

    /// Build an AngularDimension for the two edges, measuring the angle from the
    /// model. Returns false if either edge is not found or is degenerate.
    static bool dimensionAngle(const topo::Solid& solid, const topo::TopologyID& edgeA,
                               const topo::TopologyID& edgeB, AngularDimension& out);

    /// Radius of the circular model edge with @p edgeId, measured by sampling
    /// its curve and fitting a circle. Returns false if the edge is missing,
    /// has no curve, or is not circular within tolerance (straight edges,
    /// free-form splines).
    static bool measureRadius(const topo::Solid& solid, const topo::TopologyID& edgeId,
                              double& outRadius);

    /// Build a RadialDimension for @p edgeId. @p diameter selects ⌀ display.
    /// Returns false if the edge is not found or not circular.
    static bool dimensionRadius(const topo::Solid& solid, const topo::TopologyID& edgeId,
                                bool diameter, RadialDimension& out);
};

}  // namespace hz::model
