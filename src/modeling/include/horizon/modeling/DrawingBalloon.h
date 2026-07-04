#pragma once

#include "horizon/math/Vec2.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// A BOM balloon on a drawing view: a numbered circle with a leader pointing at
/// a model feature.
///
/// Like dimensions and GD&T frames, the balloon is anchored to a model edge by
/// TopologyID, so it re-projects with the view and stays attached to the same
/// feature across rebuilds. The @p item number links it to a bill-of-materials
/// line (see BomGenerator).
struct DrawingBalloon {
    int item = 0;                   ///< BOM item number shown inside the circle
    topo::TopologyID feature;       ///< the model edge the leader points at
    math::Vec2 offset{15.0, 15.0};  ///< circle center relative to the edge midpoint (sheet units)
    double radius = 4.0;            ///< balloon circle radius (sheet units)
};

}  // namespace hz::model
