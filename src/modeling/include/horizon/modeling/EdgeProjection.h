#pragma once

#include <memory>
#include <string>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// An edge's name as what refers to it keeps it (Phase 157): the curve, not
/// one of its chords (model::logicalEdge()), and without the `/piece:<n>` a
/// Boolean gives an edge it keeps or cuts, so it names the edge through
/// later features.
std::string wholeEdgeName(const std::string& tag);

/// The edge of @p solid named @p edge (a name as wholeEdgeName() gives it: a
/// curve's chords together) drawn straight onto @p plane (Phase 157), in
/// the plane's own 2D coordinates:
/// - a line, for a straight edge;
/// - a circle or an arc, for a round edge lying parallel to the plane;
/// - otherwise a polyline through its chords' ends (a circle seen at a
///   slant, a spline).
/// A new entity, with nothing else set: its caller names the edge it
/// follows. Null, and why in @p why ("is not there", "is seen end on"),
/// when there is nothing to draw.
std::shared_ptr<draft::DraftEntity> projectEdge(const topo::Solid& solid, const std::string& edge,
                                                const draft::SketchPlane& plane,
                                                std::string* why = nullptr);

}  // namespace hz::model
