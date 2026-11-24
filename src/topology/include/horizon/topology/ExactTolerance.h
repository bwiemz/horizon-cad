#pragma once

// The size of a solid with exact geometry, and so its model tolerance, in one place: the mesher,
// the read point and the exact validator's callers all take it from here.

#include "horizon/math/ModelTolerance.h"
#include "horizon/topology/Solid.h"

namespace hz::topo {

/// Whether any live face of @p solid has an exact surface.
bool hasExactFaces(const Solid& solid);

/// The size of @p solid: the diagonal of the box around its live vertices and points along its
/// live edges' exact curves (17 on each, both ends included), for a round part's vertices may
/// be one point on a circle. 0 when nothing live gives a point, or the box is not finite.
double exactExtent(const Solid& solid);

/// The model tolerance of @p solid: ModelTolerance::forExtent(exactExtent(solid)).
math::ModelTolerance exactTolerance(const Solid& solid);

}  // namespace hz::topo
