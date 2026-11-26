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

/// How far a solid's edge and vertex tolerances may grow to cover the gaps measured in its exact
/// geometry, as an OpenCASCADE reader's per-entity tolerances do: the STEP reader's, for the gaps
/// in its file, and a non-similar copy's (model::Pattern::transformed), for its refitted pcurves.
/// A gap within the model resolution needs nothing; one up to the limit, a thousandth of the
/// solid's size (the resolution where its size cannot be measured), is covered; one past it is
/// not, and the solid is not exact.
struct ToleranceGrowth {
    /// For a solid of size @p extent (exactExtent).
    explicit ToleranceGrowth(double extent);

    math::ModelTolerance resolution;  ///< ModelTolerance::forExtent(extent)
    bool sized = false;               ///< the limit is a thousandth of the size
    double limit = 0.0;               ///< the most a tolerance grows to

    /// Grows @p tolerance to cover @p gap: true where it is covered (within the resolution, or
    /// grown to it), false where the gap is past the limit (nothing is grown).
    bool grow(double& tolerance, double gap) const;
};

}  // namespace hz::topo
