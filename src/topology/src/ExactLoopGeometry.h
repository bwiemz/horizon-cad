#pragma once

// Internal to the exact validator: the 2D geometry of a face's loops in (u, v), once they are
// unwrapped (horizon/topology/LoopUnwrap.h).

#include <functional>
#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::topo::detail {

/// How far apart two (u, v) points may be and still be the same point of the surface, at a
/// given point.
using UvTolerance = std::function<double(const math::Vec2&)>;

/// Signed area (positive counter-clockwise) of the chords' polygon.
double signedArea(const LoopPolygon& loop);

/// True when two non-adjacent chords of the loop cross for real: a crossing of chords is
/// confirmed by bisecting both curves until their deviation from the chords is below the
/// tolerance, so the sag of a coarse sampling does not count.
bool selfIntersects(const LoopPolygon& loop, const UvTolerance& tolerance);

/// True when a chord of @p p and one of @p q cross for real (confirmed as above).
bool loopsCross(const LoopPolygon& p, const LoopPolygon& q, const UvTolerance& tolerance);

/// True when @p point lies outside @p outer by more than the chords' greatest sag, so the
/// chord approximation cannot be what puts it there.
bool clearlyOutside(const LoopPolygon& outer, const math::Vec2& point);

/// True when @p point lies inside @p loop by more than the chords' greatest sag (each chord's
/// own, as for clearlyOutside).
bool clearlyInside(const LoopPolygon& loop, const math::Vec2& point);

}  // namespace hz::topo::detail
