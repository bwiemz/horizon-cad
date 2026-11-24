#pragma once

// Internal to the exact validator: the 2D geometry of a face's loops in (u, v).

#include <functional>
#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::topo::detail {

/// One chord of a sampled pcurve: the curve between parameters t0 and t1, shifted by whole
/// periods so the loop it belongs to is continuous; a and b are its (shifted) ends.
struct LoopSegment {
    const geo::Curve2D* curve = nullptr;
    double t0 = 0.0;
    double t1 = 0.0;
    math::Vec2 shift;
    math::Vec2 a;
    math::Vec2 b;
};

/// A loop as a closed run of chords, continuous in unwrapped (u, v).
using LoopPolygon = std::vector<LoopSegment>;

/// How far apart two (u, v) points may be and still be the same point of the surface, at a
/// given point.
using UvTolerance = std::function<double(const math::Vec2&)>;

/// Samples each half-edge's pcurve at @p samples parameters (both ends included, either way
/// round) and chains them, shifting each by whole periods (periodU, periodV; 0 where the surface
/// is not periodic in that direction) so it continues the previous one. False when the loop
/// closes only after winding a whole number of periods round the surface; @p out is then not
/// meaningful. @p joinAllowance[k] is how far apart (a distance on the surface) half-edge k's end
/// and the next one's start may be and still join; on an axis where the raw jump is already
/// within it (a pole or an apex, where the derivative vanishes) nothing is shifted: that jump is a
/// move along the singular side, not a wrap. Every half-edge must carry a pcurve.
bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, const geo::Surface& surface,
                double periodU, double periodV, const std::vector<double>& joinAllowance,
                LoopPolygon& out);

/// Shifts @p loop by whole periods so it lies in the window of @p reference (nearest by the
/// middle of its extent), for loops of one face that were each unwrapped from their own start.
void alignToReference(LoopPolygon& loop, const LoopPolygon& reference, double periodU,
                      double periodV);

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

}  // namespace hz::topo::detail
