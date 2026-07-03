#pragma once

#include "horizon/math/Vec3.h"

namespace hz::model::measure {

/// Geometric 3D measurement primitives. UI face/edge measurement resolves the
/// selected geometry to points/segments and calls these.

/// Straight-line distance between two points.
double distance(const math::Vec3& a, const math::Vec3& b);

/// Angle (radians, in [0, π]) between two directions. Returns 0 if either is
/// degenerate.
double angleBetween(const math::Vec3& u, const math::Vec3& v);

/// Shortest distance from point @p p to segment [a, b].
double pointToSegment(const math::Vec3& p, const math::Vec3& a, const math::Vec3& b);

/// Shortest distance between segments [a0,a1] and [b0,b1].
double segmentToSegment(const math::Vec3& a0, const math::Vec3& a1, const math::Vec3& b0,
                        const math::Vec3& b1);

}  // namespace hz::model::measure
