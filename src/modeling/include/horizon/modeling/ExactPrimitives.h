#pragma once

#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model::exact {

/// Primitives built from whole faces carrying exact geometry: planes, cylinders, circles and
/// lines, with pcurves on every half-edge. They are not faceted. Faces are named as
/// PrimitiveFactory names its faceted ones, without facet suffixes ("box/top",
/// "cylinder/side", ...).

/// Each returns nullptr for a degenerate request: min not below max on every axis, a radius or
/// height that is not positive, or a hole that does not lie strictly inside the plate.

/// An axis-aligned block: 8 vertices, 12 edges, 6 planar faces.
std::unique_ptr<topo::Solid> makeBlock(const math::Vec3& min, const math::Vec3& max);

/// base.origin at the bottom cap's centre, height along base.z; one seam edge at u = 0
/// (along base.x). A left-handed base gives the same solid as the right-handed frame with the
/// same origin, z and x. Two
/// vertices (one on each rim), three edges (two rim circles and the seam) and three faces.
std::unique_ptr<topo::Solid> makeCylinder(const geo::Frame& base, double radius, double height);

/// A block with a round hole through it along z, centred at holeCentre (x, y). The hole's wall
/// is the face "hole/wall"; its cylinder is reversed, so the normal points into the hole.
std::unique_ptr<topo::Solid> makePlateWithHole(const math::Vec3& min, const math::Vec3& max,
                                               const math::Vec2& holeCentre, double holeRadius);

}  // namespace hz::model::exact
