#pragma once

#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model::exact {

/// Primitives built from whole faces carrying exact geometry: planes, cylinders, cones, spheres,
/// tori and surfaces of revolution, circles and lines, with pcurves on every half-edge. They are
/// not faceted, and carry no NURBS carrier (`Face::surface`). Faces are named as PrimitiveFactory
/// names its faceted ones, as model::logicalFace reads them once nameFacetsLogically has named
/// the facets: "box/top", "cylinder/side", "cone/side", "sphere/band", "torus/patch", ...
/// Edges are not named (model::nameEdgesLogically names them).
///
/// Every surface of revolution is turned about base.z with its seam at u = 0, along base.x, where
/// PrimitiveFactory's rings start. A left-handed base gives the same solid as the right-handed
/// frame with the same origin, z and x. Where a profile meets the axis (a sphere's poles, a
/// cone's apex) there is no edge: the face's loop reaches the point by a jump in (u, v) from one
/// pcurve's end to the next one's start, as an imported sphere's does.

/// Each returns nullptr for a degenerate request: min not below max on every axis, a size,
/// radius or height that is not positive or not finite, a hole that does not lie strictly
/// inside the plate, or a frame whose z is zero.

/// An axis-aligned block: 8 vertices, 12 edges, 6 planar faces.
std::unique_ptr<topo::Solid> makeBlock(const math::Vec3& min, const math::Vec3& max);

/// PrimitiveFactory::makeBox placed by @p base: from base.origin, @p width along base.x,
/// @p height along base.y, @p depth along base.z (a left-handed base's y included). 8 vertices,
/// 12 edges, 6 planar faces, "box/bottom" (z = 0), "box/top", "box/front" (y = 0), "box/back",
/// "box/left" (x = 0) and "box/right".
std::unique_ptr<topo::Solid> makeBox(const geo::Frame& base, double width, double height,
                                     double depth);

/// base.origin at the bottom cap's centre, height along base.z; one seam edge at u = 0
/// (along base.x). Two vertices (one on each rim), three edges (two rim circles and the seam)
/// and three faces.
std::unique_ptr<topo::Solid> makeCylinder(const geo::Frame& base, double radius, double height);

/// A sphere about base.origin, its poles on base.z: one face ("sphere/band"), one seam edge (a
/// meridian from the south pole to the north, at u = 0), its two vertices the poles.
std::unique_ptr<topo::Solid> makeSphere(const geo::Frame& base, double radius);

/// PrimitiveFactory::makeCone placed by @p base: @p bottomRadius at base.origin, @p topRadius at
/// @p height along base.z. Its side ("cone/side") is a `Cone` opening toward its wider end (its
/// half-angle positive, as STEP wants it: one narrowing upward is placed upside down, its u
/// running clockwise about base.z from the same seam), or a `Cylinder` where the radii are alike
/// within the model resolution of the cone's size (as master tags it). A radius within that
/// resolution is an apex: the side comes to a point there and that end has no cap; otherwise
/// "cone/bottom" and "cone/top" are discs. A radius that is negative or not finite, or both radii
/// within the resolution, is no cone.
std::unique_ptr<topo::Solid> makeCone(const geo::Frame& base, double bottomRadius, double topRadius,
                                      double height);

/// A torus about base.z, its tube's centre @p majorRadius from base.origin: one face
/// ("torus/patch"). Where the tube clears the axis (majorRadius > minorRadius) it is a `Torus`
/// bounded by two seams (the outer equator and the tube's circle at u = 0) meeting at one
/// vertex. Otherwise (a horn, or a spindle whose tube crosses the axis) it is the "apple": the
/// part of the tube's circle on base.x's side of the axis turned about it, a NURBS surface of
/// revolution with poles where the circle meets the axis, bounded by one seam (that arc) from
/// the south pole to the north; a horn's two poles are one vertex, and a tube that clears the
/// axis by no more than the model resolution is a horn. majorRadius must be positive. A horn's
/// pole is a cusp, which the read point's mesher does not mesh (nor a spindle's within some 5%
/// of a horn): it says so.
std::unique_ptr<topo::Solid> makeTorus(const geo::Frame& base, double majorRadius,
                                       double minorRadius);

/// The lune a revolve of a semicircle by @p angle (in (0, 2 pi)) makes: the sphere about
/// base.origin between the meridians at u = 0 and u = angle ("sphere/band"), each meridian
/// running from pole to pole, and two half-disc caps ("sphere/cap_start" at u = 0,
/// "sphere/cap_end" at u = angle) meeting along the axis. Two vertices (the poles), three edges
/// (the two meridians and the axis) and three faces.
std::unique_ptr<topo::Solid> makeSphereSector(const geo::Frame& base, double radius, double angle);

/// A block with a round hole through it along z, centred at holeCentre (x, y). The hole's wall
/// is the face "hole/wall"; its cylinder is reversed, so the normal points into the hole.
std::unique_ptr<topo::Solid> makePlateWithHole(const math::Vec3& min, const math::Vec3& max,
                                               const math::Vec2& holeCentre, double holeRadius);

}  // namespace hz::model::exact
