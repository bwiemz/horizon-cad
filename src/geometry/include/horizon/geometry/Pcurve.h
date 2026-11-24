#pragma once

#include <memory>
#include <string>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"

namespace hz::geo {

/// A curve on a surface, in the surface's parameters: what a half-edge keeps of its edge.
struct Pcurve {
    /// Null exactly when no pcurve could be built at all (then @c problem says why and
    /// @c deviation is infinite); callers check @c problem first. A pcurve that was built but
    /// strays more than the tolerance is returned with @c problem set.
    std::shared_ptr<const Curve2D> curve;
    /// The pcurve's own parameter at the curve's t0.
    double start = 0.0;
    /// The pcurve's own parameter at the curve's t1.
    double end = 0.0;
    /// The largest distance measured between S(curve(s)) and the curve C.
    double deviation = 0.0;
    /// Empty when the pcurve lies within the tolerance; otherwise says why it does not.
    std::string problem;
};

/// The curve in @p surface's (u, v) that @p curve follows from @p t0 to @p t1, within
/// @p tolerance: continuous across a seam (u is unwrapped, so a rim over the seam runs past the
/// end of u's range), starting at t0's point and ending at t1's. A reversed range (t1 < t0)
/// gives a pcurve that runs the other way.
///
/// A curve that is a closed form on its surface (a line on a plane, a generator, a parallel, a
/// meridian, a circle or ellipse on a plane) gets the exact 2D curve, found geometrically and
/// confirmed by measuring it, never by the types alone. A cylinder's or cone's u window is the
/// start point's inverted u, wrapped into [0, 2 pi): a generator on the seam is at u = 0, or at
/// 2 pi where the start point inverts there (at a cone's apex, where u means nothing, the end
/// off the axis gives it). A meridian that starts at a pole takes the side its arc travels to,
/// so one that does not cross a pole stays in v's range; only a meridian through a pole
/// continues past it with v outside the range, which the surface still evaluates correctly.
/// A circle's sense is its turn about x cross y, so left-handed frames are read correctly.
///
/// Any other curve is sampled (adaptively, and again where the fit strays), inverted from
/// each previous point, and fitted by a cubic Nurbs2D by chord length, with up to 2048 samples.
/// A curve that is farther than @p tolerance from the surface, or that the fit cannot follow
/// within it, comes back with @c problem set to say so (and the largest deviation measured).
Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1,
                double tolerance);

}  // namespace hz::geo
