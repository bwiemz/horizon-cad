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
    /// How far the curve itself stands off the surface: the largest distance measured from a
    /// point of it to the surface (for a closed form, its deviation; on the general path, its
    /// samples' gaps). No pcurve can come nearer the curve than this; a problem whose stand-off
    /// is above the tolerance is the curve's, not the fit's. Infinite when nothing was built.
    double standOff = 0.0;
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
/// confirmed by measuring it, never by the types alone. A planar surface's (u, v) is an affine
/// map of the plane, so on a Plane, or on a bilinear patch that is a parallelogram (as Horizon
/// writes a planar face), a B-spline curve's pcurve is its control points mapped into (u, v)
/// with its weights and knots (a straight one of two points is the Line2D between its ends): its
/// deviation is bounded from those control points' distances off the plane (and the patch's
/// departure from a parallelogram), so it is never below the true one, and the form is taken
/// only where that bound is within what is allowed. A cylinder's or cone's u window is the
/// start point's inverted u, wrapped into [0, 2 pi): a generator on the seam is at u = 0, or at
/// 2 pi where the start point inverts there (at a cone's apex, where u means nothing, the end
/// off the axis gives it). A meridian that starts at a pole takes the side its arc travels to,
/// so one that does not cross a pole stays in v's range; only a meridian through a pole
/// continues past it with v outside the range, which the surface still evaluates correctly.
/// A circle's sense is its turn about x cross y, so left-handed frames are read correctly.
///
/// Any other curve is sampled (adaptively, and again where the fit strays), inverted from
/// each previous point, and fitted by a cubic Nurbs2D by chord length, with up to 2048 samples.
/// Refinement is steered by a quick look inside each sample interval; the fit returned is
/// measured at seven points in each, its excess over the curve's own gap counted 3% more.
/// A curve that is farther than @p tolerance from the surface, or that the fit cannot follow
/// within it, comes back with @c problem set to say so (and the largest deviation measured).
///
/// @p standOffLimit is how far the curve may stand off the surface and still be taken (a
/// reader whose edge tolerances grow to cover the gaps in its file); at or below @p tolerance,
/// or not finite, nothing beyond the tolerance is. A curve measured to stand off by a gap g
/// within it is fitted to min(@p standOffLimit, 2 g + @p tolerance) rather than to
/// @p tolerance: no pcurve comes nearer the curve than its gap, so the gap is measured, not
/// chased (the factor 2 covers a gap that peaks between the samples that measured it). A
/// closed form, whose deviation is its stand-off, is taken where that deviation is within the
/// limit.
Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1, double tolerance,
                double standOffLimit = 0.0);

}  // namespace hz::geo
